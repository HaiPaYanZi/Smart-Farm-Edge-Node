#include "nvs.h"
#include "param.h"
#include "log.h"
#include <stm32f1xx_hal.h>
#include <string.h>

/*
  Flash 参数存储实现(双区轮换,文档《03》§6.1)

  落盘记录布局(整块按 uint16_t 半字写入 Flash):
    magic      uint16  0xA5A5:本区被本固件写过
    payload_crc uint16 参数载荷的 CRC16(掉电写一半/位翻转都能查出来)
    sequence   uint32  写入序号:两区都有效时,序号大的是最新参数
    version    uint16  参数结构版本:NVS_VERSION 不匹配按无效处理
    payload    param_payload_t 参数载荷(来自 param.c)

  双区策略:保存永远写"对侧区"(当前有效区的另一侧),先擦后写再校验;
   掉电发生在任意时刻,至少有一个区是完整的,上电总能恢复出最后一份好参数。
*/

#define NVS_REGION_A_ADDR   0x0800F000U     /* 区 A 起始地址(第 30 页,高密度器件 2KB/页) */
#define NVS_REGION_B_ADDR   0x0800F800U     /* 区 B 起始地址(第 31 页,区 A 的下一页) */
#define NVS_PAGE_SIZE       0x00000800U     /* F103 高密度器件页大小:2KB */
#define NVS_MAGIC           0xA5A5U         /* 落盘魔术字 */

#define SAVE_DEBOUNCE_TICKS 20              /* 落盘防抖:20 拍 × 100ms = 2s */

/* 落盘记录:整块写进 Flash 页(52 字节,远小于 2KB 页) */
typedef struct {
    uint16_t magic;             /* 魔术字 0xA5A5:标记本区是本固件写的 */
    uint16_t payload_crc;       /* 参数载荷的 CRC16(校验数据完整性) */
    uint32_t sequence;          /* 写入序号:越大越新(双区比对用) */
    uint16_t version;           /* 参数结构版本号(不匹配 → 本区作废) */
    param_payload_t payload;    /* 参数载荷:来自 param.c 的全部可持久化参数 */
} nvs_record_t;

/* 运行时状态(仅本模块与中断上下文无关,普通变量即可) */
static uint32_t active_region_addr = 0;   /* 当前有效区地址:0=尚无有效区(首次上电) */
static uint32_t next_sequence = 0;        /* 下一次落盘要写的序号(从有效区序号+1 续) */
static uint16_t debounce_counter = 0;     /* 落盘防抖倒计时:>0 时在等参数写停 */

/**
  * 函    数：CalcCrc16
  * 功    能：计算 CRC16(与 Modbus RTU 同款多项式 0x8005 反序 0xA001,初值 0xFFFF)
  * 参    数：data 数据首地址,length 数据长度(字节)
  * 返 回 值：16 位 CRC 校验值
  * 说    明：逐位算法不用查表(参数区才几十字节,速度无所谓,省 512 字节 Flash 表)
  */
static uint16_t CalcCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;      /* CRC 初值:0xFFFF(Modbus 约定) */
    uint16_t index;             /* 字节循环下标 */
    uint8_t bit;                /* 位循环下标(每个字节 8 位) */

    for (index = 0; index < length; index++)
    {
        crc ^= data[index];                     /* 当前字节并入 CRC 低位 */
        for (bit = 0; bit < 8; bit++)           /* 逐位处理(LSB 先) */
        {
            if (crc & 0x0001U)                  /* 最低位为 1:右移并异或多项式 */
            {
                crc = (crc >> 1) ^ 0xA001U;     /* 0xA001 是 0x8005 的位反序(LSB-first 用) */
            }
            else                                /* 最低位为 0:只右移 */
            {
                crc >>= 1;
            }
        }
    }
    return crc;                 /* 返回最终校验值 */
}

/**
  * 函    数：ReadRegion
  * 功    能：读一个区的落盘记录并校验完整性
  * 参    数：addr 区起始地址,out 读出记录的输出缓冲
  * 返 回 值：1=有效(魔术字+版本+CRC 全过);0=无效(未写过/写坏/版本不匹配)
  * 说    明：读 Flash 就是读内存(F103 的 Flash 统一编址),无需解锁
  */
static uint8_t ReadRegion(uint32_t addr, nvs_record_t *out)
{
    memcpy(out, (const void *)addr, sizeof(nvs_record_t));   /* 直接从 Flash 映射地址拷贝整块 */

    if (out->magic != NVS_MAGIC)    return 0;   /* 魔术字不对:这页没被写过(或全 0xFF 擦除态) */
    if (out->version != NVS_VERSION) return 0;  /* 参数结构版本不匹配:布局变了,旧数据不能要 */

    /* 重算载荷 CRC,与落盘时存的比对:掉电写一半/Flash 位翻转都能查出来 */
    if (CalcCrc16((const uint8_t *)&out->payload, sizeof(out->payload)) != out->payload_crc)
    {
        return 0;                               /* CRC 不通过:数据损坏 */
    }
    return 1;                                   /* 全部校验通过:本区有效 */
}

/**
  * 函    数：SaveToFlash
  * 功    能：把当前参数打包落盘到指定区(擦除+写入+回读校验)
  * 参    数：addr 目标区起始地址
  * 返 回 值：1=落盘成功;0=擦除或写后校验失败(保持旧有效区不变)
  * 说    明：擦写期间关中断——Flash 擦写时不能同时从 Flash 取指/取向量,
  *           中断函数代码也在 Flash 里,必须全关;整页擦+写约几十 ms,
  *           期间串口/Modbus 会丢数据(协议重发机制可容忍,参数保存并不频繁)
  */
static uint8_t SaveToFlash(uint32_t addr)
{
    nvs_record_t record;                        /* 待落盘的记录(栈上打包) */
    nvs_record_t verify;                        /* 写完后回读校验的缓冲 */
    FLASH_EraseInitTypeDef erase_init;          /* HAL 页擦除参数结构 */
    uint32_t page_error = 0;                    /* 擦除失败的页地址(HAL 输出参数) */
    const uint16_t *halfword_source;            /* 记录按半字写的源指针 */
    uint32_t halfword_index;                    /* 半字序号(记录 52 字节 = 26 个半字) */

    /* 打包记录:头 + 当前参数(Param_GetPayload 整体拷出) */
    record.magic       = NVS_MAGIC;
    record.version     = NVS_VERSION;
    record.sequence    = next_sequence;
    Param_GetPayload(&record.payload);
    record.payload_crc = CalcCrc16((const uint8_t *)&record.payload, sizeof(record.payload));

    __disable_irq();                            /* 关中断:擦写期间不能进任何 ISR(见函数头注释) */
    HAL_FLASH_Unlock();                         /* 解锁 Flash 控制器(擦写前必须) */

    /* 擦除目标页(2KB 一页,F103 只能整页擦) */
    erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;  /* 按页擦除 */
    erase_init.PageAddress = addr;                   /* 目标页起始地址 */
    erase_init.NbPages     = 1;                      /* 只擦一页 */
    HAL_FLASHEx_Erase(&erase_init, &page_error);     /* 执行擦除,page_error 返回失败页 */
    if (page_error != 0xFFFFFFFFU)                   /* 擦除失败:page_error 记录了失败页地址 */
    {
        HAL_FLASH_Lock();                            /* 上锁后退出 */
        __enable_irq();
        LOG_ERROR("NVS erase failed at 0x%08X", (unsigned int)page_error);
        return 0;                                    /* 不写数据,保持旧有效区不变 */
    }

    /* 按半字写入整块记录(F103 编程最小单位是半字) */
    halfword_source = (const uint16_t *)&record;
    for (halfword_index = 0; halfword_index < sizeof(record) / 2; halfword_index++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + halfword_index * 2, halfword_source[halfword_index]);
    }

    HAL_FLASH_Lock();                           /* 重新上锁,防止误操作 */
    __enable_irq();                             /* 恢复中断 */

    /* 回读校验:确认写进去的与想写的一致(防编程失败静默丢数据) */
    if (!ReadRegion(addr, &verify))
    {
        LOG_ERROR("NVS save failed at 0x%08X, keep old region", (unsigned int)addr);
        return 0;                               /* 校验失败:不切换有效区,下次保存重试 */
    }
    return 1;                                   /* 落盘成功 */
}

/**
  * 函    数：NVS_Init
  * 功    能：上电初始化:读双区取最新有效参数载入 param;两区全无效时用默认值并落一份盘
  * 参    数：无
  * 返 回 值：无
  * 说    明：必须在 Param_Init 之后调用(默认值已载入,有 Flash 数据则覆盖)
  */
void NVS_Init(void)
{
    nvs_record_t record_a;                      /* 区 A 的落盘记录 */
    nvs_record_t record_b;                      /* 区 B 的落盘记录 */
    uint8_t valid_a = ReadRegion(NVS_REGION_A_ADDR, &record_a);  /* 区 A 是否有效 */
    uint8_t valid_b = ReadRegion(NVS_REGION_B_ADDR, &record_b);  /* 区 B 是否有效 */
    nvs_record_t *chosen = NULL;                /* 选中的最新有效记录指针 */
    uint32_t chosen_addr = 0;                   /* 选中记录所在区的地址 */

    if (valid_a && valid_b)                     /* 两区都有效:序号大的是最新一次保存 */
    {
        if (record_a.sequence >= record_b.sequence) { chosen = &record_a; chosen_addr = NVS_REGION_A_ADDR; }
        else                                        { chosen = &record_b; chosen_addr = NVS_REGION_B_ADDR; }
    }
    else if (valid_a)                           /* 只有区 A 有效 */
    {
        chosen = &record_a; chosen_addr = NVS_REGION_A_ADDR;
    }
    else if (valid_b)                           /* 只有区 B 有效(上一次保存写 B 后掉电在写 A 之前) */
    {
        chosen = &record_b; chosen_addr = NVS_REGION_B_ADDR;
    }
    else                                        /* 两区全无效:首次上电或参数区被整体擦除 */
    {
        LOG_INFO("NVS empty, use default params and save");
        next_sequence = 0;                      /* 序号从 0 开始 */
        if (SaveToFlash(NVS_REGION_A_ADDR))     /* 把默认参数落一份盘,让双区状态确定 */
        {
            active_region_addr = NVS_REGION_A_ADDR;  /* 落盘成功才记录有效区(失败则下次 Tick 重试) */
            next_sequence = 1;                  /* 序号续上:防止下次保存又写 seq=0 造成双区同序号 */
        }
        return;                                 /* 默认值已在 Param_Init 载入,无需再加载 */
    }

    /* 把最新有效参数载入参数区(Param_Load 会置 dirty,控制任务第一拍刷进 PID) */
    Param_Load(&chosen->payload);
    Param_ClearSaveRequest();                   /* 清落盘请求:刚读出来的数据不需要再写回 */
    active_region_addr = chosen_addr;           /* 记录当前有效区,下次保存写对侧 */
    next_sequence = chosen->sequence + 1;       /* 序号续上(溢出回绕见下方注释) */
    /* 序号是 uint32:写满 2^32 次才会回绕出错,工程生命周期内不可能,不做回绕处理 */
}

/**
  * 函    数：NVS_Tick
  * 功    能：落盘调度(控制任务 100ms 节拍调用):参数变更防抖 2s 后落盘
  * 参    数：无
  * 返 回 值：无
  * 说    明：防抖的意义——远程整定往往连续写多个寄存器(40001~40004 一条条发),
  *           每写一个就擦一次 Flash 既伤寿命(1 万次/页)又长时间关中断;
  *           等写停 2s 再落盘,整定过程只擦写一次
  */
void NVS_Tick(void)
{
    if (Param_TakeSaveRequest())                /* 有新的参数变更 */
    {
        debounce_counter = SAVE_DEBOUNCE_TICKS; /* 重置防抖倒计时:2s 内再来变更就重新计时 */
    }
    else if (debounce_counter > 0)              /* 正在防抖等待中 */
    {
        debounce_counter--;                     /* 倒计时一拍 */
        if (debounce_counter == 0)              /* 2s 内没有新变更:写停,落盘 */
        {
            /* 写对侧区:当前有效区的另一侧(首上电无效区时写 A) */
            uint32_t target_addr = (active_region_addr == NVS_REGION_B_ADDR)
                                 ? NVS_REGION_A_ADDR : NVS_REGION_B_ADDR;
            if (SaveToFlash(target_addr))       /* 落盘成功才切换有效区和序号 */
            {
                active_region_addr = target_addr;
                next_sequence++;
                LOG_INFO("NVS saved seq=%d", (int)next_sequence);
            }
        }
    }
    /* else:无事可做,一拍什么都不干 */
}
