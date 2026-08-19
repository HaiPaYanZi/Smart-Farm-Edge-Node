#include "ota.h"
#include "ota_common.h"      /* 三方共享:分区地址/标志区结构体/故障码 */
#include "stm32f1xx_hal.h"   /* HAL_FLASH 页擦写 API */

/*
  OTA 触发实现(App 侧)

  升级请求落点:标志区 0x08003000(与 Bootloader/PC 工具共用 ota_common.h 结构体)
  写 Flash 细节:
    - 整页擦除(2KB 页)后逐半字编程,结构体 21 字节按半字对齐写 22 字节(末字节 0xFF)
    - 擦写期间 CPU 取指 stall 约 40ms:只会在 Modbus/调试任务上下文调用,可接受
    - 不打断调度器:延时复位交给 OTA_Poll 在控制任务里做,保证 Modbus 应答先发出去
*/

static volatile uint8_t s_reset_pending = 0;   /* 1=标志区已写好,等待复位窗口结束 */
static uint32_t s_pending_tick = 0;            /* 触发时刻(HAL_GetTick 毫秒值) */

/* 复位延时:200ms,覆盖 Modbus 应答帧发送时间(64B @9600bps ≈ 70ms,留足余量) */
#define OTA_RESET_DELAY_MS  200UL

/**
  * 函    数：ota_current_part
  * 功    能：推导当前运行在哪个分区(由向量表地址反推)
  * 返 回 值：'A' 或 'B'
  * 说    明：App 链接基址 = 分区起始 + 0x200(固件头段),SystemInit 已按此设好 VTOR
  */
static uint8_t ota_current_part(void)
{
    if (SCB->VTOR == OTA_PART_B_ADDR + OTA_APP_OFFSET)   /* 向量表在 B 区:当前跑 B */
    {
        return 'B';
    }
    return 'A';                                          /* 其余情况一律按 A 处理 */
}

/**
  * 函    数：flag_area_write
  * 功    能：把标志结构体写进升级标志区(整页擦除 + 逐半字编程)
  * 参    数：flag 要写入的标志结构体(已由调用方填好字段)
  * 说    明：HAL_FLASHEx_Erase 阻塞至擦完(约 20~40ms);编程按 2 字节半字,
  *           结构体 21 字节,尾字节补 0xFF 凑整
  */
static void flag_area_write(const ota_flag_t *flag)
{
    const uint8_t *bytes = (const uint8_t *)flag;        /* 结构体的字节视图 */
    uint32_t offset;                                     /* 结构体内写入偏移 */
    FLASH_EraseInitTypeDef erase_init;                   /* 页擦除参数 */
    uint32_t page_error = 0;                             /* 擦除错误页码(正常为 0xFFFFFFFF) */

    HAL_FLASH_Unlock();                                  /* 解锁 Flash 控制器 */

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;        /* 按页擦除 */
    erase_init.PageAddress = OTA_FLAG_ADDR;              /* 0x08003000 所在页 */
    erase_init.NbPages = 1;                              /* 只擦 1 页(2KB) */
    HAL_FLASHEx_Erase(&erase_init, &page_error);         /* 擦页(标志区旧内容全清) */

    for (offset = 0; offset < sizeof(ota_flag_t); offset += 2)
    {
        uint16_t half_word = (uint16_t)bytes[offset];    /* 低字节 */
        if (offset + 1 < sizeof(ota_flag_t))             /* 结构体内还有下一字节 */
        {
            half_word |= (uint16_t)bytes[offset + 1] << 8;  /* 高字节 */
        }
        else                                             /* 奇数尾:高字节补 0xFF */
        {
            half_word |= 0xFF00;
        }
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          OTA_FLAG_ADDR + offset, half_word);  /* 半字编程 */
    }

    HAL_FLASH_Lock();                                    /* 上锁 */
}

/**
  * 函    数：OTA_RequestUpgrade
  * 功    能：发起升级请求:写标志区(upgrade_req=1)→ 启动复位倒计时
  * 说    明：标志区魔数不对(全新芯片首次升级)时先做全结构初始化,
  *           active_part 由当前运行分区推导,保证 Boot 复位后回滚目标正确
  */
void OTA_RequestUpgrade(void)
{
    ota_flag_t flag;                                     /* 新标志内容 */
    const ota_flag_t *flag_in_flash = (const ota_flag_t *)OTA_FLAG_ADDR; /* 标志区指针 */

    flag = *flag_in_flash;                               /* 读旧内容(保留 active_part 等) */
    if (flag.magic != OTA_FLAG_MAGIC)                    /* 首次使用:全新初始化 */
    {
        flag.magic = OTA_FLAG_MAGIC;
        flag.active_part = ota_current_part();           /* 当前运行分区 */
        flag.new_part = (flag.active_part == 'A') ? 'B' : 'A';   /* 对侧分区 */
        flag.fail_code = 0;
        flag.new_length = 0;
        flag.new_crc32 = 0;
        flag.seq = 0;
    }
    flag.upgrade_req = 1;                                /* 1=请求升级 */
    flag.seq++;                                          /* 写入序号 +1(调试排查用) */

    flag_area_write(&flag);                              /* 落盘(掉电也保持升级请求) */

    s_pending_tick = HAL_GetTick();                      /* 复位倒计时起点 */
    s_reset_pending = 1;                                 /* 启动倒计时 */
}

/**
  * 函    数：OTA_Poll
  * 功    能：复位倒计时轮询(控制任务 100ms 节拍调用)
  * 说    明：到点软复位 → Bootloader 读标志区见 upgrade_req=1 → 进升级模式;
  *           200ms 延时保证触发前的 Modbus 应答/调试回显已从串口发完
  */
void OTA_Poll(void)
{
    if (s_reset_pending &&
        HAL_GetTick() - s_pending_tick >= OTA_RESET_DELAY_MS)
    {
        NVIC_SystemReset();                              /* 软复位,进入 Boot 升级流程 */
    }
}
