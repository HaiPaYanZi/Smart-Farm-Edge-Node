/**
  ******************************************************************************
  * @file    boot_proto.c
  * @brief   OTA 升级协议状态机(Boot 侧,文档《07》命令集全实现)
  *
  * 分层结构:
  *   字节流 ──帧解析状态机──→ 完整帧(CRC16 校验) ──命令分发──→ 业务处理
  *   (boot_uart 环形缓冲)  (本文件)                    (本文件)
  *
  * 命令处理一览(与 PC 工具 ota_tool.py 严格对齐):
  *   0x01 握手       → 0x81 状态+版本+分区
  *   0x02 开始传输   → 0x82 接受/版本过低/长度超限(头 16B 写入目标分区起始)
  *   0x03 数据包     → 0x83 包号+结果(256B/包,攒满 2KB 页擦写一次 Flash)
  *   0x04 传输结束   → 0x84 已收包数(残页补 0xFF 落盘)
  *   0x05 全片校验   → 0x85 通过/不通过(软件 CRC32,与 zlib.crc32 对齐)
  *   0x06 执行升级   → 0x86 接受(写标志区 upgrade_req=2)→ 软复位
  *   0x07 查询进度   → 0x87 已收包数/总包数
  *
  * 超时回滚(文档《07》§3.3):5s 无新帧 → 清升级请求 → 软复位回原分区。
  *   该机制同时覆盖 E07"传输 50% 断电重启":复位后 Boot 见升级请求,
  *   5s 等不到 PC 工具自动回滚,原分区正常启动。
  ******************************************************************************
  */
#include "stm32f1xx.h"      /* NVIC_SystemReset/Flash 地址访问 */
#include <stddef.h>          /* NULL */
#include "boot_proto.h"
#include "boot_uart.h"
#include "boot_flash.h"
#include "boot_crc.h"

/* ==================== 帧接收状态机 ==================== */
typedef enum {
    FRAME_WAIT_HEAD1 = 0,   /* 等帧头第 1 字节 0xAA */
    FRAME_WAIT_HEAD2,       /* 等帧头第 2 字节 0x55 */
    FRAME_RX_BODY,          /* 收帧体:cmd+seq+len+data+crc+tail */
} frame_state_t;

/* ==================== 命令层状态机 ==================== */
typedef enum {
    PROTO_IDLE = 0,         /* 等握手(升级模式入口) */
    PROTO_RX_DATA,          /* 收数据包中(0x02 已接受) */
    PROTO_VERIFIED,         /* CRC32 校验已通过,等 0x06 执行 */
} proto_state_t;

/* ==================== 协议上下文(升级会话的全部状态) ==================== */
typedef struct {
    proto_state_t state;            /* 命令层状态 */
    frame_state_t frame_state;      /* 帧接收状态 */
    uint8_t  frame_body[OTA_FRAME_MAX_LEN];  /* 帧体缓冲(cmd~tail,最大 520B) */
    uint16_t frame_pos;             /* 帧体已收字节数 */
    uint16_t frame_len;             /* 本帧体总长(收完 len 字段后算出) */

    uint8_t  target_part;           /* 升级目标分区 'A' 或 'B'(对侧) */
    uint32_t target_addr;           /* 目标分区起始地址 */
    uint32_t write_offset;          /* 已接受固件字节数(不含 16B 头) */
    uint16_t expected_packet;       /* 期望的下一个包号(0 起) */
    uint16_t packet_count;          /* 已收包数 */
    uint32_t total_packets;         /* 总包数(0x04 告知,(length+255)/256) */
    ota_fw_header_t fw_header;      /* 本次升级固件头(0x02 收到) */
    uint32_t verified_crc32;        /* 0x05 校验通过时的 CRC32(写进标志区) */
    uint32_t last_rx_tick;          /* 上次收到有效帧的时刻(ms,超时回滚用) */

    uint8_t  page_buffer[OTA_FLASH_PAGE_SIZE]; /* 页缓冲:攒满 2KB 才擦写一页 */
    uint16_t page_buffer_len;       /* 页缓冲已攒字节数 */
} proto_ctx_t;

static proto_ctx_t s_ctx;           /* 协议上下文(单实例,Boot 里只会有一个升级会话) */

/* 活动分区(握手应答和头校验用,由 boot_proto_start 传入保存) */
static uint8_t s_active_part = 'A';

/* ==================== 内部函数前置声明 ==================== */
static void proto_send_frame(uint8_t cmd, uint16_t seq, const uint8_t *data, uint16_t len);
static void proto_process_frame(const uint8_t *body, uint16_t body_len);

/**
  * @brief  读升级标志区(0x08003000)当前内容
  * @param  flag 输出:读到的标志结构体
  * @note   直接按地址读结构体(Flash 可随机读,无需初始化)。
  *          公共接口:main.c 三态判定与协议层共用
  */
void boot_flag_read(ota_flag_t *flag)
{
  const ota_flag_t *flag_in_flash = (const ota_flag_t *)OTA_FLAG_ADDR; /* 标志区地址指针 */
  *flag = *flag_in_flash;               /* 结构体拷贝 */
}

/**
  * @brief  写升级标志区:擦页 → 写结构体(掉电保持)
  * @param  flag 要写入的标志结构体
  * @retval 0=成功 1=擦写失败
  * @note   调用方负责先读旧值改字段再写回;擦页约 20~40ms。
  *          公共接口:main.c 三态判定与协议层共用
  */
uint8_t boot_flag_write(const ota_flag_t *flag)
{
  uint8_t result;                       /* 操作结果 */

  boot_flash_unlock();                  /* 解锁 Flash */
  result = boot_flash_erase_page(OTA_FLAG_ADDR);          /* 擦标志页 */
  if (result != 0)
  {
    /* 擦除失败(实物排查 2026-08-17:0x06 后标志区 upgrade_req=1 残留,
       怀疑标志页没写成——打印区分是擦还是写失败) */
    boot_uart_send_string("OTA: flag erase fail\r\n");
    boot_flash_lock();
    return result;
  }
  result = boot_flash_program(OTA_FLAG_ADDR, (const uint8_t *)flag, sizeof(ota_flag_t));
  if (result != 0)
  {
    boot_uart_send_string("OTA: flag program fail\r\n");
  }
  boot_flash_lock();                    /* 上锁 */
  return result;
}

/**
  * @brief  发送一帧:拼帧 → 算 CRC16 → 串口发出
  * @param  cmd  命令字节
  * @param  seq  序列号(应答回显原帧序号)
  * @param  data 数据域(可为 NULL)
  * @param  len  数据域长度
  */
static void proto_send_frame(uint8_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
  uint8_t frame_buffer[OTA_FRAME_MAX_LEN];      /* 出帧缓冲 */
  uint16_t crc16_value;                         /* 帧 CRC16 */
  uint16_t pos = 0;                             /* 拼帧位置 */

  frame_buffer[pos++] = OTA_FRAME_HEAD1;        /* 帧头 0xAA */
  frame_buffer[pos++] = OTA_FRAME_HEAD2;        /* 帧头 0x55 */
  frame_buffer[pos++] = cmd;                    /* 命令 */
  frame_buffer[pos++] = (uint8_t)(seq & 0xFF);  /* 序号低字节 */
  frame_buffer[pos++] = (uint8_t)(seq >> 8);    /* 序号高字节 */
  frame_buffer[pos++] = (uint8_t)(len & 0xFF);  /* 长度低字节 */
  frame_buffer[pos++] = (uint8_t)(len >> 8);    /* 长度高字节 */
  if (len > 0 && data != NULL)
  {
    uint16_t index;                             /* 数据域下标 */
    for (index = 0; index < len; index++)
    {
      frame_buffer[pos++] = data[index];
    }
  }
  /* CRC16-MODBUS 覆盖范围:cmd ~ data 末尾(即帧体前 5+len 字节) */
  crc16_value = boot_crc16_update(OTA_CRC16_INIT, &frame_buffer[2], (uint32_t)5 + len);
  frame_buffer[pos++] = (uint8_t)(crc16_value & 0xFF);  /* CRC 低字节 */
  frame_buffer[pos++] = (uint8_t)(crc16_value >> 8);    /* CRC 高字节 */
  frame_buffer[pos++] = OTA_FRAME_TAIL;         /* 帧尾 0x0D */

  boot_uart_send(frame_buffer, pos);            /* 整帧发出 */
}

/**
  * @brief  校验固件头(0x02 处理):魔数/长度/版本三重检查
  * @param  header 收到的固件头
  * @retval 0=接受  1=版本过低  3=长度超限(与应答码 OTA_ACK_* 对齐)
  * @note   魔数不符也算拒绝(返回 2 分区无效口径,表示头不合法)
  */
static uint8_t header_check(const ota_fw_header_t *header)
{
  if (header->magic != OTA_FW_MAGIC)            /* 魔数不对:非法固件文件 */
  {
    return OTA_ACK_PART_INVALID;
  }
  if (header->length > OTA_FW_MAX_SIZE)         /* 长度超过分区容量 */
  {
    return OTA_ACK_LEN_OVER;
  }
  /* 版本比较:与活动分区现有固件比,不允许降级(语义化版本规则) */
  {
    const ota_fw_header_t *active_header =
        (const ota_fw_header_t *)(s_active_part == 'B' ? OTA_PART_B_ADDR : OTA_PART_A_ADDR);
    if (active_header->magic == OTA_FW_MAGIC)   /* 活动区有有效固件才比较 */
    {
      uint32_t active_version = ((uint32_t)active_header->major << 16)
                              | ((uint32_t)active_header->minor << 8)
                              | active_header->patch;
      uint32_t new_version = ((uint32_t)header->major << 16)
                           | ((uint32_t)header->minor << 8)
                           | header->patch;
      if (new_version < active_version)         /* 新固件版本更低:拒绝 */
      {
        return OTA_ACK_VER_LOW;
      }
    }
  }
  return OTA_ACK_OK;
}

/**
  * @brief  把页缓冲写进目标分区的指定页(擦+写,内部补 0xFF 对齐)
  * @param  page_offset 页在固件数据内的起始偏移(必须 2KB 对齐)
  * @retval 0=成功 1=Flash 操作失败
  */
static uint8_t page_flush(uint32_t page_offset)
{
  /* 补齐后的整页数据:必须 static,不能放栈上!
     Boot 栈仅 2KB(startup 文件 Stack_Size=0x800),2KB 局部数组 + 调用帧
     必然栈溢出;Cortex-M 栈向下生长,栈底 0x20001458 紧贴 .bss 里
     s_ctx.page_buffer 尾部——溢出把 page_buffer 尾部与 page_buffer_len
     踩成栈残留垃圾,每页写入尾部数据被污染,全片 CRC 必失败
     (实物抓到的坑:verify fail local=0x3704353B,五次结构性假设全不命中) */
  static uint8_t flush_data[OTA_FLASH_PAGE_SIZE];
  uint32_t index;                               /* 数据下标 */

  for (index = 0; index < OTA_FLASH_PAGE_SIZE; index++)
  {
    if (index < s_ctx.page_buffer_len)
    {
      flush_data[index] = s_ctx.page_buffer[index];  /* 有效数据 */
    }
    else
    {
      flush_data[index] = 0xFF;                 /* 不足一页:补 0xFF(Flash 擦除态) */
    }
  }

  /* 数据区已在 handle_start 一次性擦净(数据跨页不能逐页擦),这里只写不擦。
     若编程失败(理论不应发生)返回 1,调用方拒绝该包让主机重传 */
  boot_flash_unlock();
  if (boot_flash_program(s_ctx.target_addr + OTA_APP_OFFSET + page_offset,
                         flush_data, OTA_FLASH_PAGE_SIZE) != 0)
  {
    boot_flash_lock();
    return 1;                                   /* 编程失败 */
  }
  boot_flash_lock();

  s_ctx.page_buffer_len = 0;                    /* 清空页缓冲,开始攒下一页 */
  return 0;
}

/* ==================== 命令处理 ==================== */

/**
  * @brief  0x01 握手:应答状态+版本+分区(文档《07》§3.1)
  * @param  seq 原帧序号(应答回显)
  */
static void handle_handshake(uint16_t seq)
{
  uint8_t reply_data[4];                        /* 应答数据域 */
  const ota_fw_header_t *active_header =
      (const ota_fw_header_t *)(s_active_part == 'B' ? OTA_PART_B_ADDR : OTA_PART_A_ADDR);

  reply_data[0] = 0x00;                         /* 状态:就绪 */
  if (active_header->magic == OTA_FW_MAGIC)
  {
    reply_data[1] = active_header->major;       /* 版本:主版本 */
    reply_data[2] = active_header->minor;       /* 版本:次版本 */
  }
  else
  {
    reply_data[1] = 0;                          /* 活动区无有效固件:版本 0 */
    reply_data[2] = 0;
  }
  reply_data[3] = s_active_part;                /* 当前分区 'A'/'B' */

  proto_send_frame(OTA_CMD_HANDSHAKE_ACK, seq, reply_data, 4);
}

/**
  * @brief  0x02 开始传输:校验头 → 写头入目标分区 → 复位会话缓冲
  * @param  seq    原帧序号
  * @param  data   固件头 16B
  * @param  len    数据域长度(必须 16)
  */
static void handle_start(uint16_t seq, const uint8_t *data, uint16_t len)
{
  uint8_t reply_data[1];                        /* 应答数据域(结果码) */

  if (len != sizeof(ota_fw_header_t))           /* 头长度不对:拒绝 */
  {
    reply_data[0] = OTA_ACK_PART_INVALID;
    proto_send_frame(OTA_CMD_START_ACK, seq, reply_data, 1);
    return;
  }

  /* 头校验:魔数/长度/版本 */
  reply_data[0] = header_check((const ota_fw_header_t *)data);
  if (reply_data[0] != OTA_ACK_OK)
  {
    proto_send_frame(OTA_CMD_START_ACK, seq, reply_data, 1);
    return;
  }

  /* 头写入目标分区起始(先擦该页再写,页内其余字节保持 0xFF)。
     注意:数据从 分区+0x200 起(非页边界),每 2KB 数据横跨 2 个 Flash 页,
     必须一次性把数据区覆盖的所有页擦净(含头所在页),page_flush 只写不擦,
     否则写未擦除页会 PGERR 且被静默吞掉,升级全片 CRC 必失败(实物抓到的坑) */
  boot_flash_unlock();
  {
    const ota_fw_header_t *header = (const ota_fw_header_t *)data;  /* 固件头视图 */
    uint32_t data_start = s_ctx.target_addr + OTA_APP_OFFSET;       /* 数据区起点(含头页) */
    uint32_t data_end   = (data_start + header->length + OTA_FLASH_PAGE_SIZE - 1)
                        & ~(OTA_FLASH_PAGE_SIZE - 1);               /* 终点向上取整到页 */
    uint32_t page_addr  = data_start & ~(OTA_FLASH_PAGE_SIZE - 1);  /* 起点对齐到页 */
    uint8_t  erase_fail = 0;                                        /* 擦除失败标志 */

    while (page_addr <= data_end)              /* 擦数据区涉及的全部页(含终点所在页) */
    {
      if (boot_flash_erase_page(page_addr) != 0)
      {
        erase_fail = 1;
        break;
      }
      page_addr += OTA_FLASH_PAGE_SIZE;
    }

    if (erase_fail != 0 ||
        boot_flash_program(s_ctx.target_addr, data, sizeof(ota_fw_header_t)) != 0)
    {
      boot_flash_lock();
      reply_data[0] = OTA_ACK_PART_INVALID;    /* Flash 写失败:拒绝 */
      proto_send_frame(OTA_CMD_START_ACK, seq, reply_data, 1);
      return;
    }
  }
  boot_flash_lock();

  /* 会话复位:清数据区计数,准备收包 */
  s_ctx.fw_header = *(const ota_fw_header_t *)data;  /* 保存头 */
  s_ctx.write_offset = 0;                       /* 已写数据清零 */
  s_ctx.expected_packet = 0;                    /* 从 0 号包收 */
  s_ctx.packet_count = 0;
  s_ctx.page_buffer_len = 0;                    /* 页缓冲清空 */
  s_ctx.total_packets = (uint32_t)((s_ctx.fw_header.length + OTA_PACKET_SIZE - 1)
                                   / OTA_PACKET_SIZE);  /* 总包数自算(向上取整) */
  s_ctx.state = PROTO_RX_DATA;                  /* 进入收包状态 */
  s_ctx.verified_crc32 = 0;

  reply_data[0] = OTA_ACK_OK;
  proto_send_frame(OTA_CMD_START_ACK, seq, reply_data, 1);
  boot_uart_send_string("OTA: start accepted, packets=");
  boot_uart_send_hex((uint8_t)(s_ctx.total_packets >> 8));   /* 总包数高字节(十六进制显示) */
  boot_uart_send_hex((uint8_t)(s_ctx.total_packets & 0xFF)); /* 总包数低字节 */
  boot_uart_send_newline();
}

/**
  * @brief  0x03 数据包:包号校验 → 攒页缓冲 → 满页擦写(文档《07》§3.3)
  * @param  seq  原帧序号
  * @param  data 数据域 = 包号(2B 低前) + 固件数据(≤256B)
  * @param  len  数据域长度
  */
static void handle_data(uint16_t seq, const uint8_t *data, uint16_t len)
{
  uint8_t reply_data[3];                        /* 应答:包号(2B)+结果(1B) */
  uint16_t packet_number;                       /* 本帧携带的包号 */
  uint16_t payload_len;                         /* 固件数据字节数 */

  if (s_ctx.state != PROTO_RX_DATA)             /* 未开始传输:拒绝 */
  {
    return;
  }
  if (len < 2)                                  /* 连包号都不够:非法帧 */
  {
    return;
  }

  packet_number = (uint16_t)(data[0] | (uint16_t)data[1] << 8);  /* 包号(低字节在前) */
  payload_len = (uint16_t)(len - 2);            /* 数据域去掉包号后的长度 */

  /* 包号分支:等于期望=正常;小于=重传旧包;大于=中间丢包 */
  if (packet_number == s_ctx.expected_packet)
  {
    /* 数据不能超过 256B,且不能越过固件总长 */
    if (payload_len > OTA_PACKET_SIZE ||
        s_ctx.write_offset + payload_len > s_ctx.fw_header.length)
    {
      reply_data[0] = (uint8_t)(packet_number & 0xFF);
      reply_data[1] = (uint8_t)(packet_number >> 8);
      reply_data[2] = OTA_ACK_CRC_FAIL;         /* 数据非法:按校验失败处理,主机重发 */
      proto_send_frame(OTA_CMD_DATA_ACK, seq, reply_data, 3);
      return;
    }

    /* 数据拷进页缓冲 */
    {
      uint16_t index;                           /* 数据下标 */
      for (index = 0; index < payload_len; index++)
      {
        s_ctx.page_buffer[s_ctx.page_buffer_len + index] = data[2 + index];
      }
    }
    s_ctx.page_buffer_len = (uint16_t)(s_ctx.page_buffer_len + payload_len);
    s_ctx.write_offset += payload_len;
    s_ctx.expected_packet++;                    /* 期望包号 +1 */
    s_ctx.packet_count++;

    /* 页缓冲攒满 2KB:写一页(最后一页不满等 0x04 时 flush) */
    if (s_ctx.page_buffer_len >= OTA_FLASH_PAGE_SIZE)
    {
      uint32_t page_offset = s_ctx.write_offset - s_ctx.page_buffer_len;
      if (page_flush(page_offset) != 0)
      {
        /* Flash 写失败:拒绝本包,主机重发(连续失败则主机中止) */
        reply_data[0] = (uint8_t)(packet_number & 0xFF);
        reply_data[1] = (uint8_t)(packet_number >> 8);
        reply_data[2] = OTA_ACK_CRC_FAIL;
        proto_send_frame(OTA_CMD_DATA_ACK, seq, reply_data, 3);
        return;
      }
    }

    reply_data[0] = (uint8_t)(packet_number & 0xFF);
    reply_data[1] = (uint8_t)(packet_number >> 8);
    reply_data[2] = OTA_ACK_OK;
    proto_send_frame(OTA_CMD_DATA_ACK, seq, reply_data, 3);
  }
  else if (packet_number < s_ctx.expected_packet)
  {
    /* 旧包重传(主机没收到上次应答):忽略数据,重答成功 */
    reply_data[0] = (uint8_t)(packet_number & 0xFF);
    reply_data[1] = (uint8_t)(packet_number >> 8);
    reply_data[2] = OTA_ACK_OK;
    proto_send_frame(OTA_CMD_DATA_ACK, seq, reply_data, 3);
  }
  else
  {
    /* 丢包(包号超前):应答"失败+期望包号",主机从期望包续传 */
    reply_data[0] = (uint8_t)(s_ctx.expected_packet & 0xFF);
    reply_data[1] = (uint8_t)(s_ctx.expected_packet >> 8);
    reply_data[2] = OTA_ACK_CRC_FAIL;
    proto_send_frame(OTA_CMD_DATA_ACK, seq, reply_data, 3);
  }
}

/**
  * @brief  0x04 传输结束:flush 残页 → 核对总包数 → 应答已收包数
  * @param  seq  原帧序号
  * @param  data 总包数(2B 低前)
  */
static void handle_finish(uint16_t seq, const uint8_t *data, uint16_t len)
{
  uint8_t reply_data[2];                        /* 应答:已收包数(2B) */

  if (s_ctx.state != PROTO_RX_DATA || len < 2)
  {
    return;
  }

  /* 残页落盘(最后一页不满 2KB,补 0xFF 写) */
  if (s_ctx.page_buffer_len > 0)
  {
    uint32_t page_offset = s_ctx.write_offset - s_ctx.page_buffer_len;
    if (page_flush(page_offset) != 0)
    {
      return;                                 /* 残页写失败:无应答,主机超时重发 0x04 */
    }
  }

  /* 主机告知的总包数与自算值比对:不符则应答里带回已收数,主机续传 */
  {
    uint32_t host_packets = (uint32_t)(data[0] | (uint16_t)data[1] << 8);
    if (host_packets != s_ctx.total_packets)
    {
      s_ctx.total_packets = host_packets;       /* 以主机为准(头算的是向上取整,一致才正常) */
    }
  }

  reply_data[0] = (uint8_t)(s_ctx.packet_count & 0xFF);
  reply_data[1] = (uint8_t)(s_ctx.packet_count >> 8);
  proto_send_frame(OTA_CMD_FINISH_ACK, seq, reply_data, 2);
}

/**
  * @brief  0x05 全片校验:软件 CRC32 读全片与主机值比对(文档《07》§3.4)
  * @param  seq  原帧序号
  * @param  data 固件 CRC32(4B 低前)
  */
static void handle_verify(uint16_t seq, const uint8_t *data, uint16_t len)
{
  uint8_t reply_data[1];                        /* 应答:结果 */
  uint32_t crc_mid;                             /* CRC32 中间值 */
  uint32_t remain;                              /* 剩余未算字节数 */
  uint32_t read_offset;                         /* 读取位置 */

  if (s_ctx.state != PROTO_RX_DATA || len < 4)
  {
    return;
  }

  /* 分段读 Flash 计算 CRC32(每段 256B,栈上小缓冲) */
  crc_mid = OTA_CRC32_INIT;
  remain = s_ctx.fw_header.length;
  read_offset = 0;
  while (remain > 0)
  {
    uint32_t segment = remain > 256 ? 256 : remain;   /* 本段字节数 */
    uint8_t segment_buffer[256];                      /* 段缓冲(栈上,256B) */
    const uint8_t *flash_ptr =
        (const uint8_t *)(s_ctx.target_addr + OTA_APP_OFFSET + read_offset);
    uint32_t index;                                   /* 段内下标 */
    for (index = 0; index < segment; index++)
    {
      segment_buffer[index] = flash_ptr[index];       /* Flash 直接读 */
    }
    crc_mid = boot_crc32_update(crc_mid, segment_buffer, segment);
    remain -= segment;
    read_offset += segment;
  }

  /* 主机值(低字节在前)与本地最终值比对 */
  {
    uint32_t host_crc = (uint32_t)data[0] | (uint32_t)data[1] << 8
                      | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
    uint32_t local_crc = boot_crc32_final(crc_mid);

    if (host_crc == local_crc)
    {
      reply_data[0] = OTA_ACK_OK;
      s_ctx.verified_crc32 = local_crc;         /* 记录通过值(0x06 写标志区用) */
      s_ctx.state = PROTO_VERIFIED;             /* 进入待执行状态 */
    }
    else
    {
      reply_data[0] = OTA_ACK_CRC_FAIL;
      boot_uart_send_string("OTA: verify fail local=");
      boot_uart_send_hex((uint8_t)(local_crc >> 24));
      boot_uart_send_hex((uint8_t)(local_crc >> 16));
      boot_uart_send_hex((uint8_t)(local_crc >> 8));
      boot_uart_send_hex((uint8_t)local_crc);
      boot_uart_send_newline();
    }
  }
  proto_send_frame(OTA_CMD_VERIFY_ACK, seq, reply_data, 1);
}

/**
  * @brief  0x06 执行升级:写标志区(upgrade_req=2)→ 应答 → 软复位(文档《07》§3.5)
  * @param  seq  原帧序号
  * @param  data 目标分区(1B,必须=对侧分区)
  * @note   复位后 Boot 主流程见 upgrade_req=2:全片 CRC32 复验 → 通过切分区跳新,
  *         失败保持旧分区并置故障码 0x0302(文档回滚路径)
  */
static void handle_execute(uint16_t seq, const uint8_t *data, uint16_t len)
{
  uint8_t reply_data[1];                        /* 应答:结果 */

  if (s_ctx.state != PROTO_VERIFIED || len < 1)
  {
    return;
  }
  if (data[0] != s_ctx.target_part)             /* 目标分区与对侧不符:拒绝 */
  {
    reply_data[0] = OTA_ACK_PART_INVALID;
    proto_send_frame(OTA_CMD_EXECUTE_ACK, seq, reply_data, 1);
    return;
  }

  /* 写标志区:升级请求=待验证,记录长度/CRC/目标分区 */
  {
    ota_flag_t flag;                            /* 新标志内容 */
    uint8_t wr_result;                          /* 写入结果(0=成功 1=失败) */
    boot_flag_read(&flag);                      /* 先读旧值(保留 active_part 等) */
    flag.upgrade_req = 2;                       /* 2=升级完成待验证 */
    flag.new_part = s_ctx.target_part;
    flag.new_length = s_ctx.fw_header.length;
    flag.new_crc32 = s_ctx.verified_crc32;
    flag.fail_code = 0;                         /* 清故障码 */
    flag.seq++;                                 /* 写入序号 +1 */
    wr_result = boot_flag_write(&flag);
    /* 实物排查 2026-08-17:升级后标志区 upgrade_req=1 残留(0x06 写没生效),
       这里打印写入结果与关键字段,PC 端据此定位 */
    boot_uart_send_string("OTA: flag wr=");
    boot_uart_send_hex(wr_result);              /* 0=写入成功 1=失败 */
    boot_uart_send_string(" req=");
    boot_uart_send_hex(flag.upgrade_req);
    boot_uart_send_string(" new=");
    boot_uart_send_hex(flag.new_part);
    boot_uart_send_string(" len=");
    boot_uart_send_hex((uint8_t)(flag.new_length >> 16));
    boot_uart_send_hex((uint8_t)(flag.new_length >> 8));
    boot_uart_send_hex((uint8_t)flag.new_length);
    boot_uart_send_newline();
  }

  reply_data[0] = OTA_ACK_OK;
  proto_send_frame(OTA_CMD_EXECUTE_ACK, seq, reply_data, 1);
  boot_uart_send_string("OTA: execute, reboot...\r\n");

  /* 等最后一帧发完再复位:轮询 TC(发送完成)标志 */
  while ((USART1->SR & USART_SR_TC) == 0) {}
  NVIC_SystemReset();                           /* 软复位,Boot 主流程处理验证与切换 */
}

/**
  * @brief  0x07 查询进度(文档《07》§3.6,可选)
  * @param  seq 原帧序号
  */
static void handle_progress(uint16_t seq)
{
  uint8_t reply_data[4];                        /* 应答:已收包数(2B)+总包数(2B) */

  reply_data[0] = (uint8_t)(s_ctx.packet_count & 0xFF);
  reply_data[1] = (uint8_t)(s_ctx.packet_count >> 8);
  reply_data[2] = (uint8_t)(s_ctx.total_packets & 0xFF);
  reply_data[3] = (uint8_t)(s_ctx.total_packets >> 8);
  proto_send_frame(OTA_CMD_PROGRESS_ACK, seq, reply_data, 4);
}

/* ==================== 帧解析与分发 ==================== */

/**
  * @brief  处理一帧完整帧体(cmd~tail,CRC 已校验通过)
  * @param  body     帧体起始(指向 cmd 字节)
  * @param  body_len 帧体长度(5+len+2+1)
  */
static void proto_process_frame(const uint8_t *body, uint16_t body_len)
{
  uint8_t cmd = body[0];                        /* 命令字节 */
  uint16_t seq = (uint16_t)(body[1] | (uint16_t)body[2] << 8);   /* 序号 */
  uint16_t len = (uint16_t)(body[3] | (uint16_t)body[4] << 8);   /* 数据域长度 */
  const uint8_t *data_ptr = &body[5];           /* 数据域起始 */

  s_ctx.last_rx_tick = g_tick_ms;               /* 刷新收帧时间(超时回滚计时) */

  /* 按命令分发(数据帧只在收包状态处理,握手任何时候都答) */
  switch (cmd)
  {
    case OTA_CMD_HANDSHAKE:                     /* 0x01 握手 */
      handle_handshake(seq);
      break;

    case OTA_CMD_START:                         /* 0x02 开始传输 */
      handle_start(seq, data_ptr, len);
      break;

    case OTA_CMD_DATA:                          /* 0x03 数据包 */
      handle_data(seq, data_ptr, len);
      break;

    case OTA_CMD_FINISH:                        /* 0x04 传输结束 */
      handle_finish(seq, data_ptr, len);
      break;

    case OTA_CMD_VERIFY:                        /* 0x05 全片校验 */
      handle_verify(seq, data_ptr, len);
      break;

    case OTA_CMD_EXECUTE:                       /* 0x06 执行升级 */
      handle_execute(seq, data_ptr, len);
      break;

    case OTA_CMD_PROGRESS:                      /* 0x07 查询进度 */
      handle_progress(seq);
      break;

    default:                                    /* 未知命令:丢弃不应答 */
      break;
  }
  (void)body_len;
}

/**
  * @brief  进入升级模式:初始化协议上下文
  * @param  active_part 当前活动分区 'A' 或 'B'
  */
void boot_proto_start(uint8_t active_part)
{
  s_active_part = active_part;                  /* 保存活动分区 */
  s_ctx.state = PROTO_IDLE;                     /* 等握手 */
  s_ctx.frame_state = FRAME_WAIT_HEAD1;         /* 帧解析从头开始 */
  s_ctx.frame_pos = 0;
  s_ctx.frame_len = 0;
  s_ctx.target_part = (active_part == 'A') ? 'B' : 'A';   /* 目标=对侧分区 */
  s_ctx.target_addr = (s_ctx.target_part == 'A') ? OTA_PART_A_ADDR : OTA_PART_B_ADDR;
  s_ctx.write_offset = 0;
  s_ctx.expected_packet = 0;
  s_ctx.packet_count = 0;
  s_ctx.total_packets = 0;
  s_ctx.page_buffer_len = 0;
  s_ctx.last_rx_tick = g_tick_ms;              /* 超时从进入升级模式起算 */
  s_ctx.verified_crc32 = 0;

  boot_uart_send_string("OTA: bootloader upgrade mode, waiting host...\r\n");
}

/**
  * @brief  协议主循环:每圈调用(帧解析 + 超时回滚检查)
  * @note   字节流解析容错:帧头后任何字节不符就丢弃回 WAIT_HEAD1,
  *         Boot 调试打印的文本混在串口流里也不会卡死解析
  */
void boot_proto_poll(void)
{
  /* ---- 第 1 步:超时检查(5s 无有效帧 → 回滚原分区) ---- */
  if (g_tick_ms - s_ctx.last_rx_tick > OTA_PACKET_TIMEOUT_MS)
  {
    ota_flag_t flag;                            /* 标志区内容 */
    boot_flag_read(&flag);
    if (flag.magic == OTA_FLAG_MAGIC)
    {
      flag.upgrade_req = 0;                     /* 清升级请求 */
      flag.fail_code = 0;
      flag.seq++;
      boot_flag_write(&flag);                   /* 落盘 */
    }
    boot_uart_send_string("OTA: timeout, rollback...\r\n");
    while ((USART1->SR & USART_SR_TC) == 0) {}
    NVIC_SystemReset();                         /* 回原分区运行 */
    return;
  }

  /* ---- 第 2 步:字节流帧解析 ---- */
  while (boot_uart_available() > 0)
  {
    int32_t byte_value = boot_uart_read_byte();   /* 取一个字节 */
    if (byte_value < 0)
    {
      break;
    }

    switch (s_ctx.frame_state)
    {
      case FRAME_WAIT_HEAD1:                      /* 等 0xAA */
        if (byte_value == OTA_FRAME_HEAD1)
        {
          s_ctx.frame_state = FRAME_WAIT_HEAD2;
        }
        break;

      case FRAME_WAIT_HEAD2:                      /* 等 0x55 */
        if (byte_value == OTA_FRAME_HEAD2)
        {
          s_ctx.frame_state = FRAME_RX_BODY;
          s_ctx.frame_pos = 0;
          s_ctx.frame_len = 0;
        }
        else if (byte_value != OTA_FRAME_HEAD1)   /* 不是 0xAA 0x55:丢弃回等头 1 */
        {
          s_ctx.frame_state = FRAME_WAIT_HEAD1;
        }
        /* 若是 0xAA:留在 HEAD2 状态继续等 0x55(支持 AA AA 55 连续) */
        break;

      case FRAME_RX_BODY:                         /* 收帧体 */
        s_ctx.frame_body[s_ctx.frame_pos++] = (uint8_t)byte_value;

        /* 收到 5 字节(cmd+seq+len)后算出帧体总长 */
        if (s_ctx.frame_pos == 5)
        {
          uint16_t data_len = (uint16_t)(s_ctx.frame_body[3]
                             | (uint16_t)s_ctx.frame_body[4] << 8);  /* 数据域长度 */
          if (data_len > OTA_FRAME_DATA_MAX)      /* 长度非法:丢弃整帧 */
          {
            s_ctx.frame_state = FRAME_WAIT_HEAD1;
            s_ctx.frame_pos = 0;
            break;
          }
          /* 帧体总长 = cmd+seq+len(5) + data + crc(2) + tail(1) */
          s_ctx.frame_len = (uint16_t)(5 + data_len + 3);
        }

        /* 收满整帧体:校验 CRC16 与帧尾 */
        if (s_ctx.frame_pos >= s_ctx.frame_len && s_ctx.frame_len > 0)
        {
          uint16_t crc_received;                  /* 帧内 CRC */
          uint16_t crc_calc;                      /* 计算 CRC */
          uint16_t data_len = (uint16_t)(s_ctx.frame_body[3]
                             | (uint16_t)s_ctx.frame_body[4] << 8);

          crc_received = (uint16_t)(s_ctx.frame_body[5 + data_len]
                        | (uint16_t)s_ctx.frame_body[6 + data_len] << 8);
          crc_calc = boot_crc16_update(OTA_CRC16_INIT, s_ctx.frame_body,
                                       (uint32_t)5 + data_len);
          if (crc_received == crc_calc &&
              s_ctx.frame_body[7 + data_len] == OTA_FRAME_TAIL)  /* 帧尾检查 */
          {
            proto_process_frame(s_ctx.frame_body, s_ctx.frame_len);  /* 分发处理 */
          }
          /* CRC/帧尾不符:静默丢弃(主机超时重发),回到等帧头 */
          s_ctx.frame_state = FRAME_WAIT_HEAD1;
          s_ctx.frame_pos = 0;
          s_ctx.frame_len = 0;
        }
        break;

      default:
        s_ctx.frame_state = FRAME_WAIT_HEAD1;     /* 异常状态兜底 */
        break;
    }
  }
}
