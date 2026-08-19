#ifndef __OTA_COMMON_H__
#define __OTA_COMMON_H__

/**
  ******************************************************************************
  * @file    ota_common.h
  * @brief   OTA 升级三方共享定义(节点 1.7,文档《07》+《05》§3.2)
  *
  * 三个工程共用本文件,改协议/分区必须同步改三处:
  *   1. Bootloader 工程(project/Boot)      —— include 本头
  *   2. App 工程(project/MDK-ARM)          —— include 本头
  *   3. PC 升级工具(tools/ota_tool.py)     —— Python 不读 C 头,常量在工具里手抄,
  *                                          每处定义旁注释"与 ota_common.h 对齐"
  ******************************************************************************
  */
#include <stdint.h>

/* ==================== Flash 分区布局(文档《07》§5,按 RCT6 实物勘误) ====================
   RCT6 为高密度器件:Flash 256KB,页大小 2KB(文档原写 1KB/页有误,勘误见文档变更记录)
   ┌──────────────┬───────────────┬────────────────────────────────────────┐
   │ Bootloader   │ 0x08000000    │ 8KB   升级流程+跳转                     │
   │ 参数区(NVS)   │ 0x0800F000    │ 2 页  现有实现,本节点不动(双区轮换)      │
   │ 升级标志区    │ 0x08003000    │ 1 页  boot_flag_t,掉电保持              │
   │ App A        │ 0x08010000    │ 100KB 默认运行分区                      │
   │ App B        │ 0x08020000    │ 100KB 升级目标分区                      │
   └──────────────┴───────────────┴────────────────────────────────────────┘
   固件头 16B 放分区起始地址,App 向量表从 分区+0x200 开始
   (固件头不能与向量表重叠,故向量表整体后移 512B,VTOR 需 512 对齐刚好满足) */
#define OTA_BOOT_ADDR         0x08000000UL   /* Bootloader 起始 */
#define OTA_BOOT_SIZE         (8UL * 1024UL) /* Bootloader 大小 8KB */
#define OTA_FLAG_ADDR         0x08003000UL   /* 升级标志区起始(1 页 2KB) */
#define OTA_PART_A_ADDR       0x08010000UL   /* App A 区起始 */
#define OTA_PART_B_ADDR       0x08020000UL   /* App B 区起始 */
#define OTA_PART_SIZE         (100UL * 1024UL) /* 单分区大小 100KB */
#define OTA_APP_OFFSET        0x200UL        /* App 向量表相对分区起始的偏移(给固件头留 512B) */
#define OTA_FLASH_PAGE_SIZE   2048UL         /* 高密度器件页大小 2KB */

/* 升级数据包大小 256B(文档《07》§3.3"每包数据必须写满 256B") */
#define OTA_PACKET_SIZE       256UL

/* 固件最大长度 = 分区大小 - 固件头段 512B(超限 Boot 拒绝,应答 0x03 长度超限) */
#define OTA_FW_MAX_SIZE       (OTA_PART_SIZE - OTA_APP_OFFSET)

/* ==================== 固件头(文档《05》§3.2,16B,放分区起始) ==================== */
#define OTA_FW_MAGIC          0x4D4F4749UL   /* "GWNM" 小端存储后的值 */

#pragma pack(push, 1)                        /* 按 1 字节对齐:结构体严格 16 字节 */
typedef struct {
    uint32_t magic;      /* 固件魔数 0x4D4F4749 "GWNM":Boot 判定分区是否有效 */
    uint8_t  major;      /* 主版本:架构/协议不兼容变更 */
    uint8_t  minor;      /* 次版本:新增功能 */
    uint8_t  patch;      /* 补丁版本:缺陷修复 */
    uint8_t  flags;      /* 保留(文档:bit0 A 区有效 bit1 B 区有效,实际有效性由标志区 active_part 管理) */
    uint32_t length;     /* 固件长度(不含 16B 头,单位字节) */
    uint32_t crc32;      /* 固件数据 CRC32(不含头,PC 工具打包时计算) */
} ota_fw_header_t;
#pragma pack(pop)

/* ==================== 升级标志区(0x08003000,整页 2KB 只用一个结构体) ==================== */
#define OTA_FLAG_MAGIC        0x42544653UL   /* "SFTB" 标志区自身有效标记 */

#pragma pack(push, 1)                        /* 1 字节对齐:跨编译器结构体布局一致 */
typedef struct {
    uint32_t magic;          /* 标志区魔数:上电读后不等则视为"首次上电" */
    uint8_t  upgrade_req;    /* 升级请求状态:0=正常 1=请求升级(Boot 进升级模式) 2=升级完成待验证 */
    uint8_t  active_part;    /* 当前活动分区:'A'=0x41 或 'B'=0x42 */
    uint8_t  new_part;       /* 本次升级写入的分区(对侧分区),'A' 或 'B' */
    uint16_t fail_code;      /* 上次升级失败码(文档《07》:0x0302 回滚,0=无故障,2 字节) */
    uint32_t new_length;     /* 本次升级固件长度(0x05 校验通过后写入) */
    uint32_t new_crc32;      /* 本次升级固件 CRC32(0x05 校验通过后写入) */
    uint32_t seq;            /* 标志区写入序号:每次写入 +1(调试排查用) */
} ota_flag_t;
#pragma pack(pop)

/* ==================== 协议帧(文档《07》§2) ====================
   帧结构: 0xAA 0x55 | cmd(1B) | seq(2B) | len(2B) | data(N≤512) | CRC16(2B) | 0x0D
   CRC16 = CRC16-MODBUS(多项式 0xA001),覆盖范围:cmd ~ data 末尾 */
#define OTA_FRAME_HEAD1       0xAA           /* 帧头字节 1 */
#define OTA_FRAME_HEAD2       0x55           /* 帧头字节 2 */
#define OTA_FRAME_TAIL        0x0D           /* 帧尾字节 */
#define OTA_FRAME_DATA_MAX    512            /* 数据域最大字节数(文档:0~512) */
#define OTA_FRAME_OVERHEAD    8              /* 帧固定开销:帧头2+cmd1+seq2+len2+帧尾1 = 8 */

/* 帧总长 = 开销 8 + 数据 N + CRC16 2 */
#define OTA_FRAME_MAX_LEN     (OTA_FRAME_OVERHEAD + OTA_FRAME_DATA_MAX + 2)

/* ==================== 命令集(文档《07》§3) ====================
   方向约定:HOST=主机(PC 工具)→设备,DEV=设备(Boot)→主机 */
#define OTA_CMD_HANDSHAKE     0x01           /* HOST:握手,数据域=固件版本(2B,可 0) */
#define OTA_CMD_HANDSHAKE_ACK 0x81           /* DEV :状态(1B)+当前版本(2B)+分区(1B) */

#define OTA_CMD_START         0x02           /* HOST:开始传输,数据域=固件头 16B */
#define OTA_CMD_START_ACK     0x82           /* DEV :结果(1B) 0=接受 1=版本过低 2=分区无效 3=长度超限 */

#define OTA_CMD_DATA          0x03           /* HOST:数据包,数据域=包号(2B)+数据(256B) */
#define OTA_CMD_DATA_ACK      0x83           /* DEV :包号(2B)+结果(1B) 0=成功 1=校验失败(重传) */

#define OTA_CMD_FINISH        0x04           /* HOST:传输结束,数据域=总包数(2B) */
#define OTA_CMD_FINISH_ACK    0x84           /* DEV :已收包数(2B),与总包数不符则主机从缺失包续传 */

#define OTA_CMD_VERIFY        0x05           /* HOST:全片校验,数据域=固件 CRC32(4B) */
#define OTA_CMD_VERIFY_ACK    0x85           /* DEV :结果(1B) 0=通过 1=不通过 */

#define OTA_CMD_EXECUTE       0x06           /* HOST:执行升级,数据域=目标分区(1B,须=对侧分区) */
#define OTA_CMD_EXECUTE_ACK   0x86           /* DEV :结果(1B) 0=已接受即将重启 */

#define OTA_CMD_PROGRESS      0x07           /* HOST:查询进度(可选) */
#define OTA_CMD_PROGRESS_ACK  0x87           /* DEV :已收包数(2B)+总包数(2B) */

/* 应答结果码 */
#define OTA_ACK_OK            0x00           /* 成功/接受 */
#define OTA_ACK_VER_LOW       0x01           /* 版本过低拒绝 */
#define OTA_ACK_PART_INVALID  0x02           /* 分区无效 */
#define OTA_ACK_LEN_OVER      0x03           /* 长度超限 */
#define OTA_ACK_CRC_FAIL      0x01           /* 校验失败(0x83/0x85 复用) */

/* 升级超时(文档《07》§3.3:收包 N s 无新包 → 丢弃本次升级回滚)
   5000 → 30000:人肉验证(手动 HEX 发帧)操作窗口太紧,放宽到 30s;
   工具传输时每包都有应答,正常流程不会拖到超时,30s 只是兜底 */
#define OTA_PACKET_TIMEOUT_MS 30000UL

/* 升级失败码(文档《07》§3.5:回滚置故障码 0x0302) */
#define OTA_FAIL_ROLLBACK     0x0302         /* 新分区校验失败,已回滚 */

#endif /* __OTA_COMMON_H__ */
