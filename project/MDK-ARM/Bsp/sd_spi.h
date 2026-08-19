#ifndef __SD_SPI_H__
#define __SD_SPI_H__

#include <stdint.h>

/* 返回码 */
#define SD_OK       0   /* 成功 */
#define SD_ERROR    1   /* 命令/数据错误(卡不应答、token 不对、写拒绝) */
#define SD_TIMEOUT  2   /* 等待超时(卡忙死/初始化不配合) */

/* SD 卡扇区固定 512B(diskio 层 ioctl(GET_SECTOR_SIZE) 也要用) */
#define SD_BLOCK_SIZE   512

/* SD 卡初始化:SPI 低速握手(CMD0/CMD8/ACMD41)成功后切 18MHz 高速,
   返回 SD_OK 才能执行读写。首次挂载(FatFs disk_initialize)时自动调用 */
uint8_t SD_Init(void);

/* 读单块:sector 为逻辑块号(LBA),固定 512 字节读进 buf */
uint8_t SD_ReadBlock(uint8_t *buf, uint32_t sector);

/* 连续读 count 块(CMD18 多块读,结束自动发 CMD12 停止) */
uint8_t SD_ReadMultiBlock(uint8_t *buf, uint32_t sector, uint16_t count);

/* 写单块:写完自动等卡忙结束 */
uint8_t SD_WriteBlock(const uint8_t *buf, uint32_t sector);

/* 连续写 count 块:内部逐块写,每块等忙 */
uint8_t SD_WriteMultiBlock(const uint8_t *buf, uint32_t sector, uint16_t count);

/* 读卡容量(总块数,由 CSD 寄存器解析)——disk_ioctl(GET_SECTOR_COUNT) 用 */
uint32_t SD_GetSectorCount(void);

#endif /* __SD_SPI_H__ */
