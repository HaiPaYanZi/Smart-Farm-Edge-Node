/* SPI 版 SD 卡驱动(F103 SPI2 + PB12 片选)
 *
 * 原理一句话:SD 卡原生 SPI 协议——上电后先用低速时钟(≤400kHz)发 CMD0
 * 把卡从 SD 模式拉进 SPI 模式,握手成功后再把 SPI 切到高速,
 * 之后所有读写都是"命令帧 + 数据 token + 512B 数据"的块传输。
 * CMD55+ACMD41 是 SD 卡"应用专用命令"的标准入口,卡初始化靠它轮询。
 *
 * 接线:PB13=SCK  PB14=MISO  PB15=MOSI  PB12=CS(低有效)
 * 注意:SD 卡 CS 平时必须拉高(释放),选中才拉低;总线空闲要发时钟维持。
 */
#include "sd_spi.h"
#include <stm32f1xx_hal.h>
#include "log.h"                /* 临时:SD_Init 失败步骤定位日志(排查后保留无妨) */

/* SPI2 句柄:spi.c 由 CubeMX 生成 */
extern SPI_HandleTypeDef hspi2;

/* ---- 片选引脚(PB12,CubeMX 已配为推挽输出,初始高) ---- */
#define SD_CS_PORT      GPIOB
#define SD_CS_PIN       GPIO_PIN_12
#define SD_CS_LOW()     HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

/* ---- SPI 速率:初始化必须 <400kHz,握手完成后切到最大 18MHz ---- */
#define SD_INIT_PRESCALER   SPI_BAUDRATEPRESCALER_256  /* 36MHz/256 ≈ 140kHz */
#define SD_HIGH_PRESCALER   SPI_BAUDRATEPRESCALER_2    /* 36MHz/2   = 18MHz  */

/* ---- 命令帧首字节 = 0x40 | 索引(bit6=1 表示 host→card 命令) ---- */
#define CMD0    0x40    /* GO_IDLE_STATE     :复位,卡回空闲态(进 SPI 模式的关键) */
#define CMD8    0x48    /* SEND_IF_COND      :2.0 协议握手,回显 0x1AA 证明支持 */
#define CMD9    0x49    /* SEND_CSD          :读 CSD 寄存器(拿容量) */
#define CMD12   0x4C    /* STOP_TRANSMISSION :结束多块读/写 */
#define CMD17   0x51    /* READ_SINGLE_BLOCK :读单块 */
#define CMD18   0x52    /* READ_MULTIPLE     :读多块(直到 CMD12) */
#define CMD24   0x58    /* WRITE_BLOCK       :写单块 */
#define CMD25   0x59    /* WRITE_MULTIPLE    :写多块 */
#define CMD55   0x77    /* APP_CMD           :前缀,下一条命令变成 ACMD */
#define ACMD41  0x69    /* (CMD55 后)SD_SEND_OP_COND:初始化,0x40000000=HCS 位 */

/* ---- 数据 token ---- */
#define SD_TOKEN_READ   0xFE    /* 读数据起始 */
#define SD_TOKEN_WRITE  0xFE    /* 写单块数据起始 */
#define SD_TOKEN_MWRITE 0xFC    /* 写多块每块数据起始 */

/* 忙等待超时计数(72MHz 下约数百 ms,卡再慢也够) */
#define SD_BUSY_TIMEOUT 0x100000

/* 卡状态 */
static uint8_t   s_sd_type = 0;      /* 0=未初始化,1=标准容量卡(V1),2=高容量卡(SDHC V2) */
static uint32_t  s_sector_cnt = 0;   /* 总块数(初始化成功后有效) */

/* SPI 硬件错误标志:任何一次收发返回非 HAL_OK 就置位。
   用来区分两类失败:外设级失败(主机没发出去) vs 卡不应答(发了没人回)。
   HAL_ERROR = 句柄状态不对(未初始化/被占用),HAL_TIMEOUT = 外设无时钟等不到 TXE */
static volatile uint8_t s_spi_hw_err = 0;

/*
单字节收发:SPI 全双工,发一个字节同时收回一个字节。
读数据就是"发 0xFF 收数据",写数据就是"发数据收垃圾"。
*/
static uint8_t SD_Byte(uint8_t out)
{
    uint8_t in = 0xFF;//SPI的发送与接收其实就是对寄存器进行数据的交换
    if (HAL_SPI_TransmitReceive(&hspi2, &out, &in, 1, 100) != HAL_OK)//参数：SPI 句柄,发送数据指针,接收数据指针,数据长度,超时时间
    {
        s_spi_hw_err = 1;   /* 记一次外设级失败,SD_Init 的失败日志会区分报告 */
    }
    return in;
}

/* 空闲时钟:CS 释放期间也持续发时钟,SD 卡靠时钟推进状态机 */
static void SD_ClkIdle(uint32_t n)
{
    while (n--) { SD_Byte(0xFF); }
}

/* 发命令帧并等 R1 应答。
   命令帧 = 6 字节:首字节(命令号) + 4 字节参数(大端) + 1 字节 CRC|停止位。
   SPI 模式下除 CMD0/CMD8 外 CRC 不校验(填 0x01),参数自定义。 */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t frame[6];
    uint8_t r1;

    frame[0] = cmd;
    frame[1] = (arg >> 24) & 0xFF;
    frame[2] = (arg >> 16) & 0xFF;
    frame[3] = (arg >> 8)  & 0xFF;
    frame[4] = arg & 0xFF;
    frame[5] = (crc << 1) | 1;      /* CRC 左移 1 位,最低位恒 1(停止位) */

    for (uint8_t i = 0; i < 6; i++) { SD_Byte(frame[i]); }

    /* R1 前可能有若干 0xFF(卡正在处理),读到 bit7=0 才算应答开始 */
    r1 = 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
        r1 = SD_Byte(0xFF);
        if ((r1 & 0x80) == 0) { break; }
    }
    return r1;
}

/* 等卡空闲:卡忙时数据线输出 0x00,空闲回 0xFF。
   写块后必须等卡把数据真正落盘(可能要几十 ms)。 */
static uint8_t SD_WaitBusy(void)
{
    uint32_t cnt = SD_BUSY_TIMEOUT;
    while (SD_Byte(0xFF) != 0xFF) {
        if (--cnt == 0) { return SD_TIMEOUT; }
    }
    return SD_OK;
}

/* 读 CSD 寄存器并解析总块数。
   CSD 是 16 字节的"卡身份卡"寄存器:V2 卡容量字段在 byte7~9,
   V1 老卡在 byte6~8 + byte9~10 的低位,公式不一样。 */
static uint32_t SD_ReadCSD(void)
{
    uint8_t csd[16];
    uint8_t token;

    SD_CS_LOW();
    SD_ClkIdle(1);
    if (SD_SendCmd(CMD9, 0, 0x01) != 0x00) { SD_CS_HIGH(); return 0; }

    /* 等数据起始 token,再收 16 字节 + 2 字节 CRC(丢弃) */
    token = 0xFF;
    while (token == 0xFF) { token = SD_Byte(0xFF); }
    if (token != SD_TOKEN_READ) { SD_CS_HIGH(); return 0; }
    for (uint8_t i = 0; i < 16; i++) { csd[i] = SD_Byte(0xFF); }
    SD_Byte(0xFF); SD_Byte(0xFF);
    SD_CS_HIGH();

    /* CSD_STRUCTURE = csd[0] 的 bit7:6 */
    if ((csd[0] >> 6) >= 2) {
        /* V2 卡(SDHC):容量块数 = (C_SIZE+1) × 1024, C_SIZE 占 csd[7:9] 共 22 位 */
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
        return (c_size + 1) << 10;
    } else {
        /* V1 老卡:块数 = (C_SIZE+1) × 2^(C_SIZE_MULT+2) */
        uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        uint32_t c_mult = ((uint32_t)(csd[9] & 0x03) << 1) | (csd[10] >> 7);
        return (c_size + 1) << (c_mult + 2);
    }
}

/* ==================== 对外接口 ==================== */

uint8_t SD_Init(void)
{
    uint8_t r1;
    uint32_t tries;

    /* 上电稳定延时:SD 模块由外部电源板独立供电,上电瞬间模块 LDO 建压、
       卡内部上电复位都未完成,立刻发 CMD0 会无应答(R1=0xFF)。
       等 100ms 再开始握手;重试路径也经过这里,总间隔 = 100ms + 任务层 1s */
    HAL_Delay(100);

    s_spi_hw_err = 0;   /* 每次尝试重新统计外设级错误 */

    /* 1. 上电时序:CS 保持高,先发 80 个时钟(SD 规范:上电后至少 74 个时钟) */
    SD_CS_HIGH();
    SD_ClkIdle(80);

    /* 2. CMD0 复位进 SPI 模式。
       CRC 填 0x95(SD 规范固定值):复位前卡还在 SD 模式,那时 CRC 校验强制开启,
       只有 CMD0/CMD8 必须带正确 CRC,之后 SPI 模式下 CRC 就不查了。
       R1=0x01 表示卡已进空闲态。 */
    SD_CS_LOW();
    /* CS 电平回读自检:推挽输出的实际电平能通过 IDR 读回。
       若读回仍为高,说明引脚被外部钳住(接线短路到 3.3V/5V)或引脚配置失效——
       直接回答"CS 到底选中卡没有",不用万用表 */
    if (HAL_GPIO_ReadPin(SD_CS_PORT, SD_CS_PIN) != GPIO_PIN_RESET)
    {
        LOG_ERROR("sd: CS pin stuck HIGH, card never selected!");
        SD_CS_HIGH();
        return SD_ERROR;
    }
    r1 = SD_SendCmd(CMD0, 0x00000000, 0x95);
    SD_CS_HIGH();
    if (r1 != 0x01)
    {
        if (s_spi_hw_err)
        {
            /* 主机侧:SPI 外设根本没发出去,查 SPI2 初始化/时钟,不是卡的问题 */
            LOG_ERROR("sd: SPI hw fail, HAL returned error (SPI2 not working)");
        }
        else
        {
            /* 卡侧:主机正常发出了命令,卡不应答,查模块供电/接线/卡 */
            LOG_ERROR("sd: CMD0 fail, R1=0x%02X", r1);
        }
        return SD_ERROR;
    }

    /* 3. CMD8 握手:参数 0x1AA = 供电电压 2.7~3.6V + 检查模式 0b1010。
       卡应答 R1=0x01 后还会回 4 字节,后 2 字节必须回显 0x01AA,
       能回显说明是 2.0 协议卡(SDHC),回不了则是 V1 老卡。 */
    SD_CS_LOW();
    r1 = SD_SendCmd(CMD8, 0x000001AA, 0x87);
    if (r1 == 0x01) {
        uint8_t resp[4];
        for (uint8_t i = 0; i < 4; i++) { resp[i] = SD_Byte(0xFF); }
        SD_CS_HIGH();
        if (resp[2] != 0x01 || resp[3] != 0xAA) { LOG_ERROR("sd: CMD8 echo fail: %02X %02X", resp[2], resp[3]); return SD_ERROR; }
        s_sd_type = 2;          /* 高容量卡 */
    } else {
        SD_CS_HIGH();
        s_sd_type = 1;          /* V1 老卡:ACMD41 不带 HCS 位 */
    }

    /* 4. CMD55+ACMD41 轮询初始化:卡内部初始化要时间,反复问直到 R1=0x00(就绪)。
       HCS 位(bit30)=1 声明支持高容量卡,让卡把容量按块上报。 */
    tries = SD_BUSY_TIMEOUT;
    do {
        SD_CS_LOW();
        SD_ClkIdle(1);                      /* 命令间必须空时钟 */
        r1 = SD_SendCmd(CMD55, 0, 0x01);
        if (r1 <= 1) {
            r1 = SD_SendCmd(ACMD41, (s_sd_type == 2) ? 0x40000000UL : 0, 0x01);
        }
        SD_CS_HIGH();
        if (r1 == 0x00) { break; }
    } while (--tries);
    if (r1 != 0x00) { LOG_ERROR("sd: ACMD41 timeout, last R1=0x%02X", r1); return SD_TIMEOUT; }

    /* 5. 握手完成,SPI 切高速(F1 库没有现成的改分频宏,直接操作 CR1 的 BR 位;
           先关 SPI 再改,避免高速时钟毛刺被卡吃错数据) */
    __HAL_SPI_DISABLE(&hspi2);
    hspi2.Instance->CR1 = (hspi2.Instance->CR1 & ~SPI_CR1_BR) | SD_HIGH_PRESCALER;
    __HAL_SPI_ENABLE(&hspi2);

    /* 6. 读 CSD 拿容量(disk_ioctl 查询用,挂载本身不依赖它) */
    s_sector_cnt = SD_ReadCSD();

    return SD_OK;
}

uint8_t SD_ReadBlock(uint8_t *buf, uint32_t sector)
{
    uint8_t token;

    SD_CS_LOW();
    SD_ClkIdle(1);
    if (SD_SendCmd(CMD17, sector, 0x01) != 0x00) { SD_CS_HIGH(); return SD_ERROR; }

    /* 等数据起始 token:0xFE 之前全是 0xFF 填充 */
    token = 0xFF;
    while (token == 0xFF) { token = SD_Byte(0xFF); }
    if (token != SD_TOKEN_READ) { SD_CS_HIGH(); return SD_TIMEOUT; }

    /* 收 512 字节数据 + 2 字节 CRC(SPI 模式不校验,读走即可) */
    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) { buf[i] = SD_Byte(0xFF); }
    SD_Byte(0xFF); SD_Byte(0xFF);

    SD_CS_HIGH();
    return SD_OK;
}

uint8_t SD_ReadMultiBlock(uint8_t *buf, uint32_t sector, uint16_t count)
{
    uint8_t token;

    SD_CS_LOW();
    SD_ClkIdle(1);
    if (SD_SendCmd(CMD18, sector, 0x01) != 0x00) { SD_CS_HIGH(); return SD_ERROR; }

    for (uint16_t block = 0; block < count; block++) {
        /* 每块等一次 token,再收 512 字节 */
        token = 0xFF;
        while (token == 0xFF) { token = SD_Byte(0xFF); }
        if (token != SD_TOKEN_READ) { SD_CS_HIGH(); return SD_TIMEOUT; }
        for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) { buf[block * SD_BLOCK_SIZE + i] = SD_Byte(0xFF); }
        SD_Byte(0xFF); SD_Byte(0xFF);   /* 每块 CRC 都要读走 */
    }

    /* CMD12 停止多块传输,再吞一个忙字节(CMD12 应答后数据线会回 0xFF) */
    SD_ClkIdle(1);
    SD_SendCmd(CMD12, 0, 0x01);
    SD_Byte(0xFF);

    SD_CS_HIGH();
    return SD_OK;
}

uint8_t SD_WriteBlock(const uint8_t *buf, uint32_t sector)
{
    uint8_t resp;

    SD_CS_LOW();
    SD_ClkIdle(1);
    if (SD_SendCmd(CMD24, sector, 0x01) != 0x00) { SD_CS_HIGH(); return SD_ERROR; }

    /* 发数据:起始 token + 512 字节 + 2 字节 CRC(SPI 模式不校验,发任意) */
    SD_Byte(SD_TOKEN_WRITE);
    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) { SD_Byte(buf[i]); }
    SD_Byte(0xFF); SD_Byte(0xFF);

    /* 数据应答:bit0=1 完成,bit1=1 接受,bit2=1 CRC 错,bit3=1 写错。
       0x05 = 接受,0x0B/0x0D = 拒收 */
    resp = SD_Byte(0xFF);
    if ((resp & 0x1F) != 0x05) { SD_CS_HIGH(); return SD_ERROR; }

    /* 等卡写完(内部擦写要时间) */
    if (SD_WaitBusy() != SD_OK) { SD_CS_HIGH(); return SD_TIMEOUT; }

    SD_CS_HIGH();
    return SD_OK;
}

uint8_t SD_WriteMultiBlock(const uint8_t *buf, uint32_t sector, uint16_t count)
{
    /* 多块写(CMD25)每块要 0xFC token 且结束要 CMD12,时序更绕;
       这里循环单块写,每块完整等忙,功能等价、出错面小。 */
    for (uint16_t block = 0; block < count; block++) {
        if (SD_WriteBlock(buf + block * SD_BLOCK_SIZE, sector + block) != SD_OK) {
            return SD_ERROR;
        }
    }
    return SD_OK;
}

uint32_t SD_GetSectorCount(void)
{
    return s_sector_cnt;
}
