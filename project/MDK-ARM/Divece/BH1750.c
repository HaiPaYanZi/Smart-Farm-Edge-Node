#include "BH1750.h"
#include "dsp_soft_IIC.h"
#include "cmsis_os.h"

/*
  手册 §5 + §10:
    BH1750 与 AHT20 不同——没有寄存器地址,没有参数。
    每条命令 = 起始 → 地址+写 → 1 字节操作码 → 停止
    关键:每条命令后必须跟 Stop,不能连发(手册第 10 页 3)Write Format 明确要求)。

  读取 = 起始 → 地址+读 → 高字节[15:8] → Ack0 → 低字节[7:0] → Ack1(NACK) → 停止
  数据拼接后除以 1.2 即得 lx(勒克斯)。
*/

/*
  1. 发送命令字节(写时序,每发完必须 Stop)
     手册强调:"BH1750FVI is not able to accept plural command without stop condition."
*/
static void BH1750_SendCmd(uint8_t cmd)
{
    MyI2C_Start();
    MyI2C_SendByte(BH1750_ADDRESS & 0xFE);   /* 0x46 = 地址 + 写位 */
    MyI2C_ReceiveAck();                       /* 从机应答检查 */
    MyI2C_SendByte(cmd);                      /* 命令操作码 */
    MyI2C_ReceiveAck();                       /* 从机应答检查 */
    MyI2C_Stop();                             /* 必须 Stop(BH1750 硬要求) */
}

/*
  2. 初始化:上电 → 设置连续 H 分辨率模式
     上电后等一小段时间,设为连续模式后传感器自动开始测量。
     以后每次 Read 只管读,不用重复发触发命令。
*/
void BH1750_Init(void)
{
    BH1750_SendCmd(BH1750_CMD_POWER_ON);      /* 先上电(退出断电模式) */
    osDelay(10);                                /* 上电后稳定一下 */
    BH1750_SendCmd(BH1750_CMD_CONT_H);        /* 设为连续 H 分辨率模式 */ 
    //也可以使用其他模式，看手册与命令宏定义
    osDelay(180);                               /* 手册:首次 H 分辨率测量最慢 180ms */
}

/*
  3. 读光照强度(连续模式,BH1750 自动持续更新)
     读时序:起始 → 地址+读 → 高字节 → Ack0 → 低字节 → Ack1(NACK) → 停止
     返回 0=成功,1=从机无应答。
*/
uint8_t BH1750_Read(float *Light)
{
    uint8_t high, low;          /* 高字节[15:8] / 低字节[7:0] */
    uint16_t raw;               /* 16 位原始光照值 */
    uint8_t AckFlag;            /* 应答标志 */

    /* 读 2 字节 */
    MyI2C_Start();
    MyI2C_SendByte(BH1750_ADDRESS | 0x01);   /* 0x47 = 地址 + 读位 */
    AckFlag = MyI2C_ReceiveAck();
    if (AckFlag != 0) { MyI2C_Stop(); return 1; }

    /* 高字节 */
    high = MyI2C_ReceiveByte();
    MyI2C_SendAck(0);                         /* ACK,继续要低字节 */

    /* 低字节 */
    low = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);                         /* NACK,不再要了 */
    MyI2C_Stop();

    /*-----------------------------数  据  换  算------------------------------*/
    /* 手册公式:lx = raw / 1.2 */
    raw = ((uint16_t)high << 8) | low;
    *Light = (float)raw / 1.2f;               /* 勒克斯(lx) */

    return 0;
}

/*
  4. 复位:数据寄存器清零(注意必须在 Power On 状态下发 Reset 命令)
*/
void BH1750_Reset(void)
{
    BH1750_SendCmd(BH1750_CMD_POWER_ON);      /* 确保上电(断电模式下 Reset 无效) */
    osDelay(10);
    BH1750_SendCmd(BH1750_CMD_RESET);         /* 清零数据寄存器 */
    osDelay(10);
    BH1750_SendCmd(BH1750_CMD_CONT_H);        /* 恢复连续 H 模式(Reset 后模式丢失) */
    osDelay(180);
}
