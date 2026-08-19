#ifndef __BH1750_H__
#define __BH1750_H__

#include <stdint.h>

#define BH1750_ADDRESS          0x46    /* BH1750的IIC地址，7位地址0x23，左移1位后为0x46(ADDR引脚接地) */

/*
  命令表(手册 §5 Instruction Set):
    BH1750 只有 1 字节操作码,无参数字节。每条命令后必须跟 Stop(手册明确要求)。
    推荐连续 H 分辨率模式(0x10):分辨率 1 lx,测量时间 120ms。
*/
#define BH1750_CMD_POWER_ON     0x01    /* 上电命令 */
#define BH1750_CMD_POWER_DOWN   0x00    /* 休眠命令 */
#define BH1750_CMD_RESET        0x07    /* 数据寄存器清零(必须 Power On 状态下发) */
#define BH1750_CMD_CONT_H       0x10    /* 连续 H 分辨率模式(1 lx,推荐日常使用) */
#define BH1750_CMD_CONT_H2      0x11    /* 连续 H 分辨率模式2(0.5 lx,弱光用) */
#define BH1750_CMD_CONT_L       0x13    /* 连续 L 分辨率模式(4 lx,16ms 快速但精度低) */
#define BH1750_CMD_ONE_H        0x20    /* 单次 H 分辨率模式(测完自动休眠) */

void BH1750_Init(void);                      /* 上电 + 设置连续 H 分辨率模式 */
uint8_t BH1750_Read(float *Light);            /* 读光照强度(单位:lx),返回 0=成功 1=失败 */
void BH1750_Reset(void);                      /* 数据寄存器清零 */

#endif
