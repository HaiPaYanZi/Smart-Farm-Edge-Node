#ifndef __AHT20_H__
#define __AHT20_H__

#include <stdint.h>

#define AHT20_ADDRESS 0x70 //AHT20的IIC地址，7位地址0x38，左移1位后为0x70

//命令数据帧结构：发送初始化命令 0xBE,参数 0x08 0x00

#define AHT20_CMD_INIT 0xBE //AHT20初始化命令
#define AHT20_INIT_PARAM1 0x08 //AHT20初始化参数
#define AHT20_INIT_PARAM2 0x00 //AHT20初始化参数2

#define AHT20_CMD_TRIG         0xAC    /* 触发测量命令    */
#define AHT20_TRIG_PARAM1      0x33    /* 触发测量参数第 1 字节 */
#define AHT20_TRIG_PARAM2      0x00    /* 触发测量参数第 2 字节 */

#define AHT20_CMD_RESET        0xBA    /* 软复位命令(手册表9,无参数) */

uint8_t AHT20_Init(void);//初始化AHT20传感器
uint8_t AHT20_Read(float *Temperature, float *Humidity);//读AHT20的温湿度信息
void AHT20_Reset(void);//复位AHT20传感器

#endif
