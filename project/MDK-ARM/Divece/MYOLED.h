#ifndef __MYOLED_H__
#define __MYOLED_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*==================== OLED配置参数 ====================*/
#define OLED_ADDRESS 0x78		//OLED的I2C从机地址 0x78 = 0x3C << 1，0x3C是OLED的I2C地址，左移1位是因为HAL库的I2C函数需要传入8位地址并加一个读写位

/*==================== 字体大小定义 ====================*/
#define OLED_6X8	8	    //宽6像素，高8像素(已精简统一使用6*8字库,8*16已删除)

/*字符宽度（X方向步进）*/
#define OLED_CHAR_WIDTH(FontSize) 6	//统一6*8字体,字符宽度固定为6像素

/*==================== 全局变量声明 ====================*/
/*OLED显存数组，所有的显示函数，都只是对此显存数组进行读写*/
/*随后调用OLED_Update函数或OLED_UpdateArea函数*/
/*才会将显存数组的数据发送到OLED硬件，进行显示*/
extern uint8_t OLED_DisplayBuf[8][128];

/*==================== 基本函数 ====================*/
void OLED_Init(void);					//OLED初始化
void OLED_Clear(void);					//将OLED显存数组全部清零
void OLED_Update(void);					//将OLED显存数组更新到OLED屏幕
void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);	//将OLED显存数组部分更新到OLED屏幕
// X(0-127)(左右),Y(0-63)(上下)
/*==================== 文本显示（像素坐标 + 字体大小） ====================*/
void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize);		//OLED显示一个字符
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize);	//OLED显示字符串（支持ASCII码和中文混合写入）
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);			//OLED显示数字（十进制，正整数）
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);	//OLED显示有符号数字（十进制，整数）
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);		//OLED显示十六进制数字（十六进制，正整数）
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);		//OLED显示二进制数字（二进制，正整数）
void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);	//OLED显示浮点数字（十进制，小数）
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...);	//OLED使用printf函数打印格式化字符串（支持ASCII码和中文混合写入）

/*==================== 图像显示 ====================*/
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);	//OLED显示图像

/*==================== 区域操作 ====================*/
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);	//将OLED显存数组部分清零
void OLED_Reverse(void);			//将OLED显存数组全部取反
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);	//将OLED显存数组部分取反

#endif
