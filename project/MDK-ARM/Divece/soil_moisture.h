#ifndef __SOIL_MOISTURE_H__
#define __SOIL_MOISTURE_H__

#include <stdint.h>

#define SOIL_FILTER_WINDOW      8       /* 环形缓冲窗口大小 */

#define SOIL_ADC_DRY            4000    /* 标定:完全干燥时的 ADC 值 */
#define SOIL_ADC_WET            1500    /* 标定:完全湿润时的 ADC 值 */

void    SoilMoisture_Init(void);              /* 创建环形缓冲(清零) */
uint8_t SoilMoisture_Read(float *Moisture);   /* 采一次填入缓冲,返回窗口平均(%) */

#endif
