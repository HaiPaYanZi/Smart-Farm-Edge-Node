#include "soil_moisture.h"
#include "adc.h"
/*
  土壤湿度驱动(电容式模拟量传感器)
  引脚:PA1 → ADC1_IN1,单次转换,软件触发,12 位右对齐(CubeMX 已配置)

  Init:环形缓冲清零,建好结构。上电调一次即可。
  Read:每次调用采 1 个点 ADC 填入环形缓冲(旧值自然被覆盖),
        窗口内始终是最近 WINDOW 次不同时刻的采样,取平均后换算百分比。
        每次调用自给自足,调一次拿一次有效值。
*/

static uint16_t ring_buf[SOIL_FILTER_WINDOW];       /* 环形缓冲 */
static uint8_t  ring_idx;                            /* 写入位置(0~WINDOW-1) */


/* 初始化:缓冲清零 */
void SoilMoisture_Init(void)
{
    uint8_t i;
    for (i = 0; i < SOIL_FILTER_WINDOW; i++)
    {
        ring_buf[i] = 0;
    }
    ring_idx = 0;
}


/* 读土壤湿度(0~100%),返回 0=成功 */
uint8_t SoilMoisture_Read(float *Moisture)
{
    uint32_t sum;                                   /* 累加和(uint32_t 防止溢出) */
    uint16_t avg;                                   /* 窗口平均值 */
    uint8_t i;

    /* ① 本次调用只采 1 个点,写入环形缓冲,旧值被自然覆盖 */
    /*    窗口里始终是"最近 WINDOW 次不同时刻的采样" → 真正的滑动平均 */
    /*ADC采集操作*/  //更换芯片可以改这个采集步骤
    HAL_ADC_Start(&hadc1);                      /* 启动单次转换 */
    HAL_ADC_PollForConversion(&hadc1, 10);      /* 等待转换完成(超时 10ms) */
    ring_buf[ring_idx] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);                       /* 停止,为下次 Start 准备 */

    ring_idx++;
    if (ring_idx >= SOIL_FILTER_WINDOW)
    {
        ring_idx = 0;                           /* 超出窗口,绕回开头 */
    }

    /* ② 窗口平均 */
    sum = 0;
    for (i = 0; i < SOIL_FILTER_WINDOW; i++)
    {
        sum += ring_buf[i];
    }
    avg = (uint16_t)(sum / SOIL_FILTER_WINDOW);

    /* ③ 换算百分比(反比:越湿 → 电压越低 → ADC 越小) */
    if (avg >= SOIL_ADC_DRY)                        /* 比干标定值还大,钳位到 0% */
    {
        *Moisture = 0.0f;
    }
    else if (avg <= SOIL_ADC_WET)                   /* 比湿标定值还小,钳位到 100% */
    {
        *Moisture = 100.0f;
    }
    else
    {
        *Moisture = (float)(SOIL_ADC_DRY - avg)
                  / (float)(SOIL_ADC_DRY - SOIL_ADC_WET)
                  * 100.0f;
    }

    return 0;
}
