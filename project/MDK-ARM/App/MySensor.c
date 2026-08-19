#include "MySensor.h"
#include "AHT20.h"
#include "BH1750.h"
#include "soil_moisture.h"

/*
  传感器统一取值模块(应用层:规则引擎的数据源)

  设计:快照模式。
    每个控制周期(500ms)开始时 MySensor_Update() 统一读一遍所有传感器,
    读到的值存进快照;规则引擎评估时 MySensor_GetValue() 只查快照,
    不重复发起 I2C/ADC 读取(一次周期内 AHT20 只读一次)。

  失效降级:某传感器读取失败 → 快照填 SENSOR_INVALID,
    规则引擎跳过对应规则,其余规则继续运行(文档《03》§4.2)。

  RTOS 化(节点 1.5):快照升级为共享区,加互斥锁保护即可,接口不变。
*/

/* 传感器快照(读取失败填 SENSOR_INVALID) */
static float snap_temp;
static float snap_humi;
static float snap_light;
static float snap_soil;

/* 刷新快照:控制周期开始调一次 */
void MySensor_Update(void)
{
    float temp, humi, light, soil;

    /* AHT20:一次读取同时得温湿度 */
    if (AHT20_Read(&temp, &humi) == 0)//注意0是正常返回，是正确的，返回1才是错误的
    {
        snap_temp = temp;
        snap_humi = humi;
    }
    else
    {
        snap_temp = SENSOR_INVALID;     /* 温湿度同时失效 */
        snap_humi = SENSOR_INVALID;
    }

    /* BH1750 光照 */
    if (BH1750_Read(&light) == 0)
    {
        snap_light = light;
    }
    else
    {
        snap_light = SENSOR_INVALID;
    }

    /* 土壤湿度(含滑动滤波) */
    if (SoilMoisture_Read(&soil) == 0)
    {
        snap_soil = soil;
    }
    else
    {
        snap_soil = SENSOR_INVALID;
    }
}

/* 规则引擎取值回调:查快照,不碰硬件 */
float MySensor_GetValue(uint16_t sensor_id)
{
    switch (sensor_id)
    {
        case SENSOR_TEMP:   return snap_temp;
        case SENSOR_HUMI:   return snap_humi;
        case SENSOR_LIGHT:  return snap_light;
        case SENSOR_SOIL:   return snap_soil;
        default:            return SENSOR_INVALID;   /* 未知编号,按失效处理 */
    }
}
