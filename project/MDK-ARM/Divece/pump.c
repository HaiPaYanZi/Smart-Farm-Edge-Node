#include "pump.h"
#include "bsp_gpio.h"

/*
  水泵器件驱动(Divece 层)

  水泵是"开关型"执行器(继电器控制,无调速),器件层只做一件事:
  把逻辑"开/关"翻译成 GPIO 电平,交给 BSP 层输出。
*/

void Pump_Control(uint8_t on)
{
    GPIO_OutWrite(GPIO_OUT_PUMP, on);
}

/* 读实际状态:直接问 BSP 层引脚电平(不维护软件状态,显示永远与实际一致) */
uint8_t Pump_IsOn(void)
{
    return GPIO_OutRead(GPIO_OUT_PUMP);
}
