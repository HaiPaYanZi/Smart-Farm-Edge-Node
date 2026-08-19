#include "buzzer.h"
#include "bsp_gpio.h"

/*
  蜂鸣器器件驱动(Divece 层)

  有源蜂鸣器:给电平就响,器件层只做"逻辑开/关 → GPIO 电平"翻译。
  报警节奏(长响/短响)由调用方控制,不在本层。
*/

void Buzzer_Control(uint8_t on)
{
    GPIO_OutWrite(GPIO_OUT_BUZZER, on);
}
