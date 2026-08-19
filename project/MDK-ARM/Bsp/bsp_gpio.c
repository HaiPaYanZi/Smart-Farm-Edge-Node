#include "bsp_gpio.h"

/*
  板级 GPIO 输出封装(芯片资源层)

  只管"把某个引脚输出高/低",不知道也不关心这个引脚驱动的是什么。
  继电器、蜂鸣器等用途的映射在 Divece 层器件驱动里做。
*/

/* GPIO 初始化:PB0(水泵继电器)/PB4(蜂鸣器)推挽输出,初始低 */
void GPIO_OutInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_4;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;    /* 推挽输出 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* 开关量,低频足够 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 初始全部输出低电平 → 器件默认关(避免上电瞬间继电器误动作) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

/* 输出电平控制 */
void GPIO_OutWrite(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 读引脚当前电平(读 IDR 输入寄存器:输出脚的 IDR 同步反映引脚实际电平) */
uint8_t GPIO_OutRead(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0;
}
