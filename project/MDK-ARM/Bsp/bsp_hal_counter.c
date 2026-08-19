#include "bsp_hal_counter.h"
#include "main.h"      /* Hall_Pin 定义(PB1) */

/*
  霍尔脉冲计数(BSP 层)

  原理:磁铁每转一圈经过霍尔模块一次 → 输出一个上升沿 → EXTI1 中断,
        ISR 里计数累加。
  注意:
    1. pulse_cnt 用 volatile——ISR 修改、任务读取,防止编译器优化进寄存器
    2. EXTI1 优先级 5(CubeMX 配置):等于 FreeRTOS 可调 API 的临界优先级,
       所以 ISR 里只做自增,绝不调任何 FreeRTOS/HAL 阻塞 API
    3. 任务侧用"两次差分"算转速(见 fan.c),不直接在 ISR 里算——省时
*/

/* 累计脉冲计数(ISR 里 ++,volatile 防优化) */
static volatile uint32_t pulse_cnt = 0;

/* 初始化:清零计数 */
void HAL_Counter_Init(void)
{
    HAL_Counter_Reset();
}

/* 读累计脉冲数 */
uint32_t HAL_Counter_GetPulses(void)
{
    return pulse_cnt;
}

/* 清零累计脉冲 */
void HAL_Counter_Reset(void)
{
    pulse_cnt = 0;
}

/* EXTI 回调:PB1 上升沿触发(由 HAL_GPIO_EXTI_IRQHandler 调用)
   中断上下文:只自增,不调 API;按引脚判断,未来按键接 EXTI 也能共存 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Hall_Pin)
    {
        pulse_cnt++;
    }
}
