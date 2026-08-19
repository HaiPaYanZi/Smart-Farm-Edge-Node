#include "Delay.h"
/**
 * @brief  微秒级延时函数（软件循环实现，适用于72MHz STM32F1）
 * @param  us : 延时的微秒数
 * @retval 无
 */
#include "core_cm3.h" //包含 DWT 定义
//为什么使用DWT？
//因为DWT是Cortex-M内核提供的一个调试和跟踪单元，
//它包含了一个高精度的计数器（CYCCNT），可以用来实现精确的延时。
//相比于软件循环延时，使用DWT计数器可以获得更高的精度和稳定性，尤其是在高频率下。

//之前使用__NOP()循环延时，是编译器级别的，在不同的优化级别下循环的延时时间又有误差
//虽然__NOP()可以做长时间的阻塞延时，但是在freertos中是最不愿意看到的，而且也没有什么任务需要长时延时
//所以使用DWT计数器来实现微秒级延时是一个更好的选择。
void delay_us(uint32_t us)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // 使能 DWT //就是开启这个内核硬件设备
    DWT->CYCCNT = 0;                                   // 清零计数器 
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;              // 启动计数
    uint32_t target = us * (SystemCoreClock / 1000000); // 72MHz:1μs = 72 周期 //SystemCoreClock是系统时钟频率，72MHz是72MHz时钟频率
    while (DWT->CYCCNT < target);//利用循环等待，到达目标值
}

