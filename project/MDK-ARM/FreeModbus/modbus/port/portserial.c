/*
 * FreeModbus 移植层:portserial.c(USART3 + MAX3485 RS485)
 *
 * 硬件:USART3(PB10/PB11)9600bps 8N1(CubeMX 配置),DE=PC0 高电平=发送
 *
 * 核心点(中断模型,三段接力):
 *   RXNE 中断:收到一个字节 → 读 DR → 交给协议栈(pxMBFrameCBByteReceived)
 *   TXE 中断:发送寄存器空了 → 协议栈给下一字节;给不出 → 关 TXE 开 TC
 *   TC 中断:整个帧移出移位寄存器 → DE 拉低回接收 + 发布 EV_FRAME_SENT
 *   (DE 为什么在 TC 拉低而不是 TXE?TXE 只代表"数据寄存器空",
 *    最后一个字节还在移位寄存器里没发完,此时切方向会截断帧尾!)
 *
 * 注意:
 *   1. 绕开 HAL_UART_IRQHandler:它会吞掉 RXNE 字节,不适合 FreeModbus 逐字节模型
 *      (it.c 的 USART3_IRQHandler 已接管为 vMBPortSerialISR,CubeMX 重新生成后要恢复)
 *   2. ISR 里不调任何 FreeRTOS 阻塞 API
 */

#include "port.h"      /* 必须最先:定义 BOOL/UCHAR 等类型 */
#include "mb.h"
#include "mbport.h"
#include "usart.h"
#include "main.h"      /* RS485_DE_Pin / RS485_DE_GPIO_Port(PC0) */

/**
  * 函    数：vMBPortSerialEnable
  * 功    能：串口收发中断的使能/禁止(协议栈按收发状态机调用)
  * 参    数：rx_enable TRUE=开接收中断 / tx_enable TRUE=开发送中断并切 DE 到发送
  * 返 回 值：无
  */
void vMBPortSerialEnable(BOOL rx_enable, BOOL tx_enable)
{
    if (tx_enable)
    {
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);  /* DE 拉高:RS485 方向切到发送 */
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);                         /* 开发送空中断 */
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);
    }

    if (rx_enable)
    {
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);                        /* 开接收中断 */
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
    }
}

/**
  * 函    数：xMBPortSerialInit
  * 功    能：串口初始化(USART3 已由 CubeMX 配置,此处校验)
  * 参    数：port 串口号(忽略) baud 波特率 data_bits 数据位 parity 校验
  * 返 回 值：TRUE 初始化成功
  */
BOOL xMBPortSerialInit(UCHAR port, ULONG baud, UCHAR data_bits, eMBParity parity)
{
    /* 如需运行时动态改波特率:__HAL_UART_SET_BAUDRATE(&huart3, baud); */
    (void)port;         /* 未用参数,防止编译警告 */
    (void)baud;
    (void)data_bits;
    (void)parity;
    return TRUE;
}

/**
  * 函    数：xMBPortSerialPutByte
  * 功    能：发送一个字节(TXE 中断驱动,逐字节发送)
  * 参    数：byte 要发送的字节
  * 返 回 值：TRUE 成功
  */
BOOL xMBPortSerialPutByte(CHAR byte)
{
    huart3.Instance->DR = (uint8_t)byte;   /* 写数据寄存器(写 DR 自动清 TXE 标志) */
    return TRUE;
}

/**
  * 函    数：xMBPortSerialGetByte
  * 功    能：接收一个字节
  * 参    数：byte 出参,收到的字节
  * 返 回 值：TRUE 成功
  */
BOOL xMBPortSerialGetByte(CHAR *byte)
{
    *byte = (CHAR)(huart3.Instance->DR & 0xFF);   /* 读数据寄存器(读 DR 自动清 RXNE 标志) */
    return TRUE;
}

/**
  * 函    数：vMBPortSerialISR
  * 功    能：USART3 中断处理(由 stm32f1xx_it.c 的 USART3_IRQHandler 调用)
  * 参    数：无
  * 返 回 值：无
  * 说    明：三段接力:RXNE 收字节 → TXE 发字节 → TC 收尾切 DE
  */
void vMBPortSerialISR(void)
{
    CHAR byte;   /* 收到的字节缓存 */

    /* ① 接收:收到一个字节 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
        if (xMBPortSerialGetByte(&byte))
        {
            pxMBFrameCBByteReceived();   /* 交给协议栈(内部重启 3.5T 超时、判断帧完成) */
        }
    }

    /* ② 发送:发送寄存器空,协议栈给下一字节 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE) != RESET)
    {
        if (!pxMBFrameCBTransmitterEmpty())
        {
            /* 没有更多字节:关 TXE,开 TC,等最后一字节移出移位寄存器 */
            __HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);
            __HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
        }
    }

    /* ③ 发送完成:整个帧真正发完(移位寄存器空) */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_TC);                /* 清 TC 标志(写 0 清) */
        __HAL_UART_DISABLE_IT(&huart3, UART_IT_TC);                  /* 关 TC 中断 */

        vMBPortSerialEnable(TRUE, FALSE);                                           /* 只关发送,接收必须保持开启:等下一帧请求 */
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);        /* DE 拉低:回接收 */
        xMBPortEventPost(EV_FRAME_SENT);                                            /* 通知协议栈:帧发完(名字必须和 mbport.h 声明一致) */
    }
}
