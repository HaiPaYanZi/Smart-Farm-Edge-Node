#include "dsp_soft_IIC.h"
#include "Delay.h"


/**
 * @brief  I2C写SCL线电平
 * @param  BitValue : 要写入的电平值，0=低电平，1=高电平
 * @retval 无
 */
void MyI2C_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(IIC_GPIO_PORT, I2C_SCL, (GPIO_PinState)BitValue);
    delay_us(10);
}

/**
 * @brief  I2C写SDA线电平
 * @param  BitValue : 要写入的电平值，0=低电平，1=高电平
 * @retval 无
 */
void MyI2C_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(IIC_GPIO_PORT, I2C_SDA, (GPIO_PinState)BitValue);
    delay_us(10);
}

/**
 * @brief  I2C读SDA线电平
 * @param  无
 * @retval 返回SDA线的电平状态（GPIO_PIN_SET 或 GPIO_PIN_RESET）
 */
uint8_t MyI2C_R_SDA(void)
{
    return HAL_GPIO_ReadPin(IIC_GPIO_PORT, I2C_SDA);
}

/**
 * @brief  I2C初始化（配置GPIO为开漏输出模式）
 * @param  无
 * @retval 无
 */
void MyI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOB时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置SCL和SDA引脚为开漏输出 */
    GPIO_InitStruct.Pin   = I2C_SCL | I2C_SDA;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;    /* I2C总线需要开漏输出 */
    GPIO_InitStruct.Pull  = GPIO_PULLUP;            /* 内部上拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   /* 高速模式 */
    HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct);

    /* 初始状态：SCL和SDA均置高（总线空闲状态） */
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

/**
 * @brief  I2C起始信号：SCL高电平时，SDA由高变低
 * @param  无
 * @retval 无
 */
void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);
    delay_us(5);
    MyI2C_W_SCL(1);
    delay_us(5);
    MyI2C_W_SDA(0);
    delay_us(5);
    MyI2C_W_SCL(0);
}

/**
 * @brief  I2C停止信号：SCL高电平时，SDA由低变高
 * @param  无
 * @retval 无
 */
void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);
    delay_us(5);
    MyI2C_W_SCL(1);
    delay_us(5);
    MyI2C_W_SDA(1);
    delay_us(5);
}

/**
 * @brief  I2C发送一个字节
 * @param  Byte : 要发送的字节数据（MSB优先）
 * @retval 无
 */
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SDA(Byte & (0x80 >> i));  /* 先送出最高位(MSB) */
        delay_us(2);
        MyI2C_W_SCL(1);                    /* SCL上升沿，从机读取数据 */
        delay_us(5);
        MyI2C_W_SCL(0);
        delay_us(2);
    }
}

/**
 * @brief  I2C接收一个字节
 * @param  无
 * @retval 接收到的字节数据
 */
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t Byte = 0x00;

    MyI2C_W_SDA(1);  /* 释放SDA线，交由从机控制 */

    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1);                    /* SCL上升沿，读取从机数据 */
        delay_us(5);
        if (MyI2C_R_SDA() == GPIO_PIN_SET)
        {
            Byte |= (0x80 >> i);           /* 高位在前(MSB) */
        }
        MyI2C_W_SCL(0);
        delay_us(2);
    }
    return Byte;
}

/**
 * @brief  I2C发送应答位
 * @param  AckBit : 0=ACK(应答), 1=NACK(非应答)
 * @retval 无
 */
void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    delay_us(2);
    MyI2C_W_SCL(1);                        /* SCL上升沿，从机读取应答 */
    delay_us(5);
    MyI2C_W_SCL(0);
    delay_us(2);
}

/**
 * @brief  I2C接收应答位
 * @param  无
 * @retval 收到的应答位，0=ACK, 1=NACK
 */
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;

    MyI2C_W_SDA(1);  /* 释放SDA线 */
    delay_us(2);
    MyI2C_W_SCL(1);                        /* SCL上升沿，读取从机应答 */
    delay_us(5);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);
    delay_us(2);

    return AckBit;
}

/*=============================================================================
 *                      高级封装：寄存器读写时序
 *============================================================================*/

/**
 * @brief  向设备指定寄存器写入一个字节
 * @param  DevAddr : 设备地址（7位地址左移1位，第0位R/W已被清零，即写地址）
 * @param  RegAddr : 寄存器地址
 * @param  Data    : 要写入的数据
 * @retval 0=操作成功, 1=从机无应答
 */
uint8_t MyI2C_WriteReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(DevAddr);            /* 发送设备地址 + 写标志 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(RegAddr);            /* 发送寄存器地址 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(Data);               /* 发送数据 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_Stop();
    return 0;
}

/**
 * @brief  向设备指定寄存器写入多个字节（连续写入）
 * @param  DevAddr : 设备地址（写地址）
 * @param  RegAddr : 起始寄存器地址
 * @param  pData   : 要写入的数据缓冲区指针
 * @param  Len     : 写入字节数
 * @retval 0=操作成功, 1=从机无应答
 */
uint8_t MyI2C_WriteMultiReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Len)
{
    uint16_t i;

    MyI2C_Start();
    MyI2C_SendByte(DevAddr);            /* 发送设备地址 + 写标志 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(RegAddr);            /* 发送起始寄存器地址 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }
    for (i = 0; i < Len; i++)
    {
        MyI2C_SendByte(pData[i]);       /* 连续发送数据 */
        if (MyI2C_ReceiveAck() != 0)
        {
            MyI2C_Stop();
            return 1;
        }
    }
    MyI2C_Stop();
    return 0;
}

/**
 * @brief  从设备指定寄存器读取一个字节
 * @param  DevAddr : 设备地址（写地址，第0位为0）
 * @param  RegAddr : 寄存器地址
 * @param  pData   : 存放读取数据的指针
 * @retval 0=操作成功, 1=从机无应答
 */
uint8_t MyI2C_ReadReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData)
{
    MyI2C_Start();
    MyI2C_SendByte(DevAddr);            /* 发送设备地址 + 写标志 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(RegAddr);            /* 发送寄存器地址 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }

    /* 重复起始信号，切换为读模式 */
    MyI2C_Start();
    MyI2C_SendByte(DevAddr | 0x01);     /* 发送设备地址 + 读标志 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }

    *pData = MyI2C_ReceiveByte();       /* 接收一个字节 */
    MyI2C_SendAck(1);                   /* 发送NACK，通知从机停止发送 */
    MyI2C_Stop();
    return 0;
}

/**
 * @brief  从设备指定寄存器读取多个字节（连续读取）
 * @param  DevAddr : 设备地址（写地址，第0位为0）
 * @param  RegAddr : 起始寄存器地址
 * @param  pData   : 存放读取数据的缓冲区指针
 * @param  Len     : 要读取的字节数
 * @retval 0=操作成功, 1=从机无应答
 */
uint8_t MyI2C_ReadMultiReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Len)
{
    uint16_t i;

    MyI2C_Start();
    MyI2C_SendByte(DevAddr);            /* 发送设备地址 + 写标志 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }
    MyI2C_SendByte(RegAddr);            /* 发送起始寄存器地址 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }

    /* 重复起始信号，切换为读模式 */
    MyI2C_Start();
    MyI2C_SendByte(DevAddr | 0x01);     /* 发送设备地址 + 读标志 */
    if (MyI2C_ReceiveAck() != 0)
    {
        MyI2C_Stop();
        return 1;
    }

    for (i = 0; i < Len; i++)
    {
        pData[i] = MyI2C_ReceiveByte(); /* 读取一个字节 */

        if (i < Len - 1)
        {
            MyI2C_SendAck(0);           /* 非最后字节 → 发送ACK，让从机继续发送 */
        }
        else
        {
            MyI2C_SendAck(1);           /* 最后字节 → 发送NACK，通知从机停止 */
        }
    }

    MyI2C_Stop();
    return 0;
}

/**
 * @brief  检测I2C设备是否在线
 * @param  DevAddr : 设备地址（写地址）
 * @retval 0=设备应答(在线), 1=无应答(不在线)
 */
uint8_t MyI2C_CheckDevice(uint8_t DevAddr)
{
    uint8_t ack;

    MyI2C_Start();
    MyI2C_SendByte(DevAddr);            /* 发送设备地址 + 写标志 */
    ack = MyI2C_ReceiveAck();           /* 检测是否有应答 */
    MyI2C_Stop();

    return ack;                         /* 0=有应答, 1=无应答 */
}
