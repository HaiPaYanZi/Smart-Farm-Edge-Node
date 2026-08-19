#ifndef __BSP_SOFT_IIC_H__
#define __BSP_SOFT_IIC_H__

#include <stm32f1xx_hal.h>

#define I2C_SDA  GPIO_PIN_7
#define I2C_SCL  GPIO_PIN_6
#define IIC_GPIO_PORT  GPIOB

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

/* 高级封装：寄存器读写时序 */
uint8_t MyI2C_WriteReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
uint8_t MyI2C_WriteMultiReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Len);
uint8_t MyI2C_ReadReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData);
uint8_t MyI2C_ReadMultiReg(uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Len);
uint8_t MyI2C_CheckDevice(uint8_t DevAddr);

#endif
/*
  1. 为什么 SDA 要开漏?上拉电阻值怎么选?
  选SDA开漏是因为在总线不被控制时可以默认高电平，当低电平/高低电平切换 
  就说明有从机/主机在占用总线，而且开漏模式还有线与功能，只要有人拉低就代表有人在使用，
  实现了一个简易的仲裁功能，，电阻一般都是使用4.7K，功耗适中，速度也适中满足大部分的需求
  也可以加大或减小(1~10K)，比如越小速度越快、功耗越大，就适合高速度设备
  2. 起始/停止条件分别是什么电平组合?
  起始：SCL高电平 SDA高到低  停止:SCL高 SDA低到高
  3. 数据是在 SCL 高电平期间采样还是低电平?
  高电平采样
  4. ACK 是拉高还是拉低?谁发?
  接收方发
  发送方发完一字节,接收方在第 9 个时钟拉低 SDA ="我收到了,继续"。
  也就是"谁收数据,谁回 ACK"。从机回主机 ACK 表示"我在线、地址对";
  主机回从机 ACK表示"继续发,我还要"
  5. 读数据时最后一个字节为什么 NACK?
  主机告诉从机停止发送数据
  6. 7 位地址怎么转成总线字节?
  ADDR<<1 | R/W
  7. 软件 I2C 和硬件 I2C 区别?
  软件:IO 翻转,代码延时,灵活、不占外设,但占CPU、速度慢;方便维护和移植
  硬件:外设自动产生时序,不占 CPU,但是STM32有锁死外设机制，错误标志(BERR/OVR/AF)没清导致 BUSY
  位卡住,清标志或重新初始化外设就能恢复。
  另外补一个更显深度的点:硬件 I2C 时序由外设保证,不受中断影响;
  软件I2C 会被中断打断(比如 SCL 拉高一半来了中断,高位时间被拉长)——这就是为什么软件 I2C
  在时序要求严格的场景不够稳。

  8. 你的软件 I2C 速度多少?→ 
  你代码里每次操作延时 2~10us,约 50~100kHz,标准模式(100kHz)够用

  我问的问题
  9.你是怎么判断延时是使用2~10us的，频率你是怎么算的？
  算频率:一个 bit 周期 = SCL 一个高低循环。
  看你 MyI2C_SendByte 的每一位:
  SDA 置位 → delay(2us) → SCL 拉高 → delay(5us) → SCL 拉低 → delay(2us)
  一位 ≈ 2+5+2 = 9us,加上 GPIO 翻转本身的几 ns 开销 → f ≈ 1/9us ≈ 100kHz
  左右,正好落在标准模式(100kHz)附近。
    10.ACK与NACK的区别？
  ┌──────┬────────────────┬─────────────────┐
  │      │      ACK       │      NACK       │
  ├──────┼────────────────┼─────────────────┤
  │ 电平 │ 低(SDA 被拉低)  │ 高(SDA 释放)     │
  ├──────┼────────────────┼─────────────────┤
  │ 含义 │ 收到了,继续     │ 收不到/不想继续   │
  ├──────┼────────────────┼─────────────────┤
  │ 谁发 │ 每次的接收方    │ 接收方           │
  └──────┴────────────────┴─────────────────┘

  具体到你的代码场景:
  - MyI2C_ReceiveAck() 返回 0 = 从机拉低了 = ACK = 设备在线、乐意继续(你的 MyI2C_CheckDevice
  就靠这个判断设备在不在)
  - 读数据时主机 MyI2C_SendAck(0) = "继续发";最后一个字节 MyI2C_SendAck(1) = "不要了"
    11.IIC是一个怎样的协议(基础)？
    I2C 是两线制同步串行总线:一根时钟 SCL、一根数据SDA,半双工。
    主从架构,主机产生时钟、发起通信,每个从机有唯一 7 位地址。
    一帧的标准结构:起始 →从机地址+R/W 位 → ACK → 数据+ACK(可多组)→ 停止。
    标准模式 100kHz、快速模式400kHz。
    典型应用:EEPROM、温度/光照传感器、OLED 屏。
    12.多字节发送的过程是怎样的？
    MyI2C_WriteMultiReg();
    Start
    → 发从机地址(写),等 ACK        ← 设备选择
    → 发起始寄存器地址,等 ACK        ← 数据写哪
    → 发 data[0],等 ACK              ← 从机内部寄存器指针自动 +1
    → 发 data[1],等 ACK
    → ... 循环 Len 次
    Stop
    两个关键点:
  - 每发一字节都要等 ACK,你的代码每个 ACK 都检查、失败立即 Stop 返回1——这是好习惯,面试官看到会说"错误处理意识不错"
  - 连续发多个字节时,从机内部寄存器地址自动递增(器件特性),
  所以 OLED 一次写完一屏显示数据、AHT20一次写完整校准命令,都是靠这个连续模式
    13.你有接触到双start吗，为什么？你是怎么知道这样使用的？
    有，因为在发一个命令时有先发从机地址后等待ACK再发寄存器地址，中间不能断开/被其他从机抢占
    所以不能是start-stop-start，可以直接start-start
    "数据手册的读时序要求先写寄存器地址、再用重复起始切换读模式,我按手册时序实现"
    14.封装是你写的吗，思路是怎么样的？
    我写的
    1. 拆协议:I2C 说白了就是"时钟+数据的电平翻转组合"→ 拆成最小原语(起始/停止/发字节/收字节)
    2. 写原语:用 GPIO 翻转 + 延时实现,不依赖硬件 I2C 外设 → 换芯片只要改引脚宏,移植性强
    3. 组合成寄存器读写:绝大多数 I2C 器件(传感器/EEPROM/OLED)都是"设备地址+寄存器地址+数据"的固定模式,封装
    ReadReg/WriteMultiReg 后,上层驱动直接调用
    4. 加错误返回:每个 ACK 都检查,返回 0 成功/1 无应答 → 上层能判断"设备在不在、该不该重试"
    15.IIC的元操作是几个?
    4个
    ① Start(起始)    ② Stop(停止)
    ③ SendByte(发字节)  ④ ReceiveByte(收字节)
    起始/停止 + 8 位数据 + 第 9 位应答
    16.为什么不用硬件 I2C？
    学习阶段用软件版才能把协议吃透;
    这项目传感器速度要求低(几十kHz),软件版够用;
    软件版移植性更好。
    "什么场景该用硬件 I2C"——高速(400kHz+)、大数据量(一次读几 KB)、不想占CPU 的时候。
*/

