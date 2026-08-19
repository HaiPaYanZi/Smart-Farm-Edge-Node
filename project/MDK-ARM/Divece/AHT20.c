#include "AHT20.h"
#include "dsp_soft_IIC.h"
#include "cmsis_os.h"


/*
  1. 上电后先等 40ms(传感器需要时间进入空闲态)
  2. 读 1 字节状态字:用"起始 → 设备地址+读 → 收 1 字节(最后记得回 NACK)→停止"的时序
  3. 检查状态字的 Bit[3] 校准位:
    - 为 1 = 已校准,初始化完成,直接返回
    - 为 0 = 未校准 → 发送初始化命令 0xBE,参数 0x08 0x00
    (写时序:起始 → 地址+写 → 命令字节 → 参数 →停止),然后等 10ms

  注意:写命令时每发一个字节后都要等从机 ACK——你的原语里有现成的 MyI2C_ReceiveAck(),返回值 0 表示收到 ACK。
*/
/*
  手册 §5.5 + 表11 时序图:
    S → 地址+写(0x70) → 0xBA → ACK → P
  软复位后传感器重新初始化,恢复默认状态,需等待 ≤20ms。
  无参数,仅发一个命令字节。
*/
void AHT20_Reset(void)
{
    MyI2C_Start();
    MyI2C_SendByte(AHT20_ADDRESS & 0xFE);       /* 0x70 = 地址 + 写位 */
    MyI2C_ReceiveAck();                         /* ACK 检查 */
    MyI2C_SendByte(AHT20_CMD_RESET);            /* 命令:0xBA */
    MyI2C_ReceiveAck();                         /* ACK 检查 */
    MyI2C_Stop();
    osDelay(20);                                /* 手册:软复位所需时间 ≤20ms */
}

uint8_t AHT20_Init(void)
  {
      uint8_t status;     /* 状态字 */
      uint8_t AckFlag;    /* 应答标志 */
      osDelay(40);/* 上电后传感器最多 20ms 进入空闲态,等 40ms 更保险*/
      /*读 1 字节状态字,检查校准位
      时序:起始 → 地址+读 → 收字节 → 回 NACK → 停止(数据必须在 Start 之后、Stop 之前读!)*/
      MyI2C_Start();
      MyI2C_SendByte(AHT20_ADDRESS | 0x01);           /* 0x71 = 地址 + 读位 */
      AckFlag = MyI2C_ReceiveAck();                   /* 检查从机是否在线(第9个时钟) */
      if (AckFlag != 0) { /* 设备不应答,直接报错 */
        MyI2C_Stop(); 
        return 1; 
      }   

      status = MyI2C_ReceiveByte();                   /* 接收状态字 */
      MyI2C_SendAck(1);                               /* 读最后一个字节必须回 NACK */
      MyI2C_Stop();

      if (status & 0x08){ return 0;}/* 已校准,初始化完成 退出*/
/*------------------------------------校   准   过  程------------------------------------*/                         
      /*未校准:发初始化命令 0xBE + 参数 0x08 0x00(手册 5.4 节第1步)
         时序:起始 → 地址+写 → 命令 → 参数1 → 参数2 → 停止 */
      MyI2C_Start();
      MyI2C_SendByte(AHT20_ADDRESS & 0xFE);           /* 0x70 = 地址 + 写位 */

      AckFlag = MyI2C_ReceiveAck();
      if (AckFlag != 0) { MyI2C_Stop(); return 1; }

      MyI2C_SendByte(AHT20_CMD_INIT);                 /* 命令:0xBE */

      AckFlag = MyI2C_ReceiveAck();
      if (AckFlag != 0) { MyI2C_Stop(); return 1; }

      MyI2C_SendByte(AHT20_INIT_PARAM1);              /* 参数1:0x08 */

      AckFlag = MyI2C_ReceiveAck();
      if (AckFlag != 0) { MyI2C_Stop(); return 1; }

      MyI2C_SendByte(AHT20_INIT_PARAM2);              /* 参数2:0x00 */

      AckFlag = MyI2C_ReceiveAck();
      if (AckFlag != 0) { MyI2C_Stop(); return 1; }
      
      MyI2C_Stop();
      osDelay(10);                                    /* 初始化后等 10ms(手册) */
      return 0;
  }
/*
第 1 步:发触发测量命令(写时序): 触发命令发了 0xAC + 0x33 0x00,每字节查一次 ACK
第 2 步:等 80ms
第 3 步:连续读 6 字节(读时序,含忙检测):连读 6 字节,前 5 个回 SendAck(0),最后一个回 SendAck(1)
第 4 步:数据拼接(最容易错):拼接用了 uint32_t 变量(不能用 uint8_t),字节 3 的高 4 位湿度、低 4 位温度,互不干扰
第 5 步:换算(浮点运算):换算用了浮点字面量
第 6 步:通过指针带出结果
*/

uint8_t AHT20_Read(float *Temperature, float *Humidity)
{
  uint8_t status;     /* 状态字字节 */
  uint8_t DataBuf[5]; /* 读到的 5 字节数据 */
  //uint8_t CRC;        /* CRC 校验值 */
  uint8_t AckFlag;    /* 应答标志 */

  //起始 → 地址+写(0x70) → ACK检查 → 命令 0xAC → ACK检查 → 参数 0x33 → ACK检查 → 参数 0x00 → ACK检查 → 停止
  MyI2C_Start();
  //呼叫设备
  MyI2C_SendByte(AHT20_ADDRESS & 0xFE);           /* 0x70 = 地址 + 写位 */
  AckFlag = MyI2C_ReceiveAck();
  if (AckFlag != 0) { MyI2C_Stop(); return 1; }
  //发命令给设备
  MyI2C_SendByte(AHT20_CMD_TRIG);
  AckFlag = MyI2C_ReceiveAck();
  if (AckFlag != 0) { MyI2C_Stop(); return 1; }
  MyI2C_SendByte(AHT20_TRIG_PARAM1);
  AckFlag = MyI2C_ReceiveAck();
  if (AckFlag != 0) { MyI2C_Stop(); return 1; }
  MyI2C_SendByte(AHT20_TRIG_PARAM2);
  AckFlag = MyI2C_ReceiveAck();
  if (AckFlag != 0) { MyI2C_Stop(); return 1; }

  MyI2C_Stop();
  osDelay(80);   //等待80ms，让AHT20去采集数据
  /*-------------------------读  取 6字节+1字节CRC  数  据---------------------------*/
  /*
  起始 → 地址+读(0x71) → ACK检查
      → [读第0字节 → SendAck(0)] → [读第1字节 → SendAck(0)]
      → [读第2字节 → SendAck(0)] → [读第3字节 → SendAck(0)]
      → [读第4字节 → SendAck(0)] → [读第5字节 → SendAck(0)] 
      → [读第CRC字节 → SendAck(1)] 
      → 停止
  */
  MyI2C_Start();
  MyI2C_SendByte(AHT20_ADDRESS | 0x01);           /* 0x71 = 地址 + 读位 */
  AckFlag = MyI2C_ReceiveAck();
  if (AckFlag != 0) {MyI2C_Stop(); return 1; }
  
  //读取状态字
  status = MyI2C_ReceiveByte();
  if (status & 0x80)/* Bit7=1,传感器还在忙 */
  {
      MyI2C_SendAck(1);/* 先回 NACK,结束本次传输——在 ReceiveByte 之后、SendAck 之前判断,只发一次应答 */
      MyI2C_Stop();
      return 2;/* 返回忙错误码,上层可重试 */
  }
  MyI2C_SendAck(0);/* 不忙,继续,后面还要读数据 */

    for (int i = 0; i < 5; i++)
    {
        DataBuf[i] = MyI2C_ReceiveByte();
        if (i < 4){MyI2C_SendAck(0);}/* 前 4 个数据字节回 ACK */
        else {MyI2C_SendAck(1);}/* 第 5 个数据字节是最后一字节,回 NACK */
    }
  MyI2C_Stop();

  /*------------------------------数  据  拼  接------------------------------*/
  /* 湿度 20 位:字节1[19:12] | 字节2[11:4] | 字节3高4位[3:0] */
    uint32_t Humidityvalue = ((uint32_t)DataBuf[0] << 12)
                           | ((uint32_t)DataBuf[1] << 4)
                           | ((uint32_t)(DataBuf[2] >> 4));   /* 右移4位取高4位 */
    /* 温度 20 位:字节3低4位[19:16] | 字节4[15:8] | 字节5[7:0] */
    uint32_t Temperaturevalue = ((uint32_t)(DataBuf[2] & 0x0F) << 16)  /* ← 0x0F 掩码! */
                              | ((uint32_t)DataBuf[3] << 8)
                              | ((uint32_t)DataBuf[4]);
  //换算成float
  //湿度换算为0-100%
  *Humidity    = (float)Humidityvalue    / 1048576.0f * 100.0f;    /* %RH */
  //温度换算为0-125℃
 *Temperature = (float)Temperaturevalue / 1048576.0f * 200.0f - 50.0f; /* ℃ */

  /*-----------------------------CRC  校  验--------------------------*/
  //根据需求选择是否进行CRC校验，这里暂时不进行CRC校验
  // CRC[7:0]=1 + x4 + x5 + x8

  return 0;
}
  
