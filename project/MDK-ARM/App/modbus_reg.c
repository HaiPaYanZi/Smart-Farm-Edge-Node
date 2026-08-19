#include "modbus_reg.h"
#include "MySensor.h"
#include "rule_engine.h"
#include "fan.h"
#include "pump.h"
#include "light.h"
#include "stall_detect.h"
#include "param.h"
#include "ota.h"

/*
  Modbus 寄存器回调(应用层,《06-Modbus寄存器表》)

  ★核心点:回调的 address 是"1 基区偏移",不是完整寄存器号!
    FreeModbus 把线上偏移(0 基)加 1 后传给回调(mbfuncholding.c 的 usRegAddress++)
    对应关系:40001→1、40008→8、30001→1、00001→1
    想加寄存器:改下面的 size 宏 + switch 加 case,三步搞定

  寄存器布局:
    30001 温度 0.1°C   30005 风机转速 RPM
    30002 湿度 0.1%    30006 风机占空比 %
    30003 光照 lx      30007 状态字(bit5=堵转)
    30004 土壤 0.1%
    40001 PID Kp ×100  40005 控制模式(0自动/1手动/2待机)
    40002 PID Ki ×100  40006 手动风机占空比 ×10
    40003 PID Kd ×100  40007 风机滞回温差 ×10
    40004 目标温度 ×10  40008 补光阈值 lx(2000)   40010 灌溉阈值 0.1%(300=30.0%)
                       40009 补光回差 lx(1000)   40011 灌溉回差 0.1%(150=15.0%)
                       40012 从站地址(修改后重启生效)
                       40020 OTA 升级触发(写 0x5A5A 进升级模式,文档《07》)
    00001 水泵手动启停         00002 补光手动启停

  数据格式:Modbus 大端字节序(高字节在前);40001~40012 全部走参数区(见 param.c):
    写回调只存参数+置 dirty,控制任务 100ms 后 Param_Sync 加载生效(文档《03》§5.1),
    防抖 2s 后由 nvs.c 落盘 Flash(掉电不丢)。
  范围检查:地址超范围返回 MB_ENOREG → 协议栈自动回异常码 02。
*/

/* 风机目标占空比(freertos.c 定义):手动模式写 40006 时直接生效 */
extern volatile uint8_t g_fan_duty;

#define INPUT_START     1       /* 输入寄存器 30001 对应偏移 1 */
#define INPUT_SIZE      7       /* 数量(30001~30007) */

#define HOLDING_START   1       /* 保持寄存器 40001 对应偏移 1 */
#define HOLDING_SIZE    20      /* 数量(40001~40020) */

#define COIL_START      1       /* 线圈 00001 对应偏移 1 */
#define COIL_SIZE       2       /* 数量(00001~00002) */

/**
  * 函    数：eMBRegInputCB
  * 功    能：读输入寄存器回调(30001~30007 传感器实时数据,只读)
  * 参    数：buffer 数据缓冲(大端打包,每寄存器 2 字节)
  *           address 寄存器偏移(1 基:30001→1)
  *           reg_count 读几个寄存器
  * 返 回 值：MB_ENOERR 成功 / MB_ENOREG 地址超范围(自动回异常码 02)
  */
eMBErrorCode eMBRegInputCB(UCHAR *buffer, USHORT address, USHORT reg_count)
{
    eMBErrorCode status = MB_ENOREG;   /* 默认:地址非法 */
    USHORT i;                          /* 循环计数:第几个寄存器 */

    /* 地址范围检查:请求的 [起始, 起始+数量) 必须在映射范围内 */
    if ((address >= INPUT_START) && (address + reg_count <= INPUT_START + INPUT_SIZE))
    {
        for (i = 0; i < reg_count; i++)
        {
            USHORT value = 0;                            /* 当前寄存器的数值 */
            switch (address + i)                         /* 按偏移查表 */
            {
                case 1: value = (USHORT)(MySensor_GetValue(SENSOR_TEMP)  * 10.0f); break;  /* 30001,0.1°C */
                case 2: value = (USHORT)(MySensor_GetValue(SENSOR_HUMI)  * 10.0f); break;  /* 30002,0.1% */
                case 3: value = (USHORT)(MySensor_GetValue(SENSOR_LIGHT));        break;  /* 30003,lx */
                case 4: value = (USHORT)(MySensor_GetValue(SENSOR_SOIL)  * 10.0f); break;  /* 30004,0.1% */
                case 5: value = Fan_GetRpm();                                           break;  /* 30005,RPM */
                case 6: value = Fan_GetDuty();                                          break;  /* 30006,% */
                case 7:                                                                  /* 30007,状态字 */
                    value = 0;
                    if (StallDetect_IsStalled()) value |= (1 << 5);                      /* bit5 堵转 */
                    break;
            }
            /* 大端字节序:高字节放前 */
            buffer[i * 2]     = (UCHAR)(value >> 8);
            buffer[i * 2 + 1] = (UCHAR)(value & 0xFF);
        }
        status = MB_ENOERR;                              /* 全部填完,成功 */
    }
    return status;
}

/**
  * 函    数：eMBRegHoldingCB
  * 功    能：保持寄存器回调(40001~40011 配置参数,可读可写)
  * 参    数：buffer 数据缓冲
  *           address 寄存器偏移(1 基:40001→1)
  *           reg_count 寄存器数量
  *           mode MB_REG_READ=读 / MB_REG_WRITE=写
  * 返 回 值：MB_ENOERR 成功 / MB_ENOREG 地址超范围
  * 说    明：写 40008~40011 会同步更新规则引擎阈值(远程改参数立即生效)
  */
eMBErrorCode eMBRegHoldingCB(UCHAR *buffer, USHORT address, USHORT reg_count, eMBRegisterMode mode)
{
    eMBErrorCode status = MB_ENOREG;   /* 默认:地址非法 */
    USHORT i;                          /* 循环计数:第几个寄存器 */

    if ((address >= HOLDING_START) && (address + reg_count <= HOLDING_START + HOLDING_SIZE))
    {
        if (mode == MB_REG_WRITE)                      /* 写:从缓冲取值,更新配置 */
        {
            for (i = 0; i < reg_count; i++)
            {
                /* 大端拆包:高字节在前 */
                USHORT value = (USHORT)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
                switch (address + i)
                {
                    case 1:  /* 40001 PID Kp ×100:参数区 + dirty,控制任务下周期生效 */
                        Param_SetKp((float)value / 100.0f);
                        break;
                    case 2:  /* 40002 PID Ki ×100 */
                        Param_SetKi((float)value / 100.0f);
                        break;
                    case 3:  /* 40003 PID Kd ×100 */
                        Param_SetKd((float)value / 100.0f);
                        break;
                    case 4:  /* 40004 目标温度 ×10(有符号:高字节符号位) */
                        Param_SetTarget((float)(int16_t)value / 10.0f);
                        break;
                    case 5:  /* 40005 控制模式:0自动 1手动 2待机(越界由参数区按待机处理) */
                        Param_SetMode((uint8_t)value);
                        break;
                    case 6:  /* 40006 手动风机占空比 ×10(手动模式下立即生效) */
                        Param_SetManualDuty((float)value / 10.0f);
                        if (Param_GetMode() == MODE_MANUAL)
                        {
                            g_fan_duty = (uint8_t)Param_GetManualDuty();  /* 手动模式:直接写目标占空比 */
                        }
                        break;
                    case 7:  /* 40007 风机滞回温差 ×10:温度 < 目标-温差 → 停转 */
                        Param_SetFanHyst((float)value / 10.0f);
                        break;
                    case 8:  /* 40008 补光阈值(lx):低于则开补光(参数区,收编自 g_rules) */
                        Param_SetLightThreshold((float)value);
                        break;
                    case 9:  /* 40009 补光回差(lx):高于 阈值+回差 则关 */
                        Param_SetLightHysteresis((float)value);
                        break;
                    case 10:  /* 40010 灌溉阈值(0.1%):低于则开泵 */
                        Param_SetIrrigationThreshold((float)value / 10.0f);   /* 寄存器的 0.1% 转成 % */
                        break;
                    case 11:  /* 40011 灌溉回差(0.1%) */
                        Param_SetIrrigationHysteresis((float)value / 10.0f);
                        break;
                    case 12:  /* 40012 从站地址(1~247):仅存参数区,重启后 eMBInit 才用 */
                        Param_SetSlaveAddr((uint8_t)value);
                        break;
                    case 20:  /* 40020 OTA 升级触发:写 0x5A5A 进升级模式(文档《07》,与《03》§8 冲突已按 07 勘误) */
                        if (value == 0x5A5A)
                        {
                            OTA_RequestUpgrade();   /* 写标志区+200ms 后软复位,应答帧先发完 */
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        else    /* MB_REG_READ:读,把配置值打包进缓冲 */
        {
            for (i = 0; i < reg_count; i++)
            {
                USHORT value = 0;                            /* 当前寄存器的数值 */
                switch (address + i)
                {
                    case 1: value = (USHORT)(Param_GetKp() * 100.0f);              break;  /* 40001 */
                    case 2: value = (USHORT)(Param_GetKi() * 100.0f);              break;  /* 40002 */
                    case 3: value = (USHORT)(Param_GetKd() * 100.0f);              break;  /* 40003 */
                    case 4: value = (USHORT)(int16_t)(Param_GetTarget() * 10.0f);  break;  /* 40004,有符号 */
                    case 5: value = (USHORT)Param_GetMode();                        break;  /* 40005 */
                    case 6: value = (USHORT)(Param_GetManualDuty() * 10.0f);       break;  /* 40006 */
                    case 7: value = (USHORT)(Param_GetFanHyst() * 10.0f);          break;  /* 40007 */
                    case 8: value = (USHORT)Param_GetLightThreshold();         break;  /* 40008 */
                    case 9: value = (USHORT)Param_GetLightHysteresis();        break;  /* 40009 */
                    case 10: value = (USHORT)(Param_GetIrrigationThreshold() * 10.0f); break;  /* 40010,%→0.1% */
                    case 11: value = (USHORT)(Param_GetIrrigationHysteresis() * 10.0f);break;  /* 40011 */
                    case 12: value = (USHORT)Param_GetSlaveAddr();                 break;  /* 40012 */
                    case 20: value = 0;                                             break;  /* 40020,OTA 触发(只写不读) */
                    default:    break;
                }
                buffer[i * 2]     = (UCHAR)(value >> 8);
                buffer[i * 2 + 1] = (UCHAR)(value & 0xFF);
            }
        }
        status = MB_ENOERR;                              /* 处理成功 */
    }
    return status;
}

/**
  * 函    数：eMBRegCoilsCB
  * 功    能：线圈回调(00001~00002 执行器手动启停,可读可写)
  * 参    数：buffer 位缓冲(1 位 1 个线圈)
  *           address 线圈偏移(1 基:00001→1)
  *           coil_count 线圈数量
  *           mode MB_REG_READ=读 / MB_REG_WRITE=写
  * 返 回 值：MB_ENOERR 成功 / MB_ENOREG 地址超范围
  */
eMBErrorCode eMBRegCoilsCB(UCHAR *buffer, USHORT address, USHORT coil_count, eMBRegisterMode mode)
{
    eMBErrorCode status = MB_ENOREG;   /* 默认:地址非法 */
    USHORT i;                          /* 循环计数:第几个线圈 */

    if ((address >= COIL_START) && (address + coil_count <= COIL_START + COIL_SIZE))
    {
        if (mode == MB_REG_WRITE)                      /* 写:直接操作执行器 */
        {
            for (i = 0; i < coil_count; i++)
            {
                /* 线圈在缓冲里按位存储:第 i 个线圈 = 第 i bit */
                UCHAR value = (buffer[i / 8] >> (i % 8)) & 0x01;
                switch (address + i)
                {
                    case 1: Pump_Control(value);   break;   /* 00001 水泵 */
                    case 2: Light_Control(value);  break;   /* 00002 补光 */
                }
            }
        }
        else    /* MB_REG_READ:读,把状态打包成位 */
        {
            for (i = 0; i < (coil_count + 7) / 8; i++)
            {
                buffer[i] = 0;                           /* 先清零 */
            }
            for (i = 0; i < coil_count; i++)
            {
                USHORT coil_value = 0;                   /* 当前线圈的状态:0 或 1 */
                switch (address + i)
                {
                    case 1: coil_value = (RuleEngine_GetActuator(ACT_PUMP)  == ACT_STATE_ON) ? 1 : 0; break;
                    case 2: coil_value = (RuleEngine_GetActuator(ACT_LIGHT) == ACT_STATE_ON) ? 1 : 0; break;
                }
                if (coil_value)
                {
                    buffer[i / 8] |= (UCHAR)(1 << (i % 8));   /* 对应位置 1 */
                }
            }
        }
        status = MB_ENOERR;                              /* 处理成功 */
    }
    return status;
}

/**
  * 函    数：eMBRegDiscreteCB
  * 功    能：离散输入回调(未启用,功能码已在 mbconfig.h 关闭)
  * 参    数：buffer 数据缓冲(未用) address 地址(未用) discrete_count 数量(未用)
  * 返 回 值：MB_ENOREG 永远(表示无此寄存器区)
  */
eMBErrorCode eMBRegDiscreteCB(UCHAR *buffer, USHORT address, USHORT discrete_count)
{
    (void)buffer;           /* 未用参数 */
    (void)address;
    (void)discrete_count;
    return MB_ENOREG;   /* 未启用 */
}
