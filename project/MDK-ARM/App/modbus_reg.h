#ifndef __MODBUS_REG_H__
#define __MODBUS_REG_H__

#include "mb.h"     /* eMBErrorCode / eMBRegisterMode */

/* Modbus 寄存器回调(应用层,对接《06-Modbus寄存器表》)
   协议栈 eMBPoll 处理帧时调用这些回调:
   - eMBRegInputCB   30001 输入寄存器:传感器实时数据(只读)
   - eMBRegHoldingCB 40001 保持寄存器:配置参数(读写,写时同步规则引擎阈值)
   - eMBRegCoilsCB   00001 线圈:执行器手动启停(读写)
   - eMBRegDiscreteCB 10001 离散输入:未启用 */

eMBErrorCode eMBRegInputCB(UCHAR *buffer, USHORT address, USHORT reg_count);
eMBErrorCode eMBRegHoldingCB(UCHAR *buffer, USHORT address, USHORT reg_count, eMBRegisterMode mode);
eMBErrorCode eMBRegCoilsCB(UCHAR *buffer, USHORT address, USHORT coil_count, eMBRegisterMode mode);
eMBErrorCode eMBRegDiscreteCB(UCHAR *buffer, USHORT address, USHORT discrete_count);

#endif
