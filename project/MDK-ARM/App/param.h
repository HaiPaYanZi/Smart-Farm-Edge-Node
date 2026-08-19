#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>

/*
  参数区(应用层配置,对应文档《03》§6.1 的参数区概念)

  数据流向:Modbus 写回调 → Param_Set*(存参数+置 dirty+置落盘请求)
           → 控制任务 100ms Param_Sync() 加载进 PID 实例/规则引擎并清积分
           → nvs.c 的 NVS_Tick() 防抖 2s 后把整个参数结构落盘(掉电不丢)。

  所有可持久化参数集中在 param_payload_t 里:NVS 保存/加载就是对
  这个结构体整体 memcpy,加参数时改结构体+Set/Get 一对接口即可,
  并记得把 nvs.h 的 NVS_VERSION 加 1(旧 Flash 数据自动作废)。

  注意:阈值 40008~40011 已从 g_rules 收编进本模块(2026-08-14),
        g_rules 仍是规则引擎的运行时,参数变更由 Param_Sync 同步过去。
*/

/* 控制模式(寄存器 40005 取值) */
#define MODE_AUTO       0   /* 自动:规则引擎 + 温度 PID 全自动 */
#define MODE_MANUAL     1   /* 手动:执行器由 Modbus(40006/线圈)直接控制 */
#define MODE_STANDBY    2   /* 待机:全部执行器停止输出 */

/* 可持久化参数集合(NVS 落盘的整体,默认值按文档《06》§3.1) */
typedef struct {
    float pid_kp;                  /* PID 比例系数(寄存器 40001,×100,默认 8.0) */
    float pid_ki;                  /* PID 积分系数(寄存器 40002,×100,默认 0.3) */
    float pid_kd;                  /* PID 微分系数(寄存器 40003,×100,默认 2.0) */
    float pid_target_temp;         /* PID 目标温度 ℃(寄存器 40004,×10,默认 25.0) */
    float manual_fan_duty;         /* 手动模式风机占空比 %(寄存器 40006,×10,默认 50.0) */
    float fan_hysteresis;          /* 风机滞回温差 ℃(寄存器 40007,×10,默认 2.0) */
    float light_threshold;         /* 补光阈值 lx(寄存器 40008,默认 2000) */
    float light_hysteresis;        /* 补光回差 lx(寄存器 40009,默认 1000) */
    float irrigation_threshold;    /* 灌溉阈值 %(寄存器 40010,×10,默认 30.0) */
    float irrigation_hysteresis;   /* 灌溉回差 %(寄存器 40011,×10,默认 15.0) */
    uint8_t control_mode;          /* 控制模式(寄存器 40005,默认自动) */
    uint8_t slave_address;         /* Modbus 从站地址(寄存器 40012,默认 4,重启生效) */
} param_payload_t;

void Param_Init(void);                  /* 上电载入默认值 + PID 实例固定工作参数 */

/* ---- Modbus 写入口:存参数+置 dirty+置落盘请求,不直接操作执行器 ---- */
void Param_SetMode(uint8_t mode);       /* 0/1/2,越界按 2(待机)处理 */
void Param_SetKp(float kp);
void Param_SetKi(float ki);
void Param_SetKd(float kd);
void Param_SetTarget(float target);     /* 目标温度 ℃ */
void Param_SetManualDuty(float duty);   /* 手动风机占空比 0~100% */
void Param_SetFanHyst(float hyst);      /* 风机滞回温差:温度 < 目标-温差 → 停转 */
void Param_SetLightThreshold(float lx);         /* 补光阈值:低于则开补光 */
void Param_SetLightHysteresis(float lx);        /* 补光回差:高于 阈值+回差 则关 */
void Param_SetIrrigationThreshold(float pct);   /* 灌溉阈值:低于则开泵 */
void Param_SetIrrigationHysteresis(float pct);  /* 灌溉回差:高于 阈值+回差 则关泵 */
void Param_SetSlaveAddr(uint8_t addr);  /* 从站地址:修改后重启生效 */

/* ---- 读出口:控制任务/显示任务/Modbus 读回调用 ---- */
uint8_t Param_GetMode(void);
float   Param_GetKp(void);
float   Param_GetKi(void);
float   Param_GetKd(void);
float   Param_GetTarget(void);
float   Param_GetManualDuty(void);
float   Param_GetFanHyst(void);
float   Param_GetLightThreshold(void);
float   Param_GetLightHysteresis(void);
float   Param_GetIrrigationThreshold(void);
float   Param_GetIrrigationHysteresis(void);
uint8_t Param_GetSlaveAddr(void);

/* ---- NVS 对接接口(供 nvs.c 调用,应用代码不用管) ---- */
void Param_Load(const param_payload_t *payload);   /* 上电加载 Flash 参数:静默写入参数区(不触发落盘),但置 dirty 刷 PID */
void Param_GetPayload(param_payload_t *out);       /* 读出当前全部参数(NVS 落盘前打包用) */
uint8_t Param_TakeSaveRequest(void);               /* 取走"落盘请求"标志:NVS_Tick 每拍调用,有请求返回 1 并清标志 */
void Param_ClearSaveRequest(void);                 /* 清除落盘请求(NVS 上电加载后调用,防止把刚读出的参数原样写回) */

/* ---- 参数同步:dirty 时刷进 PID 实例/规则引擎并清积分(控制任务 100ms 调) ---- */
void Param_Sync(void);

#endif
