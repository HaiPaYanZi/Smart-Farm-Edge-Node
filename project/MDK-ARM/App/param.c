#include "param.h"
#include "PID.h"
#include "rule_engine.h"
#include <string.h>

/*
  参数区实现:所有可持久化参数集中在 param_payload_t 里,默认值按文档
  《06-Modbus寄存器表》§3.1(Kp=8.0/Ki=0.3/Kd=2.0/目标25℃/自动/手动50%/
  滞回2.0℃/补光2000lx回差1000/灌溉30%回差15%/从站4)。
*/

extern PID_t g_pid;     /* PID 运行时实例(pid.c 定义):参数同步时把配置刷进去 */

/* 参数存储本体:Modbus 任务写、控制任务读,M3 内核单值读写原子无需锁;
   掉电保存由 nvs.c 通过 Param_GetPayload/Param_Load 整体搬进搬出 */
static param_payload_t s_param;

static volatile uint8_t param_dirty_flag = 0;    /* 参数变更标志:写入口置 1,Param_Sync 消费后清 0 */
static volatile uint8_t save_request_flag = 0;   /* 落盘请求标志:写入口置 1,NVS_Tick 取走后清 0 */

/**
  * 函    数：Param_Init
  * 功    能：上电初始化参数区:载入默认参数并设置 PID 实例的固定工作参数(限幅等)
  * 参    数：无
  * 返 回 值：无
  * 说    明：默认值只保证"没有 Flash 数据时也能运行";有 Flash 数据时
  *           随后 NVS_Init 会调 Param_Load 覆盖成上次保存的值
  */
void Param_Init(void)
{
    /* 载入默认参数(文档《06》§3.1) */
    s_param.pid_kp                 = 8.0f;
    s_param.pid_ki                 = 0.3f;
    s_param.pid_kd                 = 2.0f;
    s_param.pid_target_temp        = 25.0f;
    s_param.manual_fan_duty        = 50.0f;
    s_param.fan_hysteresis         = 2.0f;
    s_param.light_threshold        = 2000.0f;
    s_param.light_hysteresis       = 1000.0f;
    s_param.irrigation_threshold   = 30.0f;
    s_param.irrigation_hysteresis  = 15.0f;
    s_param.control_mode           = MODE_AUTO;
    s_param.slave_address          = 4;

    PID_Init(&g_pid);           /* 清 PID 运行状态:积分/误差/输出全部归零 */

    /* 设置 PID 实例的固定工作参数(这些不随 Modbus 参数变化) */
    g_pid.OutMax = 100.0f;      /* 输出上限 100%:对应 PWM 满占空比 */
    g_pid.OutMin = 0.0f;        /* 输出下限 0%:对应风机停转 */
    g_pid.OutOffset = 0.0f;     /* 输出偏移不用,固定 0 */

    /* 积分限幅按默认 Ki 计算(原理见 Param_Sync 的注释),首次同步时还会按实际 Ki 重算 */
    g_pid.ErrorIntMax =  g_pid.OutMax / s_param.pid_ki;
    g_pid.ErrorIntMin = -g_pid.ErrorIntMax;

    param_dirty_flag = 1;       /* 触发首次同步:把默认参数刷进 PID 实例 */
}

/* ================= Modbus 写入口 =================
   共同约定:只存参数+置两个标志,不碰执行器、不碰 Flash;
   实际生效(刷 PID/规则引擎)在控制任务 Param_Sync,落盘在 NVS_Tick */

/**
  * 函    数：Param_SetMode
  * 功    能：设置控制模式(寄存器 40005 写回调入口)
  * 参    数：mode 模式值 0=自动 1=手动 2=待机
  * 返 回 值：无
  * 说    明：越界值保守处理为待机(全停最安全);模式本身无需 dirty,控制任务每拍都读
  */
void Param_SetMode(uint8_t mode)
{
    if (mode > MODE_STANDBY)    /* 模式值越界(大于 2) */
    {
        mode = MODE_STANDBY;    /* 保守按待机处理:全部执行器停止,最安全 */
    }
    s_param.control_mode = mode;/* 存入参数区,控制任务下个 100ms 节拍读到即生效 */
    save_request_flag = 1;      /* 置落盘请求:模式也要掉电保存 */
}

/**
  * 函    数：Param_SetKp
  * 功    能：设置 PID 比例系数(寄存器 40001 写回调入口)
  * 参    数：kp 比例系数(调用方已把寄存器值 ÷100 还原成浮点)
  * 返 回 值：无
  */
void Param_SetKp(float kp)
{
    s_param.pid_kp = kp;        /* 存入参数区 */
    param_dirty_flag = 1;       /* 置变更标志:PID 参数变了,控制任务要重载+清积分 */
    save_request_flag = 1;      /* 置落盘请求:防抖后写入 Flash,掉电不丢 */
}

/**
  * 函    数：Param_SetKi
  * 功    能：设置 PID 积分系数(寄存器 40002 写回调入口)
  * 参    数：ki 积分系数(调用方已 ÷100)
  * 返 回 值：无
  */
void Param_SetKi(float ki)
{
    s_param.pid_ki = ki;
    param_dirty_flag = 1;
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetKd
  * 功    能：设置 PID 微分系数(寄存器 40003 写回调入口)
  * 参    数：kd 微分系数(调用方已 ÷100)
  * 返 回 值：无
  */
void Param_SetKd(float kd)
{
    s_param.pid_kd = kd;
    param_dirty_flag = 1;
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetTarget
  * 功    能：设置 PID 目标温度(寄存器 40004 写回调入口)
  * 参    数：target 目标温度 ℃(调用方已 ÷10)
  * 返 回 值：无
  */
void Param_SetTarget(float target)
{
    s_param.pid_target_temp = target;   /* 存入参数区 */
    param_dirty_flag = 1;               /* 目标变了旧积分作废,同样走重载流程 */
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetManualDuty
  * 功    能：设置手动模式风机占空比(寄存器 40006 写回调入口)
  * 参    数：duty 占空比 %(调用方已 ÷10),内部限幅到 0~100
  * 返 回 值：无
  * 说    明：该值不参与 PID,不需要 dirty;手动模式下由写回调直接写 g_fan_duty 生效
  */
void Param_SetManualDuty(float duty)
{
    if (duty < 0.0f)    duty = 0.0f;      /* 下限保护:占空比不能为负 */
    if (duty > 100.0f)  duty = 100.0f;    /* 上限保护:占空比最大 100% */
    s_param.manual_fan_duty = duty;       /* 存入参数区 */
    save_request_flag = 1;                /* 也参与持久化,置落盘请求 */
}

/**
  * 函    数：Param_SetFanHyst
  * 功    能：设置风机滞回温差(寄存器 40007 写回调入口)
  * 参    数：hyst 滞回温差 ℃(调用方已 ÷10):温度低于 目标-温差 时风机停转
  * 返 回 值：无
  */
void Param_SetFanHyst(float hyst)
{
    if (hyst < 0.0f)    hyst = 0.0f;      /* 下限保护:温差不能为负 */
    s_param.fan_hysteresis = hyst;        /* 存入参数区 */
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetLightThreshold
  * 功    能：设置补光阈值(寄存器 40008 写回调入口)
  * 参    数：lx 光照强度阈值 lx:低于则开补光
  * 返 回 值：无
  */
void Param_SetLightThreshold(float lx)
{
    s_param.light_threshold = lx;
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetLightHysteresis
  * 功    能：设置补光回差(寄存器 40009 写回调入口)
  * 参    数：lx 回差 lx:高于 阈值+回差 则关补光
  * 返 回 值：无
  */
void Param_SetLightHysteresis(float lx)
{
    s_param.light_hysteresis = lx;
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetIrrigationThreshold
  * 功    能：设置灌溉阈值(寄存器 40010 写回调入口)
  * 参    数：pct 土壤湿度阈值 %:低于则开泵
  * 返 回 值：无
  */
void Param_SetIrrigationThreshold(float pct)
{
    s_param.irrigation_threshold = pct;
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetIrrigationHysteresis
  * 功    能：设置灌溉回差(寄存器 40011 写回调入口)
  * 参    数：pct 回差 %:高于 阈值+回差 则关泵
  * 返 回 值：无
  */
void Param_SetIrrigationHysteresis(float pct)
{
    s_param.irrigation_hysteresis = pct;
    save_request_flag = 1;
}

/**
  * 函    数：Param_SetSlaveAddr
  * 功    能：设置 Modbus 从站地址(寄存器 40012 写回调入口)
  * 参    数：addr 从站地址(1~247)
  * 返 回 值：无
  * 说    明：仅存储不改当前协议栈地址,重启后 eMBInit 才用它(见 freertos.c 的 Modbus_Task)
  */
void Param_SetSlaveAddr(uint8_t addr)
{
    s_param.slave_address = addr;   /* 存入参数区(重启生效) */
    save_request_flag = 1;
}

/* ================= 读出口 ================= */

/**
  * 函    数：Param_GetMode
  * 功    能：读当前控制模式(控制任务每拍判断行为,OLED 系统页显示)
  * 返 回 值：0=自动 1=手动 2=待机
  */
uint8_t Param_GetMode(void)                   { return s_param.control_mode; }

/**
  * 函    数：Param_GetKp
  * 功    能：读 PID 比例系数(Modbus 读回 40001、调试命令 pid 查询用)
  * 返 回 值：比例系数浮点值
  */
float   Param_GetKp(void)                     { return s_param.pid_kp; }

/**
  * 函    数：Param_GetKi
  * 功    能：读 PID 积分系数
  * 返 回 值：积分系数浮点值
  */
float   Param_GetKi(void)                     { return s_param.pid_ki; }

/**
  * 函    数：Param_GetKd
  * 功    能：读 PID 微分系数
  * 返 回 值：微分系数浮点值
  */
float   Param_GetKd(void)                     { return s_param.pid_kd; }

/**
  * 函    数：Param_GetTarget
  * 功    能：读 PID 目标温度
  * 返 回 值：目标温度 ℃
  */
float   Param_GetTarget(void)                 { return s_param.pid_target_temp; }

/**
  * 函    数：Param_GetManualDuty
  * 功    能：读手动模式风机占空比
  * 返 回 值：占空比 %
  */
float   Param_GetManualDuty(void)             { return s_param.manual_fan_duty; }

/**
  * 函    数：Param_GetFanHyst
  * 功    能：读风机滞回温差
  * 返 回 值：滞回温差 ℃
  */
float   Param_GetFanHyst(void)                { return s_param.fan_hysteresis; }

/**
  * 函    数：Param_GetLightThreshold
  * 功    能：读补光阈值(Modbus 读回 40008 用)
  * 返 回 值：光照阈值 lx
  */
float   Param_GetLightThreshold(void)         { return s_param.light_threshold; }

/**
  * 函    数：Param_GetLightHysteresis
  * 功    能：读补光回差
  * 返 回 值：回差 lx
  */
float   Param_GetLightHysteresis(void)        { return s_param.light_hysteresis; }

/**
  * 函    数：Param_GetIrrigationThreshold
  * 功    能：读灌溉阈值(Modbus 读回 40010 用)
  * 返 回 值：土壤湿度阈值 %
  */
float   Param_GetIrrigationThreshold(void)    { return s_param.irrigation_threshold; }

/**
  * 函    数：Param_GetIrrigationHysteresis
  * 功    能：读灌溉回差
  * 返 回 值：回差 %
  */
float   Param_GetIrrigationHysteresis(void)   { return s_param.irrigation_hysteresis; }

/**
  * 函    数：Param_GetSlaveAddr
  * 功    能：读 Modbus 从站地址(Modbus 读回 40012、重启初始化 eMBInit 用)
  * 返 回 值：从站地址(默认 4)
  */
uint8_t Param_GetSlaveAddr(void)              { return s_param.slave_address; }

/* ================= NVS 对接接口 ================= */

/**
  * 函    数：Param_Load
  * 功    能：上电加载 Flash 里的参数(NVS_Init 调用):整体覆盖参数区
  * 参    数：payload NVS 读出的参数载荷指针(已通过 CRC 校验)
  * 返 回 值：无
  * 说    明：静默写入——只置 dirty(让 Param_Sync 把加载值刷进 PID),
  *          不置落盘请求(数据本来就来自 Flash,再写回去是浪费一次擦写)
  */
void Param_Load(const param_payload_t *payload)
{
    memcpy(&s_param, payload, sizeof(param_payload_t));  /* 整体覆盖参数区 */
    param_dirty_flag = 1;       /* 置变更标志:加载值要刷进 PID 实例/规则引擎 */
}

/**
  * 函    数：Param_GetPayload
  * 功    能：读出当前全部参数(NVS 落盘前打包用)
  * 参    数：out 输出缓冲(调用方提供 param_payload_t 变量地址)
  * 返 回 值：无
  */
void Param_GetPayload(param_payload_t *out)
{
    memcpy(out, &s_param, sizeof(param_payload_t));      /* 整体拷出参数区 */
}

/**
  * 函    数：Param_TakeSaveRequest
  * 功    能：取走"落盘请求"标志(NVS_Tick 每拍调用)
  * 参    数：无
  * 返 回 值：1=有参数变更待落盘(本次调用同时清掉标志);0=没有
  */
uint8_t Param_TakeSaveRequest(void)
{
    if (save_request_flag)
    {
        save_request_flag = 0;  /* 取走标志:NVS_Tick 会开始防抖倒计时 */
        return 1;
    }
    return 0;
}

/**
  * 函    数：Param_ClearSaveRequest
  * 功    能：清除落盘请求(NVS 上电加载后调用,防止把刚读出的参数原样写回 Flash)
  * 参    数：无
  * 返 回 值：无
  */
void Param_ClearSaveRequest(void)
{
    save_request_flag = 0;      /* 清标志即可,防抖倒计时在 nvs.c 内部管理 */
}

/* ================= 参数同步 ================= */

/**
  * 函    数：Param_Sync
  * 功    能：参数同步(控制任务 100ms 节拍调用):参数被改过时,把新参数
  *           刷进 PID 实例和规则引擎,并清积分
  * 参    数：无
  * 返 回 值：无
  * 说    明：文档《03》§5.1 规定写回调禁止耗时操作、实际生效由控制任务下周期
  *           加载,本函数就是那个"加载"动作
  */
void Param_Sync(void)
{
    if (!param_dirty_flag)
    {
        return;                 /* 参数没变,啥也不干(100ms 每次进来都很便宜) */
    }
    param_dirty_flag = 0;       /* 先清标志再加载,防止加载期间新写入被漏掉 */

    /* 参数刷进 PID 实例(运行时用实例,参数区只负责存储) */
    g_pid.Kp = s_param.pid_kp;
    g_pid.Ki = s_param.pid_ki;
    g_pid.Kd = s_param.pid_kd;
    g_pid.Target = s_param.pid_target_temp;

    /* 积分限幅随 Ki 重算:保证"积分项(Ki×积分)最多贡献满量程输出",
       避免 Ki 调大后积分项把输出顶出限幅范围(超出的部分白白积在积分器里) */
    g_pid.ErrorIntMax =  g_pid.OutMax / s_param.pid_ki;
    g_pid.ErrorIntMin = -g_pid.ErrorIntMax;

    /* 参数刷进规则引擎:阈值/回差(40008~40011 已收编进参数区) */
    g_rules[0].threshold_lo = s_param.light_threshold;                  /* R2 补光下限 */
    g_rules[0].hysteresis    = s_param.light_hysteresis;                /* R2 补光回差 */
    g_rules[0].threshold_hi  = s_param.light_threshold + s_param.light_hysteresis;  /* 派生值:上限=下限+回差 */
    g_rules[1].threshold_lo  = s_param.irrigation_threshold;            /* R3 灌溉下限 */
    g_rules[1].hysteresis    = s_param.irrigation_hysteresis;           /* R3 灌溉回差 */
    g_rules[1].threshold_hi  = s_param.irrigation_threshold + s_param.irrigation_hysteresis;  /* 派生值 */

    /* 参数变更清积分:旧参数攒下的积分/误差对新的控制律没有意义,
       不清的话输出会从旧状态突跳(整定 Kp 时最常见) */
    PID_Reset(&g_pid);
}
