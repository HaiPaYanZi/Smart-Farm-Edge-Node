#include "rule_engine.h"

/*
  规则引擎(边缘自治核心)

  原理:
    规则表驱动 + 回差(滞回)判定 + 保持型输出。
    每 500ms 评估一次(控制任务内调用):
      - 传感器值 < 下限      → 执行下限动作(如:湿度<30% 开泵)
      - 传感器值 > 上限      → 执行上限动作(如:湿度>45% 关泵)
      - 上下限之间(回差区)  → 状态保持,不动作
    回差区的存在避免了"阈值附近抖动导致执行器频繁启停"。

  降级:传感器读取失败(回调返回 SENSOR_INVALID)→ 该规则跳过,
    其余规则继续运行;执行器状态表由引擎内部维护,输出用
    RuleEngine_GetActuator 读取,再交给控制任务映射到 PWM/GPIO。
*/

/* 执行器输出状态表(引擎内部维护,保持型输出):ACT_STATE_OFF=关 ACT_STATE_ON=开 */
static uint8_t act_state[ACT_NUM] = {ACT_STATE_OFF, ACT_STATE_OFF, ACT_STATE_OFF}; //初始的执行器都默认关

/* 预置规则表(初值仅兜底):阈值/回差由参数区(param.c 的 Param_Sync)每拍同步,
   支持 Modbus 40008~40011 在线修改 + Flash 掉电保存(nvs.c),见文档《03》§6.1 */
rule_t g_rules[RULE_NUM] = {
    /* R2 补光:光照 <2000lx 开,>3000lx 关(回差 1000),补光灯, 开关 ,高于上限的动作:开 ,低于下限的动作:关 ,开启规则 */
    {SENSOR_LIGHT, 3000.0f, 2000.0f, 1000.0f, ACT_LIGHT, ACTION_OFF, ACTION_ON, RULE_ENABLE},
    /* R3 灌溉:土壤湿度 <30% 开,>45% 关(回差 15) */
    {SENSOR_SOIL, 45.0f, 30.0f, 15.0f, ACT_PUMP, ACTION_OFF, ACTION_ON, RULE_ENABLE},
};

/* 规则引擎初始化:清空执行器状态 */
void RuleEngine_Init(void)
{
    uint8_t i;
    for (i = 0; i < ACT_NUM; i++)
    {
        act_state[i] = ACT_STATE_OFF;   /* 全部执行器初始为关 */
    }
}

/* 规则评估(500ms 周期,控制任务内调用) */
void RuleEngine_Tick(float (*get_sensor)(uint16_t sensor_id))
{
    uint8_t i;    //规则索引
    float value;  //传感器值

    for (i = 0; i < RULE_NUM; i++)
    {
        if (g_rules[i].mode != RULE_ENABLE)
        {
            continue;           /* 禁用(含手动模式覆盖)的规则跳过 */
        }

        value = get_sensor(g_rules[i].sensor_id);   /* 取传感器值 */
        if (value == SENSOR_INVALID)
        {
            continue;           /* 传感器失效 → 该规则降级禁用,其余继续 */
        }

        /* 回差判定:低于下限执行下限动作,高于上限执行上限动作,中间保持 */
        if (value < g_rules[i].threshold_lo)
        {
            if (g_rules[i].action_lo == ACTION_ON)//如果下限阈值开了，就开执行器，做对应动作
            {
                act_state[g_rules[i].actuator_id] = ACT_STATE_ON;
            }
            else if (g_rules[i].action_lo == ACTION_OFF)
            {
                act_state[g_rules[i].actuator_id] = ACT_STATE_OFF;
            }
        }
        else if (value > g_rules[i].threshold_hi)
        {
            if (g_rules[i].action_hi == ACTION_ON)
            {
                act_state[g_rules[i].actuator_id] = ACT_STATE_ON;
            }
            else if (g_rules[i].action_hi == ACTION_OFF)
            {
                act_state[g_rules[i].actuator_id] = ACT_STATE_OFF;
            }
        }
        /* 上下限之间:回差区,状态保持,不动作 */
    }
}

/* 读取执行器当前状态 */
uint8_t RuleEngine_GetActuator(uint8_t actuator_id)
{
    if (actuator_id >= ACT_NUM)
    {
        return ACT_STATE_OFF;   /* 非法编号,按关处理 */
    }
    return act_state[actuator_id];
}
