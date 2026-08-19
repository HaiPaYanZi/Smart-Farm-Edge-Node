#ifndef __RULE_ENGINE_H__
#define __RULE_ENGINE_H__

#include <stdint.h>

/* 传感器输入源(回调函数 get_sensor 的输入,由调用方映射) */
#define SENSOR_TEMP     0//温度
#define SENSOR_HUMI     1//湿度
#define SENSOR_LIGHT    2//环境光
#define SENSOR_SOIL     3//土壤湿度

/* 执行器编号 */
#define ACT_FAN         0           //通风机
#define ACT_LIGHT       1           //补光灯
#define ACT_PUMP        2           //水泵
#define ACT_NUM         3           /* 执行器总数,状态表大小 */

/* 动作定义(rule_t.action_hi / action_lo 取值) */
#define ACTION_NONE     0           /* 无动作,不改变执行器状态 */
#define ACTION_ON       1           /* 开 */
#define ACTION_OFF      2           /* 关 */

/* 执行器状态(act_state 取值)——状态是事实,动作是指令,两者语义不同,不能混用 */
#define ACT_STATE_OFF   0           /* 关 */
#define ACT_STATE_ON    1           /* 开 */

/* 规则模式 */
#define RULE_DISABLE    0           /* 禁用 */
#define RULE_ENABLE     1           /* 使能(自动) */

/* 传感器无效标志:回调约定——传感器读取失败时返回该值,对应规则自动降级禁用 */
#define SENSOR_INVALID  (-1.0f)

/* 规则表项*/
typedef struct {
    uint16_t sensor_id;      /* 输入源: SENSOR_TEMP/HUMI/LIGHT/SOIL */
    float    threshold_hi;   /* 上限阈值 */
    float    threshold_lo;   /* 下限阈值 */
    float    hysteresis;     /* 回差(记录用,实际回差由 上限-下限 区间体现) */
    uint8_t  actuator_id;    /* 执行器: ACT_FAN/ACT_LIGHT/ACT_PUMP */
    uint8_t  action_hi;      /* 高于上限的动作: 0无/1开/2关 */
    uint8_t  action_lo;      /* 低于下限的动作: 0无/1开/2关 */
    uint8_t  mode;           /* 0=禁用 1=使能(自动) */
} rule_t;

/* 预置规则表(§3.2:R2 补光、R3 灌溉;R1 通风走 PID,不在规则表内) */
#define RULE_NUM        2       /* 默认规则条数 */
extern rule_t g_rules[RULE_NUM];

/* 规则引擎初始化:清空执行器状态表(上电/模式切换时调用) */
void RuleEngine_Init(void);

/* 规则评估:每 500ms 调用一次(控制任务内)
   参数 get_sensor:传感器取值回调,由调用方提供(解耦引擎与具体传感器驱动)
   返回值约定:读取失败返回 SENSOR_INVALID → 该规则本次跳过(降级) */
void RuleEngine_Tick(float (*get_sensor)(uint16_t sensor_id));//函数指针,指向传感器取值回调函数

/* 读取执行器当前状态:返回 ACT_STATE_OFF=关 / ACT_STATE_ON=开 */
uint8_t RuleEngine_GetActuator(uint8_t actuator_id);

#endif
