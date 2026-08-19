#ifndef __PID_H__
#define __PID_H__

/* PID 结构体(江协风格命名:Kp/Ki/Kd 是行业标准符号,Target/Actual 表示目标与实际) */
typedef struct {		//定义PID结构体变量类型
	float Target;		//目标值:期望温度(℃),由参数区(param.c)同步进来
	float Actual;		//实际值:当前测量温度(℃),控制任务每拍喂最新快照
	float Out;			//输出值:0~100 占空比 %,作用于风机 PWM

	float Kp;			//比例项权重:误差 × Kp,误差越大输出越大
	float Ki;			//积分项权重:误差积分 × Ki,消除稳态误差
	float Kd;			//微分项权重:温度变化率 × Kd,抑制突变(微分先行)

	float Error0;		//本次误差 = 目标值 - 实际值
	float Error1;		//上次误差(位置式 PID 公式用)
	float ErrorInt;		//误差积分累加值(抗饱和:输出饱和时停止累加)

	float OutMax;		//输出限幅的最大值(本工程固定 100)
	float OutMin;		//输出限幅的最小值(本工程固定 0)

	//优化
	float OutOffset;    //输出偏移设置(本工程不用,固定 0)
	float Actual1;      //上次实际值,用于微分先行(对实际值差分而不是误差差分)
	float ErrorIntMax;  //积分限幅最大值(随 Ki 动态重算 = OutMax/Ki)
	float ErrorIntMin;  //积分限幅最小值(负的积分限幅最大值)
} PID_t;

/* PID 运行时实例(全局唯一,风机温度环):param.c 的 Param_Sync 维护参数,
   task_control 每 100ms 调用 PID_Update 计算 */
extern PID_t g_pid;

void PID_Init(PID_t *p);    /* 初始化:清运行状态(积分/误差/输出归零),上电时调用 */
void PID_Update(PID_t *p);  /* PID 计算:位置式+抗积分饱和+输出限幅,每 100ms 调用 */
void PID_Reset(PID_t *p);   /* 重置:清积分/误差/输出,参数变更、模式切换、停转区间时调用 */

#endif
