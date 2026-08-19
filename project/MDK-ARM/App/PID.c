#include <stm32f1xx_hal.h>
#include "PID.h"

/* PID 运行时实例:参数由 param.c 同步,计算由控制任务驱动 */
PID_t g_pid;

/**
  * 函    数：PID初始化
  * 参    数：p 指定结构体的地址
  * 返 回 值：无
  */
 void PID_Init(PID_t *p)
 {
     /*把PID表示状态的参数清零，避免之前遗留的参数对本次启动造成影响*/
     p->Target = 0;
     p->Actual = 0;
     p->Out = 0;
     p->Error0 = 0;
     p->Error1 = 0;
     p->ErrorInt = 0;
     p->Actual1 = 0;
 }

 /**
   * 函    数：PID重置(清积分/误差/输出)
   * 参    数：p 指定结构体的地址
   * 返 回 值：无
   * 说    明：以下三种场合必须调用,否则输出会突跳:
   *           1. PID 参数(Kp/Ki/Kd/目标)被修改后——旧积分按旧参数攒的,作废;
   *           2. 模式切换(手动/待机切回自动)——积分停算期间误差会攒大;
   *           3. 滞回停转区间——低温停转时不许积分继续往上顶,回温后从 0 平稳起步。
   */
 void PID_Reset(PID_t *p)
 {
     p->Out = 0;
     p->Error0 = 0;
     p->Error1 = 0;
     p->ErrorInt = 0;
     p->Actual1 = 0;
 }

 /**
   * 函    数：PID计算及结构体变量值更新
   * 参    数：p 指定结构体的地址
   * 返 回 值：无
   */
 void PID_Update(PID_t *p)
 {
     /*获取本次误差和上次误差*/
     p->Error1 = p->Error0;					//获取上次误差
     p->Error0 = p->Actual - p->Target;		//获取本次误差,实际值减目标值:温度高于目标→正误差→正输出(风机降温语义)

     /*抗积分饱和(clamping法,文档《03》§2.2):
       输出已顶到上限且误差还是正的(继续往上推) → 积分停止累加;
       输出已贴到下限且误差还是负的 → 同样停止。
       误差反向时积分正常累加,把输出从饱和点拉回来。
       不加这一条的话,输出饱和期间积分还在涨,等误差转负时
       输出迟迟回不来,温度过冲明显(整定时表现为"冲过目标很久才回头") */
     uint8_t output_saturated = ((p->Out >= p->OutMax) && (p->Error0 > 0))   /* 正向饱和 */
                             || ((p->Out <= p->OutMin) && (p->Error0 < 0));  /* 反向饱和 */

     /*外环误差积分（累加）*/
     /*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
     /*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
     /*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
     if (p->Ki == 0)					//Ki为0:积分项不参与,直接清零
     {
         p->ErrorInt = 0;			//误差积分直接归0
     }
     else if (!output_saturated)		//Ki不为0且输出未饱和:正常累加
     {
         p->ErrorInt += p->Error0;	//进行误差积分
		if(p->ErrorInt > p->ErrorIntMax) p->ErrorInt = p->ErrorIntMax;
		if(p->ErrorInt < p->ErrorIntMin) p->ErrorInt = p->ErrorIntMin;
     }
     /*else 饱和中:积分保持不动——既不累加也不清零。
       清零会丢掉已攒的积分,饱和解除后要重新慢慢积,积分作用被削弱;
       累加则攒出 windup(见上面的 sat 注释)。所以"停住"才是正确语义 */

     /*PID计算*/
     /*使用位置式PID公式，计算得到输出值*/
     p->Out = p->Kp * p->Error0
            + p->Ki * p->ErrorInt
           // + p->Kd * (p->Error0 - p->Error1);
			- p->Kd *(p->Actual - p->Actual1);//使用微分先行去抑制突变，就是将对误差的控制改成对目标值的控制

	 //输出偏移
	 if(p->Out > 0) p->Out += p->OutOffset;
	 if(p->Out < 0) p->Out -= p->OutOffset;

     /*输出限幅*/
     if (p->Out > p->OutMax) {p->Out = p->OutMax;}	//限制输出值最大为结构体指定的OutMax
     if (p->Out < p->OutMin) {p->Out = p->OutMin;}	//限制输出值最小为结构体指定的OutMin

	 p->Actual1 = p->Actual;//更新上次目标值
 }
