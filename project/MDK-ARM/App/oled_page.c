#include "oled_page.h"
#include "MYOLED.h"
#include "MySensor.h"
#include "fan.h"
#include "light.h"
#include "pump.h"
#include "stall_detect.h"
#include "param.h"
#include "main.h"          /* KEY1(PC9) 引脚宏 */

/*
  OLED 三页界面

  布局:128×64,6×8 字体 → 每行 21 字符,共 8 行(行距 8px)
  页 1 传感器页:
    [0] T:28.8C
    [1] H:41.8%
    [2] L:1234lx
    [3] S:78.9%
    [7] 1/3
  页 2 执行器页:
    [0] FAN:60% 1234rpm
    [1] LIGHT:ON
    [2] PUMP:OFF
    [3] STALL:NORMAL
  页 3 系统页:
    [0] MODE:AUTO
    [1] RUN:123s
    [2] FAULT:NONE
    [3] VER:V1.0

  数据全部来自快照/状态查询(无 I2C 竞争);写屏由显示任务持锁调用。
*/

#define PAGE_NUM        3

static uint8_t page = 0;        /* 当前页 0~2 */
static uint8_t key1_state = 0;  /* KEY1 状态:0=释放 1=按下(松手前不重复切页) */

static uint32_t run_sec = 0;    /* 运行秒数(500ms 刷新 ×2) */

/* ---------- 页 1:传感器 ---------- */
static void show_sensor_page(void)
{
    float t = MySensor_GetValue(SENSOR_TEMP);
    float h = MySensor_GetValue(SENSOR_HUMI);
    float l = MySensor_GetValue(SENSOR_LIGHT);
    float s = MySensor_GetValue(SENSOR_SOIL);

    OLED_ShowString(0, 0, "T:", 8);
    OLED_ShowFloatNum(12, 0, t, 2, 1, 8);
    OLED_ShowChar(30, 0, 'C', 8);

    OLED_ShowString(0, 8, "H:", 8);
    OLED_ShowFloatNum(12, 8, h, 2, 1, 8);
    OLED_ShowChar(30, 8, '%', 8);

    OLED_ShowString(0, 16, "L:", 8);
    OLED_ShowNum(12, 16, (uint32_t)l, 4, 8);
    OLED_ShowString(36, 16, "lx", 8);

    OLED_ShowString(0, 24, "S:", 8);
    OLED_ShowFloatNum(12, 24, s, 2, 1, 8);
    OLED_ShowChar(30, 24, '%', 8);

    OLED_ShowString(0, 56, "1/3", 8);
}

/* ---------- 页 2:执行器 ---------- */
static void show_actuator_page(void)
{
    /* 风机:占空比 + 转速 */
    OLED_ShowString(0, 0, "FAN:", 8);
    OLED_ShowNum(30, 0, Fan_GetDuty(), 3, 8);
    OLED_ShowChar(48, 0, '%', 8);
    OLED_ShowNum(54, 0, Fan_GetRpm(), 4, 8);
    OLED_ShowString(78, 0, "rpm", 8);

    /* 补光/水泵:显示"执行器实际输出状态"(读引脚/PWM 实际值而非规则引擎状态表,
       手动模式下 Modbus 线圈直接操作执行器时显示依然真实) */
    OLED_ShowString(0, 8, "LIGHT:", 8);
    OLED_ShowString(42, 8, Light_IsOn() ? "ON" : "OFF", 8);

    OLED_ShowString(0, 16, "PUMP:", 8);
    OLED_ShowString(42, 16, Pump_IsOn() ? "ON" : "OFF", 8);

    /* 堵转状态 */
    OLED_ShowString(0, 24, "STALL:", 8);
    switch (StallDetect_GetState())
    {
        case STALL_NORMAL:     OLED_ShowString(42, 24, "NORMAL", 8); break;
        case STALL_MONITORING: OLED_ShowString(42, 24, "CHECK", 8);  break;
        case STALL_FAULT:      OLED_ShowString(42, 24, "FAULT!", 8); break;
        case STALL_FAULT_SEVERE: OLED_ShowString(42, 24, "SEVERE", 8); break;
    }

    OLED_ShowString(0, 56, "2/3", 8);
}

/* ---------- 页 3:系统 ---------- */
static void show_system_page(void)
{
    OLED_ShowString(0, 0, "MODE:", 8);
    switch (Param_GetMode())            /* 实际模式(40005/调试命令改,不再是写死 AUTO) */
    {
        case MODE_MANUAL:  OLED_ShowString(30, 0, "MANUAL", 8); break;
        case MODE_STANDBY: OLED_ShowString(30, 0, "STBY", 8);   break;
        default:           OLED_ShowString(30, 0, "AUTO", 8);   break;
    }

    OLED_ShowString(0, 8, "RUN:", 8);
    OLED_ShowNum(30, 8, run_sec, 6, 8);
    OLED_ShowString(66, 8, "s", 8);

    OLED_ShowString(0, 16, "FAULT:", 8);
    OLED_ShowString(42, 16, StallDetect_IsStalled() ? "STALL" : "NONE", 8);

    OLED_ShowString(0, 24, "VER:V1.0", 8);

    OLED_ShowString(0, 56, "3/3", 8);
}

/* 刷新当前页 */
void OLED_Page_Show(void)
{
    switch (page)
    {
        case 0:  show_sensor_page();   break;
        case 1:  show_actuator_page(); break;
        default: show_system_page();   break;
    }
    OLED_Update();          /* 显存 → 屏幕 */

    run_sec += 2;           /* 500ms 调用 ×2 = 1 秒 */
}

/* 直接切页(调试命令用) */
void OLED_Page_JumpTo(uint8_t p)
{
    if (p < PAGE_NUM)
    {
        page = p;
        OLED_Clear();               /* 只清显存,不触发 I2C(下一帧重绘时发送) */
    }
}

/* 按键轮询:KEY1(PC9,上拉,按下=低)切页
   边沿触发:按下瞬间切一页,松手解锁后才能再切(长按也只切一次)。
   100ms 轮询间隔本身就是消抖——机械抖动只有 ~10ms,连续两拍采不到,
   所以不用"按住 N 拍才认"的旧写法,按一下即响应 */
void OLED_Page_KeyScan(void)
{
    uint8_t is_pressed = (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET); /* 本拍电平:1=按下 */

    if (is_pressed && key1_state == 0)      /* 释放→按下边沿:切页 */
    {
        key1_state = 1;                     /* 锁住,长按期间不重复切 */
        page = (page + 1) % PAGE_NUM;       /* 下一页,循环 */
        OLED_Clear();                       /* 切页先清屏(下一帧重绘) */
    }
    else if (!is_pressed)
    {
        key1_state = 0;                     /* 释放解锁,等下次按下 */
    }
}
