#ifndef __OTA_H__
#define __OTA_H__

/*
  OTA 升级触发(App 侧,文档《07》)

  触发路径(两条,效果相同):
    1. Modbus 写 40020=0x5A5A(远程主站发起)
    2. 调试命令 ota(串口 CLI 发起)
  流程:OTA_RequestUpgrade() 把升级请求写进标志区(0x08003000,掉电保持),
       OTA_Poll() 等 200ms(让 Modbus 应答帧发完)后软复位;
       复位后 Bootloader 见 upgrade_req=1 → 进升级模式等 PC 工具握手。
*/

/* 触发升级:写标志区 upgrade_req=1 并启动复位倒计时(调用后 200ms 内复位) */
void OTA_RequestUpgrade(void);

/* 复位倒计时轮询:控制任务 100ms 节拍里调用,计时到就软复位 */
void OTA_Poll(void);

#endif /* __OTA_H__ */
