#ifndef __VERSION_H__
#define __VERSION_H__

/*
  固件版本号(语义化版本:主.次.补丁)

  版本比较规则(Boot 的 0x02 开始传输时校验,拒绝降级):
    新固件 (major,minor,patch) 数值组合 < 活动分区固件 → 应答"版本过低"拒绝
  发布新固件时:改这里的三个宏,PC 打包工具会读它们写进固件头 16B
*/
#define APP_VERSION_MAJOR   1       /* 主版本:架构/协议不兼容变更 +1 */
#define APP_VERSION_MINOR   7       /* 次版本:新增功能(1.7=OTA 升级支持) */
#define APP_VERSION_PATCH   4       /* 补丁版本:缺陷修复 */  //默认是0
#define APP_VERSION_STR     "1.7.4" /* 版本字符串(debug version 命令打印,与宏保持一致) */

#endif /* __VERSION_H__ */
