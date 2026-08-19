# CubeMX 重新生成代码后的恢复清单

> 本项目的 FreeModbus/FatFS 移植有手改代码，CubeMX 重新生成（GENERATE CODE）后
> 必须先按本清单逐项恢复，再编译。**顺序无先后，全都要做。**

## 1. stm32f1xx_it.c —— USART3 中断接管

FreeModbus 的 RTU 从机用寄存器级串口中断（vMBPortSerialISR 逐字节收），
CubeMX 生成的 HAL 处理会吞掉 RXNE 字节。

- 重新生成后 `USART3_IRQHandler` 里的 `HAL_UART_IRQHandler(&huart3);` 会被恢复
- **恢复动作**：把这一行注释掉（函数体里已有提醒注释，照着做即可）

## 2. ffconf.h —— FatFs 三项手改配置

CubeMX 生成 FATFS/Target/ffconf.h 时按默认值覆盖，以下三项是手改的：

| 项 | 手改值 | 默认值 | 不改的后果 |
|---|---|---|---|
| `_USE_WRITE` | 1 | 0 | f_write 直接链接错误 |
| `_USE_IOCTL` | 1 | 0 | ioctl(CTRL_SYNC/GET_SECTOR_COUNT) 失效 |
| `_FS_REENTRANT` | 0 | 1 | R0.11 自带 syscall.c 是 CMSIS v1 API，与 FreeRTOS v2 冲突编译报错 |

## 3. 其他 CubeMX 不会动的（无需恢复，供确认）

- FreeRTOS heap：`FREERTOS.configTOTAL_HEAP_SIZE=16384` 已写进 .ioc，重新生成后
  FreeRTOSConfig.h 会保持 16KB，**不会回退**
- 自建任务（debug_cmd / modbus / sd_log）都在 USER CODE 区，重新生成后原样保留
- TIM2（Modbus 3.5T 超时）、SPI2+PB12（SD 卡）都在 .ioc 里，重新生成后配置保留
- HAL 心跳是 TIM4（stm32f1xx_hal_timebase_tim.c 的 htim4），所以 TIM4 不能挪作他用

## 4. 手改文件清单（排查用，重生成均不动）

- `MDK-ARM/App/` 下全部文件（App 分组是手加进 uvprojx 的，重生成不动）
- `MDK-ARM/Bsp/sd_spi.c/h`、`MDK-ARM/Divece/`、`MDK-ARM/FreeModbus/`
- `Core/Src/stm32f1xx_it.c`（仅第 1 条需要恢复）
- `FATFS/Target/user_diskio.c` 的 SD 对接代码全部在 USER CODE 区（已核实），重生成保留，**无需恢复**
