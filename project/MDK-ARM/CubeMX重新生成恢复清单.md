# CubeMX 重新生成代码后的恢复清单（操作卡）

> CubeMX（GENERATE CODE）后**先按本清单逐项核对/恢复，再编译**。全都要过一遍。
> 完整规范见 `docs/05-配置管理规范.md` §5。2026-08-19 全量盘点补全。

## 1. Keil Target —— App IROM1 基址（OTA 双分区核心！）

CubeMX 重生成会把 Keil Target 内存配置重置为默认 **0x08000000 / 0x80000**，
与 Boot 区重叠，双分区全废，**烧录即坏**。

- 位置：Options for Target → **Target** 页 → IROM1
- **正确值：Start=0x10200，Size=0x18E00**（sct 是自动生成的，源头在这里）

## 2. stm32f1xx_it.c —— USART3 中断接管（Modbus 核心）

FreeModbus 的 RTU 从机用寄存器级串口中断（vMBPortSerialISR 逐字节收），
CubeMX 生成的 HAL 处理会吞掉 RXNE 字节。

- 重新生成后 `USART3_IRQHandler` 里的 `HAL_UART_IRQHandler(&huart3);` 会被恢复
- **恢复动作**：把这一行注释掉（函数体里已有提醒注释，照着做即可）
- `vMBPortSerialISR()` 调用本身在 USER CODE 区，重生成保留

## 3. ffconf.h —— FatFs 三项手改配置

CubeMX 生成 FATFS/Target/ffconf.h 时按默认值覆盖，以下三项是手改的：

| 项 | 手改值 | 默认值 | 不改的后果 |
|---|---|---|---|
| `_USE_WRITE` | 1 | 0 | f_write 直接链接错误 |
| `_USE_IOCTL` | 1 | 0 | ioctl(CTRL_SYNC/GET_SECTOR_COUNT) 失效 |
| `_FS_REENTRANT` | 0 | 1 | R0.11 自带 syscall.c 是 CMSIS v1 API，与 FreeRTOS v2 冲突编译报错 |

## 4. Keil 工程结构 —— 自定义源码组与 include 路径

CubeMX 重生成 .uvprojx 时按 .ioc 重建工程，**核对以下仍在**：

- **自定义源码组**：App / Bsp / Divece / FreeModbus / Port 5 个组及其文件
  （丢了 → 器件驱动/协议栈脱离工程 → 编译报"未定义"）
- **include 路径自定义段**：`./app;./Bsp;./Divece;./FreeModbus` 等
  （丢了 → 头文件找不到 → 编译报错）

## 5. 栈大小与金丝雀（核对，一般不用动）

- `.ioc` 已同步 `ProjectManager.StackSize=0x800` → startup 自动 0x800，**不会回退**
- **main.c USER CODE PD 区宏 `STACK_SIZE_BYTES` 必须 = 2048**（与 startup 一致，重生成保留但核对）
- 金丝雀代码全在 USER CODE 区（BEGIN 1 调用点/PV 区 extern/PFP 原型/BEGIN 4 实现），重生成保留

## 6. 其他 CubeMX 不会动的（供确认，无需恢复）

- FreeRTOS heap：`FREERTOS.configTOTAL_HEAP_SIZE=16384` 已写进 .ioc，重生成后
  FreeRTOSConfig.h 保持 16KB，**不会回退**
- 自建任务（debug_cmd / modbus / sd_log）都在 USER CODE 区，重新生成后原样保留
- TIM2（Modbus 3.5T 超时）、SPI2+PB12（SD 卡）、PB14 上拉（SPI2 MISO）都在 .ioc 里，重生成后配置保留
- HAL 心跳是 TIM4（stm32f1xx_hal_timebase_tim.c 的 htim4），所以 TIM4 不能挪作他用
- printf 重定向（usart.c USER CODE BEGIN 1）重生成保留

## 7. 手改文件清单（排查用，重生成均不动）

- `MDK-ARM/App/` 下全部文件（App 分组是手加进 uvprojx 的，重生成不动；含 version.h）
- `MDK-ARM/Bsp/`、`MDK-ARM/Divece/`、`MDK-ARM/FreeModbus/`（器件驱动/协议栈）
- `Core/Src/stm32f1xx_it.c`（仅第 2 条需要恢复）
- `FATFS/Target/user_diskio.c` 的 SD 对接代码全部在 USER CODE 区（已核实），重生成保留，**无需恢复**
- `Boot/` 整个工程（独立手写 Bootloader，与 CubeMX 无关）
- `shared/`、`pc_tool/`（三方共享头、上位机工具）

## 8. 恢复后的验证三步（必做）

1. **重新编译**：Boot + App 两个工程 0 Error 0 Warning；
2. **看 MAP**：STACK 段 2048B、`__initial_sp` 在栈顶、ER_IROM1 基址 = 0x08010200（App）；
3. **烧录冒烟**：启动日志正常 → **Modbus 轮询收发正常**（验证 USART3 接管）→ **OTA 升级走一遍**（`part X valid, fail=00 00`）。

> 变更记录：2026-08-19 补全 IROM1 基址（§1）、自定义组/路径（§4）、栈/金丝雀（§5）、验证三步（§8）。
