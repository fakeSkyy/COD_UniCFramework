# COD_UniCFramework 亮点

STM32H723 机器人固件框架。核心目标是让**换芯片、换 RTOS、换外设**都不必碰应用代码。

以下每条都可以用一条命令验证，命令附在各节。

---

## 1. Vendor 依赖收束到一个翻译单元

整个仓库里，**只有一个 `.c` 文件**同时知道平台层和芯片后端：

```bash
$ grep -rl "impl_stm32_bind.h" --include=*.c --include=*.h 01_application 02_device 03_platform 06_utils
01_application/board/board_devices.c
```

上面三层（应用、设备驱动、平台、工具）**完全不含 HAL**：

```bash
$ grep -rln "stm32h7xx_hal\|stm32f4xx_hal" --include=*.c --include=*.h 01_application 02_device 03_platform 06_utils
(无输出)
```

这不是约定，是可机械检查的性质。一条 grep 返回一个文件，就是这条不变量还成立的证据。

意义：BMI088 驱动、AHRS 滤波器、任务代码在没有 HAL 的环境里也能编译 —— 它们只见到 `PLAT_SPI_TransmitReceive` 这样的中立接口。

---

## 2. 五层，各层职责不重叠

| 层 | 文件 | 代码行 | 职责 |
|---|---|---|---|
| `01_application` | 10 | 1977 | 板级组装 + 任务 |
| `02_device` | 21 | 6554 | 器件驱动（BMI088、DJI/DM 电机、WS2812…） |
| `03_platform` | 27 | 4457 | 中立外设 API（`PLAT_*`） |
| `04_impl` | 60 | 13867 | 后端实现（STM32H7 / STM32F4 / FreeRTOS / RTT） |
| `06_utils` | 27 | 7204 | 无依赖算法（卡尔曼、AHRS、PID、LPF、TD…） |

依赖方向严格单向：`01 → 02 → 03 → 04`，`06` 谁都能用且不依赖任何人。

---

## 3. ops 契约：平台层与后端通过函数表解耦

`04_impl/common/` 下 13 个契约头定义每类外设的 vtable。例如 SPI：

```c
typedef struct
{
    bool (*transmit)(void* ctx, const uint8_t* tx, uint16_t len, uint32_t timeout);
    bool (*transmit_receive_async)(void* ctx, const uint8_t* tx, uint8_t* rx, uint16_t len);
    bool (*cs_assert)(void* ctx);      /* 声明总线归属，多传输事务才能原子 */
    void (*cs_deassert)(void* ctx);
    bool (*is_busy)(void* ctx);
    ...
} SPI_Ops_s;
```

契约头**同时是文档**：每个函数指针旁写清了返回 false 的确切含义、谁负责片选、并发时的行为。写新后端时照着实现即可，不必读现有后端的代码。

平台类共 14 个：

```
bsp:  adc can dma_buf dwt flash gpio iic pwm spi uart
rtos: memory mutex sem task
```

---

## 4. 两个芯片后端完全对称

```bash
$ diff <(grep -o "IMPL_BACKEND_[A-Za-z]*" 04_impl/bsp/stm32h7/impl_stm32_bind.h | sort -u) \
       <(grep -o "IMPL_BACKEND_[A-Za-z]*" 04_impl/bsp/stm32f4/impl_stm32_bind.h | sort -u)
(无差异)
```

两个芯片暴露完全相同的类集合。换芯片就是把 `board_devices.c` 里那一行 include 指向另一个目录：

```c
#include "impl_stm32_bind.h"   /* 全文件唯一提到芯片的地方 */
```

绑定通过宏拼接完成，调用方写的是类名而不是芯片名：

```c
#define IMPL_OPS(Prefix)      IMPL_PASTE(Prefix, _GetOps)()
#define IMPL_CTX(Prefix, ...) IMPL_PASTE(Prefix, _CreateCtx)(__VA_ARGS__)
```

---

## 5. 板级配置是数据，代码从它生成

`board_devices.def` 是一张表，**不是代码**：

```c
BOARD_DEVICE(Timebase,  timebase,   DWT,   SystemCoreClock)
BOARD_DEVICE(ImuAccel,  imu_accel,  SPI,   &hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, SPI_XFER_IT)
BOARD_DEVICE(StatusLed, status_led, SPI,   &hspi6, NULL, 0u, SPI_XFER_IT)
BOARD_DEVICE(DebugUart, debug_uart, UART,  &huart10, UART_XFER_IT)
BOARD_BUS(CAN1, hfdcan1)
```

这一个文件被 include 六次（`board_devices.c` 四次、`board.h` 两次），每次配不同的宏，展开出：静态存储、bring-up 序列、失败上报名、访问器、CAN 总线枚举、句柄查找表。

**加一个外设 = 加一行。** 存储、初始化检查、失败名、`Board_Xxx()` 访问器全部自动生成，无法遗漏。

写错也基本能在编译期抓到，因为每个字段都被粘成真实符号：

```
board_devices.def:183:33: error: unknown type name 'SPl_Instance_s';
                                 did you mean 'SPI_Instance_s'?
```

---

## 6. 构造受编译期管控

所有 `PLAT_*_Init` / `PLAT_*_Create` 都藏在一道门后：

```c
#define PLAT_ALLOW_CONSTRUCTION   /* 只有 board_devices.c 定义它 */
#include "board.h"
```

应用或驱动里误调构造函数是**编译错误**，不是 code review 意见。

全仓库另有 27 处 `_Static_assert`，把"两处必须一致"的事实钉在编译期。例如 CAN 句柄表与枚举同源：

```c
_Static_assert((sizeof handle_of / sizeof handle_of[0]) == (size_t) BOARD_CAN_COUNT,
               "handle table and Board_CANBus_e must come from the same list");
```

---

## 7. 零动态分配（上层）

```bash
$ grep -rn "malloc\|calloc\|free(" --include=*.c 01_application 02_device 03_platform 06_utils
(仅 plat_memory.c 一处，转发给 impl 的 ops)
```

实例存储全部是调用方持有的静态对象：数量在编译期已定，分配不带来任何好处，只多一条与硬件无关的失败路径。

`UTIL_AHRS` 也不分配 —— 卡尔曼矩阵放在调用方给的 buffer 里，长度由 `UTIL_AHRS_BUF_SIZE` 从状态维度推出，不用手工同步。

---

## 8. 任务归属：模块自持，中心只管优先级

`app_tasks.c` 里**没有任何任务** —— 没有栈、没有 body、没有 `PLAT_Task_Create`：

```c
if (!App_Indicator_StartTask(PRIO_INDICATOR)) { ... }
if (!App_Imu_StartTask(PRIO_IMU))             { ... }
```

栈深、周期、循环体都跟着它们服务的工作放在各模块里，因为只有模块知道。

**唯一留在中心的是优先级**，因为它在单个模块内部无法定义 —— "心跳要最先被饿死"是关于*其他任务*的断言；而且它稀缺，`configMAX_PRIORITIES` 只有 7 个格子，所有模块共享。分散选号迟早撞车，而证据会散在两个互不 include 的文件里。

---

## 9. 状态指示灯：图案是数据

一个 WS2812，多个子系统要用。做法不是每个子系统一个任务，而是它们上报**条件**，由指示器决定长什么样：

```c
static const indicator_pattern_s patterns[INDICATOR_CONDITION_COUNT] = {
    [INDICATOR_HEARTBEAT]   = {.g = 80, .flashes = 2, .name = "alive"},
    [INDICATOR_CAN_LOST]    = {.r = 80, .g = 40, .flashes = 3, .name = "CAN lost"},
    [INDICATOR_LOW_BATTERY] = {.r = 80, .g = 20, .flashes = 4, .name = "low battery"},
    [INDICATOR_FAULT]       = {.r = 80, .flashes = 0, .name = "fault"},
};
```

心跳**不是特例**，是同一张表里排名最低的一行。排名就是 enum 顺序，加一个条件 = 一个 enumerator + 一行 designated initializer，漏了会编译失败（表按 `INDICATOR_CONDITION_COUNT` 定长）。

所有图案共享 1 s 拍长，因为拍长是观察者形成预期的单位 —— 每个图案自带周期会让"灯停了"和"灯在显示更慢的东西"无法区分。

上报接口 ISR 安全、幂等、电平触发：

```c
App_Indicator_Set(INDICATOR_CAN_LOST, true);
App_Indicator_SetFault(3u);   /* 闪 3 下 */
```

---

## 10. 注释写的是"为什么"，不是"是什么"

全仓库 34059 行，其中**代码 14236 行，注释与空行 19823 行（58%）**。

这个比例在别处会是坏味道，这里不是 —— 注释内容主要是**硬件事故记录**，也就是"改这一行之前必须知道的事"。例：

> **SPI2 预分频器必须是 32 而不是 8。** BMI088 上限 10 MHz，内核时钟 240 MHz，所以 32 给出 7.5 MHz，而 8 给出 24 MHz，超限 2.4 倍。
>
> 这个故障不像时钟问题：陀螺仪照样答对 `0x0F`，加速度计返回一个**稳定且合理**的 `0x23`（正确值 `0x1E`），重试结果一致，不像噪声。每次 SPI 调用都报成功，只有数据是错的。两个 die 的差别在于加速度计每次读多插一个 dummy byte，需要多一次总线翻转才能撑住。
>
> **所以"一个器件答对了"不能证明总线速率合法。**

同类记录还有：DMA1/DMA2 无法访问 DTCM（`.bss` 在那里）所以遥测只能用中断模式；CYCCNT 在 550 MHz 下每 7.8 s 回绕（F407 是 25.6 s）所以时间线重建必须更频繁地轮询；SPI6 的三个 CubeMX 参数（HSE 24 MHz、/4 预分频、Data Size 8）是承重的且都不会报错。

---

## 11. 构建：CubeMX 与手写部分结构性分离

根 `CMakeLists.txt` **include** vendor 的 CMake，而不是复制它的源文件列表：

```cmake
set(VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/05_vender/stm32cubemx)
add_subdirectory(${VENDOR_DIR}/cmake/stm32cubemx)
```

在 CubeMX 里启用一个外设，构建自动跟上，这个文件不用动。之前的做法是根目录一个手写 Makefile 同时列出两边的源文件，而一次 Generate Code 静默替换了 vendor 那半、把框架源文件列表一起带走了 —— 这就是现在这个结构的由来。

构建守卫会区分三种失败原因并分别报告：链接脚本路径没被重定向、脚本文件不存在、配置时漏了 `-DCMAKE_TOOLCHAIN_FILE`（最后这条会导致 flags 为空，是最容易误诊的一个）。

当前状态：

```
   text    data     bss     dec filename
 148040     348   69320  217708 COD_UniFramework_H7.elf

FLASH   14.15% of 1 MB      DTCMRAM  53.16% of 128 KB
零 warning（CFLAGS 带 -Wall）
```

---

## 12. 算法库不依赖硬件

`06_utils` 下 14 个模块只用 `float`、只 include 自己和 `<math.h>`，可以在 host 上编译测试：

```
util_ahrs        姿态解算（四元数 + 陀螺零偏的卡尔曼滤波）
util_kf          通用卡尔曼，带新息门控和自动复位
util_lpf         一阶 / 二阶 biquad（TDF-II），带 Jury 稳定性判据
util_td          Han 跟踪微分器（ADRC 组件）
util_pid / util_rls / util_maf / util_fast_math / util_crc / ...
```

`util_ahrs` 已在硬件上验证：把加速度计原始值独立解算出的姿态与滤波器输出对比，**roll/pitch 差 0.1° 以内**（-0.90° / -4.26° 对 -0.98° / -4.13°），异常计数器全为 0。

---

## 已知边界

诚实记录，避免误导：

- **yaw 无界漂移**，实测约 3.8°/min。没有磁力计，没有任何东西观测绕重力轴的旋转，所以 yaw 是纯陀螺积分。roll/pitch 有重力做参考，不漂。
- **蜂鸣器在 TIM12 是推断，不是原理图**。依据是 TIM12 的 CubeMX 配置形状像音调发生器（预分频 0、周期 20999，即整个计数范围留给频率改写），而 TIM3 是固定频率、变占空比的形状。上板前需对着板子确认。
- **STM32F4 后端未在硬件上回归**。代码对称、能编译，但当前只有 H7 板子。
- **`ref/` 是重构前的整棵旧代码树**（`algorithm/`、`application/`、`bsp/`、`components/`），不在 `CMakeLists.txt` 里，仅供参考。
