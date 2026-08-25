# FreeRTOS 移植记录

STM32H723VGTx / arm-none-eabi-gcc 15.2 / 上游 FreeRTOS-Kernel V11.3.0

> 本文最初写于 F407 时期。项目后来移植到 STM32H723VGTx + CMake,**内核来源、目录划分、
> 组件取舍、配置理由全部不变**,变化只有三处:port 从 `ARM_CM4F` 沿用到 M7(上游对
> r0p1 之后的版本正是这样推荐,本片 r1p2)、文件名 `stm32h7xx_it.c` → `stm32h7xx_it.c`、
> 构建命令 `make` → `cmake --build build`。文中已就地更新;凡提到 F407 的地方是在讲
> 历史对照。

本文记录 2026-08-07 把 FreeRTOS 从 CubeMX 托管改为自行移植的完整过程:做了什么、为什么这么做、以及如何自行复核。

---

## 1. 出发点

改造前的状态:

- 内核是 CubeMX 随 STM32Cube_FW_F4_V1.28.3 分发的 **V10.3.1**(2020 年),位于 `05_vender/Middlewares/Third_Party/FreeRTOS/`。
- `FreeRTOSConfig.h` 由 .ioc 生成。除 `USER CODE` 标记内的区域外,手改的值会在下次 Generate Code 时被还原。
- 任务通过 **CMSIS-RTOS v1**(`cmsis_os.c`)创建,三个任务体是 `__weak` 存根,内容为 `for(;;) osDelay(1)`,仓库里没有任何强符号覆盖它们。
- 框架自己的 `PLAT_Task_*` / `PLAT_Mutex_*` 抽象**没有任何调用点**,`--gc-sections` 把它们整个从 ELF 里丢掉了。

也就是说:RTOS 在运行,但什么也没做;框架的任务抽象从未被执行过。

### 促成改造的两个具体问题

**配置无法持久。** 两个错误检测开关取的是 FreeRTOS 的默认值 0,而不是有意的选择:

| 开关 | 改造前 | 后果 |
|---|---|---|
| `configCHECK_FOR_STACK_OVERFLOW` | 0 | 栈溢出不检测 |
| `configUSE_MALLOC_FAILED_HOOK` | 0 | 分配失败无回报 |

栈溢出这条对本框架格外要紧:`PLAT_Task_Create` 的设计是由调用方传入栈数组,**栈大小是该 API 唯一无法自行校验的参数**。写错时没有任何征兆,只是静默覆盖相邻的 RAM。

**CMSIS v1 是一层与框架竞争的抽象,且自身有缺陷。** `cmsis_os.h:287` 写:

```c
uint32_t stacksize;    ///< stack size requirements in bytes; 0 is default stack size
```

而 `osThreadCreate` 把它原样传给 `xTaskCreateStatic`,后者的单位是**字**:

```c
handle = xTaskCreateStatic(..., thread_def->stacksize, ...);
```

文档与实现差 4 倍。照文档写 `osThreadStaticDef(..., 1024, buf, ...)` 配 `uint32_t buf[256]`,就是用 1024 字节的缓冲申请 4096 字节的栈 —— 溢出 3 KB,且检测是关的。

框架的 `plat_task.h:77` 早已识别并封掉了这个坑(以字节为单位,由知道字宽的后端负责换算)。同一项目里两套任务 API,只有走 CMSIS 那条路的不遵守自己的规范。

另外这是 CMSIS-RTOS **v1**,ARM 已废弃(现行 v2);`main.c:118` 的注释甚至还写着 "in cmsis_os2.c",模板都对不上了。

---

## 2. 内核来源与校验

### 2.1 来源

```
仓库:   https://github.com/FreeRTOS/FreeRTOS-Kernel
Tag:    V11.3.0
Commit: 9b777ae5c5b8e9e456065a00294d1e5f5f9facf5
```

选 **tag `V11.3.0` 而非 `main` 分支**:当时 main 的 HEAD 是 `V11.1.0+`(开发态版本号),tag 才是发布点。

### 2.2 如何自行复核

内核目录内有 `MANIFEST.sha256`,记录了全部 27 个文件的校验和。三种独立方式,任选:

**方式一 —— 校验和自检(最快,不需要网络):**

```bash
cd 05_vender/freertos && sha256sum -c MANIFEST.sha256
```

**方式二 —— git blob 哈希对比上游(不需要下载内核):**

git 的 blob 哈希只取决于文件内容,所以本地算出的值必须与上游 tag 记录的一致。

```bash
git hash-object 05_vender/freertos/tasks.c
# 2b7d31198c8cddef001cac620a1378766c0b2117
```

与上游对照:

```bash
git ls-remote https://github.com/FreeRTOS/FreeRTOS-Kernel.git refs/tags/V11.3.0
# 或克隆后:git rev-parse V11.3.0:tasks.c
```

六个源文件的哈希(改造时实测,上游与本仓库完全一致):

| 文件 | blob 哈希 |
|---|---|
| `tasks.c` | `2b7d31198c8cddef001cac620a1378766c0b2117` |
| `queue.c` | `905dbd942f8ee8bddf37f0cf5f17181e8434a5a9` |
| `list.c` | `5cdc8629634908ce1930222373ff6b9c8e5ccf99` |
| `portable/GCC/ARM_CM4F/port.c` | `3d82e198b94c3b79501aff38b7e21ef2dbc91320` |
| `portable/GCC/ARM_CM4F/portmacro.h` | `d1f1a40a6032a4129803e355ea6ee7874d5b480b` |
| `portable/MemMang/heap_4.c` | `a2b93aff1913c21663d610a0cb1105422da1e4f8` |

**方式三 —— 独立重新下载并逐字节 diff:**

```bash
cd /tmp && curl -sSL -o v11.tar.gz \
  https://codeload.github.com/FreeRTOS/FreeRTOS-Kernel/tar.gz/refs/tags/V11.3.0
tar xzf v11.tar.gz
D=/tmp/FreeRTOS-Kernel-11.3.0
K=<仓库>/05_vender/freertos
for f in tasks.c queue.c list.c \
         portable/GCC/ARM_CM4F/port.c portable/GCC/ARM_CM4F/portmacro.h \
         portable/MemMang/heap_4.c; do
  diff -q "$D/$f" "$K/$f" && echo "IDENTICAL $f"
done
diff -r "$D/include" "$K/include" && echo "IDENTICAL include/"
```

改造时用上述三种方式全部验证过,结果一致:**内核源码零修改**。参考:V11.3.0 tarball 的 sha256 为
`76530a6bab55233e34e07c8df59f0d4c2e06db473763f8d33277d4fa10950084`(GitHub 生成的 tarball 理论上不保证长期字节稳定,所以这个值仅供参考,权威依据是上面的 blob 哈希)。

### 2.3 内核源码未作任何修改

移植**没有**改动任何一行内核代码。全部适配都在内核之外完成:配置宏、链接期解析的钩子函数、以及异常处理器改名。这是刻意的 —— 内核保持 pristine,以后跟版本就是替换目录再重跑校验,不需要重新施加补丁。

顺带一提,ST 分发的 V10.3.1 其实也没改过内核源码(此前查证过,`tasks.c` 里看似 ST 相关的匹配全是 `listGET_OWNER_OF_HEAD_ENTRY` 之类宏名里的 `ST_` 子串)。所以换成上游买到的是版本自由与配置所有权,不是"摆脱 ST 的私自改动"。

### 2.4 组件取舍

上游内核是一组可选模块,不是一个整体。下面是本次实际取了什么、排除了什么。

#### 已移植

| 组件 | 文件 | 状态 | 说明 |
|---|---|---|---|
| 任务与调度器 | `tasks.c` | 编译进镜像 | 核心,不可选 |
| 队列 / 信号量 / 互斥量 | `queue.c` | 编译进镜像 | `impl_mutex.c` 依赖它 —— 信号量是队列的特例 |
| 链表 | `list.c` | 编译进镜像 | 内核数据结构,不可选 |
| Cortex-M4F port | `portable/GCC/ARM_CM4F/{port.c,portmacro.h}` | 编译进镜像 | 上游 59 个 GCC port 里只取这一个 |
| 堆管理 | `portable/MemMang/heap_4.c` | 编译进镜像 | 5 个变体里只取 heap_4(合并相邻空闲块) |
| 全部头文件 | `include/` 21 个 | 全量装入 | 与上游逐字节一致,一个没删 |
| 任务通知 | (在 `tasks.c` 内) | 启用 | `configUSE_TASK_NOTIFICATIONS 1`,`PLAT_Task_Notify` / `PLAT_Task_Wait` 的底座 |
| 优先级继承互斥量 | (在 `queue.c` 内) | 启用 | `configUSE_MUTEXES 1` |
| 静态分配 | — | 启用 | `PLAT_Task_Create` / `PLAT_Mutex_Init` 都走静态 |

`include/` 是**全量**装入的,所以头文件层面什么都不缺 —— 日后要用软件定时器,不需要回上游翻 `timers.h`。

#### 未移植

| 组件 | 上游文件 | 排除方式 | 为什么 |
|---|---|---|---|
| 软件定时器 | `timers.c` | 源码未装 + `configUSE_TIMERS 0` | 框架用 `PLAT_Task_DelayUntil` 做周期、DWT 做计时 |
| 事件组 | `event_groups.c` | 源码未装 + `configUSE_EVENT_GROUPS 0` | 无调用点 |
| 流 / 消息缓冲 | `stream_buffer.c` | 源码未装 + `configUSE_STREAM_BUFFERS 0` | 框架用 `util_ringbuf` 和 `util_msgbus` |
| 协程 | `croutine.c` | 源码未装 + `configUSE_CO_ROUTINES 0` | 已废弃的遗留特性 |
| 其余 4 个堆变体 | `heap_1/2/3/5.c` | 未装 | 一个构建只能有一个分配器 |
| 其余 58 个 GCC port | `portable/GCC/*` | 未装 | 其他架构 |
| 其他编译器 port | `IAR/`、`ARMClang/`、`Keil/` 等 22 个目录 | 未装 | 本项目用 arm-none-eabi-gcc |
| MPU 保护 | `portable/GCC/ARM_CM4_MPU/` | 未装 + `configENABLE_MPU 0` | 见下 |
| CMSIS-RTOS 壳 | `CMSIS_RTOS/cmsis_os.c` | **主动删除** | 本次改造的目的之一,见第 1 节 |

#### 三点说明

**这不是"功能缺失",是让配置与构建对齐。** 那四个可选模块在原 vendor 构建里是**编译了再被 `--gc-sections` 整个丢弃**(改造前逐个验证过代表符号:`xTimerCreate`、`xEventGroupCreate`、`xStreamBufferSend`、`vCoRoutineSchedule` 全部 dropped)。所以不装它们**不省 Flash,只省编译时间** —— 别把这当性能收益。真正的收益是配置文件如实描述镜像内容。

**`configUSE_EVENT_GROUPS` / `configUSE_STREAM_BUFFERS` 是 V11 新增的开关,默认值都是 1。** 初次移植时漏了这两项,于是配置声称有这两个功能而源码根本没装。链接能过(无任何引用,`nm -u` 确认),属于会让配置文件逐渐失去参考价值的漂移。已显式置 0;重建后镜像尺寸**一字节未变**(63744 / 284 / 57112),反证了它们本来就不在镜像里。

**要加回某个模块,两件事都得做**:把源文件加进 Makefile 的 `FRAMEWORK_C_SOURCES`,并把对应开关改成 1。只改一处的结果是链接错误,或者配置骗人。

#### 关于 MPU

F407 确实有 MPU(`stm32f407xx.h:48` 的 `__MPU_PRESENT 1`),8 个区域。它按区域规定读写/执行权限,越权访问立刻进 `MemManage_Handler`,`MMFAR` 里留着出错地址 —— 也就是把"错误的传播"变成"错误的定位"。在 FreeRTOS 下主要用于让每个任务只能访问自己的栈,以及把代码段设只读、数据段设 XN。

本次没有启用,四个原因:

1. **区域预算不够。** MPU port 自己就要占 5 个 region(`portmacro.h:183`:`portSTACK_REGION` = `configTOTAL_MPU_REGIONS - 5`),8 减 5 只剩 3 个可配置。本板有 SPI、两路 CAN、三路 UART、TIM、I2C、Flash,每个外设区域都要占一个,3 个不够分。
2. **要换 port 且 API 不同。** 得从 `ARM_CM4F` 换成 `ARM_CM4_MPU`,任务改用 `xTaskCreateRestricted` 并逐个声明允许访问的内存区域 —— 这意味着**重写 `PLAT_Task_Create`**,让平台层接口多出"任务能碰哪些内存"这一整个概念,而那是与 RTOS 强绑定的,违背 `plat_task.h` 当前的设计意图。
3. **一半收益已经拿到。** `configCHECK_FOR_STACK_OVERFLOW 2` 和 `configUSE_MALLOC_FAILED_HOOK` 已开启,栈溢出与堆耗尽都会带任务名停机。MPU 的增量主要是"实时拦截"而非"能否发现"。
4. **这套 RTOS 代码尚未上板。** 见第 6 节。此时引入 MPU 等于让首次 bringup 同时调试两个未验证的系统,而 MPU 配错的典型症状(启动即 MemManage)与调度器本身故障难以区分。

若日后确实需要内存保护,成本低得多的两条路:把 `HardFault_Handler` 改成解析 `SCB->CFSR` / `HFSR` / `BFAR` 并报出错地址(不需要 MPU);或者只用 CMSIS 的 `ARM_MPU_SetRegion` 保护单个区域(例如把 Flash 设只读),占 1–2 个 region,不动 port 也不改 `PLAT_Task_Create`。

---

## 3. 目录结构

内核与自有适配代码**分处两层**,这是刻意的。

```
05_vender/                          厂商拥有,可被整体替换
├── stm32cubemx/                    CubeMX 生成,Generate Code 会重写
│   ├── Makefile  *.ioc  *.ld
│   └── Core/  Drivers/  USB_DEVICE/  Middlewares/ST/
├── freertos/                       上游 V11.3.0 pristine,勿手改
│   ├── MANIFEST.sha256             ← 来源与校验和
│   ├── VERSION                     ← "V11.3.0"
│   ├── LICENSE.md
│   ├── tasks.c  queue.c  list.c
│   ├── include/                    (21 个头文件)
│   └── portable/
│       ├── GCC/ARM_CM4F/{port.c,portmacro.h}
│       └── MemMang/heap_4.c
└── segger_rtt/                     上游 V8.58.0 pristine,同一套规矩
    ├── MANIFEST.sha256   VERSION   LICENSE.md
    └── RTT/                        RTT.c/.h、printf、ConfDefaults、ASM

04_impl/rtos/freertos/              本项目自有(RTOS 侧)
├── FreeRTOSConfig.h                配置,不受 CubeMX 支配
├── rtos_hooks.c                    内核要求的回调 + SysTick_Handler
├── rtos_fault.c                    四个 fault 处理器,解码 SCB 并经 RTT 上报
├── rtos_tasks.c  rtos_tasks.h      任务声明与调度器启动
├── PORTING.md                      本文
├── task/impl_task.c                PLAT_Task 后端
├── mutex/impl_mutex.c              PLAT_Mutex 后端
├── sem/impl_sem.c                  PLAT_Sem 后端
└── memory/impl_memory.c            PLAT_Memory 后端

04_impl/bsp/segger_rtt/             本项目自有(RTT 侧)
└── SEGGER_RTT_Conf.h               只写与上游默认值不同的项
```

**为什么这样切分。** `05_vender` 的语义是"别人的代码,可整体替换" —— 它的 CubeMX 那一半已经被重新生成、连带毁掉过一次根 Makefile。内核放进去符合这个语义(上游发布物,升级即换目录);而 `FreeRTOSConfig.h` 和钩子是**本项目做的决定**,放进 vendor 会模糊这条界线,升级内核时还得先从旧树里把它们抢救出来。

分开之后,升级内核 = 替换 `05_vender/freertos/` + 重跑校验,没有任何自有文件需要抢救。SEGGER RTT 后来按同一套规矩整理:`05_vender/segger_rtt/` 放上游 V8.58.0,`04_impl/bsp/segger_rtt/SEGGER_RTT_Conf.h` 放本项目的覆盖项。

**为什么适配代码落在 `04_impl/rtos/freertos/`** 而不是单独一层:它与 `impl_task.c` / `impl_mutex.c` / `impl_memory.c` 是同一件事的两面 —— 都是"把 FreeRTOS 接到 `PLAT_*` 抽象背后"。放在一起,换 RTOS 时要改的东西全在一个目录内。这也让 `03_platform` 与 `04_impl` 的结构同构:

```
03_platform/            04_impl/
├── bsp/                ├── bsp/
│   ├── can/ spi/ ...   │   └── stm32f4/
└── rtos/               │       └── can/ spi/ ...
    ├── task/           └── rtos/
    ├── mutex/              └── freertos/
    └── memory/                 └── task/ mutex/ memory/
```

第一级是**能力类别**(bsp / rtos),第二级才是具体后端。加第二颗 MCU 或换 RTOS 时,结构不用动。

---

## 4. 移植步骤

### 步骤 0:在 CubeMX 中关闭 FreeRTOS

在 .ioc 里 Middleware → FreeRTOS 设为 Disabled,然后 Generate Code。

**要留意它会带走什么。** 实测结果:

| 文件 | 结果 |
|---|---|
| `05_vender/Core/Inc/FreeRTOSConfig.h` | **删除** |
| `05_vender/Core/Src/freertos.c` | **删除** |
| `05_vender/Makefile` 里的内核源码条目 | 删除 |
| `05_vender/Makefile` 里的 FreeRTOS `-I` 路径 | **残留 3 行** |
| `05_vender/Middlewares/.../FreeRTOS/` 源码树 | 保留 |
| `stm32h7xx_it.c` | **新增三个空的内核异常处理器** |
| `main.c` 的 `MX_FREERTOS_Init()` / `osKernelStart()` | 删除 |

`FreeRTOSConfig.h` 被删这一点值得注意:如果打算保留原配置作为起点,**先备份**。本次是重写而非复制。

### 步骤 1:取得内核并装入

```bash
git clone --depth 1 --branch V11.3.0 \
    https://github.com/FreeRTOS/FreeRTOS-Kernel.git /tmp/frtos
cd <仓库>
mkdir -p 05_vender/freertos/portable/GCC/ARM_CM4F \
         05_vender/freertos/portable/MemMang \
         04_impl/rtos/freertos

U=/tmp/frtos; K=05_vender/freertos
cp $U/tasks.c $U/queue.c $U/list.c $K/
cp -r $U/include $K/include
cp $U/portable/GCC/ARM_CM4F/port.c $U/portable/GCC/ARM_CM4F/portmacro.h \
   $K/portable/GCC/ARM_CM4F/
cp $U/portable/MemMang/heap_4.c $K/portable/MemMang/
cp $U/LICENSE.md $K/
echo "V11.3.0" > $K/VERSION
```

然后生成校验清单(内容见 `MANIFEST.sha256` 头部注释)。

### 步骤 2:检查 V10 → V11 的破坏性变更

移植前必须先确认自己实际调用的 API。本项目 impl 层用到的:

| API | V11 状态 |
|---|---|
| `xTaskCreateStatic` | 签名不变 |
| `ulTaskNotifyTake` / `xTaskNotifyGive` / `vTaskNotifyGiveFromISR` | 保留 |
| `xSemaphoreCreateMutexStatic` / `xSemaphoreTake` / `xSemaphoreGive` | 保留 |
| `xTaskGetSchedulerState` / `xTaskGetCurrentTaskHandle` / `xTaskGetTickCount` | 保留 |
| `pvPortMalloc` / `vPortFree` | 保留 |
| `vTaskDelayUntil` | **被 `xTaskDelayUntil` 取代**(旧名保留为兼容宏) |
| `xPortIsInsideInterrupt` | 保留(`portmacro.h:191`) |

结构体尺寸也要核 —— 平台层用 `_Static_assert` 卡了上界:

| 类型 | V10.3.1 | V11.3.0 | 平台层上界 |
|---|---|---|---|
| `StaticTask_t` | 100 B | **84 B** | `PLAT_TASK_TCB_BYTES` = 128 |
| `StaticSemaphore_t` | 72 B | 72 B | `PLAT_MUTEX_STORAGE_BYTES` = 96 |

V11 反而更小,两个断言继续成立,`plat_*.h` 无需改动。

新增的必需配置项:`configNUMBER_OF_CORES`(V11 加了 SMP 支持)、`configTICK_TYPE_WIDTH_IN_BITS`(取代废弃的 `configUSE_16_BIT_TICKS`)。

### 步骤 3:写 `FreeRTOSConfig.h`

完整文件在 `04_impl/rtos/freertos/FreeRTOSConfig.h`,每一项都带选择理由。关键几处:

**必须开的两个错误检测:**

```c
/* Mode 2:创建时用已知字节填充整个栈,每次上下文切换检查末尾 20 字节,
 * 在 mode 1 的栈指针越界测试之上。mode 1 单独用会漏掉"溢出发生后又
 * 退回来"的情况,而深调用链正是这种。 */
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK   1
```

**`configASSERT` 要能留下线索:**

```c
/* CubeMX 的默认是 taskDISABLE_INTERRUPTS(); for(;;);  —— 正确,但关中断
 * 死循环、不记录任何东西,所以每一次内核断言从外面看都一样:板子没反应。
 * RTT 已经链接在内,文件名和行号值得这两条指令。 */
#define configASSERT(x)  if ((x) == 0) { RTOS_AssertFailed(__FILE__, __LINE__); }
```

`RTOS_AssertFailed` 在 `rtos_hooks.c` 实现。注意它**先写 RTT 再关中断** —— RTT 写的是调试器轮询的控制块,中断已经关掉的话,半条消息就是永远的最后一条。停机时会看到形如:

```
*** FreeRTOS assert: tasks.c:2319 ***
```

(`tasks.c:2319` 是 V11.3.0 里 `xTaskDelayUntil` 的 `uxSchedulerSuspended == 0` 断言,举例而已 —— 行号随版本变化。)

**中断优先级(此处几乎没有余量):**

```c
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
```

CubeMX 把本项目每个外设中断都设成 5,所以全部刚好合法。**余量为零**:任何一个中断提到 4 就不能再调 `PLAT_Task_Notify`,`vPortValidateInterruptPriority` 会在第一次尝试时停机。

**异常处理器改名(见步骤 4):**

```c
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
/* SysTick 故意不映射 —— 原因见步骤 4 */
```

### 步骤 4:接三个内核异常处理器

这是最容易静默出错的一步。

port 用自己的名字实现三个异常(`vPortSVCHandler` / `xPortPendSVHandler` / `xPortSysTickHandler`),而 `startup_stm32h723xx.s` 的向量表引用的是 CMSIS 名字(`SVC_Handler` 等)。同时 CubeMX 在关闭 FreeRTOS 后**在 `stm32h7xx_it.c` 里生成了三个同名空函数**。

**SVC 与 PendSV —— 用宏改名,并从 `stm32h7xx_it.c` 删除:**

两者都是 `__attribute__((naked))`,以异常方式进入,要求 LR 保存 EXC_RETURN、栈指针与 CPU 交接时完全一致。C 包装函数的 prologue 会同时破坏这两个前提,**所以不能像 SysTick 那样转发**,只能靠 `FreeRTOSConfig.h` 里的宏改名。

对应地,`stm32h7xx_it.c` 里那两个函数必须删掉,并留下说明:

```c
/* NOTE — SVC_Handler and PendSV_Handler are deliberately absent from this file.
 * ...
 * CubeMX generates these two whenever FreeRTOS is disabled in the .ioc, and it
 * owns the function bodies here, so a Generate Code will put them back. That
 * shows up as a "multiple definition of SVC_Handler" link error, which is the
 * good outcome — delete them again when it happens. It cannot fail silently. */
```

这一点要记牢:**下次 Generate Code 会让链接失败**,报 `multiple definition`。这是好结果 —— 删掉再编即可,不会静默出错。

**SysTick —— 用包装转发,定义在框架侧的 `rtos_hooks.c`:**

`xPortSysTickHandler` 是普通函数,可以安全调用。转发而非改名的好处是留了加守卫的位置(调度器未启动时不能调它)。

> **H7 移植时这里改过位置。** F407 时期它放在 `stm32f4xx_it.c` 的 `USER CODE` 区,理由是"能活过 Generate Code"。H7 的重新生成**直接产出了一个完全没有 `SysTick_Handler` 的 `it.c`** —— USER CODE 区连同它所在的函数一起消失了,于是只剩 startup 文件里的弱符号 `Default_Handler`(死循环)。内核 tick 永远不会到达,调度器启动后什么都不会跑。
>
> 所以现在它定义在 `04_impl/rtos/freertos/rtos_hooks.c`。USER CODE 区只能保住"函数体内的内容",保不住"函数本身存在"。放在框架侧,再生成碰不到;而如果哪天 CubeMX 又生成一个,链接会报重复定义 —— 响亮,且指向正确的文件。

```c
/* rtos_hooks.c */

/* port.c 定义了它但没有任何 FreeRTOS 公共头导出 —— port 期望
 * FreeRTOSConfig.h 把它改名成 SysTick_Handler,而本项目不这么做。 */
void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}
```

守卫是必要的:SysTick 只在内核启用后才响,但 `xPortSysTickHandler` 会进临界区并操作 tick 链表,而这两者在 `vTaskStartScheduler` 之前都不存在。

**为什么 SysTick 不像另外两个那样改名** —— 本项目的 HAL 时基是 **TIM2**(CubeMX 特意这样配,让 `HAL_Delay` 工作在内核不屏蔽的优先级上)。内核仍需要 SysTick 异常驱动自己的 tick,并且**在 `vPortSetupTimerInterrupt` 里自己配置 SysTick 外设**。所以 SysTick 完全归 FreeRTOS,TIM2 完全归 HAL,互不冲突;改名反而会把内核处理器放到它自己的初始化代码没预期的位置。

**验证向量表** —— 这一步做完必须查,否则错了不会有任何编译期迹象:

```bash
arm-none-eabi-objdump -s -j .isr_vector build/COD_UniFramework_H7.elf | head -8
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -wE 'SVC_Handler|PendSV_Handler|SysTick_Handler'
```

偏移 `0x2C` = SVC(#11)、`0x38` = PendSV(#14)、`0x3C` = SysTick(#15),小端。三项都应指向 `nm` 报的地址。

### 步骤 5:写内核要求的钩子

FreeRTOS 靠**名字在链接期**解析这些回调,没有注册调用 —— 所以少一个是 undefined reference,签名写错是静默不匹配。全部在 `rtos_hooks.c`:

| 函数 | 由什么开关要求 |
|---|---|
| `vApplicationGetIdleTaskMemory` | `configSUPPORT_STATIC_ALLOCATION` |
| `vApplicationStackOverflowHook` | `configCHECK_FOR_STACK_OVERFLOW` |
| `vApplicationMallocFailedHook` | `configUSE_MALLOC_FAILED_HOOK` |
| `RTOS_AssertFailed` | 本项目的 `configASSERT` |

`configUSE_TIMERS` 为 0,所以不需要 `vApplicationGetTimerTaskMemory`。

三个失败钩子都经 RTT 报告后停机。这正是打开这些开关的意义 —— 否则每种情况都表现为"板子不动了",无从区分。

### 步骤 6:建立真实任务,替换 CMSIS

> **本节已过时(2026-08-11)。** `rtos_tasks.c` 已不存在:任务清单搬到
> `01_application/tasks/app_tasks.c`,启动调度器改用 `PLAT_Task_StartScheduler()`。
> 那个 `vTaskStartScheduler` 调用曾是该文件唯一的 vendor 符号,也是它留在
> `04_impl` 的唯一理由。入口 `RTOS_StartTasks()` 随之改名 `App_StartTasks()`,
> 三个示例应用一并删除,app 层现在只剩脚手架。
>
> 保留下文,因为它记录的"为什么必须替换 CMSIS-RTOS"仍然成立。

`rtos_tasks.c` 取代 CubeMX 的 `freertos.c`。要点:

- 任务用 **`PLAT_Task_Create`** 创建,项目里只有一套任务 API。栈以**字节**为单位传入,由后端换算成字。
- 除 `vTaskStartScheduler` 外**不出现任何 FreeRTOS 名字**。任务体只调 `App_*` 和 `PLAT_*`,所以换 RTOS 只需重写 impl 层,不动这个文件。
- 任务体不返回。初始化失败时 `vTaskSuspend(NULL)` 挂起自己,而不是返回 —— 返回等于自我删除,port 未必支持。

`main.c` 只加一行,在 `USER CODE` 区内:

```c
	if (!RTOS_StartTasks())
	{
		Error_Handler();
	}
```

### 步骤 7:改根 Makefile

```make
KERNEL_DIR    = 05_vender/freertos
RTOS_IMPL_DIR = 04_impl/rtos/freertos

FRAMEWORK_C_SOURCES = \
$(KERNEL_DIR)/tasks.c \
$(KERNEL_DIR)/queue.c \
$(KERNEL_DIR)/list.c \
$(KERNEL_DIR)/portable/GCC/ARM_CM4F/port.c \
$(KERNEL_DIR)/portable/MemMang/heap_4.c \
$(RTOS_IMPL_DIR)/rtos_hooks.c \
$(RTOS_IMPL_DIR)/rtos_tasks.c \
...

FRAMEWORK_C_INCLUDES = \
-I$(RTOS_IMPL_DIR) \
-I$(KERNEL_DIR)/include \
-I$(KERNEL_DIR)/portable/GCC/ARM_CM4F \
...
```

`config` 目录必须在内核 `include` **之前**没有硬性要求(两者无同名文件),但先放配置目录符合"配置优先"的直觉。

---

## 5. 顺带修掉的 impl 层问题

移植过程中,V11 的 API 变化和新的 config 让四处此前发现的问题一并解决:

**`taskSCHEDULER_SUSPENDED` 未处理**(此前审计中唯一会挂死的路径)。两处判断只比了 `NOT_STARTED`,而 `xTaskGetSchedulerState` 有三个返回值:

```c
- return xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED;
+ return xTaskGetSchedulerState() != taskSCHEDULER_RUNNING;
```

调度器挂起时旧代码返回"可以阻塞",而 `xQueueSemaphoreTake` 对 `(SUSPENDED && timeout != 0)` 断言、`xTaskDelayUntil` 对 `uxSchedulerSuspended != 0` 断言 —— 配上关中断死循环的 `configASSERT` 就是停机。当前不可达(仓库里没有 `vTaskSuspendAll` 调用;heap_4 内部会挂起,但那个窗口里不会回调框架代码),属于潜伏缺口。

**`task_notify` 里一个基于错误理解的守卫。** 原注释称调度器启动前 `pxCurrentTCB` 为 NULL 会解引用 —— 机制是错的:`prvAddNewTaskToReadyList` 创建第一个任务时就设了 `pxCurrentTCB`,且那处比较在"目标正阻塞于通知"分支内,调度器未跑时进不去。而守卫本身在**丢通知**,与 `plat_task.h:129` 承诺的 "a notification to a task that is not waiting is remembered" 矛盾。守卫已删除。

**`task_delay_until` 缺溢出守卫。** 同文件的 `ticks_of` 专门防了 `pdMS_TO_TICKS` 的 32 位回绕,该函数却直接调用 —— `period_ms` > 4294967 会绕成 0,再被 `if (period == 0)` 抬成 1 ms。已补齐。

**超期判定改用内核返回值。** V10.3.1 只有 void 的 `vTaskDelayUntil`,所以此前得在调用前手算 `(xTaskGetTickCount() - cursor) < period` 来推断。V11 的 `xTaskDelayUntil` 直接返回是否延时:

```c
BaseType_t delayed = xTaskDelayUntil(&cursor, period);
return delayed != pdFALSE;
```

注意 cursor 的语义**没变**,仍是每次固定推进一个周期、从不设为当前 tick:超期 N 个周期的循环会连续 N 次立即返回并报 false,直到追上现在。1 kHz 循环卡了 50 ms 就是 49 次无延时迭代。需要避免突发动作的调用方必须处理这个返回值。

---

## 6. 验证结果

```
cmake --build build --clean-first -j16    零 warning 零 error
Flash                       63,744 B text + 284 B data
RAM                         57,112 B bss / 128 KB
```

**平台层首次进入 ELF** —— 这是本次改造最实质的产出:

```
0800d864 T PLAT_Task_Create        ← 改造前被 --gc-sections 全部丢弃
0800bd3c T RTOS_StartTasks
0800aeec T xTaskDelayUntil
0800bc48 T vApplicationStackOverflowHook
```

CMSIS 符号(`osThreadCreate` / `osDelay` / `osKernelStart`)已从 ELF 中彻底消失。

向量表三项经 `objdump` 核对无误。clangd 对新旧六个文件全部 0 error。

### 未验证的部分

**整个 RTOS 从未在硬件上运行过。** 调度器启动、上下文切换、两个任务的实际时序、栈是否够用 —— 全部只有静态与编译期证据。

首次烧写时预期在 RTT 上看到:

```
RTOS: starting scheduler (FreeRTOS V11.3.0)
motor: online          (或 OFFLINE)
```

若停住不动,现在断言会给出文件与行号,而不是此前那样毫无输出。

栈大小是猜的(控制环 2 KB、监控 1 KB)。上板后用 `uxTaskGetStackHighWaterMark`(已在 config 里开启)实测再调,不要继续沿用估值。

---

## 7. 后续维护

### 升级内核

1. 从上游取新 tag,按步骤 1 覆盖 `05_vender/freertos/`。
2. 重新生成 `MANIFEST.sha256`,更新 `VERSION`。
3. 按步骤 2 的表逐项核 API 与结构体尺寸 —— 尤其两个 `_Static_assert` 的上界。
4. `cmake --build build --clean-first -j16`,要求零 warning。

因为内核未打任何补丁,这个流程不需要重新施加改动。

### CubeMX 再次 Generate Code 之后

预期会发生:

- `stm32h7xx_it.c` 里重新出现 `SVC_Handler` / `PendSV_Handler` → **链接报 multiple definition** → 删掉即可。这是唯一会因再生成而破坏构建的地方,且不会静默。
- `SysTick_Handler` 里的转发调用在 `USER CODE` 区内,应当存活。若不在了,按步骤 4 补回。
- `main.c` 的 `RTOS_StartTasks()` 调用在 `USER CODE BEGIN 2` 内,应当存活。
- `05_vender/Makefile` 里可能再次出现指向已移走目录的 FreeRTOS `-I` 路径。**无害**(GCC 静默忽略不存在的 `-I`),不必手改 —— 手改 vendor 文件正是本次重构要摆脱的模式。

### 不要做的事

- 不要手改 `05_vender/freertos/` 下的任何文件。要改行为就改 `FreeRTOSConfig.h` 或钩子。若确有必须打的补丁,单独存为 patch 文件并在 `MANIFEST.sha256` 里注明,别让修改隐没在源码树中。
- 不要把 `FreeRTOSConfig.h` 挪回 `05_vender/Core/Inc/` —— 那等于把所有权交还 CubeMX。
- 不要重新启用 CMSIS-RTOS。项目已有 `PLAT_Task_*`,两套并存正是此前问题的来源。
