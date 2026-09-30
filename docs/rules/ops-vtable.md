# ops + 不透明 context —— 这个框架的核心机制

`03_platform` 是**厂商中立**的,`04_impl` 是**厂商特定**的。两者之间只有一个东西:
一张函数指针表(ops vtable)加一个 `void*` context。这份文档讲它在本仓库里的**每一处
具体体现**,每条都能用一行命令验证。

规范性的层级与命名规则在 [`structure.md`](structure.md)。这里讲机制。

## 一、为什么是 vtable 而不是条件编译

换芯片的常见做法是 `#ifdef STM32H7`。这个框架不这么做,原因不是审美:

- `#ifdef` 让**编译不到的那一半永远不被检查**。F4 后端现在不在构建里,但它和 H7 后端
  实现同一个 `UART_Ops_s`,所以任何契约变更都会在编译 F4 时报错 —— 前提是有人编译它。
  vtable 至少让"两个后端形状相同"成为一个可以 `diff` 的事实。
- `#ifdef` 让平台层**知道**有哪些芯片。vtable 让它一个都不知道:`03_platform` 里
  `stm32` 只在一句注释里出现过一次(第三节末尾验证)。
- 主机测试需要一个假后端。填一张结构体比伪造一整套 `HAL_*` 函数便宜得多 ——
  `tests/unit/platform/bsp/suites/test_plat_uart.c` 就是拿 CMock 生成的桩填了
  `UART_Ops_s` 的六个成员,然后测真的 `plat_uart.c`。

代价是每次调用多一次间接跳转。第七节量化了它。

## 二、三个角色

```
     应用/设备层         只见 <Class>_Instance_s*,不见 ops 也不见 ctx
         │
         │  PLAT_SPI_Send(spi, ...)
         ▼
   03_platform          转发器:spi->ops->transmit(spi->ctx, ...)
         │              ── 从不解引用 ctx ──
         │
    [ SPI_Ops_s ]       契约,住在 04_impl/common/impl_spi.h
         │              两层都 include 它,谁都不拥有它
         ▼
   04_impl 后端         填表 + 定义 ctx 的真实类型
                        IMPL_STM32_SPI_Context_s { hspi, cs_port, cs_pin, ... }
```

三者的文件位置就是这个分工:

| 角色 | 文件 | 认识什么 |
|---|---|---|
| 契约 | `04_impl/common/impl_<class>.h` | 只有 `stdint.h` / `stdbool.h`。**没有 HAL 也能编译** |
| 中立 API + 转发器 | `03_platform/{bsp,rtos}/<class>/plat_<class>.{h,c}` | 契约 + 自己的实例结构 |
| 后端 | `04_impl/bsp/stm32h7/<class>/impl_stm32_<class>.{c,h}` | 契约 + 厂商 SDK |

`04_impl/common/` 不在任何后端目录下面,这是有意的 —— 它以前在
`04_impl/bsp/stm32f4/<class>/` 里,加第二个后端时就得二选一:留一个空的 F4 目录只为
放共享头,或者复制一份让两份漂移。**N 个后端共享的契约不属于其中任何一个。**

## 三、实例结构:两个字段撑起整个解耦

每个外设类的实例结构前两个字段完全一样:

```c
struct SPI_Instance_s
{
    const SPI_Ops_s*     ops;    /* 后端 vtable(来自 *_GetOps) */
    void*                ctx;    /* 不透明,后端拥有的设备描述符 */
    PLAT_SPI_TxCallback  tx_cb;  /* 用户回调,平台层自己管 */
    ...
};
```

`ctx` 是 `void*` 而不是某个 struct 指针,这是整个模式的支点。平台层**无法**解引用它,
所以它无法依赖后端的内存布局。后端那边它是有类型的:

```c
/* 04_impl/bsp/stm32h7/uart/impl_stm32_uart.h */
typedef struct
{
    UART_HandleTypeDef* huart;
    UART_Xfer_Mode_e    mode;
    IMPL_UART_RxCb      rx_cb;      /* 平台层注入的 trampoline */
    void*               arg;        /* 平台层给的 token */
    uint8_t* volatile   rx_buf;
    bool                rx_circular;
    volatile uint16_t   rx_pos;
    ...
} IMPL_STM32_UART_Context_s;
```

这个头是**唯一**暴露 `UART_HandleTypeDef` 这个厂商模型的地方,而它只被组合根 include。

验证平台层确实不认识厂商:

```bash
$ grep -rn 'stm32\|HAL_' 03_platform --include=*.c --include=*.h
03_platform/bsp/dma_buf/plat_dma_buf.h:58: * impl_stm32_uart.c, which refuses ...
```

唯一一处命中在**注释里**,指向后端的一处运行时检查。没有一行代码。

## 四、十三个契约,一张表

`04_impl/common/` 下有 13 个契约头。按函数指针数量:

| 契约 | 成员数 | 有 attach_cb | 备注 |
|---|---|---|---|
| `impl_task.h` | 14 | | 最大的一张,`start_scheduler` 的存在理由见下 |
| `impl_iic.h` | 11 | ✓ | 阻塞/异步 × 寄存器/裸传输的组合 |
| `impl_spi.h` | 10 | ✓ | `cs_assert` 是 `bool` —— 它同时是总线认领 |
| `impl_flash.h` | 9 | | 扇区几何全部由后端回答 |
| `impl_adc.h` | 7 | ✓ | 三种模式各一组:阻塞 / IT / DMA |
| `impl_mutex.h` | 7 | | 递归锁是**独立入口**,不是普通锁上的标志位 |
| `impl_uart.h` | 6 | ✓ | |
| `impl_gpio.h` | 5 | | 最简单的一张,`set/reset/toggle/write/read` |
| `impl_pwm.h` | 5 | | |
| `impl_can.h` | 5 | ✓ | |
| `impl_sem.h` | 5 | | `init_binary` / `init_counting` 分开 |
| `impl_dwt.h` | 4 | | |
| `impl_memory.h` | 2 | | `alloc` / `free`,全框架唯一的分配开关 |

`03_platform/bsp/dma_buf/` 是**唯一没有 ops 的平台类** —— `PLAT_DMA_BUF` 是个
section 放置 + 对齐的宏,没有运行时行为可以转发。

### 契约头同时是文档

这是 vtable 在这个仓库里最实用的一面。契约不是一行 `bool (*transmit)(...)`,而是这行
加上它上面几十行说明"返回 false 到底意味着什么"。以 `impl_uart.h` 的 `start_rx` 为例,
契约里写明的事有:

- `buf` 必须活到 `stop_rx`
- 后端**可以在任何时刻写 `buf` 的任何部分** —— 连续流式的后端会循环复用它
- rx trampoline 交付的是**新到的一段字节,不一定是一整帧**;idle framing 会把两帧粘
  在一起,循环 DMA 会把跨越缓冲区末尾的一段拆成两次回调

第三条是**只有契约能说的事**。它不是任何单个后端的实现细节,而是所有后端的调用方都必须
容忍的下界。写第二个后端的人照着契约实现就够了,不必读现有后端的 `.c`。

## 五、两步创建与那道编译期的门

拿到一个可用实例是两步:

```c
/* 1. 后端造 ctx —— 参数是厂商的东西 */
void* ctx = IMPL_STM32_SPI_CreateCtx(&hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, SPI_XFER_IT);

/* 2. 平台层把 ops 和 ctx 包进实例 */
PLAT_SPI_Init(&s_imu_accel, IMPL_STM32_SPI_GetOps(), ctx);
```

**两个参数都是厂商符号**,所以调这两个函数的人按定义就在指名一颗芯片。这是组合根的活,
别处不许干 —— 而且这条不是 code review 意见,是编译错误:

```c
/* 03_platform/bsp/spi/plat_spi.h 末尾 */
#ifdef PLAT_ALLOW_CONSTRUCTION
bool PLAT_SPI_Init(SPI_Instance_s* inst, const SPI_Ops_s* ops, void* ctx);
SPI_Instance_s* PLAT_SPI_Create(const SPI_Ops_s* ops, void* ctx);
#endif
```

九个 `03_platform/bsp/*/plat_*.h` 都有这道门(`rtos` 类不需要,见第六节)。定义这个宏的
`.c` 有十个:组合根,加九个**构造函数自己所在的翻译单元**(给自己开门,头里写了理由)。
门要挡的是应用层和设备层,那里可以机械检查:

```bash
$ grep -rl 'define PLAT_ALLOW_CONSTRUCTION' --include=*.c 01_application 02_device
01_application/board/board_stm32h7.c
```

旁边那条 grep 是组合根规则的检查,形状相同:

```bash
$ grep -rl impl_stm32_ --include=*.c 01_application 02_device
01_application/board/board_stm32h7.c
```

**这两条 grep 只返回一个文件,是这整套机制唯一的对外承诺。** 别的都是内部安排。

### 为什么固定外设走 Init 而不是 Create

`PLAT_<Class>_Create(ops, ctx)` 是 `PLAT_malloc` + `Init` 的薄包装。板上外设数量编译期
就定了,所以组合根给的是静态存储:

```c
static SPI_Instance_s s_imu_accel;
```

分配在这里没有任何好处,只多一条**与硬件无关**的失败路径(堆不够)。后果可以在镜像里看到:

```bash
$ arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -E 'T PLAT_[A-Z]+_Create'
080094e8 T PLAT_CAN_Create      ← 只剩这一个
```

CAN 是例外,而且理由和 vtable 无关:一个机器人上有几个 CAN 节点是**运行时**决定的,没有
固定存储可以交进去。

## 六、RTOS 类不带 ops 参数

`plat_task` / `plat_mutex` / `plat_sem` / `plat_memory` 的公开 API **既不收 ops 也不收
context**:

```c
bool PLAT_Sem_InitBinary(Sem_s* s);                 /* 没有 ops 参数 */
bool PLAT_Task_Create(Task_s* task, PLAT_Task_Entry entry, ...);
```

因为一次构建里只可能有一个 RTOS 后端,所以它们直接调 `IMPL_<X>_GetOps()`:

```c
const IMPL_Sem_Ops_s* ops = IMPL_Sem_GetOps();
if (ops == NULL || ops->init_binary == NULL) return false;
```

这不是偷懒,而是**让 `06_utils` 能用一把锁而不必指名厂商**。`util_msgbus` 就是这么用
`plat_mutex.h` 的 —— 它是全仓库唯一被批准向上依赖 `03_platform` 的 util,理由写在
`structure.md` 里。

代价是 ctx 没地方放,而 RTOS 对象是有状态的。解法是**调用方持有的不透明存储块**:

```c
#define PLAT_SEM_STORAGE_BYTES 96u

typedef struct
{
    union { uint64_t align; uint8_t bytes[PLAT_SEM_STORAGE_BYTES]; } storage;
    bool initialized;
} Sem_s;
```

96 这个数由**平台层**定,而不是从 impl 层取 —— 这正是让 `plat_sem.h` 不含 impl 类型的
原因。后端在编译期断言自己装得下:

```c
/* 04_impl/rtos/freertos/sem/impl_sem.c */
_Static_assert(sizeof(StaticSemaphore_t) <= 96u, ...);
```

所以换一个对象更大的 RTOS 会**构建失败**,而不是溢出这个块。union 里那个 `uint64_t`
是为了 8 字节对齐,因为后端可能在里面放指针或 64 位计数器。

`impl_task.h` 里 `start_scheduler` 的注释值得一读:表里别的成员都是固件跑起来之后调的,
换后端时可以想象没有它们;这一个存在的理由不同 —— **没有它,组装应用任务列表的那个文件
就得指名 RTOS 才能启动它**,那是唯一会把任务组装钉死在 impl 层的符号。有了它,
`app_tasks.c` 就是普通的应用代码。

## 七、转发器的代价,量化

平台层每次调用都是一次 vtable 转发。`-O2` 下它编成一次尾调用:

```
08009360 <PLAT_SPI_Send>:
 8009360:  ldrd  ip, r0, [r0]      ; 同时取 ops 和 ctx(相邻字段,一条指令)
 8009364:  push  {lr}
 8009366:  ldr.w lr, [ip]          ; ops->transmit
 800936a:  mov   ip, lr
 800936c:  ldr.w lr, [sp], #4
 8009370:  bx    ip                ; 尾跳,不建栈帧
```

6 条指令,加一条对齐用的 `nop`,`nm --print-size` 报 18 字节。`ops` 和 `ctx` 是实例结构的
前两个字段,所以一条 `ldrd` 就取到两个 —— 这不是巧合,字段顺序是照这个排的。

镜像里 `PLAT_*` 导出函数共 34 个,最小的 4 字节(`PLAT_CAN_OnReceive`,只是一次赋值),
纯转发的多在 8~18 字节。带参数校验或单位换算的那几个更大(`PLAT_PWM_SetDutyPercent`
104 字节,因为它做了钳位和百分比→CCR 的换算):

```bash
$ arm-none-eabi-nm --print-size --size-sort build/COD_UniFramework_H7.elf | grep ' T PLAT_'
```

**`-O0` 对这个架构不是中性选择。** 每个转发器变成 14 条指令带完整栈帧,`BUILD_TYPE`
从 `Debug` 换成 `RelWithDebInfo` 省下 63 KB text —— 这是 2026-09-03 换默认构建类型的
实测理由,不是猜测。

## 八、回调怎么穿过这一层

五个类有 `attach_cb`(`spi` `uart` `adc` `can` `iic`)。中断在后端里发生,用户回调注册
在平台层,中间靠 **trampoline** 连:

```c
/* plat_uart.c —— 中立的,不认识厂商 */
static void plat_uart_rx_tramp(void* arg, const uint8_t* data, uint16_t len)
{
    UART_Instance_s* uart = arg;
    if (uart->ring_on) UTIL_RingBuf_PutN(&uart->rx_ring, data, len);
    if (uart->rx_cb != NULL) uart->rx_cb(uart, data, len);
}

bool PLAT_UART_Init(UART_Instance_s* inst, const UART_Ops_s* ops, void* ctx)
{
    ...
    /* 一次性挂上;用户回调是懒查的,所以后来注册不需要重新 attach */
    ops->attach_cb(ctx, plat_uart_rx_tramp, plat_uart_tx_tramp, plat_uart_err_tramp, inst);
}
```

`arg` 就是平台实例本身。后端把它原样存着,中断里原样传回来 —— 后端因此不需要知道
`UART_Instance_s` 是什么。

**关键的一点:trampoline 挂一次就够了。** `PLAT_UART_OnReceive` 只是写 `inst->rx_cb`,
它是在 trampoline 里被**懒查**的,所以先 Init 后注册、注册两次、注册 NULL 都不需要
重新 attach。平台层一共 14 个这样的 trampoline。

错误码也在契约里中立化。后端把自己的硬件标志位映射到公共集合:

```c
#define UART_ERR_FRAMING 0x01u
#define UART_ERR_PARITY  0x02u
#define UART_ERR_NOISE   0x04u
#define UART_ERR_OVERRUN 0x08u
#define UART_ERR_DMA     0x10u
```

所以上层永远看不到厂商错误码。

## 九、契约相同,实现可以不同 —— 而这正是这层该干的事

两个后端的目录集合完全一样:

```bash
$ diff <(ls 04_impl/bsp/stm32h7) <(ls 04_impl/bsp/stm32f4)
(无差异)
```

**对称的是签名,不是实现。** CAN 的"一个节点认领一段 id"是最好的例子:

- FDCAN 有 `FDCAN_FILTER_RANGE`,H7 后端**一个滤波元件**覆盖整段。
- bxCAN 没有精确范围滤波器。而 `0x201..0x204` 既不是 2 的幂长度也不对齐,用掩码会
  **多收** —— 最窄的掩码同时放进 `0x200..0x207`,把 DJI 的控制 id 和三个 GM6020 的
  反馈一起收了。所以 F4 后端把整段**展开成逐 id 的滤波槽和注册表条目**。

调用方两边都是:一个节点,一个回调,一次 teardown。**差异被吸收在后端里,而不是泄漏成
两套 API。** 每个后端的头文件里写着自己这么做的硬件理由。

同一件事的另一个例子:`SPI_Ops_s` 的 `cs_assert` 返回 `bool`。它不只是拉低片选,还
**认领总线** —— BMI088 的两个 die 共用 `hspi2`,如果持有 CS 跨越一个事务而仲裁只跨越
单次传输,另一个器件就能在一个事务的两次传输之间抢到总线,于是两个片选同时为低、两个
die 同时驱动 MISO,数据错了而任何一层都没有错误。契约把这件事写成了返回值,
`dev_bmi088` 就必须检查它。

## 十、这层是怎么被测的

ops 表是个普通结构体,所以主机测试**填一张假的**就能测真的平台层:

```c
/* tests/unit/platform/bsp/suites/test_plat_uart.c */
#define PLAT_ALLOW_CONSTRUCTION      /* 测试有权构造 */
#include "plat_uart.h"

static const UART_Ops_s uart_ops = {
    .transmit       = PBSP_UART_Transmit,      /* CMock 生成的桩 */
    .receive        = PBSP_UART_Receive,
    .transmit_async = PBSP_UART_TransmitAsync,
    .attach_cb      = PBSP_UART_AttachCb,
    .start_rx       = PBSP_UART_StartRx,
    .stop_rx        = PBSP_UART_StopRx,
};

/* 然后逐个断言转发正确 */
PBSP_UART_Receive_ExpectAndReturn(backend_ctx, data, 8u, 21u, 3u);
TEST_ASSERT_EQUAL_UINT16(3u, PLAT_UART_Receive(&uart, data, 8u, 21u));
```

`backend_ctx` 在测试里就是 `(void*) 0xAA71u` —— 一个假地址。这**能跑**,而且它能跑
本身就是"平台层从不解引用 ctx"这条性质的证明。

组合根也是这么测的:`test_board_stm32h7.c` 声明六张空 ops 表(八个设备只用到六个类)和
一个 `unsigned contexts[DEVICE_COUNT]` 当假 context,验证 bring-up 顺序、失败时停在第一个
出错的设备、以及反序 teardown。

## 十一、加一个后端要做什么

1. 在 `04_impl/bsp/<vendor>/<class>/` 下实现契约的**每一个**成员。
2. 暴露三件套:`IMPL_<VENDOR>_<CLASS>_CreateCtx()` / `_GetOps()` / `_DestroyCtx()`。
3. 写 `board_<vendor>.c`,把上面那三个调用抄一遍。
4. 改 `CMakeLists.txt` 的源文件列表**一行**。

`03_platform` 里**一个文件都不改**。两个组合根同时在构建里是链接错误(同名符号),
这是有意的 —— 一块板子只有一个组合根。

注意第 3 步现在还是**没有实例的**:`01_application/board/` 下只有 `board.h` 和
`board_stm32h7.c`,没有 `board_stm32f4.c`。F4 那半边(`04_impl/bsp/stm32f4/`)是完整的,
组合根那一半还没写过 —— 所以"换芯片只改一行 CMake"这句话,前四步里第 1、2、4 步有实证,
第 3 步没有。

## 十二、几个容易踩的地方

**契约头必须在没有 HAL 的情况下能编译。** 这是 `04_impl/common/` 的硬性约束,它保证了
`03_platform` 的主机测试不需要任何厂商 SDK。往契约头里 include 一个 `stm32*.h` 会立刻
毁掉整个测试树。

**后端不许向上调 `03_platform`。** 后端要分配内存用 `IMPL_malloc`,不是 `PLAT_malloc`
(同一个 ops,但不跨层向上),更不是 `pvPortMalloc` —— 直接调后者会绕过
`04_impl/rtos/freertos/memory/impl_memory.c` 里那个唯一的分配开关。

**ops 表里的成员可以是 NULL,而平台层必须容忍。** `impl_task.h` 的 `fault_init` 就是
这么设计的:没有可分离故障的核心可以留 NULL,平台层把缺失的条目当作"无事可做"。
`impl_mutex.h` 的 `init_recursive` 同理 —— 没有递归锁的后端留 NULL,平台层在 Init 时
报告失败。**加成员时想清楚缺失是失败还是无操作,并写进契约。**

**没有调用者的东西不在镜像里。** ops 表也一样。今天镜像里有 8 张:

```bash
$ arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -E '_ops$'
0801a198 r stm32_can_ops       0801a0e0 r stm32_dwt_ops
0801a15c r stm32_flash_ops     0801a0cc r stm32_pwm_ops
0801a134 r stm32_spi_ops       0801a0b4 r stm32_uart_ops
0801a014 r impl_task_ops       0801a04c r impl_memory_ops
```

缺席的是 `gpio` `adc` `iic` `mutex` `sem` —— 组合根没有注册这些设备,或者还没有任务用
到那把锁。这**不是缺陷**,但它意味着**这五个类唯一被执行过的地方是 `tests/`**。所以
"这个后端能用"这句话的依据是主机测试,不是任何在硬件上跑过的东西。这份清单是关于**今天
的调用图**的说法,不是关于这些模块的 —— 重新测量,别复用上面的结果。
