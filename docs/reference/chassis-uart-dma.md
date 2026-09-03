# Chassis UART DMA 双缓冲设计与寄存器解析

> 分析对象：  
> `/home/stg/Downloads/Chassis/Bsp/Src/bsp_uart.c`  
> `/home/stg/Downloads/Chassis/Bsp/Inc/bsp_uart.h`
>
> 相关模块：  
> `/home/stg/Downloads/Chassis/Components/Device/Inc/remote_control.h`  
> `/home/stg/Downloads/Chassis/Components/Device/Inc/referee_system.h`  
> `/home/stg/Downloads/Chassis/Core/Src/usart.c`

## 1. 设计概述

该工程使用 STM32F4 DMA Stream 的双存储区模式 DBM，为 USART3 遥控器数据和 USART6 裁判系统数据分别配置两块接收缓冲区：

```text
UART 数据寄存器 DR
        ↓ DMA 外设到内存传输
    ┌───┴───┐
    ↓       ↓
Memory 0  Memory 1
    M0       M1
```

代码同时使用 UART IDLE 空闲中断判断一次接收结束，并在 IDLE 回调中完成：

1. 读取 DMA 当前目标 `CT`；
2. 停止 DMA；
3. 手动切换 `CT`；
4. 解析当前缓冲区；
5. 重装 `NDTR`；
6. 重新开启 IDLE、UART DMA 请求和 DMA Stream。

因此该方案准确来说是：

> **DMA DBM 提供 M0/M1 两个内存地址，UART IDLE 负责分帧，软件手动控制 CT 完成乒乓切换。**

它并不是完全由 DMA Transfer Complete 驱动的标准硬件双缓冲。当前代码在解析期间关闭了 DMA，因此也没有真正实现“CPU 处理 A 时 DMA 同时接收 B”。

---

## 2. 两路 UART 和缓冲区

### 2.1 USART3：遥控器 SBUS/DBUS

缓冲区定义：

```c
#define SBUS_RX_BUF_NUM 18u

uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
```

逻辑结构：

```text
M0AR → SBUS_MultiRx_Buf[0][18]
M1AR → SBUS_MultiRx_Buf[1][18]
```

DMA 配置：

```text
USART3_RX
DMA1 Stream1
Channel 4
外设到内存
DMA_CIRCULAR
字节对齐
内存地址递增
```

### 2.2 USART6：裁判系统

缓冲区定义：

```c
#define REFEREE_RXFRAME_LENGTH 136

uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
```

逻辑结构：

```text
M0AR → Referee_System_Info_MultiRx_Buf[0][136]
M1AR → Referee_System_Info_MultiRx_Buf[1][136]
```

DMA 配置：

```text
USART6_RX
DMA2 Stream1
Channel 5
外设到内存
DMA_CIRCULAR
字节对齐
内存地址递增
```

### 2.3 `bsp_uart.h` 的作用

本工程的 `bsp_uart.h` 没有定义双缓冲对象，只声明了：

- `BSP_USART_Init()`；
- VOFA 数据帧类型；
- `VofaSendMsg()`。

实际接收缓冲区位于遥控器和裁判系统设备模块。

---

## 3. 初始化调用

```c
void BSP_USART_Init(void)
{
    USART_RxDMA_MultiBufferStart(
        &huart3,
        (uint32_t *)&huart3.Instance->DR,
        (uint32_t *)SBUS_MultiRx_Buf[0],
        (uint32_t *)SBUS_MultiRx_Buf[1],
        SBUS_RX_BUF_NUM);

    USART_RxDMA_MultiBufferStart(
        &huart6,
        (uint32_t *)&huart6.Instance->DR,
        (uint32_t *)Referee_System_Info_MultiRx_Buf[0],
        (uint32_t *)Referee_System_Info_MultiRx_Buf[1],
        REFEREE_RXFRAME_LENGTH);
}
```

参数对应关系：

| 参数 | 含义 |
|---|---|
| `huart` | UART HAL 句柄 |
| `SrcAddress` | UART 数据寄存器 `DR` 地址 |
| `DstAddress` | DMA Memory 0 地址 |
| `SecondMemAddress` | DMA Memory 1 地址 |
| `DataLength` | 单块缓冲区容量 |

DMA 方向已由 CubeMX 配置为 `DMA_PERIPH_TO_MEMORY`，所以数据流为：

```text
USARTx_DR → DMA_SxPAR → DMA Stream → M0AR/M1AR
```

---

## 4. 初始化代码逐步解析

### 4.1 设置 HAL 接收类型

```c
huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
```

该字段不是硬件寄存器，而是 HAL 软件状态。`HAL_UART_IRQHandler()` 会检查它，决定 IDLE 时是否调用：

```c
HAL_UARTEx_RxEventCallback(huart, Size);
```

### 4.2 设置 HAL 接收长度

```c
huart->RxXferSize = DataLength * 2;
```

得到：

```text
USART3：RxXferSize = 18 × 2  = 36
USART6：RxXferSize = 136 × 2 = 272
```

`RxXferSize` 同样是 HAL 软件字段，不是 DMA 寄存器。HAL 使用它和 `NDTR` 计算回调参数：

```text
Size = RxXferSize - NDTR
```

当前实现故意将 `RxXferSize` 设为单块容量的两倍，这是其 IDLE 分帧技巧的一部分。

### 4.3 开启 UART DMA 接收请求

```c
SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
```

对应寄存器位：

```text
USART_CR3.DMAR = 1
```

开启后，USART 接收到数据时会向 DMA 控制器发出接收请求。若 `DMAR=0`，即使 DMA Stream 已使能，也不会有 UART 数据进入 DMA。

### 4.4 开启 UART IDLE 中断

```c
__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
```

对于 STM32F4，该宏最终设置：

```text
USART_CR1.IDLEIE = 1
```

当 `USART_SR.IDLE=1` 时，产生 USART 中断。

### 4.5 停止 DMA Stream

```c
do
{
    __HAL_DMA_DISABLE(huart->hdmarx);
} while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);
```

对应：

```text
DMA_SxCR.EN = 0
```

修改 `PAR`、`M0AR`、`M1AR`、`NDTR` 和部分 `CR` 配置前，需要先关闭 Stream，并等待硬件确认 `EN` 清零。

### 4.6 配置外设地址 PAR

```c
huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;
```

实际写入：

```text
DMA_SxPAR = &USARTx->DR
```

由于方向是外设到内存，DMA 每次收到请求后从 `PAR` 指向的 UART `DR` 读取数据。

### 4.7 配置 M0AR 和 M1AR

```c
huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;
huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;
```

对应：

```text
DMA_SxM0AR = Buffer 0 地址
DMA_SxM1AR = Buffer 1 地址
```

只有启用 `DMA_SxCR.DBM` 后，M1AR 才作为第二存储区参与传输。

### 4.8 配置 NDTR

```c
huart->hdmarx->Instance->NDTR = DataLength;
```

初始值：

```text
USART3：NDTR = 18
USART6：NDTR = 136
```

`NDTR.NDT` 表示剩余数据单元数量。当前配置为字节传输，因此一个数据单元就是一个字节。

每收到一个字节：

```text
NDTR = NDTR - 1
```

当 NDTR 递减到 0，在 Circular/DBM 模式下，硬件会重新装载计数器并切换目标存储区。

### 4.9 开启 DBM

```c
SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
```

对应：

```text
DMA_SxCR.DBM = 1
```

DBM 开启后：

- `M0AR` 和 `M1AR` 都参与传输；
- `CT` 指示当前活动目标；
- 当前目标写满后，硬件可自动切到另一目标；
- 在本项目稳定阶段，代码主要依赖 IDLE 后软件手动切换 CT。

### 4.10 开启 DMA

```c
__HAL_DMA_ENABLE(huart->hdmarx);
```

对应：

```text
DMA_SxCR.EN = 1
```

此时接收通道生效需要同时满足：

```text
USART_CR3.DMAR = 1
DMA_SxCR.EN    = 1
```

---

## 5. USART 相关寄存器

## 5.1 `USART_SR`：状态寄存器

### `IDLE`

```text
USART_SR.IDLE
```

表示 UART 接收线路在接收过至少一个数据后，持续一个帧时间没有新数据。

本设计使用它作为一次接收的边界：

```text
收到若干字节 → 总线空闲 → IDLE=1 → 进入 USART IRQ
```

在 STM32F4 上，清除 IDLE 通常需要按规定读取 SR，再读取 DR。HAL 使用：

```c
__HAL_UART_CLEAR_IDLEFLAG(huart);
```

完成该序列。

### `RXNE`

```text
USART_SR.RXNE
```

表示接收数据寄存器非空。DMA 读取 `USART_DR` 后，RXNE 被清除。

正常 DMA 接收时不需要 CPU 在每个字节上响应 RXNE 中断。

### `ORE`

```text
USART_SR.ORE
```

Overrun Error。若前一个数据没有被及时读取，新数据又到达，就会产生溢出。

当前代码在解析缓冲区时关闭 DMA，若此期间新数据到达，就可能触发 ORE。

### `FE`、`NE`、`PE`

| 位 | 含义 |
|---|---|
| `FE` | Framing Error，帧格式错误 |
| `NE` | Noise Error，噪声错误 |
| `PE` | Parity Error，奇偶校验错误 |

健壮实现应统计并恢复这些错误，而不能只处理正常 IDLE。

## 5.2 `USART_DR`：数据寄存器

```text
USART_DR
```

UART 收到的数据进入该寄存器。本工程把它的地址写入：

```text
DMA_SxPAR
```

DMA 从 DR 读取后写入当前 M0/M1 缓冲区。

## 5.3 `USART_CR1`

### `IDLEIE`

```text
USART_CR1.IDLEIE = 1
```

允许 IDLE 事件产生 USART 中断。

相关代码：

```c
__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
```

### `RE`

```text
USART_CR1.RE = 1
```

Receiver Enable，允许 UART 接收器工作。通常由 `HAL_UART_Init()` 配置。

### `UE`

```text
USART_CR1.UE = 1
```

USART Enable。该位为 0 时 USART 整体未启用。

## 5.4 `USART_CR3`

### `DMAR`

```text
USART_CR3.DMAR = 1
```

允许 UART 接收向 DMA 发送请求。

相关代码：

```c
SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
```

### `EIE`

```text
USART_CR3.EIE
```

允许 Frame Error、Noise Error、Overrun Error 等产生中断。HAL 标准 DMA 启动路径通常会设置该位，自定义寄存器启动时应确认错误中断状态。

---

## 6. DMA Stream 相关寄存器

STM32F4 DMA 每个 Stream 的核心寄存器为：

```text
DMA_SxCR
DMA_SxNDTR
DMA_SxPAR
DMA_SxM0AR
DMA_SxM1AR
DMA_SxFCR
```

## 6.1 `DMA_SxCR`：Stream 配置寄存器

### `EN`

```text
DMA_SxCR.EN
```

| 值 | 含义 |
|---|---|
| 0 | Stream 关闭 |
| 1 | Stream 使能 |

修改关键配置前应清零并等待硬件确认。

### `DBM`

```text
DMA_SxCR.DBM
```

| 值 | 含义 |
|---|---|
| 0 | 单缓冲区，仅使用 M0AR |
| 1 | 双缓冲区，使用 M0AR 和 M1AR |

本设计核心配置：

```c
SET_BIT(DMA_SxCR, DMA_SxCR_DBM);
```

### `CT`

```text
DMA_SxCR.CT
```

Current Target：

| CT | DMA 当前目标 |
|---|---|
| 0 | M0AR |
| 1 | M1AR |

必须区分两个场景。

#### 场景 A：缓冲区尚未写满，IDLE 发生

```text
CT=0 → 当前数据位于 M0
CT=1 → 当前数据位于 M1
```

#### 场景 B：缓冲区写满并发生硬件切换

```text
CT=0 → 当前正在写 M0，M1 刚完成
CT=1 → 当前正在写 M1，M0 刚完成
```

因此，IDLE 事件和 Transfer Complete 事件不能不加区分地使用同一套 CT 判断逻辑。

### `CIRC`

```text
DMA_SxCR.CIRC
```

Circular Mode。USART3/USART6 RX 都由 CubeMX 配置为 Circular。

当 NDTR 到 0 时，硬件重新装载计数器并继续工作。DBM 本身也具有循环切换 M0/M1 的使用方式。

### `DIR`

```text
DMA_SxCR.DIR
```

本项目为外设到内存：

```text
DIR = 00：Peripheral-to-memory
```

### `MINC`

```text
DMA_SxCR.MINC = 1
```

每传输一个字节，内存地址递增：

```text
buffer[0] → buffer[1] → buffer[2] → ...
```

### `PINC`

```text
DMA_SxCR.PINC = 0
```

外设地址不递增，因为每次都从同一个 `USART_DR` 读取。

### `MSIZE` 与 `PSIZE`

当前 UART 数据宽度为字节：

```text
MSIZE = byte
PSIZE = byte
```

因此 `NDTR` 的一个数据单元对应一个字节。

### `CHSEL`

选择 DMA Channel：

```text
USART3 RX：DMA1 Stream1 Channel 4
USART6 RX：DMA2 Stream1 Channel 5
```

Stream/Channel 映射由芯片固定，必须查具体 STM32F4 Reference Manual/Datasheet。

### `PL`

DMA 优先级。当前两路 RX 配置为 Low。多个 Stream 同时请求时，优先级会影响 DMA 仲裁顺序。

### `TCIE`、`HTIE`、`TEIE`、`DMEIE`

| 位 | 含义 |
|---|---|
| `TCIE` | Transfer Complete 中断使能 |
| `HTIE` | Half Transfer 中断使能 |
| `TEIE` | Transfer Error 中断使能 |
| `DMEIE` | Direct Mode Error 中断使能 |

当前自定义启动函数直接调用 `__HAL_DMA_ENABLE()`，没有像 `HAL_DMA_Start_IT()` 那样完整安装回调并统一配置 DMA 中断。因此该设计主要依赖 UART IDLE，不应假设 DMA TC/HT 回调已正确建立。

## 6.2 `DMA_SxNDTR`：剩余数量寄存器

核心字段：

```text
DMA_SxNDTR.NDT
```

表示当前还需要传输多少个数据单元。

例如稳定阶段 USART3：

```text
初始 NDTR = 36
收到 18 字节
NDTR = 18
```

HAL 计算：

```text
Size = RxXferSize - NDTR
     = 36 - 18
     = 18
```

注意：`RxXferSize` 是 HAL 软件变量，`NDTR` 是硬件寄存器，两者必须具有一致的计算约定。

## 6.3 `DMA_SxPAR`：外设地址寄存器

```text
DMA_SxPAR = &USARTx->DR
```

方向为外设到内存时，DMA 从该地址读取。

## 6.4 `DMA_SxM0AR`：Memory 0 地址

保存第一块目标内存地址：

```text
USART3：&SBUS_MultiRx_Buf[0][0]
USART6：&Referee_System_Info_MultiRx_Buf[0][0]
```

## 6.5 `DMA_SxM1AR`：Memory 1 地址

保存第二块目标内存地址：

```text
USART3：&SBUS_MultiRx_Buf[1][0]
USART6：&Referee_System_Info_MultiRx_Buf[1][0]
```

仅在 DBM 开启时参与传输。

## 6.6 `DMA_SxFCR`：FIFO 控制寄存器

当前 CubeMX 配置：

```text
FIFOMode = DMA_FIFOMODE_DISABLE
```

即 Direct Mode。UART 每收到数据，DMA 直接按配置搬运到内存。

相关位还包括：

- `DMDIS`：Direct Mode Disable；
- `FTH`：FIFO 阈值；
- `FEIE`：FIFO Error Interrupt Enable；
- `FS`：FIFO 状态。

本设计未使用 DMA FIFO。

## 6.7 DMA 中断状态和清除寄存器

DMA 控制器还有：

```text
DMA_LISR / DMA_HISR
DMA_LIFCR / DMA_HIFCR
```

分别用于：

- 读取 TC、HT、TE、DME、FE 等中断状态；
- 清除对应中断标志。

具体 Stream 使用 LISR 还是 HISR 取决于 Stream 编号。HAL 的 `HAL_DMA_IRQHandler()` 会完成这些操作。

---

## 7. HAL 的 IDLE 长度计算

本工程 HAL 的关键逻辑为：

```c
uint16_t nb_remaining_rx_data =
    (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);

huart->RxXferCount = nb_remaining_rx_data;

HAL_UARTEx_RxEventCallback(
    huart,
    huart->RxXferSize - huart->RxXferCount);
```

因此：

$$
Size = RxXferSize - NDTR
$$

当前代码让：

$$
RxXferSize = 2N
$$

稳定阶段让：

$$
NDTR_{initial}=2N
$$

收到 $k$ 字节后：

$$
NDTR=2N-k
$$

于是：

$$
Size=2N-(2N-k)=k
$$

所以稳定阶段 HAL 回调的 `Size` 正好等于本次接收长度。

---

## 8. USART3 稳定阶段时序

设：

```text
N = 18
RxXferSize = 36
NDTR 初始值 = 36
当前 CT = 0
```

### 8.1 DMA 接收

```text
DMA 当前写 M0
收到 18 字节
NDTR：36 → 18
CT 仍为 0
```

因为 NDTR 没有减到 0，硬件不会自动切换 CT。

### 8.2 UART IDLE

总线空闲后：

```text
USART_SR.IDLE = 1
```

USART IRQ 调用：

```text
HAL_UART_IRQHandler
    ↓
HAL_UARTEx_RxEventCallback(huart, 18)
```

### 8.3 用户处理

`CT=0` 分支：

```c
__HAL_DMA_DISABLE(huart->hdmarx);
huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
__HAL_DMA_SET_COUNTER(huart->hdmarx, 36);

if (Size == 18)
{
    SBUS_TO_RC(SBUS_MultiRx_Buf[0], &remote_ctrl);
}
```

逻辑：

```text
停止 DMA
    ↓
将 CT 从 M0 切到 M1
    ↓
NDTR 重新设为 36
    ↓
解析 M0 中的 18 字节
```

### 8.4 重启

统一回调结尾执行：

```c
huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
__HAL_DMA_ENABLE(huart->hdmarx);
```

下一帧写 M1。

完整乒乓过程：

```text
帧 1 → M0 → IDLE → 解析 M0 → 切 M1
帧 2 → M1 → IDLE → 解析 M1 → 切 M0
帧 3 → M0 → IDLE → 解析 M0 → 切 M1
```

---

## 9. USART6 稳定阶段时序

设裁判系统本次收到 30 字节：

```text
RxXferSize = 272
NDTR 初始值 = 272
收到 30 字节后 NDTR = 242
```

HAL 计算：

```text
Size = 272 - 242 = 30
```

处理器执行：

```text
停 DMA
    ↓
切换 CT
    ↓
Referee_System_Frame_Update(当前缓冲区)
    ↓
memset 清空当前缓冲区
    ↓
NDTR 重装为 272
    ↓
重新开启 DMA
```

与固定 18 字节 SBUS 不同，裁判系统只要求：

```c
if (Size >= 10)
```

然后由协议解析器从缓冲区中解析完整帧。

---

## 10. 初始化阶段的特殊问题

初始化时的配置不是：

```text
RxXferSize = 2N
NDTR       = 2N
```

而是：

```text
RxXferSize = 2N
NDTR       = N
```

这使首次接收与后续接收具有不同的 Size 计算行为。

### 10.1 第一次收到 k<N 字节

初始：

```text
RxXferSize = 2N
NDTR       = N
```

收到 $k$ 字节后：

```text
NDTR = N-k
```

HAL 计算：

$$
Size=2N-(N-k)=N+k
$$

这不是真实接收长度。

例如 SBUS 第一次只收到 10 字节：

```text
RxXferSize = 36
NDTR       = 18 - 10 = 8
Size       = 36 - 8 = 28
```

实际收到 10 字节，但回调参数是 28。

### 10.2 第一次刚好收到 N 字节

初始 NDTR 为 N，收到 N 字节后 NDTR 到 0。由于 DBM/Circular 模式，硬件会：

1. 重新装载 NDTR；
2. 自动切换 CT；
3. 开始使用另一块 Memory。

假设初始 `CT=0`：

```text
M0 收满 N 字节
    ↓
硬件自动 CT=1
    ↓
M0 才是刚完成缓冲区
M1 是当前活动缓冲区
```

但当前处理代码在 `CT=1` 分支中解析 M1：

```c
SBUS_TO_RC(SBUS_MultiRx_Buf[1], &remote_ctrl);
```

因此首个满长度帧存在选择错误缓冲区的风险。

后续处理器把 NDTR 改为 `2N`，收到 N 字节时 NDTR 不会到 0，硬件不自动切 CT，软件逻辑才进入预期的稳定阶段。

---

## 11. 当前实现不是严格的并行双缓冲

理想硬件双缓冲：

```text
DMA 写 M0
    ↓ M0 完成，硬件立即切换
DMA 写 M1 ─────────────┐
                       │ 同时
CPU 处理 M0 ───────────┘
```

当前代码：

```text
DMA 写 M0
    ↓ IDLE
关闭 DMA
    ↓
切 CT
    ↓
CPU 处理 M0
    ↓
重新开启 DMA
    ↓
DMA 才开始写 M1
```

由于解析函数运行期间 DMA 被关闭，以下函数执行时间都会形成接收盲区：

```c
SBUS_TO_RC(...);
Referee_System_Frame_Update(...);
memset(...);
```

如果接收盲区内新数据到达，可能出现：

- UART ORE；
- 字节丢失；
- 下一帧错位；
- CRC 错误。

---

## 12. 当前实现的主要风险

### 12.1 `NDTR=2N` 大于单块实际容量 N

实际缓冲区：

```text
SBUS：每块 18 字节
裁判系统：每块 136 字节
```

后续 NDTR：

```text
SBUS：36
裁判系统：272
```

如果在收到 N 字节前后没有及时出现 IDLE，DMA 会继续向当前内存写入第 `N+1` 个字节，造成越界。

对于二维数组：

```c
uint8_t buf[2][N];
```

M0 越界可能覆盖 M1；M1 越界可能破坏其他全局变量。

该设计依赖强假设：

> 每次接收一定会在不超过 N 字节时出现 IDLE。

### 12.2 首次 `RxXferSize` 与 `NDTR` 不一致

首次 `Size` 带 N 字节偏移，首个满帧还可能因硬件自动切 CT 而选错缓冲区。

### 12.3 回调中关闭 DMA 后未等待 EN 清零

初始化函数正确等待：

```c
while (DMA_SxCR.EN != 0)
{
}
```

但用户处理函数只执行：

```c
__HAL_DMA_DISABLE(huart->hdmarx);
```

随后立即修改 CT 和 NDTR。更安全的写法应等待 `EN=0` 后再修改关键寄存器。

### 12.4 USART6 无效短数据时不重装 NDTR

USART6 只有在：

```c
Size >= 10
```

时才执行：

```c
__HAL_DMA_SET_COUNTER(..., 272);
```

但 CT 在判断前已经切换。如果 `Size<10`：

- CT 已切换；
- NDTR 保留旧值；
- DMA 被重新开启；
- 下一次 Size 计算可能错误。

重装 NDTR 应与数据有效性判断分离。

### 12.5 中断中直接解析协议

`HAL_UARTEx_RxEventCallback()` 运行在 USART IRQ 上下文。当前代码直接调用协议解析函数，会增加中断占用时间。

更合理的中断职责是：

1. 保存缓冲区指针和长度；
2. 立即重启接收；
3. 使用 FreeRTOS `FromISR` API 通知任务；
4. 在任务上下文解析协议。

### 12.6 DMA TC/HT/Error 路径不完整

自定义启动直接配置寄存器并使能 DMA，没有完整执行 HAL 标准 DMA 启动中的：

- `RxState` 状态设置；
- DMA Transfer Complete 回调安装；
- Half Transfer 回调安装；
- DMA Error 回调安装；
- DMA 中断统一使能；
- UART ORE 清除；
- UART Error Interrupt 配置。

所以该实现主要依赖 UART IDLE 路径，连续流量和错误恢复能力不足。

---

## 13. 推荐方案一：标准硬件 DBM

适合固定 18 字节 SBUS 帧。

### 13.1 配置原则

```text
M0 容量 = N
M1 容量 = N
NDTR    = N
DBM     = 1
TCIE    = 1
```

让硬件在一块缓冲区写满后自动切换 CT，不在软件中手动翻转 CT。

### 13.2 TC 时判断已完成缓冲区

```c
if ((dma->CR & DMA_SxCR_CT) != 0U)
{
    /* DMA 当前正在写 M1，M0 刚完成 */
    completed_buffer = buffer[0];
}
else
{
    /* DMA 当前正在写 M0，M1 刚完成 */
    completed_buffer = buffer[1];
}
```

### 13.3 时序

```text
DMA 写 M0
    ↓ TC，硬件 CT=1
DMA 立即写 M1
    ↕ 并行
任务解析 M0
    ↓ M1 TC，硬件 CT=0
DMA 立即写 M0
    ↕ 并行
任务解析 M1
```

### 13.4 过载策略

如果 CPU 在 DMA 再次回到某块缓冲区前仍未处理完成，必须定义：

- 丢弃旧帧；
- 丢弃新帧；
- 增加缓冲块；
- 只保留最新遥控器状态；
- 记录 overrun 计数；
- 进入失联或安全状态。

---

## 14. 推荐方案二：Circular DMA + IDLE

适合裁判系统这种变长、可能粘包或拆包的字节流。

### 14.1 基本方法

使用一块足够大的 Circular DMA Buffer，不使用手动 CT：

```text
UART → Circular DMA Buffer
             ↓
      IDLE / HT / TC
             ↓
      计算新增数据区间
             ↓
      软件 RingBuffer
             ↓
      任务解析协议帧
```

### 14.2 计算 DMA 当前写入位置

```c
uint16_t pos = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
```

若：

```text
pos > old_pos
```

新增区间为：

```text
[old_pos, pos)
```

若：

```text
pos < old_pos
```

说明发生环绕，新增数据为：

```text
[old_pos, BUFFER_SIZE)
+
[0, pos)
```

### 14.3 优点

- IDLE 时无需停止 DMA；
- 不需要手动切 CT；
- NDTR 不会超过实际缓冲区容量；
- 可以处理连续帧、粘包和拆包；
- 协议解析可放入任务；
- 接收盲区更小。

---

## 15. 推荐的寄存器操作顺序

如果必须停止并重配 DMA Stream，应遵循：

```text
1. 清 DMA_SxCR.EN
2. 等待 EN 实际变为 0
3. 清除 DMA TC/HT/TE/DME/FE 标志
4. 配置 PAR
5. 配置 M0AR/M1AR
6. 配置 NDTR
7. 配置 CR 中的 CHSEL/DIR/MINC/PSIZE/MSIZE/CIRC/DBM/CT
8. 清 UART ORE/IDLE 等遗留标志
9. 设置 USART_CR3.DMAR
10. 设置 USART_CR1.IDLEIE
11. 设置 DMA_SxCR.EN
```

运行期间要避免在 Stream 仍使能时修改不允许动态修改的字段。

---

## 16. 调试时应该观察什么

### 16.1 寄存器

建议在调试器中观察：

```text
USARTx->SR
USARTx->CR1
USARTx->CR3
DMAx_Streamy->CR
DMAx_Streamy->NDTR
DMAx_Streamy->PAR
DMAx_Streamy->M0AR
DMAx_Streamy->M1AR
DMAx->LISR/HISR
```

重点关注：

- IDLE 是否置位；
- ORE 是否出现；
- DMAR 是否意外清零；
- EN 是否真的清零；
- CT 是否按预期切换；
- NDTR 在一帧内如何递减；
- TC/HT/TE 标志是否出现。

### 16.2 软件计数器

建议增加：

```c
volatile uint32_t uart_idle_count;
volatile uint32_t dma_tc_count;
volatile uint32_t uart_ore_count;
volatile uint32_t invalid_size_count;
volatile uint32_t buffer_overrun_count;
```

### 16.3 GPIO 时序测量

在中断入口和出口翻转 GPIO：

```text
GPIO 高：进入 UART IRQ
GPIO 低：退出 UART IRQ
```

用逻辑分析仪同时观察 UART RX 和 GPIO，可以测量：

- 从最后一个停止位到 IDLE ISR 的延迟；
- ISR 执行时间；
- DMA 关闭窗口；
- 下一帧是否在 DMA 重启前到达。

---

## 17. 面试解释模板

可以这样概括该设计：

> USART3 和 USART6 的 RX DMA 配置为 Circular，并额外开启 DBM；M0AR 和 M1AR 分别指向两块接收缓冲区。UART IDLE 中断用于判断一段数据结束，HAL 通过 `RxXferSize-NDTR` 计算 Size。处理函数读取 `DMA_SxCR.CT` 判断当前 Memory，关闭 DMA 后手动切换 CT，解析旧缓冲区，再重装 NDTR 并恢复 DMAR、IDLEIE 和 DMA EN。代码将 RxXferSize 和稳定阶段 NDTR 设置为单块容量的两倍，使收到一帧 N 字节时 Size 等于 N。但该设计存在首次 NDTR 不一致、NDTR 大于实际单块容量、异常连续流量越界、解析期间停收和错误恢复不足等问题。固定长度 SBUS 更适合标准 DBM+TC，变长裁判数据更适合 Circular DMA+IDLE+软件 RingBuffer。

---

## 18. 关键寄存器速查表

| 寄存器/位 | 作用 | 本设计用途 |
|---|---|---|
| `USART_SR.IDLE` | 接收线路空闲 | 判断一次接收结束 |
| `USART_SR.RXNE` | 接收数据寄存器非空 | DMA 读取 DR 后清除 |
| `USART_SR.ORE` | 接收溢出 | DMA 关闭窗口内可能发生 |
| `USART_DR` | UART 接收/发送数据 | DMA 外设源地址 |
| `USART_CR1.IDLEIE` | IDLE 中断使能 | 允许 IDLE 进入 USART IRQ |
| `USART_CR1.RE` | 接收器使能 | 允许 UART 接收 |
| `USART_CR1.UE` | USART 总使能 | 启用 USART |
| `USART_CR3.DMAR` | UART RX DMA 请求使能 | 连接 UART RX 与 DMA |
| `USART_CR3.EIE` | UART 错误中断使能 | ORE/FE/NE 错误处理 |
| `DMA_SxCR.EN` | DMA Stream 使能 | 启停 DMA |
| `DMA_SxCR.DBM` | 双存储区模式 | 启用 M0/M1 |
| `DMA_SxCR.CT` | 当前目标 Memory | 判断/切换 M0、M1 |
| `DMA_SxCR.CIRC` | 循环模式 | NDTR 到 0 后继续运行 |
| `DMA_SxCR.DIR` | 数据方向 | 外设到内存 |
| `DMA_SxCR.MINC` | 内存地址递增 | 连续写入数组 |
| `DMA_SxCR.PINC` | 外设地址递增 | 关闭，固定读取 DR |
| `DMA_SxCR.MSIZE` | 内存数据宽度 | Byte |
| `DMA_SxCR.PSIZE` | 外设数据宽度 | Byte |
| `DMA_SxCR.CHSEL` | DMA Channel | USART3 Ch4、USART6 Ch5 |
| `DMA_SxCR.TCIE` | 传输完成中断 | 标准 DBM 应开启 |
| `DMA_SxCR.HTIE` | 半传输中断 | 流式接收可使用 |
| `DMA_SxCR.TEIE` | 传输错误中断 | 错误恢复 |
| `DMA_SxNDTR.NDT` | 剩余传输数量 | 与 RxXferSize 计算 Size |
| `DMA_SxPAR` | 外设地址 | 指向 USART_DR |
| `DMA_SxM0AR` | Memory 0 地址 | 第一块接收缓冲 |
| `DMA_SxM1AR` | Memory 1 地址 | 第二块接收缓冲 |
| `DMA_SxFCR` | FIFO 配置 | 当前使用 Direct Mode |
| `DMA_LISR/HISR` | DMA 状态标志 | TC/HT/TE/DME/FE 状态 |
| `DMA_LIFCR/HIFCR` | DMA 标志清除 | 清除对应中断标志 |

---

## 19. 总结

该设计的核心机制为：

```text
M0AR/M1AR 提供两块 DMA 内存
        +
DBM 开启双目标模式
        +
CT 指示或切换当前目标
        +
NDTR 记录剩余数量
        +
UART IDLE 判断一次接收结束
        +
HAL 使用 RxXferSize-NDTR 计算 Size
```

稳定阶段通过：

```text
RxXferSize = 2N
NDTR       = 2N
收到 N 字节后 Size=N
```

再由软件停止 DMA、翻转 CT、解析当前缓冲区，实现交替接收。

但是当前实现需要特别注意：

1. 初始化时 `RxXferSize=2N`、`NDTR=N`，首次 Size 不正确；
2. 首个满帧可能在硬件自动切 CT 后选错缓冲区；
3. `NDTR=2N` 大于单块实际容量 N，缺少 IDLE 时可能越界；
4. 数据解析期间 DMA 关闭，并未真正实现处理与接收并行；
5. USART6 短数据路径没有重装 NDTR；
6. DMA TC/HT/Error 和 UART ORE 等恢复路径不完整。

固定长度 SBUS 推荐使用标准 DBM+TC；变长裁判系统数据推荐使用 Circular DMA+IDLE+软件 RingBuffer。这样更容易保证连续接收、内存安全和实时性。
