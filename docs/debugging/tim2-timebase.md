# 一次"板子完全不动"的定位过程:空的 TIM2_IRQHandler

STM32H723VGTx / arm-none-eabi-gcc 15.2 / OpenOCD 0.12.0 / WCH CMSIS-DAP

本文记录 2026-08-07 排查"H7 移植后固件烧进去毫无反应"的完整过程。写下来的理由不是这个 bug 特别难,而是**它的表象离根因隔了四层**,而排查过程中有两个独立的错误在互相掩护 —— 那种情况下按症状找原因会一直找错方向。

---

## 1. 最初看到的

VS Code 里按 F5,得到:

```
0x0801cb1e in UART_SetConfig (huart=0x0) at stm32h7xx_hal_uart.c:3085
Program stopped, probably due to a reset and/or halt issued by debugger
xPSR: 0x61000000 pc: 0x20000064 psp: 0x2000a410
rtt: No control block found
Failed to read memory at 0x20020000
...
[stm32h7x.cpu0] clearing lockup after double fault
```

四条线索,**其中三条是误导**:

| 线索 | 看起来像 | 实际 |
|---|---|---|
| `UART_SetConfig (huart=0x0)` | UART 用了空句柄 | 无关。GDB 附加瞬间的任意位置 |
| `pc: 0x20000064` | 跑到 RAM 里执行 | 是后果,不是原因 |
| `No control block found` | RTT 没配好 | 真的没配好,但与崩溃无关 |
| `double fault` | 栈溢出? | 是后果,隔了四层 |

`0x20000064` 用 `nm` 查会落在 `USBD_Interface_fops_HS` 附近,于是很容易开始查 USB。那是错的方向 —— 那个地址在 `.data` 段里,GDB 只是报了最近的符号。

---

## 2. 两个互相掩护的问题

### 2.1 探针的复位线没接

`openocd_dap.cfg` 当时写的是:

```tcl
reset_config srst_only srst_nogate connect_assert_srst
```

这假定探针的 nRESET 接到了目标。实际没接,而 **OpenOCD 不会报错** —— 它以为复位成功了。

后果是:每次 `reset halt` 之后,看到的都是**上一轮崩溃的残留状态**,不是新的复位状态。

判断方法很简单 —— `reset halt` 之后 PC 应该在 `Reset_Handler`:

```bash
openocd -f openocd_dap.cfg -c "init" -c "reset halt" -c "reg pc" -c "shutdown"
```

坏的情况(复位没生效):

```
current mode: Handler HardFault
pc: 0x2000002e          ← 还在上一轮的 fault 里
```

改成 `reset_config none`(走调试单元的 SYSRESETREQ,不依赖复位线)之后:

```
current mode: Thread
pc (/32): 0x080032a0    ← Reset_Handler 起始地址
msp (/32): 0x20020000   ← DTCMRAM 顶部
CFSR/HFSR: 0x00000000   ← 干净
```

**这一步必须先做。** 在此之前所有的观察都不可信,包括寄存器和栈。

### 2.2 RTT 搜索范围越界一个字

```
rtt setup 0x20000000 0x20000 "SEGGER RTT"
```

`0x20000000 + 0x20000` = `0x20020000`,正好是 DTCMRAM 末尾**之后**一个字。OpenOCD 读到那里报 `Failed to read memory`,然后**放弃整个扫描**:

```
Failed to read memory at 0x20020000
rtt: No control block found
```

改成 `0xB000`(44 KB)即可 —— `_SEGGER_RTT` 实测在 `0x20009184`。

顺带一个容易踩的点:**`rtt setup` 必须在固件跑起来之后执行**,控制块是运行时初始化的。`-c "rtt setup" -c "reset run"` 这个顺序找不到东西。

---

## 3. 真正定位:一次栈回溯

清掉上面两个障碍后,不再猜,直接问栈:

```bash
# 后台起 openocd
openocd -f openocd_dap.cfg -c "init" -c "reset run" &

cat > /tmp/bt.gdb <<'EOF'
set confirm off
set pagination off
target extended-remote localhost:3333
monitor halt
bt
EOF
arm-none-eabi-gdb -q --batch -x /tmp/bt.gdb build/COD_UniFramework_H7.elf
```

一次就出来了:

```
#0  TIM2_IRQHandler () at stm32h7xx_it.c:304
#1  <signal handler called>
#2  HAL_GetTick () at stm32h7xx_hal.c:340
#3  HAL_Delay (Delay=10) at stm32h7xx_hal.c:416
#4  USB_SetCurrentMode (mode=USB_DEVICE_MODE) at stm32h7xx_ll_usb.c:276
#5  HAL_PCD_Init at stm32h7xx_hal_pcd.c:182
#6  USBD_LL_Init at usbd_conf.c:338
#7  USBD_Init at usbd_core.c:138
#8  MX_USB_DEVICE_Init () at usb_device.c:71
#9  main () at main.c:140
```

读法:**`main()` 卡在第 140 行的 `MX_USB_DEVICE_Init()`,里面在等 `HAL_Delay(10)`,而 CPU 实际一直在 `TIM2_IRQHandler` 里。**

一行就够了:`HAL_Delay` 等的是 `uwTick`,`uwTick` 由 TIM2 中断累加,而 CPU 出不了那个中断 —— 说明中断标志没被清。

打开 `stm32h7xx_it.c:304`:

```c
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}
```

空的。没有 `HAL_TIM_IRQHandler(&htim2)`。

---

## 4. 因果链:一行缺失如何变成 double fault

TIM2 是本项目的 HAL timebase(不是 SysTick,那个整个归 FreeRTOS)。缺了那行的后果是连锁放大的:

```
TIM2_IRQHandler 不清 UIF 标志
  → 中断返回后立刻重入          → CPU 100% 在 handler 里
  → uwTick 永不递增             → HAL_GetTick() 冻结
  → HAL_Delay() 永不返回        → main() 卡在 MX_USB_DEVICE_Init()
  → Board_Init 从未执行         → 调度器从未启动
  → 中断反复入栈耗尽 MSP        → double fault → lockup
```

所以最终看到的 `clearing lockup after double fault` 与根因隔了**四层**,而且落地的 PC 在 USB 描述符里,长得像 USB 的问题。

---

## 5. 为什么 CubeMX 会漏掉这一行

不是随机疏漏 —— 生成器把 TIM2 走了另一条路径,而两条路径之间有缝。

TIM2 的 tick 需要三段代码接起来:

```
TIM2 硬件中断
  → TIM2_IRQHandler                       (stm32h7xx_it.c)      ← 缺这段
  → HAL_TIM_IRQHandler(&htim2)            (HAL 驱动)
  → 查注册表找回调
  → TimeBase_TIM_PeriodElapsedCallback    (timebase_tim.c:143)
  → HAL_IncTick()
```

CubeMX **正确生成了首尾两段**:

- `stm32h7xx_hal_timebase_tim.c:104` — 用 `HAL_TIM_RegisterCallback` 注册回调
- `stm32h7xx_hal_timebase_tim.c:148` — 回调里调 `HAL_IncTick()`
- `stm32h7xx_hal_conf.h:217` — `USE_HAL_TIM_REGISTER_CALLBACKS 1U` 也开了

唯独中间那段是空壳。

### `.ioc` 里能看出原因

NVIC 每条中断是一串冒号分隔的布尔位,**第 8 位控制"是否生成 HAL handler 调用"**。逐条比对本项目全部 40 个中断,规律一致:

| 第 8 位 | 数量 | it.c 里有 HAL 调用 |
|---|---|---|
| `true` | 13 | 全部有(FDCAN×6、UART×6、OTG_HS) |
| `false` | 27 | 全部没有 |

而 TIM2 是**唯一的例外** —— 第 8 位是 `false`,却是那 27 个里唯一一个必须有 handler 调用才能工作的:

```
NVIC.TIM2_IRQn=true\:15\:0\:false\:false\:true\:false\:false\:true
                                                     ↑ 第 8 位
NVIC.FDCAN1_IT0_IRQn=true\:5\:0\:false\:false\:true\:true\:true\:true
                                                          ↑ 对照
```

另外 26 个 `false` 的(DMA、SPI2、EXTI、以及四个 fault 向量等)之所以无害,是因为它们在本项目里没有被使用,或者由框架自己实现(见 `04_impl/rtos/freertos/rtos_fault.c`)。

原因在 `.ioc` 第 596 行:

```
NVIC.TimeBase=TIM2_IRQn
NVIC.TimeBaseIP=TIM2
```

CubeMX 把 TIM2 当作**系统 timebase** 特殊对待 —— 生成专门的 `timebase_tim.c`、注册回调、使能中断 —— 同时忘了它终究还是个需要清标志的普通定时器。**特殊路径与通用路径之间的缝。**

---

## 6. 修法

在 USER CODE 区补一行(放 USER CODE 内,才能扛住下次 Generate Code):

```c
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE END TIM2_IRQn 0 */
  ...
}
```

`htim2` 在 `stm32h7xx_it.c:85` 已有 extern 声明,不需要额外加。

验证方式不是"看起来能跑了",而是查符号 —— 修之前 `HAL_TIM_IRQHandler` **根本不在镜像里**(无调用点,被 `--gc-sections` 丢了):

```bash
arm-none-eabi-nm build/COD_UniFramework_H7.elf | grep -w HAL_TIM_IRQHandler
# 修之前:空
# 修之后:0800cba4 T HAL_TIM_IRQHandler
```

修好后板子的实际输出:

```
RTOS: starting scheduler (FreeRTOS V11.3.0)
control: motor not seen in 2 s; starting anyway
stack free: control 1700/2048 B, monitor 788/1024 B
```

---

## 7. 可复用的东西

### 排查顺序

1. **先确认调试器本身可信。** `reset halt` 后 PC 是否在 `Reset_Handler`、CFSR/HFSR 是否为 0。不成立就先修调试配置 —— 否则后面每一次观察都可能是在看过期状态。
2. **用栈回溯,不要用寄存器猜。** 这次 `bt` 一次给出九层完整调用链,而读 CFSR/BFAR 只告出"BusFault + 地址是垃圾",指不出方向。
3. **区分症状与根因。** `double fault`、`pc` 在 RAM 里、`huart=0x0` 全是下游现象。往上游走,不要在崩溃点附近打转。
4. **改完查符号,不查"能跑"。** `nm` 能证明 `HAL_TIM_IRQHandler` 从"不在镜像里"变成"在"。

### `HAL_Delay` 永不返回 = timebase 断了

这是一个特征很强的症状。`HAL_Delay` 只依赖 `uwTick`,而 `uwTick` 只由 timebase 中断累加。卡在 `HAL_Delay` 里,基本只有三种可能:

- timebase 中断的 handler 没清标志(本次)
- timebase 中断没使能,或优先级被更高优先级长期屏蔽
- `HAL_Init` 之前就调了 `HAL_Delay`

FreeRTOS 项目里还有第四种:调度器启动后在临界区里调 `HAL_Delay`。

### 这类问题在本项目已出现三次

| 时间 | 现象 | 根因 |
|---|---|---|
| F4 时期 | 整个 `PLAT_Task_*` 层从未被链接 | CMSIS-RTOS 抢走了任务创建 |
| H7 移植 | 固件构建通过但什么都不做 | `USER CODE BEGIN 2` 被清空,框架整体被 gc-sections 丢弃 |
| H7 移植 | 板子完全不动 | 本文:`TIM2_IRQHandler` 空 |

三次都是同一类:**代码生成器认为某段代码"不属于用户",于是既不生成也不保留。**

这也是根 `CMakeLists.txt` 只 include 而不复制 vendor 半边的理由。但 `main.c` 和 `stm32h7xx_it.c` 里的 USER CODE 区仍然要人工守住 —— 现在有三个具体案例说明为什么。

### 抓 RTT 的最短路径

RTT server + `nc` 需要两个进程共存。如果只是想看一眼输出,直接用 gdb 读缓冲更省事:

```bash
openocd -f openocd_dap.cfg -c "init" -c "reset run" -c "sleep 40000" &
sleep 5
cat > /tmp/r.gdb <<'EOF'
set confirm off
set pagination off
set height 0
set remotetimeout 20
target extended-remote localhost:3333
monitor halt
printf "WrOff=%d RdOff=%d\n", _SEGGER_RTT.aUp[0].WrOff, _SEGGER_RTT.aUp[0].RdOff
printf "MSG: %s\n", _SEGGER_RTT.aUp[0].pBuffer
EOF
arm-none-eabi-gdb -q --batch -x /tmp/r.gdb build/COD_UniFramework_H7.elf
```

`WrOff > RdOff` 就说明固件确实写了东西 —— 这一条能立刻区分"固件没输出"和"我没读到"。本次排查中就靠它确认了 147 字节已经躺在缓冲里,问题只在读取侧。
