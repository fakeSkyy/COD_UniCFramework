# HardFault 处理

CubeMX 生成的四个故障处理器是裸 `while (1)`，什么都不报。这个框架把它们换成了一套诊断：`04_impl/rtos/freertos/rtos_fault.c`，311 行。

---

## 1. 换掉的是什么

CubeMX 的版本：

```c
void HardFault_Handler(void)
{
  while (1) { }
}
```

故障发生后，调试器显示的 PC 在这个 `while` 里，**所有有价值的寄存器都已经被覆盖**。而内核其实保留了一切 —— 出错的 PC、故障状态位、越界地址 —— 只是躺在内存和 SCB 里没人读。

---

## 2. 四个处理器都是 `naked`

```c
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile("mov r0, lr           \n"   /* lr 此刻是 EXC_RETURN */
                   "movs r1, #0          \n"   /* 哪一种故障 */
                   "b   rtos_fault_dispatch\n");
}
```

实际固件里确认无 prologue：

```
08008a38 <HardFault_Handler>:
 8008a38:  mov  r0, lr              ← 第一条指令就是它
 8008a3a:  movs r1, #0
 8008a3c:  b.w  rtos_fault_dispatch
```

**为什么必须 naked。** 普通函数里编译器会先插入 prologue 建立栈帧，而故障处理器要读的恰好是 prologue 会破坏的两样东西：

| | naked | 非 naked |
|---|---|---|
| `lr` | 仍是 `EXC_RETURN` | `push {lr}` 后被后续调用覆盖 |
| 栈指针 | 一字节未动 | `sub sp, #N` 已移位，N 随编译器而变 |

实测非 naked 版本的 prologue（同一个函数体，三种写法）分别是 `push {r3, lr}`、`push {lr}; sub sp,#12`、`sub sp, #8` —— 偏移量取决于优化级别和函数内容，硬编码就是在赌编译器。

更糟的是 `push` **会写内存**，写在紧邻故障帧的位置。如果这次故障本身就是栈溢出，那两次写可能正好压在要读的数据上。

**代价**：函数体只能写内联汇编（没有栈帧，C 局部变量无处安放）。所以用 `b`（跳转）而非 `bl`（调用）把活交给普通 C 函数 —— 跳转不压栈，参数已安全传进 `r0`/`r1`。

**故障种类传小整数而不是字符串指针**：这个架构上 `mov` 取不到字面量地址，而在 naked asm 里走 literal pool 恰好是故障处理器最不该有的那种脆弱。

---

## 3. 选对栈：一个读错就全错的判断

```c
const Fault_Frame_s* frame = (exc_return & 0x4u) ? (Fault_Frame_s*) __get_PSP()
                                                 : (Fault_Frame_s*) __get_MSP();
```

`EXC_RETURN` 的 bit 2 说明八个字压在哪个栈上。**读错的话打印出来是八个字的垃圾 —— 那比不打印更糟，因为它看起来像真数据。**

FreeRTOS 下这一点是致命的：任务代码跑在 PSP 上，中断跑在 MSP 上，两者都会故障。

这也是为什么**不能把 CubeMX 的处理器重定向到 USER CODE 区** —— 那个插入点在 prologue 之后，`EXC_RETURN` 已经不在 `lr` 里了，无从判断该读哪个栈。

---

## 4. 解码 12 个故障位，写的是含义不是位名

```
INVSTATE:    Thumb bit clear — bad function pointer      ← 跳进数据了
UNDEFINSTR:  not an instruction — executing data?
PRECISERR at 0x%08X                                      ← BFAR 有效
IMPRECISERR: async bus error, PC and BFAR unreliable     ← 异步,两者都不可信
MSTKERR:     stacking failed — stack overflow
DACCVIOL:    data access denied at 0x%08X                ← MMFAR
IACCVIOL / MUNSTKERR / UNALIGNED / DIVBYZERO / IBUSERR
VECTTBL:     bad vector fetch — VTOR or the vector table is wrong
```

两条最容易误诊的都特别标注了：

- **`IMPRECISERR`** 标明"PC 和 BFAR 都不可靠"。在 H7 上它通常是**写一个时钟没开的外设**，而报出来的 PC 已经越过那条 store 了 —— 不说清楚会让人对着一个无关的行号查半天。
- **`FORCED`** 打印 `escalated from a configurable fault`。默认配置下几乎每个 HardFault 都是升级来的，所以这一行是在说"真正的原因在 CFSR 里，这个 HardFault 只是信使"。

最后还打印 `msp` / `psp`：PSP 落在任务栈数组之外、或 MSP 越过 `_estack`，那就是栈溢出，不管 CFSR 说什么。

---

## 5. `RTOS_FaultInit`：把那三个处理器变成可达的

```c
SCB->SHCSR |= MEMFAULTENA | BUSFAULTENA | USGFAULTENA;
SCB->CCR   |= DIV_0_TRP;
__DSB(); __ISB();
```

不开这三位，`MemManage`/`BusFault`/`UsageFault` 三个处理器**根本不可达** —— 内核出厂时把它们关着，一律升级成 HardFault。报告仍然会产生（HardFault 也解码 CFSR），但**恢复出来的帧可能属于升级过程而不是原始访问**，而那个帧的 PC 是整份报告的全部价值。

`DIV_0_TRP` 也开了：整数除零默认返回 0 并继续执行，对控制代码是危险默认 —— 一个陈旧的除数会产出一个看起来合理的数字而不是停下来。

`UNALIGN_TRP` **故意不开**：非对齐访问在这个核上合法，编译器会生成，开了会在正确代码上故障，包括 vendor 库里面。

调用点在 `app_tasks.c:108`，任何任务能故障之前。

---

## 6. 走 RTT 而不是 UART

故障可能发生在中断被屏蔽、UART 的 DMA 半配置好的状态。RTT 是一次内存写加调试器轮询，**不需要任何外设工作**。

而这个文件**故意不用 `UTIL_LOG_*`** —— 全框架唯一的例外。理由：故障处理器运行在已经出错之后，栈可能快耗尽，故障本身还可能是通过损坏的函数指针跳来的。每多一层就多一次"在报告故障时自己故障"的机会，那会变成静默死锁 —— 正是这个文件要防的事。

`util_log` 还会带来一个能屏蔽这段输出的运行时等级检查，以及每行一个前缀，而这里要的是一整块带标签的报告。

---

## 7. CubeMX 那四个被删掉了，而且删得响亮

`stm32h7xx_it.c` 里现在没有这四个函数（已确认为 0 处定义），只留一段注释说明。

关键在于 CubeMX 生成的版本**不在任何 USER CODE 区内**，所以 Generate Code 会把它们恢复回来。放到框架代码里定义，使得冲突变成：

```
multiple definition of `HardFault_Handler'
```

**响亮、立刻、点名两个文件。** 比静默恢复成 `while(1)` 好得多。

这条判据值得单独说：链接脚本的 AXI SRAM 段**没有**这个性质 —— 删掉一个段不会撞任何东西，只会让 DMA 缓冲区静默失效。所以那个改动不做，而这个做。**判据不是"文件归谁"，而是"改动被回退时会不会有人立刻知道"。**

---

## 8. 另外三个兜底

`rtos_hooks.c`：

| 钩子 | 报什么 | 为什么值得 |
|---|---|---|
| `vApplicationStackOverflowHook` | **任务名** | 栈大小是 `PLAT_Task_Create` 唯一无法检查的参数（调用方给的）。太小不会创建失败，只会静默覆盖 linker 放在数组后面的东西，症状出现在别处。`configCHECK_FOR_STACK_OVERFLOW = 2` |
| `vApplicationMallocFailedHook` | 当前 `configTOTAL_HEAP_SIZE` | 给出**原因**：没有它，"堆小了 32 字节"和"外设真的初始化失败"是同一个 `Error_Handler()` |
| `RTOS_AssertFailed` | file:line | 消息在关中断**之前**发出，否则半条消息会成为最后的输出 |

三个都不返回。栈溢出钩子里中断已关、调度器正在切换中，没有可返回的地方 —— 要返回到的那个栈就是被破坏的那个。

---

## 未验证的部分

**这套东西没有在硬件上触发过。** 代码路径、寄存器位、`naked` 的正确性都是静态检查加反汇编确认的；实际故障时报告能否完整打出来（尤其栈快耗尽的情况），没测过。

验证方法：临时加一句 `*(volatile uint32_t*)0 = 1;`，应该看到 `PRECISERR at 0x00000000`。
