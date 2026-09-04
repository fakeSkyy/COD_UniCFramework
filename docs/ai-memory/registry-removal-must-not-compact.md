---
name: registry-removal-must-not-compact
description: 注册表的 Remove 用墓碑法就地清 key,压缩表会让查无关 key 的 ISR 短暂漏掉活跃条目
type: decision
verified: 2026-09-02,28 个 util_registry 用例 + 4 次变异测试(压缩/清序/ForEach 守卫/槽位复用)
agents: claude
---

# 注册表的删除为什么不压缩表

`UTIL_Registry_Remove`(2026-09-02 加入)就地清 key 退休一个槽位,**不压缩表**。所以 `count` 是
**高水位**而不是活跃条目数,`Add` 会优先复用被退休的槽位再考虑增长。

## 为什么不压缩

压缩(把末尾条目搬进空洞)更整洁,但会破坏 `Find` 的无锁保证。

要搬动条目就必须先减 `count`,而在**减完到搬完之间**,被搬动的那个条目不可达 —— 一个查
**无关 key** 的 ISR 在那个窗口里会漏掉一个仍然活跃的条目。原型实测确认过这个窗口。

就地清 key 不可能影响任何其他条目:`Find` 按 key 匹配,而且没有东西移动。

清的顺序也是承重的,**key 先 value 后**。反过来会留下一个活 key 绑到 NULL,而 CAN 的接收路径把
那个读成「还没注册回调」(`c->rx_cb == NULL`)而不是「不存在」—— 两者行为不同。

## 起因:一个中断上下文里的 use-after-free

`IMPL_STM32_CAN_DestroyCtx` 原来只做 `IMPL_free(ctx)`,而 `CreateCtx` 已经把这个 ctx 按接收 id
注册进了 `bus->route`。free 之后路由表留着野指针,而接收路径在**中断里**解引用 lookup 的结果
(`c->rx_cb`,然后 `c->arg`)。

头文件当时把「路由表项比 context 活得久」写成了设计选择,类比共享的总线记录。**那个类比不成立**:
总线记录是共享的、被同一句柄上的下一个节点幂等重用;路由表项只属于这一个 context,没有任何东西
会替换它。

一直没炸的唯一原因是生产代码里没有任何调用者销毁 CAN 节点(`Board_CANCreate` 零调用点)。

## 同一个理由适用于范围表,而且我第一版写错了

H7 的 CAN 后端除注册表外还有一张**范围表**(`route_find` 在注册表未命中后查它),它同样需要在
`DestroyCtx` 里退役。我第一版写的是压缩,还在注释里说明"这里压缩是安全的" —— **然后去验证那句话,
发现它是假的**:被销毁节点的滤波器仍然装着,帧继续到,ISR 可能正在扫这张表,压缩会让被移动的条目
短暂不可见,于是**一个无关范围**的帧被丢掉。改成原地墓碑(`owner = NULL`,`first = CAN_STD_ID_MAX`,
`last = 0`,即一个匹配不到任何 id 的空区间)。

**一张被 ISR 无锁扫描的表,不管它是哪张,都不能压缩。** 参见 [[can-range-claim-not-mask]]。
另外:注释里写"这样做是安全的"时,那句话本身就是待验证的断言。

## 怎么应用

- 要 free 一个已注册的值,**先 `Remove` 再 free**。
- 给注册表加删除相关功能时不要「顺手压缩一下」。
- `ForEach` 必须跳过 key 为 NULL 的槽位 —— `count` 仍然跨过它们。

## 测试上的一个盲区

CMock 的 `IMPL_free` **不污染内存**,所以主机测试看不到 use-after-free 本身:被 free 的 context
读起来仍然完好。回归测试只能断言**可观察行为** —— 销毁后的帧不触发回调、以及那个接收 id 可以被
重新占用。用旧的有缺陷版本跑,该用例报 `Expected 0 Was 1`。

另一个坑:`ctest -R 'impl_stm32_can'` 匹配的是别的目标,新用例还必须加进
`tests/impl/bsp/stm32h7/CMakeLists.txt` 的 CASES 列表才会被 CTest 收录 —— 否则它编译了、能单独
跑,但从不参与 `ctest`。参见 [`host-tests-blind-spots`](host-tests-blind-spots.md)。
