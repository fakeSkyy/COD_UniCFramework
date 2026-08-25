# 主机测试报告

## 现行基线（2026/8/24）

当前普通 host Debug 配置为 **218/218 CTest passed**；启用 instrumentation 的配置为
**216/216 CTest passed**（性能门禁因 instrumentation 会改变时序与 ELF 尺寸而不注册）。这两个
数字是现行全量基线；下文出现的 34/34、58/58 等数字均保留为对应阶段的历史快照，不应解读为
当前最终总数。

现行测试能力还包括三个确定性 property executable：ring buffer、CRC，以及通过真实
`DEV_Remote_Create` 注册 callback 的 DBUS remote。Remote property 明确覆盖 channel
`-660/+660`、mouse `-32000/+32000`、switch 合法端点 1/3，并锁定 timeout tick N-1 在线、
tick N lost 的边界。三个 Clang/libFuzzer harness 默认各执行 512 次 bounded smoke，runner
向每个 harness 传入同一固定且可覆盖的 seed，状态文件记录 seed；缺失 compiler 的负向自测还
证明旧 PASS 会先失效并最终变为无歧义 FAIL。

