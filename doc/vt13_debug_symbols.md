> 2026-11-25：以下仅为历史记录。用户已要求撤回全部VT13隔离测试，本文涉及的实验宏、初始化诊断及扩展UART字段已从当前代码移除；不要再执行旧试验步骤或加载其中旧AXF用于当前固件。当前入口见[VT13操控说明](vt13_control.md)。

# VT13 客户端对照中的调试符号核对

## 2026-11-24 两张截图的异常

用户报告：关闭客户端时初始化从未成功，开启客户端也发生过一次初始化后失能，再次初始化成功。客户端开启并不保证每次成功，不能把客户端状态直接认定为唯一根因。两张图都在 C 档，不能由图内档位判断 N/S 是否正常发布。

截图中 `vt13_uart_stats=0x2000B3A0`，没有 `parser.keymouse_invalid_frames`，`gimbal_init_debug=0x2000A12C`；这些地址及结构匹配磁盘上 **20:23 的旧 Keil AXF**。但图内 `input_mode` 为 0x3DF/0x393、`rx_poll_enabled=0x04000400`，均不是对应字段的合法运行值。`captured=0` 时旧记录枚举位置还出现类似浮点值的位图。不能按这些错位字段宣布 UART 启动失败、CRC 出错或接收停流。

本机 `fromelf --text -s` 核对：

| 产物 | gimbal_init_debug | vt13_uart_stats | 串口结构大小 |
|---|---|---|---|
| `MDK-ARM/standard_tpye_c/standard_tpye_c.axf`，20:23 | 0x2000A12C | 0x2000B3A0 | 84 字节 |
| `MDK-ARM/vt13-control-verification/client-independent-20261124/eide-vt13/standard_robot.axf`，20:49 | 0x2000A53C | 0x2000B38C | 88 字节 |

20:49 产物与上一轮保存的 304 个源码/头文件/工程/链接输入散列仍一致。地址只适用于这两个具体 AXF；下一次链接可能变化，不能写死到生产代码或长期 Watch 指针。

## 候选还原与限制

旧符号的串口基址比修正版高 20 字节，而且旧 parser 结构少一个 4 字节字段。按修正版布局逐个地址对照，能还原出下表。该还原**以板上实际运行 20:49 修正版为条件**，不是完整板上代码身份校验；仍需加载匹配符号确认。

| 修正版字段 | 无客户端图 | 有客户端图 |
|---|---:|---:|
| parser.bytes | 180927 | 166941 |
| parser.valid_frames | 8615 | 7950 |
| parser.crc_errors | 0 | 0 |
| input_age_ms / last_byte_age_ms | 5 / 7 | 13 / 8 |
| max_input_age_ms | 308 | 308 |
| timeout_events | 68 | 61 |
| input_valid / input_mode / rx_poll_enabled | 1 / C / 1 | 1 / C / 1 |

例如旧图的 `start_failed` 实际地址正好是修正版的 `parser.valid_frames`；旧 `parser.crc_errors` 正好是修正版 `max_input_age_ms`。错位后的单字节 `running` 不足以恢复完整的基础字段错误计数，不作猜测。

若还原得到确认，退出后当时仍有新鲜遥控输入，但存在累计超时；不是“退出客户端后完全停发”的证据。两图 Watch 字段异步刷新，计数并非同拍；累计最大帧龄和超时数也不能归属到某一次初始化，不能直接据此延长超时或取消失联停机。

## 仅重载本次修正版的符号

保持 C 档、确认电机已停，在当前工程的 Keil Command 中执行：

```text
LOAD vt13-control-verification/client-independent-20261124/eide-vt13/standard_robot.axf NOCODE CLEAR INCREMENTAL
```

`NOCODE` 只加载符号，不覆盖程序；`CLEAR INCREMENTAL` 在 Cortex-M 上替换旧符号而不重置目标。依据 [Keil LOAD 文档](https://www.keil.com/support/man/docs/uv4/uv4_cm_load.asp)。若命令要求停止，只在 C 档停止后执行，完成后恢复 Run；不带动力打断点。独立命令文件已保存到本轮忽略产物，但 `INCLUDE` 官方要求先停止程序，因此此处直接使用 `LOAD`，不在 N/S 执行命令文件。

重载后应看到 `vt13_uart_stats=0x2000B38C`、`sizeof(vt13_uart_stats)=88`，并新增 `parser.keymouse_invalid_frames`；`gimbal_init_debug.test_input_loss_latch` 应与当前隔离配置 1 一致。只加载符号不能证明板上镜像身份完全一致；若这些值仍不合理，先核对实际下载的 AXF/HEX 来源，不再依据结构内垃圾值修改控制条件。

当前继续只检查退出客户端后的 `input_valid/input_age_ms/timeout_events` 和上述配置标记，不处理登录后探头不可用的问题，也不改初始化/接收保护。下一步根据匹配后的实际输入和初始化状态决定修正位置。

原图、两产物散列、逐字段地址还原检查和符号命令文件位于 `MDK-ARM/vt13-control-verification/client-symbols-20261124/`。本轮未修改生产源码、参数或 Keil 工程，未重跑固件构建、烧录、提交或推送；只读符号核对不是实车故障已经解决。
