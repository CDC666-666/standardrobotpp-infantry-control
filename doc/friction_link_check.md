# N 档 Fn 摩擦轮无法关闭检查

> 2026-11-27 提交前复核：47 项 VT13 主机回归通过，但 `test_can_dji.py` 的 24 个构造场景中有 18 个失败，仍可复现共享发送帧串总线/数据；历史“已修正”不代表当前生产代码。本轮仅归档现有改动，不恢复旧 CAN 修正，也未烧录或独立实车验证。用户确认功率控制此前失败源于裁判系统，这一结论不自动解释摩擦轮现象或排除独立 CAN 缺陷。

2026-11-25，适用 StandardRobotpp `master@ec77f6b` 及当前未提交工作区。用户确认：接入选手端、N 档、不按鼠标右键，第一次左 Fn 开轮，第二次不能停。板上完整固件身份和同期运行值尚未核验，问题未解决。

## 2026-11-26 当前实现：普通关轮只归零速度目标

按用户明确要求，修改VT13普通关轮路径：整车正常使能时，fric_flag=0仍将两轮速度参考置0，但不再在ShootSendCmd中覆盖速度环电流。正常周期保留ShootConsole已计算的零速PID输出；发送前才释放右键/退出有效自瞄、参考仍非0时，按零速重算，防止继续下发旧的开轮驱动电流。关轮拨弹保持禁止，C/失联/云台许可失效仍执行ShootForceDisable零电流及PID清除。

生产改动仅在shoot_fric_trigger.c，不调整PID、速度、电流上限，不恢复当前已撤回的CAN修改或隔离试验。当前普通关轮时，电流可以为非0制动电流；电流大小及停车时间仍需实车验证，不承诺瞬时停转。

发射替身回归已改用实际pid.c/h，检查N/S及双向零速制动、同拍输出保留、稳定零速、第二次Fn关轮仍禁止拨弹、发送前自瞄退出重算及C/云台失效零输出。47项VT13主机回归通过；实际CAN发送仍为替身，现有test_can_dji.py的共享缓冲专项不包含在此47项内，未把它当成已修复。原生构建与最终验证结果以集中手册本次记录和friction-zero-speed-20261126/证据为准。未烧录/实车验证/提交推送。

下方关轮强制零电流的分析、截图及测试结果均为修改前历史；当前行为以上述实现为准。

## 2026-11-26 修改前分析：左轮关轮无力，需求为主动零速控制

用户新实验确认：左轮在fric_flag=0时无力，仍会转，但外物阻停后不再自行转动。该观察与撤去电流后的惯性滑行吻合，不能据此证明此前右轮持续加速的原因。板上镜像仍未完整核验；本轮按用户要求只分析，不改代码、不测试/构建/烧录。

当前磁盘链路明确为：

1. ShootSetMode将fric_flag=0映射为FRIC_NOT_READY。
2. ShootReference将两轮速度参考设为0。
3. 正常使能时，ShootConsole仍运行原速度PID，计算零速所需的制动电流。
4. ShootSendCmd在无自动开轮且手动开关关闭时，再把两轮set.curr强制置0，覆盖刚算出的制动电流。

因此速度目标0已经实现，但当前普通关轮最终撤销了驱动/主动制动，不维持零速闭环。失能时ShootForceDisable也会将电流置0并清PID，但它是C档/失联/云台许可失效的路径，不能与普通关轮合并成同一种带力停机。

用户想要的行为应定义为：在整车正常使能且电机反馈健康时，fric_flag=0立即切换速度参考为0，并保持速度环输出，主动减速至近零并抵抗重新转动；fric_flag=0仍禁止拨弹。C档、失联、反馈异常等保护优先撤销电流，不能因fric_flag也为0而误启零速带力控制。本轮没有实施该方案。

限制：零电流与零速度是不同控制量；目标可立即归零，实际停车受惯量和可用制动力矩限制，不可能物理上瞬间停住。速度闭环也不保持固定角位置。当前摩擦轮PID为Kp70/Ki0/Kd4.3、原始电流指令限幅16000；直接放行现有零速输出可能出现较强制动，需后续验证限流、冲击及回馈能量，不能把现有开轮参数直接视为已验收的制动参数。用户实验只覆盖左轮，右轮异常需保留独立结论边界，不建议继续用外物接触高速轮确认。

版本补充：本轮磁盘master@ec77f6b的CAN_cmd_dji.c已是共享静态缓冲版本，上一轮每调用独占帧的修改当前不在磁盘生产代码中；没有恢复它，也不猜测是谁或为何撤回。下方11月25日修正及构建记录保留为历史，不能当作当前代码或板上已经包含修复。

## 2026-11-25 历史修正：消除 DJI 发送缓冲被抢占串写

用户最新反馈：`fric_flag=0`、两路set电流0时，左轮减速、右轮加速。截图当前反馈为左约+380.34、右约−747.80；单张图只能证明采样值，不能独立证明趋势，板上固件身份及实际总线报文仍待核验。

**[源码及构造执行确认]** 底盘CAN1 0x200和摩擦轮CAN2 0x200共用 `CAN_cmd_dji.c` 的文件静态 `CAN_CTRL_DATA`。底盘写入hcan1后，若被High优先级发射任务抢占，发射任务会把同一缓冲的hcan改为hcan2；底盘恢复后继续写自己的0x200和电流，最终却发往CAN2。新测试在实际DJI源码的临时副本中只插入抢占钩子，BSP用实际函数、HAL用记录替身，确认：

```text
请求：CAN1 0x200 [-3000,-3000,3000,3000]
实际提交给HAL：CAN2 0x200 [-3000,-3000,3000,3000]
```

该路径会绕过 `SHOOT.fric_motor[].set.curr`：即使发射结构中的电流为0，错误底盘报文也可携带非零电流进入摩擦轮总线。前两槽均负时，对正转左轮产生制动方向、对反转右轮产生驱动方向，与用户报告相符。这里的电流为人工构造，不能当成实车采到的−3000，也不能宣布现场全部定因。

**[代码修正]** `MultipleMotorControl()` 改为每次调用使用栈上独占 `CanCtrlData_s`；`CanCmdDjiMotor()` 统一调用它，移除共享帧。HAL在同步调用内将报文写入硬件邮箱，不保留栈缓冲指针。只改变报文所有权，不改电机ID、总线、PID、Fn规则或零电流关轮行为。未改BSP邮箱分配/返回值处理；该层与其他发送模块的并发/发送失败问题仍是独立限制，不能把本修正称作整个CAN栈已无并发风险。

**[软件验证]** 新增 `tests/test_can_dji.py`，24组场景（其中20组构造抢占）覆盖两种外层报文、两种发送入口、字段赋值及BSP取邮箱处的5个抢占点，并检查非法总线及有符号边界。修正前18组失败，修正后全通过；与原VT13回归合计48项主机测试通过。同步更新旧 `diagnose_chassis_link.py` 的CAN检查，独立CAN段通过；没有重跑其中无关的底盘历史工况。HAL替身不验证物理报文、真实邮箱抢占或动力学。

**[原生构建]** 基于当前撤回隔离试验后的工程及配置，默认VT13/旧控制源原EIDE+ARMCC全量构建各0错误、14条既有警告；304个最终输入散列核对一致。使用本次子进程DOTNET_ROLL_FORWARD=Major，不改变系统/工程设置。失败日志、基线散列、构建脚本/日志和独立AXF/HEX在 `MDK-ARM/vt13-control-verification/friction-can-20261125/`。

当前未由Codex烧录、未实车验证、未提交或推送。用户需全量重新编译下载，并使用同次AXF符号验证Fn关轮后两轮不再持续加速；当前关轮仍为零电流滑行，不是主动制动。不要继续按早期只读阶段的“未改生产代码”描述本次工作，也不要执行已标历史的隔离/旧符号加载步骤。

## 最新现场反馈：实体 Fn 已使最终标志归零

用户进一步确认：再次按实体左Fn后，观察到 `SHOOT.fric_flag` 从1变0，并非在Watch中手动改值；摩擦轮仍转，主观感觉逐渐加速。实际转速、电流和CAN报文尚未采集，不能把“加速感觉”当作实测速度曲线，也不能继续用漏掉第二次Fn或正常惯性滑行解释全部现象。

源码复核：关轮时 `ShootReference()` 将两轮速度参考设为0；`ShootConsole()` 仍会按零速参考计算速度环电流，但VT13正常关轮路径在 `ShootSendCmd()` 内再次将两轮电流强制清零。因此当前关轮实现撤销主动制动，不等于瞬间停车；若实际仍持续加速，则需要进一步核实运行版本、最终输出和总线，不能据此直接改Fn或PID。

下一步观察关轮后的四项，保持与当前板上镜像匹配的符号：

```text
\shoot_fric_trigger\SHOOT.REF.fric_speed_ref_R
\shoot_fric_trigger\SHOOT.REF.fric_speed_ref_L
\shoot_fric_trigger\SHOOT.fric_motor[0].set.curr
\shoot_fric_trigger\SHOOT.fric_motor[1].set.curr
```

两轮参考应持续为0，正常关轮的最终发送电流应为0。`set.curr` 同一拍先由Console写入、再由SendCmd覆盖，Watch可能读到中间制动电流，因此一次非零截图不等于最终下发非零；需要多次刷新或最终发送记录佐证。

- 标志0但参考持续非零：继续核对state、任务执行和板上镜像/符号，不能以本机源码推定实车同版。
- 参考0但软件电流反复非零：先区分Console中间值与SendCmd最终值，再定位分支和运行版本。
- 参考及软件电流均0、轮子仍持续加速：转向实际CAN2 0x200、电调/接线与运行身份；软件为0不等于电机已收到零指令。

公共DJI发送函数共用静态 `CAN_CTRL_DATA`，BSP忽略 `HAL_CAN_AddTxMessage()` 返回值；当前云台与发射同为High优先级、底盘为AboveNormal。既有构造抢占已证明共享缓冲可被覆盖，但未证明现场发生或导致本次现象。当前正常发射路径是CAN2 0x200前两槽，云台CAN2 0x1FF、拨弹CAN1 0x1FF，未见本配置另一正常调用直接发同一摩擦轮帧；不在缺乏实车证据时扩大修复。

若现场仍加速，先回C；C也不停则断开电机电源。本轮只核对源码、记录用户反馈和后续观察项，没有生产代码修改或新增实车验证。检查期间新增的 `Src/main.c` HAL_Init行空白差异为外部现场，保留且不归入本轮改动。

## 已核对的链路

1. `vt13_remote.c` 从整帧 bit63 提取 `function_left`；鼠标右键从 bit130..131 提取，二者不重叠。官方 VT03/VT13 v1.0（2025.04）第7页定义左自定义键为按下1、松开0，并非每次按键翻转一次的协议位。来源：[官方使用说明](https://rm-static.djicdn.com/tem/17348/RoboMaster%20Referee%20System%20Video%20Transmission%20Module%20VT03%26VT13%20User%20Guide.pdf)。不据此推定现场帧必然符合预期。
2. `Vt13MapControl()` 把 `function_left` 映射为 `fn_down`；Fn 不依赖客户端键鼠段有效性。
3. 发射任务每拍执行 `ShootSetMode → ShootReference → ShootConsole → ShootSendCmd`。`Vt13UpdateFriction()` 在先见松开后，每个按下沿翻转 `shoot_control.friction_on`；持续按住不重复翻转。
4. 实际开轮条件是 `auto_target || shoot_control.friction_on`，其中 `auto_target = Vt13AutoAimRequested(input) && status.auto_active`。N档要求输入右键为1；N档且输入右键为0时，即使 `auto_active` 为1也不会自动开轮。因此自动开轮覆盖不是已确认的现场解释。
5. N档右键0、Fn关轮后，两轮速度目标为0；`ShootSendCmd()` 会将两轮电流直接清零，并通过 `ShootCanFeed()` 撤销拨弹。当前配置为 CAN2 `0x200` 第1/2槽，电机ID1/2。发送函数被调用不等于物理总线或电机收到报文。

## 本轮验证及限制

在 `tests/test_vt13_firmware.py` 增补 `check_n_fn_second_press_stops()`，编译执行实际发射 C 文件和实际 VT13 控制层。四种组合覆盖右键0、左键0/1、云台 `auto_active` 0/1；人工到速反馈、拨轮持续请求连发，执行松开→首次按下→松开→第二次按下→重复10拍。均验证首次开轮、第二次关轮、速度参考归零、最终两轮及拨弹电流归零。

`python -m unittest discover -s tests -p 'test_vt13_*.py' -v` 共47项通过（新增场景包含在原实际模块测试内）。PID、CAN、遥控快照和云台状态使用硬件替身；本专项未接入真实UART或实际CAN发送函数，不覆盖实物按键脉冲、接收时序、任务抢占和电调行为。现有协议/UART测试独立通过，不能拼接成现场全链路验收。

本轮没有修改生产控制、参数或工程，没有ARMCC构建、烧录、提交或推送。保留全部原有现场修改。已知公共CAN发送缓冲存在可重入风险，但本轮没有证据表明它导致当前停轮失败，不扩大修复范围。

## 直接观察，无需新固件

先核对板上下载镜像与调试AXF来自同一次构建。`vt13_debug_symbols.md` 已随隔离试验撤回标为历史，不执行其中旧LOAD；历史AXF地址不能套用到新构建。程序持续Run，不在N档暂停、单步或设置控制断点。

### Watch 显示 cannot evaluate 时

2026-11-25 用户截图：四个未限定模块的表达式全部显示 `<cannot evaluate>`，这不是运行值为0。源码中的 `shoot_input`、`shoot_control` 和 `SHOOT` 均为文件作用域static。已用ARMCC fromelf核对现有Keil AXF和client-independent-20261124默认VT13 AXF：三个对象均保留为局部数据符号，不能把该截图直接归因为优化删除。

此前直接输入简名的步骤不完整。Keil对未限定的static符号按当前PC所在模块查找；跨模块读取应限定源模块。先将四项替换为：

```text
\shoot_fric_trigger\shoot_input.fn_down
\shoot_fric_trigger\shoot_input.mouse_right
\shoot_fric_trigger\shoot_control.friction_on
\shoot_fric_trigger\SHOOT.fric_flag
```

每行开头是一个反斜杠；模块名不带`.c`。依据[Keil完整符号名规则](https://www.keil.com/support/man/docs/uv4cl/uv4cl_db_exp_symb_fq.htm)和[未限定符号查找规则](https://www.keil.com/support/man/docs/uv4cl/uv4cl_db_exp_symb_nonq.htm)。此法有官方语法和本机符号存在性支持，尚未在用户当前Keil会话验证。

如果仍不可见，打开 `View → Symbol Window`，在Program Variables中找到 `shoot_fric_trigger` 模块，将 `shoot_input`、`shoot_control`、`SHOOT` 拖入Watch并展开成员；以当前Symbols窗口给出的实际完整名字为准。若模块或对象根本不存在，先核对当前加载的AXF、控制源配置和Debug Information，不拿历史地址强行读取。官方[Symbols窗口说明](https://www.keil.com/support/man/docs/uv4cl/uv4cl_db_dbg_symwin.htm)支持从该窗口拖入Watch。需要周期刷新时勾选 `View → Periodic Window Update`；它不能修复符号解析失败。

本次只修正观察方法，不新增全局诊断量、不重编译或烧录；不能因Watch不可见认定必须修改固件。

下表用简名说明各成员含义；实际加入Watch时使用上文完整模块名，均直接读值、不调用函数：

| 变量 | 用途 |
|---|---|
| `shoot_input.mode` | N应为1 |
| `shoot_input.fn_down` | 实际被发射任务消费的Fn输入 |
| `shoot_input.mouse_right` | 确认物理未按右键时收到的值也为0 |
| `shoot_control.friction_on` | Fn手动开轮锁存 |
| `SHOOT.fric_flag` | 最终开轮标志 |
| `SHOOT.REF.fric_speed_ref_R` / `SHOOT.REF.fric_speed_ref_L` | 两轮目标速度 |
| `SHOOT.fric_motor[0].set.curr` / `[1].set.curr` | 当前普通关轮保留零速PID电流，运动中可非0；C/失联仍应为0 |

固定机构并隔离发弹，保持N、右键松开；每一步停留约1秒便于Watch采样：全部松开→按住Fn→松开Fn→再次按住Fn。先比较前三个关键值 `fn_down / friction_on / fric_flag`，正常序列为 `0/0/0 → 1/1/1 → 0/1/1 → 1/0/0`。此后Fn仍按住不应重开。Watch非同步刷新，稳定值仍不足以证明每个瞬态均被记录。

- Fn松开后仍为1，或第二次始终没有新变化：先定位输入、漏掉释放帧、实际固件/符号；不把它当作电机控制问题。
- Fn变化正确但手动锁存未翻转：检查失能复位、异常边沿与板上版本；不能仅凭两张异步Watch图确定事件顺序。
- 手动锁存为0、最终开轮标志仍为1：检查实际 `mode/mouse_right/auto_active`，以及板上镜像与源码是否一致。
- 最终开轮标志、目标速度和软件电流都为0，但轮子持续保持转速：转向实际CAN报文、发送结果、电调与接线；短时间惯性滑行要与持续驱动区分。

发生异常后回C停机。下一步取上述四阶段实际值后再决定修改点，不能将本机回归通过写成现场故障已排除。
