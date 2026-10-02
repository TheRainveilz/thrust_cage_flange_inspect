# thrust_cage_flange_inspect

金属推力保持架垫片 —— **双站视觉检测线**。一台 i3-4130T、一块 Arduino UNO，两台传感相机各守一个工位，
各拍各的件、互不配对、互不干扰；一站挂了另一站照跑。

| 站 | 相机 | 判什么 | NG 动作 | 算法 |
|---|---|---|---|---|
| **第一站 · 正反面翻边止口** | `169.254.44.201` | 正/反面朝向 | **D8 吹气**剔除 | `inspector_pure.py`（黄金基线，正 323/352、反 623/623，零逃逸） |
| **第二站 · 兜孔缺粒** | `169.254.44.202` | 18 个兜孔是否都压到位有球 | **D9 开闸**放件落回收盒 | `inspector_missing.py`（二期判据两级：孔级高光 + 件级半径比中位数；598 张全量 逃逸 0 / 过杀 0/423） |

两站共用同一套 10001 私有协议取图（`inspector_pure.Vn2000Source`），因此残帧标记、与传感器张数
一一对齐等取图逻辑两站完全一致，差别只有 IP 与落盘目录。

---

## 架构

```
main_pipeline.py  ← systemd 只跑这一个（主程序 / 监督者）
   │  · 独占 UNO 串口（唯一主人）
   │  · VerdictServer 监听 127.0.0.1:47653，收两站上报的 NG
   │  · front 的 NG → pulse(D8=吹气)；missing 的 NG → pulse(D9=开闸)
   │  · 监控两子进程：一条挂了 → 告警 + 短退避重启它，不动另一条
   │  · 驱动 UNO 双色 LED 健康态（D10 绿 / D11 红）
   │
   ├─ 子进程 A: main_pipeline.py --child front
   │      uno_relay.UnoRelayController = station_link.IpcRelay  （唯一执行点被透明转发）
   │      → inspector_pure.main([--mode camera --ip .201 --holes 0 --quiet --timing])
   │
   └─ 子进程 B: inspector_missing.py --mode camera --ip .202 --quiet --timing
          缺粒检测，NG 同样经 IpcRelay 上报主程序

        两条 IPC 通道各自保活；任一侧发现链路死了 → 该站主动停线（fail-safe）
```

### 关键决策（为什么这么绕）

- **一块 UNO 只有一条串口** → 串口收敛到唯一主人（主程序），两子进程只判定、NG 经 IPC 上报、主程序
  代触发。回退选项：换两块 UNO（每站各一块独占自己串口）可把 `station_link.py` 与 `--child` 整层删掉，
  执行路径回到与单站时代完全一致，代价是多一块板 + 一个 USB 口。
- **`inspector_pure.py` 一行不改** → 子进程 A 在调 `ip.main()` 之前把 `uno_relay.UnoRelayController`
  换成 `IpcRelay`，`main()` 里的局部 import 就拿到替身，唯一执行点 `emit_control`（`if not is_ok: uno.pulse()`）
  被透明转发。扩展一律靠**从外部覆盖模块全局**（`_run_child` / `inspector_missing._apply_dirs`），
  全局在调用时才读，启动前覆盖即生效。
- **常驻**：相机没上电不是故障、不退出（≈10 年建链窗口覆盖 `CAM_STARTUP_WAIT_S`），哪路相机先通先
  工作那路。中途掉线仍走有限 3 次重连 → 耗尽则退 → 主程序重启该站（让 IPC 断开、LED 如实转红，
  绝不原地无限重连假装还活着）。
- **双色 LED 健康态**：绿的充要 = UNO 连着 且**每个站**（进程活 + IPC 连上 + **正在判定** `is_inspecting`）。
  用 `is_inspecting` 而非 `is_connected`——IPC 握手在相机通之前就成立，那时必须红。看门狗 3s，
  `LED_REFRESH_S(1.0s) < 3s`，否则看门狗误判主程序失联把绿打成红。开机默认红（还没确认过健康）。

## 跑起来

产线（唯一入口，systemd 只跑这一个）：

```bash
./run_pipeline.sh                       # = python src/flange_inspect/main_pipeline.py
```

无 UNO 时主程序**拒绝启动任何检测站**（fail-safe，rc=2）——绝不跑一个触发不了执行器的检测器。

单站离线调试（不接硬件、不碰串口）：

```bash
python src/flange_inspect/inspector_pure.py    --mode local --dir <正反面样本> --holes 0
python src/flange_inspect/inspector_missing.py --mode local --dir <缺粒样本>
python tools/uno_manual_console.py             # 手动逐路发 D8/D9，现场核对光耦→PLC 接线
```

## 安全铁律（不可协商）

- **零逃逸**：真值 NG 判 OK 绝不可接受；过杀（OK 判 NG）可接受。任何「判定→执行」链路失效必须**停该站**，
  而不是继续判却触发不了执行器（那样 NG 件直接流过去）。
- **`inspector_pure.py` 一行不改**：黄金基线，改了它「判定一致」就不再是黄金基线的结论。
- **`--no-uno` 绝不透传给产线子进程**：带它时 IPC 照样建链，但检测器此后永不调 `pulse()` → 主程序一条
  NG 都收不到 → 判定照跑、执行器不动 = 逃逸。只在 `--mode local` 离线跑样本时用。
- **日志字符串与 verify 同步**：`[PIPE] 站 <station> NG -> 触发 D<pin>` 被 `tools/verify_l2_pipeline.py`
  的 `check_actuation` 逐字解析。改这行日志必须同步改 verify。
- **引脚映射整词匹配**：D8 发 `NG`（第一站吹气）、D9 发 `NG2`（第二站开闸）。`NG` 是 `NG2` 的前缀，
  数命令必须整词匹配，否则两站同跑时 D8/D9 会串成一堆。

## 目录与日志

| 用途 | 目录 |
|---|---|
| 第一站（经主程序） | `data/pure/{logs,OK,NG,RAW}` |
| 第二站 | `data/missing/{logs,OK,NG,RAW}` |
| 主程序 | `data/supervisor/logs` |

两站日志系统**同源**（`inspector_missing` 复用 `inspector_pure.setup_runtime_logging`，只覆盖目录）：
保留 **7 天**、单文件滚动 **20MB**、每 **6h** 巡清一次，按北京时间 `YYYY/MM/DD` 分天。各进程各自起清理
线程守自己那棵目录树。产线只清 `data/pure/logs` 与 `data/missing/logs`；老的 `data/logs` 只有单独手跑
`inspector_pure` 时才写，产线不碰、可不管。

产线默认**不存 RAW**（图太多，只留 `.log`），仅 `--debug` 时存 RAW + 结果图。RAW 一旦开仍要求与传感器
张数一一对齐、残帧标 `PARTIAL`。

## 数据集（Git LFS，已定版不再新增）

| 数据集 | 张数 | 布局 |
|---|---|---|
| `datasets/flange` | 975 jpg | `OK` / `NG` |
| `datasets/缺粒样本` | 40 png | `正面/{OK,NG}`、`反面/{NG,手动反面样本}` |

`.gitattributes` 里 `datasets/**/*.{jpg,jpeg,png}` 全走 LFS，新增图片自动纳管。

## 验证

| 层 | 内容 | 状态 |
|---|---|---|
| **L1** | 缺粒算法零逃逸（`tools/verify_l1_missing.py`） | ✅ 7/0/0/0（逃逸 0 / 过杀 0） |
| **L2** | 主程序代管串口 + IPC 转发的**等价性**（`tools/verify_l2_pipeline.py`）：同一样本目录，独立跑 vs 经主程序跑，逐帧判定必须逐条相同；每次 NG 恰好一次对应引脚脉冲、不串路 | ✅ 55/0/0/0（无 CH340 时满跑；开发机插着 CH340 时②段 SKIP，为 52/0/0/1，非回归） |
| **L3** | 双站联调：两台真相机 + 真 UNO，D8/D9 各走各路不串 | ⏳ 待现场 |
| **L4** | 安全验证：①杀主程序 → 两子进程都停 ②杀一个子进程 → 另一个照跑且被重启 ③断 IPC → 该站停线而非空判 | ⏳ 待现场（③必须实测，不能只看代码） |

逐帧判定只能从 `.log` 读（产线子进程带 `--quiet`，逐帧 `[INSPECT-DONE]` 只进文件不进控制台），
所以两条链比对时都从 `.log` 取，同一把尺子。固件 `firmware/uno_plc_trigger/uno_plc_trigger.ino`
**已重烧**（D8/D9 两路独立非阻塞脉冲 + `HEALTHY`/`FAULT` + 看门狗 3s，开机默认红闪）。

## 还差什么

**只差上生产机接两个传感器实测**：接好两台相机（`.201` / `.202`）+ UNO（D8/D9 各接执行器、
D10/D11 接双色 LED）→ 跑 `./run_pipeline.sh` → 过 L3 / L4。软件层 L1 / L2 已全绿。

## 硬件链

```
相机 .201 ─┐                                  ┌─ D8 ─→ 光耦 ─→ PLC ─→ 第一站吹气
           ├→ i3-4130T (main_pipeline) → UNO ─┤─ D9 ─→ 光耦 ─→ PLC ─→ 第二站开闸落料
相机 .202 ─┘                                  ├─ D10 ─→ 绿 LED（两相机+UNO 全在判定）
                                              └─ D11 ─→ 红 LED（任一未就绪 / 失联，闪烁）
```

一块 UNO，固件把「一路脉冲」扩成 D8 / D9 两路独立计时（各自到时自动回 LOW，互不阻塞）。
串口/波特率以 `uno_relay.py` 为唯一定义源；`pulse()` 无参默认仍是 D8（向后兼容）。

## 文件地图

| 文件 | 作用 |
|---|---|
| `src/flange_inspect/main_pipeline.py` | 主程序：独占 UNO、VerdictServer、拉起/监控两子进程、`--child front` 入口、LED 健康态、日志汇聚 |
| `src/flange_inspect/station_link.py` | 判定上报通道：`IpcRelay`（子进程侧鸭子替身）+ `VerdictServer`（主程序侧），localhost TCP 行协议 |
| `src/flange_inspect/inspector_pure.py` | 第一站算法（**黄金基线，勿改**），也是取图类 `Vn2000Source` / 日志系统的来源 |
| `src/flange_inspect/inspector_missing.py` | 第二站算法（缺粒），复用 `inspector_pure` 的取图/日志/图像助手，自带 `main()` 与目录常量 |
| `src/flange_inspect/uno_relay.py` | UNO 串口控制器：`UNO_PIN=8` / `UNO_PIN2=9`、`pulse()`、`set_led()`、自愈重连 |
| `firmware/uno_plc_trigger/uno_plc_trigger.ino` | UNO 固件：D8/D9 双路脉冲 + 双色 LED + 看门狗 |
| `tools/uno_manual_console.py` | 手动逐路发 D8/D9/STATUS，核对接线 |
| `tools/verify_l1_missing.py` / `verify_l2_pipeline.py` | L1 / L2 验证闸 |
| `tools/README.md` | **tools 目录索引**：验证闸 / 硬件调试 / 第一站调参诊断 三类，每个脚本一行（其余 `dbg_report.py`、`analyze_feature_ab.py`、`_diag_*` 都在里面） |
| `docs/camera_config.md` / `docs/dbg_report.md` | 第一站相机成像标定记录（带变更账本）/ `dbg_report` 调试外挂说明 |
| `docs/missing_camera_config.md` | 第二站（缺粒）相机成像标定记录：光极性/三半径收敛等成像前提 + `.202` 参数待现场标定清单（算法判据在 `inspector_missing.py`，不重复） |
| `docs/tuning_notes.md` | 第一站算法调参史 / 证伪账：代码只存最终阈值，这本存「为什么这么选、试过哪些没走通」（特征A/B、反面定位P0、广度闸、成像天花板、为什么不用C++） |

## 第一站成像与标定细节

第一站的 10001 私有协议、几何标定值、触发源设置（**上线必须 IO 硬触发**）、以及「取图通了 ≠ 可以判定」
的整套成像清单，都是 `inspector_pure` 时代验证下来、现在仍然有效的知识。完整内容见 **代码头部 10 个分节
阈值常量** 与 [`docs/camera_config.md`](docs/camera_config.md)（相机参数是标定值的另一半 + 换相机后的
复检清单）。几条必记的硬约束：

- **分辨率绑死标定**：直连帧 1280×800 无压缩灰度，一帧恰好 1024000 字节。标定像素值（孔径 50.7px /
  节圆 396.9px）绑在 1216×1024 存图取景上，直连取景要 `--calib` 重标（实测整体缩到 ~0.80 倍，
  `HOLE_R_MIN_RATIO` 只剩 6% 余量）；按 `r` 比例写的常量仍成立。
- **触发源必须 IO 硬触发**（一个触发沿 = 一件 = 一帧，结构上不可能判到上一件）。用过
  `--trigger MainRunOnce` 后相机会被收尾的 `StopRun` 停住，要在 MJ 里重置运行态才再出图。
- **正反鉴别力全在每孔压痕个数**（`MIN_VALID_MARKS=2`），不是圆度阈值——提 `MARK_CIRCULARITY_MIN`
  会先杀正面、救不了反面（它只剔毛刺碎轮廓）。
- 上线前仍要处理的成像问题：画面中央那条饱和亮带、确认并关掉 `ATMode:2` 自动曝光。

PLC 对接走 UNO → 光耦这条硬件链（见上「硬件链」），不再用软件 Modbus。

## 常见坑与现场排查

上机时最容易卡住的几处，都是**踩过的**，按现象查：

| 现象 | 根因 | 处理 |
|---|---|---|
| 主程序起不来，`rc=2`，日志「拒绝启动任何检测站」 | **没插 UNO**（或 CH340 没认出） | 这是 fail-safe 设计，不是 bug：绝不跑一个触发不了执行器的检测器。插好 UNO 再跑 |
| 相机连不上 / 连错相机 | 同网段现在有**两台**相机，`discover_camera_ip()`（唯一应答者才算）**必然失效**回退到默认 IP | 两站都由 `main_pipeline.STATIONS` 里的 `ip` **显式指定**（已这样做）。单站 `run_inspector.sh` 不带 `--ip` 的自动探测在双相机现场**不再可靠**，调试也要显式给 IP |
| 切回 IO 硬触发后相机 23 s 一帧不出 | 上一轮用过软触发（`--trigger MainRunOnce`），相机被收尾的 `StopRun` 停在非运行态 | 去 MJ 里把方案重新置为运行态（或再发一次软触发）。`external` 模式自己从不发 `StopRun`，上线运行不会造成这个状态 |
| 跑完 NG 证据图不见了 / `data/.../RAW` 空 | 有清理软件在删目录（现场出现过 `result` 目录自行消失，算法只创建从不删除）；且产线**默认不存 RAW**，仅 `--debug` 存 | 先确认是不是没开 `--debug`；要留证据先查清哪个清理软件在删那棵目录树 |
| 第二站整批判反（该 OK 的判 NG 或反之） | **光极性搞反**——判据认定「有球 = 兜孔中心比台面**暗** + 一个球面高光点」，光路一改极性就全错（这个坑踩过一次） | 现场按 `docs/missing_camera_config.md` 复核光极性；改完必须重跑 `verify_l1_missing.py` 零逃逸自检 |
| 缺粒站判定可疑 | 二期判据标定样本量小：`datasets/缺粒样本` 只有 40 张；2026-10-02 又用 `I:\data.zip\data\missing\Class` 的 598 张（正面OK 423 / 正面NG 44 / 反面NG 44 / 无件 87）复标过一次 | 攒更多样本复跑 `tools/verify_l1_missing.py --dir <目录>`，**逃逸恒为 0** 才谈得上信它；过杀方向可接受但第二站 NG 要人工挑，别让它涨 |
| 第二站每帧都判 NG、`--debug` 叠加图 18 槽全绿 | 件级闸拦下的：整圈半径比中位数 < `PART_R_RATIO_MED_MIN` = 球没压到位（反面浮球）或节圆拟合跑偏 | 叠加图脚注会标 `[PART rr_med<...]`；看同帧的 `pitch_r` 与 `|z|`（相位一致性）——`pitch_r` 偏离 ~316 或 `|z|<0.8` 就是定位崩，不是缺粒 |
| 第二站**好件被判缺粒**，`--debug` 脚注里 `pitch_r`≈316、`|z|`>0.9（定位全对），只是某个槽高光读成 100 上下 | 高光量测区**太窄**装不下高光斑。同轴光下球面高光系统性朝光轴(画面中心)偏，偏移/球半径实测 p50=0.155、max=0.741，旧掩膜 0.315r 把 **4.4% 的槽**高光整个切在窗外 | 已修（2026-10-02，量测区 0.35r→0.60r，正面OK 过杀 1/423→0/423、逃逸 0）。**这是量测缺陷不是阈值取舍**——`BALL_HIGHLIGHT_MIN=120` 一分没让，有工件帧上真缺粒/反面的空兜高光上限反而从 81 降到 75~78。别靠调闸去补 |
| 第二站**定位歪**（叠加图 18 槽整体偏出工件、明明是好件却判 NG） | 节圆拟合被假圆骗走。已修两处（2026-10-02，正面OK 过杀 5/423→1/423、逃逸 0）：①tier1 漏传候选半径**上界** `r_ceil`——近乎共线的三点外接出上万 px 的假大圆，再靠"容差随半径膨胀"刷票压过真节圆；②孔心点集脏时 tier1/tier2 常**近平票**，改用"节圆与外圆同心"的先验取舍 | 代码里已带，现场不用调。自检：`--debug` 脚注的 `pitch_r` 应在 315±5、`|z|>0.85`。**别再抬 `PITCH_FIT_MIN_HOLES`**（实测抬到 6 净过杀反升、8/10 把好帧判成定位失败）；改完必跑 `verify_l1_missing.py --dir <目录>` |

> ⚠ **`--no-uno` 绝不能进产线子进程**：带它时 IPC 照建链，但检测器此后永不调 `pulse()`，主程序一条 NG
> 都收不到 → 判定照跑、执行器不动 = **逃逸**。只在 `--mode local` 离线跑样本时用。

## 环境

```bash
python -m venv venv && source venv/bin/activate      # 产线 Ubuntu；Windows 用 .venv\Scripts\activate
pip install opencv-python numpy pyserial              # camera 直连只用标准库 socket；串口用 pyserial
```

Python ≥ 3.8；OpenCV 3/4/5 均可（`findContours` 返回值已兼容）。




