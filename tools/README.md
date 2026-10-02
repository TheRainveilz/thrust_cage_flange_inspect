# tools/ —— 索引

**每个脚本的"怎么用 / 为什么这么写"都写在各自文件头部的 docstring 里**（那是随代码走、不会漂的地方）。
本文件只做**分类导航**：以后要改某样东西时，先在这儿找到它属于哪一类、干嘛的，再去翻它的 docstring。

分四类：**① 验证闸**（上线前必须绿的硬闸）、**② 硬件调试**（现场核对接线）、**③ 第一站阈值调优 / 诊断**
（离线拿样本调参，产线节拍里不跑）、**④ 第二站（缺粒）回归 / 诊断外挂**（同上，只服务 `inspector_missing.py`）。

> 解释器有坑：`_diag_*` 的 docstring 写的是 `.venv/Scripts/python.exe`，但 `dbg_report` 需要 `cv2`
> 而仓库 `.venv` **没装 cv2**（用系统 `D:\Python\python.exe`）。跑之前先确认你挑的解释器装了 opencv/numpy。

---

## ① 验证闸（软件层上线前的硬闸，见 README「验证」表）

| 脚本 | 干嘛 | 怎么跑 |
|---|---|---|
| `verify_l1_missing.py` | **L1**：缺粒算法**零逃逸**。脱开 IPC/串口/主程序，只跑算法本身，确认它单独判得对（`--no-uno`、可重复跑对拍） | `python tools/verify_l1_missing.py`（`--dir` 默认缺粒样本、`--repeat 2`） |
| `verify_l2_pipeline.py` | **L2**：主程序代管串口 + IPC 转发的**等价性**。同一样本，独立跑 vs 经主程序跑，逐帧判定必须逐条相同；每次 NG 恰好一次对应引脚脉冲、不串路。**全程 `--dry-uno`** | `python tools/verify_l2_pipeline.py [--skip-both]` |
| `verify_common.py` | 上面两者的**共享库**（无 CLI）：跑检测器、按字节增量取"本次运行新增的 .log"、解析逐帧判定、真值只认目录名。改验证工具时先看它 | 库，不单独跑 |

> ⚠ `[PIPE] 站 <station> NG -> 触发 D<pin>` 这行日志被 `verify_l2_pipeline.check_actuation` **逐字解析**，
> 改这行日志必须同步改 verify（见 README「安全铁律」）。

## ② 硬件调试（现场核对光耦→PLC 接线）

| 脚本 | 干嘛 | 怎么跑 |
|---|---|---|
| `uno_manual_console.py` | 交互式控制台，手动逐路发 **OK / NG(→D8 第一站吹气) / NG2(→D9 第二站开闸) / STATUS**，用万用表/LED 逐路核对接线。复用 `uno_relay.UnoRelayController`（含 CH340 自动探测 / 自愈） | 直接跑，跟菜单交互 |

## ③ 第一站阈值调优 / 诊断（离线，拿样本调参；**产线节拍里不跑**）

只服务**第一站正反面**（`inspector_pure.py`）。都通过 import 复用主算法、**从外部覆盖模块全局**来试参数，
一行不改主算法。

**可复用的两把主力工具**（有完整 argparse，交付级）：

| 脚本 | 干嘛 | 怎么跑 |
|---|---|---|
| `dbg_report.py` | **主力外挂**：CSV（一行一拐角、36 列）+ 固定版式拼图 + `--sweep`（圆度×压痕数网格）+ `--sweep-radius`（半径上界扫描）+ 零逃逸约束下的推荐阈值。完整说明见 **`docs/dbg_report.md`** | `python dbg_report.py --dir <样本> --holes 0 --sweep`（cv2！用系统 Python） |
| `analyze_feature_ab.py` | 批量统计特征 A / B 通过率写 CSV，带一堆阈值覆盖开关（`--feature-a-mode`/`--collar-gate`/`--min-marks`/`--no-pitch-gate`/…），自动写 `artifacts/reports/` | `python tools/analyze_feature_ab.py --dir <样本> --holes 0` |

**一次性诊断脚本 `_diag_*`**（无 argparse，**改文件顶部常量**来调，读完即用完的探针，别当稳定接口）：

| 脚本 | 回答的问题 |
|---|---|
| `_diag_feature_a_sweep.py` | 把 `FEATURE_A_MODE` 扫过 contour/hough/or/and 四值，各自的 NG 裕度——A 切 hough-only 能不能当真闸 |
| `_diag_feature_a_rings.py` | 特征 A 在反面到底"过"在什么上（同心环边，不是翻边深度） |
| `_diag_a_collar.py` | 领圈环深/暗/宽度测量——领圈闸能不能把骗过 A 的反面孔挡掉 |
| `_diag_margin_cfg5.py` | CFG5（精确节圆拟合）下真实的 NG 侧裕度：每张 NG 离逃逸差几痕（闸保护/A否决/只差B/只差1痕 分桶） |
| `_diag_breadth.py` | 件级"压痕广度"（多少孔/拐角见痕）能不能当第二道件级闸 |
| `_diag_localize.py` | 多少检出孔真落在节圆上（定位质量，off-pitch 会毁掉后续 ROI） |

---

## ④ 第二站（缺粒）—— 回归闸 / 诊断外挂

只服务**第二站兜孔缺粒**（`inspector_missing.py`）。同样**产线节拍里不跑**，只离线拿样本跑。

| 脚本 | 干嘛 | 怎么跑 |
|---|---|---|
| `_missing_equiv.py` | **回归闸：改过第二站算法必跑。** 598 帧全量快照（判定 + 每槽半径比/高光）：改前存 `before`、改后存 `<tag>`、再 `cmp`，要求**判定逐帧不劣化 + 逃逸恒为 0 + 正面OK 过杀不增加**。可临时覆盖两个候选加速开关来隔离变量 | `python tools/_missing_equiv.py before\|<tag>\|cmp before <tag>`，覆盖写法 `... <tag> <降采样> <跳首遍0/1>` |
| `_missing_hough_probe.py` | 两个候选加速开关值不值得开：首遍严阈值 `p2=55` 的命中率、候选数分布 | `python tools/_missing_hough_probe.py` → `_hough_probe.txt` |
| `_missing_prof.py` | 单帧耗时分解（preprocess / 找孔候选 / 定位 / refine / 逐槽），改完再量瓶颈在哪 | `python tools/_missing_prof.py` → `_timing_missing.txt` |
| `_missing_micro.py` | 微基准：整幅 `astype` 开销 / Hough 各分辨率 / 掩膜建图 —— `_F32View` 与掩膜缓存的立论依据 | `python tools/_missing_micro.py` → `_micro_missing.txt` |
| `_missing_back_final.py` | `反面/NG` 全 44 张按用户 2026-10-02 定性分类 + 两级闸各拦下几张 | `python tools/_missing_back_final.py` → `_back_final.txt` |
| `_missing_sweep.py` | 全量逐帧逐槽落盘成 JSON（其余分布分析的源头） | `python tools/_missing_sweep.py` |

> ⚠ 两条：
> **①`_equiv_before.json` / `_equiv_before_t.json` 是 `cmp` 的冻结基线，别删。**它是改优化前的 598 帧快照
> （中位 72.72ms），任何后续改动都拿它比。
> **②`_missing_equiv.py` 的前 4 行常量目录指向产线机外的 `I:/data.zip/data/missing/Class/`**（598 帧），
> 换机器要改。它被 `inspector_missing.py` 的 `detect_hole_candidates_fast` / `_F32View` 注释**点名要求重跑**。

---

## ⑤ 文档渲染（把仓库 Markdown 转成离线可看的 HTML）

| 脚本 | 干嘛 | 怎么跑 |
|---|---|---|
| `render_docs_html.py` | 把 `README.md` / `docs/*.md` / `tools/README.md` 转成 `docs_html/` 下同构的 `.html`（文档间 `.md` 链接自动改写成 `.html`，带表格/代码/引用样式，双击即看）。**零依赖**、只用标准库，没网/没 pip 的产线机也能跑；用了新语法就补它、别引依赖 | `python tools/render_docs_html.py [--open]`（任何仓库 Python 3.8+，**不需要 cv2**） |

> 产物 `docs_html/` 默认在 `.gitignore` 里（按需重跑即可）；想把 HTML 一起交给没装 Python 的人直接看，就把 `.gitignore` 里那行删掉再提交。

## 要不要给每个脚本单独写文档？——不用

问过一轮：这些脚本**都已经带了实打实的头部 docstring**（怎么跑、为什么这么写、跟谁耦合都在里面），
没有空壳。再单独写一份份 `.md` 只会**和 docstring 重复、然后各自漂**——改了代码忘了改 md，比没有还糟。

所以最值的做法就是**这一份索引**（你现在看的）：以后想改哪样东西，先在这儿定位到分类和文件，
再翻它的 docstring 看细节。只有 `dbg_report.py` 因为流程复杂、要贴报告、要对照标定值，才值得
额外一份 `docs/dbg_report.md`——其余的 docstring 足够。

> ④ 里的 `_missing_*` 沿用同样的姿态：**`_` 前缀 = 一次性探针，别当稳定接口**（改文件顶部常量来调）。
> 2026-10-02 清过一轮：二期标定那一回的一次性探针（高光窗口比定标、孔级判据重打分、定位 tier 规则、
> 球径/广度闸、反面三类定性等 ~29 个脚本 + ~27MB 产物）已删——**它们的结论都已落在 memory / 本文档 /
> `inspector_missing.py` 注释里**，留下的这 6 个是"以后还会再问一次"的那几个。
