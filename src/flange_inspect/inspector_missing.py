#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""兜孔缺粒检测站（第二站）。

**独立于第一站**：自己一台传感相机(169.254.44.202，同 10001 私有协议)、自己的工位、自己的
硬件触发，与"正反面翻边止口"那站不需要帧配对。跑在**独立进程**里(见 main_pipeline.py)：
两个检测站同机时，各自的相机 RX 线程与 GIL 才不会互相饿死出残帧。

**它只判定，不直接驱动执行器**：一块 UNO 只有一条串口，产线上由主程序独占；本站判 NG 时调
`uno.pulse()`，那时 `UnoRelayController` 已被主程序换成 IpcRelay，判定经 localhost 报给
主程序、由它触发 D9(见 station_link.py)。**本站的执行动作不是吹气，是开闸门**——D9 脉冲让
闸门打开，判 NG 的工件掉下去落进回收盒(第一站才是吹气把件从料道吹掉)。

**复用的是第一站已经过黄金基线验证的定孔链**：`detect_hole_candidates`(Hough 粗找圆) →
`refine_hole`(沿射线精定圆心/半径) → 节圆拟合定位工件 → `hole_on_pitch`。
保持架的兜孔本身就是圆孔，这套链在 inspector_pure.py 里已经是向量化且验证过的东西，
比新写一套定位省事也更可信。**inspector_pure.py 一行不改**。

**唯一的例外是定位**：本站用自己的 `locate_part_fast()` 替掉 `inspector_pure.locate_part()`。
同一条链、同一套验收标准，只把"找外圆当锚"那一步降到 1/4 分辨率且只当 seed —— 因为本站样本上
Hough 候选常年整片是伪圆，每帧都掉进 locate_part 的全幅大半径 Hough(实测 98~288ms，最坏帧
380ms，打穿 300ms 预算)。理由、安全性与等价性实测都写在 `LOCATE_SEED_DOWNSCALE` 注释里。

**二期判据(已用 598 张全量样本复跑标定，零逃逸)**：一句话 ——
**先用 Hough 候选把节圆与相位锁出来，再合成整圈 18 个兜孔的标称位置、逐个精定位**；
然后**分两级判**：孔级看「这个兜有没有球」，件级看「整圈球坐没坐到位」。
两级用的量是**两个不同的维度**，绝不能在同一个槽上硬 AND(旧写法就是这么错的，见参数区注释)。

为什么要合成 18 槽：一期只量 Hough 真检出来的那几个兜孔，`正面/NG` 里有 9 张只检出 4~7 个
候选，整帧靠"候选不足"兜住 —— 那**不是算法看见了空兜，是数量闸拦下来的**。合成之后每个兜孔
都被量到，判定不再依赖 Hough 的召回率(相位只要 3 个兜孔就能锁)。

⚠ **仍待做的事**：`datasets/缺粒样本/` 只有 40 张，2026-10-02 换成 `I:\data.zip\data\missing\Class`
的 598 张(正面OK 423 / 正面NG 44 / 反面NG 44 / 无件 87)复跑标定。样本量仍然不算大(尤其反面只有
44 张)，上线前应再攒样本复跑，且零逃逸自检每次都要过(逃逸恒为 0 才谈得上上线)。
一处曾经"已知未修"、**已于 2026-10-02 修掉**的定位缺陷，根因与当初的猜测不同，记下以免重蹈：
现象是节圆 RANSAC 偶尔锁定到假圆(实测 4/423 张正面OK，拟合半径 270/331/359/475 而真值 ~316，
相位一致性掉到 0.20~0.77，`--debug` 叠加图随之画歪)。当初猜的正解"把相位一致性加进 tier1 接受
条件"是**错的**。真根因是 **tier1 漏传了候选半径上界 `r_ceil`**(第一站 locate_part 两处都传了，
这份为第二站抄的副本没有)：内点容差是相对半径的(tol = 0.06*r)，三个近乎共线的孔心外接出上万 px
的假大圆、容差跟着涨到几千 px(比整张图还大)，靠票数在 argmax 里压过真节圆；随后共识重拟合把半径
拉回中等值、内点塌到 4~5 个，而 `PITCH_FIT_MIN_HOLES=3` 恰是三点抽样的最小样本数、任何三点圆都
天然满足 -> tier1 一路放行。补上界后 4 张里 3 张归位；余下两张是**近平票**(tier1/tier2 内点数差
≤1)但圆心偏出 300px，靠"节圆与外圆同心"的物理先验取舍修掉(详见 `locate_part_fast`)。
全量 598 张账面：正面OK 过杀 **5/423 -> 0/423**、逃逸恒 0。最后那张过杀**也不是定位问题**，
是**高光量测窗口太窄**——已一并修掉，见下面第二条。
**仍不建议抬 `PITCH_FIT_MIN_HOLES`**：实测抬到 6 只救回一半、净过杀反而上升；抬到 8/10 会把好帧
直接弄成定位失败。

另一处、也是**已于 2026-10-02 修掉**的量测缺陷：球面高光的量测区半径原先取 0.35r(有效掩膜
0.315r)，**装不下它要量的那个高光斑**。同轴光下球面高光出现在"法线指向镜头"的那一点，于是
系统性朝光轴(画面中心)偏，离轴越远偏得越多——全量 9847 槽实测偏移/球半径 p50=0.155、p99=0.428、
max=0.741，方向 cos(偏移, 指向画面中心) 在良品帧上 p50=0.97，是**确定性光学位移不是球坐偏**；
**4.4% 的槽高光斑整个落在窗外**，读成低值 -> 好件判缺粒。用户报的 `F000163_RAW` #7
(rr=0.157 已是"球压到位"的标称值、定位全对 pitch_r=317/coh=0.98，高光却只有 104) 就是它：
窗口一放到 0.50r 立刻读到 238，全帧最亮。量测区半径已改 0.60r，账面 正面OK 过杀 **1/423 -> 0/423**、
真缺粒/反面的空兜地板不升反略降(81->75、79->78)，逃逸恒 0。**这是量测缺陷不是阈值取舍**，
所以没动 `BALL_HIGHLIGHT_MIN=120`——那个闸的裕度一分没让。详见参数区。
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

import inspector_pure as ip
from inspector_pure import (AcquisitionError, RUNTIME_LOGGER, crop_pad,
                            fit_circle_ransac, fit_pitch_anchored,
                            hole_on_pitch, imwrite_unicode, preprocess, refine_hole,
                            setup_console, setup_runtime_logging, unique_image_name)

# ============================  路径：本站自己的目录  ============================
# 与第一站(data/logs、data/{OK,NG,RAW})**完全不同**：两站各自写各自的目录，
# 日志文件序号和存图绝不互踩(DailyDirRotatingHandler 靠扫目录算序号)。
PROJECT_ROOT = ip.PROJECT_ROOT
MISSING_ROOT = os.path.join(PROJECT_ROOT, "data", "missing")
RUNTIME_LOG_DIR = os.path.join(MISSING_ROOT, "logs")
OK_SAVE_DIR = os.path.join(MISSING_ROOT, "OK")
NG_SAVE_DIR = os.path.join(MISSING_ROOT, "NG")
RAW_SAVE_DIR = os.path.join(MISSING_ROOT, "RAW")
DEFAULT_SAMPLE_DIR = os.path.join(PROJECT_ROOT, "datasets", "缺粒样本", "正面")

# ============================  判定结论  ============================
OK_PASS = "OK"
NG_EMPTY_POCKET = "NG_EMPTY_POCKET"  # 真的看到空兜(缺粒)
NG_LOCATE_FAIL = "NG_LOCATE_FAIL"  # 定位不出工件(残件/底板/视野异常)
NG_POCKET_NOT_FOUND = "NG_POCKET_NOT_FOUND"  # 兜孔定不出来/不够数
NG_INVALID_FRAME = "NG_INVALID_FRAME"  # 图像读不出来
NG_TIMEOUT = "NG_TIMEOUT"  # 超出判定时限，fail-safe 判 NG

# ============================  算法参数  ============================
# ---- 兜孔几何与定位：复用第一站的找圆/定圆链(同一类圆孔、同一套相机取景)。
#      POCKET_MIN_COUNT 是"Hough 粗候选至少要几个"的早期快速否决。一期取 8，是因为一期只量
#      Hough 真检出来的兜孔、检得越少越不可信；二期改成**合成整圈 18 槽**之后，度量不再依赖
#      Hough 召回率(锁相位只要 3 个)，故降到 4 —— 只留"明显没东西可看就早早判 NG"这一层意思。
POCKET_MIN_COUNT = 4

# ---- 专机专用的兜孔数：这个保持架一圈就是 18 个兜孔、等角分布。
#      **这是机械常量，不是调出来的阈值**：样本实测相邻兜孔角差恒为 20° 的整数倍、
#      逐图求和恒为 18。换机型才需要改这里。
N_POCKETS = 18
POCKET_ANGLE_STEP = 360.0 / N_POCKETS
PHASE_MIN_POCKETS = 3  # 锁整圈相位至少要几个已定出的兜孔；少于此判 NG(看不清就不放行)

# ---- 判据(二期)：**分两级，两级量的是两个不同的维度，不要在一个槽上硬 AND**
#
#   【孔级】这个兜里有没有球  —— 只看**球面高光**
#       同轴光垂直照下来，球是凸的，把光反射回镜头形成一个小亮点；空兜没有。
#       这个量**干净隔离正面缺球**：598 张全量实测，507 个空兜的高光最高只有 81，
#       而判 OK 的良品 6480 个槽最低 167、p1=202 —— 中间是空的，取 120 两侧各留 39 / 47 级。
#
#   【件级】整圈球坐没坐到位  —— 看**整圈半径比的中位数**
#       精定位半径(以节圆半径为 1)在三种落座状态下收敛到三个不同的值：
#         · 球压到位：暗盘 + 极亮球面高光点，球几乎填满正面孔口 → **≈0.157**
#         · 空兜    ：透光亮孔(光穿过孔打到下方白底板)，露出的是更小的孔底 → **≈0.118**
#         · 反面浮球：没压进去 → **≈0.118~0.140**(比"球压到位"明显小)
#           产线上的反面 = 机器把球往反面上压、**压不进去、球浮着**，球径肉眼可辨地偏小。
#           但早期样本里的反面**多数不是这种**，而是"正面OK的件手工翻过来拍的"——球其实压到位了。
#           两类靠半径比**分不开**(手动翻面 0.131 正落在真反面 0.118~0.140 中间)，也不需要分：都判 NG。
#           写这条是提醒：**别拿"手动翻面"那几张的读数去论证"第二站兜得住真反面"**。
#           `Class/反面/NG/` 全 44 张已由用户逐张定性完毕(2026-10-02)：
#             真反面   14 = `Class/反面/真反面/` 全量(其中 10 张是**故意少放球**模拟缺粒，
#                          只有 4 张球装满 —— 见下)
#             手动反面 25 = `Class/反面/手动反面/` 的 24 张 + `20261001.120646.293.000084`
#                           (**人造样本，产线不会出现**)
#             直接NG    5 = 两张没拍全的(`20261001.114846.574.000028` `.759.000029`)
#                           + `F000005` `F000006` `F000007`(F000007 还是个坏件)
#           14 + 25 + 5 = 44 ✓
#       ⚠ **半径比在孔级不隔离正/反，千万别拿它当孔级硬闸**：单槽半径比对**一颗球坐得
#         多深**极敏感，良品整圈实测 0.107~0.168，反面NG 整圈实测 0.107~0.213 —— 两组在
#         孔级完全重叠。旧写法 `rr>=0.144 and hl>=120` 就是这么错的：实测把 **45/423 张
#         好件判死**，坏槽高光是 204~234(球明明在，只是坐得浅一点)，半径比 0.131~0.144。
#         用户 2026-10-01 报的 `#17 0.139/208` 就是这个类的样本。
#       但整圈**一起**下移才是反面浮球/定位跑偏的特征，取件级中位数后两组分得很开：
#         正面OK 423 帧的中位数 0.1520~0.1682(min/p5 = 0.1520/0.1540)，
#         反面NG  44 帧的中位数 0.1058~0.1400 → **真实间隙 [0.1400, 0.1520]**。
#         闸取 0.145，靠 NG 一侧，两侧余量 0.0050(反面) / 0.0070(良品)。
#       (2026-10-02 全量 598 张复测。旧注释里的 "良品 min 0.1335 / 反面 max 0.1422" 与当前代码
#        对不上——过杀 0/423 就意味着没帧中位数低于 0.145——那两个数是更早代码状态留下的，已作废。)
#       ⚠ **件级闸是承重的，不是"保险"**：新判据下 44 张反面里有 **27 张孔级全绿**(每个槽的
#         高光都 ≥120)，只有件级中位数拦得住 —— 去掉它这 27 张全部逃逸。旧写法之所以看着
#         "反面每张都留 ≥11 个空槽"，是孔级 rr 硬闸顺带把反面的槽判空了造出来的**假余量**
#         (同一批反面，旧判据 未见球槽数 11/18/18，新判据 0/0/18)。
#         这 27 张按 2026-10-02 的定性拆开(`tools/_missing_back_final.py` → `_back_final.txt`)：
#           真反面   4 张(全部是 4 张球装满的) + 手动反面 23 张(人造) + 直接NG 0 张
#         ⇒ **能当产线证据的只有那 4 张**；手动反面那 23 张虽然也只被件级拦住，
#           但它们是"良品翻过来拍"的，**产线上不存在这种件，不能算件级闸的战功**。
#       ⚠ 件级闸裕度实测 **0.0050(反面侧) / 0.0070(良品侧)**，比看着窄；更麻烦的是**样本面**：
#         用户已把真反面标到 `Class/反面/真反面/`，共 14 张。但**空槽数反映的是"用户放了几颗球"
#         (有一部分是故意少放、为模拟缺粒)，不是闸的行为** —— 别拿空槽多寡去推断闸稳不稳。
#         真正代表产线的是**球装满(18/18 全见高光)的真反面，只有 4 张**：
#           `000002` `000031` `000032` `20261001.121133.282.000151`
#         这 4 张孔级**全部 18/18 全绿**，**只有件级中位数闸拦得住**(rr_med 0.1181~0.1288)。
#         (自洽校验：14 张真反面里 **nEmpty==0 ⟺ 孔级全绿 ⟺ 只有件级拦** 恰好就是这 4 张，
#          一一对应、无例外 —— 所以"球装满的只有 4 张"不是估的，是数出来的。)
#         ⇒ **一件球装满的反面来到产线，孔级 100% 放过它**；拦住它的只有件级闸，
#           而这件事的实测证据就是这 4 张。**补拍"球装满的真反面"是上线前优先级最高的一件事。**
#       ⚠ **想靠"球径"另加一道闸(孔级/广度闸)是没用的**：2026-10-02 试过孔级球径闸与
#         仿第一站的"件级球径广度闸(整件 ≥K 槽球径偏小)"，结论是**都不如件级中位数**——
#         孔级球径在正/反间重叠(良品内圈槽最低 0.1071 vs 反面 p25 0.1310)，任何逐槽阈值都误杀良品；
#         广度闸在 K=4/5/6/8 各档上**补不到任何中位数闸漏掉的帧**(严格子集，纯属白加复杂度)。
#         根因：反面是"整圈一起下移"，中位数正是为这种整体平移设计的稳健统计量。
#
#   ⇒ **全量 598 张实测对比**(同一批图，只换判据)：
#        旧 `孔级 rr>=0.144 且 hl>=120`   → 正面OK 过杀 63/423 (14.9%)、逃逸 0/0/0
#        新 `孔级 hl + 件级 rr_med`       → 正面OK 过杀  5/423 ( 1.2%)、逃逸 0/0/0
#        新判据 + 定位补上界/近平票取舍   → 正面OK 过杀  1/423 ( 0.24%)、逃逸 0/0/0
#        再 + 高光量测区 0.35r -> 0.60r   → 正面OK 过杀  0/423 ( 0.00%)、逃逸 0/0/0
#     过杀降 12 倍而安全不变。旧孔级 rr 闸对"挡逃逸"的唯一贡献是那 27 张反面，而这 27 张
#     正是件级中位数抓的(且那 27 张在旧判据下也是靠 rr 判空的，两条路殊途同归)。
#     最后那张过杀也不是定位问题，是**高光量测窗口太窄**(见模块头与下方量测口径)。
#     ⚠ 这是 598 张的 in-sample 结果，且反面只有 44 张 —— 上线前必须再复跑零逃逸自检。
BALL_HIGHLIGHT_MIN = 120.0    # 孔级闸：球面高光下限(良品 p1=231，有工件帧的空兜 max=78)
PART_R_RATIO_MED_MIN = 0.145  # 件级闸：整圈半径比中位数下限(间隙 [0.1422, 0.1520]，靠 NG 一侧)
REFINE_R_SEED_RATIO = 0.157   # 精定位的初始半径(取"球压到位"的标称值，两侧都能收敛过去)

# ---- 球面高光的量测口径：兜孔内 (99分位 − 中位数)。
#      基线用中位数：局部油污、整片台面亮度漂移都只影响中位数以外的少数像素，分位数差因此稳。
#
#      **量测区半径 0.35r -> 0.60r (2026-10-02)**：原先那个窗口**装不下它要量的东西**。
#      同轴光下球面高光出现在"法线指向镜头"的那一点，于是它**系统性地朝光轴(画面中心)偏**，
#      偏离槽心的距离随离轴角增大。全量 9847 槽实测：偏移/球半径 p50=0.155、p90=0.257、
#      p99=0.428、max=0.741；而偏移方向与"槽心指向画面中心"的 cos 在良品帧上 p50=0.97、
#      91% 的槽 >0.7 —— 是**确定性光学位移，不是球坐偏**。旧掩膜半径只有 0.35*0.9=0.315r，
#      于是 **4.4% 的槽高光斑整个落在窗外**，读出来是个低值 -> 好件被误判缺粒。
#      (2026-10-02 用户报的那张 F000163 #7：rr=0.157 已是"球压到位"的标称值、定位全对
#       pitch_r=317/coh=0.98，高光却只有 104；窗口一放到 0.50r 立刻读到 238，全帧最亮。)
#
#      放大到 0.60r 之后：真缺粒(正面/NG 的 507 个空槽)高光上限 **81 -> 75**、反面(184 个)
#      **79 -> 78** —— 有工件的帧里空兜内部本来就是暗的、周边也没有镜面亮点，放宽窗口不但
#      不侵蚀余量，反而因为窗口够大、p99 不再被单颗噪点/油污顶起来，地板还略降。
#      窗口再往大推到 1.15r(掩膜越过孔壁)时才有槽开始掉出闸(正面/OK 反而新增 1 个判空槽)
#      —— 所以停在"整兜以内"，留 40% 余量到孔壁。
#      副作用只在**无件**帧(空工位)：那里没有兜孔，18 个槽铺在夹具的亮斑上，放大窗口会让
#      读数变高(103 -> 228)。但那些帧每张仍剩 13+ 个空槽、且 rr_med 多在件级闸以下，
#      全量 87 张仍全部判 NG —— 全量 598 张 逃逸 0 / 正面OK 过杀 1/423 -> 0/423。
HIGHLIGHT_R_RATIO = 0.67  # 高光量测区半径系数(以精定位半径 r 为单位)；有效掩膜 = 0.9*0.67 = 0.60r
HIGHLIGHT_PCT = 99  # 高光取 99 分位：只认"少数极亮像素"，抗油污/杂散反光
MAX_EMPTY_POCKETS = 0  # 允许的空兜数(h 级)；0 = 18 个兜孔必须个个见球面高光

# ---- 判定时限 fail-safe：与第一站同一条产线的节拍预算(同源，改一处两站都生效) ----
#      两个值都在 import 时从 inspector_pure 取，是**快照不是活引用**：唯一来源是
#      inspector_pure.py:110/111，运行期再改 ip.* 不会追到这里来。
RESULT_DEADLINE_MS = ip.RESULT_DEADLINE_MS
TIMEOUT_ESCALATE_N = ip.TIMEOUT_ESCALATE_N  # 连续这么多帧超时 -> 停线；0=永不自动停(纯 fail-safe)
#      超时**口径**与第一站一致 = **入队 → 判定**(`window_ms`，含排队)，不是纯算法耗时。
#      两道检查共用这一条钟(主判定 :897、出队过期预筛 :858)，细节见主判定那段注释。
#      没有入队时间戳的取图源(watch/http/本地文件夹)退回纯计算口径，行为与改前相同。

# ---- 定位：与 inspector_pure.locate_part 同一条链、同一套验收标准，**只改一处** ——
#      "找外圆当锚"那一步降到 1/4 分辨率，而且它的结果**只当 seed**。
#
#      为什么必须改：本站样本上 Hough 候选经常整片是伪圆(精定位孔心对最小二乘节圆的中位残差
#      25~47px、容差仅 16px -> tier1 三点 RANSAC 内点数 0)，于是每一帧都掉进 locate_part 的
#      tier2：全幅 800×1280、半径 144~480px 的 cv2.HoughCircles。实测 98~288ms/帧，40 张样本里
#      11 张走这条路(含**全部 3 张良品**)，最坏帧 380ms —— 直接打穿 300ms 判定预算，良品会被
#      判 NG，连够 TIMEOUT_ESCALATE_N 帧就停线。
#
#      为什么这样改是安全的：seed 只喂给 fit_pitch_anchored，它会在该 seed 下重新拟合、并
#      **自证内点数**(>=PITCH_FIT_MIN_HOLES 且半径 > 1.5×孔半径)，与主路径完全同一套验收标准。
#      所以 seed 不准的后果是"拟合不出 -> 返回 None -> 判 NG"，绝不会因为 seed 差就接受一个假节圆。
#      等价性实测(40 张全量)：判定 40/40 一致、节圆/相位无差异；最坏 seed 中心误差 5.5px，
#      良品帧 0.0~1.2px；tier2 耗时 207ms/帧 -> 3ms/帧。
LOCATE_SEED_DOWNSCALE = 4


# ============================  结果结构  ============================
@dataclass
class PocketResult:
    """单个兜孔(槽)的判球量测值。原始量值全留着，标定时直接看分布。"""
    index: int  # 槽序号 0..N_POCKETS-1(按相位排序，不是 Hough 检出顺序)
    cx: float  # 标称槽位(相位合成出来的那个点)，也是 refine_hole 的种子
    cy: float
    rx: float  # refine_hole 实际收敛到的圆心；与 (cx,cy) 的差就是定位漂移，叠加图上画成连线
    ry: float
    r: float  # 该槽精定位出的半径
    r_ratio: float  # r / 节圆半径 —— **件级**判据(整圈取中位数：球压到位≈0.157，空兜≈0.118)
    highlight: float  # 中心区 (99分位 − 中位数) —— 球面高光，**孔级**判据
    has_ball: bool  # 本槽是否见球面高光(孔级，只看 highlight)


@dataclass
class MissingResult:
    """整幅图的缺粒检测结果。"""
    name: str
    verdict: str = OK_PASS
    reason: str = ""
    part_cx: float = 0.0
    part_cy: float = 0.0
    pitch_r: float = 0.0
    phase_deg: float = 0.0  # 锁出来的兜孔相位(0 ~ 20°)
    # 相位一致性 |圆均值|。**只记录不设闸**：相位若锁错，整圈 18 槽一起失配 → 必判 NG，
    # 这一层自带 fail-safe；而这个值本身在各种正常帧上会飘多少还没标定过，先不加闸。
    phase_coh: float = 0.0
    locate_method: str = ""
    ball_r_med: float = 0.0  # 各槽精定位半径的中位数
    r_ratio_med: float = 0.0  # 各槽半径比的中位数
    pockets: List[PocketResult] = field(default_factory=list)
    # 落在节圆上、真正用来锁相位的那些实检兜孔 (x, y, r)。**只为 --debug 叠加图留证**：
    # 相位是整圈 18 槽的骨架，锁错了整圈一起错，所以"从哪几个孔锁出来的"必须能回头看。
    seed_pts: List[Tuple[float, float, float]] = field(default_factory=list)
    n_empty: int = 0  # 未见球的槽数(含量不出来的槽)
    n_slot_fail: int = 0  # 精定位失败/切边的槽数(一律按"没看见球"计)
    elapsed_ms: float = 0.0

    @property
    def is_ok(self) -> bool:
        """是否合格：仅当 verdict 恰为 OK_PASS。任何 NG/异常判定都为 False。"""
        return self.verdict == OK_PASS


# ============================  定位  ============================
def _outer_seed_downscaled(work: np.ndarray, scale: int = LOCATE_SEED_DOWNSCALE
                           ) -> Optional[Tuple[float, float]]:
    """在 1/scale 分辨率上跑外圆 Hough，把中心放大回去当 seed。找不到返回 None。

    **只取中心，不采信半径** —— 半径由 fit_pitch_anchored 重拟合决定。
    """
    h, w = work.shape[:2]
    if min(h, w) < scale * 64:  # 图太小就别降了，降完 Hough 也没意义
        return None
    small = cv2.resize(work, (w // scale, h // scale), interpolation=cv2.INTER_AREA)
    sw = small.shape[1]
    big = cv2.HoughCircles(small, cv2.HOUGH_GRADIENT, dp=1.0, minDist=int(0.30 * sw),
                           param1=ip.HOLE_HOUGH_P1, param2=ip.OUTER_HOUGH_P2,
                           minRadius=int(ip.OUTER_R_MIN_RATIO * sw),
                           maxRadius=int(ip.OUTER_R_MAX_RATIO * sw))
    if big is None:
        return None
    b = np.asarray(big[0], dtype=np.float64)
    bx, by, _br = b[np.argmax(b[:, 2])]
    return float(bx * scale), float(by * scale)


def locate_part_fast(work: np.ndarray, cand: np.ndarray,
                     refined: Optional[List[Tuple[float, float, float, float]]] = None
                     ) -> Tuple[Optional[Tuple[float, float, float]], str]:
    """定位工件，返回 ((cx,cy,pitch_r), 方法名)。与 inspector_pure.locate_part 同构，差别有三：
      1) tier2 的外圆搜索降到 1/4 分辨率且只当 seed(理由与实测见 LOCATE_SEED_DOWNSCALE 注释)；
      2) **两级都带上候选半径上界** r_ceil(第一站两处也带；这里曾漏掉, 是"定位歪"的头号根因, 见
         下面 r_ceil 处的长注释)；
      3) **两级都跑，近平票时按"与外圆同心"的物理先验取舍**(第一站是 tier1 不成就换 tier2)。
         本站的孔心点集比第一站脏得多(伪孔心多), tier1 的票数经常与真节圆持平, 单看票数没有
         分辨力; 见下面取舍分支的注释与全量 598 的对照实测。

    **方法名沿用第一站的 "pitch_fit(n=...)"**：节圆闸 HOLE_PITCH_GATE_METHODS 是按这个字符串
    白名单匹配的，改名等于把所有帧判成"节圆不可信"。
    """
    ring_pts, r_med_ring = None, 0.0
    if refined and len(refined) >= ip.PITCH_FIT_MIN_HOLES:
        ring_pts = np.array([[q[0], q[1]] for q in refined], np.float64)
        r_med_ring = float(np.median([q[2] for q in refined]))
    if ring_pts is None and len(cand) >= ip.PITCH_FIT_MIN_HOLES:
        ring_pts, r_med_ring = cand[:, :2].astype(np.float64), float(np.median(cand[:, 2]))
    if ring_pts is None:
        return None, "none"
    r_floor = 1.5 * r_med_ring  # 节圆必须明显大于孔半径，否则"拟出来的圆"就是某个孔本身
    # 候选半径上界(与第一站 locate_part 同一道闸, 两处都传: 主拟合 + 锚定拟合的共识重拟合)。
    # **没有它 Tier1 会稳定地被假大圆骗走**: 内点容差是相对半径的(tol = max(0.06*r, 8px)), 三个近乎
    # 共线的孔心外接出上万 px 的圆(实测本站 62950px), 容差跟着涨到 3777px(比整张图还大) -> 一帧里
    # 每个孔都"贴"在它上面, 靠票数在 argmax 里压过真节圆; 随后共识重拟合把半径拉回中等值、容差收紧,
    # 共识瞬间塌到 4~5 个 -> 返回一个支撑不足的假圆。而 PITCH_FIT_MIN_HOLES(=3) 恰是三点抽样的最小
    # 样本数, 任何三点圆都天然满足, 于是 tier1 一路放行、能给出正确 10 内点的 tier2 根本没机会跑。
    # 实测 4 张良品被这条正反馈判死(tier1 最终返回的圆心偏到 (770.6,361.9) / (604.3,413.2)、半径
    # 359 / 270, 而真值都是 圆心≈(650,402)、半径≈316)。
    # 上界 = 11 × 中位孔半径(本站 节圆316px/孔半径49px ≈ 6.4, 留 ~70% 余量)。回退: 传 None。
    r_ceil = ip.PITCH_FIT_MAX_R_RATIO * r_med_ring

    # tier1：三点 RANSAC 直接拟节圆(与第一站一致, 必须带上同一个 r_ceil)
    fit = fit_circle_ransac(ring_pts, ip.PITCH_FIT_ITERS, ip.PITCH_FIT_TOL_RATIO,
                            ip.PITCH_FIT_TOL_MIN_PX, ip.PITCH_FIT_MIN_HOLES,
                            r_floor, ip.PITCH_FIT_RANSAC_MAX_COMBOS, r_ceil=r_ceil)
    ok1 = fit is not None and fit[2] > r_floor and fit[3] >= ip.PITCH_FIT_MIN_HOLES

    # tier2：拿外圆中心当 seed(孔阵与工件外圆同心)，把 3 点 RANSAC 降成半径投票 + 干净子集重拟合
    seed = _outer_seed_downscaled(work)
    anch = None
    if seed is not None:
        anch = fit_pitch_anchored(ring_pts, seed[0], seed[1], r_floor,
                                  ip.PITCH_FIT_TOL_RATIO, ip.PITCH_FIT_TOL_MIN_PX,
                                  ip.PITCH_FIT_MIN_HOLES, ip.PITCH_FIT_ITERS,
                                  ip.PITCH_FIT_RANSAC_MAX_COMBOS, r_ceil=r_ceil)
    ok2 = anch is not None and anch[2] > r_floor and anch[3] >= ip.PITCH_FIT_MIN_HOLES

    if not ok1 and not ok2:
        return None, "none"
    if ok2 and not ok1:
        return (anch[0], anch[1], anch[2]), "pitch_fit(n=%d)" % anch[3]
    if ok1 and not ok2:
        return (fit[0], fit[1], fit[2]), "pitch_fit(n=%d)" % fit[3]

    # 两个都拟出来了：**内点数差 ≤1 视为平手**，这时按"节圆与外圆同心"的物理先验取圆心离 seed
    # 近的那个；差 ≥2 则尊重票数，不拿同心先验去压一个明显支撑更好的圆。
    # 为什么需要这一手：点集被污染时 tier1 会挑到"几个伪孔心凑成的小圆"——它的票数与真节圆一样
    # 低(实测 4 vs 4、5 vs 4)，argmax 靠组合枚举顺序决出胜负，圆心能偏到 300px 外(000138_origin
    # 的 (327,445) vs 真值 (652,402))。票数在这种近平票下没有分辨力，圆心位置才有。
    # 为什么不能直接让 tier2 优先(实测): 全量 598 反而从 3 张过杀涨到 5 张——tier2 的 seed 是
    # 1/4 降采样 Hough，外圆被视场切边时会偏，那些帧上 tier1 才是对的。所以只在平手时才交给它。
    if fit[3] >= anch[3] + 2:
        best = fit
    elif anch[3] >= fit[3] + 2:
        best = anch
    elif (np.hypot(fit[0] - seed[0], fit[1] - seed[1])
          <= np.hypot(anch[0] - seed[0], anch[1] - seed[1])):
        best = fit
    else:
        best = anch
    return (best[0], best[1], best[2]), "pitch_fit(n=%d)" % best[3]


# ============================  判据  ============================
# ============================  找孔候选（加速版）  ============================
# 本站不再直接调 inspector_pure.detect_hole_candidates：它有两个**白烧**，见下面函数的注释。
#
# ⚠ **两个开关的默认值都是"关"= 与原版逐位等价**，加速只来自 L1/L3(见下面 _F32View)。
#   原因见函数注释末尾"为什么默认关" —— 这两个开关都是**真改机制**(候选集系统性变)，
#   而本项目对这类改动的既定原则是"全量 sweep 只证样本、不证产线"(2026-10-01 第一站同款改动
#   已按此否决)。两个开关都**已实测、结论都在注释里**，要开就改下面一行，不用重做实验。
CAND_DOWNSCALE = 1            # 降采样倍数(1=关)。试过 2 -> **验不过**(1 张正面OK 被过杀)
CAND_SKIP_FIRST_PASS = False  # 跳过"更严的累加器阈值"那一遍。**过了验收线**，但见下面权衡


def detect_hole_candidates_fast(work: np.ndarray) -> np.ndarray:
    """`inspector_pure.detect_hole_candidates` 的加速版：返回同一套 (x,y,r) 粗候选，只是更快。

    原版有两处白烧(全量 598 张实测)，都做成了开关，但**默认都关**。

    ## 白烧①：白跑的那一遍阈值 —— 实现了，实测**过了验收线**，但默认关

    原版先跑 `p2=HOLE_HOUGH_P2`(严, 55)，候选不足 `MIN_HOLE_COUNT` 再跑
    `p2=HOLE_HOUGH_P2_FALLBACK`(松, 32)。实测 **84.2% 的帧第一遍一个圆都找不到**
    (正面/OK 均值 0.2 个、反面 0.5 个、无件 0.0 个) —— 那一遍 ~9.2ms 纯浪费。

    **全量证据(598 帧，只跳这一遍)**：
      · 判定 **598/598 逐帧完全相同**；
      · **逃逸 0，正面OK 过杀 0/423**(与改动前一致) —— 你定的验收线全过；
      · 逐槽读数有 **659 条微动**(最大 |Δ半径比| 0.021、|Δ高光| 4)：约 16% 的帧上严阈值本来能
        凑够 4 个候选、原版会用那个更小的集合，跳过后改用松阈值的集合。判定没跟着动；
      · 中位 49.67ms -> **40.20ms**。

    **为什么最后还是默认关**：这条**不是**逐位等价(读数确实变了)，它是"真改机制" —— 换了个
    更松的检测器，候选集系统性变化。本项目对这类改动的既定原则是**"全量 sweep 只证样本、
    不证产线"**(来料是振动盘随机姿态 + NG 多数 + 拒件回流重拍，分布 ≠ 数据集；回流还放大
    每趟的逃逸抽奖)，2026-10-01 第一站 `detect_hole_candidates` 上**同款改动已按此否决**。
    而省下的 9.6ms 在 200ms 预算里没有实际意义(本站改前中位 72.7ms，本来就有余量) ——
    **拿"先严后松"这道保守余量换 9.6ms，在零逃逸机器上不划算**。
    要开：`CAND_SKIP_FIRST_PASS = True`，验收证据已在上(638 帧全量比对，判定零变化)。

    ## 白烧②：没降采样 —— 实现了，**验不过，默认关(CAND_DOWNSCALE=1)**

    HoughCircles 的累加器 `dp=1.0` 与原图同分辨率，降到 1/2 幅面 Hough 本身能快 ~3 倍
    (微基准 1280×800 **11.60ms** -> 640×400 **3.89ms**)，而且半径/间距参数本来就是按图宽归一化的
    比例量(`HOLE_R_MIN_RATIO` 等)，语义不随缩放变。**但它过不了验收线**：

      598 帧全量比对，`正面/OK/20261001_160755_819_CAM_F000064_RAW.png` 被过杀(OK -> NG_EMPTY_POCKET)。
      根因：降采样让节圆拟合从 7 个内点变成 13 个，中心挪 1.7px、节圆半径挪 2.2px、相位挪 0.37°，
      于是第 8 槽的**种子位置**偏了一点 —— 而那一槽原版的半径比 0.1103 本来就是 18 槽里最低的
      (临界)，种子一动 `refine_hole` 就抓到了旁边一条小边(半径比塌到 0.0486、高光 36)，被记成
      "空兜" -> 整帧 NG。**这不是降采样算错了，是整条量测链对种子位置本来就敏感**：
      一张 OK 件的第 8 槽离临界只差 2px 的定位扰动，这个脆弱面本身值得记住。
      代价 3.7ms/帧，换成"零过杀"，值。

    ## 两个开关都关时 = 原版的逐位等价(自证)
    `CAND_DOWNSCALE=1 + CAND_SKIP_FIRST_PASS=False` 时本函数与原版**逐位相同**：598 帧判定与
    逐槽读数(半径比/高光)**最大差值恰为 0**。这是"加速版没改写算法"的自证，改任何一处都要重跑。

    ⚠ **改动这里(或改 inspector_pure 的 refine_hole/locate_part)必须重跑 `tools/_missing_equiv.py`
    全量比对**(`before` 快照 + 新快照 + `cmp`)，要求三条同时成立：判定逐帧不劣化、
    **逃逸恒为 0**、正面OK 过杀不增加。
    """
    s = max(1, int(CAND_DOWNSCALE))
    img = work if s == 1 else cv2.resize(work, None, fx=1.0 / s, fy=1.0 / s,
                                         interpolation=cv2.INTER_AREA)
    w = img.shape[1]
    r_lo = max(3, int(ip.HOLE_R_MIN_RATIO * w))
    r_hi = max(r_lo + 2, int(ip.HOLE_R_MAX_RATIO * w))
    min_dist = max(8, int(ip.HOLE_MIN_DIST_RATIO * w))
    thresholds = [] if CAND_SKIP_FIRST_PASS else [ip.HOLE_HOUGH_P2]
    if ip.HOLE_HOUGH_P2_FALLBACK > 0 and ip.HOLE_HOUGH_P2_FALLBACK < ip.HOLE_HOUGH_P2:
        thresholds.append(ip.HOLE_HOUGH_P2_FALLBACK)
    cand = np.zeros((0, 3), np.float64)
    for p2 in thresholds:
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=ip.HOLE_HOUGH_DP, minDist=min_dist,
                                   param1=ip.HOLE_HOUGH_P1, param2=p2,
                                   minRadius=r_lo, maxRadius=r_hi)
        if circles is not None:
            cand = np.asarray(circles[0], dtype=np.float64)
        if len(cand) >= ip.MIN_HOLE_COUNT:
            break
    if len(cand) > ip.MAX_HOLE_CANDIDATES:  # 半径由大到小截断, 防异常图卡死
        cand = cand[np.argsort(-cand[:, 2])][:ip.MAX_HOLE_CANDIDATES]
    if s > 1 and len(cand):
        cand[:, :3] *= float(s)  # 中心与半径都还原到原图尺度
    return cand


# ============================  量测加速：免掉重复的整幅转换  ============================
class _F32View:
    """把**已经算好**的 float32 整幅图喂给 `inspector_pure.refine_hole`，替掉它内部那句
    `gray_f = gray.astype(np.float32)`。

    **为什么要这个替身**：那句 astype 每次都把 **1280×800 整幅**重转一遍(float32 要新分配 4MB)。
    而 `refine_hole` 一帧要被调 **30~36 次**(候选精修 ~18 次 + 逐槽精修 18 次)，
    同一张图因此被转了 30 多遍 —— 微基准单次 astype **0.85ms**，全量实测(598 帧)整帧中位
    **72.72ms -> 49.67ms**。
    「那就传 float32 进去」躲不掉：`astype(float32)` 作用在 float32 上仍然整幅拷贝(微基准 0.65ms/次)。

    所以这里给 refine_hole 一个**鸭子类型替身**。它只用到两样东西 ——
    `.astype(np.float32)` 与 `.shape[:2]` —— 替身把 astype 直接返回缓存的**同一块**数组(零拷贝)、
    shape 透传。**数值逐位相同**(不是近似：`cv2.remap` 拿到的就是原来那个 float32 数组)。

    ⚠ **这是刻意的接口假设，不是通用 ndarray 替身**：若 `inspector_pure.refine_hole` 改用别的
    ndarray 接口(下标、ufunc、切片…)这里会**立刻 AttributeError 炸出来**，不会静默算错。
    动过 inspector_pure 的 refine_hole 之后，必须重跑 `tools/_missing_equiv.py` 全量比对。
    """
    __slots__ = ("_a",)

    def __init__(self, arr: np.ndarray) -> None:
        self._a = arr

    def astype(self, dtype: "np.dtype | type", **kw) -> np.ndarray:
        return self._a

    @property
    def shape(self):
        return self._a.shape


# 圆盘掩膜缓存：掩膜只由 (h, w, half) 决定，而一帧 18 槽里 half 只有 6~8 种取值。
# 原先每槽都重新 mgrid+hypot 建一遍(0.032ms × 18 = 0.6ms/帧)，纯重复。
_MASK_CACHE: Dict[Tuple[int, int, int], np.ndarray] = {}


def _disk_mask(h: int, w: int, half: int) -> np.ndarray:
    """半径为 0.9*half 的圆盘布尔掩膜(h×w)。表达式与原实现逐字相同 ⇒ 逐位相同。"""
    key = (h, w, half)
    m = _MASK_CACHE.get(key)
    if m is None:
        yy, xx = np.mgrid[0:h, 0:w]
        m = np.hypot(xx - (w - 1) / 2.0, yy - (h - 1) / 2.0) <= half * 0.9
        if len(_MASK_CACHE) > 256:  # 兜底：正常远小于此，异常输入也不让它无限涨
            _MASK_CACHE.clear()
        _MASK_CACHE[key] = m
    return m


def _measure_slot(work: np.ndarray, gray_f: np.ndarray, px: float, py: float,
                  pitch_r: float, cos_t: np.ndarray, sin_t: np.ndarray
                  ) -> Optional[Tuple[float, float, float, float, float, bool]]:
    """量一个兜孔槽(标称位置 px,py)，返回 (收敛圆心x, 收敛圆心y, r, 半径比, 高光, has_ball)；
    量不出来返回 None。

    从标称位置出发精定位 —— 这一步同时兼顾"球盘边缘"与"孔壁边缘"两种可能，**收敛到哪一边
    本身带信息**：球压到位时收敛到 ≈0.157×节圆半径，空兜/未压入收敛到 ≈0.12~0.13。
    但**收敛到哪一边不作为本槽的判据**：单槽半径比对"一颗球坐得多深"太敏感，良品/反面在孔级
    完全重叠(见参数区注释)。半径比一律带回去给**件级**中位数用，本槽的 has_ball 只看高光。
    量不出来(切边等)一律返回 None，由调用方按"没看见球"记 —— 绝不能拿边缘复制出来的像素
    冒充"有高光"，那是拿假数据换放行。

    收敛圆心(hx,hy)也一并返回：判定只用到 r/高光，但 --debug 的叠加图要画**实际定到哪**，
    而不是只知道"从哪出发" —— 两者差得远就说明精定位被旁边的棱/孔壁拽走了。

    `work` 传的是 `_F32View`(见该类注释)，不是原图 —— 只为省掉 refine_hole 里重复的整幅转换。
    """
    got = refine_hole(work, float(px), float(py), float(REFINE_R_SEED_RATIO * pitch_r),
                      cos_t, sin_t)
    if got is None:
        return None
    hx, hy, hr, _contrast = got
    r_ratio = hr / max(pitch_r, 1e-6)

    half = max(4, int(round(HIGHLIGHT_R_RATIO * hr)))
    patch, out_of_frame = crop_pad(gray_f, hx, hy, half)
    if out_of_frame or patch.size == 0:
        return None
    h, w = patch.shape[:2]
    m = _disk_mask(h, w, half)
    if int(m.sum()) < 4:
        return None
    # 基线用中位数：局部油污、整片台面亮度漂移都只影响中位数以外的少数像素，分位数差因此稳
    highlight = float(np.percentile(patch[m], HIGHLIGHT_PCT) - np.median(patch[m]))
    # 孔级只判"有没有球" = 只看高光。半径比**刻意不参与**本槽判定，它归件级中位数用
    # (旧写法在这里 AND 了 rr>=0.144，实测把 45/423 张好件判死，见参数区注释)。
    has_ball = bool(highlight >= BALL_HIGHLIGHT_MIN)
    return hx, hy, hr, r_ratio, highlight, has_ball


def inspect_missing(bgr: np.ndarray, name: str = "") -> MissingResult:
    """单帧缺粒判定。**任何"看不清/定不出"都判 NG**(fail-safe：逃逸不可接受，过杀可接受)。"""
    t0 = time.perf_counter()
    res = MissingResult(name=name)

    def finish() -> MissingResult:
        res.elapsed_ms = (time.perf_counter() - t0) * 1000.0
        return res

    bgr, gray, work, _clahe, _scale = preprocess(bgr)
    # 整幅转 float32 **一次**，之后所有 refine_hole 调用都用这个替身(见 _F32View)。
    # 原先 refine_hole 内部每调一次就转一遍整幅图，一帧 30+ 次 = ~29ms 白烧。
    work_view = _F32View(work.astype(np.float32))
    cand = detect_hole_candidates_fast(work)
    if len(cand) < POCKET_MIN_COUNT:
        res.verdict = NG_POCKET_NOT_FOUND
        res.reason = "兜孔候选不足: %d < %d" % (len(cand), POCKET_MIN_COUNT)
        return finish()

    # 先精定位再定位工件：refine_hole 不需要工件中心，而节圆拟合在精定位孔心上更干净
    # (粗 Hough 候选里混着大量伪圆)。与 inspector_pure.inspect() 同一套顺序与参数。
    ang = np.radians(np.arange(0.0, 360.0, ip.REFINE_ANGLE_STEP_DEG))
    cos_t = np.cos(ang).astype(np.float32)
    sin_t = np.sin(ang).astype(np.float32)
    refined: List[Tuple[float, float, float, float]] = []
    for (x0, y0, r0) in cand:
        got = refine_hole(work_view, float(x0), float(y0), float(r0), cos_t, sin_t)
        if got is not None:
            refined.append(got)
    if len(refined) < POCKET_MIN_COUNT:
        res.verdict = NG_POCKET_NOT_FOUND
        res.reason = "精定位后有效兜孔不足: %d < %d" % (len(refined), POCKET_MIN_COUNT)
        return finish()

    part, method = locate_part_fast(work, cand, refined)
    res.locate_method = method
    if part is None:
        res.verdict = NG_LOCATE_FAIL
        res.reason = "定位失败: 图中找不到保持架"
        return finish()
    # 工件在位守门：locate_part_fast 结构性地产不出 mask_centroid(它只走节圆拟合)，这里保留
    # 这条判断是为了"换个定位器也仍然守门"——它是第一站 REJECT_MASK_CENTROID 语义的镜像。
    if ip.REJECT_MASK_CENTROID and method == "mask_centroid":
        res.part_cx, res.part_cy, res.pitch_r = part
        res.verdict = NG_LOCATE_FAIL
        res.reason = "工件不在位: 仅靠 mask_centroid 兜底定位，疑似残件或底板"
        return finish()
    res.part_cx, res.part_cy, res.pitch_r = part

    # 节圆不可信就判不了"这个兜孔是不是兜孔"。第一站的孔级闸在节圆不可信时会全部否决，
    # 本站没有"全部否决"这个选项(否决=全判 NG)，但也不能放行 -> 直接整帧 NG。
    if ip.HOLE_PITCH_GATE and method.split("(")[0] not in ip.HOLE_PITCH_GATE_METHODS:
        res.verdict = NG_LOCATE_FAIL
        res.reason = "节圆不可信(method=%s): 无法逐个兜孔判定，fail-safe 整帧判 NG" % method
        return finish()

    r_med = float(np.median([q[2] for q in refined]))  # 仅用于日志：候选圆的典型半径
    h_img, w_img = gray.shape[:2]
    gray_f = gray.astype(np.float32)

    # ---- ① 锁相位：兜孔一圈 N_POCKETS 个、等角分布，只要在节圆上定出 >=PHASE_MIN_POCKETS 个，
    #      就能把整圈相位锁出来。用**圆均值**：角度乘 N_POCKETS 后等角分布的各分量同相叠加，
    #      随机的伪圆/噪声方向互相抵消，而取模 360° 天然消掉了"这是第几个兜孔"的歧义。
    on_pitch = [q for q in refined
                if hole_on_pitch(q[0], q[1], res.part_cx, res.part_cy, res.pitch_r)]
    # 先留证再判：即便相位锁不住(下面立刻 NG)，--debug 的叠加图也画得出"当时看见了哪几个孔"
    res.seed_pts = [(float(q[0]), float(q[1]), float(q[2])) for q in on_pitch]
    if len(on_pitch) < PHASE_MIN_POCKETS:
        res.verdict = NG_POCKET_NOT_FOUND
        res.reason = ("节圆上只定出 %d 个兜孔(<%d)，锁不住相位"
                      % (len(on_pitch), PHASE_MIN_POCKETS))
        return finish()
    theta = np.array([np.arctan2(q[1] - res.part_cy, q[0] - res.part_cx) for q in on_pitch])
    z = np.mean(np.exp(1j * N_POCKETS * theta))
    res.phase_coh = float(abs(z))
    res.phase_deg = float((np.degrees(np.angle(z)) / N_POCKETS) % POCKET_ANGLE_STEP)

    # ---- ② 合成整圈 18 槽并逐个量测 ----
    #      **这一步是二期的关键**：不再"只量 Hough 检出来的那几个"(那样漏检的兜孔等于从没被判过，
    #      整帧只是被"候选不足"拦下 —— 那是数量闸，不是算法看见了空兜)，而是按锁出来的相位把
    #      18 个标称位置全铺出来，每个都精定位一次。判定因此不再依赖 Hough 的召回率。
    # 切边闸只护一件事：兜孔的**球环整圈是否落在画面内**。中心离每条边 >= 一个球环半径
    # (= REFINE_R_SEED_RATIO * 节圆半径 ≈ 47px)时，refine 能像内部槽一样量准；更近则球环被
    # 画框截断，由 refine_hole 自身的越界掩膜(inspector_pure.py:1898 出框射线取 NaN)+ 最少
    # 跨越点闸(REFINE_MIN_EDGE_PTS=40)拒掉 -> 返回 None -> 按无球计(fail-safe 不变)。
    # 原先的 ×1.6 是拍脑袋的安全膨胀：它把**完全量得出来**的近边槽(实测中心离边 61~75px、
    # 球环半径仅 ~47px、整圈在框内)误判成切边->无球，在良品上是**底边过杀**风险(探针实测
    # #4/#5 精定位收敛良好却被切边阈扔掉)。收到 ×1.0：既补回这些槽，又不放宽任何量测本身的闸。
    margin = REFINE_R_SEED_RATIO * res.pitch_r
    pockets: List[PocketResult] = []
    n_fail = 0
    for k in range(N_POCKETS):
        a = np.radians(res.phase_deg + k * POCKET_ANGLE_STEP)
        px = res.part_cx + res.pitch_r * np.cos(a)
        py = res.part_cy + res.pitch_r * np.sin(a)
        if not (margin <= px <= w_img - margin and margin <= py <= h_img - margin):
            n_fail += 1  # 切边的槽判不了球，与"没看见球"同等待遇(绝不放行)
            RUNTIME_LOGGER.debug("[SLOT] %s #%d (%.1f,%.1f) 切边 -> 记为无球", name, k, px, py)
            continue
        got = _measure_slot(work_view, gray_f, px, py, res.pitch_r, cos_t, sin_t)
        if got is None:
            n_fail += 1
            RUNTIME_LOGGER.debug("[SLOT] %s #%d (%.1f,%.1f) 精定位/量测失败 -> 记为无球",
                                 name, k, px, py)
            continue
        hx, hy, hr, r_ratio, highlight, has_ball = got
        pockets.append(PocketResult(index=k, cx=px, cy=py, rx=hx, ry=hy, r=hr,
                                    r_ratio=r_ratio, highlight=highlight,
                                    has_ball=has_ball))
        RUNTIME_LOGGER.debug(
            "[SLOT] %s #%d (%.1f,%.1f) r=%.1f 半径比=%.3f 高光=%.1f has_ball=%s",
            name, k, px, py, hr, r_ratio, highlight, has_ball)
    res.pockets = pockets
    res.n_slot_fail = n_fail
    if pockets:
        res.ball_r_med = float(np.median([p.r for p in pockets]))
        res.r_ratio_med = float(np.median([p.r_ratio for p in pockets]))

    # ---- ③ 判定(两级，见参数区注释)：任何一级不过都判 NG，量不出来的槽按没球算(fail-safe)。
    #
    #   孔级(存在性)：18 个兜孔必须个个见球面高光 —— 这是**正面缺球**的判据。
    #   件级(落座状态)：整圈半径比中位数必须够大 —— 这是**反面浮球 / 定位跑偏**的判据。
    #     两级量的是两个不同维度，谁也替不了谁：反面浮球时高光照样达标(实测 76.7% 的槽 ≥120)，
    #     而正面只缺一颗球时中位数几乎不动(rr_med 最高可到 0.165)。所以两道闸都得在。
    n_empty = sum(1 for p in pockets if not p.has_ball) + n_fail
    res.n_empty = n_empty
    if n_empty > MAX_EMPTY_POCKETS:
        res.verdict = NG_EMPTY_POCKET
        res.reason = ("缺粒: %d/%d 个兜孔未见球面高光(高光<%.0f；其中 %d 个量不出来)"
                      % (n_empty, N_POCKETS, BALL_HIGHLIGHT_MIN, n_fail))
    elif res.r_ratio_med < PART_R_RATIO_MED_MIN:
        # 高光都过了但整圈半径比偏小：球没压到位(反面浮球)，或节圆拟合跑偏(锁到内孔) ——
        # 两种都不放行。这是**件级**判据，所以 reason 里给的是中位数、不是某个槽。
        res.verdict = NG_EMPTY_POCKET
        res.reason = ("整圈未压到位: 半径比中位数 %.3f < %.3f(球未全部压到位，或节圆拟合不可信; "
                      "相位一致性 %.2f)" % (res.r_ratio_med, PART_R_RATIO_MED_MIN, res.phase_coh))
    else:
        res.verdict = OK_PASS
        res.reason = ("全部 %d 个兜孔均见球面高光，整圈半径比中位数 %.3f(相位 %.1f°)"
                      % (N_POCKETS, res.r_ratio_med, res.phase_deg))
    return finish()


# ============================  取图/驱动  ============================
def _apply_dirs(args: argparse.Namespace) -> None:
    """把本站的目录常量与存图开关打到 inspector_pure 模块上，实现两站隔离。

    **必须在 setup_runtime_logging() 和构建取图源之前做**：那些函数里的
    RUNTIME_LOG_DIR / OK_SAVE_DIR / NG_SAVE_DIR / RAW_SAVE_DIR 都是 inspector_pure 的
    模块级全局，运行时按名字读；改模块属性即改它们读到的值。
    """
    ip.RUNTIME_LOG_DIR = RUNTIME_LOG_DIR
    ip.OK_SAVE_DIR = OK_SAVE_DIR
    ip.NG_SAVE_DIR = NG_SAVE_DIR
    ip.RAW_SAVE_DIR = RAW_SAVE_DIR
    # 上面四个只改"存到哪儿"；RAW **开不开**是 SAVE_RAW_IMAGE 这个独立开关。
    # 必须在 build_source() 之前设：RAW 存图器是 Vn2000Source.__init__ 构造时建的
    # (inspector_pure.py:1086 `RawFrameSaver() if (SAVE_RAW_IMAGE and ...) else None`)，
    # 构造完再打开就晚了 —— 第一帧不会落盘。语义与第一站 inspector_pure.main() 的
    # `if args.debug: SAVE_RAW_IMAGE = True` 对齐(见 inspector_pure.py:3056)。
    if args.debug:
        ip.SAVE_RAW_IMAGE = True


def build_source(args: argparse.Namespace):
    """建取图源。相机模式复用第一站的 Vn2000Source(同 10001 私有协议)。"""
    if args.mode == "local":
        src = ip.LocalFolderSource(args.dir, ip.LOCAL_RECURSIVE)
        print("[INFO] 取图模式: LOCAL  %s  (%d 张)" % (args.dir, len(src)))
        return src
    if args.mode == "watch":
        src = ip.WatchFolderSource(args.dir, ip.WATCH_RECURSIVE, ip.WATCH_MAX_FRAMES)
        print("[INFO] 取图模式: WATCH  %s" % args.dir)
        return src
    if args.mode == "http":
        print("[INFO] 取图模式: HTTP  %s" % ip.CAMERA_URL_TEMPLATE.format(camera_ip=args.ip))
        return ip.HttpCameraSource(args.ip, ip.HTTP_MAX_FRAMES, ip.HTTP_INTERVAL_S)
    src = ip.Vn2000Source(args.ip, ip.CAMERA_PORT, ip.CAM_MAX_FRAMES)
    how = ("IO 外部硬触发(被动等相机推帧)" if ip.Vn2000Source._is_external()
           else "软触发 %s" % ip.CAM_TRIGGER_ORDER)
    print("[INFO] 取图模式: CAMERA 直连 %s:%d  触发=%s" % (args.ip, ip.CAMERA_PORT, how))
    return src


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """本站自己的参数区。阈值等一律留在文件头部常量里，不收进命令行(免得两处配置打架)。"""
    ap = argparse.ArgumentParser(description="兜孔缺粒检测站(第二站)")
    ap.add_argument("--mode", choices=("local", "camera", "watch", "http"), default="local",
                    help="取图方式(默认 local，离线跑样本)")
    ap.add_argument("--dir", default=DEFAULT_SAMPLE_DIR, help="local/watch 样本目录")
    ap.add_argument("--ip", default="169.254.44.202", help="本站相机 IP(缺粒站)")
    ap.add_argument("--trigger", choices=("external", "MainRunOnce", "ContinuousImageCapture"),
                    help="覆盖相机触发方式(默认 external 硬触发)")
    ap.add_argument("--limit", type=int, default=0, help="相机模式最多取多少帧(0=无限)")
    ap.add_argument("--quiet", action="store_true", help="控制台只留 WARNING+，逐帧明细只进文件")
    ap.add_argument("--timing", action="store_true", help="打印耗时统计")
    ap.add_argument("--debug", action="store_true",
                    help="逐兜孔明细进日志，并存结果图：叠加图(把 18 个槽的定位结果全画出来)"
                         "+ 原始帧。默认只存判 NG 的帧。")
    ap.add_argument("--save-ok", dest="save_ok", action="store_true",
                    help="配合 --debug：判 OK 的帧也存一份。调判据余量时要看良品的槽才够用；"
                         "占盘，产线上别开。")
    ap.add_argument("--no-uno", action="store_true",
                    help="不驱动执行器(离线跑样本用；产线上绝不要加)")
    return ap.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """入口。经主程序启动时，UnoRelayController 已被换成 IpcRelay，判定会转发给主程序开闸 D9(放 NG 件落盒)。"""
    setup_console()
    args = parse_args(argv)
    if args.trigger:
        ip.CAM_TRIGGER_ORDER = args.trigger
    if args.limit > 0:
        ip.CAM_MAX_FRAMES = args.limit
    _apply_dirs(args)  # 必须在 setup_runtime_logging()/build_source() 之前：目录隔离 + --debug 开 RAW
    setup_runtime_logging()
    log = RUNTIME_LOGGER
    if args.debug:
        # --debug 承诺的"逐兜孔明细进日志"是 [SLOT] 那些 RUNTIME_LOGGER.debug() 行
        # (见 inspect_missing 内的 SLOT 明细)，默认 INFO 级别下打不出来，必须降 logger 门槛。
        # 与 inspector_pure.py:3059 同义。--quiet 抬的是控制台 handler、--debug 降的是 logger，
        # 两者正交：`--quiet --debug` = 明细只进文件、不刷屏(正是产线组合)。
        log.setLevel(logging.DEBUG)
    log.info("[MISSING] 第二站=兜孔缺粒 | 日志=%s | 结果图 OK=%s / NG=%s | 判定经主程序开闸 D9(放 NG 件落盒)",
             RUNTIME_LOG_DIR, OK_SAVE_DIR, NG_SAVE_DIR)
    log.info("[MISSING] 二期判据已用 598 张全量样本复跑标定(正面OK 423/正面NG 44/反面NG 44/无件 87；"
             "过杀 0/423、逃逸 0)：孔级 球面高光>=%.0f(18 个兜孔逐个) ＋ 件级 整圈半径比中位数>=%.3f。"
             "样本量仍不算大(尤其反面 44 张)，上线前须再复跑零逃逸自检。",
             BALL_HIGHLIGHT_MIN, PART_R_RATIO_MED_MIN)

    if args.quiet:
        # 控制台抬到 WARNING：逐帧 [INSPECT-DONE] 这类 INFO 不再经本进程 stdout 冒出来，
        # 也就不会被主程序 _drain 排空到控制台 —— 但**文件 handler 不动**，逐帧明细照进
        # data/missing/logs 的 .log(事后对账/verify 都从 .log 读，见 verify_l1_missing)。
        # 刻意放在上面两条 [MISSING] 启动横幅之后：那两行要留在控制台(经主程序汇聚显示)。
        # 与 inspector_pure.py:3070 同一手法；--debug 降 logger 门槛与这里抬 handler 门槛正交,
        # `--quiet --debug` = 明细只进文件、不刷屏(产线组合)。
        for _h in RUNTIME_LOGGER.handlers:
            if getattr(_h, "stream", None) is sys.stdout:
                _h.setLevel(logging.WARNING)
        print("[日志] 控制台已静默(仅 WARNING+)，完整逐帧明细写入: %s" % RUNTIME_LOG_DIR,
              flush=True)

    try:
        source = build_source(args)
    except Exception as exc:  # noqa: BLE001
        log.critical("[FATAL] 取图源建立失败: %s", exc)
        return 2

    # 执行器在"图源(相机)已就绪"之后再连 —— 与 inspector_pure 的 main() 同序(先 build_source
    # 再 connect UNO)。这一顺序对 IpcRelay 很关键：_run_child 已先 connect() 建好 IPC，这里的
    # 第二次 connect() 恰好落在"相机通了"的时刻，IpcRelay 会借它给主程序发 UP 点亮"全部正常"绿灯。
    # 离线跑样本(--no-uno)时执行器没有意义；产线上永远走主程序(那时这里是 IpcRelay)。
    uno = None
    if not args.no_uno:
        try:
            from uno_relay import UnoRelayController
            uno = UnoRelayController()
            if not uno.connect():
                log.warning("[WARN] 执行器未连接，检测继续运行，但 NG 不会开闸落料")
                uno = None
        except Exception as exc:  # noqa: BLE001
            log.warning("[WARN] 执行器初始化失败: %s", exc)
            uno = None

    deadline_active = args.mode != "local"  # 本地样本没有触发节拍，不算时限
    counts = {"ok": 0, "ng": 0, "bad": 0, "timeout": 0, "timeout_backlog": 0,
              "uno_fail": 0, "no_actuator": 0, "labeled": 0, "hit": 0, "miss": 0}
    consecutive_timeouts = 0
    all_ratios: List[float] = []  # 标定用：全帧兜孔"半径比"汇总
    all_highlights: List[float] = []  # 标定用：全帧兜孔"球面高光"汇总
    tally: Dict[str, List[int]] = {}  # 目录 -> [张数, 判OK数, 判NG数]：一眼看清哪套样本偏成什么样
    t_start = time.time()
    exit_code = 0

    def actuate(image_name: str) -> str:
        """判 NG 立刻开闸落料(关键路径)：D9 脉冲让闸门打开，放这件 NG 掉进回收盒。返回开闸状态文字。"""
        if uno is None:
            counts["no_actuator"] += 1
            return "无UNO未开闸"
        if uno.pulse():
            return "已开闸"
        counts["uno_fail"] += 1
        RUNTIME_LOGGER.error("[IO-ERROR] image=%s 脉冲发送失败", image_name)
        return "开闸失败"

    try:
        for seq, (name, bgr) in enumerate(source.frames(), start=1):
            if bgr is None:
                counts["bad"] += 1
                log.warning("[FRAME-INVALID] %s，按 NG 处理", name)
                log.info("[INSPECT-DONE] #%d %s  判定 NG  耗时 0.0ms  [%s]  图像读不出来(无效帧 NG)\n",
                         seq, name, actuate(name))
                continue

            # --- 出队过期预筛(队列保护/防雪崩)：已在队列堆到过期就**跳过算法** ---
            #     与第一站 inspector_pure.py:3249 同一条机制。本站是 IO 硬触发，件在往下流，
            #     队列积压说明**前面若干帧已经超预算**，此刻再把这帧算完只是白烧 CPU ——
            #     它反正已经赶不上开闸，算出来的读数也不会改变处置(下面照样强制 NG + 开闸)。
            #     跳过它等于把 CPU 让给还没排到的帧，让积压有机会排空。
            #     **只在相机模式生效**：LocalFolderSource / WatchFolderSource / HttpCameraSource
            #     都不设 current_frame_enqueued_at(那是 Vn2000Source 采集线程入队时打的，
            #     inspector_pure.py:1637)，getattr 取到 None ⇒ 这条分支在离线跑样本与
            #     全部 L1/L2/对拍闸里**恒不触发**，对既有验收是逐位中性的。
            enqueued_at = getattr(source, "current_frame_enqueued_at", None)
            if deadline_active and enqueued_at is not None:
                age_ms = max(0.0, (time.perf_counter() - float(enqueued_at)) * 1000.0)
                if age_ms > RESULT_DEADLINE_MS:
                    res = MissingResult(
                        name=name, verdict=NG_TIMEOUT,
                        reason="队列积压 %.1fms > %.1fms，跳过检测强制 NG"
                               % (age_ms, RESULT_DEADLINE_MS))
                    counts["ng"] += 1
                    counts["timeout"] += 1
                    counts["timeout_backlog"] += 1
                    consecutive_timeouts += 1
                    # 先开闸、再记日志：判了 NG 却没开闸 = 逃逸，这是本分支唯一不能出错的地方。
                    uno_status = actuate(name)
                    log.critical("[TIMEOUT-NG] cause=queue_backlog frame=#%d queue_wait=%.1fms "
                                 "budget=%.1fms raw_verdict=(skipped)；仍按NG开闸(fail-safe)",
                                 seq, age_ms, RESULT_DEADLINE_MS)
                    # **这里刻意与第一站不同**：第一站的积压分支只写 [TIMEOUT-NG]、不写
                    # [INSPECT-DONE]，但本站的验证工具链是按"每个 NG 恰好一条 [INSPECT-DONE]
                    # 判定 NG"对账的(verify_common.parse_frames 收逐帧序列、
                    # verify_l2_pipeline.check_actuation 拿 [SUMMARY] NG= 与 D9 脉冲数对钉)，
                    # 少一行会让帧序列与开闸次数对不上。所以照常补一行，耗时如实记 0.0ms
                    # (确实一点没算)，原因写在后面。
                    log.info("[INSPECT-DONE] #%d %s  判定 NG  耗时 0.0ms  [%s]  %s\n",
                             seq, name, uno_status, res.reason)
                    if args.debug:
                        _save_debug_image(bgr, name, res, False)
                    # 升级停线必须放在"开闸已发出之后"：触发停线的这一帧本身也要执行到位，
                    # 否则它会变成一次"判了 NG 却没开闸"的漏放。与下面算法超时分支同序
                    # (那里也有一份一模一样的守卫，注释见其上方)。
                    if TIMEOUT_ESCALATE_N > 0 and consecutive_timeouts >= TIMEOUT_ESCALATE_N:
                        raise AcquisitionError("连续 %d 帧判定超时(%.0fms 预算)"
                                               % (consecutive_timeouts, RESULT_DEADLINE_MS))
                    continue

            res = inspect_missing(bgr, name)
            decision_at = time.perf_counter()
            all_ratios.extend(p.r_ratio for p in res.pockets)
            all_highlights.extend(p.highlight for p in res.pockets)
            is_ok = res.is_ok
            reason_text = res.reason

            # --- 超时口径 = **入队 → 判定**(含排队)，与第一站 inspector_pure.py:3296 同一条钟 ---
            # 第一站算的是 window_ms，本站此前算的是 res.elapsed_ms(纯算法时间、不含排队)，
            # 于是"排队等了 180ms + 算了 40ms = 总 220ms"这种帧两边都漏：上面的出队预筛只在
            # **出队时就已经过期**才触发(180 < 200 时不触发)，而 elapsed_ms=40 < 200 也不触发。
            # 可件等的是**总延迟**，不是算法耗时 —— 总延迟越线时这一件本来就赶不上开闸了，
            # 该按 NG 处置。改成窗口口径后：
            #   window_ms = queue_wait + elapsed_ms ≥ elapsed_ms  恒成立
            #   ⇒ 新判据是旧判据的**超集**，只会把原来漏掉的帧补判成 NG，绝不会少判一件。
            #     方向单向安全(只增 NG = 过杀方向)，符合"零逃逸不让步、过杀可接受"。
            # 没有入队时间戳的取图源(watch/http/本地文件夹)退回纯计算口径 ⇒ 行为与改前逐位相同，
            # 所以 L1/L2/598 帧对拍(全 `--mode local`，且本地模式 deadline_active=False)不受影响。
            if enqueued_at is not None:
                window_ms = max(0.0, (decision_at - float(enqueued_at)) * 1000.0)
                queue_wait_ms = max(0.0, window_ms - res.elapsed_ms)
            else:
                window_ms = res.elapsed_ms
                queue_wait_ms = 0.0

            if deadline_active and window_ms > RESULT_DEADLINE_MS:
                # 判定来不及 -> 这一件多半已经来不及开闸，但绝不放行：仍按 NG 处置并记醒目日志
                is_ok = False
                counts["timeout"] += 1
                consecutive_timeouts += 1
                reason_text = "超时 %.1fms(含排队 %.1fms) > %.1fms预算(原判 %s)" % (
                    window_ms, queue_wait_ms, RESULT_DEADLINE_MS, res.verdict)
                # 三个量都记：只看 inspect 会误判成"算法慢"，只看 window 又不知道慢在哪。
                # cause 仍叫 algo_slow 与第一站对齐(那边也用它涵盖"算完了但窗口越线")，
                # 真正区分"排队型 / 计算型"的是 queue_wait 与 inspect 两个数。
                log.critical("[TIMEOUT-NG] cause=algo_slow frame=#%d queue_wait=%.1fms inspect=%.1fms"
                             " window=%.1fms budget=%.1fms raw_verdict=%s；仍按NG开闸(fail-safe)",
                             seq, queue_wait_ms, res.elapsed_ms, window_ms,
                             RESULT_DEADLINE_MS, res.verdict)
            else:
                consecutive_timeouts = 0

            # 帧间空一行分隔，与第一站 inspector_pure.py 的观感一致(那边 :3329 的 sep 与
            # log_frame_timing 末行各补一处)，否则逐帧明细糊成一片、`--debug` 下尤其难读。
            # **本站固定补**：第一站是按 `--timing` 二选一(不补是留给 [TIMING-INSPECT] 当末行)，
            # 而本站 `--timing` 没有逐帧明细行，[INSPECT-DONE] 就是本帧末行，任何时候都该补。
            if is_ok:
                counts["ok"] += 1
                log.info("[INSPECT-DONE] #%d %s  判定 OK  耗时 %.1fms  槽 %d/%d\n",
                         seq, name, res.elapsed_ms, len(res.pockets), N_POCKETS)
                if args.debug and args.save_ok:
                    _save_debug_image(bgr, name, res, True)
            else:
                counts["ng"] += 1
                log.info("[INSPECT-DONE] #%d %s  判定 NG  耗时 %.1fms  [%s]  %s\n",
                         seq, name, res.elapsed_ms, actuate(name), reason_text)
                if args.debug:
                    _save_debug_image(bgr, name, res, False)

            # 升级停线必须放在"开闸已发出之后"：触发停线的这一帧本身也要执行到位，否则它会变成
            # 一次"判了 NG 却没开闸"的漏放 —— 前 14 帧都正常开闸，偏偏触发停线的第 15 帧没有。
            # 与第一站同序(inspector_pure.py:3283 先 emit_control 吹气，:3336 才 raise)。
            # `> 0` 守卫同第一站：0 = 永不自动停线，只做逐帧 fail-safe。
            if TIMEOUT_ESCALATE_N > 0 and consecutive_timeouts >= TIMEOUT_ESCALATE_N:
                raise AcquisitionError("连续 %d 帧判定超时(%.0fms 预算)"
                                       % (consecutive_timeouts, RESULT_DEADLINE_MS))

            truth = _truth_from_path(name) if args.mode == "local" else None
            if truth is not None:
                counts["labeled"] += 1
                counts["hit" if truth == is_ok else "miss"] += 1
                row = tally.setdefault(os.path.basename(os.path.dirname(name)), [0, 0, 0])
                row[0] += 1
                row[1 if is_ok else 2] += 1
    except KeyboardInterrupt:
        log.info("[STOP] 用户中断(Ctrl-C)")
    except AcquisitionError as exc:
        log.critical("[ACQ-FATAL] 采集/判定链路无法保证逐件完整性，停线: %s", exc)
        exit_code = 3
    finally:
        closer = getattr(source, "close", None)  # LocalFolderSource 没有 close
        if closer is not None:
            try:
                closer()
            except Exception as exc:  # noqa: BLE001
                log.warning("[WARN] 取图源关闭出错: %s", exc)
        if uno is not None:
            uno.close()

    _log_summary(log, counts, all_ratios, all_highlights, tally, time.time() - t_start, args)
    return exit_code


# ============================  --debug 叠加图  ============================
# 配色(BGR)：绿=判有球 / 红=判无球 / 品红=量不出来 / 青=标称槽位 / 黄=节圆 / 蓝=锁相位的实检孔。
# 颜色只管"判据落在哪一边"，不表达置信度 —— 判据余量看每槽标出来的数字。
_DBG_COL_BALL = (80, 220, 80)      # 绿：这个槽见球面高光(孔级过)
_DBG_COL_EMPTY = (70, 70, 240)     # 红：这个槽未见球面高光(孔级不过)
_DBG_COL_FAIL = (255, 0, 220)      # 品红：切边或精定位失败，一律按无球计
_DBG_COL_NOMINAL = (230, 200, 60)  # 青：标称槽位(相位合成出来的那个点)
_DBG_COL_PITCH = (0, 210, 255)     # 黄：节圆与工件中心
_DBG_COL_SEED = (255, 150, 20)     # 蓝：落在节圆上、真正用来锁相位的实检兜孔


def _label(vis: np.ndarray, text: str, pt: Tuple[int, int], color: Tuple[int, int, int]) -> None:
    """在槽位旁边写一行小字，自动翻边避开图边(否则最右/最上的槽标签会被裁掉)。

    **文字一律用 ASCII**：cv2.putText 的 Hershey 字体只有 ASCII 字形，写中文只会印出一排
    豆腐块(要中文得挂 Pillow + 字体文件，为一个调试叠加图不值得引入新依赖)。
    """
    h, w = vis.shape[:2]
    scale, thick = 0.42, 1
    (tw, th), _base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x = pt[0] + 12
    if x + tw > w - 4:  # 贴右边界 -> 改写到左边
        x = pt[0] - 12 - tw
    y = pt[1] - 10
    if y - th < 4:  # 贴顶边 -> 改写到下边
        y = pt[1] + 10 + th
    cv2.putText(vis, text, (max(2, x), y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick,
                cv2.LINE_AA)


def _draw_slots(bgr: np.ndarray, res: MissingResult, is_ok: bool) -> np.ndarray:
    """把**全部**定位结果画到帧上，返回叠加图(不改动入参)。

    画的正好是判据的三个环节：
      ① 节圆 + 工件中心(黄) —— 所有判据都以节圆半径为 1，没有它读不出数；
      ② 锁相位用的实检兜孔(蓝) —— 整圈 18 槽的骨架就是从这几个孔推出来的；
      ③ 合成的 18 个槽 —— 青色小点=标称槽位，它到实检圆的**连线=精定位漂移**，
         圆本身绿=判有球 / 红=判无球，品红十字=这个槽量不出来(也按无球计)。

    **量不出来的槽绝不画圆**：图上多画一个圆就等于谎报"这里量到了"。每个槽都标
    `#序号 半径比/高光`，离判据线(0.144 / 120)差多少一眼可见 —— 比翻 [SUMMARY-SLOT]
    的分位数直观，也正是这张图存在的理由。

    `is_ok` 是**最终处置**(判定 + 超时兜底)，不一定等于 `res.verdict`。两者不一致时脚注会
    标出原始判定 —— 否则一张超时改判的 NG 图会写着 OK_PASS，看图的人会以为存错了目录。
    """
    vis = bgr.copy() if bgr.ndim == 3 else cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)
    vis = np.ascontiguousarray(vis)
    h_img, w_img = vis.shape[:2]

    # ① 节圆 + 工件中心。定位失败时 pitch_r=0，整块跳过(别在 (0,0) 上画个假圆)
    if res.pitch_r > 0:
        cen = (int(round(res.part_cx)), int(round(res.part_cy)))
        cv2.circle(vis, cen, int(round(res.pitch_r)), _DBG_COL_PITCH, 1, cv2.LINE_AA)
        cv2.drawMarker(vis, cen, _DBG_COL_PITCH, cv2.MARKER_CROSS, 30, 1, cv2.LINE_AA)
        # ② 锁相位的实检兜孔
        for (sx, sy, sr) in res.seed_pts:
            cv2.circle(vis, (int(round(sx)), int(round(sy))), int(round(sr)),
                       _DBG_COL_SEED, 1, cv2.LINE_AA)
        # ③ 合成的 18 槽。用与 inspect_missing 同一个公式铺点 —— 图上和日志里必须是同一次量测
        by_index = {p.index: p for p in res.pockets}
        for k in range(N_POCKETS):
            a = np.radians(res.phase_deg + k * POCKET_ANGLE_STEP)
            npt = (int(round(res.part_cx + res.pitch_r * np.cos(a))),
                   int(round(res.part_cy + res.pitch_r * np.sin(a))))
            p = by_index.get(k)
            if p is None:
                cv2.drawMarker(vis, npt, _DBG_COL_FAIL, cv2.MARKER_TILTED_CROSS, 18, 2,
                               cv2.LINE_AA)
                _label(vis, "#%d n/a" % k, npt, _DBG_COL_FAIL)
                continue
            col = _DBG_COL_BALL if p.has_ball else _DBG_COL_EMPTY
            rpt = (int(round(p.rx)), int(round(p.ry)))
            cv2.circle(vis, npt, 2, _DBG_COL_NOMINAL, -1, cv2.LINE_AA)  # 标称槽位
            cv2.circle(vis, rpt, int(round(p.r)), col, 2, cv2.LINE_AA)  # 实检圆
            cv2.line(vis, npt, rpt, _DBG_COL_NOMINAL, 1, cv2.LINE_AA)   # 精定位漂移
            _label(vis, "#%d %.3f/%.0f" % (k, p.r_ratio, p.highlight), npt, col)

    # 判定摘要压在上下两角：叠加图常被单独拿走对照，不能只有图没有结论。
    # 文字用 ASCII(见 _label 的理由)，但每个数是哪个判据、阈值多少，对着常量区一眼能认出来。
    n_ball = sum(1 for p in res.pockets if p.has_ball)
    footer = "%s  ball %d/%d  empty %d  n/a %d  rr_med %.3f  %.1fms" % (
        "OK" if is_ok else "NG", n_ball, N_POCKETS, res.n_empty - res.n_slot_fail,
        res.n_slot_fail, res.r_ratio_med, res.elapsed_ms)
    if not is_ok and n_ball == N_POCKETS and res.n_slot_fail == 0:
        # 孔级全绿却判 NG = 件级闸(整圈半径比中位数)拦下的。不标出来，看图的人会以为
        # "18 个槽全绿怎么还 NG"，反过来怀疑判据坏了。
        footer += "  [PART rr_med<%.3f]" % PART_R_RATIO_MED_MIN
    if is_ok != res.is_ok:  # 超时兜底把 OK 改判成 NG：脚注标出原始判定
        footer += "  (forced, raw=%s)" % res.verdict
    cv2.putText(vis, "pitch_r=%.1f  phase=%.2fdeg  |z|=%.2f  %s" %
                (res.pitch_r, res.phase_deg, res.phase_coh, res.locate_method),
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, _DBG_COL_PITCH, 1, cv2.LINE_AA)
    cv2.putText(vis, footer, (8, h_img - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                _DBG_COL_BALL if is_ok else _DBG_COL_EMPTY, 1, cv2.LINE_AA)
    return vis


def _save_debug_image(bgr: np.ndarray, name: str, res: MissingResult, is_ok: bool) -> None:
    """--debug 只存一张**叠加图**：把全部定位结果画出来(见 `_draw_slots`)。

    看的就是"算法把它当成哪个槽、卡在哪条判据线上" —— 原始像素随时能回样本目录里翻，
    再存一份不画线的原图只是占盘。存盘是调试用途，同步写即可。
    """
    out_dir = OK_SAVE_DIR if is_ok else NG_SAVE_DIR
    slots_path = os.path.join(out_dir, unique_image_name(name, "MISSING_slots", ".png"))
    if not imwrite_unicode(slots_path, _draw_slots(bgr, res, is_ok)):
        RUNTIME_LOGGER.warning("[WARN] 叠加图存盘失败: %s", slots_path)


def _truth_from_path(name: str) -> Optional[bool]:
    """从**目录名**推真值(离线跑样本时自检用)。返回 True=该件正常 / False=真缺粒 / None=推不出。

    本站样本文件名是纯日期戳(`20260928.180428.874.000001.origin.png`)，**文件名里没有标签**，
    标签只在目录上(`缺粒样本/正面/OK`、`缺粒样本/正面/NG`、`缺粒样本/反面/NG`)。所以
    inspector_pure.guess_label() 这类按 basename 猜标签的办法在本站样本上完全无效，只能看目录。

    逐级向上找(文件所在目录 → 父目录 ...)，取最先出现的标签目录；都没有则返回 None
    (调用方据此跳过统计，绝不拿猜出来的标签冒充真值)。
    """
    parts = os.path.normpath(name).split(os.sep)
    for seg in reversed(parts[:-1]):  # 去掉文件名本身，从最近的一级往上找
        tag = seg.strip().upper()
        if tag in ("OK", "良品", "正常", "PASS"):
            return True
        if tag in ("NG", "不良", "缺粒", "FAIL"):
            return False
    return None


def _log_summary(log, counts: Dict[str, int], ratios: List[float], highlights: List[float],
                 tally: Dict[str, List[int]], elapsed_s: float, args) -> None:
    """停机汇总。除常规计数外，打出两个判据量的分位分布 —— 下次复标定直接看这两条。"""
    log.info("[SUMMARY] processed=%d OK=%d NG=%d invalid=%d timeout_ng=%d timeout_backlog=%d "
             "uno_fail=%d no_actuator=%d elapsed=%.2fs",
             counts["ok"] + counts["ng"] + counts["bad"], counts["ok"], counts["ng"],
             counts["bad"], counts["timeout"], counts["timeout_backlog"], counts["uno_fail"],
             counts["no_actuator"], elapsed_s)
    if ratios:
        q = np.percentile(np.asarray(ratios, dtype=np.float64), [0, 1, 25, 50, 75, 99, 100])
        log.info("[SUMMARY-SLOT] 兜孔 %d 个 半径比(球盘/节圆) min/p1/p25/p50/p75/p99/max = %s；"
                 "**这是件级判据**，看的是每帧的中位数(下限 %.3f)，不是单槽 —— 单槽半径比对"
                 "'一颗球坐得多深'太敏感，良品 0.107~0.168 与反面 0.107~0.213 在孔级完全重叠。",
                 len(ratios), " / ".join("%.3f" % v for v in q), PART_R_RATIO_MED_MIN)
    if highlights:
        q = np.percentile(np.asarray(highlights, dtype=np.float64), [0, 1, 25, 50, 75, 99, 100])
        log.info("[SUMMARY-SLOT] 兜孔 %d 个 球面高光(99分位−中位数) min/p1/p25/p50/p75/p99/max = "
                 "%s；**这是孔级判据**(下限 %.0f)：有工件的帧上 空兜 max=78、良品 p1=231，中间是空的"
                 "(空工位『无件』帧上 18 个槽铺在夹具亮斑上，读数不具此含义，那些帧由件级闸兜)。",
                 len(highlights), " / ".join("%.0f" % v for v in q), BALL_HIGHLIGHT_MIN)
    if counts["no_actuator"]:
        log.error("[SUMMARY] ⚠ %d 个 NG 无执行器可开闸，实际未分选", counts["no_actuator"])
    if counts["labeled"]:
        # 真值来自目录名(见 _truth_from_path)，不是文件名 —— 这一行是二期调参时唯一能自检的东西
        log.info("[SUMMARY-TRUTH] 有真值样本 %d 张: 判对 %d / 判错 %d (准确率 %.1f%%)",
                 counts["labeled"], counts["hit"], counts["miss"],
                 100.0 * counts["hit"] / max(counts["labeled"], 1))
        for d in sorted(tally):
            n, n_ok, n_ng = tally[d]
            log.info("[SUMMARY-TRUTH]   目录 %-12s 共 %3d 张 -> 判 OK %3d / 判 NG %3d %s",
                     d, n, n_ok, n_ng, "  <-- 全判 NG(疑过杀)" if n_ok == 0 else "")
    elif args.mode == "local":
        log.info("[SUMMARY] local 模式但没从目录名认出任何真值(目录里需有 OK/NG 层级)")


if __name__ == "__main__":
    sys.exit(main())
