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
已知未修的一处：节圆 RANSAC 偶尔锁定到**中心内孔**(实测 4/423 张正面OK，拟合半径 270/331/359/475
而真值 ~316，相位一致性掉到 0.34~0.75，`--debug` 叠加图随之画歪)。这些帧目前被孔级高光闸判 NG，
即过杀。正解是把相位一致性加进 tier1 的接受条件、不过就退回 tier2，但**不能直接抬
`PITCH_FIT_MIN_HOLES`**(实测抬到 6 只救回一半、净过杀反而上升；抬到 8/10 会把好帧弄成定位失败)。
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
                            detect_hole_candidates, fit_circle_ransac, fit_pitch_anchored,
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
#         · 反面浮球：没压进去(用户：反面压不进去，所以即使球在表面也 NG)
#       ⚠ **半径比在孔级不隔离正/反，千万别拿它当孔级硬闸**：单槽半径比对**一颗球坐得
#         多深**极敏感，良品整圈实测 0.107~0.168，反面NG 整圈实测 0.107~0.213 —— 两组在
#         孔级完全重叠。旧写法 `rr>=0.144 and hl>=120` 就是这么错的：实测把 **45/423 张
#         好件判死**，坏槽高光是 204~234(球明明在，只是坐得浅一点)，半径比 0.131~0.144。
#         用户 2026-10-01 报的 `#17 0.139/208` 就是这个类的样本。
#       但整圈**一起**下移才是反面浮球/定位跑偏的特征，取件级中位数后两组分得很开：
#         良品 min 0.1335 / p5 0.1539，反面 max 0.1422 → 间隙 [0.1422, 0.1520]。
#       ⚠ **件级闸是承重的，不是"保险"**：新判据下 44 张反面里有 **27 张孔级全绿**(每个槽的
#         高光都 ≥120)，只有件级中位数拦得住 —— 去掉它这 27 张全部逃逸。旧写法之所以看着
#         "反面每张都留 ≥11 个空槽"，是孔级 rr 硬闸顺带把反面的槽判空了造出来的**假余量**
#         (同一批反面，旧判据 未见球槽数 11/18/18，新判据 0/0/18)。
#       ⚠ 件级闸裕度只有 **0.0028**(反面最高 0.1422 vs 闸 0.145)，比看起来窄 —— 反面样本
#         翻倍复跑是上线前优先级最高的一件事。
#
#   ⇒ **全量 598 张实测对比**(同一批图，只换判据)：
#        旧 `孔级 rr>=0.144 且 hl>=120`   → 正面OK 过杀 63/423 (14.9%)、逃逸 0/0/0
#        新 `孔级 hl + 件级 rr_med`       → 正面OK 过杀  5/423 ( 1.2%)、逃逸 0/0/0
#     过杀降 12 倍而安全不变。旧孔级 rr 闸对"挡逃逸"的唯一贡献是那 27 张反面，而这 27 张
#     正是件级中位数抓的(且那 27 张在旧判据下也是靠 rr 判空的，两条路殊途同归)。
#     ⚠ 这是 598 张的 in-sample 结果，且反面只有 44 张 —— 上线前必须再复跑零逃逸自检。
BALL_HIGHLIGHT_MIN = 120.0    # 孔级闸：球面高光下限(良品 p1=202，空兜 max=81)
PART_R_RATIO_MED_MIN = 0.145  # 件级闸：整圈半径比中位数下限(间隙 [0.1422, 0.1520]，靠 NG 一侧)
REFINE_R_SEED_RATIO = 0.157   # 精定位的初始半径(取"球压到位"的标称值，两侧都能收敛过去)

# ---- 球面高光的量测口径：兜孔中心 0.35r 内的 (99分位 − 中位数)。
#      基线用中位数：局部油污、整片台面亮度漂移都只影响中位数以外的少数像素，分位数差因此稳。
HIGHLIGHT_R_RATIO = 0.35  # 高光量测区半径系数(以精定位半径 r 为单位)
HIGHLIGHT_PCT = 99  # 高光取 99 分位：只认"少数极亮像素"，抗油污/杂散反光
MAX_EMPTY_POCKETS = 0  # 允许的空兜数(h 级)；0 = 18 个兜孔必须个个见球面高光

# ---- 判定时限 fail-safe：与第一站同一条产线的节拍预算(同源，改一处两站都生效) ----
#      两个值都在 import 时从 inspector_pure 取，是**快照不是活引用**：唯一来源是
#      inspector_pure.py:110/111，运行期再改 ip.* 不会追到这里来。
RESULT_DEADLINE_MS = ip.RESULT_DEADLINE_MS
TIMEOUT_ESCALATE_N = ip.TIMEOUT_ESCALATE_N  # 连续这么多帧超时 -> 停线；0=永不自动停(纯 fail-safe)

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
    """定位工件，返回 ((cx,cy,pitch_r), 方法名)。与 inspector_pure.locate_part 同构，差别只在
    tier2 的外圆搜索降到 1/4 分辨率且只当 seed(理由与实测见 LOCATE_SEED_DOWNSCALE 注释)。

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

    # tier1：三点 RANSAC 直接拟节圆(与第一站逐字一致)
    fit = fit_circle_ransac(ring_pts, ip.PITCH_FIT_ITERS, ip.PITCH_FIT_TOL_RATIO,
                            ip.PITCH_FIT_TOL_MIN_PX, ip.PITCH_FIT_MIN_HOLES,
                            r_floor, ip.PITCH_FIT_RANSAC_MAX_COMBOS)
    if fit is not None and fit[2] > r_floor and fit[3] >= ip.PITCH_FIT_MIN_HOLES:
        return (fit[0], fit[1], fit[2]), "pitch_fit(n=%d)" % fit[3]

    # tier2：拿外圆中心当 seed(孔阵与工件外圆同心)，把 3 点 RANSAC 降成半径投票 + 干净子集重拟合
    seed = _outer_seed_downscaled(work)
    if seed is None:
        return None, "none"
    anch = fit_pitch_anchored(ring_pts, seed[0], seed[1], r_floor,
                              ip.PITCH_FIT_TOL_RATIO, ip.PITCH_FIT_TOL_MIN_PX,
                              ip.PITCH_FIT_MIN_HOLES, ip.PITCH_FIT_ITERS,
                              ip.PITCH_FIT_RANSAC_MAX_COMBOS)
    if anch is not None and anch[2] > r_floor and anch[3] >= ip.PITCH_FIT_MIN_HOLES:
        return (anch[0], anch[1], anch[2]), "pitch_fit(n=%d)" % anch[3]
    return None, "none"


# ============================  判据  ============================
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
    yy, xx = np.mgrid[0:h, 0:w]
    m = np.hypot(xx - (w - 1) / 2.0, yy - (h - 1) / 2.0) <= half * 0.9
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
    cand = detect_hole_candidates(work)
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
        got = refine_hole(work, float(x0), float(y0), float(r0), cos_t, sin_t)
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
        got = _measure_slot(work, gray_f, px, py, res.pitch_r, cos_t, sin_t)
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
             "过杀 5/423、逃逸 0)：孔级 球面高光>=%.0f(18 个兜孔逐个) ＋ 件级 整圈半径比中位数>=%.3f。"
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
    counts = {"ok": 0, "ng": 0, "bad": 0, "timeout": 0, "uno_fail": 0, "no_actuator": 0,
              "labeled": 0, "hit": 0, "miss": 0}
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
                log.info("[INSPECT-DONE] #%d %s  判定 NG  耗时 0.0ms  [%s]  图像读不出来(无效帧 NG)",
                         seq, name, actuate(name))
                continue
            res = inspect_missing(bgr, name)
            all_ratios.extend(p.r_ratio for p in res.pockets)
            all_highlights.extend(p.highlight for p in res.pockets)
            is_ok = res.is_ok
            reason_text = res.reason
            if deadline_active and res.elapsed_ms > RESULT_DEADLINE_MS:
                # 判定来不及 -> 这一件多半已经来不及开闸，但绝不放行：仍按 NG 处置并记醒目日志
                is_ok = False
                counts["timeout"] += 1
                consecutive_timeouts += 1
                reason_text = "超时 %.1fms > %.1fms预算(原判 %s)" % (
                    res.elapsed_ms, RESULT_DEADLINE_MS, res.verdict)
                log.critical("[TIMEOUT-NG] frame=#%d inspect=%.1fms budget=%.1fms raw_verdict=%s"
                             "；仍按NG开闸(fail-safe)", seq, res.elapsed_ms, RESULT_DEADLINE_MS,
                             res.verdict)
            else:
                consecutive_timeouts = 0

            if is_ok:
                counts["ok"] += 1
                log.info("[INSPECT-DONE] #%d %s  判定 OK  耗时 %.1fms  槽 %d/%d",
                         seq, name, res.elapsed_ms, len(res.pockets), N_POCKETS)
                if args.debug and args.save_ok:
                    _save_debug_image(bgr, name, res, True)
            else:
                counts["ng"] += 1
                log.info("[INSPECT-DONE] #%d %s  判定 NG  耗时 %.1fms  [%s]  %s",
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
    log.info("[SUMMARY] processed=%d OK=%d NG=%d invalid=%d timeout_ng=%d "
             "uno_fail=%d no_actuator=%d elapsed=%.2fs",
             counts["ok"] + counts["ng"] + counts["bad"], counts["ok"], counts["ng"],
             counts["bad"], counts["timeout"], counts["uno_fail"], counts["no_actuator"],
             elapsed_s)
    if ratios:
        q = np.percentile(np.asarray(ratios, dtype=np.float64), [0, 1, 25, 50, 75, 99, 100])
        log.info("[SUMMARY-SLOT] 兜孔 %d 个 半径比(球盘/节圆) min/p1/p25/p50/p75/p99/max = %s；"
                 "**这是件级判据**，看的是每帧的中位数(下限 %.3f)，不是单槽 —— 单槽半径比对"
                 "'一颗球坐得多深'太敏感，良品 0.107~0.168 与反面 0.107~0.213 在孔级完全重叠。",
                 len(ratios), " / ".join("%.3f" % v for v in q), PART_R_RATIO_MED_MIN)
    if highlights:
        q = np.percentile(np.asarray(highlights, dtype=np.float64), [0, 1, 25, 50, 75, 99, 100])
        log.info("[SUMMARY-SLOT] 兜孔 %d 个 球面高光(99分位−中位数) min/p1/p25/p50/p75/p99/max = "
                 "%s；**这是孔级判据**(下限 %.0f)：空兜 max=81、良品 p1=202，中间是空的。",
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
