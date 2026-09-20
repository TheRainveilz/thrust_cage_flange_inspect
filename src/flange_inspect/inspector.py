# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import glob
import json
import logging
import os
import queue
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from logging.handlers import RotatingFileHandler
from typing import Deque, Dict, Iterator, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# ===== 新增这里！全局设置OpenCV线程数，直接运行/被import导入都生效 =====
cv2.setNumThreads(4)
# =====================================================================================
# ============================  参 数 区 (现场只改这里)  ================================
# =====================================================================================

# ---------- 1. 取图 / 运行模式 ----------
# "local" =本地文件夹遍历(离线调试)
# "camera"=厂家 10001 端口私有协议真直连(推荐: 相机直接推实时帧, 不落盘, 请求-应答天然握手)
# "watch" =监视存图目录(依赖 MJ_Aisensor 存图, 见下)
# "http"  =相机 HTTP 接口(本机这台没有 Web 服务, 走不通; 保留给别的机型)
SOURCE_MODE = "local"
LOCAL_IMAGE_DIR = r"D:\新建文件夹\WTX3000-360C (DA7486717)"   # 样本目录(正/反面混放)
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")
LOCAL_RECURSIVE = True  # 递归遍历子目录
CAMERA_IP = "169.254.44.201"  # 相机 IP(实测直连: 本机 169.254.44.200/16, 无网关)

# 真直连模式 "camera": 走厂家 10001 端口私有协议, 相机把无压缩 8 位灰度帧直接推过来, 不落盘。
# 2026-09-05 实测走通: 一帧 1280x800 = 1024000 字节, 分 788 个记录块传输; 记录格式与命令表
# 见 Wtx10001Source 的 docstring 与 docs/camera_config.md 1.8。
CAMERA_PORT = 10001  # 厂家控制/数据通道(MJ_Aisensor 连的就是这个口)
# 触发方式(上线用 "external"):
#   "external"               = IO 外部硬触发: 上位机不发任何触发命令, 只保活 + 被动等相机推帧。
#                              一个 IO 触发沿 = 拍一张 = 推一帧, 曝光时刻由工装/PLC 决定；
#                              接收线程按序入 FIFO，算法慢于节拍时会产生可观测排队延迟。
#                              ⚠ 前提: 相机侧「触发源」要在 MJ 里设成 IO 硬触发, 且方案处于运行态;
#                                 本模式下算法一律不发 StopRun(发了会把相机踢出运行态)。
#   "MainRunOnce"            = 软触发(调试用): 上位机每帧发一次执行命令, 收到整帧立刻 StopRun。
#                              ⚠ 实测发一次后相机会一直出图(≈0.8 fps)直到 StopRun, 即"连续自动跑",
#                                 曝光时刻不受工件到位信号控制, 不适合上线。
#   "ContinuousImageCapture" = 连续预览(≈6.25 fps): 只用来看图/调光/压亮带, 不用于判定。
CAM_TRIGGER_ORDER = "external"
CAM_CONNECT_TIMEOUT_S = 3.0  # 建链超时(s)
CAM_GRAB_TIMEOUT_S = 5.0  # 软触发单帧超时(s): 超时按无效帧处理并继续, 不退出
CAM_EXT_WAIT_S = 0.0  # 保留兼容旧配置；新 external RX 线程始终等待真实触发，不制造超时假 NG
CAM_EXT_HINT_S = 30.0  # external: 空等时每隔该秒数打一行"仍在等触发"; 0=不打
CAM_HEARTBEAT_S = 1.0  # 心跳周期(s)。实测不回心跳, 相机推 5~6 条后主动断开
CAM_SETTLE_S = 1.0  # 建链后先等相机把开场帧(HeartBeat/ModeState)推完
CAM_STOP_AFTER_FRAME = True  # 仅软触发有效: 取到一帧就发 StopRun, 别让相机连续跑
CAM_IMG_W = 1280  # 期望帧宽(= 传感器自报 ImageResolutionWidth)
CAM_IMG_H = 800  # 期望帧高; 实收字节数不符时按本高度反推宽度并告警
CAM_INTERVAL_S = 0.0  # 仅软触发有效: 两帧之间的间隔(s); 0=判完立刻触发下一帧
CAM_MAX_FRAMES = 0  # 0 = 无限(上线用); >0 = 取够就退出(调试用)
CAM_RECONNECT_TRY = 3  # 断链后的重连次数
CAM_FRAME_TIMEOUT_S = 2.0  # 已收到 seq=0 后，整帧必须在该时间内收完
CAM_QUEUE_SIZE = 8  # 完整帧缓冲深度：吸收连续 NG 时 UNO 串口等待造成的短时积压；配合出队过期预筛，正常填不满
CAM_SOCKET_RCVBUF = 8 * 1024 * 1024  # 仅吸收 TCP 抖动；不能作为工件安全积压队列
CAM_RECORD_MAX_PAYLOAD = 16 * 1024  # 实测图像块约 1.3 KB；异常大长度用于流重同步
CAM_VALIDATE_RECORD_CHECKSUM = False  # B 方案(2026-09-20 关, 可回退)：一帧要校验约 790 条记录,
#   纯 Python sum() 是接收侧 CPU 大头。TCP 自带 16 位校验和覆盖线路损坏, 本地有线相机极少坏包,
#   故关掉给 RX 线程减负。代价: 极少数 TCP 漏网的坏包可能进帧 → 坏像素。回退(要严格校验): 设 True。
CAM_TCP_NODELAY = True  # B 方案(可回退)：关 Nagle, 主要影响我方 ACK 及时性, 近乎零成本。回退: 设 False。
CAM_RX_PROBE = True  # A 方案(可回退)：RX 探针。统计 recv 最大间隔(GIL 被 inspect 饿死的直接证据),
#   在 [SUMMARY] 打 rx_gap_max_ms / rx_recv_calls。定位"接收慢"是 CPU 还是 GIL 用, 稳定后可设 False。
RESULT_DEADLINE_MS = 250.0  # 超时预算：图像入队 → 算法判定（含排队+BGR+inspect，不含网络取图/存图 IO）；须按现场速度/喷嘴距离实测修订
TIMEOUT_ESCALATE_N = 15  # 连续超时达到该次数升级停线；0=永不自动停(纯 fail-safe)
TIMING_RECENT_WINDOW = 200  # p95 只统计最近这些帧，避免长期运行内存增长

CAMERA_URL_TEMPLATE = "http://{camera_ip}/camera/currentImage"
HTTP_TIMEOUT_S = 2.0  # 单帧取图超时(s)
HTTP_INTERVAL_S = 0.20  # 连续取图间隔(s)
HTTP_MAX_FRAMES = 0  # 上线取 0 = 无限循环; 调试可设有限帧数
HTTP_RETRY = 3  # 取图失败重试次数
HTTP_USE_SYSTEM_PROXY = False  # 本机装了系统代理(Clash 等)时必须为 False:
#   requests 默认读 Windows 系统代理设置, 会把相机 IP
#   也丢给代理(实测报 127.0.0.1:7892 ReadTimeout)

# 目录监视模式 "watch": MJ_Aisensor 照常存图, 本算法盯着存图目录, 新文件一落地就判。
# 2026-09-05 实测: 相机 80 端口拒绝连接, 没有 Web 服务, CAMERA_URL_TEMPLATE 这条路走不通;
# 真直连已改用 "camera" 模式(10001 私有协议)。本模式保留给"必须留存图"或直连不可用的场合,
# ⚠ 前提是 MJ 真的在存图: 实测点 3 次「执行」目录里一张新图都没多, 上线前先确认这条链是通的。
WATCH_RECURSIVE = True  # 递归监视子目录
WATCH_POLL_S = 0.10  # 轮询间隔(s)
WATCH_SETTLE_S = 0.15  # 大小连续两次不变才算写完, 防读到只写了一半的图
WATCH_SKIP_EXISTING = True  # True=启动时的存量图片算已处理, 只等新图; False=先跑存量
WATCH_IDLE_TIMEOUT_S = 0.0  # 无新图超过该秒数就退出; 0=一直等(上线用)
WATCH_MAX_FRAMES = 0  # 0 = 无限

# ---------- 2. 预处理 ----------
RESIZE_MAX_SIDE = 0  # >0 按最长边缩放提速; 0=原图。所有阈值均为比例量,缩放不影响判定
MEDIAN_BLUR_K = 3  # 中值滤波核(奇数,0=关)。压制铁屑/椒盐噪点
GAUSS_BLUR_K = 5  # 高斯核(奇数,0=关)
CLAHE_CLIP = 2.0  # 限制对比度自适应直方图均衡, 抗油污/光照不均
CLAHE_GRID = (8, 8)

# ---------- 3. 圆孔粗定位 (HoughCircles) ----------
HOLE_R_MIN_RATIO = 0.030  # 圆孔半径下限 / 图像宽度  (实测样图 ≈0.042)
HOLE_R_MAX_RATIO = 0.065  # 圆孔半径上限 / 图像宽度
HOLE_MIN_DIST_RATIO = 0.060  # 相邻孔心最小间距 / 图像宽度
HOLE_HOUGH_DP = 1.0
HOLE_HOUGH_P1 = 120  # Canny 高阈值
HOLE_HOUGH_P2 = 55  # 累加器阈值: 调小=多找孔(易误检), 调大=少找孔(易漏)
HOLE_HOUGH_P2_FALLBACK = 32  # 第一遍找到的孔不足时自动降阈值重找一次(0=关闭)
MIN_HOLE_COUNT = 4  # 有效圆孔少于该数 -> 直接 NG (异常保护)。2026-09-20 由 2 提到 4：
#   977 张样本实测，全部真实 OK 件孔数 >=4(最小恰为 4)，唯一一张 NG 误判 OK(#128)只有 3 孔；
#   提到 4 对真实 OK 召回零损失，同时干掉该 NG 逃逸。回退：改回 2。
MAX_HOLE_CANDIDATES = 40  # Hough 候选上限, 防异常图卡死
REJECT_MASK_CENTROID = True  # 工件在位守门：定位落到 mask_centroid 兜底(节圆拟不出且外圆找不到)时
#   直接判工件不在位 NG。实测全部真实 OK 件都走 pitch_fit，无一靠 mask_centroid，故零召回损失；
#   残件/不锈钢底板占画面正是靠 mask_centroid 兜底才误判 OK。回退：设 False。

# ---------- 4. 工件定位: 节圆(孔心圆)拟合 ----------
PITCH_FIT_MIN_HOLES = 3  # 少于该数无法拟合节圆 -> 走兜底定位
PITCH_FIT_ITERS = 6
PITCH_FIT_TOL_RATIO = 0.06  # 内点容差 / 节圆半径
PITCH_FIT_TOL_MIN_PX = 8.0  # 内点容差下限(px)
OUTER_R_MIN_RATIO = 0.18  # 兜底: 外圆/中心大孔 Hough 半径范围 / 图像宽度
OUTER_R_MAX_RATIO = 0.60
OUTER_HOUGH_P2 = 90

# ---------- 5. 孔心精定位 (径向 50% 灰度跨越 + 圆拟合) ----------
REFINE_ANGLE_STEP_DEG = 2.0  # 射线角度步长(度)
REFINE_RADIUS_STEP_PX = 0.5  # 射线径向步长(px)
REFINE_SCAN_BAND = (0.30, 1.70)  # 射线扫描范围 / 粗半径
REFINE_INNER_BAND = 0.55  # 孔内灰度取样: r < 该值*粗半径
REFINE_LAND_BAND = 1.45  # 台面灰度取样: r > 该值*粗半径
REFINE_MIN_CONTRAST = 12  # 孔内/台面灰度差 < 该值 视为不是孔 -> 丢弃
REFINE_EDGE_START = 0.45  # 跨越点搜索起始 / 粗半径
REFINE_MIN_EDGE_PTS = 40  # 有效跨越点下限
REFINE_ITERS = 4
REFINE_INLIER_RATIO = 0.10  # 圆拟合内点容差 / 拟合半径
USE_GLOBAL_HOLE_RADIUS = True  # True=用所有孔半径中位数做统一基准 r (同一工件孔径一致)
HOLE_R_DEV_MAX = 0.25  # 单孔半径偏离中位数超过该比例 -> 该孔判为无效

# ---------- 6. 特征A: 翻边外圈 (孔 ROI 内轮廓计数) ----------
# 实测结论(见文末调试说明): "轮廓计数"在反面也可能计到 2 圈(单圈冲裁边的内外沿),
# 鉴别力弱于"同心 Hough 找翻边圆"。因此默认 contour_or_hough, 且 HOLE_LOGIC 必须保持 AND,
# 由特征B(拐角压痕)提供主要鉴别力。若现场出现反面误判 OK, 先改 contour_and_hough。
FEATURE_A_MODE = "contour_or_hough"  # "contour" / "hough" / "contour_or_hough" / "contour_and_hough"
RING_ROI_RATIO = 1.48  # 孔 ROI 外扩倍数 (含翻边区域)
RING_MASK_RATIO = 1.42  # ROI 内圆形掩膜半径 / r, 屏蔽相邻孔与拐角压痕干扰
RING_BAND = (0.86, 1.36)  # 只接受平均半径落在该环带内的轮廓 / r
RING_THRESH_PCTS = (10, 20, 30, 40, 50, 60, 70, 80, 90)  # 多阈值(灰度百分位)扫描, 抗光照不均
RING_MIN_CONTOUR_PTS = 24  # 轮廓点数下限, 滤掉毛刺碎轮廓
RING_MAX_RADIAL_STD = 0.16  # 径向标准差/平均半径 上限 -> 只留"同心圆"形状
RING_MIN_ANGLE_COVER = 0.20  # 轮廓角度覆盖率下限(0~1), 滤掉短弧碎片
RING_CLUSTER_GAP = 0.08  # 环半径聚类间隔 / r (小于该间隔视为同一圈)
RING_CLUSTER_MIN_HITS = 2  # 一个半径簇至少被 N 个阈值命中才算真环
RING_COUNT_MIN = 2  # 环数 >= 该值 判定"存在翻边外圈"(正面)
FLANGE_HOUGH_BAND = (1.12, 1.36)  # 同心 Hough 找翻边圆的半径范围 / r
FLANGE_HOUGH_P1 = 110
FLANGE_HOUGH_P2 = 20
FLANGE_CENTER_GATE = 0.14  # 翻边圆圆心允许偏离孔心 / r

# ---------- 7. 特征B: 4 个拐角小圆压痕 ----------
# (相对角度°, 距离/r)。角度以"工件中心 -> 孔心"的向外径向方向为 0°, 逆时针为正,
# 因此工件任意旋转时 4 个拐角自动跟随, 无需知道绝对角度。实测值见文件末调试说明。
CORNER_SPEC = ((-131.0, 1.58), (-59.5, 1.79), (60.0, 1.79), (131.0, 1.58))
CORNER_WIN_RATIO = 0.75  # 拐角微小 ROI 半宽 / r
MARK_R_RATIO_RANGE = (0.28, 0.55)  # 压痕半径 / r 允许范围 (实测中位数 ≈0.38)
MARK_HOUGH_P1 = 110
MARK_MAX_CANDIDATES = 6
MARK_DEDUP_DIST_RATIO = 0.18
MARK_HOUGH_P2 = 14  # 压痕 Hough 累加器阈值: 形状级预筛, 非圆毛刺无响应
MARK_CENTER_GATE = 0.22  # 压痕圆心允许偏离拐角 ROI 中心 / r (实测定位重复性 σ≈0.08)
MARK_MASK_RATIO = 1.25  # 圆度计算时的圆形裁剪掩膜半径 / 压痕半径
MARK_THRESH_PCTS = (35, 45, 55, 65)  # 多阈值扫描, 取最佳圆度
MARK_MIN_AREA_RATIO = 0.30  # 连通域面积下限 / (pi*压痕半径^2)
MARK_CIRCULARITY_MIN = 0.75  # circularity = 4*pi*area/perimeter**2 > 该值 判为有效压痕
MIN_VALID_MARKS = 2  # 单孔有效压痕数 >= 该值 -> 特征B 通过(4 个拐角允许油污遮挡 2 个)
FEATURE_B_DISABLE_SOFT_ACCEPT = False  # 特征B 兜底开关(可回退)。False=现状: 压痕圆度未过硬阈值 MARK_CIRCULARITY_MIN
#   时, 仍允许 (position_ok and shape_ok) 宽松接受。True=收紧: 只认 circ_raw > MARK_CIRCULARITY_MIN,
#   去掉宽松兜底。#128/#444 那类临界压痕就是从这条兜底溜过的; 设 True 可清零该类逃逸但会动召回。回退: 设 False。

# ---------- 8. 判定逻辑 ----------
HOLE_CHECK_COUNT = 2  # 参与判定的孔数(按质量排序取前 N); 0 = 全部孔
HOLE_LOGIC = "AND"  # 单孔内 特征A 与 特征B 的组合: "AND"(双特征联合, 勿改) / "OR"
PART_LOGIC = "OR"  # 孔之间: "OR" = 任一孔满足即 OK (按需求 5)
PART_MIN_PASS_HOLES = 1  # PART_LOGIC="OR" 下, 需要多少个受检孔同时(A&B)通过才判 OK(可回退)。
#   1=现状(任一孔过即 OK); 提到 2 = 要求至少 2 个孔都过, 收紧单孔临界逃逸。PART_LOGIC="AND" 时此值忽略。回退: 设 1。

# ---------- 9. 调试 / 存图 ----------
PRINT_DEBUG = True  # 打印每孔轮廓计数、有效压痕数、判定结果
SAVE_NG_IMAGE = True  # NG 样本本地保存
SAVE_OK_IMAGE = False  # OK 样本也保存(追溯用)
SAVE_OVERLAY = True  # 保存时叠加检测结果(孔/ROI/拐角/环) 便于现场看图排查
# ⚠ 采样标阈值时必须关掉(命令行 --no-overlay / --collect):
#   dbg_report 会去分析图上的线条, 存叠加图等于喂错数据
NG_SAVE_DIR = r"D:\zq\result\NG"  # 命令行 --save-dir / --collect DIR 可整体改到别处
OK_SAVE_DIR = r"D:\zq\result\OK"  # (在 DIR 下自动建 OK/ 与 NG/ 两个子目录)
JPEG_QUALITY = 92
SAVE_IMAGE_EXT = ".jpg"  # 存图格式: ".jpg"=省空间(走 JPEG_QUALITY) /
#   ".png"=无损(采样标阈值用, 免得把压缩自变量又加回来)
# 命令行 --save-ext / --collect 可覆盖
RAW_SAVE_EXT = ".png"  # RAW 恒定无损 PNG，与 SAVE_IMAGE_EXT 解耦。RAW 是追溯证据，必须能逐像素
#   复现产线判定：JPEG 有损压缩在圆度卡阈值(0.75)的临界帧上足以让判定翻面，无法复盘。
SAVE_RAW_IMAGE = True  # 完整帧无叠加留证；在采集线程内“帧一组装好就存”，与传感器存图一一对齐
RAW_SAVE_DIR = r"D:\zq\result\RAW"
SAVE_QUEUE_SIZE = 64  # 结果图后台存图队列深度；满了丢最旧留档图并计数，绝不阻塞检测线程
SAVE_JOIN_TIMEOUT_S = 3.0  # 停机时等后台存图线程排空并退出的上限
RAW_SAVE_ON_ASSEMBLY = True  # RAW 落盘点提前到帧组装(采集线程)：协议丢帧/队列过载/判定超时都不会吃掉 RAW
RAW_SAVE_QUEUE_SIZE = 512  # RAW 专用大队列(与结果图分开)：RAW 必须与传感器张数对齐，尽量不丢；正常永远填不满
RUNTIME_LOG_DIR = r"D:\zq\result\logs"
RUNTIME_LOG_MAX_BYTES = 10 * 1024 * 1024
RUNTIME_LOG_BACKUP_COUNT = 5

# ---------- 10. Modbus-TCP 对接 PLC (占位, 默认关闭) ----------
ENABLE_MODBUS = False
PLC_IP = "192.168.1.10"
PLC_PORT = 502
PLC_UNIT_ID = 1
PLC_COIL_OK = 0  # OK 线圈地址
PLC_COIL_NG = 1  # NG 线圈地址
PLC_REG_RESULT = 100  # 结果寄存器: 0=未检 1=OK 2=NG
PLC_REG_HEARTBEAT = 101  # 心跳寄存器

# ---------- 11. Arduino UNO PLC对接 ----------
ENABLE_UNO = True
UNO_PORT = "COM7"
UNO_PIN = 8  # Arduino UNO 输出引脚，当前接 PLC X12
UNO_BAUDRATE = 9600  # 必须与测试2.py/Arduino Serial.begin 一致
UNO_PULSE_SECONDS = 0.05  # UNO 固定NG脉冲时间，仅用于日志

# =====================================================================================
# ==============================  以下为业务逻辑, 现场无需改动  ==========================
# =====================================================================================

NG_PART_NOT_FOUND = "NG_PART_NOT_FOUND"  # 找不到垫片
NG_HOLE_NOT_FOUND = "NG_HOLE_NOT_FOUND"  # 圆孔数量不足
NG_NO_FEATURE = "NG_NO_FEATURE"  # 两个孔都没有有效翻边/压痕特征
NG_INVALID_FRAME = "NG_INVALID_FRAME"  # 非相机源图像读取失败
TIMEOUT_NG = "TIMEOUT_NG"  # 算法超时/队列积压/过载：fail-safe 强制 NG，与缺陷 NG 分开
NG_DEADLINE_EXCEEDED = TIMEOUT_NG  # 旧名兼容别名；新代码统一用 TIMEOUT_NG
OK_PASS = "OK"

RUNTIME_LOGGER = logging.getLogger("thrust_cage_inspect")
RUNTIME_LOGGER.addHandler(logging.NullHandler())


def setup_runtime_logging() -> None:
    """运行日志同时写控制台和滚动文件，采集故障重启后仍可追溯。"""
    RUNTIME_LOGGER.handlers.clear()
    RUNTIME_LOGGER.setLevel(logging.INFO)
    RUNTIME_LOGGER.propagate = False
    formatter = logging.Formatter("%(asctime)s.%(msecs)03d %(levelname)s %(message)s",
                                  datefmt="%Y-%m-%d %H:%M:%S")
    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(formatter)
    RUNTIME_LOGGER.addHandler(console)
    try:
        os.makedirs(RUNTIME_LOG_DIR, exist_ok=True)
        path = os.path.join(RUNTIME_LOG_DIR, "../../runtime.log")
        handler = RotatingFileHandler(path, maxBytes=RUNTIME_LOG_MAX_BYTES,
                                      backupCount=RUNTIME_LOG_BACKUP_COUNT,
                                      encoding="utf-8")
        handler.setFormatter(formatter)
        RUNTIME_LOGGER.addHandler(handler)
        RUNTIME_LOGGER.info("[START] 运行日志: %s", path)
    except OSError as exc:
        RUNTIME_LOGGER.error("[LOG-ERROR] 无法创建文件日志: %s", exc)


class AcquisitionError(RuntimeError):
    """相机链路已无法保证逐件完整性，必须停机处理。"""


# ------------------------------------------------------------------ 基础工具
def imread_unicode(path: str, flags: int = cv2.IMREAD_COLOR) -> Optional[np.ndarray]:
    """cv2.imread 在 Windows 下不支持中文路径, 用 np.fromfile + imdecode 代替。"""
    try:
        buf = np.fromfile(path, dtype=np.uint8)
        if buf.size == 0:
            return None
        return cv2.imdecode(buf, flags)
    except Exception as exc:  # noqa: BLE001
        print("[ERR ] 读图失败 %s : %s" % (path, exc))
        return None


def imwrite_unicode(path: str, img: np.ndarray) -> bool:
    """支持中文路径的写图。"""
    ext = os.path.splitext(path)[1] or ".jpg"
    params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY] if ext.lower() in (".jpg", ".jpeg") else []
    try:
        ok, buf = cv2.imencode(ext, img, params)
        if not ok:
            return False
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        buf.tofile(path)
        return True
    except Exception as exc:  # noqa: BLE001
        print("[ERR ] 存图失败 %s : %s" % (path, exc))
        return False


def crop_pad(src: np.ndarray, cx: float, cy: float, half: int) -> Tuple[np.ndarray, bool]:
    """以 (cx,cy) 为中心裁 2*half 方形; 越界用边缘复制补齐, 第二返回值=是否越界。"""
    x0, y0 = int(round(cx)) - half, int(round(cy)) - half
    x1, y1 = x0 + 2 * half, y0 + 2 * half
    h, w = src.shape[:2]
    pad = (max(0, -y0), max(0, y1 - h), max(0, -x0), max(0, x1 - w))
    sub = src[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
    if sub.size == 0:
        return np.zeros((2 * half, 2 * half), src.dtype), True
    if any(pad):
        sub = cv2.copyMakeBorder(sub, pad[0], pad[1], pad[2], pad[3], cv2.BORDER_REPLICATE)
    return sub, any(pad)


def fit_circle_lsq(pts: np.ndarray) -> Tuple[float, float, float]:
    """Kasa 代数法最小二乘圆拟合: x^2+y^2 = 2ax + 2by + c。"""
    x, y = pts[:, 0].astype(np.float64), pts[:, 1].astype(np.float64)
    a = np.c_[2.0 * x, 2.0 * y, np.ones(len(x))]
    sol, *_ = np.linalg.lstsq(a, x * x + y * y, rcond=None)
    cx, cy = float(sol[0]), float(sol[1])
    r = float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-9)))
    return cx, cy, r


def fit_circle_robust(pts: np.ndarray, iters: int, tol_ratio: float,
                      tol_min: float, min_pts: int) -> Optional[Tuple[float, float, float, int]]:
    """迭代剔野点的圆拟合, 返回 (cx, cy, r, 内点数)。"""
    if len(pts) < min_pts:
        return None
    cur = pts.astype(np.float64)
    cx = cy = r = 0.0
    for _ in range(max(1, iters)):
        cx, cy, r = fit_circle_lsq(cur)
        dev = np.abs(np.hypot(cur[:, 0] - cx, cur[:, 1] - cy) - r)
        keep = dev < max(tol_ratio * r, tol_min)
        if keep.sum() < min_pts or keep.all():
            break
        cur = cur[keep]
    return cx, cy, r, int(len(cur))


def odd(v: float, lo: int = 3) -> int:
    """转成 >=lo 的奇数, 供形态学/滤波核使用。"""
    k = int(round(v))
    if k < lo:
        k = lo
    return k if k % 2 == 1 else k + 1


def rotate_unit(ux: float, uy: float, deg: float) -> Tuple[float, float]:
    """把单位向量 (ux,uy) 旋转 deg 度(图像坐标系, y 向下)。"""
    t = np.radians(deg)
    c, s = float(np.cos(t)), float(np.sin(t))
    return ux * c - uy * s, ux * s + uy * c


def angular_coverage(px: np.ndarray, py: np.ndarray, bins: int = 36) -> float:
    """轮廓点相对中心的角度覆盖率(0~1), 用于滤掉短弧碎片。"""
    if len(px) == 0:
        return 0.0
    ang = (np.degrees(np.arctan2(py, px)) + 360.0) % 360.0
    idx = np.unique((ang / (360.0 / bins)).astype(np.int32))
    return float(len(idx)) / float(bins)


def circularity(contour: np.ndarray) -> float:
    """需求指定的圆度: 4*pi*area/perimeter**2。"""
    area = float(cv2.contourArea(contour))
    per = float(cv2.arcLength(contour, True))
    if per <= 1e-6:
        return 0.0
    return 4.0 * float(np.pi) * area / (per * per)


def find_contours(binary: np.ndarray, mode: int, method: int = cv2.CHAIN_APPROX_SIMPLE) -> List[np.ndarray]:
    """兼容 OpenCV 3/4/5 的 findContours 返回值。"""
    res = cv2.findContours(binary, mode, method)
    return list(res[-2])


def radial_profile(src_f32: np.ndarray, cx: float, cy: float,
                   radii: np.ndarray, cos_t: np.ndarray, sin_t: np.ndarray) -> np.ndarray:
    """沿 360° 射线采样, 返回每个半径上"跨角度中位数"曲线(越界置 NaN)。
    取中位数而不是均值: 相邻孔/局部油污只占少数角度, 中位数天然抗干扰。"""
    xs = (cx + radii[:, None] * cos_t[None, :]).astype(np.float32)
    ys = (cy + radii[:, None] * sin_t[None, :]).astype(np.float32)
    h, w = src_f32.shape[:2]
    inside = (xs > 1.0) & (ys > 1.0) & (xs < w - 2.0) & (ys < h - 2.0)
    vals = cv2.remap(src_f32, np.clip(xs, 0, w - 1), np.clip(ys, 0, h - 1),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).astype(np.float32)
    vals[~inside] = np.nan
    out = np.full(vals.shape[0], np.nan, np.float32)
    good = np.count_nonzero(~np.isnan(vals), axis=1) > 0
    if good.any():
        out[good] = np.nanmedian(vals[good], axis=1)
    return out


# ------------------------------------------------------------------ 取图层
class LocalFolderSource:
    """本地调试: 遍历文件夹里的样本图片。"""

    def __init__(self, folder: str, recursive: bool = True) -> None:
        self.folder = folder
        files: List[str] = []
        pattern = "**/*" if recursive else "*"
        for path in glob.glob(os.path.join(folder, pattern), recursive=recursive):
            if os.path.isfile(path) and os.path.splitext(path)[1].lower() in IMAGE_EXTS:
                files.append(path)
        self.files = sorted(files)

    def __len__(self) -> int:
        return len(self.files)

    def frames(self) -> Iterator[Tuple[str, Optional[np.ndarray]]]:
        for path in self.files:
            yield path, imread_unicode(path)


class HttpCameraSource:
    """HTTP 取图: http://{camera_ip}/camera/currentImage 。

    ⚠ 2026-09-05 现场实测: 相机 80 端口拒绝连接(WinError 10061), 本机这台没有 Web 服务,
    本类当前无法投产, 保留给开放了 HTTP 的机型。产线请用 WatchFolderSource("watch")。
    """

    def __init__(self, ip: str, max_frames: int = 0, interval_s: float = 0.2) -> None:
        self.url = CAMERA_URL_TEMPLATE.format(camera_ip=ip)
        self.max_frames = max_frames
        self.interval_s = interval_s
        try:
            import requests  # 延迟导入, 本地调试无需安装
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("HTTP 取图需要 requests 库: pip install requests") from exc
        self._requests = requests
        self._session = requests.Session()
        # 相机在同一网段直连, 绝不能走系统代理: requests 默认读 Windows 代理设置,
        # 会把 169.254.x.x 也发给本地代理端口, 表现为莫名的 ReadTimeout 而非连接失败。
        self._session.trust_env = HTTP_USE_SYSTEM_PROXY
        if not HTTP_USE_SYSTEM_PROXY:
            self._session.proxies = {"http": None, "https": None}

    def _grab(self) -> Optional[np.ndarray]:
        for attempt in range(max(1, HTTP_RETRY)):
            try:
                resp = self._session.get(self.url, timeout=HTTP_TIMEOUT_S)
                resp.raise_for_status()
                buf = np.frombuffer(resp.content, dtype=np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is not None:
                    return img
                print("[WARN] 第 %d 次取图: 数据无法解码" % (attempt + 1))
            except Exception as exc:  # noqa: BLE001
                print("[WARN] 第 %d 次取图失败: %s" % (attempt + 1, exc))
            time.sleep(0.05)
        return None

    def frames(self) -> Iterator[Tuple[str, Optional[np.ndarray]]]:
        n = 0
        while self.max_frames <= 0 or n < self.max_frames:
            n += 1
            yield "HTTP#%06d" % n, self._grab()
            if self.interval_s > 0:
                time.sleep(self.interval_s)


class WatchFolderSource:
    """产线取图: 监视 MJ_Aisensor 的存图目录, 新图片一落地就判一次。

    相机固件未开放 HTTP(实测 80 端口拒绝连接), 直连取图需实现厂家 10001 端口的私有协议;
    本类是拿到该协议前的产线通路, 代价是多一次落盘。**只读不删**: 存图目录同时是样本库,
    删图会毁掉标阈值要用的样本。已处理过的文件靠内存里的 seen 集合去重, 重启后按
    WATCH_SKIP_EXISTING 决定是否重跑存量。
    """

    def __init__(self, folder: str, recursive: bool = True, max_frames: int = 0) -> None:
        if not os.path.isdir(folder):
            raise RuntimeError("监视目录不存在: %s" % folder)
        self.folder = folder
        self.recursive = recursive
        self.max_frames = max_frames
        self.seen = set(self._scan()) if WATCH_SKIP_EXISTING else set()

    def _scan(self) -> List[str]:
        pattern = "**/*" if self.recursive else "*"
        out: List[str] = []
        for path in glob.glob(os.path.join(self.folder, pattern), recursive=self.recursive):
            if os.path.isfile(path) and os.path.splitext(path)[1].lower() in IMAGE_EXTS:
                out.append(path)
        return out

    @staticmethod
    def _mtime(path: str) -> float:
        """按修改时间排序 = 按到达顺序处理(存图文件名不一定单调递增)。"""
        try:
            return os.path.getmtime(path)
        except OSError:
            return 0.0

    @staticmethod
    def _settled(path: str) -> bool:
        """大小连续两次一致且非空 -> 认为已写完。防止读到只写了一半的 JPEG。"""
        try:
            size = os.path.getsize(path)
            if size <= 0:
                return False
            time.sleep(WATCH_SETTLE_S)
            return os.path.getsize(path) == size
        except OSError:
            return False

    def frames(self) -> Iterator[Tuple[str, Optional[np.ndarray]]]:
        n = 0
        t_idle = time.time()
        while self.max_frames <= 0 or n < self.max_frames:
            fresh = [p for p in self._scan() if p not in self.seen]
            if not fresh:
                if 0 < WATCH_IDLE_TIMEOUT_S <= time.time() - t_idle:
                    print("[INFO] %.1f s 无新图, 结束监视" % WATCH_IDLE_TIMEOUT_S)
                    return
                time.sleep(WATCH_POLL_S)
                continue
            fresh.sort(key=self._mtime)
            for path in fresh:
                if not self._settled(path):
                    continue  # 还在写, 下一轮再取
                self.seen.add(path)
                n += 1
                t_idle = time.time()
                yield path, imread_unicode(path)
                if 0 < self.max_frames <= n:
                    return


@dataclass(frozen=True)
class CameraFrame:
    """一张已通过协议完整性校验、等待算法消费的灰度帧。"""
    frame_id: int
    pixels: bytes
    meta: Dict[str, object]
    started_at: float
    completed_at: float
    chunk_count: int
    enqueued_at: Optional[float] = None  # 成功入队的单调时钟(perf_counter)，超时窗口起点
    enqueued_wall: Optional[float] = None  # 成功入队的墙钟(time.time())，仅供 TIMEOUT-NG 日志记录“图像入队时间”


@dataclass
class FrameTiming:
    """单帧跨接收线程和算法线程的单调时钟测量。"""
    frame_id: Optional[int] = None
    acquisition_ms: Optional[float] = None
    queue_wait_ms: Optional[float] = None
    bgr_convert_ms: Optional[float] = None
    inspect_ms: Optional[float] = None
    uno_ms: Optional[float] = None
    plc_ms: Optional[float] = None
    raw_save_ms: Optional[float] = None
    result_save_ms: Optional[float] = None
    rx_first_to_decision_ms: Optional[float] = None
    rx_first_to_control_ms: Optional[float] = None
    rx_first_to_full_ms: Optional[float] = None
    deadline_margin_ms: Optional[float] = None
    late: bool = False
    control_late: bool = False
    raw_verdict: str = ""
    effective_verdict: str = ""
    queue_depth: Optional[int] = None

    def values(self) -> Dict[str, float]:
        return {key: value for key, value in self.__dict__.items()
                if key.endswith("_ms") and isinstance(value, (int, float))}


class TimingStats:
    """固定窗口的在线耗时统计，只有 --timing 时输出。"""

    def __init__(self, window: int = TIMING_RECENT_WINDOW) -> None:
        self.window = max(1, window)
        self.samples: Dict[str, Deque[float]] = {}
        self.metric_counts: Dict[str, int] = {}
        self.totals: Dict[str, float] = {}
        self.mins: Dict[str, float] = {}
        self.maxs: Dict[str, float] = {}
        self.count = 0

    def add_values(self, values: Dict[str, float]) -> None:
        for key, value in values.items():
            value = float(value)
            self.samples.setdefault(key, deque(maxlen=self.window)).append(value)
            self.metric_counts[key] = self.metric_counts.get(key, 0) + 1
            self.totals[key] = self.totals.get(key, 0.0) + value
            self.mins[key] = min(self.mins.get(key, value), value)
            self.maxs[key] = max(self.maxs.get(key, value), value)

    def add(self, timing: FrameTiming, inspect_timings: Optional[Dict[str, float]] = None) -> None:
        self.count += 1
        self.add_values(timing.values())
        if inspect_timings:
            self.add_values({"inspect.%s" % key: value
                             for key, value in inspect_timings.items()})

    @staticmethod
    def _percentile(values: List[float], pct: float) -> float:
        if not values:
            return 0.0
        values = sorted(values)
        pos = (len(values) - 1) * pct / 100.0
        low = int(pos)
        high = min(low + 1, len(values) - 1)
        return values[low] + (values[high] - values[low]) * (pos - low)

    def summary_lines(self) -> List[str]:
        lines = ["[TIMING-SUMMARY] frames=%d window=%d" % (self.count, self.window)]
        for key in sorted(self.samples):
            recent = list(self.samples[key])
            count = self.metric_counts[key]
            lines.append(
                "[TIMING-SUMMARY] %s count=%d min=%.2f mean=%.2f recent_p95=%.2f max=%.2f"
                % (key, count, self.mins[key], self.totals[key] / count,
                   self._percentile(recent, 95.0), self.maxs[key]))
        return lines


class Wtx10001Source:
    """真直连取图: 厂家 10001 端口私有协议(相机自报 Model VN2000, WTX-3000-360C 是贴牌名)。

    2026-09-05 抓包 + 实测确认, 链路上每条记录都是同一个结构:

        +0   AA 55 AB CD             魔数
        +7   u16 小端 载荷长度        <- 唯一可靠的长度字段(偏移 4 的大端值在图像块上恒为 20)
        +9   0x01=控制帧(JSON), 0x14=图像数据块
        +10  0x00=控制帧, 0x03=图像块
        +13  u16 小端 图像块序号 0..N
        +17  u16 小端 随图结果 JSON 的长度
        +50  载荷
        尾   2 字节 = (sum(头 50 字节) + sum(载荷)) & 0xFF, 再跟一个 0x00

    一帧图 = 788 个图像块: 块 0 的载荷 = 结果 JSON + 像素, 其余块全是像素, 末块比常规块短;
    像素合计 1024000 字节 = 1280 x 800 无压缩 8 位灰度, 与传感器自报 ImageResolution 一致
    (标定样本是 1216 x 1024 的 JPEG, 两者取景不同, 换基准图时注意)。

    两条实测约定:
      - 连上后必须约 1 s 回一条 HeartBeat, 否则相机推 5~6 条心跳就主动断开;
      - 触发分两种(CAM_TRIGGER_ORDER):
          external    IO 外部硬触发(上线用): 独立 RX 线程持续收帧，主线程处理当前件时只允许
                      下一张完整帧等待；再来一张立即过载停机，不覆盖、不形成历史积压。
                      本模式下不发 StopRun，也不在两帧之间清积压。
          MainRunOnce 软触发(调试用): 清积压 -> 发命令 -> 收第一整帧 -> 发 StopRun。
                      ⚠ 相机收到一次 MainRunOnce 会一直出图(≈0.8 fps)直到 StopRun, 即连续自动跑,
                      拍照时刻与工件到位无关, 只适合台上调试。
    """

    MAGIC = b"\xaa\x55\xab\xcd"
    HDR = 50
    TRAILER = 2

    def __init__(self, ip: str, port: int = CAMERA_PORT, max_frames: int = 0) -> None:
        self.addr = (ip, port)
        self.max_frames = max_frames
        self._sock: Optional[socket.socket] = None
        self._buf = bytearray()  # 尚未切出完整记录的残留字节
        self._pix = bytearray()  # 当前帧已收到的像素
        self._meta: Dict[str, object] = {}
        self._expected_seq = 0
        self._frame_started_at = 0.0
        self._frame_chunks = 0
        self._next_frame_id = 1
        self._send_lock = threading.Lock()
        self._socket_lock = threading.Lock()
        self._stop = threading.Event()
        self._reconnect_needed = threading.Event()
        self._frames: queue.Queue[CameraFrame] = queue.Queue(maxsize=max(1, CAM_QUEUE_SIZE))
        self._fatal: Optional[BaseException] = None  # 仅协议/重连类故障置位，触发停线
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self.received_frames = 0
        self.accepted_frames = 0
        self.processed_frames = 0
        self.overload_frames = 0
        self.protocol_drops = 0
        self.record_errors = 0
        self.reconnects = 0
        self.max_queue_depth = 0
        # A 方案 RX 探针：recv 之间的最大间隔(ms) = RX 线程被饿死(拿不到 GIL 去收包)的直接证据；
        # rx_recv_calls 为 recv 调用次数, 供算平均。间隔大 + 残帧多 = inspect 慢帧占 GIL 饿死 RX。
        self.rx_gap_max_ms = 0.0
        self.rx_recv_calls = 0
        self._rx_last_recv_at: Optional[float] = None
        self.current_frame_timing: Optional[FrameTiming] = None
        self.current_frame_started_at: Optional[float] = None
        self.current_frame_enqueued_at: Optional[float] = None  # 超时窗口起点(perf_counter)
        self.current_frame_enqueued_wall: Optional[float] = None  # 图像入队墙钟，TIMEOUT-NG 日志用
        self.current_frame_camera_name: str = ""  # 相机帧号(ImageName)，TIMEOUT-NG 日志用
        # RAW 存图器必须在 RX 线程之前建好，确保第一帧就能落盘，与传感器张数对齐。
        self._raw_saver: Optional["RawFrameSaver"] = (
            RawFrameSaver() if (SAVE_RAW_IMAGE and RAW_SAVE_ON_ASSEMBLY) else None)
        self._open_socket()
        self._reconnect_needed.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat, name="wtx-heartbeat", daemon=True)
        self._heartbeat_thread.start()
        if self._is_external():
            self._rx_thread = threading.Thread(
                target=self._receive_loop, name="wtx-receiver", daemon=True)
            self._rx_thread.start()

    # ---------- 帧封装 ----------
    @classmethod
    def _pack(cls, payload: bytes) -> bytes:
        head = bytearray(cls.HDR)
        head[0:4] = cls.MAGIC
        struct.pack_into("<H", head, 7, len(payload))  # 载荷长度
        head[9] = 0x01  # 控制帧
        struct.pack_into("<H", head, 17, len(payload))  # 控制帧上该字段 = 载荷长度
        return bytes(head) + payload + bytes(((sum(head) + sum(payload)) & 0xFF, 0))

    @classmethod
    def _order(cls, name: str, **extra: object) -> bytes:
        obj: Dict[str, object] = {"CommuniInfo": {"PortCode": "Smsocket3"}, "Order": name}
        obj.update(extra)
        return cls._pack(json.dumps(obj, separators=(",", ":")).encode())

    @classmethod
    def _is_external(cls) -> bool:
        """True = IO 外部硬触发: 上位机只保活, 不发触发命令、不发 StopRun、不清积压。"""
        return str(CAM_TRIGGER_ORDER).strip().lower() in ("", "external", "io", "hard", "none")

    @classmethod
    def _trigger_frame(cls) -> bytes:
        if CAM_TRIGGER_ORDER == "ContinuousImageCapture":
            return cls._order(CAM_TRIGGER_ORDER, ContinuousImageCapture="ImageCapture")
        return cls._order(CAM_TRIGGER_ORDER)

    # ---------- 连接 ----------
    def _open_socket(self) -> None:
        """只建立 socket；线程生命周期由 __init__/close 统一管理。"""
        sock = socket.socket()
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, CAM_SOCKET_RCVBUF)
        if CAM_TCP_NODELAY:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(CAM_CONNECT_TIMEOUT_S)
        sock.connect(self.addr)
        sock.settimeout(0.5)
        with self._socket_lock:
            self._sock = sock
        self._reset_parser()
        actual = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        RUNTIME_LOGGER.info("[ACQ-CONNECT] %s:%d SO_RCVBUF=%d pending_capacity=%d",
                            self.addr[0], self.addr[1], actual, self._frames.maxsize)
        # external 立即启动 RX；若先 sleep，启动窗口内的高速触发仍只能挤在系统缓存。
        # 开场 HeartBeat/ModeState 是控制帧，解析器会自行忽略，无需 drain。
        if not self._is_external():
            if CAM_SETTLE_S > 0:
                time.sleep(CAM_SETTLE_S)
            self._drain()

    def _detach_socket(self) -> Optional[socket.socket]:
        with self._socket_lock:
            sock, self._sock = self._sock, None
        return sock

    @staticmethod
    def _close_socket(sock: Optional[socket.socket]) -> None:
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _reopen_socket(self) -> bool:
        self._close_socket(self._detach_socket())
        self._reset_parser()
        for attempt in range(1, max(1, CAM_RECONNECT_TRY) + 1):
            if self._stop.wait(0.5):
                return False
            try:
                self._open_socket()
                self.reconnects += 1
                self._reconnect_needed.clear()
                RUNTIME_LOGGER.info("[ACQ-RECONNECT] 成功 attempt=%d total=%d",
                                    attempt, self.reconnects)
                return True
            except OSError as exc:
                RUNTIME_LOGGER.warning("[ACQ-RECONNECT] 第 %d 次失败: %s", attempt, exc)
        return False

    def _send(self, data: bytes) -> None:
        with self._send_lock:
            with self._socket_lock:
                sock = self._sock
            if sock is None:
                raise OSError("连接已关闭")
            sock.sendall(data)

    def _heartbeat(self) -> None:
        """保活失败交给接收线程重连，不能静默永久退出。"""
        while not self._stop.wait(CAM_HEARTBEAT_S):
            if self._reconnect_needed.is_set():
                continue
            try:
                self._send(self._order("HeartBeat"))
            except OSError as exc:
                RUNTIME_LOGGER.warning("[ACQ-ERROR] 心跳发送失败: %s", exc)
                self._reconnect_needed.set()
                self._close_socket(self._detach_socket())

    def close(self) -> None:
        if self._stop.is_set():
            return
        if not self._is_external():
            try:
                self._send(self._order("StopRun"))
            except OSError:
                pass
        self._stop.set()
        self._close_socket(self._detach_socket())
        current = threading.current_thread()
        for thread in (self._rx_thread, self._heartbeat_thread):
            if thread is not None and thread is not current:
                thread.join(timeout=2.0)
        # RX 线程停后再关 RAW 存图器：把已入队 RAW 尽量写完，与传感器张数对齐。
        if self._raw_saver is not None:
            self._raw_saver.close()
        self._reset_parser()

    # ---------- 收帧 ----------
    def _reset_frame(self) -> None:
        self._pix = bytearray()
        self._meta = {}
        self._expected_seq = 0
        self._frame_started_at = 0.0
        self._frame_chunks = 0

    def _reset_parser(self) -> None:
        self._buf.clear()
        self._reset_frame()

    def _drop_frame(self, reason: str) -> None:
        if self._expected_seq:
            self.protocol_drops += 1
            RUNTIME_LOGGER.error(
                "[ACQ-ERROR] %s bytes=%d expected_seq=%d drops=%d",
                reason, len(self._pix), self._expected_seq, self.protocol_drops)
            # 残帧(传一半)也留档，标 PARTIAL：保证 RAW 张数与传感器对齐、可追溯断在哪。
            if self._raw_saver is not None and self._pix:
                stamp = str(self._meta.get("ImageName") or "").strip()
                safe = "".join(ch if ch.isalnum() or ch in "-_" else "_"
                               for ch in stamp)[:70]
                name = ("CAM_F%06d_%s" % (self._next_frame_id, safe) if safe
                        else "CAM_F%06d" % self._next_frame_id)
                self._raw_saver.submit(bytes(self._pix), name, partial=True)
        self._reset_frame()

    def _drain(self) -> None:
        """仅供软触发使用；external 禁止调用，否则会丢真实工件。"""
        with self._socket_lock:
            sock = self._sock
        if sock is None:
            return
        sock.settimeout(0.05)
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < 0.5 and sock.recv(1 << 18):
                pass
        except OSError:
            pass
        finally:
            sock.settimeout(0.5)
        self._reset_parser()

    def _records(self) -> List[Tuple[int, int, int, bytes]]:
        """严格切分并校验记录；用游标批量移除，避免每块移动整个 bytearray。"""
        buf = self._buf
        out: List[Tuple[int, int, int, bytes]] = []
        pos = 0
        while len(buf) - pos >= self.HDR:
            if buf[pos:pos + 4] != self.MAGIC:
                next_pos = buf.find(self.MAGIC, pos + 1)
                if next_pos < 0:
                    pos = max(pos, len(buf) - 3)  # 留下可能是下次魔数开头的 3 字节
                    break
                RUNTIME_LOGGER.warning("[ACQ-ERROR] 跳过 %d 个非协议字节", next_pos - pos)
                pos = next_pos
                continue
            n = struct.unpack_from("<H", buf, pos + 7)[0]
            if n > CAM_RECORD_MAX_PAYLOAD:
                self.record_errors += 1
                RUNTIME_LOGGER.error("[ACQ-ERROR] 非法记录长度 %d，重新同步", n)
                pos += 1
                continue
            total = self.HDR + n + self.TRAILER
            if len(buf) - pos < total:
                break
            raw = bytes(buf[pos:pos + total])
            pos += total
            head, pay = raw[:self.HDR], raw[self.HDR:self.HDR + n]
            checksum, terminator = raw[-2], raw[-1]
            if terminator != 0:
                self.record_errors += 1
                RUNTIME_LOGGER.error("[ACQ-ERROR] 记录尾字节非法: 0x%02x", terminator)
                continue
            if CAM_VALIDATE_RECORD_CHECKSUM and checksum != ((sum(head) + sum(pay)) & 0xFF):
                self.record_errors += 1
                RUNTIME_LOGGER.error("[ACQ-ERROR] 记录校验和失败 seq=%d",
                                     struct.unpack_from("<H", head, 13)[0])
                continue
            # 记录类型只按 head[10](kind) 判别: 0x03=图像块, 其余(0x00 控制/心跳/Reply/状态)
            # 交给 _feed 过滤。head[9] 实测在图像块上会取 0x14/0x15 等不同值，不是可靠判据；
            # 旧版正是只看 kind 才能持续取图，不能在此按 head[9] 做白名单，否则会整帧丢弃。
            kind = head[10]
            out.append((kind, struct.unpack_from("<H", head, 13)[0],
                        struct.unpack_from("<H", head, 17)[0], pay))
        if pos:
            del buf[:pos]
        return out

    def _feed(self, kind: int, seq: int, njson: int, pay: bytes) -> Optional[CameraFrame]:
        """按块序号和期望像素数组帧；绝不跨触发拼接。"""
        if kind != 0x03:
            return None
        now = time.perf_counter()
        if seq == 0:
            if self._expected_seq:
                self._drop_frame("新 seq=0 覆盖未完成帧")
            if njson > len(pay):
                self.protocol_drops += 1
                RUNTIME_LOGGER.error("[ACQ-ERROR] njson=%d > payload=%d", njson, len(pay))
                return None
            try:
                meta = json.loads(pay[:njson].decode("utf-8", "replace")) if njson else {}
                self._meta = meta if isinstance(meta, dict) else {}
            except (ValueError, TypeError):
                self._meta = {}
                RUNTIME_LOGGER.warning("[ACQ-ERROR] 首块结果 JSON 无法解析")
            self._pix = bytearray(pay[njson:])
            self._expected_seq = 1
            self._frame_started_at = now
            self._frame_chunks = 1
            stamp = str(self._meta.get("ImageName") or "-")
            RUNTIME_LOGGER.info("[ACQ-START] next_id=%06d camera=%s first_pixels=%d",
                                self._next_frame_id, stamp, len(self._pix))
        else:
            if not self._expected_seq:
                return None  # 建链落在半帧中，等下一个 seq=0
            if seq != self._expected_seq:
                self._drop_frame("块序号不连续 actual=%d" % seq)
                return None
            self._pix.extend(pay)
            self._expected_seq += 1
            self._frame_chunks += 1

        expected_bytes = CAM_IMG_W * CAM_IMG_H
        if len(self._pix) > expected_bytes:
            self._drop_frame("像素超长 expected=%d" % expected_bytes)
            return None
        if len(self._pix) < expected_bytes:
            return None

        frame = CameraFrame(
            frame_id=self._next_frame_id,
            pixels=bytes(self._pix),
            meta=dict(self._meta),
            started_at=self._frame_started_at,
            completed_at=now,
            chunk_count=self._frame_chunks,
        )
        # 帧一组装好就存 RAW(采集线程)：即便后面队列过载丢帧/判定超时，RAW 也已在盘上。
        if self._raw_saver is not None:
            self._raw_saver.submit(frame.pixels, self._frame_name(frame), partial=False)
        self._next_frame_id += 1
        self.received_frames += 1
        self._reset_frame()
        return frame

    def _check_frame_timeout(self) -> None:
        if (self._expected_seq and CAM_FRAME_TIMEOUT_S > 0
                and time.perf_counter() - self._frame_started_at > CAM_FRAME_TIMEOUT_S):
            self._drop_frame("帧内超时 %.2fs" % CAM_FRAME_TIMEOUT_S)

    def _recv_frames(self, timeout_s: float) -> List[CameraFrame]:
        """先消费已有记录，再 recv；一次 recv 中的多帧全部返回。"""
        deadline = None if timeout_s <= 0 else time.monotonic() + timeout_s
        completed: List[CameraFrame] = []
        while not self._stop.is_set():
            records = self._records()
            if records:
                for rec in records:
                    frame = self._feed(*rec)
                    if frame is not None:
                        completed.append(frame)
                # `_records()` 已一次切出当前缓冲中的全部完整记录。
                if completed:
                    return completed
            if deadline is not None and time.monotonic() >= deadline:
                return []
            self._check_frame_timeout()
            with self._socket_lock:
                sock = self._sock
            if sock is None:
                raise OSError("连接已关闭")
            try:
                chunk = sock.recv(1 << 18)
            except socket.timeout:
                continue
            if not chunk:
                raise OSError("相机关闭了连接")
            if CAM_RX_PROBE:
                # 记录本次 recv 拿到数据的时刻与上次的间隔：间隔越大, RX 线程被饿得越久。
                # 超时(空转)不计入, 只量真正收到字节之间的空档。
                now_rx = time.perf_counter()
                if self._rx_last_recv_at is not None:
                    gap_ms = (now_rx - self._rx_last_recv_at) * 1000.0
                    if gap_ms > self.rx_gap_max_ms:
                        self.rx_gap_max_ms = gap_ms
                self._rx_last_recv_at = now_rx
                self.rx_recv_calls += 1
            self._buf.extend(chunk)
        return []

    def _enqueue_frame(self, frame: CameraFrame) -> None:
        """入队完整帧。队满不再锁存停线：丢最旧、计数、醒目告警，继续接收(fail-safe)。

        真正溢出时最旧帧最陈旧，留着也吹不到对应工件；主循环靠“出队过期预筛 + 连续
        超时升级”来处理积压，比因一次瞬时抖动就停整条线更符合现场。协议完整性错误
        仍走 AcquisitionError 停线，与过载区分。
        """
        queued = replace(frame, enqueued_at=time.perf_counter(), enqueued_wall=time.time())
        try:
            self._frames.put_nowait(queued)
        except queue.Full:
            self.overload_frames += 1
            dropped_id = -1
            try:
                dropped = self._frames.get_nowait()  # 丢最旧的等待帧腾位
                dropped_id = dropped.frame_id
            except queue.Empty:
                pass
            RUNTIME_LOGGER.critical(
                "[OVERLOAD] new=%06d 丢弃最旧 dropped=%06d queue=%d/%d "
                "received=%d accepted=%d processed=%d overload=%d；未停线(fail-safe)，"
                "该丢帧无判定无吹气，属漏检风险，请降速/排查",
                frame.frame_id, dropped_id, self._frames.qsize(), self._frames.maxsize,
                self.received_frames, self.accepted_frames, self.processed_frames,
                self.overload_frames)
            try:
                self._frames.put_nowait(queued)
            except queue.Full:
                pass  # 单生产者场景理论到不了这里；再满就本帧也放弃(已计入 overload)
        self.accepted_frames += 1
        depth = self._frames.qsize()
        self.max_queue_depth = max(self.max_queue_depth, depth)
        RUNTIME_LOGGER.info(
            "[ACQ-DONE] id=%06d bytes=%d chunks=%d receive=%.1fms queue=%d/%d",
            frame.frame_id, len(frame.pixels), frame.chunk_count,
            (frame.completed_at - frame.started_at) * 1000.0,
            depth, self._frames.maxsize)

    def _receive_loop(self) -> None:
        """external 专用生产者：持续收包，与算法/串口/存图完全解耦。"""
        try:
            while not self._stop.is_set():
                if self._reconnect_needed.is_set() or self._sock is None:
                    if not self._reopen_socket():
                        if self._stop.is_set():
                            return
                        self._fatal = AcquisitionError("相机重连失败，采集停止")
                        RUNTIME_LOGGER.critical("[ACQ-FATAL] %s", self._fatal)
                        return
                try:
                    for frame in self._recv_frames(0.0):
                        self._enqueue_frame(frame)
                except (OSError, AcquisitionError) as exc:
                    if self._stop.is_set():
                        return
                    if isinstance(exc, AcquisitionError):
                        self._fatal = exc
                        RUNTIME_LOGGER.critical("[ACQ-FATAL] %s", exc)
                        return
                    RUNTIME_LOGGER.warning("[ACQ-ERROR] 直连中断: %s", exc)
                    self._reconnect_needed.set()
                    self._close_socket(self._detach_socket())
        except BaseException as exc:  # noqa: BLE001 - 后台线程异常必须传给主线程停线
            self._fatal = AcquisitionError("接收线程异常: %s" % exc)
            RUNTIME_LOGGER.exception("[ACQ-FATAL] 接收线程意外退出")

    def _wait_frame(self, timeout_s: float) -> Optional[CameraFrame]:
        """软触发同步等待一帧；external 由 RX 线程使用 `_recv_frames`。"""
        frames = self._recv_frames(timeout_s)
        if not frames:
            return None
        if len(frames) > 1:
            RUNTIME_LOGGER.error("[ACQ-ERROR] 软触发一次收到 %d 帧，仅返回首帧", len(frames))
        return frames[0]

    def _grab(self) -> Optional[CameraFrame]:
        """软触发单帧路径。external 不调用本方法。"""
        self._drain()
        self._send(self._trigger_frame())
        frame = self._wait_frame(CAM_GRAB_TIMEOUT_S)
        if frame is not None and CAM_STOP_AFTER_FRAME:
            self._send(self._order("StopRun"))
        return frame

    @staticmethod
    def _to_bgr(pix: bytes) -> Optional[np.ndarray]:
        """无压缩 8 位灰度 -> 3 通道 BGR, 与其它取图源一致, 业务逻辑无需区分来源。"""
        w, h = CAM_IMG_W, CAM_IMG_H
        if len(pix) != w * h:  # 相机改了分辨率/ROI
            if h > 0 and len(pix) % h == 0:
                w = len(pix) // h
                print("[WARN] 实收 %d 字节 != %d x %d, 按 %d x %d 解开(请核对 CAM_IMG_W/H)"
                      % (len(pix), CAM_IMG_W, CAM_IMG_H, w, h))
            else:
                print("[WARN] 实收 %d 字节按高 %d 除不尽, 丢弃本帧" % (len(pix), h))
                return None
        gray = np.frombuffer(pix, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    @staticmethod
    def _frame_name(frame: CameraFrame) -> str:
        stamp = str(frame.meta.get("ImageName") or "").strip()
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in stamp)[:70]
        return "CAM_F%06d_%s" % (frame.frame_id, safe) if safe else "CAM_F%06d" % frame.frame_id

    def _dequeue_external(self) -> CameraFrame:
        # 过载已改为“丢最旧继续跑”，不再置 _fatal；_fatal 只来自协议/重连类故障，需停线。
        # 但已在队列里的帧要先消费完(它们已通过完整性校验)，再看 fatal 停线。
        t_hint = time.monotonic()
        while not self._stop.is_set():
            try:
                return self._frames.get_nowait()
            except queue.Empty:
                if self._fatal is not None:
                    raise AcquisitionError(str(self._fatal))
                try:
                    return self._frames.get(timeout=0.5)
                except queue.Empty:
                    if self._fatal is not None:
                        raise AcquisitionError(str(self._fatal))
                    if CAM_EXT_HINT_S > 0 and time.monotonic() - t_hint >= CAM_EXT_HINT_S:
                        t_hint = time.monotonic()
                        RUNTIME_LOGGER.info("[ACQ-WAIT] 等 IO 触发中，链路线程仍在运行")
        raise AcquisitionError("相机源已关闭")

    def frames(self) -> Iterator[Tuple[str, Optional[np.ndarray]]]:
        n = 0
        while self.max_frames <= 0 or n < self.max_frames:
            if self._is_external():
                frame = self._dequeue_external()
            else:
                try:
                    frame = self._grab()
                except OSError as exc:
                    RUNTIME_LOGGER.warning("[ACQ-ERROR] 软触发直连中断: %s", exc)
                    if not self._reopen_socket():
                        raise AcquisitionError("相机重连失败") from exc
                    continue  # 重连不是一张产品图，不能 yield None 触发虚假 NG
                if frame is None:
                    RUNTIME_LOGGER.warning("[ACQ-ERROR] %.1fs 内未收到软触发整帧 order=%s",
                                           CAM_GRAB_TIMEOUT_S, CAM_TRIGGER_ORDER)
                    continue
            n += 1
            dequeued_at = time.perf_counter()
            convert_start = time.perf_counter()
            bgr = self._to_bgr(frame.pixels)
            convert_end = time.perf_counter()
            if bgr is None:
                raise AcquisitionError("已完成帧无法按预期分辨率转换: id=%d" % frame.frame_id)
            enqueued_at = frame.enqueued_at if frame.enqueued_at is not None else frame.completed_at
            self.current_frame_timing = FrameTiming(
                frame_id=frame.frame_id,
                acquisition_ms=max(0.0, (frame.completed_at - frame.started_at) * 1000.0),
                queue_wait_ms=max(0.0, (dequeued_at - enqueued_at) * 1000.0),
                bgr_convert_ms=max(0.0, (convert_end - convert_start) * 1000.0),
                queue_depth=self._frames.qsize(),
            )
            self.current_frame_started_at = frame.started_at
            # 超时窗口起点用“成功入队”单调时刻；墙钟与相机帧号仅供 TIMEOUT-NG 日志。
            self.current_frame_enqueued_at = enqueued_at
            self.current_frame_enqueued_wall = frame.enqueued_wall
            self.current_frame_camera_name = str(frame.meta.get("ImageName") or "")
            yield self._frame_name(frame), bgr
            self.processed_frames += 1
            if CAM_INTERVAL_S > 0 and not self._is_external():
                time.sleep(CAM_INTERVAL_S)


def build_source(mode: str, folder: str, ip: str):
    """一键切换取图方式。"""
    if mode == "camera":
        src = Wtx10001Source(ip, CAMERA_PORT, CAM_MAX_FRAMES)
        how = ("IO 外部硬触发(被动等相机推帧, 不发触发命令/StopRun)"
               if Wtx10001Source._is_external() else "软触发 %s" % CAM_TRIGGER_ORDER)
        print("[INFO] 取图模式: CAMERA 直连 %s:%d  触发=%s  期望 %d x %d 灰度"
              % (ip, CAMERA_PORT, how, CAM_IMG_W, CAM_IMG_H))
        if Wtx10001Source._is_external():
            print("[INFO] 已连上并保活, 等工件到位的 IO 触发信号...  "
                  "(相机侧「触发源」须为 IO 硬触发且方案在运行态; Ctrl-C 停机)")
        return src
    if mode == "http":
        print("[INFO] 取图模式: HTTP  %s" % CAMERA_URL_TEMPLATE.format(camera_ip=ip))
        return HttpCameraSource(ip, HTTP_MAX_FRAMES, HTTP_INTERVAL_S)
    if mode == "watch":
        src = WatchFolderSource(folder, WATCH_RECURSIVE, WATCH_MAX_FRAMES)
        print("[INFO] 取图模式: WATCH  %s" % folder)
        print("[INFO] 存量 %d 张(%s), 等新图中... Ctrl-C 停止"
              % (len(src.seen), "跳过" if WATCH_SKIP_EXISTING else "不跳过"))
        return src
    src = LocalFolderSource(folder, LOCAL_RECURSIVE)
    print("[INFO] 取图模式: LOCAL  %s  (%d 张)" % (folder, len(src)))
    return src


# ------------------------------------------------------------------ 数据结构
@dataclass
class HoleResult:
    """单个圆孔的检测结果。"""
    index: int
    cx: float
    cy: float
    r: float
    contrast: float = 0.0
    in_frame: bool = True
    raw_contour_count: int = 0  # ROI 内参与统计的原始轮廓数
    ring_count: int = 0  # 同心环数(特征A 主判据)
    ring_radii: List[float] = field(default_factory=list)
    flange_hough_r: Optional[float] = None
    feature_a: bool = False
    corner_hits: List[dict] = field(default_factory=list)
    valid_marks: int = 0  # 有效小圆压痕数(特征B)
    corners_in_frame: int = 0
    feature_b: bool = False
    passed: bool = False


@dataclass
class InspectResult:
    """整幅图的检测结果。"""
    name: str
    verdict: str = OK_PASS
    reason: str = ""
    part_cx: float = 0.0
    part_cy: float = 0.0
    pitch_r: float = 0.0
    locate_method: str = ""
    hole_r: float = 0.0
    holes: List[HoleResult] = field(default_factory=list)
    checked: List[int] = field(default_factory=list)
    elapsed_ms: float = 0.0
    timings_ms: Dict[str, float] = field(default_factory=dict)
    timing_details: Dict[str, object] = field(default_factory=dict)

    @property
    def is_ok(self) -> bool:
        return self.verdict == OK_PASS


# ------------------------------------------------------------------ 预处理
def preprocess(bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """返回 (彩色图,原始gray, work(median+clahe+gauss), clahe_only(仅CLAHE无模糊), scale)。"""
    scale = 1.0
    if RESIZE_MAX_SIDE > 0:
        long_side = max(bgr.shape[:2])
        if long_side > RESIZE_MAX_SIDE:
            scale = RESIZE_MAX_SIDE / float(long_side)
            bgr = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr.copy()
    work = gray.copy()
    if MEDIAN_BLUR_K >= 3:
        work = cv2.medianBlur(work, odd(MEDIAN_BLUR_K))
    clahe_only = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_GRID).apply(work)
    work = clahe_only.copy()
    if GAUSS_BLUR_K >= 3:
        work = cv2.GaussianBlur(work, (odd(GAUSS_BLUR_K), odd(GAUSS_BLUR_K)), 0)
    return bgr, gray, work, clahe_only, scale


# ------------------------------------------------------------------ 圆孔粗定位
def detect_hole_candidates(work: np.ndarray) -> np.ndarray:
    """HoughCircles 粗找圆孔, 返回 Nx3 的 (x, y, r)。

    第一遍用高累加器阈值(准), 数量不够时自动降阈值再找一遍(全)。
    后续还有精定位灰度对比度校验 + 孔径一致性校验兜底, 所以这里宁松勿紧。
    """
    w = work.shape[1]
    r_lo = max(3, int(HOLE_R_MIN_RATIO * w))
    r_hi = max(r_lo + 2, int(HOLE_R_MAX_RATIO * w))
    min_dist = max(8, int(HOLE_MIN_DIST_RATIO * w))
    thresholds = [HOLE_HOUGH_P2]
    if HOLE_HOUGH_P2_FALLBACK > 0 and HOLE_HOUGH_P2_FALLBACK < HOLE_HOUGH_P2:
        thresholds.append(HOLE_HOUGH_P2_FALLBACK)
    cand = np.zeros((0, 3), np.float64)
    for p2 in thresholds:
        circles = cv2.HoughCircles(work, cv2.HOUGH_GRADIENT, dp=HOLE_HOUGH_DP, minDist=min_dist,
                                   param1=HOLE_HOUGH_P1, param2=p2,
                                   minRadius=r_lo, maxRadius=r_hi)
        if circles is not None:
            cand = np.asarray(circles[0], dtype=np.float64)
        if len(cand) >= MIN_HOLE_COUNT:
            break
    if len(cand) > MAX_HOLE_CANDIDATES:  # 半径由大到小截断, 防异常图卡死
        cand = cand[np.argsort(-cand[:, 2])][:MAX_HOLE_CANDIDATES]
    return cand


def locate_part(work: np.ndarray, cand: np.ndarray) -> Tuple[Optional[Tuple[float, float, float]], str]:
    """自动定位工件(不用固定 ROI), 三级策略。返回 ((cx,cy,pitch_r), 方法名)。

    1) 节圆拟合: 孔心共圆 -> 工件中心/节圆半径, 对任意旋转偏移天然免疫,
       外圆被视场切掉也有效(现场半幅视野样图)。
    2) 大圆 Hough: 直接找垫片外圆或中心大孔。
    3) 工件掩膜质心: 最后兜底。
    """
    if len(cand) >= PITCH_FIT_MIN_HOLES:
        fit = fit_circle_robust(cand[:, :2], PITCH_FIT_ITERS, PITCH_FIT_TOL_RATIO,
                                PITCH_FIT_TOL_MIN_PX, PITCH_FIT_MIN_HOLES)
        if fit is not None:
            cx, cy, pr, n_in = fit
            if pr > 1.5 * float(np.median(cand[:, 2])) and n_in >= PITCH_FIT_MIN_HOLES:
                return (cx, cy, pr), "pitch_fit(n=%d)" % n_in

    w = work.shape[1]
    big = cv2.HoughCircles(work, cv2.HOUGH_GRADIENT, dp=1.0, minDist=int(0.30 * w),
                           param1=HOLE_HOUGH_P1, param2=OUTER_HOUGH_P2,
                           minRadius=int(OUTER_R_MIN_RATIO * w),
                           maxRadius=int(OUTER_R_MAX_RATIO * w))
    if big is not None:
        b = np.asarray(big[0], dtype=np.float64)
        bx, by, br = b[np.argmax(b[:, 2])]
        return (float(bx), float(by), float(br)), "boundary_circle"

    _, mask = cv2.threshold(work, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (odd(0.02 * w), odd(0.02 * w)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts = find_contours(mask, cv2.RETR_EXTERNAL)
    if cnts:
        big_c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(big_c) > 0.02 * work.size:
            m = cv2.moments(big_c)
            if abs(m["m00"]) > 1e-6:
                (_, _), rr = cv2.minEnclosingCircle(big_c)
                return (m["m10"] / m["m00"], m["m01"] / m["m00"], float(rr)), "mask_centroid"
    return None, "none"


# ------------------------------------------------------------------ 孔心精定位
def refine_hole(gray: np.ndarray, cx0: float, cy0: float, r0: float,
                cos_t: np.ndarray, sin_t: np.ndarray) -> Optional[Tuple[float, float, float, float]]:
    """沿 360° 射线找孔壁"灰度 50% 跨越点"再做圆拟合。
    返回 (cx, cy, r, 孔内/台面灰度差)。灰度差过小 -> 认为不是孔, 返回 None。
    该方法与孔内是亮(通孔透光)还是暗(暗场)无关, 自动判极性。
    """
    gray_f = gray.astype(np.float32)
    radii = np.arange(REFINE_SCAN_BAND[0] * r0, REFINE_SCAN_BAND[1] * r0,
                      max(0.2, REFINE_RADIUS_STEP_PX), dtype=np.float32)
    if len(radii) < 8:
        return None
    xs = (cx0 + radii[:, None] * cos_t[None, :]).astype(np.float32)
    ys = (cy0 + radii[:, None] * sin_t[None, :]).astype(np.float32)
    h, w = gray.shape[:2]
    inside = (xs > 1.0) & (ys > 1.0) & (xs < w - 2.0) & (ys < h - 2.0)
    vals = cv2.remap(gray_f, np.clip(xs, 0, w - 1), np.clip(ys, 0, h - 1),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    vals[~inside] = np.nan

    med = np.full(len(radii), np.nan, np.float32)
    good = np.count_nonzero(~np.isnan(vals), axis=1) > 0
    if not good.any():
        return None
    med[good] = np.nanmedian(vals[good], axis=1)
    in_sel = radii < REFINE_INNER_BAND * r0
    la_sel = radii > REFINE_LAND_BAND * r0
    if not in_sel.any() or not la_sel.any():
        return None
    inner = float(np.nanmedian(med[in_sel]))
    land = float(np.nanmedian(med[la_sel]))
    if not np.isfinite(inner) or not np.isfinite(land):
        return None
    contrast = abs(inner - land)
    if contrast < REFINE_MIN_CONTRAST:  # 无明显孔 -> 丢弃(抗油污误检)
        return None

    thr = 0.5 * (inner + land)
    sign = 1.0 if inner > land else -1.0
    start = np.searchsorted(radii, REFINE_EDGE_START * r0)
    pts: List[Tuple[float, float]] = []
    for j in range(vals.shape[1]):
        col = sign * (vals[:, j] - thr)
        col = np.nan_to_num(col, nan=-1.0)
        cross = np.where((col[start:-1] > 0.0) & (col[start + 1:] <= 0.0))[0]
        if cross.size:
            rr = float(radii[start + cross[0]])
            pts.append((cx0 + rr * float(cos_t[j]), cy0 + rr * float(sin_t[j])))
    if len(pts) < REFINE_MIN_EDGE_PTS:
        return None
    fit = fit_circle_robust(np.asarray(pts, np.float64), REFINE_ITERS,
                            REFINE_INLIER_RATIO, 3.0, max(12, REFINE_MIN_EDGE_PTS // 2))
    if fit is None:
        return None
    cx, cy, r, _ = fit
    if not (REFINE_SCAN_BAND[0] * r0 < r < REFINE_SCAN_BAND[1] * r0):
        return None
    return cx, cy, r, contrast


def local_enhance(sub: np.ndarray) -> np.ndarray:
    """小 ROI 局部增强: 局部 CLAHE + 高斯。翻边外圈在暗区/亮区对比度差异很大,
    局部均衡后同一套阈值才能通用(实测样图左侧暗、右侧过曝, 全局均衡不够)。"""
    out = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_GRID).apply(sub)
    if GAUSS_BLUR_K >= 3:
        out = cv2.GaussianBlur(out, (odd(GAUSS_BLUR_K), odd(GAUSS_BLUR_K)), 0)
    return out


def local_smooth(sub: np.ndarray) -> np.ndarray:
    """小 ROI 只做平滑。压痕检测实测在原始灰度上最稳: 拐角窗口本来就小,
    再做 CLAHE 会把台面机加工纹理放大成假圆(实测有效命中率反而下降)。"""
    if GAUSS_BLUR_K >= 3:
        return cv2.GaussianBlur(sub, (odd(GAUSS_BLUR_K), odd(GAUSS_BLUR_K)), 0)
    return sub


# ------------------------------------------------------------------ 特征A: 翻边外圈
def feature_a_ring_contours(gray: np.ndarray, hx: float, hy: float, r: float) -> Tuple[int, int, List[float]]:
    """孔 ROI 内轮廓计数 -> 同心环数。

    抗干扰三重过滤:
      1) 圆形掩膜 (RING_MASK_RATIO) 屏蔽相邻孔与拐角压痕;
      2) 只保留"同心"轮廓: 平均半径在 RING_BAND 内、径向标准差比 < RING_MAX_RADIAL_STD、
         角度覆盖率 > RING_MIN_ANGLE_COVER (毛刺/铁屑/划痕碎轮廓全被剔除);
      3) 多阈值扫描后按半径聚类, 只有被 >=RING_CLUSTER_MIN_HITS 个阈值重复命中的簇才算真环
         (单圈冲裁轮廓的内外两条边距离很近, 会被聚成 1 圈, 不会误判成翻边)。
    返回 (原始轮廓数, 环数, 各环半径比)。
    """
    half = int(round(RING_ROI_RATIO * r))
    if half < 6:
        return 0, 0, []
    sub, _ = crop_pad(gray, hx, hy, half)
    sub = local_enhance(sub)
    mask = np.zeros(sub.shape[:2], np.uint8)
    cv2.circle(mask, (half, half), int(round(RING_MASK_RATIO * r)), 255, -1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    lo, hi = RING_BAND[0] * r, RING_BAND[1] * r
    radii_hits: List[float] = []
    raw = 0
    for q in np.percentile(sub, RING_THRESH_PCTS):
        for flag in (cv2.THRESH_BINARY_INV, cv2.THRESH_BINARY):
            _, bw = cv2.threshold(sub, float(q), 255, flag)
            bw = cv2.morphologyEx(cv2.bitwise_and(bw, mask), cv2.MORPH_OPEN, kernel)
            for cnt in find_contours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE):
                if len(cnt) < RING_MIN_CONTOUR_PTS:
                    continue
                raw += 1
                pts = cnt.reshape(-1, 2).astype(np.float32) - float(half)
                dist = np.hypot(pts[:, 0], pts[:, 1])
                mean_r = float(dist.mean())
                if mean_r < 1e-6 or not (lo <= mean_r <= hi):
                    continue
                if float(dist.std()) / mean_r > RING_MAX_RADIAL_STD:
                    continue
                if angular_coverage(pts[:, 0], pts[:, 1]) < RING_MIN_ANGLE_COVER:
                    continue
                radii_hits.append(mean_r)

    radii_hits.sort()
    clusters: List[List[float]] = []
    for v in radii_hits:
        if not clusters or v - clusters[-1][-1] > RING_CLUSTER_GAP * r:
            clusters.append([v])
        else:
            clusters[-1].append(v)
    rings = [c for c in clusters if len(c) >= RING_CLUSTER_MIN_HITS]
    return raw, len(rings), [round(float(np.mean(c)) / r, 3) for c in rings]


def feature_a_flange_hough(gray: np.ndarray, hx: float, hy: float, r: float) -> Optional[float]:
    """同心 Hough: 在 [1.12,1.36]r 内找与孔同心的翻边圆, 作为轮廓计数的 OR 兜底。
    光照不均导致翻边外圈局部对比度低、轮廓断裂时, 形状级 Hough 仍能命中。"""
    half = int(round((FLANGE_HOUGH_BAND[1] + 0.12) * r))
    if half < 6:
        return None
    sub, _ = crop_pad(gray, hx, hy, half)
    sub = local_enhance(sub)
    circles = cv2.HoughCircles(sub, cv2.HOUGH_GRADIENT, dp=1.0,
                               minDist=max(6, int(0.20 * r)),
                               param1=FLANGE_HOUGH_P1, param2=FLANGE_HOUGH_P2,
                               minRadius=max(3, int(FLANGE_HOUGH_BAND[0] * r)),
                               maxRadius=max(5, int(FLANGE_HOUGH_BAND[1] * r)))
    if circles is None:
        return None
    best = None
    for (bx, by, br) in np.asarray(circles[0], dtype=np.float64):
        if np.hypot(bx - half, by - half) <= FLANGE_CENTER_GATE * r:
            if best is None or br > best:
                best = float(br)
    return None if best is None else best / r


# ------------------------------------------------------------------ 特征B: 拐角小圆压痕
def _best_mark_circularity(win: np.ndarray, bx: float, by: float, br: float) -> float:
    """对 Hough 命中的候选圆做多阈值分割, 取圆度最高的连通域。

    压痕与翻边外圈在图像上是紧邻的, 单一阈值会把两者粘成一团导致圆度骤降;
    这里用"圆形掩膜裁剪 + 多阈值(百分位)扫描"取最优解, 是本算法抗粘连的关键。
    """
    mask = np.zeros(win.shape[:2], np.uint8)
    cv2.circle(mask, (int(round(bx)), int(round(by))), max(2, int(round(MARK_MASK_RATIO * br))), 255, -1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    min_area = MARK_MIN_AREA_RATIO * float(np.pi) * br * br
    best = 0.0
    for q in np.percentile(win, MARK_THRESH_PCTS):
        for flag in (cv2.THRESH_BINARY_INV, cv2.THRESH_BINARY):
            _, bw = cv2.threshold(win, float(q), 255, flag)
            bw = cv2.bitwise_and(bw, mask)
            bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel)
            bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)
            for cnt in find_contours(bw, cv2.RETR_EXTERNAL):
                if cv2.contourArea(cnt) < min_area:
                    continue
                if cv2.pointPolygonTest(cnt, (float(bx), float(by)), False) < 0:
                    continue                                  # 必须包住 Hough 圆心
                best = max(best, circularity(cnt))
    return best



def _dedup_mark_candidates(candidates: np.ndarray, half: int, r: float) -> List[np.ndarray]:
    """Keep distinct Hough candidates and cap expensive circularity checks."""
    if candidates is None or len(candidates) == 0:
        return []
    # 新增：只保留形状为 (3,) 的行，过滤异常损坏的行
    valid_candidates = []
    for c in candidates:
        arr = np.asarray(c, dtype=np.float64)
        if arr.shape == (3,):
            valid_candidates.append(arr)
    if not valid_candidates:
        return []

    ordered = sorted(
        valid_candidates,
        key=lambda c: (
            np.hypot(c[0] - half, c[1] - half) / max(r, 1e-6),
            abs(c[2] / max(r, 1e-6) - 0.38),
        ),
    )
    kept: List[np.ndarray] = []
    min_dist = MARK_DEDUP_DIST_RATIO * r
    for c in ordered:
        ratio = c[2] / max(r, 1e-6)
        if not (0.20 <= ratio <= 0.70):
            continue
        if any(np.hypot(c[0] - old[0], c[1] - old[1]) < min_dist for old in kept):
            continue
        kept.append(c)
        if len(kept) >= MARK_MAX_CANDIDATES:
            break
    return kept



def feature_b_corner_marks(gray: np.ndarray, clahe_only: np.ndarray, hx: float, hy: float, r: float,
                           part_cx: float, part_cy: float,
                           corner_spec: Sequence[Tuple[float, float]] = CORNER_SPEC
                           ) -> Tuple[int, int, List[dict]]:
    global PRINT_DEBUG
    IS_DEBUG_DETAIL = PRINT_DEBUG

    ux, uy = hx - part_cx, hy - part_cy
    norm = float(np.hypot(ux, uy))
    if norm < 1e-6:
        return 0, 0, []
    ux, uy = ux / norm, uy / norm

    half = int(round(CORNER_WIN_RATIO * r))
    if half < 5:
        return 0, 0, []
    r_lo = max(3, int(round(MARK_R_RATIO_RANGE[0] * r)))
    r_hi = max(r_lo + 2, int(round(MARK_R_RATIO_RANGE[1] * r)))
    h, w = gray.shape[:2]
    valid = 0
    in_frame = 0
    details: List[dict] = []

    for ang, dist_ratio in corner_spec:
        vx, vy = rotate_unit(ux, uy, ang)
        px, py = hx + dist_ratio * r * vx, hy + dist_ratio * r * vy
        rec = {
            "angle": ang,
            "cx": px,
            "cy": py,
            "ok": False,
            "in_frame": False
        }
        if IS_DEBUG_DETAIL:
            rec["circ"] = 0.0
            rec["circ_raw"] = 0.0
            rec["circ_enhanced"] = 0.0
            rec["center_offset"] = 0.0
            rec["radius_ratio"] = 0.0
            rec["shape_ok"] = False
            rec["dynamic_roi"] = False
            rec["r"] = 0.0
            rec["mark_cx"] = 0.0
            rec["mark_cy"] = 0.0
            rec["mark_r"] = 0.0

        if not (half <= px < w - half and half <= py < h - half):
            details.append(rec)
            continue
        rec["in_frame"] = True
        in_frame += 1
        win, _ = crop_pad(gray, px, py, half)
        win = local_smooth(win)
        # 使用全局预计算的 clahe_only，不再每个拐角做 local_enhance
        win_enhanced, _ = crop_pad(clahe_only, px, py, half)

        circles = cv2.HoughCircles(win, cv2.HOUGH_GRADIENT, dp=1.0, minDist=max(6, r_lo),
                                   param1=MARK_HOUGH_P1, param2=MARK_HOUGH_P2,
                                   minRadius=r_lo, maxRadius=r_hi)
        if circles is None:
            details.append(rec)
            continue
        all_cand = np.asarray(circles[0], dtype=np.float64)
        # 增加校验：必须是 N行3列，否则直接丢弃
        if all_cand.ndim != 2 or all_cand.shape[1] != 3:
            details.append(rec)
            continue
        cand = [c for c in all_cand
                if np.hypot(c[0] - half, c[1] - half) <= MARK_CENTER_GATE * r]

        dynamic_roi = False
        if not cand:
            dynamic_gate = 0.32
            cand = [c for c in all_cand
                    if np.hypot(c[0] - half, c[1] - half) <= dynamic_gate * r]
            dynamic_roi = bool(cand)
        if not cand:
            details.append(rec)
            continue

        best = None

        for bx, by, br in _dedup_mark_candidates(np.asarray(cand, dtype=np.float64),half,r,):
            circ_raw = _best_mark_circularity(win,bx,by,br,)
            circ_enhanced = _best_mark_circularity(win_enhanced,bx,by,br,)
            circ = max(circ_raw, circ_enhanced)
            center_offset = float(np.hypot(bx - half,by - half,)/max(r, 1e-6))
            radius_ratio = float(br / max(r, 1e-6))
            position_ok = center_offset <= (0.32 if dynamic_roi else MARK_CENTER_GATE)
            shape_ok = (0.28 <= radius_ratio <= 0.55 and circ_enhanced > 0.45 and circ_raw > 0.10)
            if FEATURE_B_DISABLE_SOFT_ACCEPT:
                accepted = bool(circ_raw > MARK_CIRCULARITY_MIN)
            else:
                accepted = bool(circ_raw > MARK_CIRCULARITY_MIN or (position_ok and shape_ok))
            score = (accepted,circ_raw + circ_enhanced,-center_offset,-abs(radius_ratio - 0.38))
            if best is None or score > best[0]:
                best = (score,bx,by,br,circ_raw,circ_enhanced,circ,center_offset,radius_ratio,shape_ok,accepted)

        if best is not None:
            (_,bx,by,br,circ_raw,circ_enhanced,circ,center_offset,radius_ratio,shape_ok,accepted,) = best

            # !!!!!【业务核心】ok 标记与计数，无论是否调试，
            # 必须执行，不能包进 if IS_DEBUG_DETAIL!!!!
            if accepted:
                rec["ok"] = True
                valid += 1

        # 仅调试模式填充圆度、坐标等调试字段
        if IS_DEBUG_DETAIL:
            rec["circ"] = round(circ, 3)
            rec["circ_raw"] = round(circ_raw, 3)
            rec["circ_enhanced"] = round(circ_enhanced, 3)
            rec["center_offset"] = round(center_offset, 3)
            rec["radius_ratio"] = round(radius_ratio, 3)
            rec["shape_ok"] = bool(shape_ok)
            rec["dynamic_roi"] = bool(dynamic_roi)
            rec["r"] = round(radius_ratio, 3)
            rec["mark_cx"] = px + (bx - half)
            rec["mark_cy"] = py + (by - half)
            rec["mark_r"] = float(br)

        details.append(rec)
    return valid, in_frame, details


def count_corners_in_frame(shape: Tuple[int, int], hx: float, hy: float, r: float,
                           part_cx: float, part_cy: float) -> int:
    """预判 4 个拐角 ROI 有几个完整落在视场内 —— 用于挑选参与判定的孔,
    避免选到贴边、拐角被切掉的孔造成误 NG(现场半幅视野时很常见)。"""
    ux, uy = hx - part_cx, hy - part_cy
    norm = float(np.hypot(ux, uy))
    if norm < 1e-6:
        return 0
    ux, uy = ux / norm, uy / norm
    half = int(round(CORNER_WIN_RATIO * r))
    h, w = shape[:2]
    n = 0
    for ang, dist_ratio in CORNER_SPEC:
        vx, vy = rotate_unit(ux, uy, ang)
        px, py = hx + dist_ratio * r * vx, hy + dist_ratio * r * vy
        if half <= px < w - half and half <= py < h - half:
            n += 1
    return n


# ------------------------------------------------------------------ 主检测流程
def inspect(bgr: np.ndarray, name: str = "", timing: bool = False) -> Tuple[InspectResult, np.ndarray]:
    """单帧检测；可选记录算法内部阶段耗时，保持旧调用签名兼容。"""
    t0 = time.perf_counter()
    res = InspectResult(name=name)
    stage_start = t0

    def stage_done(key: str) -> None:
        nonlocal stage_start
        if timing:
            res.timings_ms[key] = (time.perf_counter() - stage_start) * 1000.0
        stage_start = time.perf_counter()

    def finish() -> Tuple[InspectResult, np.ndarray]:
        res.elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if timing:
            res.timings_ms["inspect_total"] = res.elapsed_ms
        return res, bgr

    bgr, gray, work, clahe_only, _ = preprocess(bgr)
    stage_done("preprocess")
    cand = detect_hole_candidates(work)
    stage_done("hole_candidates")
    part, method = locate_part(work, cand)
    stage_done("part_locate")
    res.locate_method = method
    res.timing_details["candidate_holes"] = len(cand)
    if part is None:
        res.verdict = NG_PART_NOT_FOUND
        res.reason = "定位失败: 图中找不到垫片"
        return finish()
    # 工件在位守门：mask_centroid 是"节圆拟不出、外圆也找不到"的最后兜底，正常在位的完整
    # 工件不会走到这里(实测全部真实 OK 件走 pitch_fit)。落到它 = 残件/底板占画面，直接判不在位。
    if REJECT_MASK_CENTROID and method == "mask_centroid":
        res.verdict = NG_PART_NOT_FOUND
        res.reason = "工件不在位: 仅靠 mask_centroid 兜底定位(节圆拟不出/外圆缺失)，疑似残件或底板"
        res.part_cx, res.part_cy, res.pitch_r = part
        return finish()
    res.part_cx, res.part_cy, res.pitch_r = part
    if len(cand) < MIN_HOLE_COUNT:
        res.verdict = NG_HOLE_NOT_FOUND
        res.reason = "圆孔候选不足: %d < %d" % (len(cand), MIN_HOLE_COUNT)
        return finish()

    ang = np.radians(np.arange(0.0, 360.0, REFINE_ANGLE_STEP_DEG))
    cos_t, sin_t = np.cos(ang).astype(np.float32), np.sin(ang).astype(np.float32)
    refined: List[Tuple[float, float, float, float]] = []
    for (x0, y0, r0) in cand:
        got = refine_hole(work, float(x0), float(y0), float(r0), cos_t, sin_t)
        if got is not None:
            refined.append(got)
    stage_done("hole_refine")
    res.timing_details["refined_holes"] = len(refined)
    if len(refined) < MIN_HOLE_COUNT:
        res.verdict = NG_HOLE_NOT_FOUND
        res.reason = "精定位后有效圆孔不足: %d < %d" % (len(refined), MIN_HOLE_COUNT)
        return finish()

    selection_start = time.perf_counter()
    r_med = float(np.median([q[2] for q in refined]))
    res.hole_r = r_med
    holes: List[HoleResult] = []
    h_img, w_img = gray.shape[:2]
    for i, (hx, hy, hr, contrast) in enumerate(refined):
        if abs(hr - r_med) / max(r_med, 1e-6) > HOLE_R_DEV_MAX:
            continue
        r_use = r_med if USE_GLOBAL_HOLE_RADIUS else hr
        margin = RING_ROI_RATIO * r_use
        in_frame = (margin <= hx < w_img - margin) and (margin <= hy < h_img - margin)
        holes.append(HoleResult(index=i, cx=hx, cy=hy, r=r_use,
                                contrast=contrast, in_frame=in_frame))
    if len(holes) < MIN_HOLE_COUNT:
        res.verdict = NG_HOLE_NOT_FOUND
        res.reason = "孔径一致性筛选后不足: %d < %d" % (len(holes), MIN_HOLE_COUNT)
        if timing:
            res.timings_ms["hole_select"] = (time.perf_counter() - selection_start) * 1000.0
        return finish()

    for hole in holes:
        hole.corners_in_frame = count_corners_in_frame(
            gray.shape, hole.cx, hole.cy, hole.r, res.part_cx, res.part_cy)
    holes.sort(key=lambda q: (not q.in_frame, -q.corners_in_frame, -q.contrast))
    n_check = len(holes) if HOLE_CHECK_COUNT <= 0 else min(HOLE_CHECK_COUNT, len(holes))
    if timing:
        res.timings_ms["hole_select"] = (time.perf_counter() - selection_start) * 1000.0
    res.holes = holes
    res.checked = [holes[i].index for i in range(n_check)]
    res.timing_details["checked_holes"] = n_check

    feature_a_total = feature_b_total = 0.0
    for hole in holes[:n_check]:
        hole_t0 = time.perf_counter()
        raw_cnt, ring_cnt, ring_radii = feature_a_ring_contours(gray, hole.cx, hole.cy, hole.r)
        hole.raw_contour_count = raw_cnt
        hole.ring_count = ring_cnt
        hole.ring_radii = ring_radii
        contour_ms = (time.perf_counter() - hole_t0) * 1000.0
        a_contour = ring_cnt >= RING_COUNT_MIN
        a_hough = False
        hough_ms = 0.0
        if FEATURE_A_MODE != "contour":
            hough_t0 = time.perf_counter()
            hole.flange_hough_r = feature_a_flange_hough(gray, hole.cx, hole.cy, hole.r)
            hough_ms = (time.perf_counter() - hough_t0) * 1000.0
            a_hough = hole.flange_hough_r is not None
        if FEATURE_A_MODE == "contour":
            hole.feature_a = a_contour
        elif FEATURE_A_MODE == "hough":
            hole.feature_a = a_hough
        elif FEATURE_A_MODE == "contour_and_hough":
            hole.feature_a = a_contour and a_hough
        else:
            hole.feature_a = a_contour or a_hough
        feature_a_ms = contour_ms + hough_ms
        feature_a_total += feature_a_ms

        feature_b_t0 = time.perf_counter()
        marks, corners, details = feature_b_corner_marks(
            gray, clahe_only, hole.cx, hole.cy, hole.r, res.part_cx, res.part_cy)
        feature_b_ms = (time.perf_counter() - feature_b_t0) * 1000.0
        feature_b_total += feature_b_ms
        hole.valid_marks = marks
        hole.corners_in_frame = corners
        hole.corner_hits = details
        hole.feature_b = marks >= MIN_VALID_MARKS
        hole.passed = (hole.feature_a and hole.feature_b) if HOLE_LOGIC == "AND" \
            else (hole.feature_a or hole.feature_b)
        if timing:
            res.timing_details["hole_%d" % hole.index] = {
                "feature_a_ms": feature_a_ms,
                "feature_a_contour_ms": contour_ms,
                "feature_a_hough_ms": hough_ms,
                "feature_b_ms": feature_b_ms,
            }
    if timing:
        res.timings_ms["feature_a"] = feature_a_total
        res.timings_ms["feature_b"] = feature_b_total
    stage_start = time.perf_counter()
    checked_holes = holes[:n_check]
    if PART_LOGIC == "OR":
        part_ok = sum(bool(q.passed) for q in checked_holes) >= PART_MIN_PASS_HOLES
    else:
        part_ok = all(q.passed for q in checked_holes)
    if part_ok:
        res.verdict, res.reason = OK_PASS, "存在翻边外圈 + 冲压小圆压痕(正面)"
    else:
        res.verdict = NG_NO_FEATURE
        res.reason = "所有受检孔均无有效翻边/压痕特征(反面或漏冲)"
    stage_done("final_decision")
    return finish()


# ------------------------------------------------------------------ 调试输出
def print_result(res: InspectResult) -> None:
    """仅输出完整调试明细；只有--print-detail才调用"""
    print("  工件定位: %-22s 中心=(%.1f, %.1f)  节圆R=%.1f  孔径r=%.1f  有效孔=%d  受检孔=%s"
          % (res.locate_method, res.part_cx, res.part_cy, res.pitch_r,
             res.hole_r, len(res.holes), res.checked))
    print("  %-4s %-19s %-7s %-6s %-24s %-9s %-6s %-7s %-6s %s"
          % ("孔", "孔心(x,y)", "原始轮廓", "环数", "环半径比", "翻边Hough",
             "特征A", "有效压痕", "特征B", "单孔"))
    for hole in res.holes[:len(res.checked)]:
        flange = "-" if hole.flange_hough_r is None else "%.2f" % hole.flange_hough_r
        print("  #%-3d (%7.1f,%7.1f) %-7d %-6d %-24s %-9s %-6s %d/%-5d %-6s %s"
              % (hole.index, hole.cx, hole.cy, hole.raw_contour_count, hole.ring_count,
                 str(hole.ring_radii), flange, "PASS" if hole.feature_a else "FAIL",
                 hole.valid_marks, hole.corners_in_frame,
                 "PASS" if hole.feature_b else "FAIL", "OK" if hole.passed else "NG"))
        detail = "  ".join("%+6.1f°:%s(circ=%.2f,r=%.2f)"
                           % (d["angle"], "Y" if d["ok"] else ("-" if d["in_frame"] else "x"),
                              d["circ"], d["r"]) for d in hole.corner_hits)
        print("       拐角明细 %s" % detail)


def draw_overlay(bgr: np.ndarray, res: InspectResult) -> np.ndarray:
    """叠加检测结果, 现场看图排查用。"""
    vis = bgr.copy() if bgr.ndim == 3 else cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)
    green, red, yellow, blue, cyan = (0, 220, 0), (0, 0, 255), (0, 220, 220), (255, 120, 0), (255, 255, 0)
    if res.pitch_r > 0:
        cv2.circle(vis, (int(res.part_cx), int(res.part_cy)), int(res.pitch_r), blue, 1, cv2.LINE_AA)
        cv2.drawMarker(vis, (int(res.part_cx), int(res.part_cy)), blue, cv2.MARKER_CROSS, 26, 2)
    checked = set(res.checked)
    for hole in res.holes:
        col = green if hole.passed else (red if hole.index in checked else yellow)
        cv2.circle(vis, (int(hole.cx), int(hole.cy)), int(hole.r), col, 2, cv2.LINE_AA)
        if hole.index not in checked:
            continue
        cv2.circle(vis, (int(hole.cx), int(hole.cy)), int(RING_ROI_RATIO * hole.r), cyan, 1, cv2.LINE_AA)
        for rr in hole.ring_radii:  # 命中的同心环
            cv2.circle(vis, (int(hole.cx), int(hole.cy)), int(rr * hole.r), (255, 0, 255), 1, cv2.LINE_AA)
        if hole.flange_hough_r:
            cv2.circle(vis, (int(hole.cx), int(hole.cy)),
                       int(hole.flange_hough_r * hole.r), (200, 200, 0), 1, cv2.LINE_AA)
        half = int(round(CORNER_WIN_RATIO * hole.r))
        for d in hole.corner_hits:
            c = green if d["ok"] else (red if d["in_frame"] else (128, 128, 128))
            p0 = (int(d["cx"]) - half, int(d["cy"]) - half)
            p1 = (int(d["cx"]) + half, int(d["cy"]) + half)
            cv2.rectangle(vis, p0, p1, c, 1)
            if d.get("mark_r"):
                cv2.circle(vis, (int(d["mark_cx"]), int(d["mark_cy"])), int(d["mark_r"]), c, 2, cv2.LINE_AA)
        cv2.putText(vis, "#%d ring=%d mark=%d" % (hole.index, hole.ring_count, hole.valid_marks),
                    (int(hole.cx) - half, int(hole.cy) - int(1.55 * hole.r)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    tag = "OK" if res.is_ok else "NG"
    cv2.rectangle(vis, (0, 0), (300, 44), (30, 30, 30), -1)
    cv2.putText(vis, "%s  %s" % (tag, res.verdict), (8, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, green if res.is_ok else red, 2, cv2.LINE_AA)
    return vis


def unique_image_name(base: str, suffix: str, ext: Optional[str] = None) -> str:
    """毫秒时间 + 帧名避免高速触发时同秒覆盖。ext=None 时用全局 SAVE_IMAGE_EXT。"""
    base = os.path.splitext(os.path.basename(base))[0] or "frame"
    base = "".join(ch if ch.isalnum() or ch in "-_#" else "_" for ch in base)[:80]
    now = time.time()
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
    return "%s_%03d_%s_%s%s" % (stamp, int(now * 1000) % 1000, base, suffix,
                                ext if ext is not None else SAVE_IMAGE_EXT)


def save_raw_image(bgr: np.ndarray, name: str) -> Optional[str]:
    """完整帧无叠加留证；失败属于系统故障，不冒充产品 NG。恒存无损 PNG(RAW_SAVE_EXT)。"""
    if not SAVE_RAW_IMAGE:
        return None
    path = os.path.join(RAW_SAVE_DIR, unique_image_name(name, "RAW", RAW_SAVE_EXT))
    return path if imwrite_unicode(path, bgr) else None


def save_result_image(bgr: np.ndarray, res: InspectResult, overlay: bool) -> Optional[str]:
    """NG(可选 OK) 样本本地保存。文件名带毫秒 + 帧 ID + NG 代码。"""
    if res.is_ok and not SAVE_OK_IMAGE:
        return None
    if (not res.is_ok) and not SAVE_NG_IMAGE:
        return None
    folder = OK_SAVE_DIR if res.is_ok else NG_SAVE_DIR
    path = os.path.join(folder, unique_image_name(res.name, res.verdict))
    img = draw_overlay(bgr, res) if (overlay and SAVE_OVERLAY) else bgr
    return path if imwrite_unicode(path, img) else None


class RawFrameSaver:
    """RAW 专用后台存图线程：由采集线程在“帧一组装好”时 submit，与传感器存图一一对齐。

    RAW 是最前端的追溯证据，硬要求张数与传感器一致，所以：
      - 落盘点提前到帧组装(采集线程)，协议丢帧/队列过载/判定超时都不会吃掉 RAW；
      - 独立大队列(RAW_SAVE_QUEUE_SIZE)，与可牺牲的结果图分开，尽量不丢；
      - 传一半的残帧也存(补零到整帧 + 文件名标 PARTIAL_<字节数>b)，保证张数守恒、可追溯。
    沿用 Wtx10001Source 的线程约定：daemon + Event 停机 + join 超时 + 跳过当前线程。
    """

    def __init__(self) -> None:
        self._q: "queue.Queue[tuple]" = queue.Queue(maxsize=max(1, RAW_SAVE_QUEUE_SIZE))
        self._stop = threading.Event()
        self.raw_saved = 0       # 完整帧 RAW 落盘数
        self.partial_saved = 0   # 残帧(PARTIAL)落盘数
        self.raw_dropped = 0     # 队列满被丢的 RAW 数(正常应为 0)
        self.raw_fail = 0        # 写盘失败数
        self.max_queue_depth = 0
        self._thread = threading.Thread(
            target=self._run, name="raw-saver", daemon=True)
        self._thread.start()

    def submit(self, pixels: bytes, name: str, partial: bool) -> None:
        """非阻塞提交一帧像素字节。pixels 是不可变 bytes，跨线程安全。"""
        if not SAVE_RAW_IMAGE or not pixels:
            return
        task = (bytes(pixels), name, partial)
        try:
            self._q.put_nowait(task)
        except queue.Full:
            # 大队列基本到不了这里；真满了也不能阻塞采集线程(会把 _recv_frames 卡死、
            # 反过来造成真帧积压)。只能丢最旧 + 醒目告警，供事后对账。
            self.raw_dropped += 1
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            RUNTIME_LOGGER.critical(
                "[RAW-DROP] RAW 队列满(%d)，丢最旧；raw_dropped=%d；"
                "RAW 与传感器张数将不一致，请查磁盘/降触发频率",
                self._q.maxsize, self.raw_dropped)
            try:
                self._q.put_nowait(task)
            except queue.Full:
                pass
        depth = self._q.qsize()
        self.max_queue_depth = max(self.max_queue_depth, depth)

    @staticmethod
    def _pad_to_frame(pixels: bytes) -> Optional[np.ndarray]:
        """残帧: 已收字节补零到整帧再解成 BGR。缺失部分为黑，一眼看出传输在哪断。"""
        expected = CAM_IMG_W * CAM_IMG_H
        buf = pixels[:expected]
        if len(buf) < expected:
            buf = buf + b"\x00" * (expected - len(buf))
        gray = np.frombuffer(buf, dtype=np.uint8).reshape(CAM_IMG_H, CAM_IMG_W)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def _run(self) -> None:
        while not self._stop.is_set() or not self._q.empty():
            try:
                pixels, name, partial = self._q.get(timeout=0.3)
            except queue.Empty:
                continue
            try:
                if partial:
                    bgr = self._pad_to_frame(pixels)
                    save_name = "%s_PARTIAL_%db" % (name, len(pixels))
                else:
                    bgr = Wtx10001Source._to_bgr(pixels)
                    save_name = name
                if bgr is None:
                    self.raw_fail += 1
                    RUNTIME_LOGGER.error("[RAW-ERROR] image=%s 像素无法解码", name)
                    continue
                path = save_raw_image(bgr, save_name)
                if path:
                    if partial:
                        self.partial_saved += 1
                    else:
                        self.raw_saved += 1
                    RUNTIME_LOGGER.info("[RAW-SAVED] image=%s partial=%s path=%s",
                                        save_name, partial, path)
                else:
                    self.raw_fail += 1
                    RUNTIME_LOGGER.error("[RAW-ERROR] image=%s 原图保存失败", save_name)
            except Exception:  # noqa: BLE001 - 后台写盘异常不能带垮线程
                self.raw_fail += 1
                RUNTIME_LOGGER.exception("[RAW-ERROR] 后台 RAW 存图异常 image=%s", name)

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()  # 停机前把已入队 RAW 尽量写完(_run 会排空 queue)
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=SAVE_JOIN_TIMEOUT_S)
        if not self._q.empty():
            RUNTIME_LOGGER.warning(
                "[RAW-DROP] 停机超时仍有 %d 张 RAW 未写盘", self._q.qsize())


class AsyncImageSaver:
    """结果图后台存图线程：判定/吹气后主循环只 submit，磁盘卡顿不占检测耗时也不推迟喷嘴。

    结果图(overlay)是可牺牲的追溯图，队列满时丢最旧并计数告警（判定/吹气早已完成）。
    RAW 不走这里——RAW 归采集线程的 RawFrameSaver，硬要求与传感器张数一致。
    沿用 Wtx10001Source 的线程约定：daemon + Event 停机 + join 超时。
    """

    def __init__(self) -> None:
        self._q: "queue.Queue[tuple]" = queue.Queue(maxsize=max(1, SAVE_QUEUE_SIZE))
        self._stop = threading.Event()
        self.dropped_saves = 0  # 结果图队列满被丢弃的任务数
        self.result_fail = 0    # 写盘失败(结果图)
        self.max_queue_depth = 0
        self._thread = threading.Thread(
            target=self._run, name="image-saver", daemon=True)
        self._thread.start()

    def submit_result(self, bgr: np.ndarray, res: "InspectResult", overlay: bool) -> None:
        # RAW 不再走这里(归采集线程的 RawFrameSaver)，本类只管可牺牲的结果图。
        if bgr is None:
            return
        self._submit(("result", bgr, None, res, overlay))

    def _submit(self, task: tuple) -> None:
        try:
            self._q.put_nowait(task)
        except queue.Full:
            self.dropped_saves += 1
            try:
                self._q.get_nowait()  # 丢最旧
            except queue.Empty:
                pass
            RUNTIME_LOGGER.warning(
                "[SAVE-DROP] 存图队列满(%d)，丢最旧留档图；dropped_saves=%d；"
                "判定/吹气不受影响，仅个别追溯图缺失，请查磁盘",
                self._q.maxsize, self.dropped_saves)
            try:
                self._q.put_nowait(task)
            except queue.Full:
                pass
        depth = self._q.qsize()
        self.max_queue_depth = max(self.max_queue_depth, depth)

    def _run(self) -> None:
        while not self._stop.is_set() or not self._q.empty():
            try:
                task = self._q.get(timeout=0.3)
            except queue.Empty:
                continue
            try:
                self._write(task)
            except Exception:  # noqa: BLE001 - 后台写盘异常不能带垮线程
                RUNTIME_LOGGER.exception("[SAVE-ERROR] 后台存图异常")

    def _write(self, task: tuple) -> None:
        _kind, bgr, _name, res, overlay = task
        should = (res.is_ok and SAVE_OK_IMAGE) or ((not res.is_ok) and SAVE_NG_IMAGE)
        path = save_result_image(bgr, res, overlay=overlay)
        if path:
            RUNTIME_LOGGER.info("[RESULT-SAVED] image=%s path=%s", res.name, path)
        elif should:
            self.result_fail += 1
            RUNTIME_LOGGER.error("[RESULT-ERROR] image=%s 结果图保存失败", res.name)

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()  # 停机前先把已入队留档尽量写完(_run 会排空 queue)
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=SAVE_JOIN_TIMEOUT_S)
        if not self._q.empty():
            RUNTIME_LOGGER.warning(
                "[SAVE-DROP] 停机超时仍有 %d 张留档图未写盘", self._q.qsize())


# ------------------------------------------------------------------ Modbus-TCP 占位
class ModbusReporter:
    """预留: 把判定结果写给 PLC (Modbus-TCP)。默认关闭, 不影响算法运行。

    上线时安装 pymodbus (pip install pymodbus) 并把下面注释解开即可:

        from pymodbus.client import ModbusTcpClient

        def connect(self):
            self.client = ModbusTcpClient(PLC_IP, port=PLC_PORT)
            return self.client.connect()

        def report(self, ok: bool):
            # 线圈: OK/NG 各一路, 便于 PLC 直接接分选气缸
            self.client.write_coil(PLC_COIL_OK, bool(ok),  slave=PLC_UNIT_ID)
            self.client.write_coil(PLC_COIL_NG, not ok,    slave=PLC_UNIT_ID)
            # 寄存器: 0=未检 1=OK 2=NG, 供 PLC 做流程判断/计数
            self.client.write_register(PLC_REG_RESULT, 1 if ok else 2, slave=PLC_UNIT_ID)

        def heartbeat(self, tick: int):
            self.client.write_register(PLC_REG_HEARTBEAT, tick & 0xFFFF, slave=PLC_UNIT_ID)

        def close(self):
            if self.client:
                self.client.close()

    注意现场时序: 建议 PLC 用"上升沿 + 应答清零"握手, 不要靠视觉端延时,
    否则节拍变化时会漏信号。
    """

    def __init__(self, enabled: bool = ENABLE_MODBUS) -> None:
        self.enabled = enabled
        self.client = None
        self.tick = 0
        if self.enabled:
            print("[INFO] Modbus 占位已启用, 但通信代码尚未接通 —— 请按 ModbusReporter 注释解开")

    def connect(self) -> bool:
        return False

    def report(self, ok: bool) -> None:  # noqa: ARG002
        self.tick += 1

    def close(self) -> None:
        return None


# ------------------------------------------------------------------ 换型标定工具
def calibrate(bgr: np.ndarray, name: str = "") -> None:
    """换型/换相机后用来重新标定 CORNER_SPEC 与 MARK_R_RATIO_RANGE。

    做法: 全图 Hough 找小圆 -> 按"工件中心->孔心"径向方向换算相对角度/距离比 ->
    聚类统计。把打印出来的中位数直接填进头部 CORNER_SPEC 即可。
    """
    _, gray, work, _, _ = preprocess(bgr)
    cand = detect_hole_candidates(work)
    part, method = locate_part(work, cand)
    if part is None or len(cand) < MIN_HOLE_COUNT:
        print("[CALIB] %s 定位失败, 跳过" % name)
        return
    pcx, pcy, _ = part
    ang = np.radians(np.arange(0.0, 360.0, REFINE_ANGLE_STEP_DEG))
    cos_t, sin_t = np.cos(ang).astype(np.float32), np.sin(ang).astype(np.float32)
    holes = [q for q in (refine_hole(work, float(x), float(y), float(r), cos_t, sin_t)

                         for (x, y, r) in cand) if q is not None]
    if len(holes) < MIN_HOLE_COUNT:
        print("[CALIB] %s 精定位失败, 跳过" % name)
        return
    r_med = float(np.median([q[2] for q in holes]))
    small = cv2.HoughCircles(work, cv2.HOUGH_GRADIENT, dp=1.0, minDist=max(6, int(0.35 * r_med)),
                             param1=MARK_HOUGH_P1, param2=32,
                             minRadius=max(3, int(0.10 * r_med)),
                             maxRadius=max(5, int(0.60 * r_med)))
    if small is None:
        print("[CALIB] %s 未找到候选小圆" % name)
        return
    small = np.asarray(small[0], dtype=np.float64)
    rows: List[Tuple[float, float, float]] = []
    for (hx, hy, _hr, _c) in holes:
        ux, uy = hx - pcx, hy - pcy
        nn = float(np.hypot(ux, uy))
        if nn < 1e-6:
            continue
        ux, uy = ux / nn, uy / nn
        dist = np.hypot(small[:, 0] - hx, small[:, 1] - hy)
        for (sx, sy, sr) in small[(dist > 1.15 * r_med) & (dist < 2.10 * r_med)]:
            dx, dy = sx - hx, sy - hy
            rel = float(np.degrees(np.arctan2(ux * dy - uy * dx, ux * dx + uy * dy)))
            rows.append((rel, float(np.hypot(dx, dy)) / r_med, float(sr) / r_med))
    if not rows:
        print("[CALIB] %s 环带内无小圆" % name)
        return
    arr = np.asarray(rows)
    print("[CALIB] %s  定位=%s  孔数=%d  r=%.1f  候选压痕=%d"
          % (name, method, len(holes), r_med, len(arr)))
    order = np.argsort(arr[:, 0])
    arr = arr[order]
    groups: List[List[np.ndarray]] = []
    for row in arr:
        if groups and row[0] - groups[-1][-1][0] <= 15.0:
            groups[-1].append(row)
        else:
            groups.append([row])
    spec = []
    for grp in groups:
        gg = np.asarray(grp)
        if len(gg) < 2:
            continue
        spec.append((round(float(np.median(gg[:, 0])), 1), round(float(np.median(gg[:, 1])), 2)))
        print("   簇 n=%2d  角度中位数 %+7.1f° (σ=%.1f)  距离比 %.2f (σ=%.02f)  半径比 %.2f"
              % (len(gg), np.median(gg[:, 0]), gg[:, 0].std(),
                 np.median(gg[:, 1]), gg[:, 1].std(), np.median(gg[:, 2])))
    print("   >>> 建议 CORNER_SPEC = %s" % (tuple(spec),))
    print("   >>> 建议 MARK_R_RATIO_RANGE = (%.2f, %.2f)"
          % (max(0.05, np.percentile(arr[:, 2], 5) * 0.9), np.percentile(arr[:, 2], 95) * 1.15))


# ------------------------------------------------------------------ 入口
def setup_console() -> None:
    """Windows 控制台默认 GBK, 中文调试信息会乱码 —— 强制切 UTF-8。"""
    try:
        if os.name == "nt":
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
    except Exception:  # noqa: BLE001
        pass
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:  # noqa: BLE001
            pass


def guess_label(name: str) -> Optional[bool]:
    """本地调试时从文件名猜真值(含 正/OK/front -> True; 反/NG/back -> False),
    仅用于自检统计, 猜不出返回 None, 不影响判定。"""
    base = os.path.basename(name)
    low = base.lower()
    if "正" in base or any(k in low for k in ("_ok", "ok_", "front", "zheng")):
        return True
    if "反" in base or any(k in low for k in ("_ng", "ng_", "back", "fan")):
        return False
    return None


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="推力保持架垫片 冲压翻边正/反面检测")
    ap.add_argument("--mode", choices=("local", "camera", "watch", "http"), default=SOURCE_MODE,
                    help="取图方式: local=遍历目录, camera=10001 私有协议真直连(产线推荐), "
                         "watch=监视存图目录, http=相机 HTTP 接口(本机这台无 Web 服务)")
    ap.add_argument("--print-detail", action="store_true", help="打印每孔拐角调试明细(大批量测试关闭，排查样本打开)")

    ap.add_argument("--dir", default=LOCAL_IMAGE_DIR, help="本地样本目录 / watch 的监视目录")
    ap.add_argument("--ip", default=CAMERA_IP, help="相机 IP")
    ap.add_argument("--trigger", choices=("external", "MainRunOnce", "ContinuousImageCapture"),
                    default=CAM_TRIGGER_ORDER,
                    help="camera 模式的触发方式: external=IO 外部硬触发(上线, 被动等帧), "
                         "MainRunOnce=软触发(台上调试), ContinuousImageCapture=连续预览(调光)")
    ap.add_argument("--debug", action="store_true", help="额外保存叠加调试图(OK 也存)")
    ap.add_argument("--timing", action="store_true",
                    help="输出取图、排队、算法内部、UNO/PLC和存图的详细耗时")
    ap.add_argument("--save-ok", dest="save_ok", action="store_true",
                    help="同时保存 OK 图片；可与 --no-overlay 组合保存 OK/NG 原图")
    ap.add_argument("--calib", action="store_true", help="拐角角度/尺寸标定模式(换型用)")
    ap.add_argument("--collect", metavar="DIR", default=None,
                    help="采样模式(标阈值/重标定用): 等价于 --save-dir DIR --no-overlay "
                         "--save-ext .png 且 OK 帧也存。⚠ 一轮只放一类件(正/反/边界), "
                         "跑完交给 dbg_report --dir DIR --truth front|back")
    ap.add_argument("--save-dir", dest="save_dir", metavar="DIR", default=None,
                    help="存图根目录, 在它下面自动建 OK/ 与 NG/ 两个子目录"
                         "(不给则用头部 OK_SAVE_DIR / NG_SAVE_DIR)")
    ap.add_argument("--no-overlay", dest="no_overlay", action="store_true",
                    help="存图不划线, 只存原始帧。喂 dbg_report 必须这样, 否则它会去分析图上的线条")
    ap.add_argument("--save-ext", dest="save_ext", choices=(".jpg", ".png"), default=None,
                    help="存图格式: .jpg=省空间(走 JPEG_QUALITY), .png=无损(采样用)")
    ap.add_argument("--limit", type=int, default=0, help="只处理前 N 张(本地调试用)")
    ap.add_argument("--holes", type=int, default=None,
                    help="覆盖 HOLE_CHECK_COUNT: 参与判定的孔数, 0=全部孔(排查用)")
    return ap.parse_args(argv)


def _fmt_ms(value: Optional[float]) -> str:
    return "-" if value is None else "%.2f" % value


def log_frame_timing(name: str, timing: FrameTiming, res: InspectResult) -> None:
    """--timing 的逐帧总览及算法内部明细。"""
    RUNTIME_LOGGER.info(
        "[TIMING] frame=%s image=%s raw=%s effective=%s late=%s control_late=%s "
        "acquisition=%sms queue=%sms bgr=%sms inspect=%sms uno=%sms plc=%sms "
        "raw_save=%sms result_save=%sms rx_to_decision=%sms rx_to_control=%sms "
        "rx_to_full=%sms margin=%sms queue_depth=%s",
        timing.frame_id if timing.frame_id is not None else "-", name,
        timing.raw_verdict, timing.effective_verdict, timing.late, timing.control_late,
        _fmt_ms(timing.acquisition_ms), _fmt_ms(timing.queue_wait_ms),
        _fmt_ms(timing.bgr_convert_ms), _fmt_ms(timing.inspect_ms),
        _fmt_ms(timing.uno_ms), _fmt_ms(timing.plc_ms),
        _fmt_ms(timing.raw_save_ms), _fmt_ms(timing.result_save_ms),
        _fmt_ms(timing.rx_first_to_decision_ms), _fmt_ms(timing.rx_first_to_control_ms),
        _fmt_ms(timing.rx_first_to_full_ms), _fmt_ms(timing.deadline_margin_ms),
        timing.queue_depth if timing.queue_depth is not None else "-")
    stages = " ".join("%s=%.2fms" % item for item in res.timings_ms.items())
    detail_parts = []
    for key, value in res.timing_details.items():
        if isinstance(value, dict):
            detail_parts.append("%s[%s]" % (
                key, ",".join("%s=%.2fms" % item for item in value.items())))
        else:
            detail_parts.append("%s=%s" % (key, value))
    RUNTIME_LOGGER.info("[TIMING-INSPECT] image=%s %s %s",
                        name, stages or "no-stages", " ".join(detail_parts))


def main(argv: Optional[Sequence[str]] = None) -> int:
    setup_console()
    setup_runtime_logging()
    args = parse_args(argv)
    global SAVE_OK_IMAGE, HOLE_CHECK_COUNT, CAM_TRIGGER_ORDER, CAM_MAX_FRAMES
    global SAVE_OVERLAY, SAVE_IMAGE_EXT, OK_SAVE_DIR, NG_SAVE_DIR
    if args.debug or args.save_ok:
        SAVE_OK_IMAGE = True
    if args.holes is not None:
        HOLE_CHECK_COUNT = args.holes

    # 控制每孔拐角明细打印：默认关闭，--print-detail才开启
    global PRINT_DEBUG
    PRINT_DEBUG = args.print_detail

    CAM_TRIGGER_ORDER = args.trigger
    if args.limit > 0:
        CAM_MAX_FRAMES = args.limit  # 取够就收工: IO 触发下若只在主循环里
        #   判上限, 会卡在等下一个上升沿才退出
    if args.collect:  # 采样模式: 原始帧 + 无损 + OK 也存
        SAVE_OK_IMAGE = True
        SAVE_OVERLAY = False
        SAVE_IMAGE_EXT = ".png"
    if args.no_overlay:  # 单独用也行, 与 --collect 不冲突
        SAVE_OVERLAY = False
    # --no-overlay / --save-ok 存的是"忠实原始帧"(不画线)，必须无损 PNG，理由同 RAW：
    #   JPEG 有损会在圆度卡阈值的临界帧上改变判定，存下来的图无法复现产线结论。
    #   显式 --save-ext 优先级最高，可覆盖。
    if (args.no_overlay or args.save_ok) and not args.save_ext:
        SAVE_IMAGE_EXT = ".png"
    if args.save_ext:
        SAVE_IMAGE_EXT = args.save_ext
    save_root = args.collect or args.save_dir  # 换轮次只改这一个路径, 不用动头部常量
    if save_root:
        OK_SAVE_DIR = os.path.join(save_root, "OK")
        NG_SAVE_DIR = os.path.join(save_root, "NG")
        global RAW_SAVE_DIR
        RAW_SAVE_DIR = os.path.join(save_root, "RAW")
    if args.collect or args.no_overlay or args.save_ok or args.save_ext or save_root:
        where = (os.path.join(os.path.abspath(save_root), "{OK,NG}") if save_root
                 else "%s + %s" % (OK_SAVE_DIR, NG_SAVE_DIR))
        print("[INFO] 存图 %s | 格式 %s | %s"
              % (where, SAVE_IMAGE_EXT,
                 "叠加检测结果" if SAVE_OVERLAY else "只存原始帧(可直接喂 dbg_report)"))
    if args.collect:
        print("[INFO] 采样模式: 本轮只放一类件(正/反/边界)。跑完执行:")
        print('       python dbg_report.py --dir "%s" --holes 0 --sweep --truth front|back'
              % os.path.abspath(args.collect))
    if args.timing:
        RUNTIME_LOGGER.info(
            "[TIMING-ON] 详细耗时已开启，超时预算=%.1fms 只约束算法本身(入队→判定，"
            "不含网络取图/存图 IO)；存图已异步，raw/result_save_ms 不在关键路径",
            RESULT_DEADLINE_MS)
    try:
        source = build_source(args.mode, args.dir, args.ip)
    except Exception as exc:  # noqa: BLE001
        print("[FATAL] 取图源初始化失败: %s" % exc)
        return 2
    if args.mode == "local" and len(getattr(source, "files", [])) == 0:
        print("[FATAL] 目录下没有图片: %s" % args.dir)
        return 2

    plc = ModbusReporter()
    uno = None
    if ENABLE_UNO:
        try:
            # 文件名含中文，但 Python 允许导入中文模块名。
            from uno_relay import UnoRelayController
            uno = UnoRelayController(
                port=UNO_PORT,
                pin=UNO_PIN,
                baudrate=UNO_BAUDRATE,
                pulse_seconds=UNO_PULSE_SECONDS,
            )
            if not uno.connect():
                print("[WARN] UNO 未连接，视觉检测继续运行，但 NG 不会驱动电磁阀")
                uno = None
        except Exception as exc:  # noqa: BLE001
            print("[WARN] UNO 控制器初始化失败: %s" % exc)
            uno = None
    n_ok = n_ng = n_bad = n_control_late = n_uno_fail = n_plc_fail = n_no_actuator = 0
    n_timeout = n_timeout_backlog = n_timeout_algo = 0  # 超时 NG 与其两种根因
    consecutive_timeouts = 0  # 连续超时计数，达到 TIMEOUT_ESCALATE_N 升级停线
    hit = miss = 0
    timing_stats = TimingStats()
    saver = AsyncImageSaver()  # 存图交后台线程异步执行，不占检测耗时/不推迟喷嘴
    t_start = time.time()
    exit_code = 0

    def emit_control(is_ok: bool, image_name: str, tag: str) -> Tuple[Optional[float], float]:
        """判定一出立刻吹气/报 PLC（关键路径），返回 (uno_ms, plc_ms)。"""
        nonlocal n_uno_fail, n_no_actuator, n_plc_fail
        uno_ms: Optional[float] = None
        if not is_ok:
            if uno is not None:
                uno_t0 = time.perf_counter()
                if not uno.pulse():
                    n_uno_fail += 1
                    RUNTIME_LOGGER.error("[IO-ERROR] image=%s %s 脉冲发送失败", image_name, tag)
                uno_ms = (time.perf_counter() - uno_t0) * 1000.0
            else:
                n_no_actuator += 1  # NG 但无 UNO：漏吹，必须在汇总里暴露
        plc_t0 = time.perf_counter()
        try:
            plc.report(is_ok)
        except Exception as exc:  # noqa: BLE001
            n_plc_fail += 1
            RUNTIME_LOGGER.error("[IO-ERROR] image=%s PLC 上报失败: %s", image_name, exc)
        return uno_ms, (time.perf_counter() - plc_t0) * 1000.0

    def log_timeout_ng(cause: str, frame_id, camera_name: str, enqueued_wall,
                       queue_wait_ms, inspect_ms, window_ms, raw_verdict: str) -> None:
        """始终打印的 TIMEOUT_NG 醒目日志：记录四要素+根因，供事后区分算法慢/系统卡顿。"""
        def _wall(t):
            if t is None:
                return "n/a"
            return "%s.%03d" % (time.strftime("%H:%M:%S", time.localtime(t)),
                                int(t * 1000) % 1000)
        RUNTIME_LOGGER.critical(
            "[TIMEOUT-NG] cause=%s frame=%s camera=%s enqueue=%s trigger=%s "
            "queue_wait=%sms inspect=%sms window=%sms budget=%.1fms raw_verdict=%s"
            "；仍吹NG(fail-safe)，无工件ID/编码器时可能吹到后件",
            cause, frame_id, camera_name or "n/a", _wall(enqueued_wall), _wall(time.time()),
            _fmt_ms(queue_wait_ms), _fmt_ms(inspect_ms), _fmt_ms(window_ms),
            RESULT_DEADLINE_MS, raw_verdict or "(skipped)")

    try:
        for name, bgr in source.frames():
            source_timing = getattr(source, "current_frame_timing", None)
            timing = replace(source_timing) if isinstance(source_timing, FrameTiming) else FrameTiming()
            frame_started_at = getattr(source, "current_frame_started_at", None)
            # 每帧开头打一条分隔线，肉眼一眼分清上一件结束/下一件开始（含超时/积压/无效帧）。
            RUNTIME_LOGGER.info(
                "======== 帧 #%s %s ========",
                timing.frame_id if timing.frame_id is not None else "?", name)
            if bgr is None:
                n_bad += 1
                RUNTIME_LOGGER.warning("[FRAME-INVALID] %s，按 NG 处理", name)
                timing.uno_ms, timing.plc_ms = emit_control(False, name, "无效帧 NG")
                timing.raw_verdict = timing.effective_verdict = NG_INVALID_FRAME
                n_ng += 1
                consecutive_timeouts = 0  # 无效帧不是超时，重置连续超时
                if frame_started_at is not None:
                    timing.rx_first_to_control_ms = max(
                        0.0, (time.perf_counter() - float(frame_started_at)) * 1000.0)
                    timing.rx_first_to_full_ms = timing.rx_first_to_control_ms
                if args.timing:
                    invalid_res = InspectResult(name=name, verdict=NG_INVALID_FRAME,
                                                reason="图像读取失败")
                    log_frame_timing(name, timing, invalid_res)
                    timing_stats.add(timing)
                continue

            # 超时窗口起点=成功入队(perf_counter)；不含网络取图，也不含存图 IO。
            enqueued_at = getattr(source, "current_frame_enqueued_at", None)
            enqueued_wall = getattr(source, "current_frame_enqueued_wall", None)
            camera_name = getattr(source, "current_frame_camera_name", "")

            # --- 出队过期预筛（队列保护/防雪崩）：已在队列堆到过期就跳过 inspect ---
            if enqueued_at is not None:
                age_ms = max(0.0, (time.perf_counter() - float(enqueued_at)) * 1000.0)
                if age_ms > RESULT_DEADLINE_MS:
                    res = InspectResult(name=name, verdict=TIMEOUT_NG,
                                        reason="队列积压 %.1fms > %.1fms，跳过检测强制 NG" %
                                        (age_ms, RESULT_DEADLINE_MS))
                    proc = bgr
                    timing.queue_wait_ms = age_ms
                    timing.deadline_margin_ms = RESULT_DEADLINE_MS - age_ms
                    timing.late = True
                    timing.raw_verdict = "(skipped)"
                    timing.effective_verdict = TIMEOUT_NG
                    timing.uno_ms, timing.plc_ms = emit_control(False, name, "超时 NG")
                    n_timeout += 1
                    n_timeout_backlog += 1
                    n_ng += 1
                    consecutive_timeouts += 1
                    log_timeout_ng("queue_backlog", timing.frame_id, camera_name,
                                   enqueued_wall, age_ms, None, age_ms, "(skipped)")
                    # RAW 已在采集线程组装时落盘，此处只存结果图(可牺牲)。
                    saver.submit_result(proc, res, overlay=True)
                    if args.timing:
                        log_frame_timing(name, timing, res)
                        timing_stats.add(timing, res.timings_ms)
                    if TIMEOUT_ESCALATE_N > 0 and consecutive_timeouts >= TIMEOUT_ESCALATE_N:
                        raise AcquisitionError(
                            "连续 %d 次超时，升级停线；请降速/排查算法或磁盘" % consecutive_timeouts)
                    continue

            queue_text = _fmt_ms(timing.queue_wait_ms)
            RUNTIME_LOGGER.info("[INSPECT-START] image=%s queue_wait=%sms", name, queue_text)
            if args.calib:
                calibrate(bgr, name)
                continue

            res, proc = inspect(bgr, name, timing=args.timing)
            decision_at = time.perf_counter()
            timing.inspect_ms = res.elapsed_ms
            timing.raw_verdict = res.verdict
            # 首块口径仅作诊断保留；截止判定用“入队 → 判定”窗口(不含网络/存图)。
            if frame_started_at is not None:
                timing.rx_first_to_decision_ms = max(
                    0.0, (decision_at - float(frame_started_at)) * 1000.0)
            window_ms = None
            if enqueued_at is not None:
                window_ms = max(0.0, (decision_at - float(enqueued_at)) * 1000.0)
                timing.deadline_margin_ms = RESULT_DEADLINE_MS - window_ms
                timing.late = window_ms > RESULT_DEADLINE_MS
            timeout_raw_verdict = None
            if timing.late:
                timeout_raw_verdict = res.verdict
                res.verdict = TIMEOUT_NG
                res.reason = ("算法超时 %.1fms > %.1fms；原判定=%s，按安全策略强制 NG"
                              % (window_ms or 0.0, RESULT_DEADLINE_MS, timeout_raw_verdict))
            timing.effective_verdict = res.verdict

            # 控制关键路径：判定一出立刻吹气/报 PLC，日志与存图全部后置。
            timing.uno_ms, timing.plc_ms = emit_control(res.is_ok, name, "NG")
            control_done_at = time.perf_counter()
            if frame_started_at is not None:
                timing.rx_first_to_control_ms = max(
                    0.0, (control_done_at - float(frame_started_at)) * 1000.0)
            if enqueued_at is not None:
                timing.control_late = (
                    (control_done_at - float(enqueued_at)) * 1000.0) > RESULT_DEADLINE_MS

            # --- 控制已完成，以下均为可后置的日志与异步存图 ---
            if timeout_raw_verdict is not None:
                n_timeout += 1
                n_timeout_algo += 1
                consecutive_timeouts += 1
                log_timeout_ng("algo_slow", timing.frame_id, camera_name, enqueued_wall,
                               timing.queue_wait_ms, res.elapsed_ms, window_ms, timeout_raw_verdict)
            else:
                consecutive_timeouts = 0  # 正常判定归零连续超时
            RUNTIME_LOGGER.info("[INSPECT-DONE] image=%s raw=%s effective=%s elapsed=%.1fms",
                                res.name, timing.raw_verdict, res.verdict, res.elapsed_ms)
            if res.verdict in (NG_PART_NOT_FOUND, NG_HOLE_NOT_FOUND):
                RUNTIME_LOGGER.warning("[LOCATE] method=%s verdict=%s reason=%s",
                                       res.locate_method, res.verdict, res.reason)
            if PRINT_DEBUG:
                print_result(res)
            if timing.control_late:
                n_control_late += 1
                RUNTIME_LOGGER.warning(
                    "[CONTROL-LATE] image=%s frame=%s 超预算 budget=%.1fms，已强制 NG 吹气",
                    name, timing.frame_id, RESULT_DEADLINE_MS)

            # 结果图异步：submit 后立刻返回，磁盘卡顿不占检测耗时/不推迟下一件。
            # RAW 已在采集线程“帧组装时”落盘，不在主循环，故这里只存结果图。
            saver.submit_result(proc, res, overlay=True)
            if frame_started_at is not None:
                timing.rx_first_to_full_ms = max(
                    0.0, (time.perf_counter() - float(frame_started_at)) * 1000.0)
            if args.timing:
                log_frame_timing(name, timing, res)
                timing_stats.add(timing, res.timings_ms)

            if TIMEOUT_ESCALATE_N > 0 and consecutive_timeouts >= TIMEOUT_ESCALATE_N:
                n_ok += res.is_ok
                n_ng += (not res.is_ok)
                raise AcquisitionError(
                    "连续 %d 次超时，升级停线；请降速/排查算法或磁盘" % consecutive_timeouts)

            n_ok += res.is_ok
            n_ng += (not res.is_ok)
            truth = guess_label(name)
            if truth is not None:
                hit += (truth == res.is_ok)
                miss += (truth != res.is_ok)
    except KeyboardInterrupt:  # camera/watch/http 的正常停机方式
        RUNTIME_LOGGER.info("[STOP] 用户中断")
    except AcquisitionError as exc:
        exit_code = 3
        RUNTIME_LOGGER.critical("[ACQ-FATAL] %s；已停止检测，请停线排查", exc)
    finally:
        getattr(source, "close", lambda: None)()
        saver.close()  # 尽量把已入队留档写完再退出
        plc.close()
        if uno is not None:
            uno.close()
    if not args.calib:
        total = n_ok + n_ng
        acq = getattr(source, "received_frames", 0)
        accepted = getattr(source, "accepted_frames", 0)
        overload = getattr(source, "overload_frames", 0)
        drops = getattr(source, "protocol_drops", 0)
        rec_errors = getattr(source, "record_errors", 0)
        reconnects = getattr(source, "reconnects", 0)
        max_depth = getattr(source, "max_queue_depth", 0)
        raw_saver = getattr(source, "_raw_saver", None)
        raw_saved = getattr(raw_saver, "raw_saved", 0)
        partial_saved = getattr(raw_saver, "partial_saved", 0)
        raw_dropped = getattr(raw_saver, "raw_dropped", 0)
        raw_fail = getattr(raw_saver, "raw_fail", 0)
        RUNTIME_LOGGER.info(
            "[SUMMARY] processed=%d OK=%d NG=%d invalid=%d timeout_ng=%d "
            "(backlog=%d algo=%d) raw_saved=%d partial_saved=%d raw_dropped=%d raw_fail=%d "
            "result_fail=%d dropped_saves=%d "
            "control_late=%d uno_fail=%d no_actuator=%d plc_fail=%d received=%d "
            "accepted=%d overload_drop=%d protocol_drops=%d record_errors=%d reconnects=%d "
            "max_queue=%d save_queue_max=%d elapsed=%.2fs",
            total, n_ok, n_ng, n_bad, n_timeout, n_timeout_backlog, n_timeout_algo,
            raw_saved, partial_saved, raw_dropped, raw_fail,
            saver.result_fail, saver.dropped_saves, n_control_late,
            n_uno_fail, n_no_actuator, n_plc_fail, acq, accepted, overload, drops, rec_errors,
            reconnects, max_depth, saver.max_queue_depth, time.time() - t_start)
        # RAW 对账：raw_saved+partial_saved 应等于传感器存图张数；received=完整帧，protocol_drops=残帧。
        RUNTIME_LOGGER.info(
            "[SUMMARY-RAW] RAW 落盘合计=%d(完整=%d + 残帧=%d)；应与传感器存图张数一致。"
            "received(完整)=%d protocol_drops(残帧)=%d raw_dropped=%d raw_fail=%d",
            raw_saved + partial_saved, raw_saved, partial_saved,
            acq, drops, raw_dropped, raw_fail)
        if raw_dropped or raw_fail:
            RUNTIME_LOGGER.error("[SUMMARY] ⚠ RAW 有丢弃(%d)或写盘失败(%d)，与传感器张数将不一致，"
                                 "请查磁盘/降触发频率", raw_dropped, raw_fail)
        # A 方案 RX 探针对账：rx_gap_max 大(比如 >100ms) 且 protocol_drops(残帧) 多 =
        # RX 线程被 inspect 慢帧占 GIL 饿死、来不及收包 → TCP 反压 → 残帧。此时该上 C 方案(缩图/搬 C++)。
        rx_gap_max = getattr(source, "rx_gap_max_ms", 0.0)
        rx_calls = getattr(source, "rx_recv_calls", 0)
        RUNTIME_LOGGER.info(
            "[SUMMARY-RX] rx_gap_max=%.1fms rx_recv_calls=%d checksum=%s nodelay=%s；"
            "gap 大且残帧多 → RX 被 inspect 饿死(GIL)，考虑缩图/搬 C++",
            rx_gap_max, rx_calls, CAM_VALIDATE_RECORD_CHECKSUM, CAM_TCP_NODELAY)
        if n_no_actuator:
            RUNTIME_LOGGER.error("[SUMMARY] ⚠ %d 个 NG 无 UNO 可吹气(启动未连上)，实际未分选",
                                 n_no_actuator)
        if overload:
            RUNTIME_LOGGER.error("[SUMMARY] ⚠ %d 帧因队列溢出被丢弃(无判定无吹气，漏检风险)，"
                                 "请降速/排查", overload)
        if args.timing:
            for line in timing_stats.summary_lines():
                RUNTIME_LOGGER.info(line)
        if hit + miss > 0:
            RUNTIME_LOGGER.info("[SELF-CHECK] labeled=%d hit=%d miss=%d accuracy=%.1f%%",
                                hit + miss, hit, miss, 100.0 * hit / (hit + miss))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
