#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>   // 新增这一行！！
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <optional>
#include <tuple>
#include <limits>
double np_percentile_linear(const std::vector<uint8_t>& sorted_vals, double pct);

namespace py = pybind11;

// 强制连续内存 + 类型转换的数组别名。
// Python 侧传入的 cand[:, :2]、crop 视图 win/win_enhanced 等都是非连续切片，
// 若按连续内存 (ptr[i*stride_assumed]) 读取会错位。加上 c_style|forcecast 后，
// pybind11 会在需要时自动生成一份连续副本，从根本上消除 stride 读错的问题。
using U8Array = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;
using F64Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
using F32Array = py::array_t<float, py::array::c_style | py::array::forcecast>;

template<typename T>
T clamp_val(T v, T lo, T hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 转成 >=lo 的奇数，供高斯核尺寸使用（对齐 Python odd()）。
static int odd_ksize(double v, int lo = 3)
{
    int k = static_cast<int>(std::lround(v));
    if (k < lo) k = lo;
    return (k % 2 == 1) ? k : k + 1;
}

// 忽略 NaN 的中位数（对齐 numpy nanmedian）。返回 NaN 表示无有效值。
static double nanmedian(std::vector<double>& v)
{
    std::vector<double> f;
    f.reserve(v.size());
    for (double x : v)
        if (!std::isnan(x)) f.push_back(x);
    if (f.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(f.begin(), f.end());
    size_t n = f.size();
    if (n % 2 == 1) return f[n / 2];
    return 0.5 * (f[n / 2 - 1] + f[n / 2]);
}

// ✅ 移到全局，放在这里
double median_u8(std::vector<uint8_t>& v)
{
    if(v.empty()) return 0;
    std::sort(v.begin(),v.end());
    size_t n = v.size();
    if(n%2 ==1)
        return v[n/2];
    else
        return (v[n/2 -1] + v[n/2]) / 2.0;
}

// 角度覆盖率计算
double angular_coverage(const std::vector<double>& xs, const std::vector<double>& ys, int bins=36)
{
    if(xs.empty()) return 0.0;
    std::vector<int> hit(bins,0);
    for(size_t i=0;i<xs.size();i++)
    {
        double rad = std::atan2(ys[i], xs[i]);
        double deg = rad * 180.0 / M_PI;
        deg = fmod(deg + 360.0, 360.0);
        int bidx = static_cast<int>(deg / (360.0 / bins));
        bidx = std::clamp(bidx, 0, bins-1);
        hit[bidx] = 1;
    }
    int cnt = 0;
    for(auto v:hit) cnt += v;
    return static_cast<double>(cnt)/bins;
}

// crop_pad：以(hx,hy)为中心，裁剪2*half+1大小ROI，越界补0
cv::Mat crop_pad(const cv::Mat& src, double cx, double cy, int half)
{
    int src_x = static_cast<int>(std::round(cx)) - half;
    int src_y = static_cast<int>(std::round(cy)) - half;
    int w_roi = half * 2;
    int h_roi = half * 2;

    int x0 = src_x;
    int y0 = src_y;
    int x1 = x0 + w_roi;
    int y1 = y0 + h_roi;

    int h = src.rows;
    int w = src.cols;

    // 计算上下左右padding
    int pad_top    = std::max(0, -y0);
    int pad_bottom = std::max(0, y1 - h);
    int pad_left   = std::max(0, -x0);
    int pad_right  = std::max(0, x1 - w);

    int sx0 = std::max(0, y0);
    int sy0 = std::max(0, x0);
    int src_h = std::min(y1, h) - sx0;
    int src_w = std::min(x1, w) - sy0;

    // ROI 完全在视场外：对齐 Python 原版返回 2*half 的全零图，避免负 Rect 崩溃。
    if (src_w <= 0 || src_h <= 0)
        return cv::Mat::zeros(w_roi, h_roi, src.type());

    cv::Mat sub = src(cv::Rect(sy0, sx0, src_w, src_h)).clone();
    cv::Mat dst;
    // BORDER_REPLICATE 和Python原版保持一致！！
    cv::copyMakeBorder(sub, dst, pad_top, pad_bottom, pad_left, pad_right, cv::BORDER_REPLICATE);
    return dst;
}


// local_enhance：CLAHE + 高斯模糊，参数外部传入。
// 顺序必须与 Python 原版一致：先 CLAHE，再（K>=3 时）用奇数核做高斯。
// 之前实现是先高斯再 CLAHE，两步不可交换，会改变环轮廓数量/圆度。
cv::Mat local_enhance(const cv::Mat& sub, int gauss_ksize, double clahe_clip, int clahe_grid_w, int clahe_grid_h)
{
    cv::Mat out;
    auto clahe = cv::createCLAHE(clahe_clip, cv::Size(clahe_grid_w, clahe_grid_h));
    clahe->apply(sub, out);
    if(gauss_ksize >= 3)
    {
        int k = odd_ksize(gauss_ksize);
        cv::GaussianBlur(out, out, cv::Size(k, k), 0);
    }
    return out;
}

// find_contours 封装
std::vector<std::vector<cv::Point>> find_contours(const cv::Mat& bw, int mode, int method)
{
    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bw, cnts, hierarchy, mode, method);
    return cnts;
}

// ===================== 核心函数，签名完全匹配你的声明 =====================
py::tuple feature_a_ring_contours(
    U8Array gray,
    double hx, double hy, double r,
    double RING_ROI_RATIO,
    double RING_MASK_RATIO,
    double RING_BAND_LO, double RING_BAND_HI,
    F64Array RING_THRESH_PCTS,
    int RING_MIN_CONTOUR_PTS,
    double RING_MAX_RADIAL_STD,
    double RING_MIN_ANGLE_COVER,
    double RING_CLUSTER_GAP,
    int RING_CLUSTER_MIN_HITS,
    double CLAHE_CLIP,
    int CLAHE_GRID_W, int CLAHE_GRID_H,
    int GAUSS_BLUR_K
)
{
    py::buffer_info gray_buf = gray.request();
    if (gray_buf.ndim != 2)
        throw std::runtime_error("gray must be 2D uint8 array");
    cv::Mat gray_mat(gray_buf.shape[0], gray_buf.shape[1], CV_8UC1, gray_buf.ptr);

    int half = static_cast<int>(std::round(RING_ROI_RATIO * r));
    if (half < 6)
    {
        return py::make_tuple(0, 0, std::vector<double>{});
    }

    cv::Mat sub = crop_pad(gray_mat, hx, hy, half);
    sub = local_enhance(sub, GAUSS_BLUR_K, CLAHE_CLIP, CLAHE_GRID_W, CLAHE_GRID_H);

    // 圆形掩膜
    cv::Mat mask = cv::Mat::zeros(sub.size(), CV_8UC1);
    int mask_r = static_cast<int>(std::round(RING_MASK_RATIO * r));
    cv::circle(mask, cv::Point(half, half), mask_r, 255, -1);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    const double lo = RING_BAND_LO * r;
    const double hi = RING_BAND_HI * r;

    // 读取传入的百分位列表
    py::buffer_info pct_buf = RING_THRESH_PCTS.request();
    if(pct_buf.ndim !=1)
        throw std::runtime_error("RING_THRESH_PCTS must be 1D double array");
    double* pct_ptr = static_cast<double*>(pct_buf.ptr);
    std::vector<double> thresh_pcts(pct_ptr, pct_ptr + pct_buf.shape[0]);

    std::vector<double> radii_hits;
    int raw_cnt = 0;

    for(double pct : thresh_pcts)
    {
        // 计算该百分位灰度
        cv::Mat flat = sub.reshape(1,1);
        std::vector<uint8_t> vec;
        flat.copyTo(vec);
        std::sort(vec.begin(), vec.end());
        // 保持浮点阈值，直接传给 cv::threshold（对齐 Python cv2.threshold(sub, float(q), ...)）。
        // 之前截断成 uint8 会使临界像素翻转，改变环轮廓。
        double q = np_percentile_linear(vec, pct);


        std::vector<int> flags = {cv::THRESH_BINARY_INV, cv::THRESH_BINARY};
        for(int flag : flags)
        {
            cv::Mat bw;
            cv::threshold(sub, bw, q, 255, flag);
            cv::bitwise_and(bw, mask, bw);
            cv::morphologyEx(bw, bw, cv::MORPH_OPEN, kernel);

            auto cnts = find_contours(bw, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            for(auto& cnt : cnts)
            {
                if ((int)cnt.size() < RING_MIN_CONTOUR_PTS)
                    continue;
                raw_cnt ++;

                std::vector<double> xs, ys, dists;
                xs.reserve(cnt.size());
                ys.reserve(cnt.size());
                dists.reserve(cnt.size());
                for(auto& pt : cnt)
                {
                    double px = pt.x - half;
                    double py = pt.y - half;
                    xs.push_back(px);
                    ys.push_back(py);
                    dists.push_back(std::hypot(px, py));
                }

                double sum_r = 0.0;
                for(auto d : dists) sum_r += d;
                double mean_r = sum_r / dists.size();
                if (mean_r < 1e-6 || !(mean_r >= lo && mean_r <= hi))
                    continue;

                double sum_sq = 0.0;
                for(auto d : dists) sum_sq += (d - mean_r)*(d - mean_r);
                double std_r = std::sqrt(sum_sq / dists.size());
                if (std_r / mean_r > RING_MAX_RADIAL_STD)
                    continue;

                double ang_cov = angular_coverage(xs, ys,36);

                if (ang_cov < RING_MIN_ANGLE_COVER)
                    continue;

                radii_hits.push_back(mean_r);
            }
        }
    }

    // 半径聚类
    std::sort(radii_hits.begin(), radii_hits.end());
    std::vector<std::vector<double>> clusters;
    for(double v : radii_hits)
    {
        if(clusters.empty() || v - clusters.back().back() > RING_CLUSTER_GAP * r)
        {
            clusters.emplace_back();
            clusters.back().push_back(v);
        }
        else
        {
            clusters.back().push_back(v);
        }
    }

    std::vector<double> ring_ratios;
    for(auto& c : clusters)
    {
        if ((int)c.size() >= RING_CLUSTER_MIN_HITS)
        {
            double sum_c = 0.0;
            for(auto val : c) sum_c += val;
            double avg_r = sum_c / c.size();
            double ratio = std::round(avg_r / r * 1000.0) / 1000.0;
            ring_ratios.push_back(ratio);
        }
    }

    return py::make_tuple(raw_cnt, (int)ring_ratios.size(), ring_ratios);
}
/**
 * @brief 复刻Python fit_circle_robust 迭代Kasa最小二乘圆拟合
 * @param pts_arr N×2 double点数组
 * @param iters 迭代次数
 * @param tol_ratio 容差比例
 * @param tol_min 最小像素容差
 * @param min_pts 最少有效点
 * @return nullopt失败；tuple(cx,cy,r,inlier_count)
 */
// 核心迭代拟合，直接对点向量操作，供 C++ 内部（refine_hole）复用，避免 numpy 往返。
std::optional<std::tuple<double, double, double, int>>
fit_circle_robust_core(
    const std::vector<cv::Point2d>& pts,
    int iters,
    double tol_ratio,
    double tol_min,
    int min_pts)
{
    const size_t n = pts.size();
    if (n < static_cast<size_t>(min_pts))
        return std::nullopt;

    std::vector<cv::Point2d> cur = pts;
    double cx{ 0 }, cy{ 0 }, r{ 0 };

    for (int iter = 0; iter < iters; iter++)
    {
        size_t m = cur.size();
        cv::Mat A(m, 3, CV_64F);
        cv::Mat b_mat(m, 1, CV_64F);
        for (size_t i = 0; i < m; i++)
        {
            double x = cur[i].x;
            double y = cur[i].y;
            A.at<double>(i, 0) = 2.0 * x;
            A.at<double>(i, 1) = 2.0 * y;
            A.at<double>(i, 2) = 1.0;
            b_mat.at<double>(i, 0) = x * x + y * y;
        }
        cv::Mat sol;
        cv::solve(A, b_mat, sol, cv::DECOMP_SVD);
        cx = sol.at<double>(0, 0);
        cy = sol.at<double>(1, 0);
        r = std::sqrt(std::max(sol.at<double>(2, 0) + cx * cx + cy * cy, 1e-9));

        std::vector<cv::Point2d> keep;
        double tol = std::max(tol_ratio * r, tol_min);
        for (auto& p : cur)
        {
            double d = std::hypot(p.x - cx, p.y - cy);
            if (std::abs(d - r) < tol)
                keep.push_back(p);
        }
        if ((int)keep.size() < min_pts || keep.size() == cur.size())
            break;
        cur.swap(keep);
    }
    return std::make_tuple(cx, cy, r, static_cast<int>(cur.size()));
}

// pybind 暴露的包装：F64Array 带 c_style|forcecast，pybind 会把非连续的
// cand[:, :2] 自动转成连续副本后再读取，彻底修掉之前 ptr[i*2] 的错位 bug。
std::optional<std::tuple<double, double, double, int>>
fit_circle_robust(
    F64Array pts_arr,
    int iters,
    double tol_ratio,
    double tol_min,
    int min_pts)
{
    py::buffer_info buf = pts_arr.request();
    if (buf.ndim != 2 || buf.shape[1] != 2)
        throw std::runtime_error("pts must be N x 2 double numpy array");

    const size_t n = buf.shape[0];
    auto* ptr = static_cast<double*>(buf.ptr);
    std::vector<cv::Point2d> pts;
    pts.reserve(n);
    for (size_t i = 0; i < n; i++)
        pts.emplace_back(ptr[i * 2 + 0], ptr[i * 2 + 1]);

    return fit_circle_robust_core(pts, iters, tol_ratio, tol_min, min_pts);
}

// 返回tuple: (cx, cy, r, contrast, valid_flag)
// valid_flag = 1成功，0失败
py::tuple refine_hole(
    U8Array gray,
    double cx0, double cy0, double r0,
    F32Array cos_t,
    F32Array sin_t,
    double REFINE_SCAN_BAND0, double REFINE_SCAN_BAND1,
    double REFINE_RADIUS_STEP_PX,
    double REFINE_INNER_BAND, double REFINE_LAND_BAND,
    double REFINE_MIN_CONTRAST,
    double REFINE_EDGE_START,
    int REFINE_MIN_EDGE_PTS,
    int REFINE_ITERS,
    double REFINE_INLIER_RATIO
) {
    // 本函数严格复刻 Pure_Py2.refine_hole：沿 360° 射线找孔壁"灰度 50% 跨越点"再拟合。
    // 关键点：按半径区间取中位数（不是单行索引）、阈值=0.5*(inner+land)、按极性单向找跨越、
    // 从 REFINE_EDGE_START*r0 半径处开始、拟合用 tol_min=3/min_pts=max(12,N/2)、最后校验半径范围。

    // 1. 灰度转 float32
    auto gray_buf = gray.request();
    int h = static_cast<int>(gray_buf.shape[0]);
    int w = static_cast<int>(gray_buf.shape[1]);
    cv::Mat gray_u8(h, w, CV_8UC1, gray_buf.ptr);
    cv::Mat gray_f;
    gray_u8.convertTo(gray_f, CV_32F);

    auto cos_buf = cos_t.request();
    auto sin_buf = sin_t.request();
    int n_ang = static_cast<int>(cos_buf.shape[0]);
    const float* cos_ptr = static_cast<const float*>(cos_buf.ptr);
    const float* sin_ptr = static_cast<const float*>(sin_buf.ptr);

    // 2. radii = np.arange(band0*r0, band1*r0, max(0.2, step))
    double step = std::max(0.2, REFINE_RADIUS_STEP_PX);
    double r_start = REFINE_SCAN_BAND0 * r0;
    double r_end   = REFINE_SCAN_BAND1 * r0;
    int n_r = static_cast<int>(std::ceil((r_end - r_start) / step));
    if (n_r < 0) n_r = 0;
    std::vector<double> radii(n_r);
    for (int i = 0; i < n_r; i++)
        radii[i] = r_start + i * step;
    if (n_r < 8)
        return py::make_tuple(0.0, 0.0, 0.0, 0.0, 0);

    // 3. remap 采样 vals[n_r][n_ang]，并对越界点置 NaN（对齐 inside 掩膜）
    cv::Mat map_x(n_r, n_ang, CV_32F);
    cv::Mat map_y(n_r, n_ang, CV_32F);
    std::vector<uint8_t> inside(static_cast<size_t>(n_r) * n_ang, 0);
    for (int ri = 0; ri < n_r; ri++)
    {
        double rr = radii[ri];
        for (int ai = 0; ai < n_ang; ai++)
        {
            double x = cx0 + rr * cos_ptr[ai];
            double y = cy0 + rr * sin_ptr[ai];
            inside[static_cast<size_t>(ri) * n_ang + ai] =
                (x > 1.0 && y > 1.0 && x < w - 2.0 && y < h - 2.0) ? 1 : 0;
            map_x.at<float>(ri, ai) = static_cast<float>(clamp_val(x, 0.0, (double)(w - 1)));
            map_y.at<float>(ri, ai) = static_cast<float>(clamp_val(y, 0.0, (double)(h - 1)));
        }
    }
    cv::Mat sample_mat;
    cv::remap(gray_f, sample_mat, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    const double NAN_D = std::numeric_limits<double>::quiet_NaN();

    // 4. 每个半径行的角度中位数 med[ri]，再取 inner/land 区间中位数求 contrast
    std::vector<double> med(n_r, NAN_D);
    for (int ri = 0; ri < n_r; ri++)
    {
        std::vector<double> row;
        row.reserve(n_ang);
        for (int ai = 0; ai < n_ang; ai++)
        {
            if (inside[static_cast<size_t>(ri) * n_ang + ai])
                row.push_back(sample_mat.at<float>(ri, ai));
        }
        if (!row.empty())
            med[ri] = nanmedian(row);   // row 已无 NaN，等价普通中位数
    }

    std::vector<double> inner_meds, land_meds;
    bool in_any = false, la_any = false;
    for (int ri = 0; ri < n_r; ri++)
    {
        if (radii[ri] < REFINE_INNER_BAND * r0) { in_any = true; inner_meds.push_back(med[ri]); }
        if (radii[ri] > REFINE_LAND_BAND  * r0) { la_any = true; land_meds.push_back(med[ri]); }
    }
    if (!in_any || !la_any)
        return py::make_tuple(0.0, 0.0, 0.0, 0.0, 0);

    double inner = nanmedian(inner_meds);
    double land  = nanmedian(land_meds);
    if (!std::isfinite(inner) || !std::isfinite(land))
        return py::make_tuple(0.0, 0.0, 0.0, 0.0, 0);

    double contrast = std::abs(inner - land);
    if (contrast < REFINE_MIN_CONTRAST)
        return py::make_tuple(0.0, 0.0, 0.0, contrast, 0);

    // 5. 阈值=中点，极性由 inner/land 亮暗决定，从 REFINE_EDGE_START*r0 起单向找跨越
    double thr = 0.5 * (inner + land);
    double sign = (inner > land) ? 1.0 : -1.0;
    // start = np.searchsorted(radii, REFINE_EDGE_START*r0)  (left)
    double edge_start_r = REFINE_EDGE_START * r0;
    int start = 0;
    while (start < n_r && radii[start] < edge_start_r) start++;

    std::vector<cv::Point2d> edge_pts;
    for (int j = 0; j < n_ang; j++)
    {
        // col = sign*(vals[:,j]-thr)，NaN->-1
        // cross: 第一个 col[start+k] > 0 且 col[start+k+1] <= 0
        for (int k = start; k + 1 < n_r; k++)
        {
            double v0 = inside[static_cast<size_t>(k) * n_ang + j]
                        ? (double)sample_mat.at<float>(k, j) : NAN_D;
            double v1 = inside[static_cast<size_t>(k + 1) * n_ang + j]
                        ? (double)sample_mat.at<float>(k + 1, j) : NAN_D;
            double c0 = std::isnan(v0) ? -1.0 : sign * (v0 - thr);
            double c1 = std::isnan(v1) ? -1.0 : sign * (v1 - thr);
            if (c0 > 0.0 && c1 <= 0.0)
            {
                // 取内侧半径 radii[k]（对齐 reference：rr = radii[start + cross[0]]，无插值）
                double rr = radii[k];
                edge_pts.emplace_back(cx0 + rr * cos_ptr[j], cy0 + rr * sin_ptr[j]);
                break;
            }
        }
    }
    if ((int)edge_pts.size() < REFINE_MIN_EDGE_PTS)
        return py::make_tuple(0.0, 0.0, 0.0, contrast, 0);

    // 6. 圆拟合：tol_min=3.0，min_pts=max(12, REFINE_MIN_EDGE_PTS/2)（对齐 reference）
    int fit_min_pts = std::max(12, REFINE_MIN_EDGE_PTS / 2);
    auto fit_opt = fit_circle_robust_core(edge_pts, REFINE_ITERS, REFINE_INLIER_RATIO, 3.0, fit_min_pts);
    if (!fit_opt.has_value())
        return py::make_tuple(0.0, 0.0, 0.0, contrast, 0);

    auto fit_tuple = fit_opt.value();
    double cx = std::get<0>(fit_tuple);
    double cy = std::get<1>(fit_tuple);
    double r  = std::get<2>(fit_tuple);

    // 7. 校验拟合半径落在扫描带内（对齐 reference，防止拟合到离谱半径被当成有效孔）
    if (!(REFINE_SCAN_BAND0 * r0 < r && r < REFINE_SCAN_BAND1 * r0))
        return py::make_tuple(0.0, 0.0, 0.0, contrast, 0);

    return py::make_tuple(cx, cy, r, contrast, 1);
}

/**
 * @brief 复刻 np.percentile linear 线性插值模式
 * @param sorted_vals 已经从小到大排序的uint8数组
 * @param pct 百分比 0~100
 * @return 插值后的浮点灰度值
 */
double np_percentile_linear(const std::vector<uint8_t>& sorted_vals, double pct)
{
    const size_t n = sorted_vals.size();
    if (n == 0) return 0.0;
    if (n == 1) return static_cast<double>(sorted_vals[0]);

    double idx = (static_cast<double>(n - 1)) * pct / 100.0;
    size_t lo = static_cast<size_t>(std::floor(idx));
    size_t hi = static_cast<size_t>(std::ceil(idx));

    if (lo == hi)
    {
        return static_cast<double>(sorted_vals[lo]);
    }
    double w = idx - static_cast<double>(lo);
    double v0 = static_cast<double>(sorted_vals[lo]);
    double v1 = static_cast<double>(sorted_vals[hi]);
    return v0 * (1.0 - w) + v1 * w;
}

// 圆度计算
inline double circularity(const std::vector<cv::Point>& cnt)
{
    double area = cv::contourArea(cnt);
    if (area < 1e-9)
        return 0.0;

    double peri = cv::arcLength(cnt, true);
    if (peri < 1e-9)
        return 0.0;

    return 4.0 * M_PI * area / (peri * peri);
}

// 移植 _best_mark_circularity
double _best_mark_circularity(
    U8Array win_arr,
    double bx, double by, double br,
    double MARK_MASK_RATIO,
    double MARK_MIN_AREA_RATIO,
    F64Array MARK_THRESH_PCTS)
{
    // win_arr 是 forcecast 后的连续数组：即使 Python 传入的是非连续 crop 视图，
    // pybind 也已生成连续副本，用 (h,w) 直接构造 cv::Mat 不再错行。
    py::buffer_info buf = win_arr.request();
    int h = static_cast<int>(buf.shape[0]);
    int w = static_cast<int>(buf.shape[1]);
    uint8_t* ptr = static_cast<uint8_t*>(buf.ptr);
    cv::Mat win_tmp(h, w, CV_8UC1, ptr);
    cv::Mat win = win_tmp.clone();

    cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);
    int cr = std::max(2, static_cast<int>(std::round(MARK_MASK_RATIO * br)));
    cv::circle(mask, cv::Point(static_cast<int>(std::round(bx)), static_cast<int>(std::round(by))), cr, 255, -1);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    double min_area = MARK_MIN_AREA_RATIO * M_PI * br * br;
    double best = 0.0;

    py::buffer_info pct_buf = MARK_THRESH_PCTS.request();
    double* pct_ptr = static_cast<double*>(pct_buf.ptr);
    size_t pct_cnt = pct_buf.shape[0];

    std::vector<double> percentiles;
    std::vector<uint8_t> flat_vec;
    flat_vec.reserve(static_cast<size_t>(win.rows * win.cols));
    for (int y = 0; y < win.rows; y++)
    {
        for (int x = 0; x < win.cols; x++)
        {
            flat_vec.push_back(win.at<uint8_t>(y, x));
        }
    }
    std::sort(flat_vec.begin(), flat_vec.end());

    for (size_t i = 0; i < pct_cnt; i++)
    {
        double pct = pct_ptr[i];
        // 保持浮点阈值，不截断成 uint8（对齐 Python cv2.threshold(win, float(q), ...)）。
        percentiles.push_back(np_percentile_linear(flat_vec, pct));
    }

    for (double q : percentiles)
    {
        for (int flag : {cv::THRESH_BINARY_INV, cv::THRESH_BINARY})
        {
            cv::Mat bw;
            cv::threshold(win, bw, q, 255, flag);
            cv::bitwise_and(bw, mask, bw);
            cv::morphologyEx(bw, bw, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(bw, bw, cv::MORPH_OPEN, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (auto& cnt : contours)
            {
                double area = cv::contourArea(cnt);
                if (area < min_area)
                    continue;
                // 使用Point2d避免float截断精度丢失
                if (cv::pointPolygonTest(cnt, cv::Point2d(bx, by), false) < 0)
                    continue;

                double circ_val = circularity(cnt);
                if (circ_val > best)
                    best = circ_val;
            }
        }
    }
    return best;
}

std::vector<cv::Vec3d> _dedup_mark_candidates(
    F64Array candidates_arr,
    int half,
    double r,
    double MARK_DEDUP_DIST_RATIO,
    int MARK_MAX_CANDIDATES)
{
    py::buffer_info buf = candidates_arr.request();
    size_t N = buf.shape[0];
    double* p = static_cast<double*>(buf.ptr);
    std::vector<cv::Vec3d> cand_list;
    cand_list.reserve(N);
    for (size_t i = 0; i < N; i++)
    {
        double bx = p[i * 3 + 0];
        double by = p[i * 3 + 1];
        double br = p[i * 3 + 2];
        cand_list.emplace_back(bx, by, br);
    }

    std::sort(cand_list.begin(), cand_list.end(), [&](const cv::Vec3d& a, const cv::Vec3d& b)
        {
            double da = std::hypot(a[0] - half, a[1] - half) / std::max(r, 1e-6);
            double ra = std::abs(a[2] / std::max(r, 1e-6) - 0.38);
            double db = std::hypot(b[0] - half, b[1] - half) / std::max(r, 1e-6);
            double rb = std::abs(b[2] / std::max(r, 1e-6) - 0.38);
            if (da != db) return da < db;
            return ra < rb;
        });

    std::vector<cv::Vec3d> kept;
    double min_dist = MARK_DEDUP_DIST_RATIO * r;
    for (auto& c : cand_list)
    {
        double br = c[2];
        double ratio = br / std::max(r, 1e-6);
        // ===== 新增：半径比0.2~0.7过滤，和Python原版保持一致 =====
        if (!(0.20 <= ratio && ratio <= 0.70))
            continue;

        bool dup = false;
        for (auto& old : kept)
        {
            double d = std::hypot(c[0] - old[0], c[1] - old[1]);
            if (d < min_dist)
            {
                dup = true;
                break;
            }
        }
        if (dup) continue;
        kept.push_back(c);
        if (kept.size() >= MARK_MAX_CANDIDATES) break;
    }
    return kept;
}


struct CornerResult
{
    double bx;
    double by;
    double br;
    double circ_raw;
    double circ_enhanced;
    double circ;
    double center_offset;
    double radius_ratio;
    bool shape_ok;
    bool accepted;
};

CornerResult select_best_candidate(
    F64Array cand_array,
    U8Array win,
    U8Array win_enhanced,
    int half,
    double r,
    double MARK_CENTER_GATE,
    double dynamic_gate,
    double MARK_CIRCULARITY_MIN,
    double MARK_MASK_RATIO,
    double MARK_MIN_AREA_RATIO,
    double MARK_DEDUP_DIST_RATIO,
    int MARK_MAX_CANDIDATES,
    F64Array MARK_THRESH_PCTS)
{
    CornerResult ret{};
    ret.accepted = false;
    ret.shape_ok = false;
    std::vector<cv::Vec3d> cand_list = _dedup_mark_candidates(
        cand_array, half, r,
        MARK_DEDUP_DIST_RATIO,
        MARK_MAX_CANDIDATES
    );
    if (cand_list.empty()) return ret;

    using ScoreT = std::tuple<bool, double, double, double>;
    std::tuple<ScoreT, double, double, double, double, double, double, double, double, bool, bool> best;
    bool has_best = false;

    for (auto& cp : cand_list)
    {
        double bx = cp[0];
        double by = cp[1];
        double br = cp[2];
        double circ_raw = _best_mark_circularity(
            win, bx, by, br,
            MARK_MASK_RATIO,
            MARK_MIN_AREA_RATIO,
            MARK_THRESH_PCTS
        );
        double circ_enhanced = _best_mark_circularity(
            win_enhanced, bx, by, br,
            MARK_MASK_RATIO,
            MARK_MIN_AREA_RATIO,
            MARK_THRESH_PCTS
        );
        double circ = std::max(circ_raw, circ_enhanced);

        double center_offset = std::hypot(bx - half, by - half) / std::max(r, 1e-6);
        double radius_ratio = br / std::max(r, 1e-6);

        bool position_ok = (center_offset <= dynamic_gate);
        // 和原版Python完全一致
        bool shape_ok = (0.28 <= radius_ratio && radius_ratio <= 0.55
            && circ_enhanced > 0.45 && circ_raw > 0.10);
        bool accepted = (circ_raw > MARK_CIRCULARITY_MIN) || (position_ok && shape_ok);

        auto score = std::make_tuple(accepted, circ_raw + circ_enhanced,
            -center_offset, -std::abs(radius_ratio - 0.38));

        if (!has_best || score > std::get<0>(best))
        {
            best = { score, bx,by,br, circ_raw, circ_enhanced, circ,
                     center_offset, radius_ratio, shape_ok, accepted };
            has_best = true;
        }
    }
    if (has_best)
    {
        auto& v = best;
        ret.bx = std::get<1>(v);
        ret.by = std::get<2>(v);
        ret.br = std::get<3>(v);
        ret.circ_raw = std::get<4>(v);
        ret.circ_enhanced = std::get<5>(v);
        ret.circ = std::get<6>(v);
        ret.center_offset = std::get<7>(v);
        ret.radius_ratio = std::get<8>(v);
        ret.shape_ok = std::get<9>(v);
        ret.accepted = std::get<10>(v);
    }
    return ret;
}

PYBIND11_MODULE(bcorner, m) {
    m.doc() = "feature_b_corner_marks hotloop, all params pass from python";
    py::class_<CornerResult>(m, "CornerResult")
        .def_readwrite("bx", &CornerResult::bx)
        .def_readwrite("by", &CornerResult::by)
        .def_readwrite("br", &CornerResult::br)
        .def_readwrite("circ_raw", &CornerResult::circ_raw)
        .def_readwrite("circ_enhanced", &CornerResult::circ_enhanced)
        .def_readwrite("circ", &CornerResult::circ)
        .def_readwrite("center_offset", &CornerResult::center_offset)
        .def_readwrite("radius_ratio", &CornerResult::radius_ratio)
        .def_readwrite("shape_ok", &CornerResult::shape_ok)
        .def_readwrite("accepted", &CornerResult::accepted);

    m.def("select_best_candidate", &select_best_candidate,
        py::arg("cand_array"),
        py::arg("win"),
        py::arg("win_enhanced"),
        py::arg("half"),
        py::arg("r"),
        py::arg("MARK_CENTER_GATE"),
        py::arg("dynamic_gate"),
        py::arg("MARK_CIRCULARITY_MIN"),
        py::arg("MARK_MASK_RATIO"),
        py::arg("MARK_MIN_AREA_RATIO"),
        py::arg("MARK_DEDUP_DIST_RATIO"),
        py::arg("MARK_MAX_CANDIDATES"),
        py::arg("MARK_THRESH_PCTS"),
        "hotloop, all parameters passed from python");

    m.def("fit_circle_robust", &fit_circle_robust,
        py::arg("pts"),
        py::arg("iters"),
        py::arg("tol_ratio"),
        py::arg("tol_min"),
        py::arg("min_pts"),
        R"doc(迭代鲁棒Kasa圆拟合，输入N×2 double数组；返回None或者(cx,cy,r,inlier_count))doc");
    m.def("refine_hole", &refine_hole);
    m.def("feature_a_ring_contours", &feature_a_ring_contours,
        py::arg("gray"),
        py::arg("hx"), py::arg("hy"), py::arg("r"),
        py::arg("RING_ROI_RATIO"),
        py::arg("RING_MASK_RATIO"),
        py::arg("RING_BAND_LO"), py::arg("RING_BAND_HI"),
        py::arg("RING_THRESH_PCTS"),
        py::arg("RING_MIN_CONTOUR_PTS"),
        py::arg("RING_MAX_RADIAL_STD"),
        py::arg("RING_MIN_ANGLE_COVER"),
        py::arg("RING_CLUSTER_GAP"),
        py::arg("RING_CLUSTER_MIN_HITS"),
        py::arg("CLAHE_CLIP"),
        py::arg("CLAHE_GRID_W"), py::arg("CLAHE_GRID_H"),
        py::arg("GAUSS_BLUR_K"),
        "孔同心环轮廓检测，所有配置参数由Python传入");
}
