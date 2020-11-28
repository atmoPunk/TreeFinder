#include "tree_deliniation.h"
#include <opencv2/imgproc.hpp>

bg::model::polygon<treefinder::Point> treefinder::TreeDeliniator::findTreeCrown(treefinder::Point treetop) {
    const size_t max_distance = 5 / resolution; // 5m to pixels

    Point p1(treetop.x(), treetop.y() + dec_dir(getPoint(treetop.x(), treetop.y(), nul_dir, dec_dir, max_distance)));
    auto p2_point = getPoint(treetop.x(), treetop.y(), dec_dir, dec_dir, max_distance * 3 / 5);
    Point p2(treetop.x() + dec_dir(p2_point), treetop.y() + dec_dir(p2_point));
    Point p3(treetop.x() + dec_dir(getPoint(treetop.x(), treetop.y(), dec_dir, nul_dir, max_distance)), treetop.y());
    auto p4_point = getPoint(treetop.x(), treetop.y(), dec_dir, inc_dir, max_distance * 3 / 5);
    Point p4(treetop.x() + dec_dir(p4_point), treetop.y() + inc_dir(p4_point));
    Point p5(treetop.x(), treetop.y() + inc_dir(getPoint(treetop.x(), treetop.y(), nul_dir, inc_dir, max_distance)));
    auto p6_point = getPoint(treetop.x(), treetop.y(), inc_dir, inc_dir, max_distance * 3 / 5);
    Point p6(treetop.x() + inc_dir(p6_point), treetop.y() + inc_dir(p6_point));
    Point p7(treetop.x() + inc_dir(getPoint(treetop.x(), treetop.y(), inc_dir, nul_dir, max_distance)), treetop.y());
    auto p8_point = getPoint(treetop.x(), treetop.y(), inc_dir, dec_dir, max_distance * 3 / 5);
    Point p8(treetop.x() + inc_dir(p8_point), treetop.y() + dec_dir(p8_point));

    // these +-1 are needed. Else it will stuck
    int add = 1;
    auto p1i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p1.x()),     static_cast<int64_t>(p1.y()) - add);
    auto p2i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p2.x()) - add, static_cast<int64_t>(p2.y()) - add);
    auto p3i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p3.x()) - add, static_cast<int64_t>(p3.y()));
    auto p4i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p4.x()) - add, static_cast<int64_t>(p4.y()) + add);
    auto p5i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p5.x()),     static_cast<int64_t>(p5.y()) + add);
    auto p6i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p6.x()) + add, static_cast<int64_t>(p6.y()) + add);
    auto p7i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p7.x()) + add, static_cast<int64_t>(p7.y()));
    auto p8i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p8.x()) + add, static_cast<int64_t>(p8.y()) - add);

    int64_t min_x = std::min(bg::get<0>(p2i), std::min(bg::get<0>(p3i), bg::get<0>(p4i)));
    int64_t max_x = std::max(bg::get<0>(p6i), std::max(bg::get<0>(p7i), bg::get<0>(p8i)));
    int64_t min_y = std::min(bg::get<1>(p1i), std::min(bg::get<1>(p2i), bg::get<1>(p8i)));
    int64_t max_y = std::max(bg::get<1>(p4i), std::max(bg::get<1>(p5i), bg::get<1>(p6i)));

    bg::model::polygon<bg::model::d2::point_xy<int64_t>> ret_poly;
    bg::append(ret_poly, p1i);
    bg::append(ret_poly, p2i);
    bg::append(ret_poly, p3i);
    bg::append(ret_poly, p4i);
    bg::append(ret_poly, p5i);
    bg::append(ret_poly, p6i);
    bg::append(ret_poly, p7i);
    bg::append(ret_poly, p8i);

    bg::model::polygon<bg::model::d2::point_xy<int64_t>> ret_hull;
    bg::convex_hull(ret_poly, ret_hull);

    for (int64_t i = std::max<int64_t>(min_y, 0); i <= std::min<int64_t>(max_y, chm.height - 1); ++i) {
        for (int64_t j = std::max<int64_t>(min_x, 0); j <= std::min<int64_t>(max_x, chm.width - 1); ++j) {
            auto pt = bg::model::d2::point_xy<int64_t>(j, i);
            if (bg::within(pt, ret_hull)) {
                chm(i, j) = EMPTY_VAL;
            }
        }
    }
    tree_crowns.push_back(ret_hull);
    return ret_hull;
}

treefinder::Point treefinder::TreeDeliniator::findGlobalMaxima() const {
    float max = EMPTY_VAL;
    int64_t x = -1;
    int64_t y = -1;
    for (int64_t i = 0; i < chm.height; ++i) {
        for (int64_t j = 0; j < chm.width; ++j) {
            if (chm(i, j) > max) {
                max = chm(i, j);
                x = j;
                y = i;
            }
        }
    }
    if (max < EMPTY_BOUND) {
        throw std::runtime_error("Maximas are finished");
    }
return Point(x, y);
}

treefinder::TreeDeliniator::TreeDeliniator(images::Image<float> chm, float resolution) : chm(chm), resolution(resolution) {
    auto depthmapCv = cv::Mat(chm.height, chm.width, CV_32FC1, chm.ptr());
    int blur_size = static_cast<int>(0.5 / resolution) * 2 + 1;
    cv::GaussianBlur(depthmapCv, depthmapCv, { blur_size, blur_size }, 0, 0);
}

void treefinder::TreeDeliniator::eliminateSmallValues(float boundary) {
    for (size_t i = 0; i < chm.height; ++i) {
        for (size_t j = 0; j < chm.width; ++j) {
            if (chm(i, j) <= boundary) {
                chm(i, j) = EMPTY_VAL;
            }
        }
    }
}

double treefinder::TreeDeliniator::getMedianSlope(int64_t start_x, int64_t start_y,
                                                  const std::function<int64_t(int64_t)> &dir_x,
                                                  const std::function<int64_t(int64_t)> &dir_y, int64_t distance,
                                                  bool abs) const {
    std::vector<double> slopes;
    for (int64_t j = 0; j < distance; ++j) {
        if (start_y + dir_y(j + 1) >= chm.height || start_x + dir_x(j + 1) >= chm.width) {
            continue;
        }
        if (start_y + dir_y(j + 1) < 0 || start_x + dir_x(j + 1) < 0) {
            continue;
        }
        double slope_radian = std::atan((-chm(start_y + dir_y(j), start_x + dir_x(j)) + chm(start_y + dir_y(j + 1), start_x + dir_x(j + 1))) / resolution);
        slopes.push_back(abs ? std::abs(slope_radian * 180.0 * M_1_PI) : slope_radian * 180.0 * M_1_PI);
    }
    std::sort(slopes.begin(), slopes.end());
    if (slopes.empty()) {
        return 0.0;
    }
    double median = slopes[slopes.size() / 2];
    return median;
}

int64_t treefinder::TreeDeliniator::getWindowSize(int64_t max_x, int64_t max_y, int64_t start_x, int64_t start_y,
                                                  const std::function<int64_t(int64_t)> &dir_x,
                                                  const std::function<int64_t(int64_t)> &dir_y) const {
    double max_val = chm(max_y, max_x);
    double min_val = chm(start_y, start_x);
    // TODO: remove MAGIC NUMBERS (they are from the article)
    double steepRight = std::atan(getMedianSlope(start_x, start_y, dir_x, dir_y, static_cast<int>(std::round(1.5 / resolution)), true));
    double otherHeight = (max_val + min_val) / 2.0;
    double crown_ratio_cone = 0.8;
    double eps = 5;
    double o_c = 2.0 / 3.0;
    double crown_radius_cone = (otherHeight * crown_ratio_cone) / (std::tan((90 - eps) * M_PI / 180.0)) * o_c;
    double crown_ratio_sphere = 0.7;
    double o_s = 1.0 / 3.0;
    double crown_radius_sphere = (otherHeight * crown_ratio_sphere) / 2.0 * o_s;
    double wrd = crown_radius_cone * (1.0 - (90 - eps - steepRight) / (90 - eps - 32.7))
                 + crown_radius_sphere * ((90 - eps - steepRight) / (90 - eps - 32.7));
    return static_cast<int64_t>(std::round(wrd / resolution));
}

int64_t
treefinder::TreeDeliniator::getPoint(int64_t start_x, int64_t start_y, const std::function<int64_t(int64_t)> &dir_x,
                                     const std::function<int64_t(int64_t)> &dir_y, int64_t distance) const {
    for (int64_t i = 1; i <= distance; ++i) {
        int64_t cur_y = start_y + dir_y(i);
        int64_t cur_x = start_x + dir_x(i);
        if (cur_y < 0 || cur_y >= chm.height) {
            distance = i - 1;
            break;
        }
        if (cur_x < 0 || cur_x >= chm.width) {
            distance = i - 1;
            break;
        }
        if (chm(cur_y, cur_x) < EMPTY_BOUND) {
            distance = i - 1;
            break;
        }
        // Find fall by its steepness
        auto slope_radian = std::atan((-chm(start_y + dir_y(i - 1), start_x + dir_x(i - 1)) + chm(cur_y, cur_x)) / resolution);
        auto degree = slope_radian * 180.0 * M_1_PI;
        if (degree < -85) {
            distance = i - 1;
            break;
        }
    }

    for (int64_t i = 1; i <= distance - 5; ++i) {
        auto cur = chm(start_y + dir_y(i), start_x + dir_x(i));
        // if local minimum can be boundary between crowns
        if (chm(start_y + dir_y(i - 1), start_x + dir_x(i - 1)) > cur && cur < chm(start_y + dir_y(i + 1), start_x + dir_x(i + 1))) {
            int64_t window_size = getWindowSize(start_x, start_y, start_x + dir_x(i), start_y + dir_y(i), dir_x, dir_y);
            double ls_median = getMedianSlope(start_x, start_y, dir_x, dir_y, i);
            double rs_median = getMedianSlope(start_x + dir_x(i), start_y + dir_y(i), dir_x, dir_y, window_size);
            if (ls_median < 0 && rs_median > 0) {
                return i;
            }
        }
    }
    return distance;
}

int64_t treefinder::inc_dir(int64_t i) {
    return i;
}

int64_t treefinder::dec_dir(int64_t i) {
    return -i;
}

int64_t treefinder::nul_dir(int64_t i) {
    return 0;
}
