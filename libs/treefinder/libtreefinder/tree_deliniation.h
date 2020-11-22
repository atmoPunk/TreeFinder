#pragma once

#include <libimages/images.h>
#include <cstdint>
#include <algorithm>
#include <boost/geometry.hpp>
#include <vector>
#include <functional>
#include <cmath>
#include <iostream>

namespace bg = boost::geometry;

namespace {
    const float EMPTY_VAL = -32767.0f;
    const float EMPTY_BOUND = -30000.0f;
}

namespace treefinder {
    using Point = bg::model::d2::point_xy<int64_t>;
    int64_t inc_dir(int64_t i);

    int64_t dec_dir(int64_t i);

    int64_t nul_dir(int64_t i);

    class TreeDeliniator {

    public:
        TreeDeliniator(images::Image<float> chm, float resolution);

        void eliminateSmallValues(float boundary) {
            for (size_t i = 0; i < chm.height; ++i) {
                for (size_t j = 0; j < chm.width; ++j) {
                    if (chm(i, j) <= boundary) {
                        chm(i, j) = EMPTY_VAL;
                    }
                }
            }
        }

        [[nodiscard]] Point findGlobalMaxima() const;

        template <typename DirFunc>
        double getMedianSlope(int64_t start_x, int64_t start_y, DirFunc dir_x, DirFunc dir_y, int64_t distance, bool abs = false) const {
            std::vector<double> slopes;
            for (int64_t j = 0; j < distance; ++j) {
                double slope_radian = std::atan((-chm(start_y + dir_y(j), start_x + dir_x(j)) + chm(start_y + dir_y(j + 1), start_x + dir_x(j + 1))) / 0.1); // TODO: resolution
                slopes.push_back(abs ? std::abs(slope_radian * 180.0 * M_1_PI) : slope_radian * 180.0 * M_1_PI);
            }
            std::sort(slopes.begin(), slopes.end());
            double median = slopes[slopes.size() / 2];
            return median;
        }

        template <typename DirFunc>
        int64_t getWindowSize(int64_t max_x, int64_t max_y, int64_t start_x, int64_t start_y, DirFunc dir_x, DirFunc dir_y) const {
            double max_val = chm(max_y, max_x);
            double min_val = chm(start_y, start_x);
            double steepRight = std::atan(getMedianSlope(start_x, start_y, dir_x, dir_y, static_cast<int>(std::round(1.5 * resolution)), true));
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
            return static_cast<int64_t>(std::round(wrd * resolution));
        }

        template <typename DirFunc>
        int64_t getPoint(int64_t start_x, int64_t start_y, DirFunc dir_x, DirFunc dir_y, int64_t distance) const {
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
            }

            for (int64_t i = 1; i <= distance - 5; ++i) {
                auto cur = chm(start_y + dir_y(i), start_x + dir_x(i));
                if (chm(start_y + dir_y(i - 1), start_x + dir_x(i - 1)) > cur && cur < chm(start_y + dir_y(i + 1), start_x + dir_x(i + 1))) {
                    // local minimum
                    // was 5
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
        
//        std::vector<std::pair<int64_t, int64_t>> findTreeCrown(Point treetop) {
        bg::model::polygon<Point> findTreeCrown(Point treetop);

    // private:
        images::Image<float> chm;
        std::vector<bg::model::polygon<bg::model::d2::point_xy<int64_t>>> tree_crowns;
        float resolution;
    };
}
