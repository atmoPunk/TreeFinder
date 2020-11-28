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

        void eliminateSmallValues(float boundary);

        [[nodiscard]] Point findGlobalMaxima() const;

        double getMedianSlope(
                int64_t start_x,
                int64_t start_y,
                const std::function<int64_t(int64_t)>& dir_x,
                const std::function<int64_t(int64_t)>& dir_y,
                int64_t distance,
                bool abs = false
        ) const;

        int64_t getWindowSize(
                int64_t max_x,
                int64_t max_y,
                int64_t start_x,
                int64_t start_y,
                const std::function<int64_t(int64_t)>& dir_x,
                const std::function<int64_t(int64_t)>& dir_y
        ) const;

        int64_t getPoint(int64_t start_x, int64_t start_y, const std::function<int64_t(int64_t)>& dir_x, const std::function<int64_t(int64_t)>& dir_y, int64_t distance) const;

        bg::model::polygon<Point> findTreeCrown(Point treetop);

    // private:
        images::Image<float> chm;
        std::vector<bg::model::polygon<Point>> tree_crowns;
        float resolution; // how many meters in one pixel
    };
}
