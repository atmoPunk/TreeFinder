#pragma once

#include <libimages/images.h>
#include <cstdint>
#include <algorithm>
#include <boost/geometry.hpp>
#include <vector>
#include <functional>
#include <iostream>

namespace bg = boost::geometry;

namespace {
    const float EMPTY_VAL = -32767.0f;
}

namespace treefinder {
    using Point = bg::model::d2::point_xy<size_t>;
    int64_t inc_dir(int64_t i) {
        return i;
    }

    int64_t dec_dir(int64_t i) {
        return -i;
    }

    int64_t nul_dir(int64_t i) {
        return 0;
    }

    class TreeDeliniator {

    public:
        TreeDeliniator(images::Image<float> chm, float resolution) : chm(chm), resolution(resolution) {}

        void eliminateSmallValues(float boundary) {
            for (size_t i = 0; i < chm.height; ++i) {
                for (size_t j = 0; j < chm.width; ++j) {
                    if (chm(i, j) <= boundary) {
                        chm(i, j) = EMPTY_VAL;
                    }
                }
            }
        }

        //void smooth()
       
        bg::model::d2::point_xy<size_t> findGlobalMaxima() const {
            float max = EMPTY_VAL;
            size_t x = -1;
            size_t y = -1;
            for (size_t i = 0; i < chm.height; ++i) {
                for (size_t j = 0; j < chm.width; ++j) {
                    if (chm(i, j) > max) {
                        max = chm(i, j);
                        x = j;
                        y = i;
                    }
                }
            }
            // std::cout << chm(y, x) << std::endl;
            // std::cout << "max " << x << " " << y << std::endl;
            return Point(x, y);
        }

        int64_t getPoint(int64_t start_x, int64_t start_y, std::function<int64_t(int64_t)> dir_x, std::function<int64_t(int64_t)> dir_y, int64_t distance) {
            float value = chm(start_y, start_x);
            int64_t cur_dist = 0;
            for (int64_t i = 1; i <= distance; ++i) {
                if (int64_t cur_y = start_y + dir_y(i); cur_y < 0 || cur_y >= chm.height) {
                    break;
                }
                if (int64_t cur_x = start_x + dir_x(i); cur_x < 0 || cur_x >= chm.width) {
                    break;
                }
                if (chm(start_y + dir_y(i), start_x + dir_x(i)) < value) {
                    cur_dist = i;
                    value = chm(start_y + dir_y(i), start_x + dir_x(i));
                }
            }
            return cur_dist;
        }
        
        void findTreeCrown(Point treetop) {
        //bg::model::polygon<Point> findTreeCrown(Point treetop) const {
            const size_t max_distance = 15; // in pixels for now, TODO: calculate from resolution
            
            Point p1(treetop.x(), treetop.y() + dec_dir(getPoint(treetop.x(), treetop.y(), nul_dir, dec_dir, max_distance)));
            Point p2(treetop.x() + dec_dir(getPoint(treetop.x(), treetop.y(), dec_dir, nul_dir, max_distance)), treetop.y());
            Point p3(treetop.x(), treetop.y() + inc_dir(getPoint(treetop.x(), treetop.y(), nul_dir, inc_dir, max_distance)));
            Point p4(treetop.x() + inc_dir(getPoint(treetop.x(), treetop.y(), inc_dir, nul_dir, max_distance)), treetop.y());
            

            auto p1f = bg::model::d2::point_xy<float>(p1.x(), p1.y() - 1);
            auto p2f = bg::model::d2::point_xy<float>(p2.x() - 1, p2.y());
            auto p3f = bg::model::d2::point_xy<float>(p3.x(), p3.y() + 1);
            auto p4f = bg::model::d2::point_xy<float>(p4.x() + 1, p4.y());

            bg::model::polygon<bg::model::d2::point_xy<float>> poly;
            bg::append(poly, p1f);
            bg::append(poly, p2f);
            bg::append(poly, p3f);
            bg::append(poly, p4f);

            bg::model::polygon<bg::model::d2::point_xy<float>> hull;
            bg::convex_hull(poly, hull);
            for (int64_t i = 0; i < chm.height; ++i) {
                for (int64_t j = 0; j < chm.width; ++j) {
                    auto pt = bg::model::d2::point_xy<float>(j, i);
                    if (bg::within(pt, hull)) {
                        // std::cout << "nulled " << i << " " << j << std::endl;
                        chm(i, j) = EMPTY_VAL;
                    }
                }
            }
            tree_crowns.push_back(hull);
        }

    // private:
        images::Image<float> chm;
        std::vector<bg::model::polygon<bg::model::d2::point_xy<float>>> tree_crowns;
        float resolution;
    };
}
