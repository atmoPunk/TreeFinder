#pragma once

#include <libimages/images.h>
#include <cstdint>
#include <algorithm>
#include <boost/geometry.hpp>
#include <vector>

namespace bg = boost::geometry;

namespace {
    const float EMPTY_VAL = -32767.0f;
}

namespace treefinder {
    using Point = bg::model::d2::point_xy<size_t>;
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

            return Point(x, y);
        }
        
        void findTreeCrown(Point treetop) {
        //bg::model::polygon<Point> findTreeCrown(Point treetop) const {
            // vertical profile
            const size_t max_distance = 15; // in pixels for now, TODO: calculate from resolution
            
            Point p1;
            {
                int64_t min_bound = -1;
                for (int64_t i = treetop.y(); i > std::max<uint64_t>(static_cast<uint64_t>(treetop.y()) - max_distance - 1, 0); --i) {
                    if (chm(i, treetop.x()) == EMPTY_VAL) {
                        min_bound = i + 1;
                        break;
                    }
                }
                float v = chm(treetop.y(), treetop.x());
                int64_t min_y = -1;
                for (int64_t i = treetop.y(); i > std::max<uint64_t>(static_cast<uint64_t>(treetop.y()) - max_distance - 1, 0); --i) {
                    if (chm(i, treetop.x()) != EMPTY_VAL && chm(i, treetop.x()) < v) {
                        v = chm(i, treetop.x());
                        min_y = i;
                    }
                }
                p1 = Point(treetop.x(), std::max(min_bound, min_y));
            }

            Point p2;
            {
                int64_t min_bound = 10000;
                for (int64_t i = treetop.y(); i < std::min<uint64_t>(static_cast<uint64_t>(treetop.y()) + max_distance + 1, chm.height); ++i) {
                    if (chm(i, treetop.x()) == EMPTY_VAL) {
                        min_bound = i - 1;
                        break;
                    }
                }
                float v = chm(treetop.y(), treetop.x());
                int64_t min_y = 10000;
                for (int64_t i = treetop.y(); i < std::min<uint64_t>(static_cast<uint64_t>(treetop.y()) + max_distance + 1, chm.height); ++i) {
                    if (chm(i, treetop.x()) != EMPTY_VAL && chm(i, treetop.x()) < v) {
                        v = chm(i, treetop.x());
                        min_y = i;
                    }
                }
                p2 = Point(treetop.x(), std::min(min_bound, min_y));
            }

            Point p3;
            {
                int64_t min_bound = -1;
                for (int64_t i = treetop.x(); i > std::max<uint64_t>(static_cast<uint64_t>(treetop.x()) - max_distance - 1, 0); --i) {
                    if (chm(treetop.y(), i) == EMPTY_VAL) {
                        min_bound = i + 1;
                        break;
                    }
                }
                float v = chm(treetop.y(), treetop.x());
                int64_t min_y = -1;
                for (int64_t i = treetop.x(); i > std::max<uint64_t>(static_cast<uint64_t>(treetop.x()) - max_distance - 1, 0); --i) {
                    if (chm(treetop.y(), i) != EMPTY_VAL && chm(treetop.y(), i) < v) {
                        v = chm(treetop.y(), i);
                        min_y = i;
                    }
                }
                p3 = Point(std::max(min_bound, min_y), treetop.y());
            }

            Point p4;
            {
                int64_t min_bound = 10000;
                for (int64_t i = treetop.x(); i < std::min<uint64_t>(static_cast<uint64_t>(treetop.x()) + max_distance + 1, chm.width); ++i) {
                    if (chm(treetop.y(), i) == EMPTY_VAL) {
                        min_bound = i - 1;
                        break;
                    }
                }
                float v = chm(treetop.y(), treetop.x());
                int64_t min_y = 10000;
                for (int64_t i = treetop.x(); i < std::min<uint64_t>(static_cast<uint64_t>(treetop.x()) + max_distance + 1, chm.width); ++i) {
                    if (chm(treetop.y(), i) != EMPTY_VAL && chm(treetop.y(), i) < v) {
                        v = chm(treetop.y(), i);
                        min_y = i;
                    }
                }
                p4 = Point(std::min(min_bound, min_y), treetop.y());
            }

            auto p1f = bg::model::d2::point_xy<float>(p1.x(), p1.y());
            auto p2f = bg::model::d2::point_xy<float>(p2.x(), p2.y());
            auto p3f = bg::model::d2::point_xy<float>(p3.x(), p3.y());
            auto p4f = bg::model::d2::point_xy<float>(p4.x(), p4.y());
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
                        chm(j, i) = EMPTY_VAL;
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
