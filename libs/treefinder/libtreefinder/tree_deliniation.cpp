#include "tree_deliniation.h"
#include <opencv2/imgproc.hpp>

bg::model::polygon<treefinder::Point> treefinder::TreeDeliniator::findTreeCrown(treefinder::Point treetop) {
    const size_t max_distance = 30; // in pixels for now, TODO: calculate from resolution

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
    auto p1i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p1.x()),     static_cast<int64_t>(p1.y()) - 1);
    auto p2i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p2.x()) - 1, static_cast<int64_t>(p2.y()) - 1);
    auto p3i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p3.x()) - 1, static_cast<int64_t>(p3.y()));
    auto p4i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p4.x()) - 1, static_cast<int64_t>(p4.y()) + 1);
    auto p5i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p5.x()),     static_cast<int64_t>(p5.y()) + 1);
    auto p6i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p6.x()) + 1, static_cast<int64_t>(p6.y()) + 1);
    auto p7i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p7.x()) + 1, static_cast<int64_t>(p7.y()));
    auto p8i = bg::model::d2::point_xy<int64_t>(static_cast<int64_t>(p8.x()) + 1, static_cast<int64_t>(p8.y()) - 1);

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
    return Point(x, y);
}

treefinder::TreeDeliniator::TreeDeliniator(images::Image<float> chm, float resolution) : chm(chm), resolution(resolution) {
    auto depthmapCv = cv::Mat(chm.height, chm.width, CV_32FC1, chm.ptr());
    cv::GaussianBlur(depthmapCv, depthmapCv, { 5, 5 }, 0, 0);
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
