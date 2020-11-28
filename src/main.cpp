#include <libimages/images.h>
#include <libtreefinder/treefinder.h>
#include <libtreefinder/interpolation.h>
#include <libtreefinder/tree_deliniation.h>

#include <vector>
#include <iostream>
#include <variant>

#include <boost/program_options.hpp>


template <typename T>
void plotLineLow(int64_t x0, int64_t y0, int64_t x1, int64_t y1, images::Image<T>& image, const treefinder::util::Color& color) {
    int64_t dx = x1 - x0;
    int64_t dy = y1 - y0;
    int64_t yi = 1;
    if (dy < 0) {
        yi = -1;
        dy = -dy;
    }
    int64_t D = (2 * dy) - dx;
    int64_t y = y0;

    for (int64_t x = x0 ; x <= x1; ++x) {
        if (y < image.height && x < image.width) {
            image(y, x, 0) = color.red;
            image(y, x, 1) = color.green;
            image(y, x, 2) = color.blue;
        }
        if (D > 0) {
            y = y + yi;
            D = D + (2 * (dy - dx));
        } else {
            D = D + 2 * dy;
        }
    }
}

template <typename T>
void plotLineHigh(int64_t x0, int64_t y0, int64_t x1, int64_t y1, images::Image<T>& image, const treefinder::util::Color& color) {
    int64_t dx = x1 - x0;
    int64_t dy = y1 - y0;
    int64_t xi = 1;
    if (dx < 0) {
        xi = -1;
        dx = -dx;
    }
    int64_t D = (2 * dx) - dy;
    int64_t x = x0;

    for (int64_t y = y0; y <= y1; ++y) {
        if (y < image.height && x < image.width) {
            image(y, x, 0) = color.red;
            image(y, x, 1) = color.green;
            image(y, x, 2) = color.blue;
        }
        if (D > 0) {
            x = x + xi;
            D = D + (2 * (dx - dy));
        } else {
            D = D + 2 * dx;
        }
    }
}

template <typename T>
void plotLine(int64_t x0, int64_t y0, int64_t x1, int64_t y1, images::Image<T>& image, const treefinder::util::Color& color) {
    if (std::abs(y1 - y0) < std::abs(x1 - x0)) {
        if (x0 > x1) {
            plotLineLow(x1, y1, x0, y0, image, color);
        } else {
            plotLineLow(x0, y0, x1, y1, image, color);
        }
    } else {
        if (y0 > y1) {
            plotLineHigh(x1, y1, x0, y0, image, color);
        } else {
            plotLineHigh(x0, y0, x1, y1, image, color);
        }
    }
}

struct JpegVisitor {
    explicit JpegVisitor(std::string path) : path(std::move(path)) {}

    void operator()(images::Image<unsigned char>& image) {
        image.saveJPEG(path);
    }

    void operator()(images::Image<unsigned short>& image) {
        image.saveJPEG(path);
    }

    std::string path;
};

namespace po = boost::program_options;

int main(int argc, char **argv) {

    std::string dem_path;
    std::string ortho_path;
    std::string output_path;
    float resolution;
    float boundary;
    int color_depth;
    int tree_count;
    float offset;
    int xblocksize;
    int yblocksize;
    int tfblur;
    float min_area;

    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("ortho", po::value<std::string>(&ortho_path), "path to ortho image")
            ("chm", po::value<std::string>(), "path to chm. If not set will be generated from dem")
            ("dem", po::value<std::string>(&dem_path), "path to dem")
            ("resolution", po::value<float>(&resolution), "image resolution")
            ("output", po::value<std::string>(&output_path), "output path")
            ("color-depth", po::value<int>(&color_depth)->default_value(8), "color depth of ortho image")
            ("short-bound", po::value<float>(&boundary)->default_value(1.0), "trees with lower height will be not considered")
            ("tree-count", po::value<int>(&tree_count)->default_value(1'000'000'000), "max number of trees to find")
            ("offset", po::value<float>(&offset)->default_value(0.1), "offset when generating chm")
            ("xbs", po::value<int>(&xblocksize)->default_value(50), "width of block to search local minimum in when generating chm")
            ("ybs", po::value<int>(&yblocksize)->default_value(50), "height of block to search local minimum in when generating chm")
            ("tf-blur", po::value<int>(&tfblur)->default_value(-1), "blur size when finding treetops")
            ("min-area", po::value<float>(&min_area)->default_value(0), "minimal area of a tree crown")
            ("show-all", "show or hide tree centers that were not selected");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 0;
    }

    images::Image<float> dem(dem_path);
    images::Image<float> chm;

    if (vm.count("chm")) {
        chm = images::Image<float>(vm["chm"].as<std::string>());
    } else {
       chm = treefinder::build_chm(dem.copy(), offset, xblocksize, yblocksize);
       std::cout << "chm generated" << std::endl;
    }

    std::variant<images::Image<unsigned char>, images::Image<unsigned short>> ortho;

    if (color_depth == 8) {
        ortho = images::Image<unsigned char>(ortho_path);
    } else {
        ortho = images::Image<unsigned short>(ortho_path);
    }

   treefinder::TreeDeliniator td(chm, resolution);
    treefinder::TreeFinder tf(dem.copy(), resolution, tfblur);
    auto trees = tf.find_trees(2, 8, {5, 5}, {1.5, 1.5}, 3);

    td.eliminateSmallValues(boundary);
    treefinder::util::Color red;
    treefinder::util::Color blue;
    treefinder::util::Color yellow;
    if (color_depth == 8) {
        red = {250, 40, 40};
        blue = {40, 40, 250};
        yellow = {250, 250, 40};
    } else {
        red = {250 << 8, 40 << 8, 40 << 8};
        blue = {40 << 8, 40 << 8, 250 << 8};
        yellow = {250 << 8, 250 << 8, 40 << 8};
    }

    auto new_end = std::partition(trees.begin(), trees.end(), [&](const std::pair<size_t, size_t>& pt) { return td.chm(pt.first, pt.second) > -30'000.0f; });
    if (vm.count("show-all")) {
        for (auto it = new_end; it != trees.end(); ++it) {
            if (color_depth == 8) {
                treefinder::util::draw_mark(std::get<0>(ortho), it->first, it->second, blue, 2);
            } else {
                treefinder::util::draw_mark(std::get<1>(ortho), it->first, it->second, blue, 2);
            }
        }
    }
    trees.erase(new_end, trees.end());
    std::sort(trees.rbegin(), trees.rend(), [&](const std::pair<size_t, size_t>& lhs,
                                                const std::pair<size_t, size_t>& rhs) { return td.chm(lhs.first, lhs.second) < chm(rhs.first, rhs.second); });

    int i = 0;
    for (auto trit = trees.begin(); i < tree_count && trit != trees.end(); ++i, ++trit) {
        auto max_point = *trit;

        if (td.chm(max_point.first, max_point.second) < -30000.0f) {
            std::cout << "skip :" << i << std::endl;
            if (vm.count("show-all")) {
                if (color_depth == 8) {
                    treefinder::util::draw_mark(std::get<0>(ortho), max_point.first, max_point.second, yellow, 2);
                } else {
                    treefinder::util::draw_mark(std::get<1>(ortho), max_point.first, max_point.second, yellow, 2);
                }
            }
            i -= 1;
            continue;
        }
        auto crown = td.findTreeCrown(boost::geometry::model::d2::point_xy<int64_t>(max_point.second, max_point.first));
//        remove trees with a small crown area
        if (boost::geometry::area(crown) * resolution * resolution < min_area) {
            std::cout << "skipped too small" << std::endl;
            i -= 1;
            continue;
        }
        for (auto it = ++crown.outer().begin(); it != crown.outer().end(); ++it) {
            auto prev = it;
            --prev;
            if (color_depth == 8) {
                plotLine(boost::geometry::get<0>(*prev), boost::geometry::get<1>(*prev), boost::geometry::get<0>(*it),
                         boost::geometry::get<1>(*it), std::get<0>(ortho), red);
            } else {
                plotLine(boost::geometry::get<0>(*prev), boost::geometry::get<1>(*prev), boost::geometry::get<0>(*it),
                         boost::geometry::get<1>(*it), std::get<1>(ortho), red);
            }
        }
        if (color_depth == 8) {
            treefinder::util::draw_mark(std::get<0>(ortho), max_point.first, max_point.second, red, 2);
        } else {
            treefinder::util::draw_mark(std::get<1>(ortho), max_point.first, max_point.second, red, 2);
        }
    	std::cout << boost::geometry::dsv(td.tree_crowns.back()) << "\n";
    }

    std::cout << chm.height << " " << chm.width << std::endl;

    std::cout << "total trees: " << i << std::endl;

    std::visit(JpegVisitor(output_path), ortho);
    return 0;
}
