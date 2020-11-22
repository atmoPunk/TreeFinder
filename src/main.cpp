#include <libimages/images.h>
#include <libtreefinder/treefinder.h>
#include <libtreefinder/interpolation.h>
#include <libtreefinder/tree_deliniation.h>

#include <vector>
#include <limits>
#include <iostream>
#include <fstream>
#include <algorithm>
q// #include <opencv2/imgproc.hpp>
#include <cmath>

struct Color {
    int red;
    int green;
    int blue;
};

template <typename T>
void plotLineLow(int64_t x0, int64_t y0, int64_t x1, int64_t y1, images::Image<T>& image, const Color& color) {
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
        image(y, x, 0) = color.red;
        image(y, x, 1) = color.green;
        image(y, x, 2) = color.blue;
        if (D > 0) {
            y = y + yi;
            D = D + (2 * (dy - dx));
        } else {
            D = D + 2 * dy;
        }
    }
}

template <typename T>
void plotLineHigh(int64_t x0, int64_t y0, int64_t x1, int64_t y1, images::Image<T>& image, const Color& color) {
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
        image(y, x, 0) = color.red;
        image(y, x, 1) = color.green;
        image(y, x, 2) = color.blue;
        if (D > 0) {
            x = x + xi;
            D = D + (2 * (dx - dy));
        } else {
            D = D + 2 * dx;
        }
    }
}

template <typename T>
void plotLine(int64_t x0, int64_t y0, int64_t x1, int64_t y1, images::Image<T>& image, const Color& color) {
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

int main(int argc, char **argv) {

    std::cout << argv[2] << ' ' << argv[3] << std::endl;
    const float DEM_EMPTY_VALUE = -32767.0f;

    int tree_count = 0;
    std::cout << "Enter number of trees to find: ";
    std::cin >> tree_count;

    images::Image<float> chm(argv[1]);
    treefinder::TreeDeliniator td(chm, 5);
    td.eliminateSmallValues(1.0);
    std::vector<Color> colors;
    colors.push_back({250 << 8, 40 << 8, 40 << 8});
    colors.push_back({40 << 8, 250 << 8, 40 << 8});
    colors.push_back({40 << 8, 40 << 8, 250 << 8});
    colors.push_back({190 << 8, 23 << 8, 209 << 8});
    colors.push_back({209 << 8, 199 << 8, 23 << 8});
    colors.push_back({247 << 8, 131 << 8, 0 << 8});
    int cur_color = 0;
    auto chm2 = td.chm.copy();
    images::Image<unsigned short> ortho(argv[2]);
    for (int i = 0; i < tree_count; ++i) {
        auto mx = td.findGlobalMaxima();
        auto crown = td.findTreeCrown(mx);
        for (auto it = ++crown.outer().begin(); it != crown.outer().end(); ++it) {
            auto prev = it;
            --prev;
            plotLine(boost::geometry::get<0>(*prev), boost::geometry::get<1>(*prev), boost::geometry::get<0>(*it), boost::geometry::get<1>(*it), ortho, colors[0]);
        }
//        for (int64_t x = 0; x < std::min(ortho.width, chm.width); ++x) {
//            for (int64_t y = 0; y < std::min(ortho.height, chm.height); ++y) {
//                if (td.chm(y, x) != chm2(y, x) && td.chm(y, x) == DEM_EMPTY_VALUE) {
//                    ortho(y, x, 0) = colors[0].red;
//                    ortho(y, x, 1) = colors[0].green;
//                    ortho(y, x, 2) = colors[0].blue;
//                }
//            }
//        }
        std::cout << "iter: " << i << std::endl;
    	std::cout << boost::geometry::dsv(td.tree_crowns[i]) << std::endl;
        cur_color = (cur_color + 1) % colors.size();
    }

    std::cout << ortho.height << " " << ortho.width << std::endl;
    std::cout << chm.height << " " << chm.width << std::endl;


    ortho.saveJPEG(argv[3]);

    return 0;

    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " PATH_TO_DEM PATH_TO_ORTHOIMAGE OUTPUT_IMAGE OUTPUT_TREES" << std::endl;
        return 1;
    }

    images::Image<float> dem(argv[1]);
    std::cout << "DEM loaded: " << dem.width << "x" << dem.height << std::endl; // DEM = Digital Elevation Model = карта высот (в каждом пикселе высота в метрах)
    double resolution = 0.25; // ширина пикселя в метрах - разрешение DEM-а
    std::cout << "DEM size: " << dem.width * resolution << " m x " << dem.height * resolution << " m" << std::endl;

    images::Image<unsigned char> orthoimage(argv[2]);
    std::cout << "orthoimage loaded: " << orthoimage.width << "x" << orthoimage.height << std::endl;
    if (dem.width != orthoimage.width || dem.height != orthoimage.height) {
        std::cerr << "Resolution mismatch!" << std::endl;
        return 1; // ортоизображение и DEM соотносятся пиксель в пиксель
    }


    std::vector<float> heights;
    for (size_t j = 0; j < dem.height; ++j) {
        for (size_t i = 0; i < dem.width; ++i) {
            float height = dem(j, i); // так можно считать значение высоты в метрах в j-ой строчке, i-ой колонке
            if (height != DEM_EMPTY_VALUE) {
                heights.push_back(height);
            }
            // таким же образом можно менять высоту если это зачем-то нужно:
            // dem(j, i) = 0.0f;
        }
    }
    std::sort(heights.begin(), heights.end());
    std::cout << "DEM non-empty pixels: " << (heights.size() * 100 / dem.width / dem.height) << "%" << std::endl;
    std::cout << "DEM heights min/1%/50%/99%/max: " 
              << heights[0] << " m, "
              << heights[heights.size()*1/100] << " m, "
              << heights[heights.size()*50/100] <<" m, "
              << heights[heights.size()*99/100] <<" m, "
              << heights[heights.size()-1] << " m" << std::endl;


    

    treefinder::TreeFinder tf(dem, resolution);
    auto trees = tf.find_trees(2, 7, {6, 6}, {1.2, 1.2}, 5);

    std::cout << "Trees: " << trees.size() << std::endl;
    treefinder::util::Color red = { 255, 10, 10 };
    for (const auto& tree : trees) {
        treefinder::util::draw_mark(orthoimage, tree.first, tree.second, red, 2);
    }

    orthoimage.saveJPEG(argv[3]);
    std::ofstream trees_file(argv[4]);
    if (trees_file) {
        for (const auto& tree : trees) {
            trees_file << tree.first << " " << tree.second << "\n";
        }
    }

    return 0;
}
