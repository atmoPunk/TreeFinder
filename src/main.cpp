#include <libimages/images.h>
#include <libtreefinder/treefinder.h>

#include <vector>
#include <limits>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/imgproc.hpp>


int main(int argc, char **argv) {
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

    const float DEM_EMPTY_VALUE = -32767.0f;

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
