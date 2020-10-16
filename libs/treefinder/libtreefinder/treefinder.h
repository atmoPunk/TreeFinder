#pragma once

#include <libimages/images.h>
#include <cstdint>
#include <vector>
#include <utility>

namespace treefinder {
    class TreeFinder {
    public:
    
        struct ParaboloidConfig {
            float a;
            float b;
        };

        TreeFinder(images::Image<float> depthmap, float resolution);

        std::vector<std::pair<size_t, size_t>> find_trees(size_t maxima_radius, size_t search_radius, ParaboloidConfig upper, ParaboloidConfig lower, size_t lower_offset);

    private:
        images::Image<float> depthmap;
        float resolution;

        std::vector<std::vector<float>> generate_neg_paraboloid(float a, float b, float resolution, size_t steps);
        std::vector<std::vector<float>> subtract_mid(const images::Image<float> depthmap, size_t row, size_t col, size_t rad);
    
        bool is_local_maxima(const images::Image<float>& depthmap, size_t row, size_t col, size_t size);
    
        std::vector<std::pair<size_t, size_t>> find_local_maxima(const images::Image<float>& depthmap, size_t size);

        bool matrix_ge(const std::vector<std::vector<float>>& lhs, const std::vector<std::vector<float>>& rhs);
    };

    namespace util {

        struct Color {
            uint8_t red;
            uint8_t green;
            uint8_t blue;
        };

        void draw_mark(images::Image<unsigned char>& image, int64_t row, int64_t col, Color color, size_t size);

    }
}
