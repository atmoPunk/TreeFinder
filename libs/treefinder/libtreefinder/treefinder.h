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

        TreeFinder(images::Image<float> depthmap, float resolution, int blur_size = -1);

        std::vector<std::pair<size_t, size_t>> find_trees(size_t maxima_radius, size_t search_radius, ParaboloidConfig upper, ParaboloidConfig lower, float lower_offset);

    private:
        images::Image<float> depthmap;
        float resolution;

        static std::vector<std::vector<float>> generate_neg_paraboloid(float a, float b, float resolution, size_t steps);
        static std::vector<std::vector<float>> subtract_mid(const images::Image<float>& depthmap, size_t row, size_t col, size_t rad);
    
        bool is_local_maxima(size_t row, size_t col, size_t size) const;
    
        std::vector<std::pair<size_t, size_t>> find_local_maxima(size_t size) const;

        static bool matrix_ge(const std::vector<std::vector<float>>& lhs, const std::vector<std::vector<float>>& rhs);
    };

    namespace util {

        struct Color {
            int red;
            int green;
            int blue;
        };

        template <typename T>
        void draw_mark(images::Image<T>& image, int64_t row, int64_t col, Color color, size_t size);

    }
}
