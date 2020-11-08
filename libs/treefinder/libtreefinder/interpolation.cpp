#include "interpolation.h"

namespace treefinder {

images::Image<float> create_next_layer(const images::Image<float>& prev) {
    size_t width = prev.width;
    size_t height = prev.height;
    size_t next_width = (width + 1) / 2;
    size_t next_height = (height + 1) / 2;;
    auto layer =    images::Image<float>(next_width, next_height, 1);
    auto counters = images::Image<unsigned char>(next_width, next_height, 1);
    layer.fill(EMPTY_VAL);
    counters.fill(static_cast<unsigned char>(0));
    for (size_t i = 0; i < width; ++i) {
        for (size_t j = 0; j < height; ++j) {
            if (prev(j, i) != EMPTY_VAL) {
                if (layer(j / 2, i / 2) == EMPTY_VAL) {
                    layer(j / 2, i / 2) = 0.0f;
                }
                layer(j / 2, i / 2) += prev(j, i);
                counters(j / 2, i / 2) += 1;
            }
        }
    }
    for (size_t i = 0; i < next_width; ++i) {
        for (size_t j = 0; j < next_height; ++j) {
            if (counters(j, i) != 0) {
                layer(j, i) /= counters(j, i);
            }
        }
    }
    return layer;
}

std::vector<images::Image<float>> build_pyramid(const images::Image<float>& dem) {
    std::vector<images::Image<float>> pyramid;
    auto cur = dem;
    pyramid.push_back(cur);
    while (cur.height * cur.width > 1) {
        cur = create_next_layer(cur);
        pyramid.push_back(cur);
    }
    return pyramid;
}


void descend_layer(const images::Image<float>& upper, images::Image<float>& lower) {
    size_t width = lower.width;
    size_t height = lower.height;
    for (size_t i = 0; i < width; ++i) {
        for (size_t j = 0; j < height; ++j) {
            if (lower(j, i) == EMPTY_VAL) {
                lower(j, i) = upper(j / 2, i / 2);
            }
        }
    }
}

images::Image<float> interpolate_pyramid(std::vector<images::Image<float>> pyramid) {
    for (size_t layer = pyramid.size(); layer > 1; --layer) {
        descend_layer(pyramid[layer - 1], pyramid[layer - 2]);
    }

    return pyramid[0];
}

}
