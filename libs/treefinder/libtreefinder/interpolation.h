#pragma once

#include <libimages/images.h>
#include <vector>

namespace treefinder {

const float EMPTY_VAL = -32767.0f;

images::Image<float> create_next_layer(const images::Image<float>& prev); 

std::vector<images::Image<float>> build_pyramid(const images::Image<float>& dem); 

void descend_layer(const images::Image<float>& upper, images::Image<float>& lower);

images::Image<float> interpolate_pyramid(std::vector<images::Image<float>> pyramid);

images::Image<float> build_chm(const images::Image<float>& dem, float offset, int xblocksize, int yblocksize);

} // namespace end
