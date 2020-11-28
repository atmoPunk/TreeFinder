#include "treefinder.h"
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>

namespace treefinder {

using Matrix = std::vector<std::vector<float>>;

TreeFinder::TreeFinder(images::Image<float> depthmap, float resolution, int blur_size)
    : depthmap(std::move(depthmap))
    , resolution(resolution)
{
    if (blur_size == -1) {
        blur_size = static_cast<int>(0.33 / resolution) * 2 + 1;
    }
    auto depthmapCv = cv::Mat(depthmap.height, depthmap.width, CV_32FC1, depthmap.ptr());
    cv::GaussianBlur(depthmapCv, depthmapCv, { blur_size, blur_size }, 0, 0);
}

std::vector<std::pair<size_t, size_t>> TreeFinder::find_trees(size_t maxima_radius, size_t search_radius, ParaboloidConfig upper, ParaboloidConfig lower, float lower_offset) {
    size_t search_diameter = search_radius * 2 + 1;
    Matrix upper_bound = generate_neg_paraboloid(upper.a, upper.b, resolution, search_diameter);
    Matrix lower_bound = generate_neg_paraboloid(lower.a, lower.b, resolution, search_diameter);
    for (size_t i = 0; i < search_diameter; ++i) {
        for (size_t j = 0; j < search_diameter; ++j) {
            lower_bound[i][j] -= lower_offset;
        }
    }

    std::vector<std::pair<size_t, size_t>> trees;

    auto local_maxima = find_local_maxima(maxima_radius);
    for (const auto& maximum : local_maxima) {
        try {
            auto shape = subtract_mid(depthmap, maximum.first, maximum.second, search_radius);
            if (!matrix_ge(upper_bound, shape)) {
                continue;
            }
            if (!matrix_ge(shape, lower_bound)) {
                continue;
            }
            trees.push_back(maximum);
        } catch (std::exception& e) {
            continue;
        }

    }
    return trees;
}

template <typename T>
void util::draw_mark(::images::Image<T>& image, int64_t row, int64_t col, Color color, size_t size) {
    for (size_t i = std::max<int64_t>(row - size, 0), iEnd = std::min<size_t>(row + size + 1, image.height); i < iEnd; ++i) {
        for (size_t j = std::max<int64_t>(col - size, 0), jEnd = std::min<size_t>(col + size + 1, image.width); j < jEnd; ++j) {
            image(i, j, 0) = color.red;
            image(i, j, 1) = color.green;
            image(i, j, 2) = color.blue;
        }
    }
}

Matrix TreeFinder::generate_neg_paraboloid(float a, float b, float resolution, size_t steps) {
    Matrix m(steps, std::vector<float>(steps));
    float x_start = 0.0 - (resolution * (steps - 1) / 2);
    float y_start = 0.0 - (resolution * (steps - 1) / 2);
    for (size_t row = 0; row < steps; ++row) {
        float cur_y = y_start + resolution * row;
        for (size_t col = 0; col < steps; ++col) {
            float cur_x = x_start + resolution * col;
            m[row][col] = - (cur_x * cur_x / (a * a)) - (cur_y * cur_y / (b * b));
        }
    }
    return m;
}

Matrix TreeFinder::subtract_mid(const images::Image<float>& depthmap, size_t row, size_t col, size_t rad) {
    if (row < rad || col < rad || row + rad + 1 > depthmap.height || col + rad + 1 > depthmap.width) {
        std::stringstream err;
        err << "subtract_mid is out of range with center: (" << row << ", " << col << "), "
             << "rad: " << rad << ", image size: (" << depthmap.height << ", " << depthmap.width << ")";
        throw std::out_of_range(err.str()); 
    }
    Matrix m(2 * rad + 1, std::vector<float>(2 * rad + 1));
    for (size_t r = row - rad; r < row + rad + 1; ++r) {
        for (size_t c = col - rad; c < col + rad + 1; ++c) {
            m[r - (row - rad)][c - (col - rad)] = depthmap(r, c) - depthmap(row, col);
        }
    }
    return m;
}

bool TreeFinder::is_local_maxima(size_t row, size_t col, size_t size) const {
    if (depthmap(row, col) < -30000.f) {
        return false;
    }
    bool ok = true;
    for (size_t r = row - size; r < row + size + 1; ++r) { // r and c here are always in bounds
        for (size_t c = col - size; c < col + size + 1; ++c) {
            ok &= depthmap(r, c) <= depthmap(row, col);
        }
    }
    return ok;
}

std::vector<std::pair<size_t, size_t>> TreeFinder::find_local_maxima(size_t size) const {
    std::vector<std::pair<size_t, size_t>> result;
    for (size_t i = size; i < depthmap.height - size; ++i) {
        for (size_t j = size; j < depthmap.width - size; ++j) {
            if (is_local_maxima(i, j, size)) {
                result.emplace_back(i, j);
            }
        }
    }
    return result;
}

bool TreeFinder::matrix_ge(const Matrix& lhs, const Matrix& rhs) {
    for (size_t i = 0, n = lhs.size(); i < n; ++i) {
        for (size_t j = 0, m = lhs[i].size(); j < m; ++j) {
            if (lhs[i][j] < rhs[i][j]) {
                return false;
            }
        }
    }
    return true;
}

template void util::draw_mark(::images::Image<unsigned char> &image, int64_t row, int64_t col, Color color, size_t size);
template void util::draw_mark(::images::Image<unsigned short> &image, int64_t row, int64_t col, Color color, size_t size);

}


