/*
 * Copyright (c) 2024 Yevgeniy Simonov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "noise_remover.hpp"

// STL
#include <chrono>
#include <cmath>
#include <iostream>

namespace lidar_processing_lib
{
NoiseRemover::NoiseRemover() : kdtree_(false)
{
    kdtree_.reserve(200'000U);
    neighbours_.reserve(200'000U);
}

void NoiseRemover::filter(const std::vector<PointT>& points, std::vector<NoiseRemoverLabel>& labels)
{
    labels.assign(points.size(), NoiseRemoverLabel::VALID);

    kdtree_.rebuild(points);

    const std::uint32_t min_neighbours = config_.min_neighbours;

    const double scaling_factor =
        std::pow(static_cast<double>(config_.radius_multiplier_m_per_m), 2.0);

    const float min_search_radius_m_sqr = config_.min_search_radius_m * config_.min_search_radius_m;

    for (std::uint32_t point_index = 0; point_index < points.size(); ++point_index)
    {
        const auto& point = points[point_index];

        // R = scaling * (distance * azimuth)
        // R^2 = (scaling * distance * azimuth)^2
        const double range_m_sqr = (point[0] * point[0]) + (point[1] * point[1]);
        const float dynamic_range_m_sqr =
            std::max(static_cast<float>(scaling_factor * range_m_sqr), min_search_radius_m_sqr);

        kdtree_.radius_search_k_nearest(point, dynamic_range_m_sqr, min_neighbours, neighbours_);

        if (neighbours_.size() < min_neighbours)
        {
            labels[point_index] = NoiseRemoverLabel::NOISE;
        }
    }
}
} // namespace lidar_processing_lib
