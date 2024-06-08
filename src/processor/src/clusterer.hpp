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

#ifndef CLUSTERER_HPP
#define CLUSTERER_HPP

// Internal
#include "circular_queue.hpp"

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <utility>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace clustering
{

using ClusterLabel = std::int32_t;

struct SphericalPoint
{
    float range_m;
    float azimuth_rad;   // Convention: 0 -> 2 * pi
    float elevation_rad; // Convention: 0 -> pi
};

struct Configuration
{
    float voxel_grid_range_resolution_m_ = 1.0F;
    float voxel_grid_azimuth_resolution_deg_ = 1.0F;
    float voxel_grid_elevation_resolution_deg_ = 1.0F;
};

class Clusterer
{
  public:
    static constexpr float TWO_M_PIf = static_cast<float>(2.0 * M_PI);
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0);

    Clusterer();

    void cluster(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<ClusterLabel>& labels);

  private:
    Configuration config_;

    float voxel_grid_range_resolution_m_;
    float voxel_grid_azimuth_resolution_rad_;
    float voxel_grid_elevation_resolution_rad_;

    std::vector<SphericalPoint> spherical_cloud_;

    std::uint16_t num_range_;
    std::uint16_t num_azimuth_;
    std::uint16_t num_elevation_;

    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> hash_table_;

    inline std::uint32_t flatVoxelIndex(std::uint16_t range_index,
                                        std::uint16_t azimuth_index,
                                        std::uint16_t elevation_index) const noexcept
    {
        return static_cast<std::uint32_t>(
            num_range_ * (num_azimuth_ * elevation_index + azimuth_index) + range_index);
    }

    inline std::uint16_t rangeToIndex(float range_m) const noexcept
    {
        return static_cast<std::uint16_t>(range_m / voxel_grid_range_resolution_m_);
    }

    inline std::uint16_t azimuthToIndex(float azimuth_rad) const noexcept
    {
        return static_cast<std::uint16_t>(azimuth_rad / voxel_grid_azimuth_resolution_rad_);
    }

    inline std::uint16_t elevationToIndex(float elevation_rad) const noexcept
    {
        return static_cast<std::uint16_t>(elevation_rad / voxel_grid_elevation_resolution_rad_);
    }

    void cartesianToSpherical(const pcl::PointCloud<pcl::PointXYZ>& cartesian_cloud,
                              std::vector<SphericalPoint>& spherical_cloud);

    void buildHashTable(const std::vector<SphericalPoint>& spherical_cloud,
                        std::unordered_map<std::uint32_t, std::vector<std::uint32_t>>& hash_table);
};
} // namespace clustering

#endif // CLUSTERER_HPP
