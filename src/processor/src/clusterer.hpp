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

// External
#include "ankerl/unordered_dense.h"

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
#include <unordered_map>
#include <unordered_set>
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
    float voxel_grid_range_resolution_m = 0.3F;
    float voxel_grid_azimuth_resolution_deg = 1.0F;
    float voxel_grid_elevation_resolution_deg = 1.5F;
};

class Clusterer
{
  public:
    static constexpr float TWO_M_PIf = static_cast<float>(2.0 * M_PI);
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0);
    static constexpr std::int32_t INVALID_LABEL = -1;

    Clusterer();

    template <typename PointT>
    void cluster(const pcl::PointCloud<PointT>& cloud, std::vector<ClusterLabel>& labels);

  private:
    // Elevation index should be the primary sorting key.
    // Azimuth index should be the secondary sorting key.
    // Range index should be the tertiary sorting key.

    static constexpr std::int32_t INDEX_OFFSETS[27][3] = {
        {-1, -1, -1}, {-1, 0, -1}, {-1, 1, -1}, {0, -1, -1}, {0, 0, -1},  {0, 1, -1}, {1, -1, -1},
        {1, 0, -1},   {1, 1, -1},  {-1, -1, 0}, {-1, 0, 0},  {-1, 1, 0},  {0, -1, 0}, {0, 0, 0},
        {0, 1, 0},    {1, -1, 0},  {1, 0, 0},   {1, 1, 0},   {-1, -1, 1}, {-1, 0, 1}, {-1, 1, 1},
        {0, -1, 1},   {0, 0, 1},   {0, 1, 1},   {1, -1, 1},  {1, 0, 1},   {1, 1, 1}};

    static constexpr std::int32_t INDEX_OFFSETS_WITHOUT_CORE[26][3] = {
        {-1, -1, -1}, {-1, 0, -1}, {-1, 1, -1}, {0, -1, -1}, {0, 0, -1}, {0, 1, -1}, {1, -1, -1},
        {1, 0, -1},   {1, 1, -1},  {-1, -1, 0}, {-1, 0, 0},  {-1, 1, 0}, {0, -1, 0}, {0, 1, 0},
        {1, -1, 0},   {1, 0, 0},   {1, 1, 0},   {-1, -1, 1}, {-1, 0, 1}, {-1, 1, 1}, {0, -1, 1},
        {0, 0, 1},    {0, 1, 1},   {1, -1, 1},  {1, 0, 1},   {1, 1, 1}};

    struct VoxelKey
    {
        std::int32_t range_index;
        std::int32_t azimuth_index;
        std::int32_t elevation_index;
    };

    Configuration config_;

    float voxel_grid_range_resolution_m_;
    float voxel_grid_azimuth_resolution_rad_;
    float voxel_grid_elevation_resolution_rad_;

    std::int32_t num_range_;
    std::int32_t num_azimuth_;
    std::int32_t num_elevation_;

    std::vector<SphericalPoint> spherical_cloud_;
    std::vector<std::int32_t> voxel_indices_;
    std::vector<VoxelKey> voxel_keys_;

    ankerl::unordered_dense::segmented_map<std::int32_t, ClusterLabel> voxel_labels_;
    ankerl::unordered_dense::segmented_set<std::int32_t> visited_voxels_;

    containers::CircularQueue<VoxelKey, 150'000> voxel_queue_;

    inline std::int32_t flatVoxelIndex(std::int32_t range_index,
                                       std::int32_t azimuth_index,
                                       std::int32_t elevation_index) const noexcept
    {
        return static_cast<std::int32_t>(
            num_range_ * (num_azimuth_ * elevation_index + azimuth_index) + range_index);
    }

    inline std::int32_t rangeToIndex(float range_m) const noexcept
    {
        return static_cast<std::int32_t>(range_m / voxel_grid_range_resolution_m_);
    }

    inline std::int32_t azimuthToIndex(float azimuth_rad) const noexcept
    {
        return static_cast<std::int32_t>(azimuth_rad / voxel_grid_azimuth_resolution_rad_);
    }

    inline std::int32_t elevationToIndex(float elevation_rad) const noexcept
    {
        return static_cast<std::int32_t>(elevation_rad / voxel_grid_elevation_resolution_rad_);
    }

    inline std::int32_t flatVoxelIndex(const SphericalPoint& point) const noexcept
    {
        const std::int32_t range_index = rangeToIndex(point.range_m);
        const std::int32_t azimuth_index = azimuthToIndex(point.azimuth_rad);
        const std::int32_t elevation_index = elevationToIndex(point.elevation_rad);
        const std::int32_t voxel_index =
            flatVoxelIndex(range_index, azimuth_index, elevation_index);

        return voxel_index;
    }

    template <typename PointT>
    void cartesianToSpherical(const pcl::PointCloud<PointT>& cartesian_cloud,
                              std::vector<SphericalPoint>& spherical_cloud);

    void buildHashTable();

    void clusterImpl(std::vector<ClusterLabel>& labels);

    void propagateLabel(ClusterLabel label,
                        const VoxelKey& voxel_index,
                        std::vector<ClusterLabel>& labels);
};

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                        std::vector<ClusterLabel>& labels);

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                        std::vector<ClusterLabel>& labels);

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                        std::vector<ClusterLabel>& labels);

extern template void Clusterer::cartesianToSpherical(
    const pcl::PointCloud<pcl::PointXYZ>& cartesian_cloud,
    std::vector<SphericalPoint>& spherical_cloud);

extern template void Clusterer::cartesianToSpherical(
    const pcl::PointCloud<pcl::PointXYZI>& cartesian_cloud,
    std::vector<SphericalPoint>& spherical_cloud);

extern template void Clusterer::cartesianToSpherical(
    const pcl::PointCloud<pcl::PointXYZRGB>& cartesian_cloud,
    std::vector<SphericalPoint>& spherical_cloud);
} // namespace clustering

#endif // CLUSTERER_HPP
