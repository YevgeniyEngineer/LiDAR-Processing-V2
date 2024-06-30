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

#ifndef LIDAR_PROCESSING_LIB__CLUSTERER_HPP
#define LIDAR_PROCESSING_LIB__CLUSTERER_HPP

// Internal
#include "circular_queue.hpp"
#include "union_find.hpp"

// External
#include <ankerl/unordered_dense.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

namespace lidar_processing_lib
{
using ClusterLabel = std::int32_t;

struct SphericalPoint
{
    float range_m;
    float azimuth_rad;   // Convention: 0 -> 2 * pi
    float elevation_rad; // Convention: 0 -> pi
};

struct ClustererConfiguration
{
    float voxel_grid_range_resolution_m = 0.4F;
    float voxel_grid_azimuth_resolution_deg = 1.0F;
    float voxel_grid_elevation_resolution_deg = 1.5F;

    std::uint32_t min_cluster_size = 3;
};

class Clusterer
{
  public:
    static constexpr float TWO_M_PIf = static_cast<float>(2.0 * M_PI);
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0);
    static constexpr std::int32_t INVALID_LABEL = -1;

    using VoxelIndex = std::int32_t;

    Clusterer();

    void config(const ClustererConfiguration& config)
    {
        config_ = config;
        voxel_grid_range_resolution_m_ = config_.voxel_grid_range_resolution_m;
        voxel_grid_azimuth_resolution_rad_ = config_.voxel_grid_azimuth_resolution_deg * DEG_TO_RAD;
        voxel_grid_elevation_resolution_rad_ =
            config_.voxel_grid_elevation_resolution_deg * DEG_TO_RAD;
    }

    const ClustererConfiguration& config() const noexcept
    {
        return config_;
    }

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

    static constexpr std::int32_t INDEX_OFFSETS_WITHOUT_SELF[26][3] = {
        {-1, -1, -1}, {-1, 0, -1}, {-1, 1, -1}, {0, -1, -1}, {0, 0, -1}, {0, 1, -1}, {1, -1, -1},
        {1, 0, -1},   {1, 1, -1},  {-1, -1, 0}, {-1, 0, 0},  {-1, 1, 0}, {0, -1, 0}, {0, 1, 0},
        {1, -1, 0},   {1, 0, 0},   {1, 1, 0},   {-1, -1, 1}, {-1, 0, 1}, {-1, 1, 1}, {0, -1, 1},
        {0, 0, 1},    {0, 1, 1},   {1, -1, 1},  {1, 0, 1},   {1, 1, 1}};

    struct VoxelKey final
    {
        std::int32_t range_index;
        std::int32_t azimuth_index;
        std::int32_t elevation_index;

        inline bool operator==(const VoxelKey& other) const noexcept
        {
            return (range_index == other.range_index) && (azimuth_index == other.azimuth_index) &&
                   (elevation_index == other.elevation_index);
        }
    };

    ClustererConfiguration config_;

    float voxel_grid_range_resolution_m_;
    float voxel_grid_azimuth_resolution_rad_;
    float voxel_grid_elevation_resolution_rad_;

    std::int32_t num_range_;
    std::int32_t num_azimuth_;
    std::int32_t num_elevation_;

    std::vector<SphericalPoint> spherical_cloud_;
    std::vector<VoxelIndex> point_to_voxel_indices_;
    std::vector<VoxelKey> voxel_keys_;

    ankerl::unordered_dense::segmented_map<VoxelIndex, ClusterLabel> voxel_index_labels_map_;
    ankerl::unordered_dense::segmented_set<VoxelIndex> visited_voxels_;

    lidar_processing_lib::CircularQueue<VoxelKey, 150'000> voxel_key_queue_;

    std::vector<std::uint32_t> cluster_labels_counts_;
    std::vector<ClusterLabel> cluster_labels_cache_;

    UnionFind union_find_;

    template <typename PointT>
    void cartesianToSpherical(const pcl::PointCloud<PointT>& cartesian_cloud,
                              std::vector<SphericalPoint>& spherical_cloud);

    void buildHashTable();

    void curvedVoxelClusteringImpl(std::vector<ClusterLabel>& labels);

    void propagateLabelToNeighbouringVoxels(ClusterLabel label, const VoxelKey& core_voxel_key);

    void removeSmallClusters(std::vector<ClusterLabel>& labels);
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
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__CLUSTERER_HPP
