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

#include "clusterer.hpp"

// TODO
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace lidar_processing_lib
{
Clusterer::Clusterer() : union_find_(10'000, 400)
{
    voxel_grid_range_resolution_m_ = config_.voxel_grid_range_resolution_m;
    voxel_grid_azimuth_resolution_rad_ = config_.voxel_grid_azimuth_resolution_deg * DEG_TO_RAD;
    voxel_grid_elevation_resolution_rad_ = config_.voxel_grid_elevation_resolution_deg * DEG_TO_RAD;

    spherical_cloud_.reserve(200'000);
    point_index_to_voxel_index_vector_.reserve(200'000);
    voxel_index_to_labels_map_.reserve(200'000);
    voxel_index_to_voxel_key_map_.reserve(200'000);
    union_find_index_to_voxel_index_vector_.reserve(1500);
    cluster_labels_counts_.reserve(200'000);
    cluster_labels_cache_.reserve(200'000);
}

template <typename PointT>
void Clusterer::cluster(const pcl::PointCloud<PointT>& cloud, std::vector<ClusterLabel>& labels)
{
    cartesianToSpherical(cloud, spherical_cloud_);

    buildHashTable();

    curvedVoxelClusteringImpl(labels);

    removeSmallClusters(labels);
}

template <typename PointT>
void Clusterer::cartesianToSpherical(const pcl::PointCloud<PointT>& cartesian_cloud,
                                     std::vector<SphericalPoint>& spherical_cloud)
{
    spherical_cloud.clear();
    spherical_cloud.reserve(cartesian_cloud.size());

    if (cartesian_cloud.empty())
    {
        return;
    }

    float max_range_m = std::numeric_limits<float>::lowest();
    float max_azimuth_rad = std::numeric_limits<float>::lowest();
    float max_elevation_rad = std::numeric_limits<float>::lowest();

    for (const auto& cartesian_point : cartesian_cloud.points)
    {
        // Map Azimuth angle to 0 -> 2 * pi
        float azimuth_rad = std::atan2(cartesian_point.y, cartesian_point.x);
        azimuth_rad = (azimuth_rad < 0) ? (azimuth_rad + Clusterer::TWO_M_PIf) : azimuth_rad;
        max_azimuth_rad = std::max(max_azimuth_rad, azimuth_rad);

        // Range
        const float distance_m_sqr =
            cartesian_point.x * cartesian_point.x + cartesian_point.y * cartesian_point.y;
        const float distance_m = std::sqrt(distance_m_sqr);
        const float range_m = std::sqrt(distance_m_sqr + cartesian_point.z * cartesian_point.z);
        max_range_m = std::max(max_range_m, range_m);

        // Map Elevation angle to 0 -> pi
        const float elevation_rad = std::atan(cartesian_point.z / distance_m) + M_PI_2f;
        max_elevation_rad = std::max(max_elevation_rad, elevation_rad);

        spherical_cloud.push_back({range_m, azimuth_rad, elevation_rad});
    }

    num_range_ =
        static_cast<std::int32_t>(std::ceil(max_range_m / voxel_grid_range_resolution_m_) + 1);
    num_azimuth_ = static_cast<std::int32_t>(
        std::ceil(max_azimuth_rad / voxel_grid_azimuth_resolution_rad_) + 1);
    num_elevation_ = static_cast<std::int32_t>(
        std::ceil(max_elevation_rad / voxel_grid_elevation_resolution_rad_) + 1);
}

void Clusterer::buildHashTable()
{
    voxel_index_to_labels_map_.clear();
    voxel_index_to_voxel_key_map_.clear();
    point_index_to_voxel_index_vector_.clear();

    for (const auto& point : spherical_cloud_)
    {
        const auto range_index =
            static_cast<std::int32_t>(point.range_m / voxel_grid_range_resolution_m_);
        const auto azimuth_index =
            static_cast<std::int32_t>(point.azimuth_rad / voxel_grid_azimuth_resolution_rad_);
        const auto elevation_index =
            static_cast<std::int32_t>(point.elevation_rad / voxel_grid_elevation_resolution_rad_);
        const auto voxel_index =
            range_index + ((azimuth_index + (elevation_index * num_azimuth_)) * num_range_);

        voxel_index_to_labels_map_[voxel_index] = INVALID_LABEL;
        voxel_index_to_voxel_key_map_[voxel_index] =
            VoxelKey{range_index, azimuth_index, elevation_index};
        point_index_to_voxel_index_vector_.push_back(voxel_index);
    }
}

void Clusterer::curvedVoxelClusteringImpl(std::vector<ClusterLabel>& labels)
{
    // Initialize the labels for the point cloud
    labels.assign(spherical_cloud_.size(), INVALID_LABEL);

    // Reset and prepare union-find data structure
    union_find_.reset(voxel_index_to_labels_map_.size());

    // Map each voxel index to a union-find index
    voxel_index_to_union_find_index_map_.clear();
    voxel_index_to_union_find_index_map_.reserve(voxel_index_to_labels_map_.size());
    union_find_index_to_voxel_index_vector_.clear();
    union_find_index_to_voxel_index_vector_.reserve(voxel_index_to_labels_map_.size());
    union_find_index_to_voxel_key_vector_.clear();
    union_find_index_to_voxel_key_vector_.reserve(voxel_index_to_labels_map_.size());
    std::int32_t union_find_index = 0;
    for (const auto& [voxel_index, label] : voxel_index_to_labels_map_)
    {
        voxel_index_to_union_find_index_map_[voxel_index] = union_find_index++;
        union_find_index_to_voxel_index_vector_.push_back(voxel_index);
        union_find_index_to_voxel_key_vector_.push_back(voxel_index_to_voxel_key_map_[voxel_index]);
    }

    // Iterate through each point in the point cloud
    ClusterLabel label = 0;

    for (std::int32_t core_union_find_index = 0;
         core_union_find_index <
         static_cast<std::int32_t>(union_find_index_to_voxel_index_vector_.size());
         ++core_union_find_index)
    {
        const auto& core_voxel_index =
            union_find_index_to_voxel_index_vector_[core_union_find_index];

        if (voxel_index_to_labels_map_[core_voxel_index] != INVALID_LABEL)
        {
            continue;
        }

        const auto& [range_index, azimuth_index, elevation_index] =
            union_find_index_to_voxel_key_vector_[core_union_find_index];

        bool found_neighbour = false;

        for (const auto& [range_index_offset, azimuth_index_offset, elevation_index_offset] :
             INDEX_OFFSETS_WITHOUT_SELF)
        {
            const auto range_index_with_offset = range_index + range_index_offset;
            auto azimuth_index_with_offset = azimuth_index + azimuth_index_offset;
            if (azimuth_index_with_offset < 0)
            {
                azimuth_index_with_offset = num_azimuth_ - 1;
            }
            else if (azimuth_index_with_offset >= num_azimuth_)
            {
                azimuth_index_with_offset = 0;
            }
            const auto elevation_index_with_offset = elevation_index + elevation_index_offset;

            if ((range_index_with_offset < 0) || (range_index_with_offset >= num_range_) ||
                (elevation_index_with_offset < 0) ||
                (elevation_index_with_offset >= num_elevation_))
            {
                continue;
            }

            const auto neighbour_voxel_index =
                range_index_with_offset +
                ((azimuth_index_with_offset + (elevation_index_with_offset * num_azimuth_)) *
                 num_range_);

            if (const auto voxel_labels_map_iterator =
                    voxel_index_to_labels_map_.find(neighbour_voxel_index);
                voxel_labels_map_iterator != voxel_index_to_labels_map_.cend())
            {
                found_neighbour = true;

                union_find_.unite(core_union_find_index,
                                  voxel_index_to_union_find_index_map_[neighbour_voxel_index]);
            }
        }

        if (!found_neighbour)
        {
            voxel_index_to_labels_map_[core_voxel_index] = label;
            ++label;
        }
        else
        {
            voxel_index_to_labels_map_[core_voxel_index] = label;

            const auto& union_find_set = union_find_.get_set_elements(core_union_find_index);

            for (const auto& union_find_index : union_find_set)
            {
                voxel_index_to_labels_map_
                    [union_find_index_to_voxel_index_vector_[union_find_index]] = label;
            }
        }
    }

    // Assign final labels to each point in the point cloud
    for (std::uint32_t point_index = 0; point_index < spherical_cloud_.size(); ++point_index)
    {
        labels[point_index] = union_find_.find(
            voxel_index_to_union_find_index_map_[point_index_to_voxel_index_vector_[point_index]]);
    }
}

void Clusterer::removeSmallClusters(std::vector<ClusterLabel>& labels)
{
    const auto max_label_it = std::max_element(labels.cbegin(), labels.cend());

    if (max_label_it == labels.cend())
    {
        return; // No labels
    }

    const auto max_label = *max_label_it;

    if (max_label == INVALID_LABEL)
    {
        return; // No clusters
    }

    const auto counts_size = max_label + 1;
    cluster_labels_counts_.assign(counts_size, 0U);

    for (const auto label : labels)
    {
        if (label != INVALID_LABEL)
        {
            ++cluster_labels_counts_[label];
        }
    }

    cluster_labels_cache_.assign(counts_size, INVALID_LABEL);

    std::int32_t current_label = 0;

    for (std::int32_t label = 0; label <= max_label; ++label)
    {
        if (cluster_labels_counts_[label] >= config_.min_cluster_size)
        {
            cluster_labels_cache_[label] = current_label;
            ++current_label;
        }
    }

    for (auto& label : labels)
    {
        label = cluster_labels_cache_[label];
    }
}

// Explicit template instantiations
template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                 std::vector<ClusterLabel>& labels);
template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                 std::vector<ClusterLabel>& labels);
template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                 std::vector<ClusterLabel>& labels);

template void Clusterer::cartesianToSpherical(const pcl::PointCloud<pcl::PointXYZ>& cartesian_cloud,
                                              std::vector<SphericalPoint>& spherical_cloud);
template void Clusterer::cartesianToSpherical(
    const pcl::PointCloud<pcl::PointXYZI>& cartesian_cloud,
    std::vector<SphericalPoint>& spherical_cloud);
template void Clusterer::cartesianToSpherical(
    const pcl::PointCloud<pcl::PointXYZRGB>& cartesian_cloud,
    std::vector<SphericalPoint>& spherical_cloud);
} // namespace lidar_processing_lib
