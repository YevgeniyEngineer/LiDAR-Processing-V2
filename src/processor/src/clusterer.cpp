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

namespace clustering
{
Clusterer::Clusterer()
{
    spherical_cloud_.reserve(150'000);

    voxel_grid_range_resolution_m_ = config_.voxel_grid_range_resolution_m_;
    voxel_grid_azimuth_resolution_rad_ = config_.voxel_grid_azimuth_resolution_deg_ * DEG_TO_RAD;
    voxel_grid_elevation_resolution_rad_ =
        config_.voxel_grid_elevation_resolution_deg_ * DEG_TO_RAD;
}

void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                        std::vector<ClusterLabel>& labels)
{
}

void Clusterer::cartesianToSpherical(const pcl::PointCloud<pcl::PointXYZ>& cartesian_cloud,
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
        static_cast<std::uint16_t>(std::ceil(max_range_m / voxel_grid_range_resolution_m_) + 1);

    num_azimuth_ = static_cast<std::uint16_t>(
        std::ceil(max_azimuth_rad / voxel_grid_azimuth_resolution_rad_) + 1);

    num_elevation_ = static_cast<std::uint16_t>(
        std::ceil(max_elevation_rad / voxel_grid_elevation_resolution_rad_) + 1);
}

void Clusterer::buildHashTable(
    const std::vector<SphericalPoint>& spherical_cloud,
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>>& hash_table)
{
    hash_table.clear();
    hash_table.reserve(spherical_cloud.size());

    for (std::uint32_t point_index = 0; point_index < spherical_cloud.size(); ++point_index)
    {
        const auto& point = spherical_cloud[point_index];

        const std::uint16_t range_index = rangeToIndex(point.range_m);
        const std::uint16_t azimuth_index = azimuthToIndex(point.azimuth_rad);
        const std::uint16_t elevation_index = elevationToIndex(point.elevation_rad);
        const std::uint32_t voxel_index =
            flatVoxelIndex(range_index, azimuth_index, elevation_index);

        hash_table_[voxel_index].push_back(point_index);
    }
}

} // namespace clustering
