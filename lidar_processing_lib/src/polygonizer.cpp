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

#include "polygonizer.hpp"
#include "common.hpp"

// STL
#include <cmath>
#include <iostream>
#include <limits>

namespace lidar_processing_lib
{
template <typename PointT>
void Polygonizer::convexHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices)
{
    if (points.size() < 3)
    {
        indices.resize(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        return;
    }

    sorted_indices_.resize(points.size());

    static constexpr auto counterclockwiseOrColinear =
        [](const PointT& p1, const PointT& p2, const PointT& p3) noexcept -> bool {
        return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x) <= 0;
    };

    // Sort indices lexicographically according to points xy coordinates
    std::iota(sorted_indices_.begin(), sorted_indices_.end(), 0);
    std::sort(sorted_indices_.begin(),
              sorted_indices_.end(),
              [&points](const auto i, const auto j) noexcept -> bool {
                  return points[i].x < points[j].x ||
                         (std::fabs(points[i].x - points[j].x) <
                              std::numeric_limits<decltype(points[i].x)>::epsilon() &&
                          points[i].y < points[j].y);
              });

    auto n = static_cast<std::int32_t>(points.size());
    auto k = static_cast<std::int32_t>(0);

    indices.resize(2 * points.size());

    // Build lower hull
    for (std::int32_t i = 0; i < n; ++i)
    {
        while (k > 1 && counterclockwiseOrColinear(points[indices[k - 2]],
                                                   points[indices[k - 1]],
                                                   points[sorted_indices_[i]]))
        {
            --k;
        }
        indices[k++] = sorted_indices_[i];
    }

    // Build upper hull
    for (std::int32_t i = n - 2, t = k + 1; i >= 0; --i)
    {
        while (k >= t && counterclockwiseOrColinear(points[indices[k - 2]],
                                                    points[indices[k - 1]],
                                                    points[sorted_indices_[i]]))
        {
            --k;
        }
        indices[k++] = sorted_indices_[i];
    }

    indices.resize(k - 1);
}

void Polygonizer::findAntipodalPairsOfConvexHull(const std::vector<PointXY>& convex_hull_points,
                                                 std::vector<AntipodalPair>& antipodal_pairs)
{
    antipodal_pairs.clear();

    const std::int32_t n = convex_hull_points.size();

    if (n < 2)
    {
        return;
    }

    const std::int32_t i0 = n - 1;
    std::int32_t i = 0;
    std::int32_t j = 1;

    const auto next_index = [n](const std::int32_t index) noexcept -> std::int32_t {
        return (index + 1 == n) ? 0 : (index + 1);
    };

    while (areaOfTriangle(convex_hull_points[i],
                          convex_hull_points[next_index(i)],
                          convex_hull_points[next_index(j)]) >
           areaOfTriangle(
               convex_hull_points[i], convex_hull_points[next_index(i)], convex_hull_points[j]))
    {
        j = next_index(j);
    }

    const std::int32_t j0 = j;

    // Iterate to find all antipodal pairs
    while (i != j0)
    {
        i = next_index(i);
        antipodal_pairs.push_back({i, j});

        while (areaOfTriangle(convex_hull_points[i],
                              convex_hull_points[next_index(i)],
                              convex_hull_points[next_index(j)]) >
               areaOfTriangle(
                   convex_hull_points[i], convex_hull_points[next_index(i)], convex_hull_points[j]))
        {
            j = next_index(j);

            if (!(i == j0 && j == i0))
            {
                antipodal_pairs.push_back({i, j});
            }
            else
            {
                return;
            }
        }
        if (areaOfTriangle(convex_hull_points[j],
                           convex_hull_points[next_index(i)],
                           convex_hull_points[next_index(j)]) ==
            areaOfTriangle(
                convex_hull_points[i], convex_hull_points[next_index(i)], convex_hull_points[j]))
        {
            if (!(i == j0 && j == i0))
            {
                antipodal_pairs.push_back({i, next_index(j)});
            }
            else
            {
                antipodal_pairs.push_back({next_index(i), j});
            }
        }
    }
}

BoundingBox Polygonizer::boundingBoxRotatingCalipers(const std::vector<PointXY>& convex_hull_points)
{
    BoundingBox min_box;
    min_box.is_valid = false;

    const std::int32_t n = convex_hull_points.size();

    if (n < 3)
    {
        return min_box;
    }

    findAntipodalPairsOfConvexHull(convex_hull_points, antipodal_pairs_);

    // Compute centroid of the convex hull
    PointXY centroid = {0.0, 0.0};
    for (const auto& point : convex_hull_points)
    {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    centroid.x /= n;
    centroid.y /= n;

    min_box.area = std::numeric_limits<double>::max();
    rotated_points_.resize(n);

    for (const auto [i, j] : antipodal_pairs_)
    {
        const std::array<std::int32_t, 2> pair = {i, j};

        for (const auto index : pair)
        {
            constexpr std::array<std::int32_t, 2> offsets = {-1, 1};

            for (const auto offset : offsets)
            {
                auto neighbor_index = index + offset;

                if (neighbor_index < 0)
                {
                    neighbor_index += n;
                }
                else if (neighbor_index >= n)
                {
                    neighbor_index -= n;
                }

                const auto& p0 = convex_hull_points[index];
                const auto& p1 = convex_hull_points[neighbor_index];

                const auto edge_x = p1.x - p0.x;
                const auto edge_y = p1.y - p0.y;
                const auto edge_length = std::sqrt(edge_x * edge_x + edge_y * edge_y);

                if (edge_length < 1.0e-6)
                {
                    continue;
                }

                const auto ux = edge_x / edge_length;
                const auto uy = edge_y / edge_length;

                // Rotate points
                for (std::int32_t k = 0; k < n; ++k)
                {
                    const auto& pt = convex_hull_points[k];
                    const double translated_x = pt.x - centroid.x;
                    const double translated_y = pt.y - centroid.y;

                    rotated_points_[k].x = translated_x * ux + translated_y * uy;
                    rotated_points_[k].y = -translated_x * uy + translated_y * ux;
                }

                // Calculate the bounding box of the rotated points
                auto min_x = std::numeric_limits<decltype(edge_x)>::max();
                auto max_x = std::numeric_limits<decltype(edge_x)>::lowest();
                auto min_y = std::numeric_limits<decltype(edge_x)>::max();
                auto max_y = std::numeric_limits<decltype(edge_x)>::lowest();

                for (const auto& point : rotated_points_)
                {
                    min_x = std::min(min_x, point.x);
                    max_x = std::max(max_x, point.x);
                    min_y = std::min(min_y, point.y);
                    max_y = std::max(max_y, point.y);
                }

                const double width = max_x - min_x;
                const double height = max_y - min_y;
                const double area = width * height;

                if (area < min_box.area)
                {
                    min_box.area = area;

                    min_box.corners[0] = {min_x * ux - min_y * uy + centroid.x,
                                          min_x * uy + min_y * ux + centroid.y};
                    min_box.corners[1] = {max_x * ux - min_y * uy + centroid.x,
                                          max_x * uy + min_y * ux + centroid.y};
                    min_box.corners[2] = {max_x * ux - max_y * uy + centroid.x,
                                          max_x * uy + max_y * ux + centroid.y};
                    min_box.corners[3] = {min_x * ux - max_y * uy + centroid.x,
                                          min_x * uy + max_y * ux + centroid.y};

                    min_box.angle_rad = lidar_processing_lib::atan2Approx(uy, ux);
                    min_box.is_valid = true;
                }
            }
        }
    }

    return min_box;
}

BoundingBox Polygonizer::boundingBoxPrincipalComponentAnalysis(
    const std::vector<PointXY>& convex_hull_points)
{
    BoundingBox min_box;

    if (convex_hull_points.size() < 3)
    {
        min_box.is_valid = false;
        return min_box;
    }

    const std::uint32_t n_points = convex_hull_points.size();

    if (n_points * 2 > centered_matrix_buffer_.size())
    {
        data_matrix_buffer_.resize(n_points * 2);
        centered_matrix_buffer_.resize(n_points * 2);
        rotated_points_buffer_.resize(n_points * 2);
    }

    // Step 0: Prepare matrix buffers
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data_matrix{
        data_matrix_buffer_.data(), static_cast<int>(n_points), 2};

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        centred_matrix{centered_matrix_buffer_.data(), static_cast<int>(n_points), 2};

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        rotated_points_matrix{rotated_points_buffer_.data(), static_cast<int>(n_points), 2};

    // Step 1: Convert points to Eigen matrix
    for (std::uint32_t i = 0; i < n_points; ++i)
    {
        data_matrix(i, 0) = convex_hull_points[i].x;
        data_matrix(i, 1) = convex_hull_points[i].y;
    }

    // Step 2: Calculate the mean of the points
    const Eigen::RowVector2d mean = data_matrix.colwise().mean();

    // Step 3: Center the data
    centred_matrix = data_matrix.rowwise() - mean;

    // Step 4: Compute the covariance matrix
    covariance_matrix_.noalias() = centred_matrix.transpose() * centred_matrix;
    covariance_matrix_ /= (n_points - 1);

    // Step 5: Perform eigen decomposition
    svd_.compute(covariance_matrix_, Eigen::ComputeThinV);
    if (svd_.info() != Eigen::Success)
    {
        min_box.is_valid = false;
        return min_box;
    }
    const Eigen::MatrixXd& principal_components = svd_.matrixV();

    // Step 6: Rotate the points to align with principal components
    rotated_points_matrix.noalias() = centred_matrix * principal_components;

    // Step 7: Calculate bounding box in the rotated space
    const auto min_x = rotated_points_matrix.col(0).minCoeff();
    const auto max_x = rotated_points_matrix.col(0).maxCoeff();
    const auto min_y = rotated_points_matrix.col(1).minCoeff();
    const auto max_y = rotated_points_matrix.col(1).maxCoeff();

    // Step 8: Transform bounding box corners back to the original space
    Eigen::Matrix<double, 4, 2> corners;
    corners << min_x, min_y, max_x, min_y, max_x, max_y, min_x, max_y;

    corners = corners * principal_components.transpose();
    corners.rowwise() += mean;

    // Store the bounding box
    min_box.corners = {{{corners(0, 0), corners(0, 1)},
                        {corners(1, 0), corners(1, 1)},
                        {corners(2, 0), corners(2, 1)},
                        {corners(3, 0), corners(3, 1)}}};
    min_box.area = (max_x - min_x) * (max_y - min_y);
    min_box.is_valid = true;
    min_box.angle_rad = std::atan2(principal_components(1, 0), principal_components(0, 0));

    return min_box;
}

// Explicit template specializations
template void Polygonizer::convexHull(const std::vector<PointXY>& points,
                                      std::vector<std::int32_t>& indices);
template void Polygonizer::convexHull(const std::vector<PointXYZ>& points,
                                      std::vector<std::int32_t>& indices);

/*  Keep unoptimized algorithm for historical record */

// BoundingBox Polygonizer::boundingBoxRotatingCalipers(const std::vector<PointXY>&
// convex_hull_points)
// {
//     BoundingBox min_box;
//     min_box.is_valid = false;

//     const std::size_t n = convex_hull_points.size();
//     if (n < 3)
//     {
//         return min_box;
//     }

//     min_box.area = std::numeric_limits<double>::max();

//     for (std::size_t i = 0; i < n; ++i)
//     {
//         const auto j = (i == n - 1) ? 0 : (i + 1);

//         const auto& p0 = convex_hull_points[i];
//         const auto& p1 = convex_hull_points[j];

//         // Edge represents a vector from p0 to p1
//         const auto edge_x = p1.x - p0.x;
//         const auto edge_y = p1.y - p0.y;
//         const auto edge_length = std::sqrt(edge_x * edge_x + edge_y * edge_y);

//         if (edge_length < 1.0e-8)
//         {
//             continue;
//         }

//         const auto ux = edge_x / edge_length;
//         const auto uy = edge_y / edge_length;
//         const auto vx = -uy;
//         const auto vy = ux;

//         // Calculate bounding box corners in the UV space
//         auto min_u = std::numeric_limits<decltype(edge_x)>::max();
//         auto max_u = std::numeric_limits<decltype(edge_x)>::lowest();
//         auto min_v = std::numeric_limits<decltype(edge_x)>::max();
//         auto max_v = std::numeric_limits<decltype(edge_x)>::lowest();

//         for (const auto& point : convex_hull_points)
//         {
//             const auto proj_u = point.x * ux + point.y * uy;
//             const auto proj_v = point.x * vx + point.y * vy;

//             if (proj_u < min_u)
//             {
//                 min_u = proj_u;
//             }
//             if (proj_u > max_u)
//             {
//                 max_u = proj_u;
//             }
//             if (proj_v < min_v)
//             {
//                 min_v = proj_v;
//             }
//             if (proj_v > max_v)
//             {
//                 max_v = proj_v;
//             }
//         }

//         // Calculate oriented bounding box
//         const auto width = max_u - min_u;
//         const auto height = max_v - min_v;
//         const auto area = width * height;

//         // Update minimum bounding box if it is better than previous
//         if (area < min_box.area)
//         {
//             min_box.area = area;

//             const auto min_u_ux = min_u * ux;
//             const auto max_u_ux = max_u * ux;
//             const auto min_v_vx = min_v * vx;
//             const auto max_v_vx = max_v * vx;
//             const auto min_u_uy = min_u * uy;
//             const auto max_u_uy = max_u * uy;
//             const auto min_v_vy = min_v * vy;
//             const auto max_v_vy = max_v * vy;

//             min_box.corners[0] = {min_u_ux + min_v_vx, min_u_uy + min_v_vy};
//             min_box.corners[1] = {max_u_ux + min_v_vx, max_u_uy + min_v_vy};
//             min_box.corners[2] = {max_u_ux + max_v_vx, max_u_uy + max_v_vy};
//             min_box.corners[3] = {min_u_ux + max_v_vx, min_u_uy + max_v_vy};

//             min_box.angle_rad = std::atan2(edge_y, edge_x);
//         }
//     }

//     min_box.is_valid = true;

//     return min_box;
// }

} // namespace lidar_processing_lib
