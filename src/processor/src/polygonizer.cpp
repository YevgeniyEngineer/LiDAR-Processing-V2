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

// STL
#include <cmath>
#include <iostream>
#include <limits>

namespace polygonization
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

BoundingBox Polygonizer::boundingBoxRotatingCalipers(const std::vector<PointXY>& points)
{
    BoundingBox min_box;

    if (points.size() < 3)
    {
        min_box.is_valid = false;
        return min_box;
    }

    const std::size_t n = points.size();
    auto min_area = std::numeric_limits<double>::max();

    rotated_points_.reserve(points.size());

    // Find centroid
    PointXY centroid;
    for (const auto& point : points)
    {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    centroid.x /= n;
    centroid.y /= n;

    for (std::size_t i = 0; i < n; ++i)
    {
        const PointXY& p0 = points[i];
        const PointXY& p1 = points[(i + 1) % n];

        const PointXY edge = l_subtract(p1, p0);
        const double length = std::sqrt(edge.x * edge.x + edge.y * edge.y);
        if (length <= 1e-8)
        {
            continue; // Skip degenerate edges
        }

        const PointXY edge_norm = {edge.x / length, edge.y / length};
        const double angle_rad = std::atan2(edge_norm.y, edge_norm.x);

        // Rotate points
        rotated_points_.clear();
        for (const auto& point : points)
        {
            rotated_points_.push_back(l_rotate(point, -angle_rad, centroid));
        }

        // Calculate bounding box
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();

        for (const auto& point : rotated_points_)
        {
            if (point.x < min_x)
            {
                min_x = point.x;
            }
            if (point.x > max_x)
            {
                max_x = point.x;
            }
            if (point.y < min_y)
            {
                min_y = point.y;
            }
            if (point.y > max_y)
            {
                max_y = point.y;
            }
        }

        BoundingBox box;
        box.corners = {{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}}};
        box.area = (max_x - min_x) * (max_y - min_y);
        box.angle_rad = angle_rad;

        // Update minimum bounding box
        if (box.area < min_area)
        {
            // Rotate bounding box corners back to the original orientation
            for (auto& corner : box.corners)
            {
                corner = l_rotate(corner, angle_rad, centroid);
            }

            min_area = box.area;
            min_box = box;
        }
    }

    min_box.is_valid = true;
    return min_box;
}

BoundingBox Polygonizer::boundingBoxPrincipalComponentAnalysis(const std::vector<PointXY>& points)
{
    BoundingBox min_box;

    if (points.size() < 3)
    {
        min_box.is_valid = false;
        return min_box;
    }

    const std::uint32_t n_points = points.size();

    if (n_points * 2 > centered_matrix_buffer_.size())
    {
        data_matrix_buffer_.resize(n_points * 2);
        centered_matrix_buffer_.resize(n_points * 2);
    }

    // Step 0: Prepare matrix buffers
    Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> data_matrix{
        data_matrix_buffer_.data(), static_cast<int>(n_points), 2};

    Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> centred_matrix{
        centered_matrix_buffer_.data(), static_cast<int>(n_points), 2};

    // Step 1: Convert points to Eigen matrix
    for (std::uint32_t i = 0; i < n_points; ++i)
    {
        data_matrix(i, 0) = points[i].x;
        data_matrix(i, 1) = points[i].y;
    }

    // Step 2: Calculate the mean of the points
    const Eigen::RowVector2f mean = data_matrix.colwise().mean();

    // Step 3: Center the data
    centred_matrix = data_matrix.rowwise() - mean;

    // Step 4: Compute the covariance matrix
    const Eigen::Matrix2f covariance_matrix =
        (centred_matrix.transpose() * centred_matrix) / static_cast<float>(n_points - 1);

    // Step 5: Perform eigen decomposition
    svd_.compute(covariance_matrix, Eigen::ComputeThinV);
    if (svd_.info() != Eigen::Success)
    {
        min_box.is_valid = false;
        return min_box;
    }

    const Eigen::Matrix2f principal_components = svd_.matrixV();

    // Step 6: Rotate the points to align with principal components
    const Eigen::MatrixXf rotated_data = centred_matrix * principal_components;

    // Step 7: Calculate bounding box in the rotated space
    const float min_x = rotated_data.col(0).minCoeff();
    const float max_x = rotated_data.col(0).maxCoeff();
    const float min_y = rotated_data.col(1).minCoeff();
    const float max_y = rotated_data.col(1).maxCoeff();

    // Step 8: Transform bounding box corners back to the original space
    Eigen::Matrix<float, 4, 2> corners;
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

} // namespace polygonization
