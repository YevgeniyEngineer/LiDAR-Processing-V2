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

#ifndef LIDAR_PROCESSING_LIB__POLYGONIZER_HPP
#define LIDAR_PROCESSING_LIB__POLYGONIZER_HPP

// Eigen
#include <Eigen/Dense>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
enum class Orientation : std::uint8_t
{
    ANTICLOCKWISE = 0,
    CLOCKWISE = 1,
    COLINEAR = 2
};

enum class PolygonContour : std::uint8_t
{
    OPEN = 0,
    ENCLOSED = 1
};

struct PolygonizerConfiguration
{
    Orientation orientation = Orientation::ANTICLOCKWISE;
    PolygonContour contour = PolygonContour::OPEN;

    std::uint32_t max_points = 100'000;
};

struct PointXY
{
    double x;
    double y;
};

struct PointXYZ
{
    double x;
    double y;
    double z;
};

struct BoundingBox
{
    std::array<PointXY, 4> corners;
    float area;
    float angle_rad; // Yaw angle
    bool is_valid;   // If bounding box fitting was successful
};

struct AntipodalPair final
{
    std::int32_t index_1;
    std::int32_t index_2;
};

class Polygonizer final
{
  public:
    Polygonizer() : covariance_matrix_(2, 2), svd_(2, 2, Eigen::ComputeThinV)
    {
        sorted_indices_.reserve(config_.max_points);
        rotated_points_.reserve(config_.max_points);

        data_matrix_buffer_.reserve(config_.max_points * 2);
        centered_matrix_buffer_.reserve(config_.max_points * 2);
        rotated_points_buffer_.reserve(config_.max_points * 2);

        antipodal_pairs_.reserve(config_.max_points);
    }

    /// @brief Returns a list of points on the convex hull in counter-clockwise order.
    template <typename PointT>
    void convexHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices);

    /// @brief Shamos algorithm to find altipodal pairs of convex polygon.
    /// https://en.wikipedia.org/wiki/Rotating_calipers#:~:text=In%20computational%20geometry%2C%20the%20method,diameter%20using%20Rotating%20Caliper%20method
    /// https://stackoverflow.com/questions/70152394/how-to-understand-shamos-algorithm
    void findAntipodalPairsOfConvexHull(const std::vector<PointXY>& convex_hull_points,
                                        std::vector<AntipodalPair>& antipodal_pairs);

    /// @brief Construct oriented bounding box using Rotating Calipers algorithm.
    BoundingBox boundingBoxRotatingCalipers(const std::vector<PointXY>& convex_hull_points);

    /// @brief Construct oriented bounding box using Principal Component Analysis algorithm.
    BoundingBox boundingBoxPrincipalComponentAnalysis(
        const std::vector<PointXY>& convex_hull_points);

    template <typename PointT>
    void concaveHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices);

    void config(const PolygonizerConfiguration& config)
    {
        config_ = config;

        sorted_indices_.reserve(config_.max_points);
        rotated_points_.reserve(config_.max_points);

        data_matrix_buffer_.reserve(config_.max_points * 2);
        centered_matrix_buffer_.reserve(config_.max_points * 2);
    }

    const PolygonizerConfiguration& config() const noexcept
    {
        return config_;
    }

  private:
    PolygonizerConfiguration config_;

    std::vector<std::int32_t> sorted_indices_;
    std::vector<PointXY> rotated_points_;

    std::vector<double> data_matrix_buffer_;
    std::vector<double> centered_matrix_buffer_;
    std::vector<double> rotated_points_buffer_;
    Eigen::MatrixXd covariance_matrix_;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_;

    std::vector<AntipodalPair> antipodal_pairs_;
};

/// @brief Three points are a counter-clockwise turn if ccw > 0, clockwise if
/// ccw < 0, and collinear if ccw = 0 because ccw is a determinant that
/// gives the signed area of the triangle formed by p1, p2 and p3.
template <typename PointT>
inline Orientation orientation(const PointT& p1, const PointT& p2, const PointT& p3) noexcept
{
    const auto cross_product = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

    if (cross_product > 0.0)
    {
        return Orientation::ANTICLOCKWISE;
    }
    else if (cross_product < 0.0)
    {
        return Orientation::CLOCKWISE;
    }
    else
    {
        return Orientation::COLINEAR;
    }
}

/// @brief 2D cross product.
template <typename PointT>
inline auto crossProduct(const PointT& p1, const PointT& p2) noexcept
{
    return (p1.x * p2.y) - (p2.x * p1.y);
}

/// @brief Polygon area using Shoelace formula.
template <typename PointT>
inline double polygonArea(const std::vector<PointT>& points) noexcept
{
    double area = 0.0;
    if (const auto n = static_cast<std::int32_t>(points.size()); n > 2)
    {
        for (std::int32_t i = 0; i < n - 1; ++i)
        {
            area += crossProduct(points[i], points[i + 1]);
        }
        area += crossProduct(points[n - 1], points[0]); // Closing the polygon
    }
    return std::fabs(area) * 0.5;
}

/// @brief Distance squared between two points in 2D.
template <typename PointT>
inline double distanceSquared(const PointT& p1, const PointT& p2) noexcept
{
    const double dx = p1.x - p2.x;
    const double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

/// @brief Distance between two points in 2D.
template <typename PointT>
inline double distance(const PointT& p1, const PointT& p2) noexcept
{
    return std::sqrt(distanceSquared(p1, p2));
}

/// @brief Area of a rectangle with 4 ordered vertices.
template <typename PointT>
inline double areaOfRectangle(const PointT& p1,
                              const PointT& p2,
                              const PointT& p3,
                              [[maybe_unused]] const PointT& p4) noexcept
{
    return std::sqrt(distanceSquared(p1, p2) * distanceSquared(p2, p3));
}

/// @brief Area of triangle with 3 ordered vertices.
template <typename PointT>
inline double areaOfTriangle(const PointT& p1, const PointT& p2, const PointT& p3) noexcept
{
    return std::fabs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) * 0.5);
}

template <typename PointT>
void Polygonizer::concaveHull([[maybe_unused]] const std::vector<PointT>& points,
                              [[maybe_unused]] std::vector<std::int32_t>& indices)
{
}

// Explicit template specializations
extern template void Polygonizer::convexHull(const std::vector<PointXY>& points,
                                             std::vector<std::int32_t>& indices);
extern template void Polygonizer::convexHull(const std::vector<PointXYZ>& points,
                                             std::vector<std::int32_t>& indices);

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__POLYGONIZER_HPP
