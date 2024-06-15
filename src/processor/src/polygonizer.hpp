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

#ifndef POLYGONIZER_HPP
#define POLYGONIZER_HPP

// STL
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

namespace polygonization
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

struct Configuration
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

class Polygonizer final
{
  public:
    Polygonizer() = default;

    template <typename PointT>
    void boundingBox(const std::vector<PointT>& points, std::vector<std::int32_t>& indices);

    /// @brief Returns a list of points on the convex hull in counter-clockwise order.
    template <typename PointT>
    void convexHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices);

    template <typename PointT>
    void concaveHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices);

    void config(const Configuration& config)
    {
        config_ = config;

        sorted_indices_.reserve(config_.max_points);
    }

    const Configuration& config() const noexcept
    {
        return config_;
    }

  private:
    Configuration config_;

    std::vector<std::int32_t> sorted_indices_;
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

template <typename PointT>
void Polygonizer::boundingBox(const std::vector<PointT>& points, std::vector<std::int32_t>& indices)
{
}

template <typename PointT>
void Polygonizer::concaveHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices)
{
}

// Explicit template specializations
extern template void Polygonizer::convexHull(const std::vector<PointXY>& points,
                                             std::vector<std::int32_t>& indices);
extern template void Polygonizer::convexHull(const std::vector<PointXYZ>& points,
                                             std::vector<std::int32_t>& indices);

} // namespace polygonization

#endif // POLYGONIZER_HPP
