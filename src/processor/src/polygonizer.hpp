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

template <typename PointT>
inline static Orientation orientation(const PointT& p1, const PointT& p2, const PointT& p3) noexcept
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
void Polygonizer::convexHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices)
{
    if (points.size() < 3)
    {
        indices.resize(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        return;
    }

    sorted_indices_.resize(points.size());
    indices.resize(2 * points.size());

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

    for (std::int32_t i = 0; i < n; ++i)
    {
        while (k >= 2 && orientation(points[indices[k - 2]],
                                     points[indices[k - 1]],
                                     points[sorted_indices_[i]]) != Orientation::ANTICLOCKWISE)
        {
            --k;
        }
        indices[k++] = sorted_indices_[i];
    }

    for (std::int32_t i = n - 2, t = k + 1; i >= 0; --i)
    {
        while (k >= t && orientation(points[indices[k - 2]],
                                     points[indices[k - 1]],
                                     points[sorted_indices_[i]]) != Orientation::ANTICLOCKWISE)
        {
            --k;
        }
        indices[k++] = sorted_indices_[i];
    }

    indices.resize(k - 1);
}

template <typename PointT>
void Polygonizer::boundingBox(const std::vector<PointT>& points, std::vector<std::int32_t>& indices)
{
}

template <typename PointT>
void Polygonizer::concaveHull(const std::vector<PointT>& points, std::vector<std::int32_t>& indices)
{
}

} // namespace polygonization

#endif // POLYGONIZER_HPP
