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
enum class PolygonOrientation : std::uint8_t
{
    ANTICLOCKWISE = 0,
    CLOCKWISE = 1
};

enum class PolygonContour : std::uint8_t
{
    OPEN = 0,
    CLOSED = 1
};

struct Configuration
{
    PolygonOrientation orientation = PolygonOrientation::ANTICLOCKWISE;
    PolygonContour contour = PolygonContour::OPEN;

    std::uint32_t max_points = 100'000;
};

class Polygonizer final
{
  public:
    Polygonizer()
    {
        sorted_indices_.reserve(config_.max_points);
    }

    template <typename PointT>
    void boundingBox(const std::vector<PointT>& points, std::vector<std::uint32_t>& indices);

    template <typename PointT>
    void convexHull(const std::vector<PointT>& points, std::vector<std::uint32_t>& indices);

    template <typename PointT>
    void concaveHull(const std::vector<PointT>& points, std::vector<std::uint32_t>& indices);

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

    std::vector<std::uint32_t> sorted_indices_;
};

template <typename PointT, typename T>
inline static T crossProduct(const PointT& O, const PointT& A, const PointT& B) noexcept
{
    return static_cast<T>((A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x));
}

template <typename PointT>
void Polygonizer::boundingBox(const std::vector<PointT>& points,
                              std::vector<std::uint32_t>& indices)
{
    // TODO
}

template <typename PointT>
void Polygonizer::convexHull(const std::vector<PointT>& points, std::vector<std::uint32_t>& indices)
{
    // Andrew's monotone chain algorithm

    indices.clear();

    if (points.size() < 3)
    {
        return;
    }

    sorted_indices_.resize(points.size());
    std::iota(sorted_indices_.begin(), sorted_indices_.end(), 0);

    std::sort(sorted_indices_.begin(),
              sorted_indices_.end(),
              [&points](const std::uint32_t i, const std::uint32_t j) noexcept {
                  return (points[i].x < points[j].x) ||
                         (std::fabs(points[i].x - points[j].x) < 1e-5 && points[i].y < points[j].y);
              });

    indices.reserve(2 * points.size());

    // Build the lower hull
    const std::size_t half_size = points.size();
    for (std::size_t i = 0; i < half_size; ++i)
    {
        // if crossProduct(OA, OB) < 0 - Clockwise
        while (indices.size() >= 2 &&
               crossProduct<PointT, std::int64_t>(points[indices[indices.size() - 2]],
                                                  points[indices.back()],
                                                  points[sorted_indices_[i]]) <= 0)
        {
            // Remove the last point because it's collinear or makes a right turn
            indices.pop_back();
        }

        indices.push_back(sorted_indices_[i]);
    }

    // Build the upper hull
    for (std::size_t i = half_size - 1, t = indices.size() + 1; i > 0; --i)
    {
        while (indices.size() >= t &&
               crossProduct<std::int64_t>(points[indices[indices.size() - 2]],
                                          points[indices.back()],
                                          points[sorted_indices_[i - 1]]) <= 0)
        {
            // Remove the last point because it's collinear or makes a right turn
            indices.pop_back();
        }
        indices.push_back(sorted_indices_[i - 1]);
    }

    if (config_.contour == PolygonContour::OPEN)
    {
        indices.pop_back(); // Remove the last point because it is the same as the first one
    }

    if (config_.orientation == PolygonOrientation::CLOCKWISE)
    {
        std::reverse(indices.begin(), indices.end());
    }
}

template <typename PointT>
void Polygonizer::concaveHull(const std::vector<PointT>& points,
                              std::vector<std::uint32_t>& indices)
{
    // TODO
}

} // namespace polygonization

#endif // POLYGONIZER_HPP
