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

#ifndef LIDAR_PROCESSING_LIB__GEOMETRY_HPP
#define LIDAR_PROCESSING_LIB__GEOMETRY_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace lidar_processing_lib
{
/// @brief Specifies orientation of triplet's cross product
enum class Orientation : std::uint8_t
{
    COLLINEAR = 0,
    CLOCKWISE = 1,
    COUNTERCLOCKWISE = 2
};

/// @brief Calculate orientation of the triplet pq -> qr
/// @note
/// 0 -> p, q and r are collinear
/// 1 -> Clockwise
/// 2 -> Counterclockwise
template <typename PointT>
inline constexpr Orientation orientation(const PointT& p, const PointT& q, const PointT& r) noexcept
{
    const float cross_product = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    return (std::fabs(cross_product) < std::numeric_limits<decltype(q.y)>::epsilon())
               ? Orientation::COLLINEAR
               : ((cross_product > 0) ? Orientation::CLOCKWISE : Orientation::COUNTERCLOCKWISE);
}

/// @brief Check if point q lies on line segment pr
template <typename PointT>
inline constexpr bool onSegment(const PointT& p, const PointT& q, const PointT& r) noexcept
{
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) &&
           q.y >= std::min(p.y, r.y);
}

// @brief Check if line segment 'p1q1' and 'p2q2' intersect
template <typename PointT>
inline constexpr bool doIntersect(const PointT& p1,
                                  const PointT& q1,
                                  const PointT& p2,
                                  const PointT& q2) noexcept
{
    // Find the four orientations needed for the general and special cases
    const Orientation o1 = orientation(p1, q1, p2);
    const Orientation o2 = orientation(p1, q1, q2);
    const Orientation o3 = orientation(p2, q2, p1);
    const Orientation o4 = orientation(p2, q2, q1);

    // General case
    // OR
    // Special cases:
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    // OR
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    // OR
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    // OR
    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    return static_cast<bool>((o1 != o2 && o3 != o4) ||
                             (o1 == Orientation::COLLINEAR && onSegment(p1, p2, q1)) ||
                             (o2 == Orientation::COLLINEAR && onSegment(p1, q2, q1)) ||
                             (o3 == Orientation::COLLINEAR && onSegment(p2, p1, q2)) ||
                             (o4 == Orientation::COLLINEAR && onSegment(p2, q1, q2)));
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__GEOMETRY_HPP
