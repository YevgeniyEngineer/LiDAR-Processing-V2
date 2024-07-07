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

#ifndef LIDAR_PROCESSING_LIB__STATIC_SET_HPP
#define LIDAR_PROCESSING_LIB__STATIC_SET_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
template <typename T>
class StaticSet
{
  public:
    void reserve(int num_total)
    {
        indices_.reserve(num_total);
    }

    void numRange(int num_range)
    {
        num_range_ = num_range;
    }

    void numAzimuth(int num_azimuth)
    {
        num_azimuth_ = num_azimuth;
    }

    void numElevation(int num_elevation)
    {
        num_elevation_ = num_elevation;
    }

    // Inserts a new composite index into the set
    void insert(int range_index, int azimuth_index, int elevation_index)
    {
        const auto index = computeFlattenedIndex(range_index, azimuth_index, elevation_index);
        const auto it = std::lower_bound(indices_.begin(), indices_.end(), index);
        // Only insert if not already present
        if (it == indices_.end() || *it != index)
        {
            indices_.insert(it, index);
        }
    }

    // Inserts a new flattened index into the set
    void insert(int index)
    {
        const auto it = std::lower_bound(indices_.begin(), indices_.end(), index);
        // Only insert if not already present
        if (it == indices_.end() || *it != index)
        {
            indices_.insert(it, index);
        }
    }

    // Checks if a composite index exists in the set
    bool contains(int range_index, int azimuth_index, int elevation_index) const
    {
        const auto index = computeFlattenedIndex(range_index, azimuth_index, elevation_index);
        return std::binary_search(indices_.cbegin(), indices_.cend(), index);
    }

    // Checks if a flattened index exists in the set
    bool contains(int index) const
    {
        return std::binary_search(indices_.cbegin(), indices_.cend(), index);
    }

    // Clears all indices in the set
    void clear() noexcept
    {
        indices_.clear();
    }

    // Returns the number of unique indices in the set
    std::size_t size() const
    {
        return indices_.size();
    }

  private:
    std::vector<int> indices_;

    int num_range_;
    int num_azimuth_;
    int num_elevation_;

    int computeFlattenedIndex(int range_index, int azimuth_index, int elevation_index)
    {
        return range_index + num_range_ * (azimuth_index + num_azimuth_ * elevation_index);
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STATIC_SET_HPP
