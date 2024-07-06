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

#ifndef LIDAR_PROCESSING_LIB__STATIC_MAP_HPP
#define LIDAR_PROCESSING_LIB__STATIC_MAP_HPP

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
class StaticMap
{
  public:
    void reserve(int num_totaly)
    {
        indices_.reserve(num_totaly);
        values_.reserve(num_totaly);
    }

    void clear() noexcept
    {
        indices_.clear();
        values_.clear();
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

    // Insert a value with given radial, azimuth, and elevation indices
    void insert(int range_index, int azimuth_index, int elevation_index, const T& value)
    {
        const auto index = computeFlattenedIndex(range_index, azimuth_index, elevation_index);
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);

        if (it != indices_.cend() && *it == index)
        {
            // Index already exists, update the value
            values_[it - indices_.cbegin()] = value;
        }
        else
        {
            // Insert new index and value in the correct position to maintain sorted order
            indices_.insert(it, index);
            values_.insert(values_.cbegin() + (it - indices_.cbegin()), value);
        }
    }

    // Insert flattened index
    void insert(int index, const T& value)
    {
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);

        if (it != indices_.cend() && *it == index)
        {
            // Index already exists, update the value
            values_[it - indices_.cbegin()] = value;
        }
        else
        {
            // Insert new index and value in the correct position to maintain sorted order
            indices_.insert(it, index);
            values_.insert(values_.cbegin() + (it - indices_.cbegin()), value);
        }
    }

    // Retrieve the value corresponding to the given indices
    T& at(int range_index, int azimuth_index, int elevation_index)
    {
        const auto index = computeFlattenedIndex(range_index, azimuth_index, elevation_index);
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);

        if (it == indices_.cend() || *it != index)
        {
            throw std::out_of_range("No value found for the given indices.");
        }

        return values_[it - indices_.cbegin()];
    }

    // Retrieve the value corresponding to the given flattened index
    T& at(int index)
    {
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);

        if (it == indices_.cend() || *it != index)
        {
            throw std::out_of_range("No value found for the given indices.");
        }

        return values_[it - indices_.cbegin()];
    }

    // Print all indices and values for debugging
    void printAll() const
    {
        for (std::size_t i = 0; i < indices_.size(); ++i)
        {
            std::cout << "Index: " << indices_[i] << ", Value: " << values_[i] << std::endl;
        }
    }

  private:
    std::vector<int> indices_;
    std::vector<T> values_;

    int num_range_;
    int num_azimuth_;
    int num_elevation_;

    int computeFlattenedIndex(int range_index, int azimuth_index, int elevation_index)
    {
        return range_index + num_range_ * (azimuth_index + num_azimuth_ * elevation_index);
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STATIC_MAP_HPP
