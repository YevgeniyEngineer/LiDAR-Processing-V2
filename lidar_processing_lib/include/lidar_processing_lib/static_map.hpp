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
#include <iterator>
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
    struct Iterator
    {
        using iterator_category = std::random_access_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = std::pair<int, T>;
        using pointer = value_type*;
        using reference = value_type&;

        std::vector<int>::iterator index_it;
        typename std::vector<T>::iterator value_it;

        Iterator(std::vector<int>::iterator idx, typename std::vector<T>::iterator val)
            : index_it(idx), value_it(val)
        {
        }

        // Operator to access the current pair directly, eliminating static temporary
        value_type operator*() const
        {
            return {*index_it, *value_it};
        }

        pointer operator->()
        {
            return &operator*();
        }

        // Increment and decrement operators
        Iterator& operator++()
        {
            ++index_it;
            ++value_it;
            return *this;
        }
        Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        // Additions and subtractions
        Iterator& operator+=(difference_type n)
        {
            index_it += n;
            value_it += n;
            return *this;
        }
        Iterator operator+(difference_type n) const
        {
            return Iterator(index_it + n, value_it + n);
        }
        Iterator& operator--()
        {
            --index_it;
            --value_it;
            return *this;
        }
        Iterator operator--(int)
        {
            Iterator tmp = *this;
            --(*this);
            return tmp;
        }
        Iterator& operator-=(difference_type n)
        {
            index_it -= n;
            value_it -= n;
            return *this;
        }
        Iterator operator-(difference_type n) const
        {
            return Iterator(index_it - n, value_it - n);
        }
        difference_type operator-(const Iterator& other) const
        {
            return index_it - other.index_it;
        }

        // Comparison operators
        bool operator==(const Iterator& other) const
        {
            return index_it == other.index_it;
        }
        bool operator!=(const Iterator& other) const
        {
            return !(*this == other);
        }
    };

    struct ConstIterator
    {
        using iterator_category = std::random_access_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = const std::pair<const int, const T>;
        using pointer = const value_type*;
        using reference = const value_type&;

        std::vector<int>::const_iterator index_it;
        typename std::vector<T>::const_iterator value_it;

        ConstIterator(std::vector<int>::const_iterator idx,
                      typename std::vector<T>::const_iterator val)
            : index_it(idx), value_it(val)
        {
        }

        value_type operator*() const
        {
            return {*index_it, *value_it};
        }

        pointer operator->() const
        {
            return &operator*();
        }

        ConstIterator& operator++()
        {
            ++index_it;
            ++value_it;
            return *this;
        }
        ConstIterator operator++(int)
        {
            ConstIterator tmp = *this;
            ++(*this);
            return tmp;
        }
        ConstIterator& operator+=(difference_type n)
        {
            index_it += n;
            value_it += n;
            return *this;
        }
        ConstIterator operator+(difference_type n) const
        {
            return ConstIterator(index_it + n, value_it + n);
        }
        ConstIterator& operator--()
        {
            --index_it;
            --value_it;
            return *this;
        }
        ConstIterator operator--(int)
        {
            ConstIterator tmp = *this;
            --(*this);
            return tmp;
        }
        ConstIterator& operator-=(difference_type n)
        {
            index_it -= n;
            value_it -= n;
            return *this;
        }
        ConstIterator operator-(difference_type n) const
        {
            return ConstIterator(index_it - n, value_it - n);
        }
        difference_type operator-(const ConstIterator& other) const
        {
            return index_it - other.index_it;
        }
        bool operator==(const ConstIterator& other) const
        {
            return index_it == other.index_it;
        }
        bool operator!=(const ConstIterator& other) const
        {
            return !(*this == other);
        }
    };

    void reserve(int num_total)
    {
        indices_.reserve(num_total);
        values_.reserve(num_total);
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
        const auto it = std::lower_bound(indices_.begin(), indices_.end(), index);

        if (it != indices_.end() && *it == index)
        {
            // Index already exists, update the value
            values_[it - indices_.begin()] = value;
        }
        else
        {
            // Insert new index and value in the correct position to maintain sorted order
            indices_.insert(it, index);
            values_.insert(values_.begin() + (it - indices_.begin()), value);
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
            throw std::out_of_range("Index not found");
        }
        return values_[it - indices_.cbegin()];
    }
    const T& at(int index) const
    {
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);
        if (it == indices_.cend() || *it != index)
        {
            throw std::out_of_range("Index not found");
        }
        return values_[it - indices_.cbegin()];
    }
    T& operator[](int index) noexcept
    {
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);
        return values_[it - indices_.cbegin()];
    }
    const T& operator[](int index) const noexcept
    {
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);
        return values_[it - indices_.cbegin()];
    }

    Iterator find(int index)
    {
        auto it = std::lower_bound(indices_.begin(), indices_.end(), index);
        if (it != indices_.end() && *it == index)
        {
            return Iterator(it, values_.begin() + (it - indices_.begin()));
        }
        return end();
    }

    ConstIterator find(int index) const
    {
        const auto it = std::lower_bound(indices_.cbegin(), indices_.cend(), index);
        if (it != indices_.cend() && *it == index)
        {
            return ConstIterator(it, values_.cbegin() + (it - indices_.cbegin()));
        }
        return end();
    }

    Iterator begin()
    {
        return Iterator(indices_.begin(), values_.begin());
    }
    Iterator end()
    {
        return Iterator(indices_.end(), values_.end());
    }
    ConstIterator begin() const
    {
        return ConstIterator(indices_.cbegin(), values_.cbegin());
    }
    ConstIterator end() const
    {
        return ConstIterator(indices_.cend(), values_.cend());
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

    inline int computeFlattenedIndex(int range_index,
                                     int azimuth_index,
                                     int elevation_index) noexcept
    {
        return range_index + num_range_ * (azimuth_index + num_azimuth_ * elevation_index);
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STATIC_MAP_HPP
