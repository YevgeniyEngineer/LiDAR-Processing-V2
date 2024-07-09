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

#ifndef LIDAR_PROCESSING_LIB__CUSTOM_COMPARATOR_HPP
#define LIDAR_PROCESSING_LIB__CUSTOM_COMPARATOR_HPP

#include <utility>

namespace lidar_processing_lib
{
template <typename Key, typename Value>
struct KeyComparator final
{
    inline bool operator()(const std::pair<Key, Value>& lhs,
                           const std::pair<Key, Value>& rhs) const noexcept
    {
        return lhs.first < rhs.first;
    }

    inline bool operator()(const std::pair<Key, Value>& lhs, const Key& rhs) const noexcept
    {
        return lhs.first < rhs;
    }

    inline bool operator()(const Key& lhs, const std::pair<Key, Value>& rhs) const noexcept
    {
        return lhs < rhs.first;
    }
};

template <typename Key, typename Value>
struct KeyEqual final
{
    inline bool operator()(const std::pair<Key, Value>& lhs,
                           const std::pair<Key, Value>& rhs) const noexcept
    {
        return lhs.first == rhs.first;
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__CUSTOM_COMPARATOR_HPP
