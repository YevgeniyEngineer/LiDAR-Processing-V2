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

// Local
#include "avl_tree.hpp"

// STL
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
class StaticSet final
{
  public:
    /// @brief Reserve nodes in AVL Tree
    inline void reserve(std::size_t num_total)
    {
        avl_tree_.reserve(num_total);
    }

    /// @brief Insert an element into the set
    inline void insert(const T& value)
    {
        avl_tree_.insert(value);
    }

    /// @brief Insert an element into the set
    inline void insert(T&& value)
    {
        avl_tree_.insert(std::move(value));
    }

    /// @brief Check if a value is included in the set
    inline bool contains(const T& value) const
    {
        return avl_tree_.find(value);
    }

    /// @brief Clear all values in the set
    inline void clear() noexcept
    {
        avl_tree_.clear();
    }

  private:
    AVLTree<std::int32_t> avl_tree_;
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STATIC_SET_HPP
