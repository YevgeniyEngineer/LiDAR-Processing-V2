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

/// Local
#include "avl_tree.hpp"
#include "custom_comparator.hpp"

/// STL
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
template <typename Key, typename Value>
class StaticMap
{
  public:
    using KeyValuePair = std::pair<Key, Value>;
    using AVLTreeType = AVLTree<KeyValuePair, KeyComparator<Key, Value>>;

    inline void reserve(std::size_t capacity)
    {
        avl_tree_.reserve(capacity);
    }

    /// @brief Clear all elements from the map
    inline void clear() noexcept
    {
        avl_tree_.clear();
    }

    /// @brief Insert a value with given key
    void insert(const Key& key, const Value& value)
    {
        auto* data_ptr = avl_tree_.find({key, Value()});
        if (data_ptr)
        {
            data_ptr->second = value;
        }
        else
        {
            avl_tree_.insert({key, value});
        }
    }

    /// @brief Retrieve the value corresponding to the given key
    Value& at(const Key& key)
    {
        auto* ptr = avl_tree_.find({key, Value()});
        if (!ptr)
        {
            throw std::out_of_range("Key not found");
        }
        return ptr->second;
    }

    const Value& at(const Key& key) const
    {
        const auto* ptr = avl_tree_.find({key, Value()});
        if (!ptr)
        {
            throw std::out_of_range("Key not found");
        }
        return ptr->second;
    }

    Value& operator[](const Key& key) noexcept
    {
        auto* data_ptr = avl_tree_.find({key, Value()});
        if (!data_ptr)
        {
            data_ptr = avl_tree_.insert({key, Value()});
        }
        return data_ptr->second;
    }

    inline bool contains(const Key& key) const
    {
        return avl_tree_.find({key, Value()}) != nullptr;
    }

    inline bool find(const Key& key, Value& value) const
    {
        const auto* ptr = avl_tree_.find({key, Value()});
        if (ptr)
        {
            value = ptr->second;
            return true;
        }
        return false;
    }

    void printAll() const
    {
        avl_tree_.printInOrder();
    }

  private:
    AVLTreeType avl_tree_;
};

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STATIC_MAP_HPP
