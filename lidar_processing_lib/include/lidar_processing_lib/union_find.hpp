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

#ifndef LIDAR_PROCESSING_LIB__UNION_FIND_HPP
#define LIDAR_PROCESSING_LIB__UNION_FIND_HPP

// Internal
#include "vector.hpp"

// STL
#include <algorithm>
#include <cstdint>
#include <vector>

namespace lidar_processing_lib
{
class UnionFind final
{
  public:
    UnionFind(std::int32_t size, std::int32_t child_capacity);
    std::int32_t find(std::int32_t element);
    void unite(std::int32_t element_1, std::int32_t element_2);
    void reserve(std::int32_t capacity, std::int32_t child_capacity);
    void reset(std::int32_t size);
    std::int32_t size() const noexcept;
    const std::vector<std::int32_t>& get_set_elements(std::int32_t x);

  private:
    // parent_[i] points to the parent of i
    std::vector<std::int32_t> parent_;

    // rank_[i] is the rank (approximate tree height) of the tree rooted at i
    std::vector<std::int32_t> rank_;

    // stores connected components of multiple disjoint sets
    lidar_processing_lib::Vector<std::vector<std::int32_t>> set_elements_;

    // stores child capacity
    std::int32_t child_capacity_;
};

inline UnionFind::UnionFind(std::int32_t size, std::int32_t child_capacity)
    : child_capacity_{child_capacity}
{
    reserve(size, child_capacity);
    reset(size);
}

inline std::int32_t UnionFind::find(std::int32_t element)
{
    if (parent_[element] != element)
    {
        parent_[element] = find(parent_[element]); // Path compression
    }
    return parent_[element];
}

inline void UnionFind::unite(std::int32_t element_1, std::int32_t element_2)
{
    const auto root_1 = find(element_1);
    const auto root_2 = find(element_2);

    if (root_1 != root_2)
    {
        if (rank_[root_1] > rank_[root_2])
        {
            parent_[root_2] = root_1;
            auto& set_1 = set_elements_[root_1];
            auto& set_2 = set_elements_[root_2];
            set_1.insert(set_1.end(), set_2.begin(), set_2.end());
            std::stable_sort(set_1.begin(), set_1.end());
            set_1.erase(std::unique(set_1.begin(), set_1.end()), set_1.end());
            set_2.clear();
        }
        else if (rank_[root_1] < rank_[root_2])
        {
            parent_[root_1] = root_2;
            auto& set_1 = set_elements_[root_1];
            auto& set_2 = set_elements_[root_2];
            set_2.insert(set_2.end(), set_1.begin(), set_1.end());
            std::stable_sort(set_2.begin(), set_2.end());
            set_2.erase(std::unique(set_2.begin(), set_2.end()), set_2.end());
            set_1.clear();
        }
        else
        {
            parent_[root_2] = root_1;
            ++rank_[root_1];
            auto& set_1 = set_elements_[root_1];
            auto& set_2 = set_elements_[root_2];
            set_1.insert(set_1.end(), set_2.begin(), set_2.end());
            std::stable_sort(set_1.begin(), set_1.end());
            set_1.erase(std::unique(set_1.begin(), set_1.end()), set_1.end());
            set_2.clear();
        }
    }
}

inline void UnionFind::reserve(std::int32_t capacity, std::int32_t child_capacity)
{
    if (child_capacity > child_capacity_)
    {
        child_capacity_ = child_capacity;
    }
    parent_.reserve(capacity);
    rank_.reserve(capacity);
    set_elements_.resize(capacity);
    for (auto& set_element : set_elements_)
    {
        set_element.reserve(child_capacity);
    }
}

inline void UnionFind::reset(std::int32_t size)
{
    rank_.assign(size, 0);
    parent_.resize(size);
    set_elements_.resize(size);
    for (std::int32_t i = 0; i < size; ++i)
    {
        parent_[i] = i;
        set_elements_[i].clear();
        set_elements_[i].push_back(i);
    }
}

inline std::int32_t UnionFind::size() const noexcept
{
    return parent_.size();
}

inline const std::vector<std::int32_t>& UnionFind::get_set_elements(std::int32_t x)
{
    return set_elements_[find(x)];
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__UNION_FIND_HPP
