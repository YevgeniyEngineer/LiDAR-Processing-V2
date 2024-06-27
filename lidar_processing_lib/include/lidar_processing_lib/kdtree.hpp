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

#ifndef LIDAR_PROCESSING_LIB__KDTREE_HPP
#define LIDAR_PROCESSING_LIB__KDTREE_HPP

// Internal
#include "priority_queue.hpp"
#include "stack.hpp"

// STL
#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
template <typename T, std::uint8_t Dim>
using Point = std::array<T, Dim>;

template <typename T, std::uint8_t Dim>
class KDTree final
{
  public:
    using PointT = Point<T, Dim>;
    using KDTreeT = KDTree<T, Dim>;

    struct Neighbour final
    {
        std::uint32_t index;
        T distance;
    };

    struct Compare final
    {
        inline bool operator()(const Neighbour& a, const Neighbour& b) const noexcept
        {
            return a.distance < b.distance;
        }
    };

    KDTree& operator=(const KDTreeT& other) = delete;
    KDTree(const KDTreeT& other) = delete;
    KDTree& operator=(KDTreeT&& other) noexcept = default;
    KDTree(KDTreeT&& other) noexcept = default;

    KDTree(bool sort = false);

    void reserve(std::uint32_t num_pts = 200'000U);
    void rebuild(const std::vector<PointT>& points);
    void k_nearest(const PointT& target, std::uint32_t num_neigh, std::vector<Neighbour>& neigh);
    void radius_search(const PointT& target, T proximity_sqr, std::vector<Neighbour>& neigh);
    void radius_search_k_nearest(const PointT& target,
                                 T proximity_sqr,
                                 std::uint32_t num_neigh,
                                 std::vector<Neighbour>& neigh);

    constexpr T dist_sqr(const PointT& a, const PointT& b) noexcept;

  private:
    struct Node final
    {
        PointT point;
        std::uint32_t index; // point index in the input cloud
        Node* left;
        Node* right;
    };

    struct RebuildStack final
    {
        typename std::vector<Node>::iterator begin;
        typename std::vector<Node>::iterator end;
        Node** node_ref;
        std::uint32_t depth;
    };

    struct KNearestStack final
    {
        const Node* node_ptr;
        std::uint32_t index;
        bool first_visit;
    };

    struct RadiusSearchStack final
    {
        const Node* node_ptr;
        std::uint32_t index;
    };

    bool sort_;

    Node* root_{nullptr};
    std::vector<Node> nodes_;
    lidar_processing_lib::Stack<RebuildStack> rebuild_stack_;
    lidar_processing_lib::Stack<KNearestStack> k_nearest_stack_;
    lidar_processing_lib::Stack<RadiusSearchStack> radius_search_stack_;
    lidar_processing_lib::PriorityQueue<Neighbour, Compare> min_heap_;

    template <std::uint8_t N = 0>
    constexpr T dist_sqr_impl(const PointT& a, const PointT& b) noexcept;
};

template <typename T, std::uint8_t Dim>
KDTree<T, Dim>::KDTree(bool sort) : sort_(sort)
{
    reserve();
}

template <typename T, std::uint8_t Dim>
template <std::uint8_t N>
inline constexpr T KDTree<T, Dim>::dist_sqr_impl(const PointT& a, const PointT& b) noexcept
{
    if constexpr (N < Dim)
    {
        return (a[N] - b[N]) * (a[N] - b[N]) + dist_sqr_impl<N + 1>(a, b);
    }
    else
    {
        return 0;
    }
}

template <typename T, std::uint8_t Dim>
inline constexpr T KDTree<T, Dim>::dist_sqr(const PointT& a, const PointT& b) noexcept
{
    return dist_sqr_impl(a, b);
}

template <typename T, std::uint8_t Dim>
inline void KDTree<T, Dim>::reserve(std::uint32_t num_pts)
{
    nodes_.reserve(num_pts);
    rebuild_stack_.reserve(num_pts);
    k_nearest_stack_.reserve(num_pts);
    radius_search_stack_.reserve(num_pts);
    min_heap_.reserve(num_pts);
}

template <typename T, std::uint8_t Dim>
inline void KDTree<T, Dim>::rebuild(const std::vector<PointT>& points)
{
    nodes_.clear();
    root_ = nullptr;

    if (points.empty())
    {
        return;
    }

    nodes_.reserve(points.size());
    for (std::uint32_t i = 0U; i < points.size(); ++i)
    {
        nodes_.push_back({points[i], i, nullptr, nullptr});
    }

    rebuild_stack_.push({nodes_.begin(), nodes_.end(), &root_, 0U});

    while (rebuild_stack_.size() > 0U)
    {
        const auto [begin, end, node_ref, depth] = rebuild_stack_.top();
        rebuild_stack_.try_pop();

        if (begin >= end)
        {
            continue;
        }

        const auto axis = depth % Dim;
        auto mid = begin + (end - begin) / 2;

        std::nth_element(begin, mid, end, [axis](const Node& a, const Node& b) noexcept -> bool {
            return a.point[axis] < b.point[axis];
        });

        *node_ref = &(*mid);

        if (mid > begin)
        {
            rebuild_stack_.push({begin, mid, &((*node_ref)->left), depth + 1});
        }

        if ((mid + 1) < end)
        {
            rebuild_stack_.push({mid + 1, end, &((*node_ref)->right), depth + 1});
        }
    }

    if (root_ == nullptr)
    {
        throw std::runtime_error("KDTree is empty");
    }
}

template <typename T, std::uint8_t Dim>
inline void KDTree<T, Dim>::k_nearest(const PointT& target,
                                      std::uint32_t num_neigh,
                                      std::vector<Neighbour>& neigh)
{
    neigh.clear();

    if ((num_neigh == 0) || (root_ == nullptr))
    {
        return;
    }

    k_nearest_stack_.push({root_, 0U, true});

    while (k_nearest_stack_.size() > 0U)
    {
        const auto [node, index, first_visit] = k_nearest_stack_.top();
        k_nearest_stack_.try_pop();

        if (node == nullptr)
        {
            continue;
        }

        if (first_visit)
        {
            const auto dist = dist_sqr(target, node->point);

            if (min_heap_.size() < num_neigh)
            {
                min_heap_.push_back({node->index, dist});
            }
            else if (dist < min_heap_.top().distance)
            {
                min_heap_.try_pop();
                min_heap_.push_back({node->index, dist});
            }

            const auto next_index = (index + 1U) % Dim;
            const auto delta = node->point[index] - target[index];
            const auto next_branch = (delta > 0) ? node->left : node->right;

            k_nearest_stack_.push({node, index, false});
            k_nearest_stack_.push({next_branch, next_index, true});
        }
        else
        {
            const auto delta = node->point[index] - target[index];
            const auto next_index = (index + 1U) % Dim;
            const auto opposite_branch = (delta > 0) ? node->right : node->left;

            if ((delta * delta <= min_heap_.top().distance) || (min_heap_.size() < num_neigh))
            {
                k_nearest_stack_.push({opposite_branch, next_index, true});
            }
        }
    }

    neigh.reserve(min_heap_.size());
    while (min_heap_.size() > 0U)
    {
        neigh.push_back(min_heap_.top());
        min_heap_.try_pop();
    }
    std::reverse(neigh.begin(), neigh.end());
}

template <typename T, std::uint8_t Dim>
inline void KDTree<T, Dim>::radius_search(const PointT& target,
                                          T proximity_sqr,
                                          std::vector<Neighbour>& neigh)
{
    neigh.clear();

    if (root_ == nullptr)
    {
        return;
    }

    radius_search_stack_.push({root_, 0U});

    while (radius_search_stack_.size() > 0U)
    {
        const auto [node, index] = radius_search_stack_.top();
        radius_search_stack_.try_pop();

        if (node == nullptr)
        {
            continue;
        }

        const auto dist = dist_sqr(target, node->point);
        if (dist <= proximity_sqr)
        {
            neigh.push_back({node->index, dist});
        }

        const auto next_index = (index + 1U) % Dim;
        const auto delta = node->point[index] - target[index];
        const auto abs_delta_sqr = delta * delta;

        if (abs_delta_sqr <= proximity_sqr)
        {
            radius_search_stack_.push({node->right, next_index});
            radius_search_stack_.push({node->left, next_index});
        }
        else
        {
            const auto next_branch = (delta > 0) ? node->left : node->right;
            radius_search_stack_.push({next_branch, next_index});
        }
    }

    if (sort_)
    {
        std::sort(neigh.begin(),
                  neigh.end(),
                  [](const Neighbour& a, const Neighbour& b) noexcept -> bool {
                      return (a.distance < b.distance);
                  });
    }
}

template <typename T, std::uint8_t Dim>
inline void KDTree<T, Dim>::radius_search_k_nearest(const PointT& target,
                                                    T proximity_sqr,
                                                    std::uint32_t num_neigh,
                                                    std::vector<Neighbour>& neigh)
{
    neigh.clear();
    if (root_ == nullptr)
    {
        return;
    }

    radius_search_stack_.push({root_, 0U});

    while (radius_search_stack_.size() > 0U)
    {
        const auto [node, index] = radius_search_stack_.top();
        radius_search_stack_.try_pop();

        if (node == nullptr)
        {
            continue;
        }

        const auto dist = dist_sqr(target, node->point);
        if (dist <= proximity_sqr)
        {
            neigh.push_back({node->index, dist});
            if (neigh.size() >= num_neigh)
            {
                break;
            }
        }

        const auto next_index = (index + 1U) % Dim;
        const auto delta = node->point[index] - target[index];
        const auto abs_delta_sqr = delta * delta;

        if (abs_delta_sqr <= proximity_sqr)
        {
            radius_search_stack_.push({node->right, next_index});
            radius_search_stack_.push({node->left, next_index});
        }
        else
        {
            const auto next_branch = (delta > 0) ? node->left : node->right;
            radius_search_stack_.push({next_branch, next_index});
        }
    }

    if (sort_)
    {
        std::sort(neigh.begin(),
                  neigh.end(),
                  [](const Neighbour& a, const Neighbour& b) noexcept -> bool {
                      return (a.distance < b.distance);
                  });
    }
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__KDTREE_HPP
