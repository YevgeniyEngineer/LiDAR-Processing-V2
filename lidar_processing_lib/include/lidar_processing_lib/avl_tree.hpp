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

#ifndef LIDAR_PROCESSING_LIB__AVL_TREE_HPP
#define LIDAR_PROCESSING_LIB__AVL_TREE_HPP

// STL
#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
template <typename T, typename Compare = std::less<T>>
class AVLTree final
{
    static_assert(std::is_default_constructible_v<T>,
                  "AVLTree requires default constructible type");
    static_assert(std::is_copy_constructible_v<T>, "AVLTree requires copy constructible type");
    static_assert(std::is_copy_assignable_v<T>, "AVLTree requires copy assignable type");

    struct Node final
    {
        T data;
        std::int32_t height;
        std::int32_t left;
        std::int32_t right;

        Node() : height(1), left(-1), right(-1)
        {
        }

        Node(const T& data,
             std::int32_t height = 1,
             std::int32_t left = -1,
             std::int32_t right = -1)
            : data(data), height(height), left(left), right(right)
        {
        }

        Node(T&& data, std::int32_t height = 1, std::int32_t left = -1, std::int32_t right = -1)
            : data(std::move(data)), height(height), left(left), right(right)
        {
        }

        Node(const Node&) = default;
        Node(Node&&) noexcept = default;
        Node& operator=(const Node&) = default;
        Node& operator=(Node&&) noexcept = default;

        void swap(Node& other) noexcept
        {
            std::swap(data, other.data);
            std::swap(height, other.height);
            std::swap(left, other.left);
            std::swap(right, other.right);
        }
    };

  public:
    /// @brief Reserve memory for the node pool
    inline void reserve(std::size_t capacity)
    {
        node_pool_.reserve(capacity);
    }

    /// @brief Insert a value into the tree and return a pointer to the inserted value
    T* insert(T&& value)
    {
        root_ = insertImpl(root_, std::forward<T>(value));
        return &node_pool_[root_].data;
    }

    /// @brief Insert a value into the tree and return a pointer to the inserted value
    T* insert(const T& value)
    {
        root_ = insertImpl(root_, value);
        return &node_pool_[root_].data;
    }

    /// @brief Search for a value
    T* find(const T& value) noexcept
    {
        const auto node_idx = findImpl(root_, value);
        if (node_idx != -1)
        {
            return &node_pool_[node_idx].data;
        }
        return nullptr;
    }

    /// @brief Search for a value
    inline const T* find(const T& value) const noexcept
    {
        const auto node_idx = findImpl(root_, value);
        if (node_idx != -1)
        {
            return &node_pool_[node_idx].data;
        }
        return nullptr;
    }

    /// @brief Public interface to clear the tree
    inline void clear() noexcept
    {
        root_ = -1;
        next_free_index_ = 0;
    }

    /// @brief Print all values in the tree
    inline void printInOrder() const
    {
        printInOrder(root_);
        std::cerr << std::endl;
    }

  private:
    std::vector<Node> node_pool_;
    std::int32_t next_free_index_ = 0;
    std::int32_t root_ = -1;
    static constexpr Compare comp_{};

    // Print all value (in-order traversal) for debugging
    void printInOrder(std::int32_t node_idx) const
    {
        if (node_idx != -1)
        {
            printInOrder(node_pool_[node_idx].left);
            std::cerr << node_pool_[node_idx].data << " ";
            printInOrder(node_pool_[node_idx].right);
        }
    }

    /// @brief Helper function to get the height of a node
    inline auto height(std::int32_t node_idx) const noexcept
    {
        return node_idx != -1 ? node_pool_[node_idx].height : 0;
    }

    /// @brief Helper function to get the balance factor of a node
    inline auto getBalance(std::int32_t node_idx) const noexcept
    {
        return node_idx != -1
                   ? height(node_pool_[node_idx].left) - height(node_pool_[node_idx].right)
                   : 0;
    }

    /// @brief Right rotate the subtree rooted with y
    inline std::int32_t rightRotate(std::int32_t y_idx) noexcept
    {
        const std::int32_t x_idx = node_pool_[y_idx].left;
        const std::int32_t T2_idx = node_pool_[x_idx].right;

        node_pool_[x_idx].right = y_idx;
        node_pool_[y_idx].left = T2_idx;

        node_pool_[y_idx].height =
            std::max(height(node_pool_[y_idx].left), height(node_pool_[y_idx].right)) + 1;
        node_pool_[x_idx].height =
            std::max(height(node_pool_[x_idx].left), height(node_pool_[x_idx].right)) + 1;

        return x_idx;
    }

    // @brief Left rotate the subtree rooted with x
    inline std::int32_t leftRotate(std::int32_t x_idx) noexcept
    {
        const std::int32_t y_idx = node_pool_[x_idx].right;
        const std::int32_t T2_idx = node_pool_[y_idx].left;

        node_pool_[y_idx].left = x_idx;
        node_pool_[x_idx].right = T2_idx;

        node_pool_[x_idx].height =
            std::max(height(node_pool_[x_idx].left), height(node_pool_[x_idx].right)) + 1;
        node_pool_[y_idx].height =
            std::max(height(node_pool_[y_idx].left), height(node_pool_[y_idx].right)) + 1;

        return y_idx;
    }

    /// @brief Helper function to insert a new value into the subtree rooted with node
    template <typename U>
    std::int32_t insertImpl(std::int32_t node_idx, U&& value)
    {
        if (node_idx == -1)
        {
            if (next_free_index_ >= static_cast<std::int32_t>(node_pool_.size()))
            {
                node_pool_.resize(node_pool_.size() * 2 + 1);
            }
            const auto new_node_idx = next_free_index_++;
            node_pool_[new_node_idx] = Node(std::forward<U>(value));
            return new_node_idx;
        }

        if (comp_(value, node_pool_[node_idx].data))
        {
            node_pool_[node_idx].left =
                insertImpl(node_pool_[node_idx].left, std::forward<U>(value));
        }
        else if (comp_(node_pool_[node_idx].data, value))
        {
            node_pool_[node_idx].right =
                insertImpl(node_pool_[node_idx].right, std::forward<U>(value));
        }
        else
        {
            // Duplicate values are not allowed in BST
            return node_idx;
        }

        node_pool_[node_idx].height =
            1 + std::max(height(node_pool_[node_idx].left), height(node_pool_[node_idx].right));

        const auto balance = getBalance(node_idx);

        if (balance > 1)
        {
            if (comp_(value, node_pool_[node_pool_[node_idx].left].data))
            {
                return rightRotate(node_idx);
            }
            else
            {
                node_pool_[node_idx].left = leftRotate(node_pool_[node_idx].left);
                return rightRotate(node_idx);
            }
        }

        if (balance < -1)
        {
            if (comp_(node_pool_[node_pool_[node_idx].right].data, value))
            {
                return leftRotate(node_idx);
            }
            else
            {
                node_pool_[node_idx].right = rightRotate(node_pool_[node_idx].right);
                return leftRotate(node_idx);
            }
        }

        return node_idx;
    }

    /// @brief Helper function to search a value in the subtree rooted with node
    std::int32_t findImpl(std::int32_t node_idx, const T& value) const noexcept
    {
        while (node_idx != -1)
        {
            if (comp_(value, node_pool_[node_idx].data))
            {
                node_idx = node_pool_[node_idx].left;
            }
            else if (comp_(node_pool_[node_idx].data, value))
            {
                node_idx = node_pool_[node_idx].right;
            }
            else
            {
                return node_idx;
            }
        }
        return -1;
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__AVL_TREE_HPP
