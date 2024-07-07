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
#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
template <typename T>
class AVLTree final
{
    static_assert(std::is_integral<T>::value, "AVLTree requires an integral type");

  public:
    /// @brief Reserve memory for the node pool
    void reserve(std::size_t capacity)
    {
        node_pool_.reserve(capacity);
    }

    /// @brief Insert a value into the tree
    void insert(T value)
    {
        root_ = insert(root_, value);
    }

    /// @brief Public interface to search for a value
    bool find(T value) const
    {
        return find(root_, value);
    }

    /// @brief Public interface to clear the tree
    void clear() noexcept
    {
        root_ = nullptr;
        node_pool_.clear();
    }

    /// @brief Print all values in the tree
    void printInOrder() const
    {
        printInOrder(root_);
        std::cerr << std::endl;
    }

  private:
    struct Node final
    {
        T data;
        std::int32_t height;
        Node* left;
        Node* right;

        Node(T data, std::int32_t height = 1, Node* left = nullptr, Node* right = nullptr)
            : data(data), height(height), left(left), right(right)
        {
        }
    };

    std::vector<std::unique_ptr<Node>> node_pool_;
    Node* root_ = nullptr;

    // Print all value (in-order traversal) for debugging
    void printInOrder(const Node* const node) const
    {
        if (node)
        {
            printInOrder(node->left);
            std::cerr << node->data << " ";
            printInOrder(node->right);
        }
    }

    /// @brief Helper function to get the height of a node
    inline auto height(const Node* const node) const noexcept
    {
        return node ? node->height : 0;
    }

    /// @brief Helper function to get the balance factor of a node
    inline auto getBalance(const Node* const node) const noexcept
    {
        return node ? height(node->left) - height(node->right) : 0;
    }

    /// @brief Right rotate the subtree rooted with y
    Node* rightRotate(Node* y) const
    {
        Node* x = y->left;
        Node* T2 = x->right;

        x->right = y;
        y->left = T2;

        y->height = std::max(height(y->left), height(y->right)) + 1;
        x->height = std::max(height(x->left), height(x->right)) + 1;

        return x;
    }

    /// @brief Left rotate the subtree rooted with x
    Node* leftRotate(Node* x) const
    {
        Node* y = x->right;
        Node* T2 = y->left;

        y->left = x;
        x->right = T2;

        x->height = std::max(height(x->left), height(x->right)) + 1;
        y->height = std::max(height(y->left), height(y->right)) + 1;

        return y;
    }

    /// @brief Helper function to insert a new value into the subtree rooted with node
    Node* insert(Node* node, T value)
    {
        if (!node)
        {
            node_pool_.emplace_back(std::make_unique<Node>(value));
            return node_pool_.back().get();
        }

        if (value < node->data)
        {
            node->left = insert(node->left, value);
        }
        else if (value > node->data)
        {
            node->right = insert(node->right, value);
        }
        else
        {
            // Duplicate values are not allowed in BST
            return node;
        }

        node->height = 1 + std::max(height(node->left), height(node->right));

        const auto balance = getBalance(node);

        // Left left case
        if (balance > 1 && value < node->left->data)
        {
            return rightRotate(node);
        }

        // Right right case
        if (balance < -1 && value > node->right->data)
        {
            return leftRotate(node);
        }

        // Left right case
        if (balance > 1 && value > node->left->data)
        {
            node->left = leftRotate(node->left);
            return rightRotate(node);
        }

        // Right left case
        if (balance < -1 && value < node->right->data)
        {
            node->right = rightRotate(node->right);
            return leftRotate(node);
        }

        return node;
    }

    /// @brief Helper function to search a value in the subtree rooted with node
    bool find(const Node* const node, T value) const
    {
        if (!node)
        {
            return false;
        }

        if (value == node->data)
        {
            return true;
        }
        else if (value < node->data)
        {
            return find(node->left, value);
        }
        else
        {
            return find(node->right, value);
        }
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__AVL_TREE_HPP
