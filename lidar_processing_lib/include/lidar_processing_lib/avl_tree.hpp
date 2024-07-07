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
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
class AVLTree final
{
  public:
    /// @brief Insert a value into the tree
    void insert(std::int32_t value)
    {
        root_ = insert(std::move(root_), value);
    }

    /// @brief Public interface to search for a value
    bool find(std::int32_t value) const
    {
        return find(root_.get(), value);
    }

    /// @brief Public interface to clear the tree
    void clear()
    {
        root_.reset();
    }

    /// @brief Print all values in the tree
    void printInOrder() const
    {
        printInOrder(root_.get());
        std::cerr << std::endl;
    }

  private:
    struct Node
    {
        std::int32_t data;
        std::int32_t height = 1;
        std::unique_ptr<Node> left = nullptr;
        std::unique_ptr<Node> right = nullptr;

        Node(std::int32_t value) : data(value)
        {
        }
    };

    std::unique_ptr<Node> root_;

    // Print all value (in-order traversal) for debugging
    void printInOrder(const Node* const node) const
    {
        if (node)
        {
            printInOrder(node->left.get());
            std::cerr << node->data << " ";
            printInOrder(node->right.get());
        }
    }

    /// @brief Helper function to get the height of a node
    inline auto height(const std::unique_ptr<Node>& node) const noexcept
    {
        return node ? node->height : 0;
    }

    /// @brief Helper function to get the balance factor of a node
    inline auto getBalance(const std::unique_ptr<Node>& node) const noexcept
    {
        return node ? height(node->left) - height(node->right) : 0;
    }

    /// @brief Right rotate the subtree rooted with y
    std::unique_ptr<Node> rightRotate(std::unique_ptr<Node> y) const
    {
        std::unique_ptr<Node> x = std::move(y->left);
        std::unique_ptr<Node> T2 = std::move(x->right);

        x->right = std::move(y);
        x->right->left = std::move(T2);
        x->right->height = std::max(height(x->right->left), height(x->right->right)) + 1;
        x->height = std::max(height(x->left), height(x->right)) + 1;

        return x;
    }

    /// @brief Left rotate the subtree rooted with x
    std::unique_ptr<Node> leftRotate(std::unique_ptr<Node> x) const
    {
        std::unique_ptr<Node> y = std::move(x->right);
        std::unique_ptr<Node> T2 = std::move(y->left);

        y->left = std::move(x);
        y->left->right = std::move(T2);

        y->left->height = std::max(height(y->left->left), height(y->left->right)) + 1;
        y->height = std::max(height(y->left), height(y->right)) + 1;

        return y;
    }

    /// @brief Helper function to insert a new value into the subtree rooted with node
    std::unique_ptr<Node> insert(std::unique_ptr<Node> node, std::int32_t value) const
    {
        if (!node)
        {
            return std::make_unique<Node>(value);
        }

        if (value < node->data)
        {
            node->left = insert(std::move(node->left), value);
        }
        else if (value > node->data)
        {
            node->right = insert(std::move(node->right), value);
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
            return rightRotate(std::move(node));
        }

        // Right right case
        if (balance < -1 && value > node->right->data)
        {
            return leftRotate(std::move(node));
        }

        // Left right case
        if (balance > 1 && value > node->left->data)
        {
            node->left = leftRotate(std::move(node->left));
            return rightRotate(std::move(node));
        }

        // Right left case
        if (balance < -1 && value < node->right->data)
        {
            node->right = rightRotate(std::move(node->right));
            return leftRotate(std::move(node));
        }

        return node;
    }

    /// @brief Helper function to search a value in the subtree rooted with node
    bool find(const Node* const node, std::int32_t value) const
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
            return find(node->left.get(), value);
        }
        else
        {
            return find(node->right.get(), value);
        }
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__AVL_TREE_HPP
