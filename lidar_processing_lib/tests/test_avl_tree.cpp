#include "lidar_processing_lib/avl_tree.hpp"

#include <gtest/gtest.h>

#include <random>
#include <unordered_set>

using namespace lidar_processing_lib;

class AVLTreeTest : public ::testing::Test
{
  protected:
    AVLTree<std::int32_t> tree;

    /// @brief Helper function to insert multiple values into the tree
    void insertValues(const std::vector<std::int32_t>& values)
    {
        for (const auto value : values)
        {
            tree.insert(value);
        }
    }

    void SetUp() override
    {
        tree.reserve(100000);
    }
};

// Test inserting and finding a single element
TEST_F(AVLTreeTest, InsertAndFindSingleElement)
{
    tree.insert(10);
    EXPECT_TRUE(tree.find(10));
    EXPECT_FALSE(tree.find(20));
}

// Test inserting and finding multiple elements
TEST_F(AVLTreeTest, InsertAndFindMultipleElements)
{
    const std::vector<int32_t> values = {10, 20, 30, 40, 50, 25};
    insertValues(values);

    for (const auto value : values)
    {
        EXPECT_TRUE(tree.find(value));
    }
    EXPECT_FALSE(tree.find(100));
}

// Test clearing the tree
TEST_F(AVLTreeTest, ClearTree)
{
    const std::vector<int32_t> values = {10, 20, 30, 40, 50, 25};
    insertValues(values);
    tree.clear();
    for (const auto value : values)
    {
        EXPECT_FALSE(tree.find(value));
    }
}

// Test inserting a large number of random values
TEST_F(AVLTreeTest, InsertRandomValues)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 999999);

    std::unordered_set<int32_t> inserted_values;
    for (std::size_t i = 0; i < 10000; ++i)
    {
        auto value = dis(gen);
        inserted_values.insert(value);
        tree.insert(value);
    }

    for (const auto value : inserted_values)
    {
        EXPECT_TRUE(tree.find(value));
    }
}
