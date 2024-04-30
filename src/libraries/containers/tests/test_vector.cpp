#include <containers/vector.hpp>
#include <gtest/gtest.h>

using namespace containers;

// Test default constructor
TEST(VectorTest, DefaultConstructor)
{
    Vector<int> v;
    EXPECT_TRUE(v.empty());
    EXPECT_EQ(v.size(), 0);
}

// Test constructor with initial size
TEST(VectorTest, ConstructorWithSize)
{
    Vector<int> v(5);
    EXPECT_FALSE(v.empty());
    EXPECT_EQ(v.size(), 5);
    EXPECT_EQ(v.capacity(), 5);
}

// Test copy constructor
TEST(VectorTest, CopyConstructor)
{
    Vector<int> v1(5, 42);
    Vector<int> v2 = v1;
    EXPECT_EQ(v2.size(), 5);
    EXPECT_EQ(v2[0], 42);
    EXPECT_EQ(v2[4], 42);
}

// Test move constructor
TEST(VectorTest, MoveConstructor)
{
    Vector<int> v1(5, 42);
    Vector<int> v2 = std::move(v1);
    EXPECT_EQ(v2.size(), 5);
    EXPECT_EQ(v2.capacity(), 5);
    EXPECT_EQ(v1.size(), 0); // Assuming v1 is left in a valid but unspecified state
}

// Test copy assignment
TEST(VectorTest, CopyAssignment)
{
    Vector<int> v1(5, 42);
    Vector<int> v2;
    v2 = v1;
    EXPECT_EQ(v2.size(), 5);
    EXPECT_EQ(v2[0], 42);
}

// Test move assignment
TEST(VectorTest, MoveAssignment)
{
    Vector<int> v1(5, 42);
    Vector<int> v2;
    v2 = std::move(v1);
    EXPECT_EQ(v2.size(), 5);
    EXPECT_TRUE(v1.empty()); // Assuming v1 is left in a valid but unspecified state
}

// Test initializer list constructor
TEST(VectorTest, InitializerListConstructor)
{
    Vector<int> v = {1, 2, 3, 4, 5};
    EXPECT_EQ(v.size(), 5);
    EXPECT_EQ(v[0], 1);
    EXPECT_EQ(v[4], 5);
}

// Test access using at() with valid index
TEST(VectorTest, AccessWithAt)
{
    Vector<int> v = {10, 20, 30};
    EXPECT_EQ(v.at(1), 20);
    EXPECT_THROW(v.at(3), std::out_of_range);
}

// Test access using operator[]
TEST(VectorTest, AccessWithSquareBrackets)
{
    Vector<int> v = {10, 20, 30};
    EXPECT_EQ(v[0], 10);
    v[0] = 15;
    EXPECT_EQ(v[0], 15);
}

// Test front() and back()
TEST(VectorTest, FrontAndBack)
{
    Vector<int> v = {10, 20, 30};
    EXPECT_EQ(v.front(), 10);
    EXPECT_EQ(v.back(), 30);
    v.front() = 15;
    v.back() = 35;
    EXPECT_EQ(v.front(), 15);
    EXPECT_EQ(v.back(), 35);
    v.clear();
    EXPECT_THROW(v.front(), std::out_of_range);
    EXPECT_THROW(v.back(), std::out_of_range);
}

// Test capacity and resizing
TEST(VectorTest, CapacityAndResizing)
{
    Vector<int> v;
    v.reserve(100);
    EXPECT_GE(v.capacity(), 100);

    v.resize(50);
    EXPECT_EQ(v.size(), 50);
    EXPECT_GE(v.capacity(), 50);

    v.resize(150, 1);
    EXPECT_EQ(v.size(), 150);
    EXPECT_EQ(v[149], 1);
    EXPECT_GE(v.capacity(), 150);
}

// Test empty state
TEST(VectorTest, Empty)
{
    Vector<int> v;
    EXPECT_TRUE(v.empty());

    v.push_back(1);
    EXPECT_FALSE(v.empty());

    v.clear();
    EXPECT_TRUE(v.empty());
}

// Test push_back and emplace_back
TEST(VectorTest, PushAndEmplaceBack)
{
    Vector<int> v;
    v.push_back(10);
    v.push_back(20);
    EXPECT_EQ(v.size(), 2);
    EXPECT_EQ(v[1], 20);

    v.emplace_back(30);
    EXPECT_EQ(v.size(), 3);
    EXPECT_EQ(v[2], 30);
}

// Test insert
TEST(VectorTest, InsertSingleElement)
{
    Vector<int> v = {10, 30};
    auto it = v.insert(v.begin() + 1, 20);
    EXPECT_EQ(v.size(), 3);
    EXPECT_EQ(v[1], 20);
    EXPECT_EQ(*it, 20);

    it = v.insert(v.end(), 40);
    EXPECT_EQ(v[3], 40);
    EXPECT_EQ(*it, 40);
}

// Test erase
TEST(VectorTest, EraseElements)
{
    Vector<int> v = {10, 20, 30, 40, 50};
    v.erase(v.begin() + 1, v.begin() + 4);
    EXPECT_EQ(v.size(), 2);
    EXPECT_EQ(v[1], 50);
}

// Test clear
TEST(VectorTest, Clear)
{
    Vector<int> v;
    v.push_back(1);
    v.push_back(2);
    ASSERT_FALSE(v.empty());
    v.clear();
    EXPECT_TRUE(v.empty());
    EXPECT_EQ(v.size(), 0);
}

// Insert
TEST(VectorTest, InsertMoveElement)
{
    Vector<int> v;
    v.insert(v.begin(), 42);
    EXPECT_EQ(v.front(), 42);
}

TEST(VectorTest, InsertMultipleElements)
{
    Vector<int> v;
    v.insert(v.begin(), 3, 42);
    EXPECT_EQ(v.size(), 3);
    EXPECT_EQ(v.front(), 42);
    EXPECT_EQ(v[1], 42);
}

TEST(VectorTest, InsertRange)
{
    std::vector<int> nums = {1, 2, 3};
    Vector<int> v;
    v.insert(v.begin(), nums.begin(), nums.end());
    EXPECT_EQ(v.size(), 3);
    EXPECT_EQ(v[0], 1);
}

TEST(VectorTest, InsertInitializerList)
{
    Vector<int> v;
    v.insert(v.begin(), {1, 2, 3});
    EXPECT_EQ(v.size(), 3);
    EXPECT_EQ(v[2], 3);
}

// Emplace
TEST(VectorTest, EmplaceElement)
{
    Vector<int> v;
    v.emplace(v.begin(), 42);
    EXPECT_EQ(v.front(), 42);
    v.clear();
    v.emplace(v.end(), 42);
    EXPECT_EQ(v.back(), 42);
}

// Erase
TEST(VectorTest, EraseSingleElement)
{
    Vector<int> v;
    v.push_back(1);
    v.push_back(2);
    v.erase(v.begin());
    EXPECT_EQ(v.front(), 2);
}

TEST(VectorTest, EraseRange)
{
    Vector<int> v;
    v.push_back(1);
    v.push_back(2);
    v.push_back(3);
    v.erase(v.begin(), v.begin() + 2);
    EXPECT_EQ(v.front(), 3);
}

// push_back and emplace_back
TEST(VectorTest, PushBack)
{
    Vector<int> v;
    v.push_back(42);
    EXPECT_EQ(v.front(), 42);
}

TEST(VectorTest, EmplaceBack)
{
    Vector<int> v;
    v.emplace_back(42);
    EXPECT_EQ(v.front(), 42);
}

// pop_back
TEST(VectorTest, PopBack)
{
    Vector<int> v;
    v.push_back(1);
    v.push_back(2);
    v.pop_back();
    EXPECT_EQ(v.back(), 1);
}

// resize
TEST(VectorTest, Resize)
{
    Vector<int> v;
    v.resize(5, 42);
    EXPECT_EQ(v.size(), 5);
    EXPECT_EQ(v[4], 42);
}

// swap
TEST(VectorTest, Swap)
{
    Vector<int> v;
    Vector<int> other_vector;
    other_vector.push_back(99);
    v.swap(other_vector);
    EXPECT_EQ(v.front(), 99);
    EXPECT_TRUE(other_vector.empty());
}
