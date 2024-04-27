#include <containers/vector.hpp>
#include <gtest/gtest.h>

using namespace containers;

// Test the default constructor
TEST(VectorTest, DefaultConstructor)
{
    Vector<int> v;
    EXPECT_TRUE(v.empty());
}

// Test construction from an initializer list
TEST(VectorTest, InitializerListConstructor)
{
    Vector<int> v = {1, 2, 3, 4, 5};
    EXPECT_EQ(v.size(), 5);
    EXPECT_EQ(v[2], 3);
}

// Test copy constructor
TEST(VectorTest, CopyConstructor)
{
    Vector<int> v1 = {1, 2, 3};
    Vector<int> v2(v1);
    EXPECT_EQ(v2.size(), 3);
    EXPECT_EQ(v2[1], 2);
}

// Test move constructor
TEST(VectorTest, MoveConstructor)
{
    Vector<int> v1 = {1, 2, 3};
    Vector<int> v2(std::move(v1));
    EXPECT_EQ(v2.size(), 3);
    EXPECT_TRUE(v1.empty()); // Assuming v1 is in a valid but unspecified state
}

// Test element access
TEST(VectorTest, ElementAccess)
{
    Vector<int> v = {1, 2, 3};
    EXPECT_EQ(v.at(1), 2);
    EXPECT_THROW(v.at(3), std::out_of_range);
    EXPECT_NO_THROW(v[2]);
}

// Test push_back and pop_back
TEST(VectorTest, PushAndPopBack)
{
    Vector<int> v;
    v.push_back(1);
    v.push_back(2);
    EXPECT_EQ(v.size(), 2);
    v.pop_back();
    EXPECT_EQ(v.size(), 1);
    EXPECT_EQ(v.front(), 1);
}

// Test capacity and resizing
TEST(VectorTest, CapacityAndResizing)
{
    Vector<int> v;
    v.reserve(10);
    EXPECT_GE(v.capacity(), 10);
    v.resize(5);
    EXPECT_EQ(v.size(), 5);
    v.resize(8, 100);
    EXPECT_EQ(v.size(), 8);
    EXPECT_EQ(v[7], 100);
}

// More tests can be added to cover all methods like `insert`, `erase`, `emplace_back`, etc.

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
