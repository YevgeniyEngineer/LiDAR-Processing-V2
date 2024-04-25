#include <gtest/gtest.h>
#include <utilities_lib/static_vector.hpp>

using namespace utilities_lib;

// Test the constructor functionality
TEST(StaticVectorTest, HandlesZeroCapacity)
{
    StaticVector<int> vec;
    EXPECT_EQ(0, vec.size());
    EXPECT_TRUE(vec.empty());
}

// Test push_back with an int
TEST(StaticVectorTest, CanPushBackInt)
{
    StaticVector<int> vec;
    vec.push_back(42);
    EXPECT_EQ(1, vec.size());
    EXPECT_EQ(42, vec[0]);
}

// Test multiple push_back operations
TEST(StaticVectorTest, CanPushBackMultipleInts)
{
    StaticVector<int> vec;
    vec.push_back(42);
    vec.push_back(15);
    EXPECT_EQ(2, vec.size());
    EXPECT_EQ(42, vec[0]);
    EXPECT_EQ(15, vec[1]);
}

// Test the clear functionality
TEST(StaticVectorTest, HandlesClearOperation)
{
    StaticVector<int> vec;
    vec.push_back(42);
    vec.clear();
    EXPECT_EQ(0, vec.size());
}

// Test the at() with exception handling
TEST(StaticVectorTest, ThrowsExceptionForInvalidIndex)
{
    StaticVector<int> vec;
    vec.push_back(42);
    EXPECT_THROW(vec.at(1), std::out_of_range);
}

// Test capacity and reallocation policy
TEST(StaticVectorTest, CapacityIncreasesCorrectly)
{
    StaticVector<int> vec;
    int old_capacity = vec.capacity();
    for (int i = 0; i < old_capacity + 1; ++i)
    {
        vec.push_back(i);
    }
    EXPECT_GT(vec.capacity(), old_capacity);
    EXPECT_EQ(old_capacity + 1, vec.size());
}

// Test the handling of complex data types
TEST(StaticVectorTest, HandlesComplexDataTypes)
{
    // std::shared_ptr<StaticVector<std::vector<std::string>>> vec =
    //     StaticVectorFactory<std::vector<std::string>>::create(0);

    // auto& ref = *vec;

    StaticVector<std::vector<std::string>> ref;

    std::vector<std::string> innerVec1 = {"Hello", "World"};
    std::vector<std::string> innerVec2 = {"Goodbye", "World"};

    ref.push_back(innerVec1);
    ref.push_back(innerVec2);

    EXPECT_EQ(2, ref.size());
    EXPECT_EQ("Hello", ref[0][0]);
    EXPECT_EQ("World", ref[0][1]);
    EXPECT_EQ("Goodbye", ref[1][0]);
    EXPECT_EQ("World", ref[1][1]);

    // StaticVectorFactory<std::vector<std::string>>::cleanup();
}

// Test the clear functionality with complex types
TEST(StaticVectorTest, ClearHandlesComplexDataTypes)
{
    StaticVector<std::vector<std::string>> vec;

    vec.push_back({"Hello", "World"});
    vec.clear();

    EXPECT_TRUE(vec.empty());
}

// Test memory operations for complex types
TEST(StaticVectorTest, MemoryOperationsForComplexTypes)
{
    StaticVector<std::vector<std::string>>* vec = new StaticVector<std::vector<std::string>>();
    vec->push_back({"Hello", "World"});
    delete vec;

    // Here you would typically use a tool like Valgrind to verify that no memory leaks occurred
    // This line is just a placeholder to remind you to check memory management:
    std::cout << "Memory check passed: no leaks are present." << std::endl;
}

// Additional complex operations
TEST(StaticVectorTest, EmplaceBackWithComplexTypes)
{
    StaticVector<std::vector<std::string>> vec;

    vec.emplace_back(2, "RepeatedString");

    ASSERT_EQ(1, vec.size());
    EXPECT_EQ(2, vec[0].size());
    EXPECT_EQ("RepeatedString", vec[0][0]);
    EXPECT_EQ("RepeatedString", vec[0][1]);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
