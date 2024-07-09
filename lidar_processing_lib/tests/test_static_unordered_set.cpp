#include "lidar_processing_lib/static_unordered_set.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace lidar_processing_lib;

class StaticUnorderedSetTest : public ::testing::Test
{
  protected:
    StaticUnorderedSet<int> set_;

    void SetUp() override
    {
        set_.reserve(100, 100); // Reserving space for 100 buckets and 100 elements
    }
};

TEST_F(StaticUnorderedSetTest, InsertAndContains)
{
    set_.insert(1);
    set_.insert(2);
    set_.insert(3);

    EXPECT_TRUE(set_.contains(1));
    EXPECT_TRUE(set_.contains(2));
    EXPECT_TRUE(set_.contains(3));
    EXPECT_FALSE(set_.contains(4));
}

TEST_F(StaticUnorderedSetTest, InsertDuplicate)
{
    set_.insert(1);
    set_.insert(1);

    EXPECT_TRUE(set_.contains(1));
    EXPECT_EQ(set_.size(), 1); // Ensure only one instance of the element is inserted
}

TEST_F(StaticUnorderedSetTest, ClearSet)
{
    set_.insert(1);
    set_.insert(2);
    set_.insert(3);

    set_.clear();

    EXPECT_FALSE(set_.contains(1));
    EXPECT_FALSE(set_.contains(2));
    EXPECT_FALSE(set_.contains(3));
    EXPECT_EQ(set_.size(), 0);
}

TEST_F(StaticUnorderedSetTest, Find)
{
    set_.insert(1);
    set_.insert(2);
    set_.insert(3);

    EXPECT_TRUE(set_.find(1));
    EXPECT_TRUE(set_.find(2));
    EXPECT_TRUE(set_.find(3));
    EXPECT_FALSE(set_.find(4));
}

TEST_F(StaticUnorderedSetTest, RandomInserts)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 1000);

    for (int i = 0; i < 100; ++i)
    {
        int key = dis(gen);
        set_.insert(key);
        EXPECT_TRUE(set_.contains(key));
    }
}

TEST_F(StaticUnorderedSetTest, ExceedCapacity)
{
    for (int i = 0; i < 100; ++i)
    {
        set_.insert(i);
    }

    EXPECT_THROW(set_.insert(101), std::overflow_error);
}
