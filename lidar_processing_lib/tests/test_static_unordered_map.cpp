#include "lidar_processing_lib/static_unordered_map.hpp"

#include <gtest/gtest.h>

#include <random>
#include <unordered_map>

using namespace lidar_processing_lib;

class StaticUnorderedMapTest : public ::testing::Test
{
  protected:
    StaticUnorderedMap<std::int32_t, std::int32_t> map_;

    void SetUp() override
    {
        static constexpr std::size_t NUM_BUCKETS = 10;
        static constexpr std::size_t NUM_ELEMENTS = 20;

        map_.reserve(NUM_BUCKETS, NUM_ELEMENTS);
    }
};

TEST_F(StaticUnorderedMapTest, InsertAndLookup)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    EXPECT_EQ(map_.at(1), 100);
    EXPECT_EQ(map_.at(2), 200);
    EXPECT_EQ(map_.at(3), 300);

    EXPECT_THROW(map_.at(4), std::out_of_range);
}

TEST_F(StaticUnorderedMapTest, Contains)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    EXPECT_TRUE(map_.contains(1));
    EXPECT_TRUE(map_.contains(2));
    EXPECT_TRUE(map_.contains(3));
    EXPECT_FALSE(map_.contains(4));
}

TEST_F(StaticUnorderedMapTest, OperatorSquareBrackets)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    EXPECT_EQ(map_[1], 100);
    EXPECT_EQ(map_[2], 200);
    EXPECT_EQ(map_[3], 300);

    map_[1] = 101;
    map_[2] = 202;
    map_[3] = 303;

    EXPECT_EQ(map_[1], 101);
    EXPECT_EQ(map_[2], 202);
    EXPECT_EQ(map_[3], 303);
}

TEST_F(StaticUnorderedMapTest, Clear)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    EXPECT_EQ(map_.size(), 3);
    map_.clear();
    EXPECT_EQ(map_.size(), 0);

    EXPECT_FALSE(map_.contains(1));
    EXPECT_FALSE(map_.contains(2));
    EXPECT_FALSE(map_.contains(3));
    EXPECT_THROW(map_.at(1), std::out_of_range);
    EXPECT_THROW(map_.at(2), std::out_of_range);
    EXPECT_THROW(map_.at(3), std::out_of_range);
}

TEST_F(StaticUnorderedMapTest, Rehash)
{
    for (std::int32_t i = 0; i < 15; ++i)
    {
        map_.insert(i, i * 10);
    }

    for (std::int32_t i = 0; i < 15; ++i)
    {
        EXPECT_EQ(map_.at(i), i * 10);
    }
}

TEST_F(StaticUnorderedMapTest, Find)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    std::int32_t value;
    EXPECT_TRUE(map_.find(1, value));
    EXPECT_EQ(value, 100);
    EXPECT_TRUE(map_.find(2, value));
    EXPECT_EQ(value, 200);
    EXPECT_TRUE(map_.find(3, value));
    EXPECT_EQ(value, 300);
    EXPECT_FALSE(map_.find(4, value));
}

TEST_F(StaticUnorderedMapTest, RandomInserts)
{
    map_.reserve(10000, 10000);

    std::unordered_map<std::int32_t, std::int32_t> reference_map;
    reference_map.reserve(1000);

    for (std::int32_t i = 0; i < 1000; ++i)
    {
        std::int32_t key = rand() % 1000;
        std::int32_t value = rand() % 1000;
        map_.insert(key, value);
        reference_map[key] = value;
        EXPECT_EQ(map_.at(key), value);
    }

    for (const auto& [key, value] : reference_map)
    {
        std::int32_t map_value;
        EXPECT_TRUE(map_.find(key, map_value));
        EXPECT_EQ(map_value, value);
    }
}
