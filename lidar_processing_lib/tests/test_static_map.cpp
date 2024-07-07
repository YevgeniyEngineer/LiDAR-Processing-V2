#include "lidar_processing_lib/static_map.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace lidar_processing_lib;

class StaticMapTest : public ::testing::Test
{
  protected:
    StaticMap<int, int> map_;

    void SetUp() override
    {
        map_.reserve(10000);
    }

    void TearDown() override
    {
        map_.clear();
    }
};

TEST_F(StaticMapTest, InsertAndRetrieve)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    EXPECT_EQ(map_.at(1), 100);
    EXPECT_EQ(map_.at(2), 200);
    EXPECT_EQ(map_.at(3), 300);

    EXPECT_EQ(map_[1], 100);
    EXPECT_EQ(map_[2], 200);
    EXPECT_EQ(map_[3], 300);
}

TEST_F(StaticMapTest, Contains)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    EXPECT_TRUE(map_.contains(1));
    EXPECT_TRUE(map_.contains(2));
    EXPECT_TRUE(map_.contains(3));
    EXPECT_FALSE(map_.contains(4));
}

TEST_F(StaticMapTest, Find)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    int value;
    EXPECT_TRUE(map_.find(1, value));
    EXPECT_EQ(value, 100);

    EXPECT_TRUE(map_.find(2, value));
    EXPECT_EQ(value, 200);

    EXPECT_TRUE(map_.find(3, value));
    EXPECT_EQ(value, 300);

    EXPECT_FALSE(map_.find(4, value));
}

TEST_F(StaticMapTest, Clear)
{
    map_.insert(1, 100);
    map_.insert(2, 200);
    map_.insert(3, 300);

    map_.clear();

    EXPECT_FALSE(map_.contains(1));
    EXPECT_FALSE(map_.contains(2));
    EXPECT_FALSE(map_.contains(3));

    EXPECT_THROW(map_.at(1), std::out_of_range);
    EXPECT_THROW(map_.at(2), std::out_of_range);
    EXPECT_THROW(map_.at(3), std::out_of_range);
}

TEST_F(StaticMapTest, RandomInserts)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 999);

    std::unordered_map<int, int> reference_map;

    for (int i = 0; i < 10000; ++i)
    {
        int key = dis(gen);
        int value = dis(gen);
        map_.insert(key, value);
        reference_map[key] = value; // In case of duplicates, store the last inserted value
    }

    for (const auto& [key, value] : reference_map)
    {
        EXPECT_TRUE(map_.contains(key));
        EXPECT_EQ(map_.at(key), value);
    }
}
