#include "lidar_processing_lib/static_map.hpp"

#include <gtest/gtest.h>
#include <map>
#include <random>
#include <unordered_set>

using namespace lidar_processing_lib;

class StaticMapTest : public ::testing::Test
{
  protected:
    StaticMap<int> map_;

    void SetUp() override
    {
        map_.reserve(200'000);
        map_.numRange(1000);
        map_.numAzimuth(1000);
        map_.numElevation(100);
    }

    void TearDown() override
    {
        // Clear the map after each test
        map_.clear();
    }
};

TEST_F(StaticMapTest, SingleInsertRetrieve)
{
    map_.insert(500, 500, 50, 12345);
    ASSERT_EQ(map_.at(500, 500, 50), 12345);
}

TEST_F(StaticMapTest, LargeIndexInsertRetrieve)
{
    map_.insert(999, 999, 99, 54321);
    ASSERT_EQ(map_.at(999, 999, 99), 54321);
}

TEST_F(StaticMapTest, NonExistingIndex)
{
    EXPECT_THROW(map_.at(1, 1, 1), std::out_of_range);
}

TEST_F(StaticMapTest, RandomInserts)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 999);

    // Using maps to store the last values for each unique index
    std::map<std::tuple<int, int, int>, int> last_values;

    for (int i = 0; i < 10000; ++i)
    {
        int range = dis(gen);
        int azimuth = dis(gen);
        int elevation = dis(gen);
        int value = dis(gen);

        // Insert the value
        map_.insert(range, azimuth, elevation, value);

        // Record the expected last value for this combination
        last_values[{range, azimuth, elevation}] = value;
    }

    // Verify that the last inserted value is the one that is retrieved
    for (auto& [key, expected_value] : last_values)
    {
        int range, azimuth, elevation;
        std::tie(range, azimuth, elevation) = key;
        int retrieved_value = map_.at(range, azimuth, elevation);
        ASSERT_EQ(retrieved_value, expected_value) << "Mismatch at range " << range << ", azimuth "
                                                   << azimuth << ", elevation " << elevation;
    }
}
