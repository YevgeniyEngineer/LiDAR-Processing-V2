#include <containers/circular_queue.hpp>
#include <gtest/gtest.h>
#include <string>

using namespace containers;

class CircularQueueIntTest : public ::testing::Test
{
  protected:
    CircularQueue<int> queue_;
};

class CircularQueueStringTest : public ::testing::Test
{
  protected:
    CircularQueue<std::string> queue_;
};

TEST_F(CircularQueueIntTest, BasicPushPop)
{
    queue_.push(1);
    EXPECT_FALSE(queue_.empty());
    EXPECT_EQ(queue_.front(), 1);
    queue_.pop();
    EXPECT_TRUE(queue_.empty());
}

TEST_F(CircularQueueIntTest, Reserve)
{
    queue_.reserve(100);
    EXPECT_EQ(queue_.capacity(), 100);
}

TEST_F(CircularQueueIntTest, ResizeCapacity)
{
    for (int i = 0; i < 10; ++i)
    {
        queue_.push(i);
    }
    EXPECT_EQ(queue_.capacity(), 16);
    EXPECT_EQ(queue_.size(), 10);
}

TEST_F(CircularQueueIntTest, PopFromEmpty)
{
    EXPECT_THROW(queue_.pop(), std::out_of_range);
}

TEST_F(CircularQueueIntTest, FrontOnEmpty)
{
    EXPECT_THROW(queue_.front(), std::out_of_range);
}

TEST_F(CircularQueueIntTest, ClearQueue)
{
    queue_.push(5);
    while (!queue_.empty())
    {
        queue_.pop();
    }
    EXPECT_EQ(queue_.size(), 0);
    EXPECT_EQ(queue_.capacity(), 1);
}

TEST_F(CircularQueueStringTest, BasicPushPop)
{
    queue_.push("hello");
    EXPECT_FALSE(queue_.empty());
    EXPECT_EQ(queue_.front(), "hello");
    queue_.pop();
    EXPECT_TRUE(queue_.empty());
}

TEST_F(CircularQueueStringTest, Reserve)
{
    queue_.reserve(100);
    EXPECT_EQ(queue_.capacity(), 100);
}

TEST_F(CircularQueueStringTest, ResizeCapacity)
{
    for (std::size_t i = 0; i < 10; ++i)
    {
        queue_.push(std::to_string(i));
    }
    EXPECT_EQ(queue_.capacity(), 16);
    EXPECT_EQ(queue_.size(), 10);
}

TEST_F(CircularQueueStringTest, ClearQueue)
{
    queue_.push(std::to_string(5));
    while (!queue_.empty())
    {
        queue_.pop();
    }
    EXPECT_EQ(queue_.size(), 0);
    EXPECT_EQ(queue_.capacity(), 1);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
