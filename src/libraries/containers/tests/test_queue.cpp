#include <containers/queue.hpp>
#include <gtest/gtest.h>
#include <string>

using namespace containers;

class QueueIntTest : public ::testing::Test
{
  protected:
    Queue<int> queue_;
};

class QueueStringTest : public ::testing::Test
{
  protected:
    Queue<std::string> queue_;
};

TEST_F(QueueIntTest, BasicPushPop)
{
    queue_.push(1);
    EXPECT_FALSE(queue_.empty());
    EXPECT_EQ(queue_.front(), 1);
    queue_.pop();
    EXPECT_TRUE(queue_.empty());
}

TEST_F(QueueIntTest, Reserve)
{
    queue_.reserve(100);
    EXPECT_EQ(queue_.capacity(), 100);
}

TEST_F(QueueIntTest, ResizeCapacity)
{
    for (int i = 0; i < 10; ++i)
    {
        queue_.push(i);
    }
    EXPECT_EQ(queue_.capacity(), 16);
    EXPECT_EQ(queue_.size(), 10);
}

TEST_F(QueueIntTest, PopFromEmpty)
{
    EXPECT_THROW(queue_.pop(), std::out_of_range);
}

TEST_F(QueueIntTest, FrontOnEmpty)
{
    EXPECT_THROW(queue_.front(), std::out_of_range);
}

TEST_F(QueueIntTest, ClearQueue)
{
    queue_.push(5);
    while (!queue_.empty())
    {
        queue_.pop();
    }
    EXPECT_EQ(queue_.size(), 0);
    EXPECT_EQ(queue_.capacity(), 1);
}

TEST_F(QueueStringTest, BasicPushPop)
{
    queue_.push("hello");
    EXPECT_FALSE(queue_.empty());
    EXPECT_EQ(queue_.front(), "hello");
    queue_.pop();
    EXPECT_TRUE(queue_.empty());
}

TEST_F(QueueStringTest, Reserve)
{
    queue_.reserve(100);
    EXPECT_EQ(queue_.capacity(), 100);
}

TEST_F(QueueStringTest, ResizeCapacity)
{
    for (std::size_t i = 0; i < 10; ++i)
    {
        queue_.push(std::to_string(i));
    }
    EXPECT_EQ(queue_.capacity(), 16);
    EXPECT_EQ(queue_.size(), 10);
}

TEST_F(QueueStringTest, ClearQueue)
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
