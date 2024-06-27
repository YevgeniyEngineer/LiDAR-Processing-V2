#include <lidar_processing_lib/circular_deque.hpp>

#include <gtest/gtest.h>

using namespace lidar_processing_lib;

TEST(CircularDequeTest, BasicOperations)
{
    CircularDeque<int, 5> deque;

    EXPECT_TRUE(deque.empty());
    EXPECT_FALSE(deque.full());

    // Insert elements at rear
    deque.push_back(1);
    deque.push_back(2);
    deque.push_back(3);
    deque.push_back(4);
    deque.push_back(5);

    EXPECT_FALSE(deque.empty());
    EXPECT_TRUE(deque.full());

    // Check front and rear elements
    EXPECT_EQ(deque.front(), 1);
    EXPECT_EQ(deque.back(), 5);

    // Attempt to insert one more element should raise an overflow error
    EXPECT_THROW(deque.push_back(6), std::overflow_error);

    // Delete a few elements from front and rear
    deque.pop_front();
    deque.pop_back();

    // Check front and rear elements again
    EXPECT_EQ(deque.front(), 2);
    EXPECT_EQ(deque.back(), 4);

    // Insert elements at front
    deque.push_front(0);
    deque.push_front(-1);

    // Check front and rear elements again
    EXPECT_EQ(deque.front(), -1);
    EXPECT_EQ(deque.back(), 4);
}

TEST(CircularDequeTest, MoveOperations)
{
    CircularDeque<int, 5> deque_1;
    deque_1.push_back(1);
    deque_1.push_back(2);
    deque_1.push_back(3);

    // Move constructor
    CircularDeque<int, 5> deque_2(std::move(deque_1));
    EXPECT_TRUE(deque_1.empty());
    EXPECT_FALSE(deque_2.empty());

    // Check elements in moved deque
    EXPECT_EQ(deque_2.front(), 1);
    EXPECT_EQ(deque_2.back(), 3);

    // Move assignment
    CircularDeque<int, 5> deque_3;
    deque_3 = std::move(deque_2);
    EXPECT_TRUE(deque_2.empty());
    EXPECT_FALSE(deque_3.empty());

    // Check elements in moved deque
    EXPECT_EQ(deque_3.front(), 1);
    EXPECT_EQ(deque_3.back(), 3);
}

TEST(CircularDequeTest, OverflowUnderflow)
{
    CircularDeque<int, 3> deque;

    // Underflow test
    EXPECT_THROW(deque.pop_front(), std::underflow_error);
    EXPECT_THROW(deque.pop_back(), std::underflow_error);
    EXPECT_THROW(deque.front(), std::underflow_error);
    EXPECT_THROW(deque.back(), std::underflow_error);

    // Overflow test
    deque.push_back(1);
    deque.push_back(2);
    deque.push_back(3);
    EXPECT_THROW(deque.push_back(4), std::overflow_error);
    EXPECT_THROW(deque.push_front(0), std::overflow_error);

    // Valid deletions
    deque.pop_front();
    deque.pop_back();
    EXPECT_NO_THROW(deque.front());
    EXPECT_NO_THROW(deque.back());
}
