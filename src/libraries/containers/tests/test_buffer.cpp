#include <containers/buffer.hpp>
#include <gtest/gtest.h>
#include <memory>

using namespace containers;

class BufferTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        buffer_ = std::make_unique<Buffer<int, 4>>();
    }

    void TearDown() override
    {
        buffer_.reset();
    }

    std::unique_ptr<Buffer<int, 4>> buffer_;
};

// Test get_loan_scoped function
TEST_F(BufferTest, GetLoanScoped)
{
    // Get a loaned item within a scoped block
    {
        auto loan = buffer_->get_loan_scoped_unique();
        *loan = 42;
        EXPECT_EQ(*loan, 42);
    }

    // After the scoped block, the loan should be automatically returned
    EXPECT_FALSE(buffer_->is_in_use(0));
}

// Test get_loan function
TEST_F(BufferTest, GetLoan)
{
    // Get a loaned item
    auto loan = buffer_->get_loan();
    *loan = 42;
    EXPECT_EQ(*loan, 42);

    // After using the loan, return it manually
    buffer_->return_loan(loan);
    EXPECT_FALSE(buffer_->is_in_use(0));
}

// Test multiple loans
TEST_F(BufferTest, MultipleLoans)
{
    // Get multiple loans
    auto loan1 = buffer_->get_loan_scoped_unique();
    auto loan2 = buffer_->get_loan_scoped_unique();
    auto loan3 = buffer_->get_loan_scoped_unique();

    *loan1 = 10;
    *loan2 = 20;
    *loan3 = 30;

    // Check loan values
    EXPECT_EQ(*loan1, 10);
    EXPECT_EQ(*loan2, 20);
    EXPECT_EQ(*loan3, 30);

    // Check in-use status
    EXPECT_TRUE(buffer_->is_in_use(0));
    EXPECT_TRUE(buffer_->is_in_use(1));
    EXPECT_TRUE(buffer_->is_in_use(2));

    // Loans should be automatically returned when out of scope
}

// Test returning loan manually
TEST_F(BufferTest, ReturnLoanManually)
{
    auto loan = buffer_->get_loan();
    *loan = 42;
    EXPECT_EQ(*loan, 42);

    // Manually return the loan
    buffer_->return_loan(loan);
    EXPECT_FALSE(buffer_->is_in_use(0));
}

// Test throwing error when getting loan from a full buffer
TEST_F(BufferTest, ThrowWhenBufferFull)
{
    // Fill the buffer
    auto loan1 = buffer_->get_loan_scoped_unique();
    auto loan2 = buffer_->get_loan_scoped_unique();
    auto loan3 = buffer_->get_loan_scoped_unique();
    auto loan4 = buffer_->get_loan_scoped_unique();

    // Attempt to get another loan (should throw)
    EXPECT_THROW(buffer_->get_loan_scoped_unique(), std::runtime_error);
}

// Test case to verify the behavior of get_loan_scoped_shared
TEST_F(BufferTest, GetLoanScopedShared)
{
    // Call get_loan_scoped_shared and capture the returned shared_ptr
    auto shared_loan = buffer_->get_loan_scoped_shared();

    // Verify that the shared_loan is not nullptr
    EXPECT_NE(shared_loan, nullptr);

    // Verify that the shared_loan points to a valid loaned element
    EXPECT_TRUE(buffer_->is_in_use(0));
}

// Test case to verify the scoped behavior of get_loan_scoped_shared
TEST_F(BufferTest, GetLoanScopedShared_ScopedBehavior)
{
    // Call get_loan_scoped_shared multiple times and capture the returned shared_ptrs
    auto shared_loan_1 = buffer_->get_loan_scoped_shared();
    auto shared_loan_2 = buffer_->get_loan_scoped_shared();

    // Verify that the shared_loan_1 and shared_loan_2 are not nullptr
    EXPECT_NE(shared_loan_1, nullptr);
    EXPECT_NE(shared_loan_2, nullptr);

    // Verify that the shared_loan_1 and shared_loan_2 point to different loaned elements
    EXPECT_NE(shared_loan_1.get(), shared_loan_2.get());

    // Create a nested scope
    std::shared_ptr<int> shared_load_3_shared;
    {
        // Capture the returned shared_ptr from get_loan_scoped_shared in the nested scope
        auto shared_loan_3 = buffer_->get_loan_scoped_shared();

        // Verify that the shared_loan_3 is not nullptr
        EXPECT_NE(shared_loan_3, nullptr);

        // Verify that the shared_loan_3 points to a different loaned element
        EXPECT_NE(shared_loan_3.get(), shared_loan_1.get());
        EXPECT_NE(shared_loan_3.get(), shared_loan_2.get());

        // Copy reference count to prevent from expiry
        shared_load_3_shared = shared_loan_3;
    }

    // Check that shared_load_3 is still in use
    EXPECT_TRUE(buffer_->is_in_use(shared_load_3_shared.get()));

    // After exiting the nested scope, verify that the previous loaned elements are still in use
    EXPECT_TRUE(buffer_->is_in_use(0));
}
