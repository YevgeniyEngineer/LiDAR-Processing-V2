#ifndef CONTAINERS_BUFFER_HPP
#define CONTAINERS_BUFFER_HPP

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <type_traits>

namespace containers
{
/*
Initialization: The loans_ and buffer_ arrays are initialized
without specific values but assume T has a default constructor.

Get a Loan: This method locates the first available item that is not in use,
swaps it from the buffer_ to the loans_ array to mark it as actively used,
and returns this item.

Return a Loan: This method finds the item in the loans_ array,
marks it as not in use, and swaps it back into the buffer_ array.
*/
template<typename T, std::size_t N>
class Buffer final
{
    static_assert(std::is_swappable_v<T>, "T should be swappable");
    static_assert(N > 0, "Buffer size must be greater than zero");
    static_assert(N % 2 == 0, "Buffer size should be divisible by 2");

  public:
    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

    Buffer()
    {
        in_use_.fill(false);
    }

    struct TReset
    {
        TReset(Buffer<T, N>& buffer, T* ptr)
            : buffer_(buffer),
              ptr_(ptr)
        {
        }

        void operator()(T* ptr)
        {
            if (ptr)
            {
                buffer_.return_loan(ptr);
            }
        }

      private:
        Buffer<T, N>& buffer_;
        T* ptr_;
    };

    std::unique_ptr<T, TReset> get_loan_scoped_unique()
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < N; ++i)
        {
            if (!in_use_[i])
            {
                in_use_[i] = true;
                std::swap(loans_[i], buffer_[i]); // Swap into active use
                return std::unique_ptr<T, TReset>(&loans_[i], TReset(*this, &loans_[i]));
            }
        }
        throw std::runtime_error("No buffer elements available");
    }

    std::shared_ptr<T> get_loan_scoped_shared()
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < N; ++i)
        {
            if (!in_use_[i])
            {
                in_use_[i] = true;
                std::swap(loans_[i], buffer_[i]); // Swap into active use
                return std::shared_ptr<T>(&loans_[i], [this](T* ptr) {
                    if (ptr)
                    {
                        return_loan(ptr);
                    }
                });
            }
        }
        throw std::runtime_error("No buffer elements available");
    }

    T* get_loan()
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < N; ++i)
        {
            if (!in_use_[i])
            {
                in_use_[i] = true;
                std::swap(loans_[i], buffer_[i]); // Swap into active use
                return &loans_[i];
            }
        }
        throw std::runtime_error("No buffer elements available");
    }

    void return_loan(T* item)
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < N; ++i)
        {
            if (item == &loans_[i])
            {
                in_use_[i] = false;
                std::swap(loans_[i], buffer_[i]); // Swap back to buffer
                return;
            }
        }
        throw std::runtime_error("Returned item does not belong to the buffer");
    }

    template<typename U = T>
    typename std::enable_if<std::is_integral<U>::value, bool>::type is_in_use(std::size_t index) const
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (index >= N)
        {
            throw std::out_of_range("Index out of range");
        }
        return in_use_[index];
    }

    template<typename U = T>
    std::enable_if_t<std::is_pointer_v<U>, bool> is_in_use(U ptr) const
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t index = 0; index < loans_.size(); ++index)
        {
            auto& loan = loans_[index];
            if (&loan == ptr)
            {
                return in_use_[index];
            }
        }
        throw std::out_of_range("Invalid pointer provided");
    }

  private:
    mutable std::mutex mutex_;
    std::array<T, N> loans_;
    std::array<T, N> buffer_;
    std::array<bool, N> in_use_;
};
} // namespace containers
#endif // CONTAINERS_BUFFER_HPP
