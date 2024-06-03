#ifndef CONTAINERS__CIRCULAR_QUEUE_HPP
#define CONTAINERS__CIRCULAR_QUEUE_HPP

#include <cstdint>
#include <iostream>
#include <memory>
#include <type_traits>

namespace containers
{
template<typename T, std::size_t Capacity>
class CircularQueue
{
  public:
    static constexpr std::size_t CAPACITY = Capacity;

    CircularQueue()
        : data_ {std::make_unique<T[]>(CAPACITY)}
    {
    }

    // For non-primite types: pass by const reference
    template<typename U = T>
    inline typename std::enable_if_t<!std::is_fundamental_v<U>, bool> push(const T& value) noexcept
    {
        return push_impl(value);
    }

    // For non-primite types: pass by r value reference
    template<typename U = T>
    inline typename std::enable_if_t<!std::is_fundamental_v<U>, bool> push(T&& value) noexcept
    {
        return push_impl(std::move(value));
    }

    // For primite types: pass by value
    template<typename U = T>
    inline typename std::enable_if_t<std::is_fundamental_v<U>, bool> push(T value) noexcept
    {
        if (size_ == CAPACITY)
        {
            return false;
        }
        data_[rear_] = value;
        rear_ = (rear_ + 1 == CAPACITY) ? 0 : rear_ + 1;
        ++size_;
        return true;
    }

    bool pop() noexcept
    {
        if (size_ == 0)
        {
            return false;
        }
        front_ = (front_ + 1 == CAPACITY) ? 0 : front_ + 1;
        --size_;
        return true;
    }

    T& front() noexcept
    {
        return data_[front_];
    }

    const T& front() const noexcept
    {
        return data_[front_];
    }

    bool empty() const noexcept
    {
        return size_ == 0;
    }

    std::size_t size() const noexcept
    {
        return size_;
    }

    std::size_t capacity() const noexcept
    {
        return CAPACITY;
    }

  private:
    std::size_t front_ {0};
    std::size_t rear_ {0};
    std::size_t size_ {0};
    std::unique_ptr<T[]> data_;

    template<typename U>
    inline bool push_impl(U&& value) noexcept
    {
        if (size_ == CAPACITY)
        {
            return false;
        }
        data_[rear_] = std::forward<U>(value);
        rear_ = (rear_ + 1 == CAPACITY) ? 0 : rear_ + 1;
        ++size_;
        return true;
    }
};
} // namespace containers

#endif // CONTAINERS__CIRCULAR_QUEUE_HPP
