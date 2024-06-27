/*
 * Copyright (c) 2024 Yevgeniy Simonov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef LIDAR_PROCESSING_LIB__CIRCULAR_DEQUE_HPP
#define LIDAR_PROCESSING_LIB__CIRCULAR_DEQUE_HPP

// STL
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace lidar_processing_lib
{
template <typename T, std::int32_t MaxSize>
class CircularDeque final
{
  public:
    static constexpr auto MAX_SIZE = MaxSize;

    CircularDeque();
    ~CircularDeque() noexcept;

    CircularDeque(const CircularDeque& other) = delete;
    CircularDeque& operator=(const CircularDeque& other) = delete;

    CircularDeque(CircularDeque&& other) noexcept;
    CircularDeque& operator=(CircularDeque&& other) noexcept;

    void push_front(const T& key);
    void push_back(const T& key);
    void pop_front();
    void pop_back();
    std::size_t size() const noexcept;
    bool full() const noexcept;
    bool empty() const noexcept;
    const T& front() const;
    const T& back() const;

  private:
    T* data_{nullptr};
    std::int32_t front_{-1};
    std::int32_t rear_{0};
    std::int32_t size_{0};
};

template <typename T, std::int32_t MaxSize>
CircularDeque<T, MaxSize>::CircularDeque()
{
    data_ = new T[MAX_SIZE];
}

template <typename T, std::int32_t MaxSize>
CircularDeque<T, MaxSize>::~CircularDeque() noexcept
{
    delete[] data_;
}

template <typename T, std::int32_t MaxSize>
CircularDeque<T, MaxSize>::CircularDeque(CircularDeque&& other) noexcept
    : data_(other.data_), front_(other.front_), rear_(other.rear_), size_(other.size_)
{
    other.data_ = nullptr;
    other.front_ = -1;
    other.rear_ = -1;
    other.size_ = 0;
}

template <typename T, std::int32_t MaxSize>
CircularDeque<T, MaxSize>& CircularDeque<T, MaxSize>::operator=(CircularDeque&& other) noexcept
{
    if (this != &other)
    {
        delete[] data_;
        data_ = other.data_;
        front_ = other.front_;
        rear_ = other.rear_;
        size_ = other.size_;

        other.data_ = nullptr;
        other.front_ = -1;
        other.rear_ = -1;
        other.size_ = 0;
    }
    return *this;
}

template <typename T, std::int32_t MaxSize>
inline std::size_t CircularDeque<T, MaxSize>::size() const noexcept
{
    return size_;
}

template <typename T, std::int32_t MaxSize>
inline bool CircularDeque<T, MaxSize>::full() const noexcept
{
    return (size_ == MAX_SIZE);
}

template <typename T, std::int32_t MaxSize>
inline bool CircularDeque<T, MaxSize>::empty() const noexcept
{
    return (size_ == 0);
}

template <typename T, std::int32_t MaxSize>
inline void CircularDeque<T, MaxSize>::push_front(const T& key)
{
    if (full())
    {
        throw std::overflow_error("Overflow: Deque is full");
    }

    if (empty())
    {
        // Queue is initially empty
        front_ = 0;
        rear_ = 0;
    }
    else if (front_ == 0)
    {
        // Front is at first position of queue
        front_ = MAX_SIZE - 1;
    }
    else
    {
        // Decrement front end by 1
        --front_;
    }

    data_[front_] = key;
    ++size_;
}

template <typename T, std::int32_t MaxSize>
inline void CircularDeque<T, MaxSize>::push_back(const T& key)
{
    if (full())
    {
        throw std::overflow_error("Overflow: Deque is full");
    }

    if (empty())
    {
        // Queue is initially empty
        front_ = 0;
        rear_ = 0;
    }
    else if (rear_ == MAX_SIZE - 1)
    {
        // Rear is at last position of queue
        rear_ = 0;
    }
    else
    {
        // Increment rear end by 1
        ++rear_;
    }

    data_[rear_] = key;
    size_++;
}

template <typename T, std::int32_t MaxSize>
inline void CircularDeque<T, MaxSize>::pop_front()
{
    if (empty())
    {
        throw std::underflow_error("Underflow: Deque is empty");
    }

    if (front_ == rear_)
    {
        // Deque has only one element
        front_ = -1;
        rear_ = -1;
    }
    else if (front_ == MAX_SIZE - 1)
    {
        // Back to initial position
        front_ = 0;
    }
    else
    {
        // Increment front by 1 to remove current front value
        ++front_;
    }

    --size_;
}

template <typename T, std::int32_t MaxSize>
inline void CircularDeque<T, MaxSize>::pop_back()
{
    if (empty())
    {
        throw std::underflow_error("Underflow: Deque is empty");
    }

    if (front_ == rear_)
    {
        // Deque has only one element
        front_ = -1;
        rear_ = -1;
    }
    else if (rear_ == 0)
    {
        rear_ = MAX_SIZE - 1;
    }
    else
    {
        --rear_;
    }

    --size_;
}

template <typename T, std::int32_t MaxSize>
inline const T& CircularDeque<T, MaxSize>::front() const
{
    if (empty())
    {
        throw std::underflow_error("Underflow: Deque is empty");
    }
    return data_[front_];
}

template <typename T, std::int32_t MaxSize>
inline const T& CircularDeque<T, MaxSize>::back() const
{
    if (empty())
    {
        throw std::underflow_error("Underflow: Deque is empty");
    }
    return data_[rear_];
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__CIRCULAR_DEQUE_HPP
