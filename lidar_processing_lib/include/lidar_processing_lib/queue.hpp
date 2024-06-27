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

#ifndef LIDAR_PROCESSING_LIB__QUEUE_HPP
#define LIDAR_PROCESSING_LIB__QUEUE_HPP

// STL
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

namespace lidar_processing_lib
{
template <typename T>
class Queue final
{
  public:
    using value_type = T;

    Queue();
    explicit Queue(std::size_t initial_capacity);
    ~Queue() noexcept;

    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;
    Queue(Queue&&) = delete;
    Queue& operator=(Queue&&) = delete;

    void reserve(std::size_t new_capacity);
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    std::size_t capacity() const noexcept;
    void push(const T& value);
    void push(T&& value);
    template <typename... Args>
    void emplace(Args&&... args);
    void pop();
    bool try_pop() noexcept;
    T& front();
    const T& front() const;

  private:
    std::unique_ptr<T[]> buffer_;
    std::size_t head_; // Index of the front element
    std::size_t tail_; // Index of the next available slot
    std::size_t size_;
    std::size_t capacity_;

    void resize(std::size_t new_capacity);
    void clear() noexcept;
    static inline std::size_t incrementWraparound(std::size_t index, std::size_t capacity) noexcept;
};

template <typename T>
inline std::size_t Queue<T>::incrementWraparound(std::size_t index, std::size_t capacity) noexcept
{
    return ((index + 1) < capacity) ? (index + 1) : 0;
}

template <typename T>
Queue<T>::Queue() : Queue(1)
{
}

template <typename T>
Queue<T>::Queue(std::size_t initial_capacity)
    : buffer_(std::make_unique<T[]>(initial_capacity)), head_(0), tail_(0), size_(0),
      capacity_(initial_capacity)
{
}

template <typename T>
Queue<T>::~Queue() noexcept
{
    clear();
}

template <typename T>
void Queue<T>::reserve(std::size_t new_capacity)
{
    resize(new_capacity);
}

template <typename T>
bool Queue<T>::empty() const noexcept
{
    return size_ == 0;
}

template <typename T>
std::size_t Queue<T>::size() const noexcept
{
    return size_;
}

template <typename T>
std::size_t Queue<T>::capacity() const noexcept
{
    return capacity_;
}

template <typename T>
void Queue<T>::push(const T& value)
{
    emplace(value);
}

template <typename T>
void Queue<T>::push(T&& value)
{
    emplace(std::move(value));
}

template <typename T>
template <typename... Args>
void Queue<T>::emplace(Args&&... args)
{
    if (size_ >= capacity_)
    {
        resize(capacity_ * 2); // Double the capacity
    }

    ::new (&buffer_[tail_]) T(std::forward<Args>(args)...);
    tail_ = incrementWraparound(tail_, capacity_);
    ++size_;
}

template <typename T>
void Queue<T>::pop()
{
    if (empty())
    {
        throw std::out_of_range("Queue::pop(): queue is empty");
    }

    buffer_[head_].~T(); // Call destructor for the element

    head_ = incrementWraparound(head_, capacity_);
    --size_;
}

template <typename T>
bool Queue<T>::try_pop() noexcept
{
    if (empty())
    {
        return false;
    }

    buffer_[head_].~T(); // Call destructor for the element

    head_ = incrementWraparound(head_, capacity_);
    --size_;

    return true;
}

template <typename T>
T& Queue<T>::front()
{
    if (empty())
    {
        throw std::out_of_range("Queue::front(): queue is empty");
    }

    return buffer_[head_];
}

template <typename T>
const T& Queue<T>::front() const
{
    if (empty())
    {
        throw std::out_of_range("Queue::front(): queue is empty");
    }

    return buffer_[head_];
}

template <typename T>
void Queue<T>::resize(std::size_t new_capacity)
{
    if (new_capacity <= capacity_)
    {
        return; // Only resize if the new capacity is larger
    }

    // Create a new vector to hold the elements with aligned storage
    auto new_buffer = std::make_unique<T[]>(new_capacity);

    // Copy construct the existing elements into the new buffer in the correct order
    std::size_t current = head_;
    for (std::size_t i = 0; i < size_; ++i)
    {
        ::new (&new_buffer[i]) T(std::move(buffer_[current]));

        buffer_[current].~T(); // Call destructor for the element

        current = incrementWraparound(current, capacity_);
    }

    // Replace the old buffer with the new one and reset head and tail
    buffer_ = std::move(new_buffer);
    head_ = 0;
    tail_ = size_; // New tail is at the index of the last element added
    capacity_ = new_capacity;
}

template <typename T>
void Queue<T>::clear() noexcept
{
    while (size() > 0)
    {
        static_cast<void>(try_pop());
    }
}
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__QUEUE_HPP
