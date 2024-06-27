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

#ifndef LIDAR_PROCESSING_LIB__PRIORITY_QUEUE_HPP
#define LIDAR_PROCESSING_LIB__PRIORITY_QUEUE_HPP

// STL
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <vector>

namespace lidar_processing_lib
{
template <typename T, typename Comparator = std::less<T>>
class PriorityQueue final
{
  public:
    using value_type = T;

    explicit PriorityQueue(Comparator comp = Comparator());

    void reserve(std::size_t new_capacity);
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    void push(const T& value);
    void push(T&& value);
    template <typename... Args>
    void emplace(Args&&... args);
    void pop();
    bool try_pop() noexcept;
    T& top();
    const T& top() const;

  private:
    std::vector<T> buffer_;
    Comparator comp_;
};

template <typename T, typename Comparator>
PriorityQueue<T, Comparator>::PriorityQueue(Comparator comp) : comp_(comp)
{
    std::make_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator>
inline void PriorityQueue<T, Comparator>::reserve(std::size_t new_capacity)
{
    buffer_.reserve(new_capacity);
}

template <typename T, typename Comparator>
inline bool PriorityQueue<T, Comparator>::empty() const noexcept
{
    return buffer_.empty();
}

template <typename T, typename Comparator>
inline std::size_t PriorityQueue<T, Comparator>::size() const noexcept
{
    return buffer_.size();
}

template <typename T, typename Comparator>
inline void PriorityQueue<T, Comparator>::push(const T& value)
{
    buffer_.push_back(value);
    std::push_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator>
inline void PriorityQueue<T, Comparator>::push(T&& value)
{
    buffer_.push_back(std::move(value));
    std::push_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator>
template <typename... Args>
inline void PriorityQueue<T, Comparator>::emplace(Args&&... args)
{
    buffer_.emplace_back(std::forward<Args>(args)...);
    std::push_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator>
inline void PriorityQueue<T, Comparator>::pop()
{
    if (empty())
    {
        throw std::out_of_range("PriorityQueue::pop(): queue is empty");
    }
    std::pop_heap(buffer_.begin(), buffer_.end(), comp_);
    buffer_.pop_back();
}

template <typename T, typename Comparator>
inline bool PriorityQueue<T, Comparator>::try_pop() noexcept
{
    if (empty())
    {
        return false;
    }
    std::pop_heap(buffer_.begin(), buffer_.end(), comp_);
    buffer_.pop_back();
}

template <typename T, typename Comparator>
inline T& PriorityQueue<T, Comparator>::top()
{
    if (empty())
    {
        throw std::out_of_range("PriorityQueue::top(): queue is empty");
    }
    return buffer_.front();
}

template <typename T, typename Comparator>
inline const T& PriorityQueue<T, Comparator>::top() const
{
    if (empty())
    {
        throw std::out_of_range("PriorityQueue::top(): queue is empty");
    }
    return buffer_.front();
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__PRIORITY_QUEUE_HPP
