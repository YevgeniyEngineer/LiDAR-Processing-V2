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

#ifndef LIDAR_PROCESSING_LIB__STACK_HPP
#define LIDAR_PROCESSING_LIB__STACK_HPP

// STL
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
template <typename T>
class Stack final
{
  public:
    using value_type = T;

    Stack() = default;
    explicit Stack(std::size_t initial_capacity);

    Stack(const Stack&) = delete;
    Stack& operator=(const Stack&) = delete;
    Stack(Stack&&) = delete;
    Stack& operator=(Stack&&) = delete;

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
    T& top();
    const T& top() const;

  private:
    std::vector<T> buffer_;
};

template <typename T>
Stack<T>::Stack(std::size_t initial_capacity)
{
    buffer_.reserve(initial_capacity);
}

template <typename T>
inline void Stack<T>::reserve(std::size_t new_capacity)
{
    buffer_.reserve(new_capacity);
}

template <typename T>
inline bool Stack<T>::empty() const noexcept
{
    return buffer_.empty();
}

template <typename T>
inline std::size_t Stack<T>::size() const noexcept
{
    return buffer_.size();
}

template <typename T>
inline std::size_t Stack<T>::capacity() const noexcept
{
    return buffer_.capacity();
}

template <typename T>
inline void Stack<T>::push(const T& value)
{
    buffer_.push_back(value);
}

template <typename T>
inline void Stack<T>::push(T&& value)
{
    buffer_.push_back(std::move(value));
}

template <typename T>
template <typename... Args>
inline void Stack<T>::emplace(Args&&... args)
{
    buffer_.emplace_back(std::forward<Args>(args)...);
}

template <typename T>
inline void Stack<T>::pop()
{
    if (!empty())
    {
        throw std::out_of_range("Stack::pop(): stack is empty");
    }
    buffer_.pop_back();
}

template <typename T>
inline bool Stack<T>::try_pop() noexcept
{
    if (empty())
    {
        return false;
    }
    buffer_.pop_back();
    return true;
}

template <typename T>
inline T& Stack<T>::top()
{
    if (empty())
    {
        throw std::out_of_range("Stack::top(): stack is empty");
    }
    return buffer_.back();
}

template <typename T>
inline const T& Stack<T>::top() const
{
    if (empty())
    {
        throw std::out_of_range("Stack::top(): stack is empty");
    }
    return buffer_.back();
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STACK_HPP
