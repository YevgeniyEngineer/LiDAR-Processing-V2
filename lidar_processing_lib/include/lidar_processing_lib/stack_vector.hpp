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

#ifndef LIDAR_PROCESSING_LIB__STACK_VECTOR_HPP
#define LIDAR_PROCESSING_LIB__STACK_VECTOR_HPP

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace lidar_processing_lib
{
template <typename T, std::size_t MaxSize>
class StackVector final
{
  public:
    static constexpr auto MAX_SIZE = MaxSize;

    StackVector() = default;
    ~StackVector() noexcept;

    StackVector(const StackVector&) = delete;
    StackVector& operator=(const StackVector&) = delete;
    StackVector(StackVector&& other) noexcept;
    StackVector& operator=(StackVector&& other) noexcept;

    template <typename... Args>
    void emplace_back(Args&&... args);
    void push_back(const T& value);
    void push_back(T&& value);
    void pop_back() noexcept;
    T& operator[](std::size_t index);
    const T& operator[](std::size_t index) const;
    std::size_t size() const noexcept;
    void clear() noexcept;
    T* data() noexcept;
    const T* data() const noexcept;
    bool operator==(std::initializer_list<T> ilist) const noexcept;

  private:
    alignas(alignof(T)) std::byte buffer_[sizeof(T) * MAX_SIZE];
    std::size_t size_{0};
};

template <typename T, std::size_t MaxSize>
inline StackVector<T, MaxSize>::~StackVector() noexcept
{
    clear();
}

template <typename T, std::size_t MaxSize>
inline StackVector<T, MaxSize>::StackVector(StackVector&& other) noexcept : size_{other.size_}
{
    for (std::size_t i = 0; i < size_; ++i)
    {
        ::new (&data()[i]) T{std::move(reinterpret_cast<T&>(other.data()[i]))};
    }
    other.clear();
}

template <typename T, std::size_t MaxSize>
inline StackVector<T, MaxSize>& StackVector<T, MaxSize>::operator=(StackVector&& other) noexcept
{
    if (this != &other)
    {
        clear();
        size_ = other.size_;
        for (std::size_t i = 0; i < size_; ++i)
        {
            ::new (&data()[i]) T{std::move(reinterpret_cast<T&>(other.data()[i]))};
        }
        other.clear();
    }
    return *this;
}

template <typename T, std::size_t MaxSize>
template <typename... Args>
inline void StackVector<T, MaxSize>::emplace_back(Args&&... args)
{
    if (size_ >= MAX_SIZE)
    {
        throw std::out_of_range{"StackVector overflow"};
    }
    ::new (&data()[size_]) T{std::forward<Args>(args)...};
    ++size_;
}

template <typename T, std::size_t MaxSize>
inline void StackVector<T, MaxSize>::push_back(const T& value)
{
    if (size_ >= MAX_SIZE)
    {
        throw std::out_of_range{"StackVector overflow"};
    }
    ::new (&data()[size_]) T{value};
    ++size_;
}

template <typename T, std::size_t MaxSize>
inline void StackVector<T, MaxSize>::push_back(T&& value)
{
    if (size_ >= MAX_SIZE)
    {
        throw std::out_of_range{"StackVector overflow"};
    }
    ::new (&data()[size_]) T{std::move(value)};
    ++size_;
}

template <typename T, std::size_t MaxSize>
inline void StackVector<T, MaxSize>::pop_back() noexcept
{
    if (size_ > 0)
    {
        data()[--size_].~T();
    }
}

template <typename T, std::size_t MaxSize>
inline T& StackVector<T, MaxSize>::operator[](std::size_t index)
{
    if (index >= size_)
    {
        throw std::out_of_range{"Index out of range"};
    }
    return reinterpret_cast<T&>(data()[index]);
}

template <typename T, std::size_t MaxSize>
inline const T& StackVector<T, MaxSize>::operator[](std::size_t index) const
{
    if (index >= size_)
    {
        throw std::out_of_range{"Index out of range"};
    }
    return reinterpret_cast<const T&>(data()[index]);
}

template <typename T, std::size_t MaxSize>
inline std::size_t StackVector<T, MaxSize>::size() const noexcept
{
    return size_;
}

template <typename T, std::size_t MaxSize>
inline void StackVector<T, MaxSize>::clear() noexcept
{
    for (std::size_t i = 0; i < size_; ++i)
    {
        data()[i].~T();
    }
    size_ = 0;
}

template <typename T, std::size_t MaxSize>
inline T* StackVector<T, MaxSize>::data() noexcept
{
    return reinterpret_cast<T*>(buffer_);
}

template <typename T, std::size_t MaxSize>
inline const T* StackVector<T, MaxSize>::data() const noexcept
{
    return reinterpret_cast<const T*>(buffer_);
}

template <typename T, std::size_t MaxSize>
inline bool StackVector<T, MaxSize>::operator==(std::initializer_list<T> ilist) const noexcept
{
    if (size_ != ilist.size())
    {
        return false;
    }
    std::size_t i = 0;
    for (const auto& elem : ilist)
    {
        if (data()[i++] != elem)
        {
            return false;
        }
    }
    return true;
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STACK_VECTOR_HPP
