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

#ifndef LIDAR_PROCESSING_LIB__VECTOR_HPP
#define LIDAR_PROCESSING_LIB__VECTOR_HPP

// STL
#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
template <typename T, typename Allocator = std::allocator<T>>
class Vector final
{
  public:
    using value_type = T;
    using allocator_type = Allocator;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = typename std::vector<T, Allocator>::pointer;
    using const_pointer = typename std::vector<T, Allocator>::const_pointer;
    using iterator = typename std::vector<T, Allocator>::iterator;
    using const_iterator = typename std::vector<T, Allocator>::const_iterator;
    using reverse_iterator = typename std::vector<T, Allocator>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<T, Allocator>::const_reverse_iterator;

    // Member functions
    Vector() noexcept(noexcept(Allocator()));
    ~Vector() noexcept = default;
    Vector(const Vector& other);
    Vector(Vector&& other) noexcept;
    Vector& operator=(const Vector& other) &;
    Vector& operator=(Vector&& other) & noexcept(
        std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
        std::allocator_traits<Allocator>::is_always_equal::value);
    explicit Vector(const Allocator& alloc) noexcept;
    Vector(const std::vector<T, Allocator>& vec);
    Vector(std::vector<T, Allocator>&& vec) noexcept(
        std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
        std::allocator_traits<Allocator>::is_always_equal::value);
    Vector(std::initializer_list<T> ilist);
    template <class InputIt>
    Vector(InputIt first, InputIt last, const Allocator& alloc = Allocator());
    Vector(std::size_t count, const T& value, const Allocator& alloc = Allocator());
    explicit Vector(std::size_t count, const Allocator& alloc = Allocator());
    Vector& operator=(const std::vector<T, Allocator>& vec) &;
    Vector& operator=(std::vector<T, Allocator>&& vec) & noexcept(
        std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
        std::allocator_traits<Allocator>::is_always_equal::value);
    Vector& operator=(std::initializer_list<T> ilist) &;
    void assign(std::size_t count, const T& value);
    template <typename InputIterator,
              typename std::enable_if<!std::is_integral<InputIterator>::value,
                                      InputIterator>::type* = nullptr>
    void assign(InputIterator first, InputIterator last);
    void assign(std::initializer_list<T> ilist);

    // Element access
    T& at(std::size_t index);
    const T& at(std::size_t index) const;
    T& operator[](std::size_t index) noexcept;
    const T& operator[](std::size_t index) const noexcept;
    T& front();
    const T& front() const;
    T& back();
    const T& back() const;
    T* data() noexcept;
    const T* data() const noexcept;

    // Iterators
    iterator begin();
    const_iterator begin() const;
    const_iterator cbegin() const noexcept;
    iterator end() noexcept;
    const_iterator end() const noexcept;
    const_iterator cend() const noexcept;
    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    const_reverse_iterator crbegin() const noexcept;
    reverse_iterator rend() noexcept;
    const_reverse_iterator rend() const noexcept;
    const_reverse_iterator crend() const noexcept;

    // Capacity
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    std::size_t max_size() const noexcept;
    void reserve(std::size_t new_capacity);
    std::size_t capacity() const noexcept;

    // Modifiers
    void clear() noexcept;
    iterator insert(const_iterator pos, const T& value);
    iterator insert(const_iterator pos, T&& value);
    iterator insert(const_iterator pos, size_type count, const T& value);
    template <typename InputIt,
              typename std::enable_if<!std::is_integral<InputIt>::value, InputIt>::type* = nullptr>
    iterator insert(const_iterator pos, InputIt first, InputIt last);
    iterator insert(const_iterator pos, std::initializer_list<T> ilist);
    template <typename... Args>
    iterator emplace(const_iterator pos, Args&&... args);
    iterator erase(const_iterator pos);
    iterator erase(const_iterator first, const_iterator last);
    void push_back(const T& value);
    void push_back(T&& value);
    template <typename... Args>
    void emplace_back(Args&&... args);
    void pop_back() noexcept;
    void resize(std::size_t new_size);
    void resize(std::size_t new_size, const T& value);
    void swap(Vector& other) noexcept(
        std::allocator_traits<Allocator>::propagate_on_container_swap::value ||
        std::allocator_traits<Allocator>::is_always_equal::value);

  private:
    std::vector<T, Allocator> buffer_;
    std::size_t size_;

    bool buffer_full() const noexcept;
    bool buffer_reached_capacity() const noexcept;
};

template <typename T, typename Allocator>
bool Vector<T, Allocator>::buffer_full() const noexcept
{
    return size_ == buffer_.size();
}

template <typename T, typename Allocator>
bool Vector<T, Allocator>::buffer_reached_capacity() const noexcept
{
    return size_ == buffer_.capacity();
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector() noexcept(noexcept(Allocator())) : buffer_(), size_(0)
{
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(const Vector& other) : buffer_(other.buffer_), size_(other.size_)
{
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(Vector&& other) noexcept
    : buffer_(std::move(other.buffer_)), size_(other.size_)
{
    other.size_ = 0;
}

template <typename T, typename Allocator>
Vector<T, Allocator>& Vector<T, Allocator>::operator=(const Vector& other) &
{
    if (this != &other)
    {
        buffer_.assign(other.buffer_.cbegin(), other.buffer_.cend());
        size_ = other.size_;
    }

    return *this;
}

template <typename T, typename Allocator>
Vector<T, Allocator>& Vector<T, Allocator>::operator=(Vector&& other) & noexcept(
    std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
    std::allocator_traits<Allocator>::is_always_equal::value)
{
    if (this != &other)
    {
        buffer_ = std::move(other.buffer_);
        size_ = other.size_;
        other.size_ = 0;
    }

    return *this;
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(const Allocator& alloc) noexcept
    : buffer_(alloc), size_(buffer_.size())
{
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(const std::vector<T, Allocator>& vec)
    : buffer_(vec), size_(buffer_.size())
{
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(std::vector<T, Allocator>&& vec) noexcept(
    std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
    std::allocator_traits<Allocator>::is_always_equal::value)
    : buffer_(std::move(vec)), size_(buffer_.size())
{
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(std::initializer_list<T> ilist) : buffer_(ilist), size_(buffer_.size())
{
}

template <typename T, typename Allocator>
template <typename InputIt>
Vector<T, Allocator>::Vector(InputIt first, InputIt last, const Allocator& alloc)
    : buffer_(first, last, alloc), size_(buffer_.size())
{
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(std::size_t count, const T& value, const Allocator& alloc)
    : buffer_(count, value, alloc), size_(buffer_.size())
{
}

template <typename T, typename Allocator>
Vector<T, Allocator>::Vector(std::size_t count, const Allocator& alloc)
    : buffer_(count, alloc), size_(buffer_.size())
{
}

/// @note The capacity of the original copied from vector is not preserved
template <typename T, typename Allocator>
Vector<T, Allocator>& Vector<T, Allocator>::operator=(const std::vector<T, Allocator>& vec) &
{
    if (&buffer_ != &vec)
    {
        buffer_.assign(vec.cbegin(), vec.cend());
        size_ = vec.size();
    }

    return *this;
}

/// @note The capacity of the original moved from vector is not preserved
template <typename T, typename Allocator>
Vector<T, Allocator>& Vector<T, Allocator>::operator=(std::vector<T, Allocator>&& vec) & noexcept(
    std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
    std::allocator_traits<Allocator>::is_always_equal::value)
{
    if (&buffer_ != &vec)
    {
        buffer_ = std::move(vec);
        size_ = vec.size();
    }

    return *this;
}

template <typename T, typename Allocator>
Vector<T, Allocator>& Vector<T, Allocator>::operator=(std::initializer_list<T> ilist) &
{
    buffer_ = ilist;
    size_ = buffer_.size();

    return *this;
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::assign(std::size_t count, const T& value)
{
    buffer_.assign(count, value);
    size_ = buffer_.size();
}

template <typename T, typename Allocator>
template <typename InputIterator,
          typename std::enable_if<!std::is_integral<InputIterator>::value, InputIterator>::type*>
void Vector<T, Allocator>::assign(InputIterator first, InputIterator last)
{
    buffer_.assign(first, last);
    size_ = buffer_.size();
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::assign(std::initializer_list<T> ilist)
{
    buffer_.assign(ilist);
    size_ = buffer_.size();
}

template <typename T, typename Allocator>
T& Vector<T, Allocator>::at(std::size_t index)
{
    if (index >= size_)
    {
        throw std::out_of_range("Vector::at(): Index out of range");
    }

    return buffer_[index];
}

template <typename T, typename Allocator>
const T& Vector<T, Allocator>::at(std::size_t index) const
{
    if (index >= size_)
    {
        throw std::out_of_range("Vector::at(): Index out of range");
    }

    return buffer_[index];
}

template <typename T, typename Allocator>
T& Vector<T, Allocator>::operator[](std::size_t index) noexcept
{
    return buffer_[index];
}

template <typename T, typename Allocator>
const T& Vector<T, Allocator>::operator[](std::size_t index) const noexcept
{
    return buffer_[index];
}

template <typename T, typename Allocator>
T& Vector<T, Allocator>::front()
{
    if (empty())
    {
        throw std::out_of_range("Vector::front(): Vector is empty");
    }
    return buffer_[0U];
}

template <typename T, typename Allocator>
const T& Vector<T, Allocator>::front() const
{
    if (empty())
    {
        throw std::out_of_range("Vector::front(): Vector is empty");
    }
    return buffer_[0U];
}

template <typename T, typename Allocator>
T& Vector<T, Allocator>::back()
{
    if (empty())
    {
        throw std::out_of_range("Vector::back(): Vector is empty");
    }
    return buffer_[size_ - 1U];
}

template <typename T, typename Allocator>
const T& Vector<T, Allocator>::back() const
{
    if (empty())
    {
        throw std::out_of_range("Vector::back(): Vector is empty");
    }
    return buffer_[size_ - 1U];
}

template <typename T, typename Allocator>
T* Vector<T, Allocator>::data() noexcept
{
    return buffer_.data();
}

template <typename T, typename Allocator>
const T* Vector<T, Allocator>::data() const noexcept
{
    return buffer_.data();
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::begin()
{
    return buffer_.begin();
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_iterator Vector<T, Allocator>::begin() const
{
    return buffer_.begin();
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_iterator Vector<T, Allocator>::cbegin() const noexcept
{
    return buffer_.cbegin();
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::end() noexcept
{
    return buffer_.begin() + static_cast<std::ptrdiff_t>(size_);
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_iterator Vector<T, Allocator>::end() const noexcept
{
    return buffer_.begin() + static_cast<std::ptrdiff_t>(size_);
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_iterator Vector<T, Allocator>::cend() const noexcept
{
    return buffer_.cbegin() + static_cast<std::ptrdiff_t>(size_);
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::reverse_iterator Vector<T, Allocator>::rbegin()
{
    return reverse_iterator(end());
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::rbegin() const
{
    return const_reverse_iterator(cend());
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::crbegin() const noexcept
{
    return const_reverse_iterator(cend());
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::reverse_iterator Vector<T, Allocator>::rend() noexcept
{
    return reverse_iterator(begin());
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::rend() const noexcept
{
    return const_reverse_iterator(cbegin());
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::crend() const noexcept
{
    return const_reverse_iterator(cbegin());
}

template <typename T, typename Allocator>
bool Vector<T, Allocator>::empty() const noexcept
{
    return (size_ == 0);
}

template <typename T, typename Allocator>
std::size_t Vector<T, Allocator>::size() const noexcept
{
    return size_;
}

template <typename T, typename Allocator>
std::size_t Vector<T, Allocator>::max_size() const noexcept
{
    return buffer_.max_size();
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::reserve(std::size_t new_capacity)
{
    return buffer_.reserve(new_capacity);
}

template <typename T, typename Allocator>
std::size_t Vector<T, Allocator>::capacity() const noexcept
{
    return buffer_.capacity();
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::clear() noexcept
{
    size_ = 0;
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos,
                                                                     const T& value)
{
    if ((pos < cbegin()) || (pos > cend()))
    {
        throw std::out_of_range("Vector::insert(): pos out of bounds");
    }

    const std::ptrdiff_t index = pos - cbegin();

    if (buffer_reached_capacity())
    {
        buffer_.resize(buffer_.size() + 1);
    }

    iterator new_pos = begin() + index;
    static_cast<void>(std::move_backward(new_pos, end(), end() + 1));
    *new_pos = value;
    ++size_;

    return new_pos;
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos, T&& value)
{
    if ((pos < cbegin()) || (pos > cend()))
    {
        throw std::out_of_range("Vector::insert(): pos out of bounds");
    }

    const std::ptrdiff_t index = pos - cbegin();

    if (buffer_reached_capacity())
    {
        buffer_.resize(buffer_.size() + 1);
    }

    iterator new_pos = begin() + index;
    static_cast<void>(std::move_backward(new_pos, end(), end() + 1));
    *new_pos = std::move(value);
    ++size_;

    return new_pos;
}

template <typename T, typename Allocator>
template <typename... Args>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::emplace(const_iterator pos,
                                                                      Args&&... args)
{
    if ((pos < cbegin()) || (pos > cend()))
    {
        throw std::out_of_range("Vector::emplace(): iterator out of range");
    }

    const std::ptrdiff_t index = pos - cbegin();

    if (buffer_reached_capacity())
    {
        buffer_.resize(buffer_.size() + 1);
    }

    iterator new_pos = begin() + index;
    static_cast<void>(std::move_backward(new_pos, end(), end() + 1));
    *new_pos = T(std::forward<Args>(args)...);
    ++size_;

    return new_pos;
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos,
                                                                     size_type count,
                                                                     const T& value)
{
    if ((pos < cbegin()) || (pos > cend()))
    {
        throw std::out_of_range("Vector::insert(): pos out of bounds");
    }

    if (count == 0)
    {
        return iterator(const_cast<T*>(pos.operator->()));
    }

    const std::ptrdiff_t index = pos - cbegin();
    const std::size_t size_after_insertion = size_ + count;
    if (size_after_insertion > buffer_.size())
    {
        buffer_.resize(size_after_insertion);
    }

    iterator new_pos = begin() + index;
    static_cast<void>(
        std::move_backward(new_pos, end(), end() + static_cast<std::ptrdiff_t>(count)));
    std::fill(new_pos, new_pos + static_cast<std::ptrdiff_t>(count), value);
    size_ = size_after_insertion;

    return new_pos;
}

template <typename T, typename Allocator>
template <typename InputIt,
          typename std::enable_if<!std::is_integral<InputIt>::value, InputIt>::type*>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos,
                                                                     InputIt first,
                                                                     InputIt last)
{
    if ((pos < cbegin()) || (pos > cend()))
    {
        throw std::out_of_range("Vector::insert(): pos out of bounds");
    }

    const std::ptrdiff_t count = last - first;
    if (count <= 0)
    {
        return iterator(const_cast<T*>(pos.operator->()));
    }

    const std::ptrdiff_t index = pos - cbegin();
    const std::size_t size_after_insertion = size_ + static_cast<std::size_t>(count);
    if (size_after_insertion > buffer_.size())
    {
        buffer_.resize(size_after_insertion);
    }

    iterator new_pos = begin() + index;
    static_cast<void>(std::move_backward(new_pos, end(), end() + count));
    static_cast<void>(std::copy(first, last, new_pos));

    size_ = size_after_insertion;

    return new_pos;
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos,
                                                                     std::initializer_list<T> ilist)
{
    if ((pos < cbegin()) || (pos > cend()))
    {
        throw std::out_of_range("Vector::insert(): pos out of bounds");
    }

    const auto count = ilist.size();
    if (count <= 0)
    {
        return iterator(const_cast<T*>(pos.operator->()));
    }

    const std::ptrdiff_t index = pos - cbegin();
    const std::size_t size_after_insertion = size_ + static_cast<std::size_t>(count);
    if (size_after_insertion > buffer_.size())
    {
        buffer_.resize(size_after_insertion);
    }

    iterator new_pos = begin() + index;
    static_cast<void>(std::move_backward(new_pos, end(), end() + count));
    static_cast<void>(std::copy(ilist.begin(), ilist.end(), new_pos));
    size_ = size_after_insertion;

    return new_pos;
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::erase(const_iterator pos)
{
    if ((pos < cbegin()) || (pos >= end()))
    {
        throw std::out_of_range("Vector::erase(): iterators out of range");
    }

    const std::ptrdiff_t index = pos - cbegin();
    iterator mut_pos = begin() + index;
    iterator next = mut_pos + 1;

    if (next != end())
    {
        static_cast<void>(std::move(next, end(), mut_pos));
    }
    --size_;

    return mut_pos;
}

template <typename T, typename Allocator>
typename Vector<T, Allocator>::iterator Vector<T, Allocator>::erase(const_iterator first,
                                                                    const_iterator last)
{
    if ((first < cbegin()) || (first > last) || (last > cend()))
    {
        throw std::out_of_range("Vector::erase(): iterators out of range");
    }

    if (first == last)
    {
        return iterator(const_cast<T*>(first.operator->()));
    }

    const std::ptrdiff_t first_index = first - cbegin();
    const std::ptrdiff_t last_index = last - cbegin();
    const std::ptrdiff_t num_elements_to_erase = last_index - first_index;

    iterator mut_first = begin() + first_index;
    iterator mut_last = begin() + last_index;

    if (mut_last != end())
    {
        static_cast<void>(std::move(mut_last, end(), mut_first));
    }

    size_ -= static_cast<std::size_t>(num_elements_to_erase);

    return mut_first;
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::push_back(const T& value)
{
    if (buffer_full())
    {
        buffer_.push_back(value);
    }
    else
    {
        buffer_[size_] = value;
    }

    ++size_;
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::push_back(T&& value)
{
    if (buffer_full())
    {
        buffer_.push_back(std::move(value));
    }
    else
    {
        buffer_[size_] = std::move(value);
    }

    ++size_;
}

template <typename T, typename Allocator>
template <typename... Args>
void Vector<T, Allocator>::emplace_back(Args&&... args)
{
    if (buffer_full())
    {
        buffer_.emplace_back(std::forward<Args>(args)...);
    }
    else
    {
        buffer_[size_] = T(std::forward<Args>(args)...);
    }

    ++size_;
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::pop_back() noexcept
{
    if (size_ > 0)
    {
        --size_;
    }
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::resize(std::size_t new_size)
{
    if (buffer_.size() < new_size)
    {
        buffer_.resize(new_size);
    }
    size_ = new_size;
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::resize(std::size_t new_size, const T& value)
{
    if (buffer_.size() < new_size)
    {
        buffer_.resize(new_size, value);
    }
    else
    {
        for (auto& buffer_value : buffer_)
        {
            buffer_value = value;
        }
    }
    size_ = new_size;
}

template <typename T, typename Allocator>
void Vector<T, Allocator>::swap(Vector& other) noexcept(
    std::allocator_traits<Allocator>::propagate_on_container_swap::value ||
    std::allocator_traits<Allocator>::is_always_equal::value)
{
    if (this != &other)
    {
        std::swap(buffer_, other.buffer_);
        std::swap(size_, other.size_);
    }
}

template <typename T, typename Allocator>
void swap(Vector<T, Allocator>& first, Vector<T, Allocator>& second) noexcept(
    std::allocator_traits<Allocator>::propagate_on_container_swap::value ||
    std::allocator_traits<Allocator>::is_always_equal::value)
{
    first.swap(second);
}

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__VECTOR_HPP
