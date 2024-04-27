#ifndef CONTAINERS_VECTOR_HPP
#define CONTAINERS_VECTOR_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace containers
{
template<typename T, typename Allocator = std::allocator<T>>
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
    Vector();
    explicit Vector(std::size_t initial_capacity);
    ~Vector() noexcept = default;

    Vector(const Vector&) = default;
    Vector(Vector&&) noexcept(std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
                              std::allocator_traits<Allocator>::is_always_equal::value) = default;
    Vector& operator=(const Vector&) = default;
    Vector&
    operator=(Vector&&) noexcept(std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
                                 std::allocator_traits<Allocator>::is_always_equal::value) = default;

    Vector(const std::vector<T, Allocator>& vec);
    Vector(std::vector<T, Allocator>&& vec);
    Vector& operator=(const std::vector<T, Allocator>& vec);
    Vector& operator=(std::vector<T, Allocator>&& vec) noexcept(
        std::allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
        std::allocator_traits<Allocator>::is_always_equal::value);
    Vector& operator=(std::initializer_list<T> ilist);

    // Element access
    T& at(std::size_t index);
    const T& at(std::size_t index) const;
    T& operator[](std::size_t index) noexcept;
    const T& operator[](std::size_t index) const noexcept;
    T& front() noexcept;
    const T& front() const noexcept;
    T& back() noexcept;
    const T& back() const noexcept;
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
    template<class InputIt>
    iterator insert(const_iterator pos, InputIt first, InputIt last);
    iterator insert(const_iterator pos, std::initializer_list<T> ilist);
    template<class... Args>
    iterator emplace(const_iterator pos, Args&&... args);
    iterator erase(const_iterator pos);
    iterator erase(const_iterator first, const_iterator last);
    void push_back(const T& value);
    void push_back(T&& value);
    template<typename... Args>
    void emplace_back(Args&&... args);
    void pop_back() noexcept;
    void resize(std::size_t new_size);
    void swap(Vector& other) noexcept(std::allocator_traits<Allocator>::propagate_on_container_swap::value ||
                                      std::allocator_traits<Allocator>::is_always_equal::value);

  private:
    std::vector<T, Allocator> buffer_;
    std::size_t size_;
};

template<typename T, typename Allocator>
Vector<T, Allocator>::Vector()
    : Vector(0)
{
}

template<typename T, typename Allocator>
Vector<T, Allocator>::Vector(std::size_t initial_capacity)
    : buffer_(),
      size_(0),
      capacity_(initial_capacity)
{
    buffer_.reserve(initial_capacity);
}

template<typename T, typename Allocator>
T& Vector<T, Allocator>::at(std::size_t index)
{
    return buffer_.at(index);
}

template<typename T, typename Allocator>
const T& Vector<T, Allocator>::at(std::size_t index) const
{
    return buffer_.at(index);
}

template<typename T, typename Allocator>
T& Vector<T, Allocator>::operator[](std::size_t index) noexcept
{
    return buffer_[index];
}

template<typename T, typename Allocator>
const T& Vector<T, Allocator>::operator[](std::size_t index) const noexcept
{
    return buffer_[index];
}

template<typename T, typename Allocator>
T& Vector<T, Allocator>::front() noexcept
{
    return buffer_.front();
}

template<typename T, typename Allocator>
const T& Vector<T, Allocator>::front() const noexcept
{
    return buffer_.front();
}

template<typename T, typename Allocator>
T& Vector<T, Allocator>::back() noexcept
{
    return buffer_.back();
}

template<typename T, typename Allocator>
const T& Vector<T, Allocator>::back() const noexcept
{
    return buffer_.back();
}

template<typename T, typename Allocator>
T* Vector<T, Allocator>::data() noexcept
{
    return buffer_.data();
}

template<typename T, typename Allocator>
const T* Vector<T, Allocator>::data() const noexcept
{
    return buffer_.data();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::begin()
{
    return buffer_.begin();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_iterator Vector<T, Allocator>::begin() const
{
    return buffer_.begin();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_iterator Vector<T, Allocator>::cbegin() const noexcept
{
    return buffer_.cbegin();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::end() noexcept
{
    return buffer_.end();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_iterator Vector<T, Allocator>::end() const noexcept
{
    return buffer_.end();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_iterator Vector<T, Allocator>::cend() const noexcept
{
    return buffer_.cend();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::reverse_iterator Vector<T, Allocator>::rbegin()
{
    return buffer_.rbegin();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::rbegin() const
{
    return buffer_.rbegin();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::crbegin() const noexcept
{
    return buffer_.crbegin();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::reverse_iterator Vector<T, Allocator>::rend() noexcept
{
    return buffer_.rend();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::rend() const noexcept
{
    return buffer_.rend();
}

template<typename T, typename Allocator>
Vector<T, Allocator>::const_reverse_iterator Vector<T, Allocator>::crend() const noexcept
{
    return buffer_.crend();
}

template<typename T, typename Allocator>
bool Vector<T, Allocator>::empty() const noexcept
{
    return (size_ == 0);
}

template<typename T, typename Allocator>
std::size_t Vector<T, Allocator>::size() const noexcept
{
    return size_;
}

template<typename T, typename Allocator>
std::size_t Vector<T, Allocator>::max_size() const noexcept
{
    return buffer_.max_size();
}

template<typename T, typename Allocator>
void Vector<T, Allocator>::reserve(std::size_t new_capacity)
{
    return buffer_.reserve(new_capacity);
}

template<typename T, typename Allocator>
std::size_t Vector<T, Allocator>::capacity() const noexcept
{
    return buffer_.capacity();
}

template<typename T, typename Allocator>
void Vector<T, Allocator>::clear() noexcept
{
    size_ = 0;
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos, const T& value)
{
    return buffer_.insert(pos, value);
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos, T&& value)
{
    return buffer_.insert(pos, std::move(value));
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos, size_type count, const T& value)
{
    return buffer_.insert(pos, count, value);
}

template<typename T, typename Allocator>
template<class InputIt>
Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos, InputIt first, InputIt last)
{
    return buffer_.insert(pos, first, last);
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::insert(const_iterator pos, std::initializer_list<T> ilist)
{
    return buffer_.insert(pos, ilist);
}

template<typename T, typename Allocator>
template<class... Args>
Vector<T, Allocator>::iterator Vector<T, Allocator>::emplace(const_iterator pos, Args&&... args)
{
    return buffer_.emplace(pos, std::forward<Args>(args)...);
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::erase(const_iterator pos)
{
    return buffer_.erase(pos);
}

template<typename T, typename Allocator>
Vector<T, Allocator>::iterator Vector<T, Allocator>::erase(const_iterator first, const_iterator last)
{
    return buffer_.erase(first, last);
}

template<typename T, typename Allocator>
void Vector<T, Allocator>::push_back(const T& value)
{
    buffer_.push_back(value);
}

template<typename T, typename Allocator>
void Vector<T, Allocator>::push_back(T&& value)
{
    buffer_.push_back(std::move(value));
}

template<typename T, typename Allocator>
template<typename... Args>
void Vector<T, Allocator>::emplace_back(Args&&... args)
{
    buffer_.emplace_back(std::forward<Args>(args)...);
}

template<typename T, typename Allocator>
void Vector<T, Allocator>::pop_back() noexcept
{
    buffer_.pop_back();
}

template<typename T, typename Allocator>
void Vector<T, Allocator>::resize(std::size_t new_size)
{
    buffer_.resize(new_size);
}

template<typename T, typename Allocator>
void Vector<T, Allocator>::swap(Vector& other) noexcept(
    std::allocator_traits<Allocator>::propagate_on_container_swap::value ||
    std::allocator_traits<Allocator>::is_always_equal::value)
{
    buffer_.swap(other);
}

template<typename T, typename Allocator>
void swap(Vector<T, Allocator>& first,
          Vector<T, Allocator>& second) noexcept(std::allocator_traits<Allocator>::propagate_on_container_swap::value ||
                                                 std::allocator_traits<Allocator>::is_always_equal::value)
{
    first.swap(second);
}

} // namespace containers

#endif // CONTAINERS_VECTOR_HPP
