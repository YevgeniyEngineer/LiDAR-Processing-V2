#ifndef UTILITIES_LIB_STATIC_VECTOR_HPP
#define UTILITIES_LIB_STATIC_VECTOR_HPP

#include <algorithm>   // std::copy
#include <cstddef>     // std::ptrdiff_t
#include <cstdint>     // std::size_t
#include <cstdlib>     // std::atexit
#include <iostream>    // std::cerr
#include <iterator>    // std::random_access_iterator
#include <memory>      // std::allocator
#include <new>         // ::new
#include <stdexcept>   // std::runtime_error, std::out_of_range
#include <type_traits> // std::is_same_v
#include <utility>     // std::forward, std::move, std::swap

namespace utilities_lib
{
template <typename T>
class StaticVector final
{
  public:
    using size_type = std::size_t;
    using value_type = T;
    using pointer = T*;
    using reference = T&;
    using const_reference = const T&;

    StaticVector() : current_size_{0}
    {
        if (0U == capacity_)
        {
            reallocate(1, current_size_);
        }

        if (!cleanup_registered_)
        {
            std::atexit(cleanup);
            cleanup_registered_ = true;
        }
    }

    ~StaticVector() noexcept
    {
        clear();
    }

    void push_back(const T& value)
    {
        if (current_size_ >= capacity_)
        {
            reallocate(capacity_ * 2U, current_size_);
        }
        else
        {
            elements_[current_size_].~T();
        }

        ::new (&elements_[current_size_]) T{value};
        ++current_size_;
    }

    void push_back(T&& value)
    {
        if (current_size_ >= capacity_)
        {
            reallocate(capacity_ * 2U, current_size_);
        }
        else
        {
            elements_[current_size_].~T();
        }

        ::new (&elements_[current_size_]) T{std::move(value)};
        ++current_size_;
    }

    template <typename... Args>
    void emplace_back(Args&&... args)
    {
        if (current_size_ >= capacity_)
        {
            reallocate(capacity_ * 2U, current_size_);
        }
        else
        {
            elements_[current_size_].~T();
        }

        ::new (&elements_[current_size_]) T{std::forward<Args>(args)...};
        ++current_size_;
    }

    reference at(size_type index)
    {
        if (index >= current_size_)
        {
            throw std::out_of_range("Index out of range");
        }

        return elements_[index];
    }

    const_reference at(size_type index) const
    {
        if (index >= current_size_)
        {
            throw std::out_of_range("Index out of range");
        }

        return elements_[index];
    }

    reference operator[](size_type index) noexcept
    {
        return elements_[index];
    }

    const_reference operator[](size_type index) const noexcept
    {
        return elements_[index];
    }

    size_type size() const noexcept
    {
        return current_size_;
    }

    bool empty() const noexcept
    {
        return (current_size_ == 0U);
    }

    void clear() noexcept
    {
        current_size_ = 0U;
    }

    size_type capacity() const noexcept
    {
        return capacity_;
    }

  private:
    static pointer elements_;
    static size_type capacity_;
    size_type current_size_{0U};
    static bool cleanup_registered_;

    static void reallocate(size_type new_capacity, size_type current_size)
    {
        if (new_capacity <= capacity_)
        {
            return;
        }

        pointer new_elements = new T[new_capacity];

        if (elements_)
        {
            std::move(elements_, elements_ + current_size, new_elements);

            // No manual destructor calls are necessary here, let delete[] handle it
            delete[] elements_;
        }

        elements_ = new_elements;
        capacity_ = new_capacity;
    }

    static void cleanup() noexcept
    {
        delete[] elements_;
        capacity_ = 0U;
        cleanup_registered_ = false;

        std::cerr << "Deallocated memory for T = " << typeid(T).name() << std::endl;
    }
};

template <typename T>
typename StaticVector<T>::pointer StaticVector<T>::elements_ = nullptr;

template <typename T>
typename StaticVector<T>::size_type StaticVector<T>::capacity_ = 0U;

template <typename T>
bool StaticVector<T>::cleanup_registered_ = false;

} // namespace utilities_lib

#endif // UTILITIES_LIB_STATIC_VECTOR_HPP
