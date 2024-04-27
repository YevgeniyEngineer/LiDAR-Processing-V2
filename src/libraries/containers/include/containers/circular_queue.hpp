#ifndef CONTAINERS_CIRCULAR_QUEUE_HPP
#define CONTAINERS_CIRCULAR_QUEUE_HPP

#include <cstdint>
#include <memory>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace containers
{
template<typename T>
class CircularQueue final
{
  public:
    CircularQueue();
    explicit CircularQueue(std::size_t initial_capacity);
    ~CircularQueue() noexcept;

    CircularQueue(const CircularQueue&) = delete;
    CircularQueue& operator=(const CircularQueue&) = delete;
    CircularQueue(CircularQueue&&) = delete;
    CircularQueue& operator=(CircularQueue&&) = delete;

    void reserve(std::size_t new_capacity);
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    std::size_t capacity() const noexcept;
    void push(const T& value);
    void push(T&& value);
    template<typename... Args>
    void emplace(Args&&... args);
    void pop();
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
};

template<typename T>
CircularQueue<T>::CircularQueue()
    : CircularQueue(1)
{
}

template<typename T>
CircularQueue<T>::CircularQueue(std::size_t initial_capacity)
    : buffer_(std::make_unique<T[]>(initial_capacity)),
      head_(0),
      tail_(0),
      size_(0),
      capacity_(initial_capacity)
{
}

template<typename T>
CircularQueue<T>::~CircularQueue() noexcept
{
    clear();
}

template<typename T>
void CircularQueue<T>::reserve(std::size_t new_capacity)
{
    resize(new_capacity);
}

template<typename T>
bool CircularQueue<T>::empty() const noexcept
{
    return size_ == 0;
}

template<typename T>
std::size_t CircularQueue<T>::size() const noexcept
{
    return size_;
}

template<typename T>
std::size_t CircularQueue<T>::capacity() const noexcept
{
    return capacity_;
}

template<typename T>
void CircularQueue<T>::push(const T& value)
{
    emplace(value);
}

template<typename T>
void CircularQueue<T>::push(T&& value)
{
    emplace(std::move(value));
}

template<typename T>
template<typename... Args>
void CircularQueue<T>::emplace(Args&&... args)
{
    if (size_ >= capacity_)
    {
        resize(capacity_ * 2); // Double the capacity
    }

    ::new (&buffer_[tail_]) T(std::forward<Args>(args)...);
    tail_ = (tail_ + 1) % capacity_;
    ++size_;
}

template<typename T>
void CircularQueue<T>::pop()
{
    if (empty())
    {
        throw std::out_of_range("Pop from empty queue");
    }

    buffer_[head_].~T(); // Call destructor for the element

    head_ = (head_ + 1) % capacity_;
    --size_;
}

template<typename T>
T& CircularQueue<T>::front()
{
    if (empty())
    {
        throw std::out_of_range("CircularQueue::front(): queue is empty");
    }

    return buffer_[head_];
}

template<typename T>
const T& CircularQueue<T>::front() const
{
    if (empty())
    {
        throw std::out_of_range("CircularQueue::front(): queue is empty");
    }

    return buffer_[head_];
}

template<typename T>
void CircularQueue<T>::resize(std::size_t new_capacity)
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

        current = (current + 1) % capacity_;
    }

    // Replace the old buffer with the new one and reset head and tail
    buffer_ = std::move(new_buffer);
    head_ = 0;
    tail_ = size_; // New tail is at the index of the last element added
    capacity_ = new_capacity;
}

template<typename T>
void CircularQueue<T>::clear() noexcept
{
    while (size() > 0)
    {
        pop();
    }
}
} // namespace containers

#endif // CONTAINERS_CIRCULAR_QUEUE_HPP
