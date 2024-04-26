#ifndef CIRCULAR_QUEUE_HPP
#define CIRCULAR_QUEUE_HPP

#include <cstdint>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace containers
{
template <typename T>
class CircularQueue final
{
  public:
    CircularQueue(const CircularQueue&) = delete;
    CircularQueue& operator=(const CircularQueue&) = delete;
    CircularQueue(CircularQueue&&) = delete;
    CircularQueue& operator=(CircularQueue&&) = delete;
    CircularQueue();
    CircularQueue(std::size_t initial_capacity);
    ~CircularQueue() noexcept;

    void reserve(std::size_t new_capacity);
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    void push(const T& value);
    void push(T&& value);
    template <typename... Args>
    void emplace(Args&&... args);
    void pop();
    T& front();
    const T& front() const;

  private:
    std::vector<T> buffer_;
    std::size_t head_; // Index of the front element
    std::size_t tail_; // Index of the next available slot
    std::size_t size_;
    std::size_t capacity_;

    void resize(std::size_t new_capacity);
    void clear() noexcept;
};
} // namespace containers

#endif // CIRCULAR_QUEUE_HPP
