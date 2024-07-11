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

#ifndef LIDAR_PROCESSING_LIB__STATIC_UNORDERED_MAP_HPP
#define LIDAR_PROCESSING_LIB__STATIC_UNORDERED_MAP_HPP

// STL
#include <cstdint>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_processing_lib
{
template <typename Key, typename Value, typename Hash = std::hash<Key>>
class StaticUnorderedMap final
{
  public:
    StaticUnorderedMap(float max_load_factor = 0.75F) : max_load_factor_(max_load_factor)
    {
        if (max_load_factor_ <= 0 || max_load_factor_ >= 1)
        {
            throw std::invalid_argument{"Max load factor must be between 0 and 1"};
        }

        buffers_[0].resize(2, std::nullopt);
        buffers_[1].resize(2, std::nullopt);
    }

    void reserve(std::uint32_t num_buckets, std::uint32_t num_elements)
    {
        if (num_buckets == 0 || num_elements == 0)
        {
            throw std::invalid_argument{"Number of buckets and elements must be greater than 0"};
        }

        num_buckets = nextPowerOfTwo(num_buckets);

        buffers_[0].resize(num_buckets, std::nullopt);
        buffers_[1].resize(num_buckets, std::nullopt);
        elements_.resize(num_elements);
        max_elements_ = num_elements;
    }

    void insert(const Key& key, const Value& value)
    {
        checkAndRehash();

        auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash])
            {
                if (element_count_ >= max_elements_)
                {
                    throw std::overflow_error{"Exceeded reserved element capacity"};
                }
                elements_[element_count_] = value;
                buffer[hash] = {key, &elements_[element_count_]};
                ++element_count_;
                return;
            }
            if (buffer[hash]->key == key)
            {
                *(buffer[hash]->value_ptr) = value;
                return;
            }
            hash = (hash + 1) & (buffer.size() - 1);
        } while (hash != start);

        throw std::overflow_error{"Hash table is full"};
    }

    Value& operator[](const Key& key)
    {
        checkAndRehash();

        auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash])
            {
                if (element_count_ >= max_elements_)
                {
                    throw std::overflow_error{"Exceeded reserved element capacity"};
                }
                buffer[hash] = {key, &elements_[element_count_]};
                ++element_count_;
                return elements_[element_count_ - 1];
            }
            if (buffer[hash]->key == key)
            {
                return *(buffer[hash]->value_ptr);
            }
            hash = (hash + 1) & (buffer.size() - 1);
        } while (hash != start);

        throw std::overflow_error{"Hash table is full"};
    }

    const Value& operator[](const Key& key) const
    {
        auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash])
            {
                throw std::out_of_range{"Key not found"};
            }
            if (buffer[hash]->key == key)
            {
                return *(buffer[hash]->value_ptr);
            }
            hash = (hash + 1) & (buffer.size() - 1);
        } while (hash != start);

        throw std::out_of_range{"Key not found"};
    }

    Value& at(const Key& key)
    {
        auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash])
            {
                break;
            }
            if (buffer[hash]->key == key)
            {
                return *(buffer[hash]->value_ptr);
            }
            hash = (hash + 1) & (buffer.size() - 1);
        } while (hash != start);

        throw std::out_of_range{"Key not found"};
    }

    const Value& at(const Key& key) const
    {
        const auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash])
            {
                break;
            }
            if (buffer[hash]->key == key)
            {
                return *(buffer[hash]->value_ptr);
            }
            hash = (hash + 1) & (buffer.size() - 1);
        } while (hash != start);

        throw std::out_of_range{"Key not found"};
    }

    Value* find(const Key& key) const
    {
        const auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash])
            {
                return nullptr;
            }
            if (buffer[hash]->key == key)
            {
                return buffer[hash]->value_ptr;
            }
            hash = (hash + 1) & (buffer.size() - 1);
        } while (hash != start);

        return nullptr;
    }

    bool contains(const Key& key) const
    {
        const auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash].has_value())
            {
                return false;
            }
            if (buffer[hash]->key == key)
            {
                return true;
            }
            hash = (hash + 1) & (buffer.size() - 1);
        } while (hash != start);

        return false;
    }

    void clear()
    {
        std::fill(buffers_[0].begin(), buffers_[0].end(), std::nullopt);
        std::fill(buffers_[1].begin(), buffers_[1].end(), std::nullopt);
        element_count_ = 0;
    }

    inline std::size_t size() const noexcept
    {
        return element_count_;
    }

    inline std::size_t bucket0Size() const noexcept
    {
        return buffers_[0].size();
    }

    inline std::size_t bucket1Size() const noexcept
    {
        return buffers_[1].size();
    }

    inline float currentLoadFactor() const noexcept
    {
        return loadFactor();
    }

  private:
    struct Bucket final
    {
        Key key;
        Value* value_ptr;
    };

    std::vector<std::optional<Bucket>> buffers_[2];
    std::vector<Value> elements_;
    std::uint32_t element_count_ = 0;
    std::uint32_t max_elements_ = 0;
    std::int32_t buffer_index_ = 0;
    float max_load_factor_;
    static constexpr Hash hash_fn_{};

    inline std::size_t hashKey(const Key& key) const
    {
        return hash_fn_(key);
    }

    inline float loadFactor() const noexcept
    {
        return static_cast<float>(element_count_) /
               static_cast<float>(buffers_[buffer_index_].size());
    }

    void checkAndRehash()
    {
        if (loadFactor() > max_load_factor_)
        {
            rehash(buffers_[buffer_index_].size() * 2);
        }
    }

    void rehash(std::uint32_t new_size)
    {
        const std::int32_t new_buffer_index = 1 - buffer_index_;
        auto& new_buffer = buffers_[new_buffer_index];
        new_size = nextPowerOfTwo(new_size);
        new_buffer.resize(new_size, std::nullopt);

        for (const auto& bucket : buffers_[buffer_index_])
        {
            if (bucket.has_value())
            {
                std::size_t hash = hashKey(bucket->key) & (new_size - 1);
                while (new_buffer[hash].has_value())
                {
                    hash = (hash + 1) & (new_size - 1);
                }
                new_buffer[hash] = bucket;
            }
        }

        buffer_index_ = new_buffer_index;
    }

    std::size_t nextPowerOfTwo(std::size_t n) const noexcept
    {
        if (n == 0)
        {
            return 1;
        }
        --n;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        n |= n >> 32;
        return n + 1;
    }
};
} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STATIC_UNORDERED_MAP_HPP
