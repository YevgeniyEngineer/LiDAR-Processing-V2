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

#ifndef LIDAR_PROCESSING_LIB__STATIC_UNORDERED_SET_HPP
#define LIDAR_PROCESSING_LIB__STATIC_UNORDERED_SET_HPP

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
template <typename Key, typename Hash = std::hash<Key>>
class StaticUnorderedSet final
{
  public:
    StaticUnorderedSet(float max_load_factor = 0.75F) : max_load_factor_(max_load_factor)
    {
        if (max_load_factor_ <= 0 || max_load_factor_ >= 1)
        {
            throw std::invalid_argument{"Max load factor must be between 0 and 1"};
        }

        buffers_[0].resize(1, std::nullopt);
        buffers_[1].resize(1, std::nullopt);
    }

    void reserve(std::uint32_t num_buckets, std::uint32_t num_elements)
    {
        if (num_buckets == 0 || num_elements == 0)
        {
            throw std::invalid_argument{"Number of buckets and elements must be greater than 0"};
        }

        buffers_[0].resize(num_buckets, std::nullopt);
        buffers_[1].resize(num_buckets, std::nullopt);
        elements_.resize(num_elements);
        max_elements_ = num_elements;
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
            hash = (hash + 1) % buffer.size();
        } while (hash != start);

        return false;
    }

    void insert(const Key& key)
    {
        if (element_count_ >= max_elements_)
        {
            throw std::overflow_error{"Exceeded reserved element capacity"};
        }

        if (loadFactor() > max_load_factor_)
        {
            rehash(buffers_[buffer_index_].size() * 2);
        }

        auto& buffer = buffers_[buffer_index_];
        std::size_t hash = hashKey(key) % buffer.size();
        const std::size_t start = hash;

        do
        {
            if (!buffer[hash])
            {
                elements_.push_back(key);
                buffer[hash] = {key};
                ++element_count_;
                return;
            }
            if (buffer[hash]->key == key)
            {
                return;
            }
            hash = (hash + 1) % buffer.size();
        } while (hash != start);

        throw std::overflow_error{"Hash table is full"};
    }

    bool find(const Key& key) const
    {
        return contains(key);
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

  private:
    struct Bucket final
    {
        Key key;
    };

    std::vector<std::optional<Bucket>> buffers_[2];
    std::vector<Key> elements_;
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

    void rehash(std::uint32_t new_size)
    {
        const std::int32_t new_buffer_index = 1 - buffer_index_;
        auto& new_buffer = buffers_[new_buffer_index];
        new_buffer.resize(new_size, std::nullopt);

        for (const auto& bucket : buffers_[buffer_index_])
        {
            if (bucket.has_value())
            {
                std::size_t hash = hashKey(bucket->key) % new_size;
                while (new_buffer[hash].has_value())
                {
                    hash = (hash + 1) % new_size;
                }
                new_buffer[hash] = bucket;
            }
        }

        buffer_index_ = new_buffer_index;
    }
};

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__STATIC_UNORDERED_SET_HPP
