#ifndef CONTAINERS_UNORDERED_MAP_HPP
#define CONTAINERS_UNORDERED_MAP_HPP

#include "hash_table.hpp"
#include "vector.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <new>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace containers
{
// See:
// https://blog.andreiavram.ro/cpp-map-static-memory/?fbclid=IwZXh0bgNhZW0CMTAAAR3sNstt0fjqJEY1Y35-rbRkhiQUa3tIsmoHd1U2UjgvKEMDfuDidGZKl5s_aem_ARjVNBbQ8i9kEEOpDx3Yw3Ly38nyg57iSWytBa8SYZDLLiiihk9DEffoRkgO8TD8h0USWKpw_Fv6BnmLC6EfIwrY
// https://en.cppreference.com/w/cpp/container/unordered_map
template<typename Key, typename Value, typename Hash = std::hash<Value>>
class UnorderedMap final
{
  public:
    using Keys = Vector<Key>;
    using Values = Vector<Value>;
    using Set = Vector<bool>;

    UnorderedMap();
    ~UnorderedMap() noexcept = default;
    UnorderedMap(const UnorderedMap& other);
    UnorderedMap(UnorderedMap&& other) noexcept;
    UnorderedMap& operator=(const UnorderedMap& other) &;
    UnorderedMap& operator=(UnorderedMap&& other) & noexcept;
    UnorderedMap(std::initializer_list<std::pair<Key, Value>> ilist);
    UnorderedMap& operator=(std::initializer_list<std::pair<Key, Value>> ilist);

    Value& at(const Key& key);
    const Value& at(const Key& key) const;
    Value& operator[](const Key& key);
    const Value& operator[](const Key& key) const;
    bool empty() const noexcept;
    bool contains(const Key& key) const noexcept;
    std::size_t size() const noexcept;
    std::size_t capacity() const noexcept;
    void clear() noexcept;
    iterator find(const Key& key);
    iterator begin() noexcept;
    iterator end() noexcept;
    iterator begin() const noexcept;
    iterator end() const noexcept;
    const_iterator cbegin() const noexcept;
    const iterator cend() const noexcept;
    std::pair<iterator, bool> insert(const std::pair<Key, Value>& value);
    std::pair<iterator, bool> insert(std::pair<Key, Value>&& value);
    template<typename... Args>
    std::pair<iterator, bool> emplace(Args&&... args);
    iterator erase(const_iterator pos);
    iterator erase(const_iterator first, const_iterator last);
    void swap(UnorderedMap& other);

  private:
    Values values_;
    Keys keys_;
    Set set_;
    Hash hash_;
};

template<typename Key, typename Value, typename Hash>
Value& UnorderedMap<Key, Value, Hash>::at(const Key& key)
{
    const auto index = hash_(key);

    if (index >= set_.size())
    {
        throw std::out_of_range("UnorderedMap::at(): index out of range");
    }

    if (!set_[index])
    {
        throw std::out_of_range("UnorderedMap::at(): key not found");
    }

    return values_[index];
}

template<typename Key, typename Value, typename Hash>
const Value& UnorderedMap<Key, Value, Hash>::at(const Key& key) const
{
    const auto index = hash_(key);

    if (index >= set_.size())
    {
        throw std::out_of_range("UnorderedMap::at(): index out of range");
    }

    if (!set_[index])
    {
        throw std::out_of_range("UnorderedMap::at(): key not found");
    }

    return values_[index];
}

template<typename Key, typename Value, typename Hash>
Value& UnorderedMap<Key, Value, Hash>::operator[](const Key& key)
{
    const auto index = hash_(key);

    keys_[index] = key;
    set_[index] = true;

    return values_[index];
}

template<typename Key, typename Value, typename Hash>
const Value& UnorderedMap<Key, Value, Hash>::operator[](const Key& key) const
{
    const auto index = hash_(key);

    keys_[index] = key;
    set_[index] = true;

    return values_[index];
}

template<typename Key, typename Value, typename Hash>
std::size_t UnorderedMap<Key, Value, Hash>::size() const noexcept
{
    return static_cast<std::size_t>(std::count(set_.cbegin(), set_.cend(), true));
}

template<typename Key, typename Value, typename Hash>
std::size_t UnorderedMap<Key, Value, Hash>::capacity() const noexcept
{
    return set_.capacity();
}

template<typename Key, typename Value, typename Hash>
void UnorderedMap<Key, Value, Hash>::clear() noexcept
{
    std::fill(set_.begin(), set_.end(), false);
}

template<typename Key, typename Value, typename Hash>
bool UnorderedMap<Key, Value, Hash>::empty() const noexcept
{
    return std::any_of(set_.cbegin(), set_.cend(), [](const auto& value) -> bool { return value; });
}

template<typename Key, typename Value, typename Hash>
bool UnorderedMap<Key, Value, Hash>::contains(const Key& key) const noexcept
{
    const auto index = hash_(key);

    return set_[index];
}

template<typename Key, typename Value, typename Hash>
void swap(UnorderedMap<Key, Value, Hash>& first, UnorderedMap<Key, Value, Hash>& second)
{
    first.swap(second);
}
} // namespace containers

#endif // CONTAINERS_UNORDERED_MAP_HPP
