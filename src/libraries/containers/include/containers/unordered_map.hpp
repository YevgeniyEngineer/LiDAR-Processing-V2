#ifndef CONTAINERS_UNORDERED_MAP_HPP
#define CONTAINERS_UNORDERED_MAP_HPP

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
    Value& at(Key&& key);
    const Value& at(Key&& key) const;
    Value& operator[](const Key& key);
    const Value& operator[](const Key& key) const;
    Value& operator[](Key&& key);
    const Value& operator[](Key&& key) const;
    bool empty() const noexcept;
    bool contains(const Key& key) const noexcept;
    std::size_t size() const noexcept;
    std::size_t max_size() const noexcept;
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
void swap(UnorderedMap<Key, Value, Hash>& first, UnorderedMap<Key, Value, Hash>& second)
{
}
} // namespace containers

#endif // CONTAINERS_UNORDERED_MAP_HPP
