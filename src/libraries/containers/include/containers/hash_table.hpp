#ifndef CONTAINERS_HASH_TABLE_HPP
#define CONTAINERS_HASH_TABLE_HPP

#include "vector.hpp"

#include <cstdint>
#include <utility>

namespace containers
{
template<typename Key, typename Value, typename Hash = std::hash<Key>>
class HashTable final
{
  public:
    static constexpr double MAX_LOAD_FACTOR = 2.5;

    HashTable();
    explicit HashTable(std::size_t num_buckets);
    explicit HashTable(std::size_t num_buckets, std::size_t num_collisions);
    void resize(std::size_t num_buckets);
    void reserve_bucket_entries(std::size_t num_collisions);
    void resize(std::size_t num_buckets, std::size_t num_collisions);
    template<typename KeyT, typename ValueT>
    void insert(KeyT&& key, ValueT&& value);
    template<typename KeyT>
    Value* find(KeyT&& key) const;
    template<typename KeyT>
    bool remove(KeyT&& key);

  private:
    using Bucket = Vector<std::pair<Key, Value>>;

    Vector<Bucket> table_;
    Hash hash_;
    std::size_t num_elements_;

    void resize(std::size_t num_buckets);
    void check_and_resize();
};

template<typename Key, typename Value, typename Hash>
HashTable<Key, Value, Hash>::HashTable()
    : table_(100)
{
}

template<typename Key, typename Value, typename Hash>
HashTable<Key, Value, Hash>::HashTable(std::size_t num_buckets)
{
    if (table_.size() < num_buckets)
    {
        resize(num_buckets);
    }
}

template<typename Key, typename Value, typename Hash>
HashTable<Key, Value, Hash>::HashTable(std::size_t num_buckets, std::size_t num_collisions)
{
    resize(num_buckets, num_collisions);
}

template<typename Key, typename Value, typename Hash>
void HashTable<Key, Value, Hash>::resize(std::size_t num_buckets)
{
    if (num_buckets == 0)
    {
        throw std::runtime_error("Invalid number of buckets");
    }

    if (table_.size() < num_buckets)
    {
        table_.resize(num_buckets);
    }
}

template<typename Key, typename Value, typename Hash>
void HashTable<Key, Value, Hash>::reserve_bucket_entries(std::size_t num_collisions)
{
    for (Bucket& bucket : table_)
    {
        bucket.reserve(num_collisions);
    }
}

template<typename Key, typename Value, typename Hash>
void HashTable<Key, Value, Hash>::resize(std::size_t num_buckets, std::size_t num_collisions)
{
    if (num_buckets == 0)
    {
        throw std::runtime_error("Invalid number of buckets");
    }

    resize(num_buckets);
    reserve_bucket_entries(num_collisions);
}

template<typename Key, typename Value, typename Hash>
template<typename KeyT, typename ValueT>
void HashTable<Key, Value, Hash>::insert(KeyT&& key, ValueT&& value)
{
    static_assert(std::is_same<KeyT, Key>::value, "KeyT must be exactly the same as Key");
    static_assert(std::is_same<ValueT, Value>::value, "ValueT must be exactly the same as Value");

    const std::size_t index = hash_(key) % table_.size();
    Bucket& bucket = table_[index];

    for (std::pair<Key, Value>& collision : bucket)
    {
        if (collision.first == key)
        {
            collision.second = std::forward<ValueT>(value);
            return;
        }
    }

    bucket.emplace_back(std::forward<KeyT>(key), std::forward<ValueT>(value));
}

template<typename Key, typename Value, typename Hash>
template<typename KeyT>
Value* HashTable<Key, Value, Hash>::find(KeyT&& key) const
{
    static_assert(std::is_same<KeyT, Key>::value, "KeyT must be exactly the same as Key");

    const std::size_t index = hash_(key) % table_.size();
    Bucket& bucket = table_[index];

    for (std::pair<Key, Value>& collision : bucket)
    {
        if (collision.first == key)
        {
            return &collision.second;
        }
    }

    return nullptr;
}

template<typename Key, typename Value, typename Hash>
template<typename KeyT>
bool HashTable<Key, Value, Hash>::remove(KeyT&& key)
{
    static_assert(std::is_same<KeyT, Key>::value, "KeyT must be exactly the same as Key");

    const std::size_t index = hash_(key) % table_.size();
    Bucket& bucket = table_[index];

    for (auto bucket_it = bucket.cbegin(); bucket_it != bucket.cend(); ++bucket_it)
    {
        if (bucket_it->first == key)
        {
            bucket.erase(bucket_it);
            --num_elements_;

            return true; // Successfully erased the key
        }
    }

    return false; // Key not found
}

} // namespace containers

#endif // CONTAINERS_HASH_TABLE_HPP
