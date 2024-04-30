#ifndef DBSCAN_CLUSTERER_1D_HPP
#define DBSCAN_CLUSTERER_1D_HPP

#include "circular_queue.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace clustering
{
class Clusterer1D final
{
  public:
    static constexpr auto UNDEFINED = std::numeric_limits<std::size_t>::max();

    Clusterer1D(const Clusterer1D&) = delete;
    Clusterer1D& operator=(const Clusterer1D&) = delete;
    Clusterer1D(Clusterer1D&&) = delete;
    Clusterer1D& operator=(Clusterer1D&&) = delete;
    Clusterer1D() = default;
    ~Clusterer1D() = default;

    void reserve(std::size_t max_pts);

    template<typename T>
    void cluster(const std::vector<T>& values, T max_distance, std::vector<std::size_t>& labels);

  private:
    std::vector<std::size_t> neighbour_indices_;
    containers::CircularQueue<std::size_t> queue_;
};
} // namespace clustering

#endif // DBSCAN_CLUSTERER_1D_HPP
