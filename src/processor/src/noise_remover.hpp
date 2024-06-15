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

// Spatial index
#include "kdtree.hpp"

// STL
#include <cstdint>
#include <vector>

namespace noise_removal
{
enum class NoiseRemoverLabel : std::uint8_t
{
    VALID = 0,
    NOISE = 1
};

struct Configuration
{
    // Search radius will be scaled by this number
    // For example, at 1 meters, if radius_multiplier_m_per_m = 0.02,
    // then search radius will be 0.02 at 1m and 0.2 at 10m
    float radius_multiplier_m_per_m = 0.02F;

    // Overrides dynamic search radius
    float min_search_radius_m = 0.1F;

    // Number of neighbours required within radius search
    // for the point to be classified as valid
    std::uint32_t min_neighbours = 4U;
};

class NoiseRemover final
{
  public:
    using PointT = spatial_index::KDTree<float, 3>::PointT;
    using NeighbourT = spatial_index::KDTree<float, 3>::Neighbour;

    NoiseRemover();

    void filter(const std::vector<PointT>& points, std::vector<NoiseRemoverLabel>& labels);

    void reserve(std::uint32_t max_pts)
    {
        kdtree_.reserve(max_pts);
        neighbours_.reserve(max_pts);
    }

    void config(const Configuration& config)
    {
        config_ = config;
    }

    const Configuration& config() const noexcept
    {
        return config_;
    }

  private:
    Configuration config_;

    spatial_index::KDTree<float, 3> kdtree_;
    std::vector<NeighbourT> neighbours_;
};
} // namespace noise_removal
