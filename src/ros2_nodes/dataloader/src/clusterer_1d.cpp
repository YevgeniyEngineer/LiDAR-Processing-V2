#include "clusterer_1d.hpp"

namespace clustering
{
void Clusterer1D::reserve(std::size_t max_pts)
{
    neighbour_indices_.reserve(max_pts);
    queue_.reserve(max_pts);
}

template <typename T>
void Clusterer1D::cluster(const std::vector<T>& values, T max_distance, std::vector<std::size_t>& labels)
{
    labels.assign(values.size(), UNDEFINED);

    if (values.empty())
    {
        return;
    }

    const auto findClosestPoints = [&values, max_distance](std::size_t index,
                                                           std::vector<std::size_t>& neighbour_indices) -> void {
        neighbour_indices.clear();

        const auto target_value = values[index];
        const auto max_index = values.size() - 1;

        for (std::size_t i = index + 1; i < values.size(); ++i)
        {
            const auto diff = values[i] - target_value;

            if (diff < max_distance)
            {
                neighbour_indices.push_back(i);
            }
            else
            {
                break;
            }
        }

        for (std::size_t i = index; i > 0U; --i)
        {
            const auto diff = target_value - values[i - 1];

            if (diff < max_distance)
            {
                neighbour_indices.push_back(i - 1);
            }
            else
            {
                break;
            }
        }
    };

    std::size_t label = 0U;

    for (std::size_t i = 0U; i < values.size(); ++i)
    {
        if (labels[i] == UNDEFINED)
        {
            queue_.push(i);
            while (queue_.size() > 0)
            {
                const auto p = queue_.front();
                queue_.pop();

                if (labels[p] == UNDEFINED)
                {
                    labels[p] = label;

                    findClosestPoints(p, neighbour_indices_);

                    for (const auto& j : neighbour_indices_)
                    {
                        if (labels[j] == UNDEFINED)
                        {
                            queue_.push(j);
                        }
                    }
                }
            }

            ++label;
        }
    }
}

// Explicit specialization
template void Clusterer1D::cluster(const std::vector<float>& values, float max_distance,
                                   std::vector<std::size_t>& labels);

} // namespace clustering
