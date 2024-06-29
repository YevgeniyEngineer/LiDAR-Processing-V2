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

#ifndef LIDAR_PROCESSING_LIB__SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTER_HPP

// Internal
#include "circular_queue.hpp"
#include "point_types.hpp"
#include "queue.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <numeric>
#include <ostream>
#include <queue>
#include <type_traits>
#include <vector>

// Trait to detect if 'ring' is a member of PointT
template <typename PointT>
class HasRing
{
  private:
    template <typename U>
    static auto test(int) -> decltype(std::declval<U>().ring, std::true_type());
    template <typename U>
    static std::false_type test(...);

  public:
    static constexpr bool value = decltype(test<PointT>(0))::value;
};

namespace lidar_processing_lib
{
enum class Label : std::uint32_t
{
    UNKNOWN = 0,
    GROUND,
    OBSTACLE
};

struct SegmenterPoint
{
    float x;
    float y;
    float z;
    Label label;
    std::uint16_t height_index;
    std::uint16_t width_index;
    std::int32_t cloud_index;
};

struct SegmenterConfiguration
{
    // sensor configurations
    // Values acceptable for Velodyne HDL-64E (64 - ring LiDAR)
    float elevation_up_deg = 2.0F;
    float elevation_down_deg = -24.8F;
    std::int32_t image_width = 2048;
    std::int32_t image_height = 64;

    // segmentation algorithm parameters
    bool assume_unorganized_cloud = false;
    float grid_radial_spacing_m = 2.0F;
    float grid_slice_resolution_deg = 1.0F;
    float ground_height_threshold_m = 0.2F;
    float road_maximum_slope_m_per_m = 0.2F;
    float min_distance_m = 2.0F;
    float max_distance_m = 100.0F;
    float sensor_height_m = 1.73F;
    float kernel_threshold_distance_m = 1.0F;
    float amplification_factor = 5.0F;
    float z_min_m = -3.0F;
    float z_max_m = 4.0F;

    // visualize intermediate results with OpenCV?
    bool display_recm_with_low_confidence_points = false;
};

inline std::ostream& operator<<(std::ostream& os, const SegmenterConfiguration& config)
{
    os << "grid_radial_spacing_m: " << config.grid_radial_spacing_m << "\n"
       << "grid_slice_resolution_deg: " << config.grid_slice_resolution_deg << "\n"
       << "ground_height_threshold_m: " << config.ground_height_threshold_m << "\n"
       << "road_maximum_slope_m_per_m: " << config.road_maximum_slope_m_per_m << "\n"
       << "min_distance_m: " << config.min_distance_m << "\n"
       << "max_distance_m: " << config.max_distance_m << "\n"
       << "sensor_height_m: " << config.sensor_height_m << "\n"
       << "kernel_threshold_distance_m: " << config.kernel_threshold_distance_m << "\n"
       << "amplification_factor: " << config.amplification_factor << "\n"
       << "z_min_m: " << config.z_min_m << "\n"
       << "z_max_m: " << config.z_max_m;

    return os;
}

class Segmenter
{
  public:
    // Constants
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0);
    static constexpr auto TWO_M_PIf = static_cast<float>(2.0 * M_PI);

    // Invalid value constants
    static constexpr float INVALID_Z = std::numeric_limits<float>::max();
    static constexpr float INVALID_DEPTH_M = std::numeric_limits<float>::max();
    static constexpr std::int32_t INVALID_INDEX = -1;

    // Colour labels
    inline static const auto CV_OBSTACLE = cv::Vec3b(0, 0, 255);
    inline static const auto CV_GROUND = cv::Vec3b(0, 255, 0);
    inline static const auto CV_INTERSECTION_OR_UNKNOWN = cv::Vec3b(0, 255, 255);
    inline static const auto CV_INTERSECTION = cv::Vec3b(255, 0, 0);
    inline static const auto CV_UNKNOWN = cv::Vec3b(255, 255, 255);

    // For memory allocation
    static constexpr std::uint32_t MAX_CLOUD_SIZE = 200'000U;

    Segmenter();
    ~Segmenter() = default;

    void config(const SegmenterConfiguration& config);

    inline const SegmenterConfiguration& config() const noexcept
    {
        return config_;
    }

    inline const cv::Mat& image() const noexcept
    {
        return image_;
    }

    template <typename PointT>
    void segment(const pcl::PointCloud<PointT>& cloud, std::vector<Label>& labels);

  private:
    struct Index
    {
        std::int32_t height_index;
        std::int32_t width_index;
    };

    SegmenterConfiguration config_;

    float grid_slice_resolution_rad_;
    std::int32_t grid_number_of_radial_rings_;
    std::int32_t grid_number_of_azimuth_slices_;

    // array<vector<Point>, Number of Rings * Number of Slices>
    // Iteration order:
    // slice 1 -> each cell
    // slice 2 -> each cell
    // ...
    // slice N -> each cell
    std::vector<std::vector<SegmenterPoint>> polar_grid_;

    std::vector<float> elevation_map_;
    std::vector<std::int32_t> cloud_mapping_indices_;
    std::vector<float> cell_z_values_;

    cv::Mat image_;
    cv::Mat display_image_;
    std::vector<float> depth_image_;

    struct RansacPoint
    {
        float x;
        float y;
        float z;
    };

    std::vector<RansacPoint> ransac_points_;

    cv::Mat kernel_;
    std::vector<cv::Mat> image_channels_;
    lidar_processing_lib::Queue<Index> index_queue_;

    Eigen::Vector<float, 24> unnormalized_weight_matrix_;
    Eigen::Vector<float, 24> weight_matrix_;
    std::array<Label, 24> kernel_label_mask_;

    void resetValues();

    inline bool isValidZ(const float z) const noexcept
    {
        if (z < 0)
        {
            return true;
        }
        else if (std::fabs(INVALID_Z - z) < std::numeric_limits<float>::epsilon())
        {
            return false;
        }
        return true;
    }

    inline bool isInvalidZ(const float z) const noexcept
    {
        if (z < 0)
        {
            return false;
        }
        else if (std::fabs(INVALID_Z - z) < std::numeric_limits<float>::epsilon())
        {
            return true;
        }
        return false;
    }

    inline bool isValidIndex(const std::int32_t index) const noexcept
    {
        return index != INVALID_INDEX;
    }

    inline bool isInvalidIndex(const std::int32_t index) const noexcept
    {
        return index == INVALID_INDEX;
    }

    template <typename PointT>
    void constructPolarGrid(const pcl::PointCloud<PointT>& cloud);
    void RECM();
    template <typename PointT>
    void JCP(const pcl::PointCloud<PointT>& cloud);
    void correctCloseRangeFalsePositivesRANSAC();
    void populateLabels(std::vector<Label>& labels);
};

extern template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                        std::vector<Label>& labels);
extern template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                        std::vector<Label>& labels);
extern template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZR>& cloud,
                                        std::vector<Label>& labels);
extern template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZIR>& cloud,
                                        std::vector<Label>& labels);

extern template void Segmenter::constructPolarGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud);
extern template void Segmenter::constructPolarGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud);
extern template void Segmenter::constructPolarGrid(const pcl::PointCloud<pcl::PointXYZR>& cloud);
extern template void Segmenter::constructPolarGrid(const pcl::PointCloud<pcl::PointXYZIR>& cloud);

extern template void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZ>& cloud);
extern template void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZI>& cloud);
extern template void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZR>& cloud);
extern template void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZIR>& cloud);

} // namespace lidar_processing_lib

#endif // LIDAR_PROCESSING_LIB__SEGMENTER_HPP
