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

#ifndef SEGMENTER_HPP
#define SEGMENTER_HPP

// Internal
#include "circular_queue.hpp"

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

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/opencv.hpp>

namespace pcl
{
struct EIGEN_ALIGN16 PointXYZIR
{
    PCL_ADD_POINT4D;                // This adds the XYZ coordinates and padding
    float intensity;                // Intensity of reflection
    std::uint16_t ring;             // Laser ring index
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
};                                  // Force SSE alignment
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

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

namespace segmentation
{
enum class Label : std::uint32_t
{
    UNKNOWN = 0,
    GROUND,
    OBSTACLE
};

struct Point
{
    float x;
    float y;
    float z;
    Label label;
    std::uint16_t height_index;
    std::uint16_t width_index;
    std::int32_t cloud_index;
};

struct Configuration
{
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

    bool display_recm_with_low_confidence_points = false;
};

inline std::ostream& operator<<(std::ostream& os, const Configuration& config)
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

    // Values acceptable for Velodyne HDL-64E (64 - ring LiDAR)
    static constexpr float ELEVATION_UP_DEG = +2.0F;
    static constexpr float ELEVATION_DOWN_DEG = -24.8F;
    static constexpr std::int32_t IMAGE_WIDTH = 2048;
    static constexpr std::int32_t IMAGE_HEIGHT = 64;

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

    void config(const Configuration& config);

    inline const Configuration& config() const noexcept
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

    Configuration config_;

    float grid_slice_resolution_rad_;
    std::int32_t grid_number_of_radial_rings_;
    std::int32_t grid_number_of_azimuth_slices_;

    // array<vector<Point>, Number of Rings * Number of Slices>
    // Iteration order:
    // slice 1 -> each cell
    // slice 2 -> each cell
    // ...
    // slice N -> each cell
    std::vector<std::vector<Point>> polar_grid_;

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
    containers::CircularQueue<Index, IMAGE_HEIGHT * IMAGE_WIDTH> index_queue_;

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
extern template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZIR>& cloud,
                                        std::vector<Label>& labels);

extern template void Segmenter::constructPolarGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud);
extern template void Segmenter::constructPolarGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud);
extern template void Segmenter::constructPolarGrid(const pcl::PointCloud<pcl::PointXYZIR>& cloud);

extern template void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZ>& cloud);
extern template void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZI>& cloud);
extern template void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZIR>& cloud);

} // namespace segmentation

#endif // SEGMENTER_HPP
