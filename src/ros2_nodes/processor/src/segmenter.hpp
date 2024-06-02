#ifndef SEGMENTER_HPP
#define SEGMENTER_HPP

// Internal
// #include "containers/queue.hpp"
// #include "containers/vector.hpp"

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <ostream>
#include <queue>
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

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
                                                                                                       ring,
                                                                                                       ring))

namespace segmentation
{
enum class Label : std::uint8_t
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
    std::int32_t height_index;
    std::int32_t width_index;
    std::int32_t cloud_index;
};

struct Configuration
{
    float grid_radial_spacing_m = 2.0F;
    float grid_slice_resolution_deg = 0.2F;
    float ground_height_threshold_m = 0.2F;
    float road_maximum_slope_m_per_m = 0.15F;
    float min_distance_m = 2.0F;
    float max_distance_m = 100.0F;
    float sensor_height_m = 1.73F;
    float kernel_threshold_distance_m = 1.0F;
    float amplification_factor = 5.0F;
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
       << "amplification_factor: " << config.amplification_factor;

    return os;
}

static constexpr std::int32_t ceil(float num) noexcept
{
    return (static_cast<float>(static_cast<std::int32_t>(num)) == num)
               ? static_cast<std::int32_t>(num)
               : static_cast<std::int32_t>(num) + ((num > 0) ? 1 : 0);
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

    // Values acceptable for Velodyne HDL-64E (tightest values)
    static constexpr float VERTICAL_RESOLUTION_DEG = 0.425F;
    static constexpr float VERTICAL_FIELD_OF_VIEW_DEG = 26.9F;

    static constexpr float HORIZONTAL_RESOLUTION_DEG = 0.17F;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_DEG = 360.0F;

    static constexpr float VERTICAL_RESOLUTION_RAD = VERTICAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float HORIZONTAL_RESOLUTION_RAD = HORIZONTAL_RESOLUTION_DEG * DEG_TO_RAD;

    static constexpr auto IMAGE_WIDTH =
        static_cast<std::int32_t>(ceil(HORIZONTAL_FIELD_OF_VIEW_DEG / HORIZONTAL_RESOLUTION_DEG));
    static constexpr auto IMAGE_HEIGHT =
        static_cast<std::int32_t>(ceil(VERTICAL_FIELD_OF_VIEW_DEG / VERTICAL_RESOLUTION_DEG));

    // Colour labels
    inline static const auto CV_OBSTACLE = cv::Vec3b(0, 0, 255);
    inline static const auto CV_GROUND = cv::Vec3b(0, 255, 0);
    inline static const auto CV_INTERSECTION_OR_UNKNOWN = cv::Vec3b(0, 255, 255);
    inline static const auto CV_INTERSECTION = cv::Vec3b(255, 0, 0);
    inline static const auto CV_UNKNOWN = cv::Vec3b(255, 255, 255);

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

    void segment(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels);

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
    std::vector<std::vector<Point>> polar_grid_;

    std::vector<float> elevation_map_;
    std::vector<std::int32_t> cloud_indices_;

    cv::Mat image_;
    cv::Mat depth_image_;
    cv::Mat grid_mapping_indices_;
    cv::Mat cloud_mapping_indices_;
    cv::Mat kernel_;
    std::vector<cv::Mat> image_channels_; // TODO: Replace with containers::Vector

    std::queue<Index> index_queue_; // TODO: Replaces with containers::Queue

    Eigen::Vector<float, 24> D_;
    Eigen::Vector<float, 24> W_;
    std::array<std::int32_t, 24> mask_;

    void resetValues();

    inline std::int32_t azimuthToIndex(float azimuth_rad) const noexcept
    {
        return static_cast<std::int32_t>(azimuth_rad / grid_slice_resolution_rad_);
    }

    inline std::int32_t radiusToIndex(float radius_m) const noexcept
    {
        return static_cast<std::int32_t>((radius_m - config_.min_distance_m) / config_.grid_radial_spacing_m);
    }

    inline std::int32_t toFlatGridIndex(std::int32_t azimuth_index, std::int32_t radial_index) const noexcept
    {
        if (azimuth_index < 0 || azimuth_index >= grid_number_of_azimuth_slices_ || radial_index < 0 ||
            radial_index >= grid_number_of_radial_rings_)
        {
            return -1;
        }

        return azimuth_index * grid_number_of_radial_rings_ + radial_index;
    }

    inline std::int32_t maxGridIndex() const noexcept
    {
        return (grid_number_of_azimuth_slices_ - 1) * grid_number_of_radial_rings_ + (grid_number_of_radial_rings_ - 1);
    }

    inline std::int32_t toFlatImageIndex(std::int32_t height_index, std::int32_t width_index) const noexcept
    {
        if (height_index < 0 || height_index >= IMAGE_HEIGHT || width_index < 0 || width_index >= IMAGE_WIDTH)
        {
            return -1;
        }

        return height_index * IMAGE_WIDTH + width_index;
    }

    inline bool isValidZ(float z) noexcept
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

    inline bool isInvalidZ(float z) noexcept
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

    inline bool isValidIndex(std::int32_t index) noexcept
    {
        return index != INVALID_INDEX;
    }

    inline bool isInvalidIndex(std::int32_t index) noexcept
    {
        return index == INVALID_INDEX;
    }

    inline bool isValidDepth(float depth) noexcept
    {
        if (depth < 0)
        {
            return false;
        }
        else if (std::fabs(INVALID_DEPTH_M - depth) < std::numeric_limits<float>::epsilon())
        {
            return false;
        }
        return true;
    }

    void RECM(const pcl::PointCloud<pcl::PointXYZIR>& cloud);
    void JCP(const pcl::PointCloud<pcl::PointXYZIR>& cloud);
    void populateLabels(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels);

    void constructElevationMap(const pcl::PointCloud<pcl::PointXYZIR>& cloud);

    void performRECMSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& cloud);

    void performJCPSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& cloud);

    void extractSegmentationLabels(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels);
};
} // namespace segmentation

#endif // SEGMENTER_HPP
