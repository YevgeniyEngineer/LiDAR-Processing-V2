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
    std::uint16_t t;
    std::uint16_t c;
    float x;
    float y;
    float z;
    Label label;
};

struct Configuration
{
    float ground_height_threshold_m = 0.2F;
    float ring_spacing_m = 2.0F;
    float road_maximum_slope_m_per_m = 0.15F;
    float min_distance_m = 0.0F;
    float max_distance_m = 100.0F;
    float grid_azimuth_resolution_rad = 0.0087; // 0.5 deg
    float sensor_height_m = 1.73F;
    float kernel_threshold_distance_m = 1.0F;
    float amplification_factor = 5.0F;
};

class Segmenter
{
  public:
    // Constants
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0);
    static constexpr auto TWO_M_PIf = static_cast<float>(2.0 * M_PI);

    // Invalid value constants
    static constexpr float INVALID_Z = std::numeric_limits<float>::max();
    static constexpr std::int32_t INVALID_INDEX = -1;

    // Values acceptable for Velodyne HDL-64E (tightest values)
    static constexpr float VERTICAL_RESOLUTION_DEG = 0.42F; // 0.33F;
    static constexpr float VERTICAL_RESOLUTION_RAD = VERTICAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float VERTICAL_FIELD_OF_VIEW_DEG = 26.9F;
    static constexpr float VERTICAL_FIELD_OF_VIEW_RAD = VERTICAL_RESOLUTION_DEG * DEG_TO_RAD;

    static constexpr float HORIZONTAL_RESOLUTION_DEG = 0.15F; // 0.1F;
    static constexpr float HORIZONTAL_RESOLUTION_RAD = HORIZONTAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_DEG = 360.0F;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_RAD = HORIZONTAL_RESOLUTION_DEG * DEG_TO_RAD;

    static constexpr float MIN_ELEVATION_DEG = -24.8F;
    static constexpr float MIN_ELEVATION_RAD = MIN_ELEVATION_DEG * DEG_TO_RAD;
    static constexpr float MAX_ELEVATION_DEG = MIN_ELEVATION_DEG + VERTICAL_FIELD_OF_VIEW_DEG;
    static constexpr float MAX_ELEVATION_RAD = MAX_ELEVATION_DEG * DEG_TO_RAD;

    inline static const auto DEPTH_IMAGE_WIDTH =
        static_cast<std::int32_t>(std::ceil(HORIZONTAL_FIELD_OF_VIEW_DEG / HORIZONTAL_RESOLUTION_DEG));

    inline static const auto DEPTH_IMAGE_HEIGHT =
        static_cast<std::int32_t>(std::ceil(VERTICAL_FIELD_OF_VIEW_DEG / VERTICAL_RESOLUTION_DEG));

    Segmenter()
        : range_image_ {cv::Mat::zeros(DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, CV_8UC3)},
          grid_mapping_indices_ {cv::Mat(DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, CV_32SC1, cv::Scalar(INVALID_INDEX))},
          kernel_ {cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5))}
    {
        grid_number_of_slices_ = static_cast<std::int32_t>(TWO_M_PIf / config_.grid_azimuth_resolution_rad);
        grid_number_of_bins_ =
            static_cast<std::int32_t>((config_.max_distance_m - config_.min_distance_m) / config_.ring_spacing_m);
        elevation_map_.assign(grid_number_of_slices_ * grid_number_of_bins_, INVALID_Z);
        cloud_indices_.assign(DEPTH_IMAGE_HEIGHT * DEPTH_IMAGE_WIDTH, INVALID_INDEX);
    }

    ~Segmenter() = default;

    void config(const Configuration& config) noexcept
    {
        config_ = config;
        grid_number_of_slices_ = static_cast<std::int32_t>(TWO_M_PIf / config_.grid_azimuth_resolution_rad);
        grid_number_of_bins_ =
            static_cast<std::int32_t>((config_.max_distance_m - config_.min_distance_m) / config_.ring_spacing_m);
        elevation_map_.assign(grid_number_of_slices_ * grid_number_of_bins_, INVALID_Z);
        cloud_indices_.assign(DEPTH_IMAGE_HEIGHT * DEPTH_IMAGE_WIDTH, INVALID_INDEX);
    }

    const Configuration& config() const noexcept
    {
        return config_;
    }

    const cv::Mat& rangeImage() const noexcept
    {
        return range_image_;
    }

    void segment(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels)
    {
        std::cerr << "Entered segment()" << std::endl;

        labels.assign(cloud.points.size(), Label::UNKNOWN);

        reset();

        constructElevationMap(cloud);

        std::cerr << "Constructed elevation map" << std::endl;

        performRECMSegmentation(cloud);

        std::cerr << "Performed RECM segmentation" << std::endl;

        performJCPSegmentation(cloud);

        std::cerr << "Performed JCP segmentation" << std::endl;

        extractSegmentationLabels(cloud, labels);

        std::cerr << "Extracted Segmentation labels" << std::endl;
    }

  private:
    struct Index
    {
        std::int32_t height_index;
        std::int32_t width_index;
    };

    Configuration config_;

    std::int32_t grid_number_of_slices_;
    std::int32_t grid_number_of_bins_;

    std::vector<float> elevation_map_;
    std::vector<std::int32_t> cloud_indices_;

    cv::Mat range_image_;
    cv::Mat grid_mapping_indices_;
    cv::Mat kernel_;
    std::vector<cv::Mat> range_image_channels_; // TODO: Replace with containers::Vector

    std::queue<Index> index_queue_; // TODO: Replaces with containers::Queue

    Eigen::Vector<float, 24> D_;
    Eigen::Vector<float, 24> W_;
    std::array<std::int32_t, 24> mask_;

    // Colour labels
    inline static const auto CV_OBSTACLE = cv::Vec3b(0, 0, 255);
    inline static const auto CV_GROUND = cv::Vec3b(0, 255, 0);
    inline static const auto CV_INTERSECTION_OR_UNKNOWN = cv::Vec3b(0, 255, 255);
    inline static const auto CV_INTERSECTION = cv::Vec3b(255, 0, 0);
    inline static const auto CV_UNKNOWN = cv::Vec3b(255, 255, 255);

    void reset()
    {
        range_image_.setTo(cv::Scalar(0));
        grid_mapping_indices_.setTo(INVALID_INDEX);
    }

    std::int32_t azimuthToIndex(float azimuth_rad) const noexcept
    {
        return static_cast<std::int32_t>(azimuth_rad / config_.grid_azimuth_resolution_rad);
    }

    std::int32_t radiusToIndex(float radius_m) const noexcept
    {
        return static_cast<std::int32_t>(radius_m / config_.ring_spacing_m);
    }

    std::int32_t toFlatGridIndex(std::int32_t azimuth_index, std::int32_t radial_index) const noexcept
    {
        if (azimuth_index < 0 || azimuth_index >= grid_number_of_slices_ || radial_index < 0 ||
            radial_index >= grid_number_of_bins_)
        {
            return -1;
        }

        return azimuth_index * grid_number_of_bins_ + radial_index;
    }

    std::int32_t toFlatDepthImageIndex(std::int32_t height_index, std::int32_t width_index) const noexcept
    {
        if (height_index < 0 || height_index >= DEPTH_IMAGE_HEIGHT || width_index < 0 ||
            width_index >= DEPTH_IMAGE_WIDTH)
        {
            return -1;
        }

        return height_index * DEPTH_IMAGE_WIDTH + width_index;
    }

    void constructElevationMap(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
    {
        std::fill(elevation_map_.begin(), elevation_map_.end(), INVALID_Z);
        std::fill(cloud_indices_.begin(), cloud_indices_.end(), INVALID_INDEX);

        elevation_map_.at(0) = -config_.sensor_height_m;

        for (std::size_t point_index = 0; point_index < cloud.points.size(); ++point_index)
        {
            const auto& point = cloud.points[point_index];

            const auto radius_m = std::sqrt(point.x * point.x + point.y * point.y);

            if (radius_m < config_.min_distance_m || radius_m > config_.max_distance_m)
            {
                continue;
            }

            float azimuth_rad = std::atan2(point.y, point.x);
            azimuth_rad = (azimuth_rad >= 0 ? azimuth_rad : (TWO_M_PIf + azimuth_rad));

            const auto radial_index = radiusToIndex(radius_m);
            const auto azimuth_index = azimuthToIndex(azimuth_rad);

            // Update elevation map
            if (radial_index < grid_number_of_bins_ && azimuth_index < grid_number_of_slices_)
            {
                const std::int32_t height_index = point.ring;

                const auto width_index =
                    static_cast<std::int32_t>(std::round((DEPTH_IMAGE_WIDTH - 1) * azimuth_rad / TWO_M_PIf));

                const auto depth_image_index = toFlatDepthImageIndex(height_index, width_index);

                if (depth_image_index == INVALID_INDEX)
                {
                    continue;
                }

                const auto flat_index = toFlatGridIndex(azimuth_index, radial_index);

                elevation_map_.at(flat_index) = std::min(elevation_map_.at(flat_index), point.z);
                range_image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
                grid_mapping_indices_.at<std::int32_t>(height_index, width_index) = flat_index;
                cloud_indices_.at(static_cast<std::size_t>(depth_image_index)) = point_index;
            }
        }

        // For each slice
        for (std::int32_t azimuth_index = 0; azimuth_index < grid_number_of_slices_; ++azimuth_index)
        {
            // For each ring (radial partition) within the slice
            for (std::int32_t radial_index = 1; radial_index < grid_number_of_bins_; ++radial_index)
            {
                const auto curr_flat_index = toFlatGridIndex(azimuth_index, radial_index);
                const auto curr_z = elevation_map_.at(curr_flat_index);

                if (std::fabs(INVALID_Z - curr_z) < std::numeric_limits<float>::epsilon())
                {
                    // Current elevation cell does not contain a valid elevation value
                    continue;
                }

                const std::int32_t prev_radial_index = radial_index - 1;
                const auto prev_flat_index = toFlatGridIndex(azimuth_index, prev_radial_index);
                const auto prev_z = elevation_map_.at(prev_flat_index);

                if (std::fabs(INVALID_Z - prev_z) < std::numeric_limits<float>::epsilon())
                {
                    // Previous elevation cell does not contain a valid elevation value
                    continue;
                }

                const auto gradient_between_cells = std::atan((curr_z - prev_z) / config_.ring_spacing_m);

                if (gradient_between_cells > config_.road_maximum_slope_m_per_m)
                {
                    elevation_map_.at(curr_flat_index) =
                        prev_z + config_.ring_spacing_m * std::tan(config_.road_maximum_slope_m_per_m);
                }
            }
        }
    }

    void performRECMSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
    {
        for (std::int32_t height_index = 0; height_index < DEPTH_IMAGE_HEIGHT; ++height_index)
        {
            for (std::int32_t width_index = 0; width_index < DEPTH_IMAGE_WIDTH; ++width_index)
            {
                const auto mapping_index = grid_mapping_indices_.at<std::int32_t>(height_index, width_index);

                if (mapping_index == INVALID_INDEX)
                {
                    continue;
                }

                const auto depth_image_index = toFlatDepthImageIndex(height_index, width_index);

                if (depth_image_index == INVALID_INDEX)
                {
                    continue;
                }

                const auto point_index = cloud_indices_.at(depth_image_index);

                if (point_index == INVALID_INDEX)
                {
                    continue;
                }

                const auto& point = cloud.points[point_index];
                const auto grid_index = grid_mapping_indices_.at<std::int32_t>(height_index, width_index);

                if (point.z >= elevation_map_[grid_index] + config_.ground_height_threshold_m)
                {
                    range_image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
                }
            }
        }
    }

    void performJCPSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
    {
        range_image_channels_.clear();

        cv::split(range_image_, range_image_channels_);
        cv::dilate(range_image_channels_[2], range_image_channels_[2], kernel_);
        cv::merge(range_image_channels_, range_image_);

        index_queue_ = {}; // TODO

        for (std::int32_t height_index = 0; height_index < DEPTH_IMAGE_HEIGHT; ++height_index)
        {
            for (std::int32_t width_index = 0; width_index < DEPTH_IMAGE_WIDTH; ++width_index)
            {
                if (range_image_.at<cv::Vec3b>(height_index, width_index) == CV_INTERSECTION_OR_UNKNOWN)
                {
                    const auto depth_image_index = toFlatDepthImageIndex(height_index, width_index);

                    if (depth_image_index == INVALID_INDEX)
                    {
                        continue;
                    }

                    const auto point_index = cloud_indices_.at(depth_image_index);

                    if (point_index != INVALID_INDEX)
                    {
                        index_queue_.push({height_index, width_index});
                        range_image_.at<cv::Vec3b>(height_index, width_index) = CV_INTERSECTION;
                    }
                    else
                    {
                        range_image_.at<cv::Vec3b>(height_index, width_index) = CV_UNKNOWN;
                    }
                }
            }
        }

        mask_.fill(INVALID_INDEX);
        D_.fill(0);
        W_.fill(0);

        static constexpr Index neighbour_offsets[24] = {{-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}, {-1, -2},
                                                        {-1, -1}, {-1, 0},  {-1, 1}, {-1, 2}, {0, -2}, {0, -1},
                                                        {0, 1},   {0, 2},   {1, -2}, {1, -1}, {1, 0},  {1, 1},
                                                        {1, 2},   {2, -2},  {2, -1}, {2, 0},  {2, 1},  {2, 2}};

        while (!index_queue_.empty())
        {
            const auto [height_index, width_index] = index_queue_.front();
            index_queue_.pop();
            const auto depth_image_index = toFlatDepthImageIndex(height_index, width_index);
            float sum_of_coefficients = 0.0F;

            for (std::size_t i = 0; i < 24; ++i)
            {
                const auto [height_offset, width_offset] = neighbour_offsets[i];
                const auto neighbour_height_index = height_index + height_offset;
                const auto neighbour_width_index = width_index + width_offset;
                const auto neighbour_depth_image_index =
                    toFlatDepthImageIndex(neighbour_height_index, neighbour_width_index);

                if (neighbour_depth_image_index == INVALID_INDEX)
                {
                    continue;
                }

                const auto neighbour_point_index = cloud_indices_.at(neighbour_depth_image_index);

                if (neighbour_height_index < 0 || neighbour_height_index >= DEPTH_IMAGE_HEIGHT ||
                    neighbour_width_index < 0 || neighbour_width_index >= DEPTH_IMAGE_WIDTH ||
                    neighbour_point_index == INVALID_INDEX)
                {
                    D_[i] = 0;
                    mask_[i] = INVALID_INDEX;
                    continue;
                }

                const auto point_index = cloud_indices_.at(depth_image_index);

                const auto& point_1 = cloud.points.at(point_index);
                const auto& point_2 = cloud.points.at(neighbour_point_index);

                const auto distance_12 = std::sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) +
                                                   (point_1.y - point_2.y) * (point_1.y - point_2.y) +
                                                   (point_1.z - point_2.z) * (point_1.z - point_2.z));

                if (distance_12 > config_.kernel_threshold_distance_m)
                {
                    D_[i] = 0;
                }
                else
                {
                    D_[i] = std::exp(-config_.amplification_factor * distance_12);
                    sum_of_coefficients += D_[i];
                }

                if (range_image_.at<cv::Vec3b>(neighbour_height_index, neighbour_width_index) == CV_GROUND)
                {
                    mask_[i] = 0;
                }
                else if (range_image_.at<cv::Vec3b>(neighbour_height_index, neighbour_width_index) == CV_OBSTACLE)
                {
                    mask_[i] = 1;
                }
                else
                {
                    mask_[i] = 2;
                }
            }

            if (std::fabs(sum_of_coefficients) > std::numeric_limits<float>::epsilon())
            {
                W_.noalias() = D_ / sum_of_coefficients;

                float weight_obstacle = 0.0F;
                float weight_ground = 0.0F;

                for (std::uint32_t i = 0; i < D_.size(); ++i)
                {
                    if (mask_[i] == 0)
                    {
                        weight_ground += W_[i];
                    }
                    else if (mask_[i] == 1)
                    {
                        weight_obstacle += W_[i];
                    }
                }

                if (weight_ground > weight_obstacle)
                {
                    range_image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
                }
                else
                {
                    range_image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
                }
            }
        }
    }

    void extractSegmentationLabels(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels)
    {
        for (std::int32_t height_index = 0; height_index < DEPTH_IMAGE_HEIGHT; ++height_index)
        {
            for (std::int32_t width_index = 0; width_index < DEPTH_IMAGE_WIDTH; ++width_index)
            {
                const auto depth_image_index = toFlatDepthImageIndex(height_index, width_index);

                if (depth_image_index == INVALID_INDEX)
                {
                    continue;
                }

                const auto point_index = cloud_indices_.at(depth_image_index);

                if (point_index == INVALID_INDEX)
                {
                    continue;
                }

                if (range_image_.at<cv::Vec3b>(height_index, width_index) == CV_GROUND)
                {
                    labels[point_index] = Label::GROUND;
                }
                else if (range_image_.at<cv::Vec3b>(height_index, width_index) == CV_OBSTACLE)
                {
                    labels[point_index] = Label::OBSTACLE;
                }
                else
                {
                    labels[point_index] = Label::UNKNOWN;
                }
            }
        }
    }
};
} // namespace segmentation

#endif // SEGMENTER_HPP
