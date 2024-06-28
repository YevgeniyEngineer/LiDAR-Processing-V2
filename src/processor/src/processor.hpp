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

#ifndef PROCESSOR_HPP
#define PROCESSOR_HPP

// LiDAR processing libraries
#include <lidar_processing_lib/clusterer.hpp>
#include <lidar_processing_lib/noise_remover.hpp>
#include <lidar_processing_lib/point_types.hpp>
#include <lidar_processing_lib/polygonizer.hpp>
#include <lidar_processing_lib/segmenter.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS2
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STL
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <variant>
#include <vector>

namespace processing
{
struct VehicleDimensions
{
    double min_length;
    double max_length;
    double min_width;
    double max_width;
    double min_height;
    double max_height;
};

struct VehicleBounds
{
    double min_volume;
    double max_volume;
    double min_area;
    double max_area;
};

// Vehicle tolerances
inline constexpr double length_tolerance_m = 0.8;
inline constexpr double width_tolerance_m = 0.5;
inline constexpr double height_tolerance_m = 0.5;

// Typical dimensions for different vehicle types
inline constexpr std::array<VehicleDimensions, 5> base_vehicle_dimensions = {{
    {4.3, 4.6, 1.6, 1.9, 1.4, 1.5}, // Compact Cars
    {4.6, 5.0, 1.6, 1.9, 1.4, 1.5}, // Sedans
    {4.6, 5.2, 1.7, 2.1, 1.7, 1.8}, // SUVs
    {5.2, 5.8, 1.9, 2.2, 1.8, 2.0}, // Trucks
    {4.9, 5.2, 1.7, 2.1, 1.7, 1.8}  // Minivans
}};

// Allowed intersection over union between convex hull and bounding box
[[maybe_unused]] inline constexpr double min_intersection_over_union = 0.4;

inline constexpr std::array<VehicleDimensions, 5> adjusted_vehicle_dimensions = []() {
    std::array<VehicleDimensions, 5> adjusted{};
    for (std::size_t i = 0; i < base_vehicle_dimensions.size(); ++i)
    {
        adjusted[i] = {base_vehicle_dimensions[i].min_length - length_tolerance_m,
                       base_vehicle_dimensions[i].max_length + length_tolerance_m,
                       base_vehicle_dimensions[i].min_width - width_tolerance_m,
                       base_vehicle_dimensions[i].max_width + width_tolerance_m,
                       base_vehicle_dimensions[i].min_height - height_tolerance_m,
                       base_vehicle_dimensions[i].max_height + height_tolerance_m};
    }
    return adjusted;
}();

inline constexpr std::array<VehicleBounds, 5> vehicle_bounds = []() {
    std::array<VehicleBounds, 5> bounds{};
    for (std::size_t i = 0; i < adjusted_vehicle_dimensions.size(); ++i)
    {
        const auto& dims = adjusted_vehicle_dimensions[i];
        bounds[i] = {dims.min_length * dims.min_width * dims.min_height,
                     dims.max_length * dims.max_width * dims.max_height,
                     dims.min_length * dims.min_width,
                     dims.max_length * dims.max_width};
    }
    return bounds;
}();

inline constexpr double min_height_threshold = []() {
    double min_height = std::numeric_limits<double>::max();
    for (const auto& vehicle : adjusted_vehicle_dimensions)
    {
        if (vehicle.min_height < min_height)
        {
            min_height = vehicle.min_height;
        }
    }
    return min_height;
}();

inline constexpr double max_height_threshold = []() {
    double max_height = std::numeric_limits<double>::lowest();
    for (const auto& vehicle : adjusted_vehicle_dimensions)
    {
        if (vehicle.max_height > max_height)
        {
            max_height = vehicle.max_height;
        }
    }
    return max_height;
}();

inline constexpr double absolute_min_volume = []() {
    double min_volume = std::numeric_limits<double>::max();
    for (const auto& bounds : vehicle_bounds)
    {
        if (bounds.min_volume < min_volume)
        {
            min_volume = bounds.min_volume;
        }
    }
    return min_volume;
}();

inline constexpr double absolute_max_volume = []() {
    double max_volume = std::numeric_limits<double>::lowest();
    for (const auto& bounds : vehicle_bounds)
    {
        if (bounds.max_volume > max_volume)
        {
            max_volume = bounds.max_volume;
        }
    }
    return max_volume;
}();

[[maybe_unused]] inline constexpr double absolute_min_area = []() {
    double min_area = std::numeric_limits<double>::max();
    for (const auto& bounds : vehicle_bounds)
    {
        if (bounds.min_area < min_area)
        {
            min_area = bounds.min_area;
        }
    }
    return min_area;
}();

[[maybe_unused]] inline constexpr double absolute_max_area = []() {
    double max_area = std::numeric_limits<double>::lowest();
    for (const auto& bounds : vehicle_bounds)
    {
        if (bounds.max_area > max_area)
        {
            max_area = bounds.max_area;
        }
    }
    return max_area;
}();

class Processor final : public rclcpp::Node
{
  public:
    static constexpr const char* NODE_NAME = "processor";
    static constexpr std::int64_t WAIT_SET_TIME_MS = 150;
    static constexpr std::size_t MAX_PTS = 200'000U;

    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Image = sensor_msgs::msg::Image;
    using PointCloudVariant = std::variant<pcl::PointCloud<pcl::PointXYZ>,
                                           pcl::PointCloud<pcl::PointXYZI>,
                                           pcl::PointCloud<pcl::PointXYZR>,
                                           pcl::PointCloud<pcl::PointXYZIR>>;

    Processor();

    void run(const PointCloud2& msg);

    const rclcpp::Subscription<PointCloud2>::SharedPtr& inputCloudSubscriber() const noexcept
    {
        return input_cloud_subscriber_;
    }

  private:
    std::string input_cloud_topic_;
    std::string ground_cloud_topic_;
    std::string obstacle_cloud_topic_;
    std::string unsegmented_cloud_topic_;
    std::string segmented_image_topic_;
    std::string clustered_cloud_topic_;
    std::string obstacle_outlines_topic_;

    cv::Mat image_cache_;
    Image image_msg_cache_;
    PointCloud2 cloud_msg_cache_;
    MarkerArray polygon_msg_cache_;

    rclcpp::Subscription<PointCloud2>::SharedPtr input_cloud_subscriber_;
    rclcpp::Publisher<PointCloud2>::SharedPtr ground_cloud_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_cloud_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr unsegmented_cloud_publisher_;
    rclcpp::Publisher<Image>::SharedPtr segmented_image_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr clustered_cloud_publisher_;
    rclcpp::Publisher<MarkerArray>::SharedPtr obstacle_outlines_publisher_;

    PointCloudVariant input_cloud_variant_;

    lidar_processing_lib::NoiseRemover noise_remover_;
    std::vector<lidar_processing_lib::NoiseRemover::PointT> noise_remover_points_;
    std::vector<lidar_processing_lib::NoiseRemoverLabel> noise_remover_labels_;
    pcl::PointCloud<pcl::PointXYZIR> denoised_input_cloud_;

    lidar_processing_lib::Segmenter segmenter_;
    std::vector<lidar_processing_lib::Label> segmentation_labels_;
    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB> unsegmented_cloud_;

    lidar_processing_lib::Clusterer clusterer_;
    std::vector<lidar_processing_lib::ClusterLabel> clustering_labels_;
    pcl::PointCloud<pcl::PointXYZRGB> clustered_cloud_;

    lidar_processing_lib::Polygonizer polygonizer_;

    std::vector<std::int32_t> polygonizer_indices_;
    std::vector<lidar_processing_lib::PointXY> polygonizer_points_;
    std::vector<lidar_processing_lib::PointXY> polygon_points_;
};

} // namespace processing

#endif // PROCESSOR_HPP
