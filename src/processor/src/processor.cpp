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

// Internal
#include "processor.hpp"

// Libraries
#include <lidar_processing_lib/stack_vector.hpp>

// STL
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <tuple>

// ROS2
#include <rclcpp/qos.hpp>

namespace processing
{
template <typename PointT>
void convert(const sensor_msgs::msg::PointCloud2& msg, pcl::PointCloud<PointT>& cloud);

// Specialization for pcl::PointXYZ
template <>
void convert<pcl::PointXYZ>(const sensor_msgs::msg::PointCloud2& msg,
                            pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud.clear();
    cloud.header.frame_id = msg.header.frame_id;
    cloud.width = msg.width;
    cloud.height = msg.height;
    cloud.header.stamp = static_cast<std::uint64_t>(
        std::round(msg.header.stamp.sec * 1.0e9 + msg.header.stamp.nanosec) / 1.0e3);
    cloud.is_dense = msg.is_dense;
    cloud.points.resize(msg.height * msg.width);

    const auto point_step = msg.point_step;
    const auto row_step = msg.row_step;

    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        const auto base_row_offset = row * row_step;
        for (std::uint32_t col = 0; col < msg.width; ++col)
        {
            auto& point = cloud.points[row * msg.width + col];
            const auto offset = base_row_offset + col * point_step;

            point.x = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[0].offset]);
            point.y = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[1].offset]);
            point.z = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[2].offset]);
        }
    }
}

// Specialization for pcl::PointXYZI
template <>
void convert<pcl::PointXYZI>(const sensor_msgs::msg::PointCloud2& msg,
                             pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    cloud.clear();
    cloud.header.frame_id = msg.header.frame_id;
    cloud.width = msg.width;
    cloud.height = msg.height;
    cloud.header.stamp = static_cast<std::uint64_t>(
        std::round(msg.header.stamp.sec * 1.0e9 + msg.header.stamp.nanosec) / 1.0e3);
    cloud.is_dense = msg.is_dense;
    cloud.points.resize(msg.height * msg.width);

    const auto point_step = msg.point_step;
    const auto row_step = msg.row_step;

    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        const auto base_row_offset = row * row_step;
        for (std::uint32_t col = 0; col < msg.width; ++col)
        {
            auto& point = cloud.points[row * msg.width + col];
            const auto offset = base_row_offset + col * point_step;

            point.x = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[0].offset]);
            point.y = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[1].offset]);
            point.z = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[2].offset]);
            point.intensity =
                *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[3].offset]);
        }
    }
}

// Specialization for pcl::PointXYZR
template <>
void convert<pcl::PointXYZR>(const sensor_msgs::msg::PointCloud2& msg,
                             pcl::PointCloud<pcl::PointXYZR>& cloud)
{
    cloud.clear();
    cloud.header.frame_id = msg.header.frame_id;
    cloud.width = msg.width;
    cloud.height = msg.height;
    cloud.header.stamp = static_cast<std::uint64_t>(
        std::round(msg.header.stamp.sec * 1.0e9 + msg.header.stamp.nanosec) / 1.0e3);
    cloud.is_dense = msg.is_dense;
    cloud.points.resize(msg.height * msg.width);

    const auto point_step = msg.point_step;
    const auto row_step = msg.row_step;

    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        const auto base_row_offset = row * row_step;
        for (std::uint32_t col = 0; col < msg.width; ++col)
        {
            auto& point = cloud.points[row * msg.width + col];
            const auto offset = base_row_offset + col * point_step;

            point.x = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[0].offset]);
            point.y = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[1].offset]);
            point.z = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[2].offset]);
            point.ring =
                *reinterpret_cast<const std::uint16_t*>(&msg.data[offset + msg.fields[3].offset]);
        }
    }
}

// Specialization for pcl::PointXYZIR
template <>
void convert<pcl::PointXYZIR>(const sensor_msgs::msg::PointCloud2& msg,
                              pcl::PointCloud<pcl::PointXYZIR>& cloud)
{
    cloud.clear();
    cloud.header.frame_id = msg.header.frame_id;
    cloud.width = msg.width;
    cloud.height = msg.height;
    cloud.header.stamp = static_cast<std::uint64_t>(
        std::round(msg.header.stamp.sec * 1.0e9 + msg.header.stamp.nanosec) / 1.0e3);
    cloud.is_dense = msg.is_dense;
    cloud.points.resize(msg.height * msg.width);

    const auto point_step = msg.point_step;
    const auto row_step = msg.row_step;

    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        const auto base_row_offset = row * row_step;
        for (std::uint32_t col = 0; col < msg.width; ++col)
        {
            auto& point = cloud.points[row * msg.width + col];
            const auto offset = base_row_offset + col * point_step;

            point.x = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[0].offset]);
            point.y = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[1].offset]);
            point.z = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[2].offset]);
            point.intensity =
                *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[3].offset]);
            point.ring =
                *reinterpret_cast<const std::uint16_t*>(&msg.data[offset + msg.fields[4].offset]);
        }
    }
}

static void convertImageToRosMessage(const cv::Mat& cv_image,
                                     std::int32_t seconds,
                                     std::uint32_t nanoseconds,
                                     const std::string& encoding,
                                     const std::string& frame_id,
                                     sensor_msgs::msg::Image& msg)
{
    // Fill message metadata
    msg.header.stamp.sec = seconds;
    msg.header.stamp.nanosec = nanoseconds;
    msg.header.frame_id = frame_id;

    // Set image dimensions and encoding
    msg.height = cv_image.rows;
    msg.width = cv_image.cols;
    msg.encoding = encoding;
    msg.is_bigendian = 0;
    msg.step = cv_image.cols * cv_image.elemSize();

    // Copy image data
    const std::size_t size = msg.step * cv_image.rows;
    msg.data.resize(size);
    std::memcpy(static_cast<void*>(&msg.data[0]), static_cast<const void*>(cv_image.data), size);
}

static void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_pcl,
                                    sensor_msgs::msg::PointCloud2& cloud_ros)
{
    cloud_ros.is_dense = cloud_pcl.is_dense;
    cloud_ros.height = cloud_pcl.height;
    cloud_ros.width = cloud_pcl.width;
    cloud_ros.point_step = sizeof(pcl::PointXYZRGB);

    static const std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>>
        fields = {{"x", offsetof(pcl::PointXYZRGB, x), Processor::PointFieldTypes::FLOAT32, 1},
                  {"y", offsetof(pcl::PointXYZRGB, y), Processor::PointFieldTypes::FLOAT32, 1},
                  {"z", offsetof(pcl::PointXYZRGB, z), Processor::PointFieldTypes::FLOAT32, 1},
                  {"rgb", offsetof(pcl::PointXYZRGB, rgb), Processor::PointFieldTypes::FLOAT32, 1}};

    for (const auto& field : fields)
    {
        sensor_msgs::msg::PointField field_cache;
        field_cache.name = std::get<0>(field);
        field_cache.offset = std::get<1>(field);
        field_cache.datatype = std::get<2>(field);
        field_cache.count = std::get<3>(field);
        cloud_ros.fields.push_back(std::move(field_cache));
    }

    cloud_ros.data.clear();
    if (!cloud_pcl.empty())
    {
        const std::size_t byte_size = sizeof(pcl::PointXYZRGB) * cloud_pcl.size();
        cloud_ros.data.resize(byte_size);
        std::memcpy(static_cast<void*>(cloud_ros.data.data()),
                    static_cast<const void*>(&cloud_pcl.at(0)),
                    byte_size);
    }
};

static void convertPolygonPointsToMarker(bool is_bounding_box,
                                         std::uint32_t polygon_index,
                                         double z_min,
                                         double z_max,
                                         const std::vector<lidar_processing_lib::PointXY>& points,
                                         const std::string& frame_id,
                                         const builtin_interfaces::msg::Time& stamp,
                                         visualization_msgs::msg::Marker& marker)
{
    // Clean cache
    marker.points.clear();

    // Check if a valid polygon
    if (points.size() < 3)
    {
        return;
    }

    // Set metadata
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 150'000'000;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "boundary";
    marker.id = polygon_index;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1f;
    marker.color.a = 1.0f;

    if (is_bounding_box)
    {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    }
    else
    {
        marker.color.r = 0.6f;
        marker.color.g = 0.6f;
        marker.color.b = 0.6f;
    }

    // Preallocate memory
    const std::size_t num_horizontal_lines = 2 * points.size();
    const std::size_t num_vertical_lines = points.size();
    const std::size_t num_pts_3d_line_list = 2 * num_horizontal_lines + 2 * num_vertical_lines;
    marker.points.reserve(num_pts_3d_line_list);

    // Lambda function to help generating flat boundaries
    const auto generateHorizontalBoundary = [&marker, &points](double z) -> void {
        for (std::size_t i = 1; i < points.size(); ++i)
        {
            const auto& prev_point = points[i - 1];
            const auto& curr_point = points[i];
            geometry_msgs::msg::Point point_cache;
            point_cache.z = z;
            point_cache.x = prev_point.x;
            point_cache.y = prev_point.y;
            marker.points.push_back(point_cache);
            point_cache.x = curr_point.x;
            point_cache.y = curr_point.y;
            marker.points.push_back(point_cache);
        }
        const auto& prev_point = points.back();
        const auto& curr_point = points.front();
        geometry_msgs::msg::Point point_cache;
        point_cache.z = z;
        point_cache.x = prev_point.x;
        point_cache.y = prev_point.y;
        marker.points.push_back(point_cache);
        point_cache.x = curr_point.x;
        point_cache.y = curr_point.y;
        marker.points.push_back(point_cache);
    };

    // Bottom boundary
    generateHorizontalBoundary(z_min);

    // Top boundary
    generateHorizontalBoundary(z_max);

    // Vertical lines connecting bottom and top boundaries
    for (const auto& point : points)
    {
        geometry_msgs::msg::Point point_cache;
        point_cache.z = z_min;
        point_cache.x = point.x;
        point_cache.y = point.y;
        marker.points.push_back(point_cache);
        point_cache.z = z_max;
        point_cache.x = point.x;
        point_cache.y = point.y;
        marker.points.push_back(point_cache);
    }
}

Processor::Processor()
    : rclcpp::Node{NODE_NAME, rclcpp::NodeOptions().use_intra_process_comms(true)}
{
    using std::placeholders::_1;

    this->declare_parameter<std::string>("input_cloud_topic");
    this->declare_parameter<std::string>("ground_cloud_topic");
    this->declare_parameter<std::string>("obstacle_cloud_topic");
    this->declare_parameter<std::string>("unsegmented_cloud_topic");
    this->declare_parameter<std::string>("segmented_image_topic");
    this->declare_parameter<std::string>("clustered_cloud_topic");
    this->declare_parameter<std::string>("obstacle_outlines_topic");

    this->declare_parameter<double>("segmentation_configuration.elevation_up_deg");
    this->declare_parameter<double>("segmentation_configuration.elevation_down_deg");
    this->declare_parameter<std::int32_t>("segmentation_configuration.image_width");
    this->declare_parameter<std::int32_t>("segmentation_configuration.image_height");

    this->declare_parameter<bool>("segmentation_configuration.assume_unorganized_cloud");
    this->declare_parameter<double>("segmentation_configuration.grid_radial_spacing_m");
    this->declare_parameter<double>("segmentation_configuration.grid_slice_resolution_deg");
    this->declare_parameter<double>("segmentation_configuration.ground_height_threshold_m");
    this->declare_parameter<double>("segmentation_configuration.road_maximum_slope_m_per_m");
    this->declare_parameter<double>("segmentation_configuration.min_distance_m");
    this->declare_parameter<double>("segmentation_configuration.max_distance_m");
    this->declare_parameter<double>("segmentation_configuration.sensor_height_m");
    this->declare_parameter<double>("segmentation_configuration.kernel_threshold_distance_m");
    this->declare_parameter<double>("segmentation_configuration.amplification_factor");
    this->declare_parameter<double>("segmentation_configuration.z_min_m");
    this->declare_parameter<double>("segmentation_configuration.z_max_m");

    this->declare_parameter<bool>(
        "segmentation_configuration.display_recm_with_low_confidence_points");

    this->declare_parameter<double>("clustering_configuration.voxel_grid_range_resolution_m");
    this->declare_parameter<double>("clustering_configuration.voxel_grid_azimuth_resolution_deg");
    this->declare_parameter<double>("clustering_configuration.voxel_grid_elevation_resolution_deg");
    this->declare_parameter<std::int32_t>("clustering_configuration.min_cluster_size");

    input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
    ground_cloud_topic_ = this->get_parameter("ground_cloud_topic").as_string();
    obstacle_cloud_topic_ = this->get_parameter("obstacle_cloud_topic").as_string();
    unsegmented_cloud_topic_ = this->get_parameter("unsegmented_cloud_topic").as_string();
    segmented_image_topic_ = this->get_parameter("segmented_image_topic").as_string();
    clustered_cloud_topic_ = this->get_parameter("clustered_cloud_topic").as_string();
    obstacle_outlines_topic_ = this->get_parameter("obstacle_outlines_topic").as_string();

    lidar_processing_lib::SegmenterConfiguration segmentation_configuration;

    segmentation_configuration.elevation_up_deg =
        this->get_parameter("segmentation_configuration.elevation_up_deg").as_double();
    segmentation_configuration.elevation_down_deg =
        this->get_parameter("segmentation_configuration.elevation_down_deg").as_double();
    segmentation_configuration.image_width =
        this->get_parameter("segmentation_configuration.image_width").as_int();
    segmentation_configuration.image_height =
        this->get_parameter("segmentation_configuration.image_height").as_int();

    segmentation_configuration.assume_unorganized_cloud =
        this->get_parameter("segmentation_configuration.assume_unorganized_cloud").as_bool();
    segmentation_configuration.grid_radial_spacing_m =
        this->get_parameter("segmentation_configuration.grid_radial_spacing_m").as_double();
    segmentation_configuration.grid_slice_resolution_deg =
        this->get_parameter("segmentation_configuration.grid_slice_resolution_deg").as_double();
    segmentation_configuration.ground_height_threshold_m =
        this->get_parameter("segmentation_configuration.ground_height_threshold_m").as_double();
    segmentation_configuration.road_maximum_slope_m_per_m =
        this->get_parameter("segmentation_configuration.road_maximum_slope_m_per_m").as_double();
    segmentation_configuration.min_distance_m =
        this->get_parameter("segmentation_configuration.min_distance_m").as_double();
    segmentation_configuration.max_distance_m =
        this->get_parameter("segmentation_configuration.max_distance_m").as_double();
    segmentation_configuration.sensor_height_m =
        this->get_parameter("segmentation_configuration.sensor_height_m").as_double();
    segmentation_configuration.kernel_threshold_distance_m =
        this->get_parameter("segmentation_configuration.kernel_threshold_distance_m").as_double();
    segmentation_configuration.amplification_factor =
        this->get_parameter("segmentation_configuration.amplification_factor").as_double();
    segmentation_configuration.z_min_m =
        this->get_parameter("segmentation_configuration.z_min_m").as_double();
    segmentation_configuration.z_max_m =
        this->get_parameter("segmentation_configuration.z_max_m").as_double();

    segmentation_configuration.display_recm_with_low_confidence_points =
        this->get_parameter("segmentation_configuration.display_recm_with_low_confidence_points")
            .as_bool();

    segmenter_.config(segmentation_configuration);

    lidar_processing_lib::ClustererConfiguration clustering_configuration;

    clustering_configuration.voxel_grid_range_resolution_m =
        this->get_parameter("clustering_configuration.voxel_grid_range_resolution_m").as_double();
    clustering_configuration.voxel_grid_azimuth_resolution_deg =
        this->get_parameter("clustering_configuration.voxel_grid_azimuth_resolution_deg")
            .as_double();
    clustering_configuration.voxel_grid_elevation_resolution_deg =
        this->get_parameter("clustering_configuration.voxel_grid_elevation_resolution_deg")
            .as_double();
    clustering_configuration.min_cluster_size =
        this->get_parameter("clustering_configuration.min_cluster_size").as_int();

    clusterer_.config(clustering_configuration);

    input_cloud_subscriber_ =
        this->create_subscription<PointCloud2>(input_cloud_topic_,
                                               rclcpp::QoS(2).reliable().durability_volatile(),
                                               []([[maybe_unused]] const PointCloud2& msg) -> void {
                                                   // Callback does nothing
                                               });

    ground_cloud_publisher_ = this->create_publisher<PointCloud2>(
        ground_cloud_topic_, rclcpp::QoS(2).reliable().durability_volatile());
    obstacle_cloud_publisher_ = this->create_publisher<PointCloud2>(
        obstacle_cloud_topic_, rclcpp::QoS(2).reliable().durability_volatile());
    unsegmented_cloud_publisher_ = this->create_publisher<PointCloud2>(
        unsegmented_cloud_topic_, rclcpp::QoS(2).reliable().durability_volatile());
    segmented_image_publisher_ = this->create_publisher<Image>(
        segmented_image_topic_, rclcpp::QoS(2).reliable().durability_volatile());
    clustered_cloud_publisher_ = this->create_publisher<PointCloud2>(
        clustered_cloud_topic_, rclcpp::QoS(2).reliable().durability_volatile());
    obstacle_outlines_publisher_ = this->create_publisher<MarkerArray>(
        obstacle_outlines_topic_, rclcpp::QoS(2).reliable().durability_volatile());

    // Reserve memory
    noise_remover_.reserve(MAX_PTS);
    noise_remover_points_.reserve(MAX_PTS);
    noise_remover_labels_.reserve(MAX_PTS);
    denoised_input_cloud_.reserve(MAX_PTS);

    ground_cloud_.points.reserve(MAX_PTS);
    obstacle_cloud_.points.reserve(MAX_PTS);
    unsegmented_cloud_.points.reserve(MAX_PTS);
    image_cache_ = cv::Mat::zeros(
        segmentation_configuration.image_height, segmentation_configuration.image_width, CV_8UC3);
    image_msg_cache_.data.reserve(3 * segmentation_configuration.image_height *
                                  segmentation_configuration.image_width);
    cloud_msg_cache_.data.reserve(MAX_PTS);

    polygonizer_indices_.reserve(MAX_PTS);
    polygonizer_points_.reserve(MAX_PTS);
    polygon_msg_cache_.markers.reserve(1000); // Max expected polygons
    polygon_points_.reserve(100);             // Max points per polygon

    RCLCPP_INFO(this->get_logger(), "%s node constructed", this->get_name());
}

void Processor::run(const PointCloud2& msg)
{
    lidar_processing_lib::StackVector<std::string, 20> field_names;

    for (const auto& field : msg.fields)
    {
        field_names.push_back(field.name);
    }

    if (field_names == std::initializer_list<std::string>{"x", "y", "z"})
    {
        using CloudType = pcl::PointCloud<pcl::PointXYZ>;
        if (!std::holds_alternative<CloudType>(input_cloud_variant_))
        {
            input_cloud_variant_.emplace<CloudType>();
        }
        auto& cloud = std::get<CloudType>(input_cloud_variant_);
        cloud.reserve(MAX_PTS);
        convert(msg, cloud);
    }
    else if (field_names == std::initializer_list<std::string>{"x", "y", "z", "intensity"})
    {
        using CloudType = pcl::PointCloud<pcl::PointXYZI>;
        if (!std::holds_alternative<CloudType>(input_cloud_variant_))
        {
            input_cloud_variant_.emplace<CloudType>();
        }
        auto& cloud = std::get<CloudType>(input_cloud_variant_);
        cloud.reserve(MAX_PTS);
        convert(msg, cloud);
    }
    else if (field_names == std::initializer_list<std::string>{"x", "y", "z", "ring"})
    {
        using CloudType = pcl::PointCloud<pcl::PointXYZR>;
        if (!std::holds_alternative<CloudType>(input_cloud_variant_))
        {
            input_cloud_variant_.emplace<CloudType>();
        }
        auto& cloud = std::get<CloudType>(input_cloud_variant_);
        cloud.reserve(MAX_PTS);
        convert(msg, cloud);
    }
    else if (field_names == std::initializer_list<std::string>{"x", "y", "z", "intensity", "ring"})
    {
        using CloudType = pcl::PointCloud<pcl::PointXYZIR>;
        if (!std::holds_alternative<CloudType>(input_cloud_variant_))
        {
            input_cloud_variant_.emplace<CloudType>();
        }
        auto& cloud = std::get<CloudType>(input_cloud_variant_);
        cloud.reserve(MAX_PTS);
        convert(msg, cloud);
    }
    else
    {
        throw std::runtime_error{"Unsupported point format. Acceptable formats: pcl::PointXYZ, "
                                 "pcl::PointXYZI, pcl::PointXYZR, pcl::PointXYZIR"};
    }

    // Ground segmentation
    std::visit(
        [this](const auto& cloud) -> void {
            const auto t_segmentation_start = std::chrono::steady_clock::now();

            segmenter_.segment(cloud, segmentation_labels_);

            ground_cloud_.clear();
            obstacle_cloud_.clear();
            unsegmented_cloud_.clear();

            for (std::uint32_t i = 0; i < cloud.size(); ++i)
            {
                const auto& p = cloud[i];
                const auto label = segmentation_labels_[i];

                if (label == lidar_processing_lib::Label::GROUND)
                {
                    ground_cloud_.emplace_back(p.x, p.y, p.z, 124, 252, 0);
                }
                else if (label == lidar_processing_lib::Label::OBSTACLE)
                {
                    obstacle_cloud_.emplace_back(p.x, p.y, p.z, 200, 0, 0);
                }
                else
                {
                    unsegmented_cloud_.emplace_back(p.x, p.y, p.z, 255, 255, 0);
                }
            }

            const auto t_segmentation_stop = std::chrono::steady_clock::now();
            const auto t_segmentation_elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(t_segmentation_stop -
                                                                      t_segmentation_start)
                    .count();

            RCLCPP_INFO(this->get_logger(), "Segmentation time [ms]: %ld", t_segmentation_elapsed);
            RCLCPP_INFO(this->get_logger(),
                        "Extracted %lu ground and %lu obstacle points from %lu-point cloud",
                        ground_cloud_.size(),
                        obstacle_cloud_.size(),
                        cloud.size());
        },
        input_cloud_variant_);

    // Obstacle clustering
    const auto t_clustering_start = std::chrono::steady_clock::now();

    clusterer_.cluster(obstacle_cloud_, clustering_labels_);

    const auto t_clustering_stop = std::chrono::steady_clock::now();
    const auto t_clustering_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          t_clustering_stop - t_clustering_start)
                                          .count();

    RCLCPP_INFO(this->get_logger(), "Clustering time [ms]: %ld", t_clustering_elapsed);

    // Populate output cloud with labels
    clustered_cloud_.clear();
    clustered_cloud_.reserve(clustering_labels_.size());

    const auto max_label_it =
        std::max_element(clustering_labels_.cbegin(), clustering_labels_.cend());

    if (max_label_it != clustering_labels_.cend() &&
        *max_label_it != lidar_processing_lib::Clusterer::INVALID_LABEL)
    {
        const auto max_label = *max_label_it;
        RCLCPP_INFO(this->get_logger(), "Extracted %d clusters", max_label);

        const auto number_of_points = clustering_labels_.size();

        polygon_msg_cache_.markers.clear();

        std::chrono::steady_clock::duration t_polygonization_total{0};

        for (std::int32_t label = 0; label <= max_label; ++label)
        {
            // Generate random RGB values for the current cluster
            const auto r = static_cast<std::uint8_t>(std::rand() % 256);
            const auto g = static_cast<std::uint8_t>(std::rand() % 256);
            const auto b = static_cast<std::uint8_t>(std::rand() % 256);

            // Polygonize each cluster simultaneously
            polygonizer_points_.clear();

            auto z_min = std::numeric_limits<double>::max();
            auto z_max = std::numeric_limits<double>::lowest();

            for (std::uint32_t point_index = 0; point_index < number_of_points; ++point_index)
            {
                if (clustering_labels_[point_index] == label)
                {
                    const auto& point = obstacle_cloud_[point_index];

                    clustered_cloud_.emplace_back(point.x, point.y, point.z, r, g, b);
                    polygonizer_points_.push_back({point.x, point.y});

                    if (point.z < z_min)
                    {
                        z_min = point.z;
                    }
                    if (point.z > z_max)
                    {
                        z_max = point.z;
                    }
                }
            }

            const auto t_polygonization_start = std::chrono::steady_clock::now();

            // Polygonize current cluster
            polygonizer_.convexHull(polygonizer_points_, polygonizer_indices_);

            // Transfer to a simplified polygon
            polygon_points_.clear();
            for (const auto& index : polygonizer_indices_)
            {
                polygon_points_.push_back(polygonizer_points_[index]);
            }

            std::chrono::steady_clock::duration bounding_box_total_time{0};
            [[maybe_unused]] const std::size_t number_of_vertices = polygon_points_.size();

            // Check if polygon can futher be simplified to a bounding box
            bool perform_polygon_simplification = false;
            bool is_bounding_box = false;
            const auto bounding_box_height = z_max - z_min;

            if (perform_polygon_simplification)
            {
                // Should have a valid hull, cluster should not be small, height should be
                // reasonable
                if (polygon_points_.size() >= 3 && polygonizer_points_.size() > 150 &&
                    bounding_box_height > min_height_threshold &&
                    bounding_box_height < max_height_threshold)
                {
                    const auto polygon_area = lidar_processing_lib::polygonArea(polygon_points_);
                    const auto polygon_volume = polygon_area * bounding_box_height;

                    // Only select reasonably large polygons for simplification
                    // if (polygon_area > absolute_min_area && polygon_area < absolute_max_area &&
                    //     polygon_volume > absolute_min_volume && polygon_volume <
                    //     absolute_max_volume)
                    if (polygon_volume > absolute_min_volume &&
                        polygon_volume < absolute_max_volume)
                    {
                        const auto t_bounding_box_start = std::chrono::steady_clock::now();

                        // const lidar_processing_lib::BoundingBox bounding_box =
                        //     polygonizer_.boundingBoxPrincipalComponentAnalysis(polygonizer_points_);

                        const lidar_processing_lib::BoundingBox bounding_box =
                            polygonizer_.boundingBoxRotatingCalipers(polygonizer_points_);

                        const auto t_bounding_box_stop = std::chrono::steady_clock::now();
                        bounding_box_total_time += t_bounding_box_stop - t_bounding_box_start;

                        // If generated bounding box is valid
                        if (bounding_box.is_valid)
                        {
                            // Check intersection over union,
                            // to decide how well bounding box fits the shape
                            [[maybe_unused]] const double intersection_over_union =
                                polygon_area / bounding_box.area;

                            if (intersection_over_union > min_intersection_over_union)
                            {
                                // Check width and length of the bounding box
                                const auto edge_1_length = lidar_processing_lib::distance(
                                    bounding_box.corners[0], bounding_box.corners[1]);
                                const auto edge_2_length = lidar_processing_lib::distance(
                                    bounding_box.corners[1], bounding_box.corners[2]);

                                const auto bounding_box_length =
                                    std::max(edge_1_length, edge_2_length);
                                const auto bounding_box_width =
                                    std::min(edge_1_length, edge_2_length);

                                for (std::size_t i = 0; i < adjusted_vehicle_dimensions.size(); ++i)
                                {
                                    const auto& vehicle = adjusted_vehicle_dimensions[i];
                                    const auto& bounds = vehicle_bounds[i];

                                    if ( // Check dimensions
                                        bounding_box_length > vehicle.min_length &&
                                        bounding_box_length < vehicle.max_length &&
                                        bounding_box_width > vehicle.min_width &&
                                        bounding_box_width < vehicle.max_width &&
                                        bounding_box_height > vehicle.min_height &&
                                        bounding_box_height < vehicle.max_height &&
                                        // Check volume and area
                                        polygon_volume > bounds.min_volume &&
                                        polygon_volume < bounds.max_volume &&
                                        polygon_area > bounds.min_area &&
                                        polygon_area < bounds.max_area)
                                    {
                                        // Simplify original polygon points
                                        polygon_points_.assign(bounding_box.corners.cbegin(),
                                                               bounding_box.corners.cend());
                                        is_bounding_box = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                [[maybe_unused]] const auto bounding_box_total_time_microsec =
                    std::chrono::duration_cast<std::chrono::microseconds>(bounding_box_total_time)
                        .count();

                // if (bounding_box_total_time_microsec > 10)
                // {
                //     std::cerr << "Total time bounding box calculation [micros]: "
                //               << bounding_box_total_time_microsec << " for " <<
                //               number_of_vertices
                //               << " vertices" << std::endl;
                // }
            }

            const auto t_polygonization_stop = std::chrono::steady_clock::now();
            t_polygonization_total += (t_polygonization_stop - t_polygonization_start);

            // Convert to marker
            polygon_msg_cache_.markers.resize(polygon_msg_cache_.markers.size() + 1);

            convertPolygonPointsToMarker(is_bounding_box,
                                         label,
                                         z_min,
                                         z_max,
                                         polygon_points_,
                                         msg.header.frame_id,
                                         msg.header.stamp,
                                         polygon_msg_cache_.markers.back());

            if (polygon_msg_cache_.markers.back().points.empty())
            {
                polygon_msg_cache_.markers.pop_back();
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Polygonization time [ms]: %ld",
            std::chrono::duration_cast<std::chrono::milliseconds>(t_polygonization_total).count());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No clusters were extracted");
    }

    // Visualizations
    {
        cv::flip(segmenter_.image(), image_cache_, 0); // '0' denotes flipping around x-axis

        convertImageToRosMessage(image_cache_,
                                 msg.header.stamp.sec,
                                 msg.header.stamp.nanosec,
                                 "bgr8",
                                 msg.header.frame_id,
                                 image_msg_cache_);

        segmented_image_publisher_->publish(image_msg_cache_);
    }

    if (!ground_cloud_.empty())
    {
        cloud_msg_cache_.data.reserve(sizeof(pcl::PointXYZRGB) * ground_cloud_.size());
        cloud_msg_cache_.header = msg.header;
        cloud_msg_cache_.is_bigendian = msg.is_bigendian;
        convertPCLToPointCloud2(ground_cloud_, cloud_msg_cache_);
        ground_cloud_publisher_->publish(cloud_msg_cache_);
    }

    if (!obstacle_cloud_.empty())
    {
        cloud_msg_cache_.data.reserve(sizeof(pcl::PointXYZRGB) * obstacle_cloud_.size());
        cloud_msg_cache_.header = msg.header;
        cloud_msg_cache_.is_bigendian = msg.is_bigendian;
        convertPCLToPointCloud2(obstacle_cloud_, cloud_msg_cache_);
        obstacle_cloud_publisher_->publish(cloud_msg_cache_);
    }

    if (!unsegmented_cloud_.empty())
    {
        cloud_msg_cache_.data.reserve(sizeof(pcl::PointXYZRGB) * unsegmented_cloud_.size());
        cloud_msg_cache_.header = msg.header;
        cloud_msg_cache_.is_bigendian = msg.is_bigendian;
        convertPCLToPointCloud2(unsegmented_cloud_, cloud_msg_cache_);
        unsegmented_cloud_publisher_->publish(cloud_msg_cache_);
    }

    if (!clustered_cloud_.empty())
    {
        cloud_msg_cache_.data.reserve(sizeof(pcl::PointXYZRGB) * clustered_cloud_.size());
        cloud_msg_cache_.header = msg.header;
        cloud_msg_cache_.is_bigendian = msg.is_bigendian;
        convertPCLToPointCloud2(clustered_cloud_, cloud_msg_cache_);
        clustered_cloud_publisher_->publish(cloud_msg_cache_);
    }

    if (!polygon_msg_cache_.markers.empty())
    {
        obstacle_outlines_publisher_->publish(polygon_msg_cache_);
    }
}
} // namespace processing

std::int32_t main(std::int32_t argc, const char* const* argv)
{
    std::int32_t ret = EXIT_SUCCESS;

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers(rclcpp::SignalHandlerOptions::All);

    try
    {
        const auto node = std::make_shared<processing::Processor>();
        const auto subscriber = node->inputCloudSubscriber();

        sensor_msgs::msg::PointCloud2 msg;
        rclcpp::MessageInfo msg_info;

        msg.data.reserve(processing::Processor::MAX_PTS * sizeof(pcl::PointXYZIR));

        rclcpp::WaitSet wait_set;

        wait_set.add_subscription(subscriber);

        while (rclcpp::ok())
        {
            const auto wait_result =
                wait_set.wait(std::chrono::milliseconds(processing::Processor::WAIT_SET_TIME_MS));

            if (wait_result.kind() == rclcpp::WaitResultKind::Ready)
            {
                if (subscriber->take(msg, msg_info))
                {
                    node->run(msg);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "Unknown exception." << std::endl;
        ret = EXIT_FAILURE;
    }

    rclcpp::shutdown();

    return ret;
}
