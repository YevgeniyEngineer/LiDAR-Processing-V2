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

// STL
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <random>
#include <tuple>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS2
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// Segmenter
#include "segmenter.hpp"

// Clusterer
#include "clusterer.hpp"

// Polygonizer
#include "polygonizer.hpp"

class Node final : public rclcpp::Node
{
  public:
    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Image = sensor_msgs::msg::Image;

    static constexpr std::size_t MAX_PTS = 200'000U;

    Node() : rclcpp::Node{"processor"}
    {
        using std::placeholders::_1;

        this->declare_parameter<std::string>("input_cloud_topic");
        this->declare_parameter<std::string>("ground_cloud_topic");
        this->declare_parameter<std::string>("obstacle_cloud_topic");
        this->declare_parameter<std::string>("unsegmented_cloud_topic");
        this->declare_parameter<std::string>("segmented_image_topic");
        this->declare_parameter<std::string>("clustered_cloud_topic");
        this->declare_parameter<std::string>("obstacle_outlines_topic");

        input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
        ground_cloud_topic_ = this->get_parameter("ground_cloud_topic").as_string();
        obstacle_cloud_topic_ = this->get_parameter("obstacle_cloud_topic").as_string();
        unsegmented_cloud_topic_ = this->get_parameter("unsegmented_cloud_topic").as_string();
        segmented_image_topic_ = this->get_parameter("segmented_image_topic").as_string();
        clustered_cloud_topic_ = this->get_parameter("clustered_cloud_topic").as_string();
        obstacle_outlines_topic_ = this->get_parameter("obstacle_outlines_topic").as_string();

        input_cloud_subscriber_ =
            this->create_subscription<PointCloud2>(input_cloud_topic_,
                                                   rclcpp::QoS(2).reliable().durability_volatile(),
                                                   std::bind(&Node::topicCallback, this, _1));

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
        input_cloud_.points.reserve(MAX_PTS);
        ground_cloud_.points.reserve(MAX_PTS);
        obstacle_cloud_.points.reserve(MAX_PTS);
        unsegmented_cloud_.points.reserve(MAX_PTS);
        image_cache_ = cv::Mat::zeros(
            segmentation::Segmenter::IMAGE_HEIGHT, segmentation::Segmenter::IMAGE_WIDTH, CV_8UC3);
        image_msg_cache_.data.reserve(3 * segmentation::Segmenter::IMAGE_HEIGHT *
                                      segmentation::Segmenter::IMAGE_WIDTH);
        cloud_msg_cache_.data.reserve(MAX_PTS);

        polygonizer_indices_.reserve(MAX_PTS);
        polygonizer_points_.reserve(MAX_PTS);
        polygon_msg_cache_.markers.reserve(1000); // Max expected polygons
        polygon_points_.reserve(100);             // Max points per polygon

        RCLCPP_INFO(this->get_logger(), "%s node constructed", this->get_name());
    }

    void topicCallback(const PointCloud2& msg);

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

    pcl::PointCloud<pcl::PointXYZIR> input_cloud_;

    segmentation::Segmenter segmenter_;
    std::vector<segmentation::Label> segmentation_labels_;
    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB> unsegmented_cloud_;

    clustering::Clusterer clusterer_;
    std::vector<clustering::ClusterLabel> clustering_labels_;
    pcl::PointCloud<pcl::PointXYZRGB> clustered_cloud_;

    polygonization::Polygonizer polygonizer_;

    std::vector<std::uint32_t> polygonizer_indices_;
    std::vector<polygonization::PointXY> polygonizer_points_;
    std::vector<polygonization::PointXY> polygon_points_;
};

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
    std::memcpy(&msg.data[0], cv_image.data, size);
}

static void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_pcl,
                                    sensor_msgs::msg::PointCloud2& cloud_ros)
{
    cloud_ros.is_dense = cloud_pcl.is_dense;
    cloud_ros.height = cloud_pcl.height;
    cloud_ros.width = cloud_pcl.width;
    cloud_ros.point_step = sizeof(pcl::PointXYZRGB);

    std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
        {"x", offsetof(pcl::PointXYZRGB, x), Node::PointFieldTypes::FLOAT32, 1},
        {"y", offsetof(pcl::PointXYZRGB, y), Node::PointFieldTypes::FLOAT32, 1},
        {"z", offsetof(pcl::PointXYZRGB, z), Node::PointFieldTypes::FLOAT32, 1},
        {"rgb", offsetof(pcl::PointXYZRGB, rgb), Node::PointFieldTypes::FLOAT32, 1}};

    for (const auto& field : fields)
    {
        sensor_msgs::msg::PointField field_cache;
        field_cache.name = std::get<0>(field);
        field_cache.offset = std::get<1>(field);
        field_cache.datatype = std::get<2>(field);
        field_cache.count = std::get<3>(field);
        cloud_ros.fields.emplace_back(std::move(field_cache));
    }

    const std::size_t byte_size = sizeof(pcl::PointXYZRGB) * cloud_pcl.size();
    cloud_ros.data.clear();
    cloud_ros.data.resize(byte_size);
    std::memcpy(cloud_ros.data.data(), &cloud_pcl.at(0), byte_size);
};

static void convertPolygonPointsToMarker(std::uint32_t polygon_index,
                                         double z_min,
                                         double z_max,
                                         const std::vector<polygonization::PointXY>& points,
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
    marker.ns = "convex_hull";
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
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;

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

void Node::topicCallback(const PointCloud2& msg)
{
    // Convert point cloud from sensor_msgs::msg::PointCloud2 to pcl::PointCloud<pcl::PointXYZIR>
    input_cloud_.clear();
    input_cloud_.header.frame_id = msg.header.frame_id;
    input_cloud_.width = msg.width;
    input_cloud_.height = msg.height;
    input_cloud_.header.stamp = static_cast<std::uint64_t>(
        std::round(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec) / 1e3);

    input_cloud_.points.resize(msg.height * msg.width);
    const auto point_step = msg.point_step;
    const auto row_step = msg.row_step;

    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        for (std::uint32_t col = 0; col < msg.width; ++col)
        {
            auto& point = input_cloud_.points[row * msg.width + col];
            const auto offset = row * row_step + col * point_step;

            point.x = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[0].offset]);
            point.y = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[1].offset]);
            point.z = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[2].offset]);
            point.intensity =
                *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[3].offset]);
            point.ring =
                *reinterpret_cast<const std::uint16_t*>(&msg.data[offset + msg.fields[4].offset]);
        }
    }

    // Ground segmentation
    const auto t_segmentation_start = std::chrono::steady_clock::now();

    segmenter_.segment(input_cloud_, segmentation_labels_);

    ground_cloud_.clear();
    obstacle_cloud_.clear();
    unsegmented_cloud_.clear();

    for (std::uint32_t i = 0; i < input_cloud_.points.size(); ++i)
    {
        const auto& p = input_cloud_.points[i];
        const auto label = segmentation_labels_[i];

        if (label == segmentation::Label::GROUND)
        {
            ground_cloud_.push_back({p.x, p.y, p.z, 124, 252, 0});
        }
        else if (label == segmentation::Label::OBSTACLE)
        {
            obstacle_cloud_.push_back({p.x, p.y, p.z, 200, 0, 0});
        }
        else
        {
            unsegmented_cloud_.push_back({p.x, p.y, p.z, 255, 255, 0});
        }
    }

    const auto t_segmentation_stop = std::chrono::steady_clock::now();
    const auto t_segmentation_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            t_segmentation_stop - t_segmentation_start)
                                            .count();

    RCLCPP_INFO(this->get_logger(), "Segmentation time [ms]: %ld", t_segmentation_elapsed);
    RCLCPP_INFO(this->get_logger(),
                "Extracted %lu ground and %lu obstacle points from %lu-point cloud",
                ground_cloud_.size(),
                obstacle_cloud_.size(),
                input_cloud_.size());

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
        *max_label_it != clustering::Clusterer::INVALID_LABEL)
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
                    clustered_cloud_.push_back({point.x, point.y, point.z, r, g, b});

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

            const auto t_polygonization_stop = std::chrono::steady_clock::now();
            t_polygonization_total += (t_polygonization_stop - t_polygonization_start);

            // Convert to marker
            polygon_msg_cache_.markers.resize(polygon_msg_cache_.markers.size() + 1);

            convertPolygonPointsToMarker(label,
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

        { // Polygonizer
            obstacle_outlines_publisher_->publish(polygon_msg_cache_);
        }
    }
}

std::int32_t main(std::int32_t argc, const char* const* argv)
{
    std::int32_t ret = EXIT_SUCCESS;

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers(rclcpp::SignalHandlerOptions::All);

    try
    {
        rclcpp::spin(std::make_shared<Node>());
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
