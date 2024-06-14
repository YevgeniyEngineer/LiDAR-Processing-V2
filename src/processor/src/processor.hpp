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

// Segmenter
#include "segmenter.hpp"

// Clusterer
#include "clusterer.hpp"

// Polygonizer
#include "polygonizer.hpp"

// STL
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

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

// OpenCV
#include <opencv2/opencv.hpp>

namespace processing
{
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

} // namespace processing

#endif // PROCESSOR_HPP
