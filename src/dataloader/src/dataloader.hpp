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

#ifndef DATALOADER_HPP
#define DATALOADER_HPP

// STL
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

// PCL
#include <pcl/PCLPointField.h> // pcl::PCLPointField::PointFieldTypes
#include <pcl/point_cloud.h>   // pcl::PointCloud
#include <pcl/point_types.h>   // pcl::PointXYZI

// ROS2
#include <rclcpp/executors.hpp>             // rclcpp::spin
#include <rclcpp/node.hpp>                  // rclcpp::Node
#include <rclcpp/publisher.hpp>             // rclcpp::Publisher
#include <rclcpp/timer.hpp>                 // rclcpp::TimerBase
#include <rclcpp/utilities.hpp>             // rclcpp::shutdown
#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs::msg::PointCloud2
#include <sensor_msgs/msg/point_field.hpp>  // sensor_msgs::msg::PointField

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

// Register the point type
POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

namespace dataloading
{
class Dataloader final : public rclcpp::Node
{
  public:
    static constexpr const char* NODE_NAME = "dataloader";

    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;

    Dataloader();

    void timerCallback();

  private:
    std::filesystem::path data_path_;
    std::string vis_cloud_topic_;
    std::string organized_cloud_topic_;
    std::chrono::milliseconds sleep_duration_ms_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

    std::vector<pcl::PointCloud<pcl::PointXYZIR>> pointclouds_;
    std::vector<pcl::PointCloud<pcl::PointXYZIR>>::const_iterator pointcloud_iterator_;

    PointCloud2 message_;

    rclcpp::Publisher<PointCloud2>::SharedPtr vis_publisher_;
    PointCloud2 vis_message_;

    void addRingInfo(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
                     pcl::PointCloud<pcl::PointXYZIR>& cloud_out);

    void loadData();
};
} // namespace dataloading

#endif // DATALOADER_HPP
