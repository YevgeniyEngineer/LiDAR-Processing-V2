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

#include "dataloader.hpp"

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>

// PCL
#include <pcl/io/pcd_io.h> // pcl::io::loadPCDFile

// ROS2
#include <rclcpp/logging.hpp> // RCLCPP_INFO
#include <rclcpp/qos.hpp>     // rclcpp::QoS

namespace dataloading
{
Dataloader::Dataloader()
    : rclcpp::Node{Dataloader::NODE_NAME, rclcpp::NodeOptions().use_intra_process_comms(true)}
{
    this->declare_parameter<std::string>("data_path");
    this->declare_parameter<std::string>("vis_cloud_topic");
    this->declare_parameter<std::string>("organized_cloud_topic");
    this->declare_parameter<std::int32_t>("frequency_hz");

    data_path_ = this->get_parameter("data_path").as_string();
    vis_cloud_topic_ = this->get_parameter("vis_cloud_topic").as_string();
    organized_cloud_topic_ = this->get_parameter("organized_cloud_topic").as_string();
    sleep_duration_ms_ = std::chrono::milliseconds(static_cast<std::int64_t>(
        1000.0 / static_cast<double>(this->get_parameter("frequency_hz").as_int())));

    loadData();

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        organized_cloud_topic_, rclcpp::QoS(2).reliable().durability_volatile());
    vis_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        vis_cloud_topic_, rclcpp::QoS(2).reliable().durability_volatile());
    timer_ =
        this->create_wall_timer(sleep_duration_ms_, std::bind(&Dataloader::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "%s node constructed", this->get_name());
}

void Dataloader::addRingInfo(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
                             pcl::PointCloud<pcl::PointXYZIR>& cloud_out)
{
    cloud_out.clear();
    if (cloud_in.points.empty())
    {
        return;
    }

    enum class Quadrants : std::int32_t
    {
        FIRST = 0,
        SECOND = 1,
        THIRD = 2,
        FOURTH = 3
    };

    auto quadrant = Quadrants::FIRST;
    auto previous_quadrant = Quadrants::FIRST;
    std::uint16_t ring_index = 63;
    pcl::PointXYZIR point_cache;
    cloud_out.points.reserve(cloud_in.points.size());

    std::uint32_t points_per_ring = 0;
    std::uint32_t max_points_per_ring = 0;

    for (const auto& point : cloud_in.points)
    {
        float azimuth_rad = std::atan2(point.y, point.x);
        azimuth_rad = (azimuth_rad < 0) ? (azimuth_rad + 2.0F * M_PIf) : azimuth_rad;

        if (azimuth_rad < M_PI_2f)
        {
            quadrant = Quadrants::FIRST;
        }
        else if (azimuth_rad < M_PIf)
        {
            quadrant = Quadrants::SECOND;
        }
        else if (azimuth_rad < 1.5F * M_PIf)
        {
            quadrant = Quadrants::THIRD;
        }
        else
        {
            quadrant = Quadrants::FOURTH;
        }

        if (quadrant == Quadrants::FIRST && previous_quadrant == Quadrants::FOURTH &&
            ring_index > 0U)
        {
            max_points_per_ring = std::max(max_points_per_ring, points_per_ring);
            points_per_ring = 0;
            --ring_index;
        }

        previous_quadrant = quadrant;

        point_cache.x = point.x;
        point_cache.y = point.y;
        point_cache.z = point.z;
        point_cache.intensity = point.intensity;
        point_cache.ring = ring_index;

        cloud_out.push_back(point_cache);
        ++points_per_ring;
    }

    // RCLCPP_INFO(this->get_logger(), "Max points per ring: %u", max_points_per_ring);
}

void Dataloader::loadData()
{
    std::vector<std::filesystem::path> files;

    for (const auto& file : std::filesystem::directory_iterator(data_path_))
    {
        if (std::filesystem::is_regular_file(file) && (file.path().extension() == ".pcd"))
        {
            files.push_back(file.path());
        }
    }

    std::sort(files.begin(), files.end());

    pcl::PointCloud<pcl::PointXYZI> cloud_in;
    pcl::PointCloud<pcl::PointXYZIR> cloud_out;

    cloud_in.reserve(200'000 * sizeof(pcl::PointXYZI));
    cloud_out.reserve(200'000 * sizeof(pcl::PointXYZIR));

    pointclouds_.reserve(files.size());

    for (const auto& file : files)
    {
        cloud_in.points.clear();

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, cloud_in) == -1)
        {
            throw std::runtime_error("Cloud not read " + file.string() + " file");
        }

        addRingInfo(cloud_in, cloud_out);

        pointclouds_.push_back(cloud_out);
    }

    pointcloud_iterator_ = pointclouds_.cbegin();

    std::size_t reservation_size = 0U;
    for (const auto& cloud : pointclouds_)
    {
        reservation_size =
            std::max(reservation_size, (cloud.points.size() * sizeof(pcl::PointXYZIR)));
    }

    // For processing
    message_.data.reserve(reservation_size);

    message_.fields.resize(5);

    message_.fields[0].name = "x";
    message_.fields[0].offset = offsetof(pcl::PointXYZIR, x);
    message_.fields[0].datatype = PointFieldTypes::FLOAT32;
    message_.fields[0].count = 1;

    message_.fields[1].name = "y";
    message_.fields[1].offset = offsetof(pcl::PointXYZIR, y);
    message_.fields[1].datatype = PointFieldTypes::FLOAT32;
    message_.fields[1].count = 1;

    message_.fields[2].name = "z";
    message_.fields[2].offset = offsetof(pcl::PointXYZIR, z);
    message_.fields[2].datatype = PointFieldTypes::FLOAT32;
    message_.fields[2].count = 1;

    message_.fields[3].name = "intensity";
    message_.fields[3].offset = offsetof(pcl::PointXYZIR, intensity);
    message_.fields[3].datatype = PointFieldTypes::FLOAT32;
    message_.fields[3].count = 1;

    message_.fields[4].name = "ring";
    message_.fields[4].offset = offsetof(pcl::PointXYZIR, ring);
    message_.fields[4].datatype = PointFieldTypes::UINT16;
    message_.fields[4].count = 1;

    // For visualization of rings
    vis_message_.data.reserve(reservation_size);

    vis_message_.fields.resize(4);

    vis_message_.fields[0].name = "x";
    vis_message_.fields[0].offset = offsetof(pcl::PointXYZIR, x);
    vis_message_.fields[0].datatype = PointFieldTypes::FLOAT32;
    vis_message_.fields[0].count = 1;

    vis_message_.fields[1].name = "y";
    vis_message_.fields[1].offset = offsetof(pcl::PointXYZIR, y);
    vis_message_.fields[1].datatype = PointFieldTypes::FLOAT32;
    vis_message_.fields[1].count = 1;

    vis_message_.fields[2].name = "z";
    vis_message_.fields[2].offset = offsetof(pcl::PointXYZIR, z);
    vis_message_.fields[2].datatype = PointFieldTypes::FLOAT32;
    vis_message_.fields[2].count = 1;

    vis_message_.fields[3].name = "rgb";
    vis_message_.fields[3].offset = offsetof(pcl::PointXYZIR, intensity);
    vis_message_.fields[3].datatype = PointFieldTypes::FLOAT32;
    vis_message_.fields[3].count = 1;
}

void Dataloader::timerCallback()
{
    static constexpr std::int64_t SECONDS_TO_NANOSECONDS = 1'000'000'000LL;

    if (pointcloud_iterator_ == pointclouds_.cend())
    {
        pointcloud_iterator_ = pointclouds_.cbegin();
    }

    message_.header.frame_id = "lidar"; // sensor frame

    const auto nanosec_timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    message_.header.stamp.sec =
        static_cast<std::int32_t>(static_cast<double>(nanosec_timestamp) / SECONDS_TO_NANOSECONDS);
    message_.header.stamp.nanosec = static_cast<std::uint32_t>(
        nanosec_timestamp -
        static_cast<std::int64_t>(static_cast<double>(message_.header.stamp.sec) *
                                  SECONDS_TO_NANOSECONDS));

    message_.height = pointcloud_iterator_->height;
    message_.width = pointcloud_iterator_->width;
    message_.is_bigendian = false;
    message_.point_step = sizeof(pcl::PointXYZIR);
    message_.row_step = sizeof(pcl::PointXYZIR) * message_.width;
    message_.is_dense = pointcloud_iterator_->is_dense;

    message_.data.resize(sizeof(pcl::PointXYZIR) * pointcloud_iterator_->size());
    std::memcpy(static_cast<void*>(message_.data.data()),
                static_cast<const void*>(pointcloud_iterator_->data()),
                sizeof(pcl::PointXYZIR) * pointcloud_iterator_->size());

    publisher_->publish(message_);

    // For visualization
    {
        vis_message_.header = message_.header;
        vis_message_.height = message_.height;
        vis_message_.width = message_.width;
        vis_message_.is_bigendian = false;
        vis_message_.point_step = sizeof(pcl::PointXYZRGB);
        vis_message_.row_step = sizeof(pcl::PointXYZRGB) * vis_message_.width;
        vis_message_.is_dense = message_.is_dense;

        vis_message_.data.resize(sizeof(pcl::PointXYZRGB) * pointcloud_iterator_->size());

        const auto max_point_ring_index_iterator = std::max_element(
            pointcloud_iterator_->points.cbegin(),
            pointcloud_iterator_->points.cend(),
            [](const auto& a, const auto& b) -> bool { return (a.ring < b.ring); });

        if (max_point_ring_index_iterator != pointcloud_iterator_->points.cend())
        {
            const auto max_ring_index = max_point_ring_index_iterator->ring;

            pcl::PointXYZRGB point_cache;
            std::ptrdiff_t pointer_offset = 0;

            static constexpr std::array<std::array<std::uint8_t, 3>, 8> color_palette = {
                {{255, 0, 0},   // Red
                 {0, 255, 0},   // Green
                 {0, 0, 255},   // Blue
                 {255, 255, 0}, // Yellow
                 {255, 0, 255}, // Magenta
                 {0, 255, 255}, // Cyan
                 {255, 165, 0}, // Orange
                 {128, 0, 128}} // Purple
            };

            static constexpr std::size_t num_colors = color_palette.size();

            for (std::uint16_t ring = 0U; ring < max_ring_index + 1U; ++ring)
            {
                const auto& ring_colour = color_palette[ring % num_colors];

                point_cache.r = ring_colour[0];
                point_cache.g = ring_colour[1];
                point_cache.b = ring_colour[2];

                for (const auto& point : pointcloud_iterator_->points)
                {
                    if (point.ring == ring)
                    {
                        point_cache.x = point.x;
                        point_cache.y = point.y;
                        point_cache.z = point.z;

                        std::memcpy(static_cast<void*>(&vis_message_.data[pointer_offset]),
                                    static_cast<const void*>(&point_cache),
                                    sizeof(point_cache));

                        pointer_offset += sizeof(pcl::PointXYZRGB);
                    }
                }
            }
        }

        vis_publisher_->publish(vis_message_);
    }

    ++pointcloud_iterator_;
}

} // namespace dataloading

std::int32_t main(std::int32_t argc, const char* const* argv)
{
    std::int32_t ret = EXIT_SUCCESS;

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    try
    {
        rclcpp::spin(std::make_shared<dataloading::Dataloader>());
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
