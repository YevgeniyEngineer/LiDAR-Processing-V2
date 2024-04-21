// STL
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// PCL
#include <pcl/PCLPointField.h> // pcl::PCLPointField::PointFieldTypes
#include <pcl/io/pcd_io.h>     // pcl::io::loadPCDFile
#include <pcl/point_cloud.h>   // pcl::PointCloud
#include <pcl/point_types.h>   // pcl::PointXYZI

// ROS2
#include <rclcpp/executors.hpp>             // rclcpp::spin
#include <rclcpp/logging.hpp>               // RCLCPP_INFO
#include <rclcpp/node.hpp>                  // rclcpp::Node
#include <rclcpp/publisher.hpp>             // rclcpp::Publisher
#include <rclcpp/qos.hpp>                   // rclcpp::QoS
#include <rclcpp/timer.hpp>                 // rclcpp::TimerBase
#include <rclcpp/utilities.hpp>             // rclcpp::shutdown
#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs::msg::PointCloud2
#include <sensor_msgs/msg/point_field.hpp>  // sensor_msgs::msg::PointField

class Node final : public rclcpp::Node
{
  public:
    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;

    Node() : rclcpp::Node("dataloader")
    {
        this->declare_parameter<std::string>("data_path");
        this->declare_parameter<std::string>("topic");
        this->declare_parameter<std::int32_t>("frequency_hz");

        data_path_ = this->get_parameter("data_path").as_string();
        topic_ = this->get_parameter("topic").as_string();
        sleep_duration_ms_ = std::chrono::milliseconds(
            static_cast<std::int64_t>(1000.0 / static_cast<double>(this->get_parameter("frequency_hz").as_int())));

        loadData();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_, 10);
        timer_ = this->create_wall_timer(sleep_duration_ms_, std::bind(&Node::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Datalogger node constructed.");
    }

  private:
    std::filesystem::path data_path_;
    std::string topic_;
    std::chrono::milliseconds sleep_duration_ms_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

    std::vector<pcl::PointCloud<pcl::PointXYZI>> pointclouds_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>>::const_iterator pointcloud_iterator_;

    PointCloud2 message_;

    void loadData()
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

        for (const auto& file : files)
        {
            pcl::PointCloud<pcl::PointXYZI> pointcloud;

            if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, pointcloud) == -1)
            {
                throw std::runtime_error("Cloud not read " + file.string() + " file");
            }

            pointclouds_.push_back(std::move(pointcloud));
        }

        pointcloud_iterator_ = pointclouds_.cbegin();

        std::size_t reservation_size = 0U;
        for (const auto& cloud : pointclouds_)
        {
            reservation_size = std::max(reservation_size, (cloud.points.size() * sizeof(pcl::PointXYZI)));
        }

        message_.data.reserve(reservation_size);
        message_.fields.reserve(4);
    }

    void timerCallback()
    {
        static constexpr std::int64_t SECONDS_TO_NANOSECONDS = 1'000'000'000LL;

        if (pointcloud_iterator_ == pointclouds_.cend())
        {
            pointcloud_iterator_ = pointclouds_.cbegin();
        }

        message_.header.frame_id = "lidar"; // sensor frame

        const auto nanosec_timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
        message_.header.stamp.sec =
            static_cast<std::int32_t>(static_cast<double>(nanosec_timestamp / SECONDS_TO_NANOSECONDS));
        message_.header.stamp.nanosec = static_cast<std::uint32_t>(
            nanosec_timestamp -
            static_cast<std::int64_t>(static_cast<double>(message_.header.stamp.sec) * SECONDS_TO_NANOSECONDS));

        message_.height = pointcloud_iterator_->height;
        message_.width = pointcloud_iterator_->width;
        message_.is_bigendian = false;
        message_.point_step = sizeof(pcl::PointXYZI);
        message_.row_step = sizeof(pcl::PointXYZI) * message_.width;
        message_.is_dense = pointcloud_iterator_->is_dense;

        message_.fields.resize(4);

        message_.fields[0].name = "x";
        message_.fields[0].offset = offsetof(pcl::PointXYZI, x);
        message_.fields[0].datatype = PointFieldTypes::FLOAT32;
        message_.fields[0].count = 1;

        message_.fields[1].name = "y";
        message_.fields[1].offset = offsetof(pcl::PointXYZI, y);
        message_.fields[1].datatype = PointFieldTypes::FLOAT32;
        message_.fields[1].count = 1;

        message_.fields[2].name = "z";
        message_.fields[2].offset = offsetof(pcl::PointXYZI, z);
        message_.fields[2].datatype = PointFieldTypes::FLOAT32;
        message_.fields[2].count = 1;

        message_.fields[3].name = "intensity";
        message_.fields[3].offset = offsetof(pcl::PointXYZI, intensity);
        message_.fields[3].datatype = PointFieldTypes::FLOAT32;
        message_.fields[3].count = 1;

        message_.data.resize(sizeof(pcl::PointXYZI) * pointcloud_iterator_->size());
        std::memcpy(static_cast<void*>(message_.data.data()), static_cast<const void*>(pointcloud_iterator_->data()),
                    sizeof(pcl::PointXYZI) * pointcloud_iterator_->size());

        publisher_->publish(message_);

        ++pointcloud_iterator_;
    }
};

std::int32_t main(std::int32_t argc, char** argv)
{
    std::int32_t ret = EXIT_SUCCESS;

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

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
