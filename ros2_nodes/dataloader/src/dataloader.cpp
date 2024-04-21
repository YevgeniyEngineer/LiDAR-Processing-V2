// STL
#include <chrono>
#include <cstdint>
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
            static_cast<std::int64_t>(1000.0 / this->get_parameter("frequency_hz").as_double()));

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

    std::vector<std::filesystem::path> files_;
    std::vector<std::filesystem::path>::const_iterator file_iterator_;

    void loadData()
    {
    }

    void timerCallback()
    {
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
