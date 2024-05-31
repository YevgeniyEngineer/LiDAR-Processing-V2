// STL
#include <cstdint>
#include <cstdlib>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS2
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
                                                                                                       ring,
                                                                                                       ring))

class Node final : public rclcpp::Node
{
  public:
    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

    Node()
        : rclcpp::Node {"processor"}
    {
        using std::placeholders::_1;

        this->declare_parameter<std::string>("input_cloud_topic");
        this->declare_parameter<std::string>("ground_cloud_topic");
        this->declare_parameter<std::string>("obstacle_cloud_topic");
        this->declare_parameter<std::string>("clustered_cloud_topic");
        this->declare_parameter<std::string>("obstacle_outlines_topic");

        input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
        ground_cloud_topic_ = this->get_parameter("ground_cloud_topic").as_string();
        obstacle_cloud_topic_ = this->get_parameter("obstacle_cloud_topic").as_string();
        clustered_cloud_topic_ = this->get_parameter("clustered_cloud_topic").as_string();
        obstacle_outlines_topic_ = this->get_parameter("obstacle_outlines_topic").as_string();

        input_cloud_subscriber_ =
            this->create_subscription<PointCloud2>(input_cloud_topic_, 10, std::bind(&Node::topicCallback, this, _1));

        ground_cloud_publisher_ = this->create_publisher<PointCloud2>(ground_cloud_topic_, 10);
        obstacle_cloud_publisher_ = this->create_publisher<PointCloud2>(obstacle_cloud_topic_, 10);
        clustered_cloud_publisher_ = this->create_publisher<PointCloud2>(clustered_cloud_topic_, 10);
        obstacle_outlines_publisher_ = this->create_publisher<MarkerArray>(obstacle_outlines_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "%s node constructed", this->get_name());
    }

    void topicCallback(const PointCloud2& msg)
    {
        RCLCPP_INFO(this->get_logger(), "Processing a new PointCloud2 message");
    }

  private:
    std::string input_cloud_topic_;
    std::string ground_cloud_topic_;
    std::string obstacle_cloud_topic_;
    std::string clustered_cloud_topic_;
    std::string obstacle_outlines_topic_;

    PointCloud2 cloud_cache_;
    MarkerArray polygon_cache_;

    rclcpp::Subscription<PointCloud2>::SharedPtr input_cloud_subscriber_;
    rclcpp::Publisher<PointCloud2>::SharedPtr ground_cloud_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_cloud_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr clustered_cloud_publisher_;
    rclcpp::Publisher<MarkerArray>::SharedPtr obstacle_outlines_publisher_;
};

std::int32_t main(std::int32_t argc, char** argv)
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
