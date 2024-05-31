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

    static constexpr std::size_t MAX_PTS = 200'000U;

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

        // Reserve memory
        input_cloud_.points.reserve(MAX_PTS);

        RCLCPP_INFO(this->get_logger(), "%s node constructed", this->get_name());
    }

    void topicCallback(const PointCloud2& msg)
    {
        RCLCPP_INFO(this->get_logger(), "Processing a new PointCloud2 message");

        // Convert point cloud from sensor_msgs::msg::PointCloud2 to pcl::PointCloud<pcl::PointXYZIR>
        input_cloud_.clear();
        input_cloud_.header.frame_id = msg.header.frame_id;
        input_cloud_.width = msg.width;
        input_cloud_.height = msg.height;
        input_cloud_.header.stamp =
            static_cast<std::uint64_t>(std::round(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec) / 1e3);

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
                point.intensity = *reinterpret_cast<const float*>(&msg.data[offset + msg.fields[3].offset]);
                point.ring = *reinterpret_cast<const std::uint16_t*>(&msg.data[offset + msg.fields[4].offset]);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Extracted %lu points from PointCloud2", input_cloud_.size());
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

    pcl::PointCloud<pcl::PointXYZIR> input_cloud_;
};

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
