// STL
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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

void convertImageToRosMessage(const cv::Mat& cv_image,
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

class Node final : public rclcpp::Node
{
  public:
    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Image = sensor_msgs::msg::Image;

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
        image_publisher_ = this->create_publisher<Image>("jcp_image", 10);
        clustered_cloud_publisher_ = this->create_publisher<PointCloud2>(clustered_cloud_topic_, 10);
        obstacle_outlines_publisher_ = this->create_publisher<MarkerArray>(obstacle_outlines_topic_, 10);

        // Reserve memory
        input_cloud_.points.reserve(MAX_PTS);
        ground_cloud_.points.reserve(MAX_PTS);
        obstacle_cloud_.points.reserve(MAX_PTS);
        image_cache_ =
            cv::Mat::zeros(segmentation::Segmenter::IMAGE_HEIGHT, segmentation::Segmenter::IMAGE_WIDTH, CV_8UC3);

        RCLCPP_INFO(this->get_logger(), "%s node constructed", this->get_name());
    }

    void topicCallback(const PointCloud2& msg);

  private:
    std::string input_cloud_topic_;
    std::string ground_cloud_topic_;
    std::string obstacle_cloud_topic_;
    std::string clustered_cloud_topic_;
    std::string obstacle_outlines_topic_;

    cv::Mat image_cache_;
    Image image_msg_cache_;
    PointCloud2 cloud_msg_cache_;
    MarkerArray polygon_msg_cache_;

    rclcpp::Subscription<PointCloud2>::SharedPtr input_cloud_subscriber_;
    rclcpp::Publisher<PointCloud2>::SharedPtr ground_cloud_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr obstacle_cloud_publisher_;
    rclcpp::Publisher<Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr clustered_cloud_publisher_;
    rclcpp::Publisher<MarkerArray>::SharedPtr obstacle_outlines_publisher_;

    pcl::PointCloud<pcl::PointXYZIR> input_cloud_;

    segmentation::Segmenter segmenter_;
    std::vector<segmentation::Label> segmentation_labels_;
    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud_;
};

inline static void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_pcl,
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

void Node::topicCallback(const PointCloud2& msg)
{
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

    // Ground segmentation
    const auto t_segmentation_start = std::chrono::steady_clock::now();

    segmenter_.segment(input_cloud_, segmentation_labels_);

    ground_cloud_.clear();
    obstacle_cloud_.clear();

    for (std::uint32_t i = 0; i < input_cloud_.points.size(); ++i)
    {
        const auto& p = input_cloud_.points[i];
        const auto label = segmentation_labels_[i];

        if (label == segmentation::Label::GROUND)
        {
            ground_cloud_.push_back({p.x, p.y, p.z, 220, 220, 220});
        }
        else if (label == segmentation::Label::OBSTACLE)
        {
            obstacle_cloud_.push_back({p.x, p.y, p.z, 0, 255, 0});
        }
    }

    const auto t_segmentation_stop = std::chrono::steady_clock::now();
    const auto t_segmentation_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(t_segmentation_stop - t_segmentation_start).count();

    RCLCPP_INFO(this->get_logger(), "Segmentation time [ms]: %ld", t_segmentation_elapsed);
    RCLCPP_INFO(this->get_logger(),
                "Extracted %lu ground and %lu obstacle points from %lu-point cloud",
                ground_cloud_.size(),
                obstacle_cloud_.size(),
                input_cloud_.size());

    // Show the JCP image
    {
        cv::flip(segmenter_.image(), image_cache_, 0); // '0' denotes flipping around x-axis

        convertImageToRosMessage(image_cache_,
                                 msg.header.stamp.sec,
                                 msg.header.stamp.nanosec,
                                 "bgr8",
                                 msg.header.frame_id,
                                 image_msg_cache_);

        image_publisher_->publish(image_msg_cache_);
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
