// STL
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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

    Node()
        : rclcpp::Node("dataloader")
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
        vis_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_rings", 10);
        timer_ = this->create_wall_timer(sleep_duration_ms_, std::bind(&Node::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "%s node constructed", this->get_name());
    }

  private:
    std::filesystem::path data_path_;
    std::string topic_;
    std::chrono::milliseconds sleep_duration_ms_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;

    std::vector<pcl::PointCloud<pcl::PointXYZIR>> pointclouds_;
    std::vector<pcl::PointCloud<pcl::PointXYZIR>>::const_iterator pointcloud_iterator_;

    PointCloud2 message_;

    rclcpp::Publisher<PointCloud2>::SharedPtr vis_publisher_;
    PointCloud2 vis_message_;

    void addRingInfo(const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
                     pcl::PointCloud<pcl::PointXYZIR>& cloud_out,
                     const float elevation_tolerance = 0.008F)
    {
        const auto elevationComparator = [](const auto& a, const auto& b) -> bool {
            const auto elev_a = std::atan2(a.z, std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z));
            const auto elev_b = std::atan2(b.z, std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z));
            return (elev_a < elev_b);
        };

        cloud_out.points.resize(cloud_in.points.size());
        cloud_out.width = cloud_in.width;
        cloud_out.height = cloud_in.height;
        cloud_out.is_dense = cloud_in.is_dense;

        for (std::size_t i = 0U; i < cloud_in.points.size(); ++i)
        {
            const auto& cloud_in_point = cloud_in.points[i];
            auto& cloud_out_point = cloud_out.points[i];

            cloud_out_point.x = cloud_in_point.x;
            cloud_out_point.y = cloud_in_point.y;
            cloud_out_point.z = cloud_in_point.z;
            cloud_out_point.intensity = cloud_in_point.intensity;
            cloud_out_point.ring = 0U;
        }

        if (!cloud_out.points.empty())
        {
            std::sort(cloud_out.begin(), cloud_out.end(), elevationComparator);

            std::uint16_t current_ring = 0U;

            const auto& first_point = cloud_out.points[0U];
            auto last_elevation = std::atan2(first_point.z,
                                             std::sqrt(first_point.x * first_point.x + first_point.y * first_point.y +
                                                       first_point.z * first_point.z));

            for (std::size_t i = 1U; i < cloud_out.points.size(); ++i)
            {
                auto& current_point = cloud_out.points[i];
                const auto current_elevation =
                    std::atan2(current_point.z,
                               std::sqrt(current_point.x * current_point.x + current_point.y * current_point.y +
                                         current_point.z * current_point.z));

                if (std::fabs(current_elevation - last_elevation) > elevation_tolerance)
                {
                    ++current_ring;
                    last_elevation = current_elevation;
                }

                current_point.ring = current_ring;
            }
        }
    }

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

        pcl::PointCloud<pcl::PointXYZI> cloud_in;

        for (const auto& file : files)
        {
            cloud_in.points.clear();

            if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, cloud_in) == -1)
            {
                throw std::runtime_error("Cloud not read " + file.string() + " file");
            }

            pcl::PointCloud<pcl::PointXYZIR> cloud_out;

            addRingInfo(cloud_in, cloud_out);

            pointclouds_.push_back(std::move(cloud_out));
        }

        pointcloud_iterator_ = pointclouds_.cbegin();

        std::size_t reservation_size = 0U;
        for (const auto& cloud : pointclouds_)
        {
            reservation_size = std::max(reservation_size, (cloud.points.size() * sizeof(pcl::PointXYZIR)));
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
            static_cast<std::int32_t>(static_cast<double>(nanosec_timestamp) / SECONDS_TO_NANOSECONDS);
        message_.header.stamp.nanosec = static_cast<std::uint32_t>(
            nanosec_timestamp -
            static_cast<std::int64_t>(static_cast<double>(message_.header.stamp.sec) * SECONDS_TO_NANOSECONDS));

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

            const auto max_point_ring_index_iterator =
                std::max_element(pointcloud_iterator_->points.cbegin(),
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
};

std::int32_t main(std::int32_t argc, const char* const* argv)
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
