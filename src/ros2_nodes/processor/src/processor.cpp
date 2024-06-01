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

// namespace pcl
// {
// struct EIGEN_ALIGN16 PointXYZIR
// {
//     PCL_ADD_POINT4D;                // This adds the XYZ coordinates and padding
//     float intensity;                // Intensity of reflection
//     std::uint16_t ring;             // Laser ring index
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
// };                                  // Force SSE alignment
// } // namespace pcl

// // Register the point type
// POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
//                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
//                                                                                                        ring,
//                                                                                                        ring))

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
        recm_depth_image_publisher_ = this->create_publisher<Image>("recm_depth_image", 10);
        jcp_depth_image_publisher_ = this->create_publisher<Image>("jcp_depth_image", 10);
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

        // Ground segmentation
        auto t_segmentation_start = std::chrono::steady_clock::now();

        const float sensor_height = -1.73;
        const int width = 2300;      // 2083;
        const int height = 68;       // 64;
        const float min_range = 2.0; // 2.0;
        const float max_range = 100.0;

        const float delta_slope_deg = 7.0;
        const float delta_slope_rad = delta_slope_deg * M_PI / 180.0;
        const float ground_height_threshold_m = 0.2;
        const float ring_spacing_m = 2.0;
        const float range_kernel_uncertainty_m = 1.0;
        const float kernel_size = 5;

        static const auto CV_OBSTACLE = cv::Vec3b(0, 0, 255);
        static const auto CV_GROUND = cv::Vec3b(0, 255, 0);
        static const auto CV_INTERSECTION_OR_UNKNOWN = cv::Vec3b(0, 255, 255);
        static const auto CV_INTERSECTION = cv::Vec3b(255, 0, 0);
        static const auto CV_UNKNOWN = cv::Vec3b(255, 255, 255);

        static constexpr float INVALID_REGION_Z = std::numeric_limits<float>::max();
        static constexpr int INVALID_INDEX = -1;

        std::vector<float> region_minz;
        std::vector<int> cloud_index;

        const auto length = static_cast<int>((max_range - min_range) / ring_spacing_m);

        region_minz.assign(width * length, INVALID_REGION_Z);
        cloud_index.assign(width * height, INVALID_INDEX);

        cv::Mat range_image = cv::Mat::zeros(height, width, CV_8UC3);
        cv::Mat region_indices = cv::Mat::zeros(height, width, CV_32SC1);

        // Range projection
        {
            for (int i = 0; i < input_cloud_.points.size(); ++i)
            {
                const auto& point = input_cloud_.points[i];
                const auto range = std::sqrt(point.x * point.x + point.y * point.y);

                if (range < min_range || range > max_range)
                {
                    continue;
                }

                const int ring_index = input_cloud_.points[i].ring;

                if (ring_index >= height)
                {
                    continue;
                }

                float azimuth_angle = 0;

                // Handle the case where x and y are effectively 0
                if (std::fabs(point.x) < std::numeric_limits<float>::epsilon() &&
                    std::fabs(point.y) < std::numeric_limits<float>::epsilon())
                {
                    azimuth_angle = 0;
                }
                else
                {
                    azimuth_angle = std::atan2(point.y, point.x);

                    if (azimuth_angle < 0)
                    {
                        azimuth_angle += static_cast<float>(2 * M_PI);
                    }
                }

                const int col = static_cast<int>(std::round((width - 1) * (azimuth_angle * 180.0 / M_PI) / 360.0));

                if (col < 0 || col >= width)
                {
                    continue;
                }

                const int region_row_index = static_cast<int>((range - min_range) / ring_spacing_m);
                const int region_index = col * length + region_row_index;
                const int index = col * height + ring_index;

                region_minz[region_index] = std::min(region_minz[region_index], point.z);
                cloud_index[index] = i;
                region_indices.at<int>(ring_index, col) = static_cast<int>(region_row_index);
                range_image.at<cv::Vec3b>(ring_index, col) = CV_GROUND;
            }
        }

        // Ring-shaped Elevation Conjunction Map (RECM)
        {
            for (int i = 0; i < region_minz.size(); ++i)
            {
                // Reset at the start of each new row
                if (i % length == 0)
                {
                    region_minz[i] = sensor_height + ground_height_threshold_m;
                }
                else
                {
                    if (std::fabs(region_minz[i - 1] - INVALID_REGION_Z) < std::numeric_limits<float>::epsilon() ||
                        std::fabs(region_minz[i + 1] - INVALID_REGION_Z) < std::numeric_limits<float>::epsilon())
                    {
                        continue; // Cannot apply smoothing
                    }

                    // Smooth elevation values by averaging with neighbors if there is a significant difference
                    if (i + 1 < region_minz.size() && std::fabs(region_minz[i] - region_minz[i - 1]) > 0.5 &&
                        std::fabs(region_minz[i] - region_minz[i + 1]) > 0.5)
                    {

                        region_minz[i] = 0.5F * (region_minz[i - 1] + region_minz[i + 1]);
                    }
                }
            }

            float prev_z = sensor_height;
            for (int i = 0; i < region_minz.size(); ++i)
            {
                if (i % length == 0)
                {
                    if (region_minz[i] < sensor_height - ground_height_threshold_m)
                    {
                        prev_z = sensor_height;
                    }
                    else
                    {
                        prev_z = std::min(region_minz[i], sensor_height);
                    }
                }
                else
                {
                    region_minz[i] = std::min(region_minz[i], prev_z + ring_spacing_m * std::tan(delta_slope_rad));
                    prev_z = region_minz[i];
                }
            }

            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    const int id = cloud_index[i * height + j];
                    if (id == INVALID_INDEX)
                    {
                        continue;
                    }

                    const int index = region_indices.at<int>(j, i);
                    const float th_height = region_minz[i * length + index];
                    if (input_cloud_.points[id].z >= (th_height + ground_height_threshold_m))
                    {
                        range_image.at<cv::Vec3b>(j, i) = CV_OBSTACLE;
                    }
                }
            }
        }

        // Jump Convolution Process (JCP)
        cv::Mat show_image;
        {
            std::vector<cv::Mat> channels;
            cv::split(range_image, channels);
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::dilate(channels[2], channels[2], element);
            cv::merge(channels, range_image);

            struct Index
            {
                int x = 0;
                int y = 0;
            };

            std::queue<Index> index_queue;

            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    if (range_image.at<cv::Vec3b>(j, i) == CV_INTERSECTION_OR_UNKNOWN)
                    {
                        const int index = cloud_index[j * height + i];

                        // Valid index
                        if (index != INVALID_INDEX)
                        {
                            index_queue.push({i, j});
                            range_image.at<cv::Vec3b>(j, i) = CV_INTERSECTION;
                        }
                        // Invalid index
                        else
                        {
                            range_image.at<cv::Vec3b>(j, i) = CV_UNKNOWN;
                        }
                    }
                }
            }

            // Flip the image vertically
            cv::flip(range_image, show_image, 0); // '0' denotes flipping around x-axis

            // Show the RECM image
            {
                Image recm_depth_image;
                convertImageToRosMessage(show_image,
                                         msg.header.stamp.sec,
                                         msg.header.stamp.nanosec,
                                         "bgr8",
                                         msg.header.frame_id,
                                         recm_depth_image);
                recm_depth_image_publisher_->publish(recm_depth_image);
            }

            Eigen::VectorXf D(24);
            int mask[24];
            std::fill(std::begin(mask), std::end(mask), INVALID_INDEX);

            static constexpr int neighborx[24] = {-2, -1, 0,  1,  2, -2, -1, 0,  1,  2, -2, -1,
                                                  1,  2,  -2, -1, 0, 1,  2,  -2, -1, 0, 1,  2};
            static constexpr int neighbory[24] = {-2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0,
                                                  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  2, 2};
            while (!index_queue.empty())
            {
                const Index id = index_queue.front();
                index_queue.pop();
                const int cloud_id = id.x * height + id.y;
                float sumD = 0;

                for (int i = 0; i < 24; ++i)
                {
                    const int nx = neighborx[i] + id.x;
                    const int ny = neighbory[i] + id.y;
                    const int ncloud_id = nx * height + ny;

                    if (nx < 0 || nx >= width || ny < 0 || ny >= height || cloud_index[ncloud_id] == INVALID_INDEX)
                    {
                        D(i) = 0;
                        mask[i] = INVALID_INDEX;
                        continue;
                    }

                    const auto& p1 = input_cloud_.points[cloud_index[cloud_id]];
                    const auto& p2 = input_cloud_.points[cloud_index[ncloud_id]];

                    const auto range_diff = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                                                      (p1.z - p2.z) * (p1.z - p2.z));

                    if (range_diff > range_kernel_uncertainty_m)
                    {
                        D(i) = 0;
                    }
                    else
                    {
                        D(i) = std::exp(-kernel_size * range_diff);
                        sumD += D(i);
                    }

                    if (range_image.at<cv::Vec3b>(ny, nx) == CV_INTERSECTION)
                    {
                        mask[i] = 2;
                    }
                    else if (range_image.at<cv::Vec3b>(ny, nx) == CV_GROUND)
                    {
                        mask[i] = 1;
                    }
                    else if (range_image.at<cv::Vec3b>(ny, nx) == CV_OBSTACLE)
                    {
                        mask[i] = 0;
                    }
                    else
                    {
                        mask[i] = -1;
                    }
                }

                if (std::fabs(sumD) > std::numeric_limits<float>::epsilon())
                {
                    Eigen::VectorXf W = D / sumD;

                    float score_r = 0.0F;
                    float score_g = 0.0F;

                    for (int i = 0; i < D.size(); ++i)
                    {
                        if (mask[i] == 0)
                        {
                            score_r += W(i);
                        }
                        else if (mask[i] == 1)
                        {
                            score_g += W(i);
                        }
                    }

                    if (score_r > score_g)
                    {
                        range_image.at<cv::Vec3b>(id.y, id.x) = CV_OBSTACLE;
                    }
                    else
                    {
                        range_image.at<cv::Vec3b>(id.y, id.x) = CV_GROUND;
                    }
                }
            }
        }

        // Show the JCP image
        {
            // Flip the image vertically
            cv::flip(range_image, show_image, 0); // '0' denotes flipping around x-axis

            Image jcp_depth_image;
            convertImageToRosMessage(show_image,
                                     msg.header.stamp.sec,
                                     msg.header.stamp.nanosec,
                                     "bgr8",
                                     msg.header.frame_id,
                                     jcp_depth_image);
            jcp_depth_image_publisher_->publish(jcp_depth_image);
        }

        // Populate labels - ground and nonground
        pcl::PointCloud<pcl::PointXYZRGB> ground_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud;

        ground_cloud.reserve(input_cloud_.size());
        obstacle_cloud.reserve(input_cloud_.size());

        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                const int index = cloud_index[i * height + j];

                if (index != INVALID_INDEX)
                {
                    const auto& p = input_cloud_[index];

                    if (range_image.at<cv::Vec3b>(j, i) == CV_GROUND)
                    {
                        ground_cloud.push_back({p.x, p.y, p.z, 220, 220, 220});
                    }
                    else if (range_image.at<cv::Vec3b>(j, i) == CV_OBSTACLE)
                    {
                        obstacle_cloud.push_back({p.x, p.y, p.z, 0, 255, 0});
                    }
                    else
                    {
                        // ???
                    }
                }
            }
        }

        // std::vector<segmentation::Label> labels;
        // segmenter_.segment(input_cloud_, labels);

        // const auto& range_image = segmenter_.rangeImage();

        // // Show the JCP image
        // {
        //     // Flip the image vertically
        //     cv::Mat show_image;
        //     cv::flip(range_image, show_image, 0); // '0' denotes flipping around x-axis

        //     Image recm_depth_image;
        //     convertImageToRosMessage(show_image,
        //                              msg.header.stamp.sec,
        //                              msg.header.stamp.nanosec,
        //                              "bgr8",
        //                              msg.header.frame_id,
        //                              recm_depth_image);
        //     recm_depth_image_publisher_->publish(recm_depth_image);
        // }

        // for (std::uint32_t i = 0; i < input_cloud_.points.size(); ++i)
        // {
        //     const auto& p = input_cloud_.points[i];
        //     const auto label = labels[i];

        //     switch (label)
        //     {
        //         case segmentation::Label::GROUND:
        //             {
        //                 ground_cloud.push_back({p.x, p.y, p.z, 220, 220, 220});
        //                 break;
        //             }
        //         case segmentation::Label::OBSTACLE:
        //             {
        //                 obstacle_cloud.push_back({p.x, p.y, p.z, 0, 255, 0});
        //                 break;
        //             }
        //         case segmentation::Label::UNKNOWN:
        //             {
        //                 break;
        //             }
        //         default:
        //             {
        //                 break;
        //             }
        //     }
        // }

        auto t_segmentation_stop = std::chrono::steady_clock::now();
        auto t_segmentation_elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(t_segmentation_stop - t_segmentation_start).count();

        RCLCPP_INFO(this->get_logger(), "Segmentation time [ms]: %ld", t_segmentation_elapsed);

        // Convert and publish
        const auto convertPCLToPointCloud2 = [](const pcl::PointCloud<pcl::PointXYZRGB>& cloud_pcl,
                                                sensor_msgs::msg::PointCloud2& cloud_ros) {
            cloud_ros.is_dense = cloud_pcl.is_dense;
            cloud_ros.height = cloud_pcl.height;
            cloud_ros.width = cloud_pcl.width;
            cloud_ros.point_step = sizeof(pcl::PointXYZRGB);

            std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
                {"x", offsetof(pcl::PointXYZRGB, x), PointFieldTypes::FLOAT32, 1},
                {"y", offsetof(pcl::PointXYZRGB, y), PointFieldTypes::FLOAT32, 1},
                {"z", offsetof(pcl::PointXYZRGB, z), PointFieldTypes::FLOAT32, 1},
                {"rgb", offsetof(pcl::PointXYZRGB, rgb), PointFieldTypes::FLOAT32, 1}};

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

        RCLCPP_INFO(this->get_logger(),
                    "Extracted %lu ground and %lu obstacle points",
                    ground_cloud.size(),
                    obstacle_cloud.size());

        if (!ground_cloud.empty())
        {
            PointCloud2 ground_msg;
            ground_msg.data.reserve(sizeof(pcl::PointXYZRGB) * ground_cloud.size());
            ground_msg.header = msg.header;
            ground_msg.is_bigendian = msg.is_bigendian;
            convertPCLToPointCloud2(ground_cloud, ground_msg);
            ground_cloud_publisher_->publish(ground_msg);
        }

        if (!obstacle_cloud.empty())
        {
            PointCloud2 obstacle_msg;
            obstacle_msg.data.reserve(sizeof(pcl::PointXYZRGB) * obstacle_cloud.size());
            obstacle_msg.header = msg.header;
            obstacle_msg.is_bigendian = msg.is_bigendian;
            convertPCLToPointCloud2(obstacle_cloud, obstacle_msg);
            obstacle_cloud_publisher_->publish(obstacle_msg);
        }
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
    rclcpp::Publisher<Image>::SharedPtr recm_depth_image_publisher_;
    rclcpp::Publisher<Image>::SharedPtr jcp_depth_image_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr clustered_cloud_publisher_;
    rclcpp::Publisher<MarkerArray>::SharedPtr obstacle_outlines_publisher_;

    pcl::PointCloud<pcl::PointXYZIR> input_cloud_;

    segmentation::Segmenter segmenter_;
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
