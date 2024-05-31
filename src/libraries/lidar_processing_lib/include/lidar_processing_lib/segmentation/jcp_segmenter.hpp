#ifndef LIDAR_PROCESSING_LIB_JCP_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB_JCP_SEGMENTER_HPP

// Internal
// #include <containers/queue.hpp>

// STL
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <queue>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Eigen
#include <Eigen/Dense>

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

namespace lidar_perception_lib::segmentation
{
struct JCPSegmenterConfig
{
    float sensor_height = 1.73;
    std::int32_t width = 2083;
    std::int32_t height = 64;
    float min_range = 2.0F;
    float max_range = 100.0F;
    float max_gradient_deg = 7.0F;
    float ground_height_uncertainty_m = 0.2F;
    float ring_spacing_m = 2.0F;
    float range_kernel_uncertainty_m = 1.0F;
    float amplification_factor = 5.0F;
};

enum class JCPSegmentationLabel : std::uint8_t
{
    UNKNOWN = 0,
    GROUND = 1,
    OBSTACLE = 2
};

class JCPSegmenter
{
  public:
    inline static const auto CV_OBSTACLE = cv::Vec3b(0, 0, 255);
    inline static const auto CV_GROUND = cv::Vec3b(0, 255, 0);
    inline static const auto CV_INTERSECTION_OR_UNKNOWN = cv::Vec3b(0, 255, 255);
    inline static const auto CV_INTERSECTION = cv::Vec3b(255, 0, 0);
    inline static const auto CV_UNKNOWN = cv::Vec3b(255, 255, 255);

    static constexpr float INVALID_REGION_Z = std::numeric_limits<float>::max();
    static constexpr std::int32_t INVALID_INDEX = -1;

    JCPSegmenter() = default;
    ~JCPSegmenter() = default;

    JCPSegmenterConfig& config() noexcept
    {
        return config_;
    }

    const JCPSegmenterConfig& config() const noexcept
    {
        return config_;
    }

    void segment(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud,
                 std::vector<JCPSegmentationLabel>& segmentation_labels);

  private:
    struct Neighbor
    {
        std::int32_t x;
        std::int32_t y;
    };

    static constexpr Neighbor neighbors[24] = {
        {-2, -2}, {-1, -2}, {0, -2}, {1, -2}, {2, -2}, {-2, -1}, {-1, -1}, {0, -1}, {1, -1}, {2, -1}, {-2, 0}, {-1, 0},
        {1, 0},   {2, 0},   {-2, 1}, {-1, 1}, {0, 1},  {1, 1},   {2, 1},   {-2, 2}, {-1, 2}, {0, 2},  {1, 2},  {2, 2}};

    JCPSegmenterConfig config_;

    cv::Mat range_image_;
    cv::Mat region_indices_;
    std::vector<float> region_minz_;
    std::vector<std::int32_t> cloud_index_;
    std::vector<cv::Mat> channels_;
    cv::Mat kernel_;
    std::queue<Neighbor> index_queue_;
    Eigen::Vector<float, 24> D_;
    Eigen::Vector<float, 24> W_;
    std::array<std::int32_t, 24> mask_;

    void projectToRangeImage(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud);
    void buildRingShapedElevationConjunctionMap(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud);
    void performCourseSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud);
    void refineSegmentationThroughJumpConvolutionProcess(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud,
                                                         std::vector<JCPSegmentationLabel>& segmentation_labels);
};
} // namespace lidar_perception_lib::segmentation

#endif // LIDAR_PROCESSING_LIB_JCP_SEGMENTER_HPP
