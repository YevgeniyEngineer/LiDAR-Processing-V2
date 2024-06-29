#ifndef LIDAR_PROCESSING_LIB__POINT_TYPES_HPP
#define LIDAR_PROCESSING_LIB__POINT_TYPES_HPP

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
struct EIGEN_ALIGN16 PointXYZR
{
    PCL_ADD_POINT4D;                // This adds the XYZ coordinates and padding
    std::uint16_t ring;             // Laser ring index
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
};

struct EIGEN_ALIGN16 PointXYZIR
{
    PCL_ADD_POINT4D;                // This adds the XYZ coordinates and padding
    float intensity;                // Intensity of reflection
    std::uint16_t ring;             // Laser ring index
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
};                                  // Force SSE alignment
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZR, (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

#endif // LIDAR_PROCESSING_LIB__POINT_TYPES_HPP
