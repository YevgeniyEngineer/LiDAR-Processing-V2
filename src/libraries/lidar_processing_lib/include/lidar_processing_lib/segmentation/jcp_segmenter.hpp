#ifndef LIDAR_PROCESSING_LIB_JCP_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB_JCP_SEGMENTER_HPP

// PCL
#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZI

namespace lidar_perception_lib::segmentation
{
class JCPSegmenter
{
  public:
    JCPSegmenter() = default;
    ~JCPSegmenter() = default;

    void segment()
    {
    }

  private:
};
} // namespace lidar_perception_lib::segmentation

#endif // LIDAR_PROCESSING_LIB_JCP_SEGMENTER_HPP
