#include <lidar_processing_lib/segmentation/jcp_segmenter.hpp>

namespace lidar_perception_lib::segmentation
{
void JCPSegmenter::segment(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud,
                           std::vector<JCPSegmentationLabel>& segmentation_labels)
{
    projectToRangeImage(input_cloud);

    buildRingShapedElevationConjunctionMap(input_cloud);

    performCourseSegmentation(input_cloud);

    refineSegmentationThroughJumpConvolutionProcess(input_cloud, segmentation_labels);
}

void JCPSegmenter::projectToRangeImage(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud)
{
}

void JCPSegmenter::buildRingShapedElevationConjunctionMap(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud)
{
}

void JCPSegmenter::performCourseSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& input_cloud)
{
}

void JCPSegmenter::refineSegmentationThroughJumpConvolutionProcess(
    const pcl::PointCloud<pcl::PointXYZIR>& input_cloud,
    std::vector<JCPSegmentationLabel>& segmentation_labels)
{
}

} // namespace lidar_perception_lib::segmentation
