#include "segmenter.hpp"

namespace segmentation
{
Segmenter::Segmenter()
    : config_ {},
      image_ {cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3)},
      depth_image_ {cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32F)},
      grid_mapping_indices_ {cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32SC1, cv::Scalar(INVALID_INDEX))},
      cloud_mapping_indices_ {cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32SC1, cv::Scalar(INVALID_INDEX))},
      kernel_ {cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5))}
{
    grid_slice_resolution_rad_ = config_.grid_slice_resolution_deg * DEG_TO_RAD;
    grid_number_of_radial_rings_ =
        static_cast<std::int32_t>((config_.max_distance_m - config_.min_distance_m) / config_.grid_radial_spacing_m);
    grid_number_of_azimuth_slices_ = static_cast<std::int32_t>(TWO_M_PIf / grid_slice_resolution_rad_);

    polar_grid_.resize(grid_number_of_radial_rings_ * grid_number_of_azimuth_slices_);

    elevation_map_.assign(grid_number_of_azimuth_slices_ * grid_number_of_radial_rings_, INVALID_Z);
    cloud_indices_.assign(IMAGE_HEIGHT * IMAGE_WIDTH, INVALID_INDEX);

    std::cerr << config_ << std::endl;
}

void Segmenter::config(const Configuration& config)
{
    config_ = config;
    grid_slice_resolution_rad_ = config_.grid_slice_resolution_deg * DEG_TO_RAD;
    grid_number_of_radial_rings_ =
        static_cast<std::int32_t>((config_.max_distance_m - config_.min_distance_m) / config_.grid_radial_spacing_m);
    grid_number_of_azimuth_slices_ = static_cast<std::int32_t>(TWO_M_PIf / grid_slice_resolution_rad_);

    polar_grid_.resize(grid_number_of_radial_rings_ * grid_number_of_azimuth_slices_);

    for (auto& cell : polar_grid_)
    {
        cell.reserve(150); // TODO: How to determine this number?
    }

    elevation_map_.assign(grid_number_of_azimuth_slices_ * grid_number_of_radial_rings_, INVALID_Z);
    cloud_indices_.assign(IMAGE_HEIGHT * IMAGE_WIDTH, INVALID_INDEX);

    std::cerr << config_ << std::endl;
}

void Segmenter::resetValues()
{
    image_.setTo(cv::Scalar(0));
    depth_image_.setTo(cv::Scalar(INVALID_DEPTH_M));

    grid_mapping_indices_.setTo(cv::Scalar(INVALID_INDEX));
    cloud_mapping_indices_.setTo(cv::Scalar(INVALID_INDEX));

    elevation_map_.assign(elevation_map_.size(), INVALID_Z);
    cloud_indices_.assign(cloud_indices_.size(), INVALID_INDEX);

    for (auto& cell : polar_grid_)
    {
        cell.clear();
    }
}

void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels)
{
    std::cerr << "Entered segment()" << std::endl;

    labels.assign(cloud.points.size(), Label::UNKNOWN);
    resetValues();

    RECM(cloud);
    JCP(cloud);
    populateLabels(cloud, labels);

    // constructElevationMap(cloud);

    // std::cerr << "Constructed elevation map" << std::endl;

    // performRECMSegmentation(cloud);

    // std::cerr << "Performed RECM segmentation" << std::endl;

    // performJCPSegmentation(cloud);

    // std::cerr << "Performed JCP segmentation" << std::endl;

    // extractSegmentationLabels(cloud, labels);

    // std::cerr << "Extracted Segmentation labels" << std::endl;
}

void Segmenter::RECM(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
{
    // Embed cloud into grid cells

    for (auto& cell : polar_grid_)
    {
        cell.clear();
    }

    Point point_cache {};

    for (std::uint32_t i = 0; i < cloud.points.size(); ++i)
    {
        const auto& point = cloud.points[i];

        const float radius_m = std::sqrt(point.x * point.x + point.y * point.y);

        if (radius_m < config_.min_distance_m || radius_m > config_.max_distance_m)
        {
            continue;
        }

        float azimuth_rad = std::atan2(point.y, point.x);

        if (azimuth_rad < 0)
        {
            azimuth_rad += TWO_M_PIf;
        }

        const auto height_index = static_cast<std::int32_t>(point.ring);
        const auto width_index = static_cast<std::int32_t>(std::round((IMAGE_WIDTH - 1) * azimuth_rad / TWO_M_PIf));
        const auto image_index = toFlatImageIndex(height_index, width_index);

        if (isInvalidIndex(image_index))
        {
            // Outside of image bounds
            continue;
        }

        point_cache.height_index = height_index;
        point_cache.width_index = width_index;
        point_cache.x = point.x;
        point_cache.y = point.y;
        point_cache.z = point.z;
        point_cache.cloud_index = i;
        point_cache.label = Label::GROUND;

        const std::int32_t radial_index = radiusToIndex(radius_m);
        const std::int32_t azimuth_index = azimuthToIndex(azimuth_rad);
        const std::int32_t cell_index = toFlatGridIndex(azimuth_index, radial_index);

        polar_grid_[cell_index].push_back(point_cache);
        elevation_map_[cell_index] = std::min(elevation_map_[cell_index], point.z);
    }

    for (auto& cell : polar_grid_)
    {
        std::sort(cell.begin(), cell.end(), [](const auto& p1, const auto& p2) {
            const float dr1 = (p1.x * p1.x) + (p1.y * p1.y);
            const float dr2 = (p2.x * p2.x) + (p2.y * p2.y);
            return dr1 < dr2;
        });
    }

    // Elevation map refinement

    for (std::int32_t cell_index = 0; cell_index < elevation_map_.size() - 1; ++cell_index)
    {
        if (cell_index % grid_number_of_radial_rings_ == 0) // we are in the first cell
        {
            elevation_map_[cell_index] =
                -config_.sensor_height_m; // TODO: Update this value with something more appropriate
        }
        else
        {
            if ((cell_index + 1) % grid_number_of_radial_rings_ == 0)
            {
                continue;
            }

            if (isInvalidZ(elevation_map_[cell_index - 1]) || isInvalidZ(elevation_map_[cell_index + 1]))
            {
                continue;
            }

            // Smooth elevation values by averaging with neighbors if there is a significant difference
            if (std::fabs(elevation_map_[cell_index] - elevation_map_[cell_index - 1]) > 0.5F &&
                std::fabs(elevation_map_[cell_index] - elevation_map_[cell_index + 1]) > 0.5F)
            {
                elevation_map_[cell_index] = 0.5F * (elevation_map_[cell_index - 1] + elevation_map_[cell_index + 1]);
            }
        }
    }

    // RECM algorithm

    for (std::int32_t curr_cell_index = 0; curr_cell_index < elevation_map_.size(); ++curr_cell_index)
    {
        if (curr_cell_index % grid_number_of_radial_rings_ == 0) // we are in the first cell
        {
            // Do nothing - already assigned ground threshold
        }
        else
        {
            const std::int32_t prev_cell_index = curr_cell_index - 1;

            if (isInvalidZ(elevation_map_[prev_cell_index]))
            {
                continue;
            }

            if (isInvalidZ(elevation_map_[curr_cell_index]))
            {
                // Override
                elevation_map_[curr_cell_index] =
                    elevation_map_[prev_cell_index] +
                    config_.grid_radial_spacing_m * std::tan(config_.road_maximum_slope_m_per_m);
            }
            else
            {
                elevation_map_[curr_cell_index] =
                    std::min(elevation_map_[curr_cell_index],
                             elevation_map_[prev_cell_index] +
                                 config_.grid_radial_spacing_m * std::tan(config_.road_maximum_slope_m_per_m));
            }
        }
    }

    for (std::uint32_t cell_index = 0; cell_index <= maxGridIndex(); ++cell_index)
    {
        auto& cell = polar_grid_[cell_index];

        for (auto& point : cell)
        {
            if (point.z >= elevation_map_[cell_index] + config_.ground_height_threshold_m)
            {
                point.label = Label::OBSTACLE;
            }
        }
    }

    // Transfer points from the grid to the image

    for (const auto& cell : polar_grid_)
    {
        for (const auto& point : cell)
        {
            const std::int32_t height_index = point.height_index;
            const std::int32_t width_index = point.width_index;

            const auto depth_squarred = point.x * point.x + point.y * point.y + point.z * point.z;

            // Take the closest depth image point
            if (depth_squarred < depth_image_.at<float>(height_index, width_index))
            {
                switch (point.label)
                {
                    case Label::GROUND:
                        {
                            image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
                            depth_image_.at<float>(height_index, width_index) = depth_squarred;
                            cloud_mapping_indices_.at<std::int32_t>(height_index, width_index) = point.cloud_index;
                            break;
                        }
                    case Label::OBSTACLE:
                        {
                            image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
                            depth_image_.at<float>(height_index, width_index) = depth_squarred;
                            cloud_mapping_indices_.at<std::int32_t>(height_index, width_index) = point.cloud_index;
                            break;
                        }
                    case Label::UNKNOWN:
                        {
                            image_.at<cv::Vec3b>(height_index, width_index) = CV_UNKNOWN;
                            depth_image_.at<float>(height_index, width_index) = depth_squarred;
                            cloud_mapping_indices_.at<std::int32_t>(height_index, width_index) = point.cloud_index;
                            break;
                        }
                    default:
                        {
                            break;
                        }
                }
            }
        }
    }
}

void Segmenter::JCP(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
{
    // JCP Algorithm

    image_channels_.clear(); // TODO

    cv::split(image_, image_channels_);
    cv::dilate(image_channels_[2], image_channels_[2], kernel_);
    cv::merge(image_channels_, image_);

    index_queue_ = {}; // TODO

    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            if (image_.at<cv::Vec3b>(height_index, width_index) == CV_INTERSECTION_OR_UNKNOWN)
            {
                const auto image_index = toFlatImageIndex(height_index, width_index);

                if (isInvalidIndex(image_index))
                {
                    continue;
                }

                if (isValidIndex(cloud_mapping_indices_.at<std::int32_t>(height_index, width_index)))
                {
                    index_queue_.push({height_index, width_index});
                    image_.at<cv::Vec3b>(height_index, width_index) = CV_INTERSECTION;
                }
                else
                {
                    image_.at<cv::Vec3b>(height_index, width_index) = CV_UNKNOWN;
                }
            }
        }
    }

    cv::Mat display_image;
    cv::flip(image_, display_image, 0);
    cv::imshow("RECM with Low Confidence Points", display_image);
    cv::waitKey(1);

    mask_.fill(INVALID_INDEX);
    D_.fill(0);
    W_.fill(0);

    static constexpr Index neighbour_offsets[24] = {
        {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}, {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, {0, -2}, {0, -1},
        {0, 1},   {0, 2},   {1, -2}, {1, -1}, {1, 0},  {1, 1},   {1, 2},   {2, -2}, {2, -1}, {2, 0},  {2, 1},  {2, 2}};

    while (!index_queue_.empty())
    {
        const auto [height_index, width_index] = index_queue_.front();
        index_queue_.pop();
        const auto point_index = cloud_mapping_indices_.at<std::int32_t>(height_index, width_index);
        const auto& point_1 = cloud.points.at(point_index);

        float sum_of_coefficients = 0.0F;

        for (std::size_t i = 0; i < 24; ++i)
        {
            const auto [height_offset, width_offset] = neighbour_offsets[i];

            const auto neighbour_height_index = height_index + height_offset;
            const auto neighbour_width_index = width_index + width_offset;
            const auto neighbour_point_index =
                cloud_mapping_indices_.at<std::int32_t>(neighbour_height_index, neighbour_width_index);

            if (neighbour_height_index < 0 || neighbour_height_index >= IMAGE_HEIGHT || neighbour_width_index < 0 ||
                neighbour_width_index >= IMAGE_WIDTH || isInvalidIndex(neighbour_point_index))
            {
                D_[i] = 0;
                mask_[i] = INVALID_INDEX;
                continue;
            }

            const auto& point_2 = cloud.points.at(neighbour_point_index);
            const auto dist_xyz = std::sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) +
                                            (point_1.y - point_2.y) * (point_1.y - point_2.y) +
                                            (point_1.z - point_2.z) * (point_1.z - point_2.z));

            if (dist_xyz > config_.kernel_threshold_distance_m)
            {
                D_[i] = 0;
                mask_[i] = INVALID_INDEX;
                continue;
            }
            else
            {
                D_[i] = std::exp(-config_.amplification_factor * dist_xyz);
                sum_of_coefficients += D_[i];

                if (image_.at<cv::Vec3b>(neighbour_height_index, neighbour_width_index) == CV_GROUND)
                {
                    mask_[i] = 0;
                }
                else if (image_.at<cv::Vec3b>(neighbour_height_index, neighbour_width_index) == CV_OBSTACLE)
                {
                    mask_[i] = 1;
                }
                else
                {
                    mask_[i] = INVALID_INDEX;
                }
            }
        }

        if (std::fabs(sum_of_coefficients) > std::numeric_limits<float>::epsilon())
        {
            W_.noalias() = D_ / sum_of_coefficients;

            float weight_obstacle = 0.0F;
            float weight_ground = 0.0F;

            for (std::uint32_t i = 0; i < W_.size(); ++i)
            {
                if (mask_[i] == 0)
                {
                    weight_ground += W_[i];
                }
                else if (mask_[i] == 1)
                {
                    weight_obstacle += W_[i];
                }
            }

            if (weight_ground > weight_obstacle)
            {
                image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
            }
            else
            {
                image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
            }
        }
    }
}

void Segmenter::populateLabels(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels)
{
    labels.assign(cloud.size(), Label::UNKNOWN);

    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            const auto cloud_index = cloud_mapping_indices_.at<std::int32_t>(height_index, width_index);

            if (isInvalidIndex(cloud_index))
            {
                continue;
            }

            if (image_.at<cv::Vec3b>(height_index, width_index) == CV_GROUND)
            {
                labels[cloud_index] = Label::GROUND;
            }
            else if (image_.at<cv::Vec3b>(height_index, width_index) == CV_OBSTACLE)
            {
                labels[cloud_index] = Label::OBSTACLE;
            }
        }
    }
}

void Segmenter::constructElevationMap(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
{
    elevation_map_.at(0) = -config_.sensor_height_m;

    for (std::size_t point_index = 0; point_index < cloud.points.size(); ++point_index)
    {
        const auto& point = cloud.points[point_index];

        const auto radius_m = std::sqrt(point.x * point.x + point.y * point.y);

        if (radius_m < config_.min_distance_m || radius_m > config_.max_distance_m)
        {
            continue;
        }

        float azimuth_rad = std::atan2(point.y, point.x);
        azimuth_rad = (azimuth_rad >= 0 ? azimuth_rad : (TWO_M_PIf + azimuth_rad));

        const auto radial_index = radiusToIndex(radius_m);
        const auto azimuth_index = azimuthToIndex(azimuth_rad);

        // Update elevation map
        if (radial_index < grid_number_of_radial_rings_ && azimuth_index < grid_number_of_azimuth_slices_)
        {
            const std::int32_t height_index = point.ring;

            const auto width_index = static_cast<std::int32_t>(std::round((IMAGE_WIDTH - 1) * azimuth_rad / TWO_M_PIf));

            const auto image_index = toFlatImageIndex(height_index, width_index);

            if (isInvalidIndex(image_index))
            {
                continue;
            }

            const auto elevation_map_flattened_index = toFlatGridIndex(azimuth_index, radial_index);

            elevation_map_.at(elevation_map_flattened_index) =
                std::min(elevation_map_.at(elevation_map_flattened_index), point.z);
            image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
            grid_mapping_indices_.at<std::int32_t>(height_index, width_index) = elevation_map_flattened_index;
            cloud_indices_.at(static_cast<std::size_t>(image_index)) = point_index;
        }
    }

    // For each slice
    for (std::int32_t azimuth_index = 0; azimuth_index < grid_number_of_azimuth_slices_; ++azimuth_index)
    {
        // For each ring (radial partition) within the slice
        for (std::int32_t radial_index = 1; radial_index < grid_number_of_radial_rings_; ++radial_index)
        {
            const auto current_elevation_map_flattened_index = toFlatGridIndex(azimuth_index, radial_index);
            const auto current_z = elevation_map_.at(current_elevation_map_flattened_index);

            if (isInvalidZ(current_z))
            {
                // Current elevation cell does not contain a valid elevation value
                continue;
            }

            const std::int32_t prev_radial_index = radial_index - 1;
            const auto previous_elevation_map_flattened_index = toFlatGridIndex(azimuth_index, prev_radial_index);
            const auto previous_z = elevation_map_.at(previous_elevation_map_flattened_index);

            if (isInvalidZ(previous_z))
            {
                // Previous elevation cell does not contain a valid elevation value
                continue;
            }

            const auto gradient_between_cells = std::atan((current_z - previous_z) / config_.grid_radial_spacing_m);

            if (gradient_between_cells > config_.road_maximum_slope_m_per_m)
            {
                elevation_map_.at(current_elevation_map_flattened_index) =
                    previous_z + config_.grid_radial_spacing_m * std::tan(config_.road_maximum_slope_m_per_m);
            }
        }
    }
}

void Segmenter::performRECMSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
{
    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            const auto mapping_index = grid_mapping_indices_.at<std::int32_t>(height_index, width_index);

            if (isInvalidIndex(mapping_index))
            {
                continue;
            }

            const auto image_index = toFlatImageIndex(height_index, width_index);

            if (isInvalidIndex(image_index))
            {
                continue;
            }

            const auto point_index = cloud_indices_.at(image_index);

            if (isInvalidIndex(point_index))
            {
                continue;
            }

            const auto& point = cloud.points[point_index];
            const auto grid_index = grid_mapping_indices_.at<std::int32_t>(height_index, width_index);

            if (point.z >= elevation_map_[grid_index] + config_.ground_height_threshold_m)
            {
                image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
            }
        }
    }
}

void Segmenter::performJCPSegmentation(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
{
    image_channels_.clear();

    cv::split(image_, image_channels_);
    cv::dilate(image_channels_[2], image_channels_[2], kernel_);
    cv::merge(image_channels_, image_);

    index_queue_ = {}; // TODO

    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            if (image_.at<cv::Vec3b>(height_index, width_index) == CV_INTERSECTION_OR_UNKNOWN)
            {
                const auto image_index = toFlatImageIndex(height_index, width_index);

                if (isInvalidIndex(image_index))
                {
                    continue;
                }

                const auto point_index = cloud_indices_.at(image_index);

                if (isValidIndex(point_index))
                {
                    index_queue_.push({height_index, width_index});
                    image_.at<cv::Vec3b>(height_index, width_index) = CV_INTERSECTION;
                }
                else
                {
                    image_.at<cv::Vec3b>(height_index, width_index) = CV_UNKNOWN;
                }
            }
        }
    }

    cv::Mat display_image;
    cv::flip(image_, display_image, 0);
    cv::imshow("RECM with Low Confidence Points", display_image);
    cv::waitKey(1);

    mask_.fill(INVALID_INDEX);
    D_.fill(0);
    W_.fill(0);

    static constexpr Index neighbour_offsets[24] = {
        {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}, {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, {0, -2}, {0, -1},
        {0, 1},   {0, 2},   {1, -2}, {1, -1}, {1, 0},  {1, 1},   {1, 2},   {2, -2}, {2, -1}, {2, 0},  {2, 1},  {2, 2}};

    while (!index_queue_.empty())
    {
        const auto [height_index, width_index] = index_queue_.front();
        index_queue_.pop();
        const auto image_index = toFlatImageIndex(height_index, width_index);
        float sum_of_coefficients = 0.0F;

        for (std::size_t i = 0; i < 24; ++i)
        {
            const auto [height_offset, width_offset] = neighbour_offsets[i];
            const auto neighbour_height_index = height_index + height_offset;
            const auto neighbour_width_index = width_index + width_offset;
            const auto neighbour_image_index = toFlatImageIndex(neighbour_height_index, neighbour_width_index);

            if (isInvalidIndex(neighbour_image_index))
            {
                continue;
            }

            const auto neighbour_point_index = cloud_indices_.at(neighbour_image_index);

            if (neighbour_height_index < 0 || neighbour_height_index >= IMAGE_HEIGHT || neighbour_width_index < 0 ||
                neighbour_width_index >= IMAGE_WIDTH || isInvalidIndex(neighbour_point_index))
            {
                D_[i] = 0;
                mask_[i] = INVALID_INDEX;
                continue;
            }

            const auto point_index = cloud_indices_.at(image_index);

            const auto& point_1 = cloud.points.at(point_index);
            const auto& point_2 = cloud.points.at(neighbour_point_index);

            const auto distance_12 = std::sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) +
                                               (point_1.y - point_2.y) * (point_1.y - point_2.y) +
                                               (point_1.z - point_2.z) * (point_1.z - point_2.z));

            if (distance_12 > config_.kernel_threshold_distance_m)
            {
                D_[i] = 0;
            }
            else
            {
                D_[i] = std::exp(-config_.amplification_factor * distance_12);
                sum_of_coefficients += D_[i];
            }

            if (image_.at<cv::Vec3b>(neighbour_height_index, neighbour_width_index) == CV_GROUND)
            {
                mask_[i] = 0;
            }
            else if (image_.at<cv::Vec3b>(neighbour_height_index, neighbour_width_index) == CV_OBSTACLE)
            {
                mask_[i] = 1;
            }
            else
            {
                mask_[i] = 2;
            }
        }

        if (std::fabs(sum_of_coefficients) > std::numeric_limits<float>::epsilon())
        {
            W_.noalias() = D_ / sum_of_coefficients;

            float weight_obstacle = 0.0F;
            float weight_ground = 0.0F;

            for (std::uint32_t i = 0; i < D_.size(); ++i)
            {
                if (mask_[i] == 0)
                {
                    weight_ground += W_[i];
                }
                else if (mask_[i] == 1)
                {
                    weight_obstacle += W_[i];
                }
            }

            if (weight_ground > weight_obstacle)
            {
                image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
            }
            else
            {
                image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
            }
        }
    }
}

void Segmenter::extractSegmentationLabels(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels)
{
    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            const auto image_index = toFlatImageIndex(height_index, width_index);

            if (isInvalidIndex(image_index))
            {
                continue;
            }

            const auto point_index = cloud_indices_.at(image_index);

            if (isInvalidIndex(point_index))
            {
                continue;
            }

            if (image_.at<cv::Vec3b>(height_index, width_index) == CV_GROUND)
            {
                labels[point_index] = Label::GROUND;
            }
            else if (image_.at<cv::Vec3b>(height_index, width_index) == CV_OBSTACLE)
            {
                labels[point_index] = Label::OBSTACLE;
            }
            else
            {
                labels[point_index] = Label::UNKNOWN;
            }
        }
    }
}

} // namespace segmentation
