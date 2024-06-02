#include "segmenter.hpp"

namespace segmentation
{
Segmenter::Segmenter()
    : config_ {},
      image_ {cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3)},
      kernel_ {cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5))}
{
    grid_slice_resolution_rad_ = config_.grid_slice_resolution_deg * DEG_TO_RAD;
    grid_number_of_radial_rings_ = static_cast<std::int32_t>(config_.max_distance_m / config_.grid_radial_spacing_m);
    grid_number_of_azimuth_slices_ = static_cast<std::int32_t>(TWO_M_PIf / grid_slice_resolution_rad_);

    polar_grid_.resize(grid_number_of_radial_rings_ * grid_number_of_azimuth_slices_);
    for (auto& cell : polar_grid_)
    {
        cell.reserve(150); // TODO: How to determine this number?
    }
    cell_z_values_.reserve(1000); // TODO: How to determine this number?

    elevation_map_.resize(grid_number_of_azimuth_slices_ * grid_number_of_radial_rings_, INVALID_Z);

    cloud_mapping_indices_.resize(IMAGE_HEIGHT * IMAGE_WIDTH, INVALID_INDEX);
    depth_image_.resize(IMAGE_HEIGHT * IMAGE_WIDTH, INVALID_DEPTH_M);
    index_queue_.reserve(MAX_CLOUD_SIZE);

    image_channels_.resize(3);
    for (auto& channel : image_channels_)
    {
        channel.create(IMAGE_HEIGHT, IMAGE_WIDTH, image_.depth());
    }

    std::cerr << config_ << std::endl;
}

void Segmenter::config(const Configuration& config)
{
    config_ = config;

    grid_slice_resolution_rad_ = config_.grid_slice_resolution_deg * DEG_TO_RAD;
    grid_number_of_radial_rings_ = static_cast<std::int32_t>(config_.max_distance_m / config_.grid_radial_spacing_m);
    grid_number_of_azimuth_slices_ = static_cast<std::int32_t>(TWO_M_PIf / grid_slice_resolution_rad_);

    polar_grid_.resize(grid_number_of_radial_rings_ * grid_number_of_azimuth_slices_);
    for (auto& cell : polar_grid_)
    {
        cell.reserve(150); // TODO: How to determine this number?
    }

    elevation_map_.assign(grid_number_of_azimuth_slices_ * grid_number_of_radial_rings_, INVALID_Z);

    std::cerr << config_ << std::endl;
}

void Segmenter::resetValues()
{
    image_.setTo(cv::Scalar(0));

    elevation_map_.assign(elevation_map_.size(), INVALID_Z);
    cloud_mapping_indices_.assign(cloud_mapping_indices_.size(), INVALID_INDEX);
    depth_image_.assign(depth_image_.size(), INVALID_DEPTH_M);

    for (auto& cell : polar_grid_)
    {
        cell.clear();
    }
}

void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels)
{
    labels.assign(cloud.points.size(), Label::UNKNOWN);

    resetValues();

    RECM(cloud);

    JCP(cloud);

    populateLabels(cloud, labels);
}

void Segmenter::RECM(const pcl::PointCloud<pcl::PointXYZIR>& cloud)
{
    // Embed cloud into grid cells

    for (auto& cell : polar_grid_)
    {
        cell.clear();
    }

    const float z_min_adjusted = -config_.sensor_height_m + config_.z_min_m;
    const float z_max_adjusted = -config_.sensor_height_m + config_.z_max_m;

    for (std::int32_t cloud_index = 0; cloud_index < cloud.points.size(); ++cloud_index)
    {
        const auto& point = cloud.points[cloud_index];

        if (point.z < z_min_adjusted || point.z > z_max_adjusted)
        {
            continue;
        }

        const float radius_m = std::sqrt(point.x * point.x + point.y * point.y);

        if (radius_m > config_.max_distance_m)
        {
            continue;
        }

        float azimuth_rad = std::atan2(point.y, point.x);
        azimuth_rad = (azimuth_rad < 0) ? (azimuth_rad + TWO_M_PIf) : azimuth_rad;

        const auto radial_index = static_cast<std::int32_t>(radius_m / config_.grid_radial_spacing_m);
        const auto azimuth_index = static_cast<std::int32_t>(azimuth_rad / grid_slice_resolution_rad_);
        const std::int32_t cell_index = azimuth_index * grid_number_of_radial_rings_ + radial_index;

        if (radius_m < config_.min_distance_m)
        {
            elevation_map_.at(cell_index) =
                std::min(-config_.sensor_height_m, std::min(elevation_map_.at(cell_index), point.z));
        }
        else
        {
            // elevation_map_.at(cell_index) = std::min(elevation_map_.at(cell_index), point.z);    // SEE BELOW

            const auto height_index = point.ring;
            const auto width_index =
                static_cast<std::uint16_t>(std::round((IMAGE_WIDTH - 1) * azimuth_rad / TWO_M_PIf));
            const auto image_index = toFlatImageIndex(height_index, width_index);

            if (isInvalidIndex(image_index))
            {
                // Outside of image bounds
                continue;
            }

            polar_grid_.at(cell_index)
                .push_back({point.x, point.y, point.z, Label::GROUND, height_index, width_index, cloud_index});
        }
    }

    for (auto& cell : polar_grid_)
    {
        std::sort(cell.begin(), cell.end(), [](const auto& p1, const auto& p2) noexcept {
            const float dr1 = (p1.x * p1.x) + (p1.y * p1.y);
            const float dr2 = (p2.x * p2.x) + (p2.y * p2.y);
            return dr1 < dr2;
        });
    }

    // Correct erroneous elevation map values due to outlier points
    for (std::int32_t azimuth_index = 0; azimuth_index < grid_number_of_azimuth_slices_; ++azimuth_index)
    {
        const auto azimuth_index_offset = azimuth_index * grid_number_of_radial_rings_;
        elevation_map_[azimuth_index_offset] = -config_.sensor_height_m; // For zeroth cell set to sensor offset

        for (std::int32_t radial_index = 1; radial_index < grid_number_of_radial_rings_; ++radial_index)
        {
            const auto cell_index = azimuth_index_offset + radial_index;
            const auto& cell = polar_grid_[cell_index];

            if (cell.empty())
            {
                continue;
            }

            cell_z_values_.clear();
            for (const auto& point : cell)
            {
                cell_z_values_.push_back(point.z);
            }

            // Sort in descending order
            std::sort(cell_z_values_.begin(), cell_z_values_.end());

            // Find z min accounting for outliers
            const std::int32_t median_index = cell_z_values_.size() / 2;
            float z_min = cell_z_values_[median_index];

            elevation_map_[cell_index] = cell_z_values_[0]; // take the smallest value

            for (std::int32_t i = median_index; i >= 1; --i)
            {
                z_min = cell_z_values_[i];
                if (z_min - cell_z_values_[i - 1] > 0.5F) // Hard-coded threshold
                {
                    elevation_map_[cell_index] = z_min; // take min value
                    break;
                }
            }
        }
    }

    // RECM algorithm

    const auto max_positive_height_difference_between_adjacent_grid_cells =
        config_.grid_radial_spacing_m * std::tan(config_.road_maximum_slope_m_per_m);

    for (std::int32_t azimuth_index = 0; azimuth_index < grid_number_of_azimuth_slices_; ++azimuth_index)
    {
        const auto azimuth_index_offset = azimuth_index * grid_number_of_radial_rings_;

        for (std::int32_t radial_index = 1; radial_index < grid_number_of_radial_rings_; ++radial_index)
        {
            const auto cell_index = azimuth_index_offset + radial_index;

            const auto curr_z = elevation_map_[cell_index];
            const auto prev_z = elevation_map_[cell_index - 1];

            if (isValidZ(prev_z))
            {
                elevation_map_[cell_index] =
                    std::min(curr_z, prev_z + max_positive_height_difference_between_adjacent_grid_cells);
            }
        }
    }

    for (std::uint32_t cell_index = 0; cell_index < polar_grid_.size(); ++cell_index)
    {
        auto& cell = polar_grid_[cell_index];
        const auto curr_z = elevation_map_[cell_index];

        for (auto& point : cell)
        {
            if (point.z >= curr_z + config_.ground_height_threshold_m)
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
            const std::int32_t image_index = toFlatImageIndex(height_index, width_index);

            if (isInvalidIndex(image_index))
            {
                continue;
            }

            const auto depth_squarred = (point.x * point.x) + (point.y * point.y);

            // Take the closest depth image point
            if (depth_squarred < depth_image_[image_index])
            {
                switch (point.label)
                {
                    case Label::GROUND:
                        {
                            image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
                            depth_image_[image_index] = depth_squarred;
                            cloud_mapping_indices_[image_index] = point.cloud_index;
                            break;
                        }
                    case Label::OBSTACLE:
                        {
                            image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
                            depth_image_[image_index] = depth_squarred;
                            cloud_mapping_indices_[image_index] = point.cloud_index;
                            break;
                        }
                    case Label::UNKNOWN:
                        {
                            image_.at<cv::Vec3b>(height_index, width_index) = CV_UNKNOWN;
                            depth_image_[image_index] = depth_squarred;
                            cloud_mapping_indices_[image_index] = point.cloud_index;
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

    cv::split(image_, image_channels_);
    cv::dilate(image_channels_[2], image_channels_[2], kernel_);
    cv::merge(image_channels_, image_);

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

                if (isValidIndex(cloud_mapping_indices_[image_index]))
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

    static cv::Mat display_image;
    cv::flip(image_, display_image, 0);
    cv::imshow("RECM with Low Confidence Points", display_image);
    cv::waitKey(1);

    mask_.fill(INVALID_INDEX);
    unnormalized_weight_matrix_.fill(0);
    weight_matrix_.fill(0);

    static constexpr Index neighbour_offsets[24] = {
        {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}, {-1, -2}, {-1, -1}, {-1, 0}, {-1, 1}, {-1, 2}, {0, -2}, {0, -1},
        {0, 1},   {0, 2},   {1, -2}, {1, -1}, {1, 0},  {1, 1},   {1, 2},   {2, -2}, {2, -1}, {2, 0},  {2, 1},  {2, 2}};

    while (!index_queue_.empty())
    {
        const auto [height_index, width_index] = index_queue_.front();
        index_queue_.pop();
        const auto point_index = cloud_mapping_indices_[toFlatImageIndex(height_index, width_index)];
        const auto& point_1 = cloud.points.at(point_index);

        float sum_of_coefficients = 0.0F;

        for (std::size_t i = 0; i < 24; ++i)
        {
            const auto [height_offset, width_offset] = neighbour_offsets[i];

            const auto neighbour_height_index = height_index + height_offset;
            const auto neighbour_width_index = width_index + width_offset;
            const auto neighbour_point_index =
                cloud_mapping_indices_[toFlatImageIndex(neighbour_height_index, neighbour_width_index)];

            if (neighbour_height_index < 0 || neighbour_height_index >= IMAGE_HEIGHT || neighbour_width_index < 0 ||
                neighbour_width_index >= IMAGE_WIDTH || isInvalidIndex(neighbour_point_index))
            {
                unnormalized_weight_matrix_[i] = 0;
                mask_[i] = INVALID_INDEX;
                continue;
            }

            const auto& point_2 = cloud.points.at(neighbour_point_index);
            const auto dist_xyz = std::sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) +
                                            (point_1.y - point_2.y) * (point_1.y - point_2.y) +
                                            (point_1.z - point_2.z) * (point_1.z - point_2.z));

            if (dist_xyz > config_.kernel_threshold_distance_m)
            {
                unnormalized_weight_matrix_[i] = 0;
                mask_[i] = INVALID_INDEX;
                continue;
            }
            else
            {
                unnormalized_weight_matrix_[i] = std::exp(-config_.amplification_factor * dist_xyz);
                sum_of_coefficients += unnormalized_weight_matrix_[i];

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
            weight_matrix_.noalias() = unnormalized_weight_matrix_ / sum_of_coefficients;

            float weight_obstacle = 0.0F;
            float weight_ground = 0.0F;

            for (std::uint32_t i = 0; i < weight_matrix_.size(); ++i)
            {
                if (mask_[i] == 0)
                {
                    weight_ground += weight_matrix_[i];
                }
                else if (mask_[i] == 1)
                {
                    weight_obstacle += weight_matrix_[i];
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
    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            const auto image_index = toFlatImageIndex(height_index, width_index);

            if (isInvalidIndex(image_index))
            {
                continue;
            }

            const auto cloud_index = cloud_mapping_indices_[image_index];

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

} // namespace segmentation
