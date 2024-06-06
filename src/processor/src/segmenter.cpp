#include "segmenter.hpp"

#include <random>

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
        cell.reserve(400); // TODO: How to determine this number?
    }
    cell_z_values_.reserve(1000); // TODO: How to determine this number?

    elevation_map_.resize(grid_number_of_azimuth_slices_ * grid_number_of_radial_rings_, INVALID_Z);

    cloud_mapping_indices_.resize(IMAGE_HEIGHT * IMAGE_WIDTH, INVALID_INDEX);
    depth_image_.resize(IMAGE_HEIGHT * IMAGE_WIDTH, INVALID_DEPTH_M);

    ransac_points_.reserve(MAX_CLOUD_SIZE);

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
        cell.reserve(400); // TODO: How to determine this number?
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

// Taken from
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
// maximum relative error about 3.6e-5
static inline float atan2Approx(const float y, const float x) noexcept
{
    const float ax = std::fabs(x);
    const float ay = std::fabs(y);
    const float mx = std::max(ay, ax);
    const float mn = std::min(ay, ax);
    const float a = mn / mx;
    /* Minimax polynomial approximation to atan(a) on [0,1] */
    const float s = a * a;
    const float c = s * a;
    const float q = s * s;
    float r = 0.024840285F * q + 0.18681418F;
    const float t = -0.094097948F * q - 0.33213072F;
    r = r * s + t;
    r = r * c + a;
    /* Map to full circle */
    if (ay > ax)
    {
        r = 1.57079637F - r;
    }
    if (x < 0)
    {
        r = 3.14159274F - r;
    }
    if (y < 0)
    {
        r = -r;
    }
    return r;
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

        float azimuth_rad = atan2Approx(point.y, point.x);
        azimuth_rad = (azimuth_rad < 0) ? (azimuth_rad + TWO_M_PIf) : azimuth_rad;

        const auto radial_index = static_cast<std::int32_t>(radius_m / config_.grid_radial_spacing_m);

        if (radial_index >= grid_number_of_radial_rings_)
        {
            continue;
        }

        const auto azimuth_index = std::min(static_cast<std::int32_t>(azimuth_rad / grid_slice_resolution_rad_),
                                            grid_number_of_azimuth_slices_ - 1);

        const auto cell_index = static_cast<std::uint32_t>(azimuth_index * grid_number_of_radial_rings_ + radial_index);

        if (radius_m < config_.min_distance_m)
        {
            elevation_map_[cell_index] =
                std::min(-config_.sensor_height_m, std::min(elevation_map_[cell_index], point.z));
        }
        else
        {
            // Calculated below
            elevation_map_[cell_index] = std::min(elevation_map_[cell_index], point.z);

            const std::uint16_t height_index = point.ring;

            if (height_index >= IMAGE_HEIGHT)
            {
                continue;
            }

            const auto width_index = static_cast<std::uint16_t>((IMAGE_WIDTH - 1) * azimuth_rad / TWO_M_PIf);

            polar_grid_[cell_index].push_back(
                {point.x, point.y, point.z, Label::GROUND, height_index, width_index, cloud_index});
        }
    }

    // RECM algorithm
    const auto max_positive_height_difference_between_adjacent_grid_cells =
        std::min(config_.grid_radial_spacing_m * std::tan(config_.road_maximum_slope_m_per_m),
                 config_.ground_height_threshold_m - std::numeric_limits<float>::epsilon());

    // Correct erroneous elevation map values due to outlier points

    for (std::int32_t azimuth_index = 0; azimuth_index < grid_number_of_azimuth_slices_; ++azimuth_index)
    {
        const auto azimuth_index_offset = azimuth_index * grid_number_of_radial_rings_;

        elevation_map_[azimuth_index_offset] =
            -config_.sensor_height_m + config_.ground_height_threshold_m; // For zeroth cell set to sensor offset
        float prev_z_min = elevation_map_[azimuth_index_offset];

        for (std::int32_t radial_index = 1; radial_index < grid_number_of_radial_rings_; ++radial_index)
        {
            const auto cell_index = azimuth_index_offset + radial_index;
            const auto& cell = polar_grid_[cell_index];

            if (cell.empty())
            {
                elevation_map_[cell_index] = prev_z_min + max_positive_height_difference_between_adjacent_grid_cells;
                prev_z_min = elevation_map_[cell_index];

                continue;
            }

            cell_z_values_.clear();
            for (const auto& point : cell)
            {
                cell_z_values_.push_back(point.z);
            }

            // Sort in increasing order
            std::sort(cell_z_values_.begin(), cell_z_values_.end());

            // Find z min accounting for outliers
            float curr_z_min = cell_z_values_[0];

            for (std::int32_t i = cell_z_values_.size() / 2; i >= 1; --i)
            {
                if (cell_z_values_[i] - cell_z_values_[i - 1] > 0.5F) // This accounts for retroreflections
                {
                    curr_z_min = cell_z_values_[i];
                    break;
                }
            }

            elevation_map_[cell_index] =
                std::min(curr_z_min, prev_z_min + max_positive_height_difference_between_adjacent_grid_cells);
            prev_z_min = elevation_map_[cell_index];
        }
    }

    // Classify obstacle points

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

    // Correct false positives close to the vehicle

    correctCloseRangeFalsePositivesRANSAC();

    // Transfer points from the grid to the image

    for (const auto& cell : polar_grid_)
    {
        for (const auto& point : cell)
        {
            const std::int32_t height_index = point.height_index;
            const std::int32_t width_index = point.width_index;
            const std::int32_t image_index = height_index * IMAGE_WIDTH + width_index;

            const auto depth_squarred = (point.x * point.x) + (point.y * point.y);

            // Take the closest depth image point
            if (depth_squarred < depth_image_[image_index])
            {
                if (point.label == Label::GROUND)
                {
                    image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
                    depth_image_[image_index] = depth_squarred;
                    cloud_mapping_indices_[image_index] = point.cloud_index;
                }
                else if (point.label == Label::OBSTACLE)
                {
                    image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
                    depth_image_[image_index] = depth_squarred;
                    cloud_mapping_indices_[image_index] = point.cloud_index;
                }
            }
        }
    }
}

void Segmenter::correctCloseRangeFalsePositivesRANSAC()
{
    static constexpr std::int32_t MAX_RADIAL_BINS = 4;
    static constexpr std::uint32_t NUMBER_OF_ITERATIONS = 60;

    ransac_points_.clear();

    for (std::int32_t azimuth_index = 0; azimuth_index < grid_number_of_azimuth_slices_; ++azimuth_index)
    {
        const auto azimuth_index_offset = azimuth_index * grid_number_of_radial_rings_;

        for (std::int32_t radial_index = 0; radial_index < MAX_RADIAL_BINS; ++radial_index)
        {
            const auto cell_index = azimuth_index_offset + radial_index;

            const auto& cell = polar_grid_[cell_index];

            const float z_min = elevation_map_[cell_index];

            for (const auto& point : cell)
            {
                if (std::fabs(z_min - point.z) <
                    2.0F * config_.ground_height_threshold_m) // Add only higher confidence points
                {
                    ransac_points_.push_back({point.x, point.y, point.z});
                }
            }
        }
    }

    // Set pivot
    RansacPoint p1 {0.0, 0.0, -config_.sensor_height_m};
    const float max_plane_cosine_angle = std::cos(std::tan(config_.road_maximum_slope_m_per_m));

    // Plane coefficients
    float a = 0.0F;
    float b = 0.0F;
    float c = 1.0F;
    float d = 0.0F;

    // For index generation
    std::mt19937 gen {42};
    std::uniform_int_distribution<std::uint32_t> dist {0, static_cast<std::uint32_t>(ransac_points_.size()) - 1U};

    // RANSAC
    std::uint32_t best_inlier_count = 0U;

    for (std::uint32_t iteration = 0; iteration < NUMBER_OF_ITERATIONS; ++iteration)
    {
        // Choose 2 random points
        const std::uint32_t p2_index = dist(gen);
        const auto& p2 = ransac_points_[p2_index];

        std::uint32_t p3_index = dist(gen);
        while (p3_index == p2_index)
        {
            p3_index = dist(gen);
        }
        const auto& p3 = ransac_points_[p3_index];

        // Calculate a plane defined by three points
        float normal_x = ((p2.y - p1.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y));
        float normal_y = ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z));
        float normal_z = ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.x));

        // Calculate normalization
        const float denominator = std::sqrt((normal_x * normal_x) + (normal_y * normal_y) + (normal_z * normal_z));

        // Check that denominator is not too small
        if (denominator < 1.0e-4F)
        {
            continue;
        }
        const float normalization = 1.0F / denominator;

        // Normalize plane coefficients
        normal_z *= normalization;

        // Constrain plane
        if (std::fabs(normal_z) < max_plane_cosine_angle)
        {
            continue;
        }

        normal_x *= normalization;
        normal_y *= normalization;

        const float plane_d = (normal_x * p1.x) + (normal_y * p1.y) + (normal_z * p1.z);

        // Count inlier points
        std::uint32_t inlier_count = 0U;
        for (const auto& point : ransac_points_)
        {
            const float orthogonal_distance =
                std::fabs((normal_x * point.x) + (normal_y * point.y) + (normal_z * point.z) - plane_d);

            if (orthogonal_distance < config_.ground_height_threshold_m)
            {
                ++inlier_count;
            }
        }

        // If the plane is best so far, update the plane coefficients
        if (inlier_count > best_inlier_count)
        {
            best_inlier_count = inlier_count;

            a = normal_x;
            b = normal_y;
            c = normal_z;
            d = plane_d;
        }
    }

    if (best_inlier_count > 0)
    {
        if (c < 0)
        {
            a = -a;
            b = -b;
            c = -c;
            d = -d;
        }

        // Decide which points are GROUND and which points are NON-GROUND
        for (std::int32_t azimuth_index = 0; azimuth_index < grid_number_of_azimuth_slices_; ++azimuth_index)
        {
            const auto azimuth_index_offset = azimuth_index * grid_number_of_radial_rings_;

            for (std::int32_t radial_index = 0; radial_index < MAX_RADIAL_BINS; ++radial_index)
            {
                const auto cell_index = azimuth_index_offset + radial_index;
                auto& cell = polar_grid_[cell_index];

                for (auto& point : cell)
                {
                    const float signed_orthogonal_distance = (a * point.x) + (b * point.y) + (c * point.z) - d;

                    if (signed_orthogonal_distance < config_.ground_height_threshold_m)
                    {
                        point.label = Label::GROUND;
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
    cv::dilate(image_channels_[2], image_channels_[2], kernel_); // dilate the red channel
    cv::merge(image_channels_, image_);

    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        const std::int32_t height_index_with_offset = height_index * IMAGE_WIDTH;

        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            auto& image_pixel = image_.at<cv::Vec3b>(height_index, width_index);

            if (image_pixel == CV_INTERSECTION_OR_UNKNOWN)
            {
                const std::int32_t image_index = height_index_with_offset + width_index;

                if (isValidIndex(cloud_mapping_indices_[image_index]))
                {
                    index_queue_.push({height_index, width_index});
                    image_pixel = CV_INTERSECTION;
                }
                else
                {
                    image_pixel = CV_UNKNOWN;
                }
            }
        }
    }

    if (config_.display_recm_with_low_confidence_points)
    {
        cv::flip(image_, display_image_, 0);
        cv::imshow("RECM with Low Confidence Points", display_image_);
        cv::waitKey(1);
    }

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
        const std::int32_t image_index = height_index * IMAGE_WIDTH + width_index;
        const auto point_index = cloud_mapping_indices_[image_index];
        const auto& point_1 = cloud.points[point_index];

        float sum_of_coefficients = 0.0F;

        for (std::size_t i = 0; i < 24; ++i)
        {
            const auto [height_offset, width_offset] = neighbour_offsets[i];

            const std::int32_t neighbour_height_index = height_index + height_offset;
            const std::int32_t neighbour_width_index = width_index + width_offset;

            if (neighbour_height_index < 0 || neighbour_height_index >= IMAGE_HEIGHT || neighbour_width_index < 0 ||
                neighbour_width_index >= IMAGE_WIDTH)
            {
                continue;
            }

            const std::int32_t neighbour_image_index = neighbour_height_index * IMAGE_WIDTH + neighbour_width_index;
            const std::int32_t neighbour_point_index = cloud_mapping_indices_[neighbour_image_index];

            if (isInvalidIndex(neighbour_point_index))
            {
                unnormalized_weight_matrix_[i] = 0.0F;
                mask_[i] = INVALID_INDEX;
                continue;
            }

            const auto& point_2 = cloud.points[neighbour_point_index];
            const auto dist_xyz = std::sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) +
                                            (point_1.y - point_2.y) * (point_1.y - point_2.y) +
                                            (point_1.z - point_2.z) * (point_1.z - point_2.z));

            if (dist_xyz > config_.kernel_threshold_distance_m)
            {
                unnormalized_weight_matrix_[i] = 0.0F;
                mask_[i] = INVALID_INDEX;
                continue;
            }
            else
            {
                unnormalized_weight_matrix_[i] = std::exp(-config_.amplification_factor * dist_xyz);
                sum_of_coefficients += unnormalized_weight_matrix_[i];

                const auto& image_pixel = image_.at<cv::Vec3b>(neighbour_height_index, neighbour_width_index);

                if (image_pixel == CV_GROUND)
                {
                    mask_[i] = 0;
                }
                else if (image_pixel == CV_OBSTACLE)
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

            for (std::size_t i = 0; i < weight_matrix_.size(); ++i)
            {
                if (mask_[i] == 1)
                {
                    weight_obstacle += weight_matrix_[i];
                }
                else if (mask_[i] == 0)
                {
                    weight_ground += weight_matrix_[i];
                }
            }

            if (weight_obstacle > weight_ground)
            {
                image_.at<cv::Vec3b>(height_index, width_index) = CV_OBSTACLE;
            }
            else
            {
                image_.at<cv::Vec3b>(height_index, width_index) = CV_GROUND;
            }
        }
    }
}

void Segmenter::populateLabels(const pcl::PointCloud<pcl::PointXYZIR>& cloud, std::vector<Label>& labels)
{
    for (std::int32_t height_index = 0; height_index < IMAGE_HEIGHT; ++height_index)
    {
        const std::int32_t height_index_with_offset = height_index * IMAGE_WIDTH;

        for (std::int32_t width_index = 0; width_index < IMAGE_WIDTH; ++width_index)
        {
            const std::int32_t image_index = height_index_with_offset + width_index;

            const auto cloud_index = cloud_mapping_indices_[image_index];

            if (isInvalidIndex(cloud_index))
            {
                continue;
            }

            const auto& image_pixel = image_.at<cv::Vec3b>(height_index, width_index);

            if (image_pixel == CV_GROUND)
            {
                labels[cloud_index] = Label::GROUND;
            }
            else if (image_pixel == CV_OBSTACLE)
            {
                labels[cloud_index] = Label::OBSTACLE;
            }
        }
    }
}

} // namespace segmentation
