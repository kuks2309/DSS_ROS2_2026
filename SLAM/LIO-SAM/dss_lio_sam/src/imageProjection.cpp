/**
 * LIO-SAM: Image Projection Node
 *
 * This node performs:
 * 1. Project LiDAR point cloud to range image
 * 2. Extract feature points (edge and planar)
 * 3. De-skew point cloud using IMU data
 */

#include "dss_lio_sam/utility.hpp"
#include <opencv2/opencv.hpp>

class ImageProjection : public rclcpp::Node
{
public:
    ImageProjection()
    : Node("image_projection")
    {
        // Declare parameters
        this->declare_parameter<std::string>("pointCloudTopic", "/velodyne_points");
        this->declare_parameter<std::string>("imuTopic", "/imu/data");
        this->declare_parameter<std::string>("odomTopic", "odometry/imu");
        this->declare_parameter<std::string>("lidarFrame", "velodyne");

        this->declare_parameter<std::string>("sensor", "velodyne");
        this->declare_parameter<int>("N_SCAN", 16);
        this->declare_parameter<int>("Horizon_SCAN", 1800);
        this->declare_parameter<int>("downsampleRate", 1);
        this->declare_parameter<double>("lidarMinRange", 1.0);
        this->declare_parameter<double>("lidarMaxRange", 100.0);

        this->declare_parameter<double>("edgeThreshold", 1.0);
        this->declare_parameter<double>("surfThreshold", 0.1);
        this->declare_parameter<int>("edgeFeatureMinValidNum", 10);
        this->declare_parameter<int>("surfFeatureMinValidNum", 100);

        this->declare_parameter<std::vector<double>>("extrinsicRot", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        this->declare_parameter<std::vector<double>>("extrinsicRPY", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("extrinsicTrans", std::vector<double>{0.0, 0.0, 0.0});

        // Get parameters
        point_cloud_topic_ = this->get_parameter("pointCloudTopic").as_string();
        imu_topic_ = this->get_parameter("imuTopic").as_string();
        odom_topic_ = this->get_parameter("odomTopic").as_string();
        lidar_frame_ = this->get_parameter("lidarFrame").as_string();

        sensor_type_ = this->get_parameter("sensor").as_string();
        n_scan_ = this->get_parameter("N_SCAN").as_int();
        horizon_scan_ = this->get_parameter("Horizon_SCAN").as_int();
        downsample_rate_ = this->get_parameter("downsampleRate").as_int();
        lidar_min_range_ = this->get_parameter("lidarMinRange").as_double();
        lidar_max_range_ = this->get_parameter("lidarMaxRange").as_double();

        edge_threshold_ = this->get_parameter("edgeThreshold").as_double();
        surf_threshold_ = this->get_parameter("surfThreshold").as_double();
        edge_feature_min_valid_num_ = this->get_parameter("edgeFeatureMinValidNum").as_int();
        surf_feature_min_valid_num_ = this->get_parameter("surfFeatureMinValidNum").as_int();

        auto ext_rot = this->get_parameter("extrinsicRot").as_double_array();
        auto ext_rpy = this->get_parameter("extrinsicRPY").as_double_array();
        auto ext_trans = this->get_parameter("extrinsicTrans").as_double_array();

        ext_rot_ << ext_rot[0], ext_rot[1], ext_rot[2],
                    ext_rot[3], ext_rot[4], ext_rot[5],
                    ext_rot[6], ext_rot[7], ext_rot[8];
        ext_rpy_ << ext_rpy[0], ext_rpy[1], ext_rpy[2];
        ext_trans_ << ext_trans[0], ext_trans[1], ext_trans[2];

        // Initialize point clouds
        allocateMemory();

        // Create subscribers
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ImageProjection::cloudCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ImageProjection::imuCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::QoS(10),
            std::bind(&ImageProjection::odometryCallback, this, std::placeholders::_1));

        // Create publishers
        cloud_info_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/deskew/cloud_info", 1);
        extracted_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/deskew/cloud_deskewed", 1);
        corner_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/feature/cloud_corner", 1);
        surface_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/feature/cloud_surface", 1);

        RCLCPP_INFO(this->get_logger(), "Image Projection node initialized");
        RCLCPP_INFO(this->get_logger(), "  Sensor: %s, N_SCAN: %d, Horizon_SCAN: %d",
                    sensor_type_.c_str(), n_scan_, horizon_scan_);
    }

private:
    void allocateMemory()
    {
        laser_cloud_in_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        laser_cloud_in_ring_.reset(new pcl::PointCloud<PointXYZIRT>());
        full_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        extracted_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        corner_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        surface_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        full_cloud_->points.resize(n_scan_ * horizon_scan_);
        range_mat_ = cv::Mat(n_scan_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));

        cloud_curvature_.resize(n_scan_ * horizon_scan_);
        cloud_neighbor_picked_.resize(n_scan_ * horizon_scan_);
        cloud_label_.resize(n_scan_ * horizon_scan_);
        cloud_smoothness_.resize(n_scan_ * horizon_scan_);
        cloud_range_.resize(n_scan_ * horizon_scan_);

        resetParameters();
    }

    void resetParameters()
    {
        laser_cloud_in_->clear();
        laser_cloud_in_ring_->clear();
        extracted_cloud_->clear();
        corner_cloud_->clear();
        surface_cloud_->clear();

        range_mat_ = cv::Mat(n_scan_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));

        std::fill(full_cloud_->points.begin(), full_cloud_->points.end(), pcl::PointXYZI());
        std::fill(cloud_curvature_.begin(), cloud_curvature_.end(), 0);
        std::fill(cloud_neighbor_picked_.begin(), cloud_neighbor_picked_.end(), 0);
        std::fill(cloud_label_.begin(), cloud_label_.end(), 0);
        std::fill(cloud_range_.begin(), cloud_range_.end(), 0);

        imu_pointer_cur_ = 0;
        first_point_flag_ = true;
        odom_deskew_flag_ = false;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu imu = *msg;

        // Transform IMU data to lidar frame
        Eigen::Vector3d acc(imu.linear_acceleration.x,
                           imu.linear_acceleration.y,
                           imu.linear_acceleration.z);
        acc = ext_rot_ * acc;
        imu.linear_acceleration.x = acc.x();
        imu.linear_acceleration.y = acc.y();
        imu.linear_acceleration.z = acc.z();

        Eigen::Vector3d gyr(imu.angular_velocity.x,
                           imu.angular_velocity.y,
                           imu.angular_velocity.z);
        gyr = ext_rot_ * gyr;
        imu.angular_velocity.x = gyr.x();
        imu.angular_velocity.y = gyr.y();
        imu.angular_velocity.z = gyr.z();

        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_queue_.push_back(imu);
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_queue_.push_back(*msg);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Check if message is valid
        if (!cachePointCloud(msg)) {
            return;
        }

        // Get IMU data for deskewing
        if (!deskewInfo()) {
            return;
        }

        // Project point cloud to range image
        projectPointCloud();

        // Extract features
        cloudExtraction();

        // Calculate smoothness
        calculateSmoothness();

        // Mark occluded points
        markOccludedPoints();

        // Extract features
        extractFeatures();

        // Publish clouds
        publishClouds();

        // Reset
        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
    {
        // Check if point cloud has ring and time fields
        bool has_ring = false;
        bool has_time = false;
        for (const auto& field : msg->fields) {
            if (field.name == "ring") has_ring = true;
            if (field.name == "time" || field.name == "t") has_time = true;
        }

        // Get timestamp
        cloud_header_ = msg->header;
        time_scan_cur_ = rclcpp::Time(cloud_header_.stamp).seconds();

        if (has_ring && has_time) {
            // Point cloud has ring and time fields - use directly
            pcl::fromROSMsg(*msg, *laser_cloud_in_ring_);

            if (laser_cloud_in_ring_->empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Empty point cloud received");
                return false;
            }

            if (laser_cloud_in_ring_->points.back().time > 0) {
                time_scan_end_ = time_scan_cur_ + laser_cloud_in_ring_->points.back().time;
            } else {
                time_scan_end_ = time_scan_cur_;
            }
        } else {
            // Standard PointXYZI - compute ring and time from geometry
            pcl::fromROSMsg(*msg, *laser_cloud_in_);

            if (laser_cloud_in_->empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Empty point cloud received");
                return false;
            }

            // Convert to PointXYZIRT with computed ring and time
            laser_cloud_in_ring_->clear();
            laser_cloud_in_ring_->reserve(laser_cloud_in_->size());

            // Compute ring based on vertical angle
            // Assume LiDAR vertical FOV from -15 to +15 degrees (30 degree total for 16-beam)
            float angle_bottom = -15.0;  // degrees
            float angle_resolution = 30.0 / (n_scan_ - 1);  // angle per ring

            // Scan period for DSS LiDAR (~6.5Hz, so ~0.15s per scan)
            // Using slightly smaller value to ensure IMU data covers the scan
            float scan_period = 0.05;

            for (size_t i = 0; i < laser_cloud_in_->size(); ++i) {
                const auto& pt = laser_cloud_in_->points[i];

                PointXYZIRT new_pt;
                new_pt.x = pt.x;
                new_pt.y = pt.y;
                new_pt.z = pt.z;
                new_pt.intensity = pt.intensity;

                // Calculate vertical angle and ring index
                float range_xy = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                float vertical_angle = std::atan2(pt.z, range_xy) * 180.0 / M_PI;
                int ring = static_cast<int>((vertical_angle - angle_bottom) / angle_resolution + 0.5);
                ring = std::max(0, std::min(n_scan_ - 1, ring));
                new_pt.ring = ring;

                // Calculate horizontal angle for time offset
                float horizontal_angle = std::atan2(pt.y, pt.x);
                // Normalize to 0~2*PI
                if (horizontal_angle < 0) horizontal_angle += 2 * M_PI;
                // Time offset based on horizontal angle (assume full 360 degree scan)
                new_pt.time = scan_period * horizontal_angle / (2 * M_PI);

                laser_cloud_in_ring_->push_back(new_pt);
            }

            time_scan_end_ = time_scan_cur_ + scan_period;

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Converted %zu points to ring/time format", laser_cloud_in_ring_->size());
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imu_mutex_);
        std::lock_guard<std::mutex> lock2(odom_mutex_);

        // Make sure IMU data available for the scan
        if (imu_queue_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "IMU queue is empty! Make sure IMU is publishing to %s", imu_topic_.c_str());
            return false;
        }

        double imu_front_time = rclcpp::Time(imu_queue_.front().header.stamp).seconds();
        double imu_back_time = rclcpp::Time(imu_queue_.back().header.stamp).seconds();

        if (imu_front_time > time_scan_cur_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "IMU data starts after scan! IMU front: %.3f, Scan cur: %.3f (diff: %.3fs)",
                imu_front_time, time_scan_cur_, imu_front_time - time_scan_cur_);
            return false;
        }

        // For DSS platform: IMU data often arrives slightly behind LiDAR due to network latency
        // Skip this check and proceed with partial deskewing using available IMU data
        // The deskewing will interpolate within the available IMU time range
        if (imu_back_time < time_scan_end_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "IMU data ends before scan end (diff: %.3fs), proceeding with available data",
                time_scan_end_ - imu_back_time);
            // Don't return false - proceed with partial data
        }

        imuDeskewInfo();
        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        imu_available_ = false;

        while (!imu_queue_.empty()) {
            if (rclcpp::Time(imu_queue_.front().header.stamp).seconds() < time_scan_cur_ - 0.01) {
                imu_queue_.pop_front();
            } else {
                break;
            }
        }

        if (imu_queue_.empty()) {
            return;
        }

        imu_pointer_cur_ = 0;

        for (int i = 0; i < (int)imu_queue_.size(); ++i) {
            sensor_msgs::msg::Imu& imu = imu_queue_[i];
            double cur_imu_time = rclcpp::Time(imu.header.stamp).seconds();

            if (cur_imu_time < time_scan_cur_) {
                continue;
            }
            if (cur_imu_time > time_scan_end_ + 0.01) {
                break;
            }

            if (imu_pointer_cur_ == 0) {
                imu_rot_x_[0] = 0;
                imu_rot_y_[0] = 0;
                imu_rot_z_[0] = 0;
                imu_time_[0] = cur_imu_time;
                ++imu_pointer_cur_;
                continue;
            }

            // Get angular velocity
            double angular_x = imu.angular_velocity.x;
            double angular_y = imu.angular_velocity.y;
            double angular_z = imu.angular_velocity.z;

            // Integrate
            double dt = cur_imu_time - imu_time_[imu_pointer_cur_ - 1];
            imu_rot_x_[imu_pointer_cur_] = imu_rot_x_[imu_pointer_cur_ - 1] + angular_x * dt;
            imu_rot_y_[imu_pointer_cur_] = imu_rot_y_[imu_pointer_cur_ - 1] + angular_y * dt;
            imu_rot_z_[imu_pointer_cur_] = imu_rot_z_[imu_pointer_cur_ - 1] + angular_z * dt;
            imu_time_[imu_pointer_cur_] = cur_imu_time;
            ++imu_pointer_cur_;
        }

        --imu_pointer_cur_;

        if (imu_pointer_cur_ <= 0) {
            return;
        }

        imu_available_ = true;
    }

    void odomDeskewInfo()
    {
        odom_deskew_flag_ = false;

        while (!odom_queue_.empty()) {
            if (rclcpp::Time(odom_queue_.front().header.stamp).seconds() < time_scan_cur_ - 0.01) {
                odom_queue_.pop_front();
            } else {
                break;
            }
        }

        if (odom_queue_.empty()) {
            return;
        }

        if (rclcpp::Time(odom_queue_.front().header.stamp).seconds() > time_scan_cur_) {
            return;
        }

        // Get start odometry
        nav_msgs::msg::Odometry start_odom = odom_queue_.front();

        // Get end odometry
        nav_msgs::msg::Odometry end_odom;
        for (const auto& odom : odom_queue_) {
            end_odom = odom;
            if (rclcpp::Time(odom.header.stamp).seconds() >= time_scan_end_) {
                break;
            }
        }

        // Store transformation
        Eigen::Affine3f start_tf = pcl::getTransformation(
            start_odom.pose.pose.position.x,
            start_odom.pose.pose.position.y,
            start_odom.pose.pose.position.z,
            0, 0, 0);  // TODO: convert quaternion to RPY

        Eigen::Affine3f end_tf = pcl::getTransformation(
            end_odom.pose.pose.position.x,
            end_odom.pose.pose.position.y,
            end_odom.pose.pose.position.z,
            0, 0, 0);

        Eigen::Affine3f trans_be = start_tf.inverse() * end_tf;

        float roll_incre, pitch_incre, yaw_incre;
        pcl::getTranslationAndEulerAngles(trans_be, odom_incre_x_, odom_incre_y_, odom_incre_z_,
                                          roll_incre, pitch_incre, yaw_incre);

        odom_deskew_flag_ = true;
    }

    void projectPointCloud()
    {
        int cloud_size = laser_cloud_in_ring_->points.size();

        for (int i = 0; i < cloud_size; ++i) {
            PointXYZIRT& point = laser_cloud_in_ring_->points[i];

            // Check range
            float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (range < lidar_min_range_ || range > lidar_max_range_) {
                continue;
            }

            // Get row index
            int row_idx = point.ring;
            if (row_idx < 0 || row_idx >= n_scan_) {
                continue;
            }

            // Downsample
            if (row_idx % downsample_rate_ != 0) {
                continue;
            }

            // Get column index
            float horizon_angle = std::atan2(point.x, point.y) * 180 / M_PI;
            int col_idx = -round((horizon_angle - 90.0) / 360.0 * horizon_scan_) + horizon_scan_ / 2;
            if (col_idx >= horizon_scan_) {
                col_idx -= horizon_scan_;
            }
            if (col_idx < 0 || col_idx >= horizon_scan_) {
                continue;
            }

            // Check if already occupied
            if (range_mat_.at<float>(row_idx, col_idx) != FLT_MAX) {
                continue;
            }

            // Deskew point
            pcl::PointXYZI deskewed_point;
            deskewPoint(&point, deskewed_point);

            // Save point
            range_mat_.at<float>(row_idx, col_idx) = range;
            int index = col_idx + row_idx * horizon_scan_;
            full_cloud_->points[index] = deskewed_point;
        }
    }

    void deskewPoint(PointXYZIRT* point, pcl::PointXYZI& deskewed_point)
    {
        double point_time = time_scan_cur_ + point->time;

        // Get rotation at point time
        float rot_x = 0, rot_y = 0, rot_z = 0;

        if (imu_available_) {
            int imu_pointer_front = 0;
            while (imu_pointer_front < imu_pointer_cur_) {
                if (point_time < imu_time_[imu_pointer_front]) {
                    break;
                }
                ++imu_pointer_front;
            }

            if (point_time > imu_time_[imu_pointer_front] || imu_pointer_front == 0) {
                rot_x = imu_rot_x_[imu_pointer_front];
                rot_y = imu_rot_y_[imu_pointer_front];
                rot_z = imu_rot_z_[imu_pointer_front];
            } else {
                int imu_pointer_back = imu_pointer_front - 1;
                double ratio_front = (point_time - imu_time_[imu_pointer_back]) /
                                    (imu_time_[imu_pointer_front] - imu_time_[imu_pointer_back]);
                double ratio_back = (imu_time_[imu_pointer_front] - point_time) /
                                   (imu_time_[imu_pointer_front] - imu_time_[imu_pointer_back]);
                rot_x = imu_rot_x_[imu_pointer_front] * ratio_front + imu_rot_x_[imu_pointer_back] * ratio_back;
                rot_y = imu_rot_y_[imu_pointer_front] * ratio_front + imu_rot_y_[imu_pointer_back] * ratio_back;
                rot_z = imu_rot_z_[imu_pointer_front] * ratio_front + imu_rot_z_[imu_pointer_back] * ratio_back;
            }
        }

        // Transform point
        if (first_point_flag_) {
            trans_start_inverse_ = (pcl::getTransformation(0, 0, 0, rot_x, rot_y, rot_z)).inverse();
            first_point_flag_ = false;
        }

        Eigen::Affine3f trans_final = pcl::getTransformation(0, 0, 0, rot_x, rot_y, rot_z);
        Eigen::Affine3f trans_bt = trans_start_inverse_ * trans_final;

        Eigen::Vector3f pt(point->x, point->y, point->z);
        pt = trans_bt * pt;

        deskewed_point.x = pt.x();
        deskewed_point.y = pt.y();
        deskewed_point.z = pt.z();
        deskewed_point.intensity = point->intensity;
    }

    void cloudExtraction()
    {
        int count = 0;
        for (int i = 0; i < n_scan_; ++i) {
            start_ring_index_[i] = count - 1 + 5;

            for (int j = 0; j < horizon_scan_; ++j) {
                if (range_mat_.at<float>(i, j) != FLT_MAX) {
                    extracted_cloud_->push_back(full_cloud_->points[j + i * horizon_scan_]);
                    cloud_range_[count] = range_mat_.at<float>(i, j);
                    ++count;
                }
            }

            end_ring_index_[i] = count - 1 - 5;
        }
    }

    void calculateSmoothness()
    {
        int cloud_size = extracted_cloud_->points.size();

        for (int i = 5; i < cloud_size - 5; i++) {
            float diff_range = cloud_range_[i - 5] + cloud_range_[i - 4]
                             + cloud_range_[i - 3] + cloud_range_[i - 2]
                             + cloud_range_[i - 1] - cloud_range_[i] * 10
                             + cloud_range_[i + 1] + cloud_range_[i + 2]
                             + cloud_range_[i + 3] + cloud_range_[i + 4]
                             + cloud_range_[i + 5];

            cloud_curvature_[i] = diff_range * diff_range;
            cloud_neighbor_picked_[i] = 0;
            cloud_label_[i] = 0;
            cloud_smoothness_[i].value = cloud_curvature_[i];
            cloud_smoothness_[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloud_size = extracted_cloud_->points.size();

        for (int i = 5; i < cloud_size - 6; ++i) {
            float depth1 = cloud_range_[i];
            float depth2 = cloud_range_[i + 1];

            if (depth1 - depth2 > 0.3) {
                cloud_neighbor_picked_[i - 5] = 1;
                cloud_neighbor_picked_[i - 4] = 1;
                cloud_neighbor_picked_[i - 3] = 1;
                cloud_neighbor_picked_[i - 2] = 1;
                cloud_neighbor_picked_[i - 1] = 1;
                cloud_neighbor_picked_[i] = 1;
            } else if (depth2 - depth1 > 0.3) {
                cloud_neighbor_picked_[i + 1] = 1;
                cloud_neighbor_picked_[i + 2] = 1;
                cloud_neighbor_picked_[i + 3] = 1;
                cloud_neighbor_picked_[i + 4] = 1;
                cloud_neighbor_picked_[i + 5] = 1;
                cloud_neighbor_picked_[i + 6] = 1;
            }
        }
    }

    void extractFeatures()
    {
        corner_cloud_->clear();
        surface_cloud_->clear();

        for (int i = 0; i < n_scan_; i++) {
            // Divide into 6 segments
            for (int j = 0; j < 6; j++) {
                int sp = (start_ring_index_[i] * (6 - j) + end_ring_index_[i] * j) / 6;
                int ep = (start_ring_index_[i] * (5 - j) + end_ring_index_[i] * (j + 1)) / 6 - 1;

                if (sp >= ep) continue;

                std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep + 1,
                    [](const smoothness_t& a, const smoothness_t& b) {
                        return a.value < b.value;
                    });

                // Extract edge features
                int largest_picked_num = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloud_smoothness_[k].ind;

                    if (cloud_neighbor_picked_[ind] == 0 &&
                        cloud_curvature_[ind] > edge_threshold_) {
                        largest_picked_num++;
                        if (largest_picked_num <= 20) {
                            cloud_label_[ind] = 1;
                            corner_cloud_->push_back(extracted_cloud_->points[ind]);
                        } else {
                            break;
                        }

                        cloud_neighbor_picked_[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            if (ind + l < (int)cloud_smoothness_.size()) {
                                cloud_neighbor_picked_[ind + l] = 1;
                            }
                        }
                        for (int l = -1; l >= -5; l--) {
                            if (ind + l >= 0) {
                                cloud_neighbor_picked_[ind + l] = 1;
                            }
                        }
                    }
                }

                // Extract surface features
                for (int k = sp; k <= ep; k++) {
                    int ind = cloud_smoothness_[k].ind;

                    if (cloud_neighbor_picked_[ind] == 0 &&
                        cloud_curvature_[ind] < surf_threshold_) {
                        cloud_label_[ind] = -1;
                        cloud_neighbor_picked_[ind] = 1;

                        for (int l = 1; l <= 5; l++) {
                            if (ind + l < (int)cloud_smoothness_.size()) {
                                cloud_neighbor_picked_[ind + l] = 1;
                            }
                        }
                        for (int l = -1; l >= -5; l--) {
                            if (ind + l >= 0) {
                                cloud_neighbor_picked_[ind + l] = 1;
                            }
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if (cloud_label_[cloud_smoothness_[k].ind] <= 0) {
                        surface_cloud_->push_back(extracted_cloud_->points[cloud_smoothness_[k].ind]);
                    }
                }
            }
        }
    }

    void publishClouds()
    {
        // Publish deskewed cloud
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*extracted_cloud_, cloud_msg);
        cloud_msg.header = cloud_header_;
        cloud_msg.header.frame_id = lidar_frame_;
        extracted_cloud_pub_->publish(cloud_msg);

        // Publish corner cloud
        pcl::toROSMsg(*corner_cloud_, cloud_msg);
        cloud_msg.header = cloud_header_;
        cloud_msg.header.frame_id = lidar_frame_;
        corner_cloud_pub_->publish(cloud_msg);

        // Publish surface cloud
        pcl::toROSMsg(*surface_cloud_, cloud_msg);
        cloud_msg.header = cloud_header_;
        cloud_msg.header.frame_id = lidar_frame_;
        surface_cloud_pub_->publish(cloud_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published clouds: deskewed=%zu, corner=%zu, surface=%zu",
                    extracted_cloud_->size(), corner_cloud_->size(), surface_cloud_->size());
    }

    // Parameters
    std::string point_cloud_topic_;
    std::string imu_topic_;
    std::string odom_topic_;
    std::string lidar_frame_;
    std::string sensor_type_;

    int n_scan_;
    int horizon_scan_;
    int downsample_rate_;
    double lidar_min_range_;
    double lidar_max_range_;

    double edge_threshold_;
    double surf_threshold_;
    int edge_feature_min_valid_num_;
    int surf_feature_min_valid_num_;

    Eigen::Matrix3d ext_rot_;
    Eigen::Vector3d ext_rpy_;
    Eigen::Vector3d ext_trans_;

    // Point clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_in_;  // Raw input (for PointXYZI)
    pcl::PointCloud<PointXYZIRT>::Ptr laser_cloud_in_ring_;  // With ring/time (computed or direct)
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surface_cloud_;

    cv::Mat range_mat_;

    // Feature extraction
    struct smoothness_t {
        float value;
        size_t ind;
    };

    std::vector<float> cloud_curvature_;
    std::vector<int> cloud_neighbor_picked_;
    std::vector<int> cloud_label_;
    std::vector<smoothness_t> cloud_smoothness_;
    std::vector<float> cloud_range_;  // Range values for extracted points

    int start_ring_index_[128];
    int end_ring_index_[128];

    // IMU data for deskewing
    std::deque<sensor_msgs::msg::Imu> imu_queue_;
    std::mutex imu_mutex_;

    std::deque<nav_msgs::msg::Odometry> odom_queue_;
    std::mutex odom_mutex_;

    double imu_time_[2000];
    double imu_rot_x_[2000];
    double imu_rot_y_[2000];
    double imu_rot_z_[2000];

    int imu_pointer_cur_ = 0;
    bool first_point_flag_ = true;
    Eigen::Affine3f trans_start_inverse_;

    float odom_incre_x_ = 0;
    float odom_incre_y_ = 0;
    float odom_incre_z_ = 0;
    bool odom_deskew_flag_ = false;

    bool imu_available_ = false;

    // Timing
    std_msgs::msg::Header cloud_header_;
    double time_scan_cur_ = 0;
    double time_scan_end_ = 0;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr extracted_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_cloud_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProjection>());
    rclcpp::shutdown();
    return 0;
}
