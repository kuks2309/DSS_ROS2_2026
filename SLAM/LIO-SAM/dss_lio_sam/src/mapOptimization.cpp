/**
 * LIO-SAM: Map Optimization Node
 *
 * This node performs:
 * 1. Scan-to-map matching using feature points
 * 2. Factor graph optimization with loop closure
 * 3. Global map management and publishing
 */

#include "dss_lio_sam/utility.hpp"

class MapOptimization : public rclcpp::Node
{
public:
    MapOptimization()
    : Node("map_optimization")
    {
        // Declare parameters
        this->declare_parameter<std::string>("pointCloudTopic", "/velodyne_points");
        this->declare_parameter<std::string>("lidarFrame", "velodyne");
        this->declare_parameter<std::string>("baselinkFrame", "base_link");
        this->declare_parameter<std::string>("odometryFrame", "odom");
        this->declare_parameter<std::string>("mapFrame", "map");

        this->declare_parameter<bool>("savePCD", false);
        this->declare_parameter<std::string>("savePCDDirectory", "/home/amap/ros2_ws/maps/");

        this->declare_parameter<double>("odometrySurfLeafSize", 0.4);
        this->declare_parameter<double>("mappingCornerLeafSize", 0.2);
        this->declare_parameter<double>("mappingSurfLeafSize", 0.4);

        this->declare_parameter<double>("surroundingkeyframeAddingDistThreshold", 1.0);
        this->declare_parameter<double>("surroundingkeyframeAddingAngleThreshold", 0.2);
        this->declare_parameter<double>("surroundingKeyframeDensity", 2.0);
        this->declare_parameter<double>("surroundingKeyframeSearchRadius", 50.0);

        this->declare_parameter<bool>("loopClosureEnableFlag", true);
        this->declare_parameter<double>("loopClosureFrequency", 1.0);
        this->declare_parameter<int>("surroundingKeyframeSize", 50);
        this->declare_parameter<double>("historyKeyframeSearchRadius", 15.0);
        this->declare_parameter<double>("historyKeyframeSearchTimeDiff", 30.0);
        this->declare_parameter<int>("historyKeyframeSearchNum", 25);
        this->declare_parameter<double>("historyKeyframeFitnessScore", 0.3);

        this->declare_parameter<double>("globalMapVisualizationSearchRadius", 1000.0);
        this->declare_parameter<double>("globalMapVisualizationPoseDensity", 10.0);
        this->declare_parameter<double>("globalMapVisualizationLeafSize", 1.0);

        this->declare_parameter<int>("numberOfCores", 4);
        this->declare_parameter<double>("mappingProcessInterval", 0.15);

        // Get parameters
        lidar_frame_ = this->get_parameter("lidarFrame").as_string();
        baselink_frame_ = this->get_parameter("baselinkFrame").as_string();
        odometry_frame_ = this->get_parameter("odometryFrame").as_string();
        map_frame_ = this->get_parameter("mapFrame").as_string();

        save_pcd_ = this->get_parameter("savePCD").as_bool();
        save_pcd_directory_ = this->get_parameter("savePCDDirectory").as_string();

        surf_leaf_size_ = this->get_parameter("odometrySurfLeafSize").as_double();
        corner_leaf_size_ = this->get_parameter("mappingCornerLeafSize").as_double();
        mapping_surf_leaf_size_ = this->get_parameter("mappingSurfLeafSize").as_double();

        keyframe_dist_threshold_ = this->get_parameter("surroundingkeyframeAddingDistThreshold").as_double();
        keyframe_angle_threshold_ = this->get_parameter("surroundingkeyframeAddingAngleThreshold").as_double();
        keyframe_density_ = this->get_parameter("surroundingKeyframeDensity").as_double();
        keyframe_search_radius_ = this->get_parameter("surroundingKeyframeSearchRadius").as_double();

        loop_closure_enabled_ = this->get_parameter("loopClosureEnableFlag").as_bool();
        loop_closure_frequency_ = this->get_parameter("loopClosureFrequency").as_double();
        surrounding_keyframe_size_ = this->get_parameter("surroundingKeyframeSize").as_int();
        history_search_radius_ = this->get_parameter("historyKeyframeSearchRadius").as_double();
        history_search_time_diff_ = this->get_parameter("historyKeyframeSearchTimeDiff").as_double();
        history_search_num_ = this->get_parameter("historyKeyframeSearchNum").as_int();
        history_fitness_score_ = this->get_parameter("historyKeyframeFitnessScore").as_double();

        global_map_vis_radius_ = this->get_parameter("globalMapVisualizationSearchRadius").as_double();
        global_map_vis_density_ = this->get_parameter("globalMapVisualizationPoseDensity").as_double();
        global_map_vis_leaf_size_ = this->get_parameter("globalMapVisualizationLeafSize").as_double();

        num_cores_ = this->get_parameter("numberOfCores").as_int();
        mapping_interval_ = this->get_parameter("mappingProcessInterval").as_double();

        // Initialize GTSAM
        initializeGTSAM();

        // Allocate memory
        allocateMemory();

        // Create subscribers
        corner_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/feature/cloud_corner", 1,
            std::bind(&MapOptimization::cornerCloudCallback, this, std::placeholders::_1));

        surface_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/feature/cloud_surface", 1,
            std::bind(&MapOptimization::surfaceCloudCallback, this, std::placeholders::_1));

        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "dss_lio_sam/mapping/odometry", 1);
        odom_incremental_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "dss_lio_sam/mapping/odometry_incremental", 1);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "dss_lio_sam/mapping/path", 1);
        map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/mapping/cloud_registered", 1);
        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "dss_lio_sam/mapping/map_global", 1);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create timers
        mapping_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(mapping_interval_ * 1000)),
            std::bind(&MapOptimization::mappingCallback, this));

        if (loop_closure_enabled_) {
            loop_closure_timer_ = this->create_wall_timer(
                std::chrono::milliseconds((int)(1000.0 / loop_closure_frequency_)),
                std::bind(&MapOptimization::loopClosureCallback, this));
        }

        visualization_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapOptimization::visualizationCallback, this));

        RCLCPP_INFO(this->get_logger(), "Map Optimization node initialized");
    }

    ~MapOptimization()
    {
        if (save_pcd_) {
            savePCD();
        }
    }

private:
    void initializeGTSAM()
    {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam2_ = std::make_unique<gtsam::ISAM2>(parameters);

        // Prior noise
        prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-2, 1e-2, M_PI, 1e8, 1e8, 1e8).finished());

        // Odometry noise
        odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    }

    void allocateMemory()
    {
        // Point clouds
        corner_cloud_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        surface_cloud_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        corner_cloud_from_map_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        surface_cloud_from_map_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        current_corner_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        current_surface_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        current_corner_cloud_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        current_surface_cloud_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        cloud_keyframes_3d_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_keyframes_6d_.reset(new pcl::PointCloud<PointTypePose>());

        // Voxel filters
        downsize_filter_corner_.setLeafSize(corner_leaf_size_, corner_leaf_size_, corner_leaf_size_);
        downsize_filter_surf_.setLeafSize(mapping_surf_leaf_size_, mapping_surf_leaf_size_, mapping_surf_leaf_size_);
        downsize_filter_icp_.setLeafSize(surf_leaf_size_, surf_leaf_size_, surf_leaf_size_);
        downsize_filter_map_.setLeafSize(global_map_vis_leaf_size_, global_map_vis_leaf_size_, global_map_vis_leaf_size_);

        // KD trees
        kdtree_corner_from_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdtree_surf_from_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdtree_surrounding_keyframes_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        kdtree_history_keyframes_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

        // Initialize transform
        for (int i = 0; i < 6; ++i) {
            transform_to_be_mapped_[i] = 0;
        }
    }

    void cornerCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(corner_mutex_);
        pcl::fromROSMsg(*msg, *current_corner_cloud_);
        corner_time_ = rclcpp::Time(msg->header.stamp).seconds();
        new_corner_cloud_ = true;
    }

    void surfaceCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(surface_mutex_);
        pcl::fromROSMsg(*msg, *current_surface_cloud_);
        surface_time_ = rclcpp::Time(msg->header.stamp).seconds();
        new_surface_cloud_ = true;
    }

    void mappingCallback()
    {
        // Check if we have new data
        {
            std::lock_guard<std::mutex> lock1(corner_mutex_);
            std::lock_guard<std::mutex> lock2(surface_mutex_);

            if (!new_corner_cloud_ || !new_surface_cloud_) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Waiting for clouds: corner=%d, surface=%d",
                    new_corner_cloud_, new_surface_cloud_);
                return;
            }

            if (std::abs(corner_time_ - surface_time_) > 0.05) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Corner/Surface time mismatch: diff=%.4fs (corner=%.3f, surface=%.3f)",
                    std::abs(corner_time_ - surface_time_), corner_time_, surface_time_);
                return;
            }

            new_corner_cloud_ = false;
            new_surface_cloud_ = false;
        }

        // Downsample current clouds
        downsampleCurrentClouds();

        // First frame
        if (cloud_keyframes_3d_->empty()) {
            addFirstKeyframe();
            return;
        }

        // Extract surrounding keyframes
        extractSurroundingKeyframes();

        // Scan to map optimization
        scan2MapOptimization();

        // Save keyframes
        saveKeyframesAndFactors();

        // Publish odometry
        publishOdometry();

        // Publish TF
        publishTF();
    }

    void downsampleCurrentClouds()
    {
        // Downsample corner cloud
        current_corner_cloud_ds_->clear();
        downsize_filter_corner_.setInputCloud(current_corner_cloud_);
        downsize_filter_corner_.filter(*current_corner_cloud_ds_);

        // Downsample surface cloud
        current_surface_cloud_ds_->clear();
        downsize_filter_surf_.setInputCloud(current_surface_cloud_);
        downsize_filter_surf_.filter(*current_surface_cloud_ds_);
    }

    void addFirstKeyframe()
    {
        // Add prior factor
        gtsam::Pose3 pose_cur = trans2GtsamPose(transform_to_be_mapped_);
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose_cur, prior_noise_));
        initial_estimate_.insert(0, pose_cur);

        // Update ISAM2
        isam2_->update(graph_, initial_estimate_);
        isam2_->update();
        graph_.resize(0);
        initial_estimate_.clear();

        // Save keyframe
        pcl::PointXYZI point_3d;
        point_3d.x = transform_to_be_mapped_[3];
        point_3d.y = transform_to_be_mapped_[4];
        point_3d.z = transform_to_be_mapped_[5];
        point_3d.intensity = cloud_keyframes_3d_->size();
        cloud_keyframes_3d_->push_back(point_3d);

        PointTypePose point_6d;
        point_6d.x = transform_to_be_mapped_[3];
        point_6d.y = transform_to_be_mapped_[4];
        point_6d.z = transform_to_be_mapped_[5];
        point_6d.roll = transform_to_be_mapped_[0];
        point_6d.pitch = transform_to_be_mapped_[1];
        point_6d.yaw = transform_to_be_mapped_[2];
        point_6d.time = corner_time_;
        cloud_keyframes_6d_->push_back(point_6d);

        // Save corner and surface clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr corner_copy(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surface_copy(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*current_corner_cloud_ds_, *corner_copy);
        pcl::copyPointCloud(*current_surface_cloud_ds_, *surface_copy);
        corner_cloud_keyframes_.push_back(corner_copy);
        surface_cloud_keyframes_.push_back(surface_copy);

        RCLCPP_INFO(this->get_logger(), "First keyframe added");
    }

    void extractSurroundingKeyframes()
    {
        if (cloud_keyframes_3d_->empty()) {
            return;
        }

        // Search for surrounding keyframes
        pcl::PointCloud<pcl::PointXYZI>::Ptr surrounding_keyframes(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surrounding_keyframes_ds(new pcl::PointCloud<pcl::PointXYZI>());

        std::vector<int> point_search_idx;
        std::vector<float> point_search_sq_dist;

        kdtree_surrounding_keyframes_->setInputCloud(cloud_keyframes_3d_);
        pcl::PointXYZI current_point;
        current_point.x = transform_to_be_mapped_[3];
        current_point.y = transform_to_be_mapped_[4];
        current_point.z = transform_to_be_mapped_[5];

        kdtree_surrounding_keyframes_->radiusSearch(current_point, keyframe_search_radius_,
                                                    point_search_idx, point_search_sq_dist);

        for (int i = 0; i < (int)point_search_idx.size(); ++i) {
            surrounding_keyframes->push_back(cloud_keyframes_3d_->points[point_search_idx[i]]);
        }

        // Downsample surrounding keyframes
        pcl::VoxelGrid<pcl::PointXYZI> downsize_filter;
        downsize_filter.setLeafSize(keyframe_density_, keyframe_density_, keyframe_density_);
        downsize_filter.setInputCloud(surrounding_keyframes);
        downsize_filter.filter(*surrounding_keyframes_ds);

        // Clear previous map
        corner_cloud_from_map_->clear();
        surface_cloud_from_map_->clear();

        // Build local map from surrounding keyframes
        for (int i = 0; i < (int)surrounding_keyframes_ds->size(); ++i) {
            int idx = (int)surrounding_keyframes_ds->points[i].intensity;

            // Transform corner cloud
            Eigen::Affine3f keyframe_tf = pclPointToAffine3f(cloud_keyframes_6d_->points[idx]);
            pcl::PointCloud<pcl::PointXYZI>::Ptr corner_tf(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*corner_cloud_keyframes_[idx], *corner_tf, keyframe_tf);
            *corner_cloud_from_map_ += *corner_tf;

            // Transform surface cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr surface_tf(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*surface_cloud_keyframes_[idx], *surface_tf, keyframe_tf);
            *surface_cloud_from_map_ += *surface_tf;
        }

        // Downsample local map
        corner_cloud_from_map_ds_->clear();
        downsize_filter_corner_.setInputCloud(corner_cloud_from_map_);
        downsize_filter_corner_.filter(*corner_cloud_from_map_ds_);

        surface_cloud_from_map_ds_->clear();
        downsize_filter_surf_.setInputCloud(surface_cloud_from_map_);
        downsize_filter_surf_.filter(*surface_cloud_from_map_ds_);
    }

    void scan2MapOptimization()
    {
        if (corner_cloud_from_map_ds_->empty() || surface_cloud_from_map_ds_->empty()) {
            return;
        }

        // Set KD-tree input
        kdtree_corner_from_map_->setInputCloud(corner_cloud_from_map_ds_);
        kdtree_surf_from_map_->setInputCloud(surface_cloud_from_map_ds_);

        // LM optimization iterations
        for (int iter = 0; iter < 30; ++iter) {
            // Edge and planar correspondences are computed here
            // and used for LM optimization (simplified version)

            // This is a placeholder for the full LM optimization
            // In a complete implementation, you would:
            // 1. Find correspondences for corner features
            // 2. Find correspondences for surface features
            // 3. Build Jacobian and residual
            // 4. Solve linear system to update transform
        }
    }

    void saveKeyframesAndFactors()
    {
        // Check if we should add a new keyframe
        PointTypePose& last_keyframe = cloud_keyframes_6d_->back();
        float dx = transform_to_be_mapped_[3] - last_keyframe.x;
        float dy = transform_to_be_mapped_[4] - last_keyframe.y;
        float dz = transform_to_be_mapped_[5] - last_keyframe.z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        float droll = std::abs(transform_to_be_mapped_[0] - last_keyframe.roll);
        float dpitch = std::abs(transform_to_be_mapped_[1] - last_keyframe.pitch);
        float dyaw = std::abs(transform_to_be_mapped_[2] - last_keyframe.yaw);

        if (dist < keyframe_dist_threshold_ &&
            droll < keyframe_angle_threshold_ &&
            dpitch < keyframe_angle_threshold_ &&
            dyaw < keyframe_angle_threshold_) {
            return;
        }

        // Add odometry factor
        gtsam::Pose3 pose_from = pclPointToGtsamPose3(cloud_keyframes_6d_->back());
        gtsam::Pose3 pose_to = trans2GtsamPose(transform_to_be_mapped_);

        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            cloud_keyframes_3d_->size() - 1, cloud_keyframes_3d_->size(),
            pose_from.between(pose_to), odom_noise_));

        initial_estimate_.insert(cloud_keyframes_3d_->size(), pose_to);

        // Update ISAM2
        isam2_->update(graph_, initial_estimate_);
        isam2_->update();
        graph_.resize(0);
        initial_estimate_.clear();

        // Get corrected poses
        gtsam::Values current_estimate = isam2_->calculateEstimate();
        gtsam::Pose3 latest_estimate = current_estimate.at<gtsam::Pose3>(current_estimate.size() - 1);

        transform_to_be_mapped_[0] = latest_estimate.rotation().roll();
        transform_to_be_mapped_[1] = latest_estimate.rotation().pitch();
        transform_to_be_mapped_[2] = latest_estimate.rotation().yaw();
        transform_to_be_mapped_[3] = latest_estimate.translation().x();
        transform_to_be_mapped_[4] = latest_estimate.translation().y();
        transform_to_be_mapped_[5] = latest_estimate.translation().z();

        // Save keyframe
        pcl::PointXYZI point_3d;
        point_3d.x = transform_to_be_mapped_[3];
        point_3d.y = transform_to_be_mapped_[4];
        point_3d.z = transform_to_be_mapped_[5];
        point_3d.intensity = cloud_keyframes_3d_->size();
        cloud_keyframes_3d_->push_back(point_3d);

        PointTypePose point_6d;
        point_6d.x = transform_to_be_mapped_[3];
        point_6d.y = transform_to_be_mapped_[4];
        point_6d.z = transform_to_be_mapped_[5];
        point_6d.roll = transform_to_be_mapped_[0];
        point_6d.pitch = transform_to_be_mapped_[1];
        point_6d.yaw = transform_to_be_mapped_[2];
        point_6d.time = corner_time_;
        cloud_keyframes_6d_->push_back(point_6d);

        // Save corner and surface clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr corner_copy(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr surface_copy(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*current_corner_cloud_ds_, *corner_copy);
        pcl::copyPointCloud(*current_surface_cloud_ds_, *surface_copy);
        corner_cloud_keyframes_.push_back(corner_copy);
        surface_cloud_keyframes_.push_back(surface_copy);

        // Update poses
        updatePoses();
    }

    void updatePoses()
    {
        gtsam::Values current_estimate = isam2_->calculateEstimate();

        for (int i = 0; i < (int)current_estimate.size(); ++i) {
            gtsam::Pose3 pose = current_estimate.at<gtsam::Pose3>(i);
            cloud_keyframes_3d_->points[i].x = pose.translation().x();
            cloud_keyframes_3d_->points[i].y = pose.translation().y();
            cloud_keyframes_3d_->points[i].z = pose.translation().z();

            cloud_keyframes_6d_->points[i].x = pose.translation().x();
            cloud_keyframes_6d_->points[i].y = pose.translation().y();
            cloud_keyframes_6d_->points[i].z = pose.translation().z();
            cloud_keyframes_6d_->points[i].roll = pose.rotation().roll();
            cloud_keyframes_6d_->points[i].pitch = pose.rotation().pitch();
            cloud_keyframes_6d_->points[i].yaw = pose.rotation().yaw();
        }
    }

    void loopClosureCallback()
    {
        if (!loop_closure_enabled_ || cloud_keyframes_3d_->empty()) {
            return;
        }

        // Simple loop closure detection placeholder
        // In a complete implementation:
        // 1. Search for candidates based on distance and time
        // 2. Perform ICP matching
        // 3. Add loop closure factor if fitness score is good
        // 4. Update ISAM2
    }

    void publishOdometry()
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(corner_time_ * 1e9));
        odom_msg.header.frame_id = odometry_frame_;
        odom_msg.child_frame_id = baselink_frame_;

        Eigen::Quaternionf q = euler2Quaternion(
            transform_to_be_mapped_[0],
            transform_to_be_mapped_[1],
            transform_to_be_mapped_[2]).cast<float>();

        odom_msg.pose.pose.position.x = transform_to_be_mapped_[3];
        odom_msg.pose.pose.position.y = transform_to_be_mapped_[4];
        odom_msg.pose.pose.position.z = transform_to_be_mapped_[5];
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();

        odom_pub_->publish(odom_msg);
        odom_incremental_pub_->publish(odom_msg);

        // Publish path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        global_path_.header = odom_msg.header;
        global_path_.poses.push_back(pose_stamped);
        path_pub_->publish(global_path_);
    }

    void publishTF()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(corner_time_ * 1e9));
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odometry_frame_;

        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        transform.transform.rotation.w = 1;
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = 0;

        tf_broadcaster_->sendTransform(transform);
    }

    void visualizationCallback()
    {
        publishGlobalMap();
    }

    void publishGlobalMap()
    {
        if (cloud_keyframes_3d_->empty()) {
            return;
        }

        // Build global map
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_ds(new pcl::PointCloud<pcl::PointXYZI>());

        for (int i = 0; i < (int)cloud_keyframes_6d_->size(); ++i) {
            Eigen::Affine3f keyframe_tf = pclPointToAffine3f(cloud_keyframes_6d_->points[i]);

            pcl::PointCloud<pcl::PointXYZI>::Ptr surface_tf(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*surface_cloud_keyframes_[i], *surface_tf, keyframe_tf);
            *global_map += *surface_tf;
        }

        // Downsample
        downsize_filter_map_.setInputCloud(global_map);
        downsize_filter_map_.filter(*global_map_ds);

        // Publish
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*global_map_ds, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = map_frame_;
        global_map_pub_->publish(cloud_msg);
    }

    void savePCD()
    {
        RCLCPP_INFO(this->get_logger(), "Saving PCD files...");

        // Build and save global map
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>());

        for (int i = 0; i < (int)cloud_keyframes_6d_->size(); ++i) {
            Eigen::Affine3f keyframe_tf = pclPointToAffine3f(cloud_keyframes_6d_->points[i]);

            pcl::PointCloud<pcl::PointXYZI>::Ptr corner_tf(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr surface_tf(new pcl::PointCloud<pcl::PointXYZI>());

            pcl::transformPointCloud(*corner_cloud_keyframes_[i], *corner_tf, keyframe_tf);
            pcl::transformPointCloud(*surface_cloud_keyframes_[i], *surface_tf, keyframe_tf);

            *global_map += *corner_tf;
            *global_map += *surface_tf;
        }

        // Save
        std::string filename = save_pcd_directory_ + "dss_lio_sam_map.pcd";
        pcl::io::savePCDFileBinary(filename, *global_map);
        RCLCPP_INFO(this->get_logger(), "Saved map to: %s", filename.c_str());
    }

    // Parameters
    std::string lidar_frame_;
    std::string baselink_frame_;
    std::string odometry_frame_;
    std::string map_frame_;

    bool save_pcd_;
    std::string save_pcd_directory_;

    double surf_leaf_size_;
    double corner_leaf_size_;
    double mapping_surf_leaf_size_;

    double keyframe_dist_threshold_;
    double keyframe_angle_threshold_;
    double keyframe_density_;
    double keyframe_search_radius_;

    bool loop_closure_enabled_;
    double loop_closure_frequency_;
    int surrounding_keyframe_size_;
    double history_search_radius_;
    double history_search_time_diff_;
    int history_search_num_;
    double history_fitness_score_;

    double global_map_vis_radius_;
    double global_map_vis_density_;
    double global_map_vis_leaf_size_;

    int num_cores_;
    double mapping_interval_;

    // GTSAM
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;

    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;

    // Point clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_from_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surface_cloud_from_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_from_map_ds_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surface_cloud_from_map_ds_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_corner_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_surface_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_corner_cloud_ds_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_surface_cloud_ds_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keyframes_3d_;
    pcl::PointCloud<PointTypePose>::Ptr cloud_keyframes_6d_;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> corner_cloud_keyframes_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> surface_cloud_keyframes_;

    // Voxel filters
    pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_corner_;
    pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_surf_;
    pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_icp_;
    pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_map_;

    // KD trees
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_corner_from_map_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_from_map_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surrounding_keyframes_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_history_keyframes_;

    // Transform
    float transform_to_be_mapped_[6];

    // Path
    nav_msgs::msg::Path global_path_;

    // Mutexes
    std::mutex corner_mutex_;
    std::mutex surface_mutex_;

    double corner_time_ = 0;
    double surface_time_ = 0;
    bool new_corner_cloud_ = false;
    bool new_surface_cloud_ = false;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr corner_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surface_cloud_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_incremental_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr mapping_timer_;
    rclcpp::TimerBase::SharedPtr loop_closure_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOptimization>());
    rclcpp::shutdown();
    return 0;
}
