/**
 * LIO-SAM: IMU Preintegration Node
 *
 * Based on livox_lio_sam implementation from parking_robot_ros2_ws
 */

#include "dss_lio_sam/utility.hpp"
#include <boost/make_shared.hpp>

class IMUPreintegration : public rclcpp::Node
{
public:
    IMUPreintegration()
    : Node("imu_preintegration")
    {
        // Declare parameters
        this->declare_parameter<std::string>("imuTopic", "/imu/data");
        this->declare_parameter<std::string>("odomTopic", "odometry/imu");
        this->declare_parameter<std::string>("lidarFrame", "velodyne");
        this->declare_parameter<std::string>("baselinkFrame", "base_link");
        this->declare_parameter<std::string>("odometryFrame", "odom");
        this->declare_parameter<std::string>("mapFrame", "map");

        this->declare_parameter<double>("imuAccNoise", 3.9939570888238808e-03);
        this->declare_parameter<double>("imuGyrNoise", 1.5636343949698187e-03);
        this->declare_parameter<double>("imuAccBiasN", 6.4356659353532566e-05);
        this->declare_parameter<double>("imuGyrBiasN", 3.5640318696367613e-05);
        this->declare_parameter<double>("imuGravity", 9.80511);
        this->declare_parameter<std::vector<double>>("extrinsicRot", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        this->declare_parameter<std::vector<double>>("extrinsicTrans", std::vector<double>{0.0, 0.0, 0.0});

        // Get parameters
        imu_topic_ = this->get_parameter("imuTopic").as_string();
        odom_topic_ = this->get_parameter("odomTopic").as_string();
        lidar_frame_ = this->get_parameter("lidarFrame").as_string();
        baselink_frame_ = this->get_parameter("baselinkFrame").as_string();
        odometry_frame_ = this->get_parameter("odometryFrame").as_string();
        map_frame_ = this->get_parameter("mapFrame").as_string();

        imu_acc_noise_ = this->get_parameter("imuAccNoise").as_double();
        imu_gyr_noise_ = this->get_parameter("imuGyrNoise").as_double();
        imu_acc_bias_n_ = this->get_parameter("imuAccBiasN").as_double();
        imu_gyr_bias_n_ = this->get_parameter("imuGyrBiasN").as_double();
        imu_gravity_ = this->get_parameter("imuGravity").as_double();

        auto ext_rot = this->get_parameter("extrinsicRot").as_double_array();
        auto ext_trans = this->get_parameter("extrinsicTrans").as_double_array();

        // Set extrinsic rotation
        ext_rot_ << ext_rot[0], ext_rot[1], ext_rot[2],
                    ext_rot[3], ext_rot[4], ext_rot[5],
                    ext_rot[6], ext_rot[7], ext_rot[8];
        ext_trans_ << ext_trans[0], ext_trans[1], ext_trans[2];

        // Initialize GTSAM
        initializeGTSAM();

        // Create subscribers and publishers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "dss_lio_sam/mapping/odometry_incremental", rclcpp::QoS(10),
            std::bind(&IMUPreintegration::odometryHandler, this, std::placeholders::_1));

        imu_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            odom_topic_ + "_incremental", rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "IMU Preintegration node initialized");
    }

private:
    void initializeGTSAM()
    {
        // IMU preintegration parameters
        auto p = gtsam::PreintegrationParams::MakeSharedU(imu_gravity_);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imu_acc_noise_, 2);
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imu_gyr_noise_, 2);
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);

        gtsam::imuBias::ConstantBias prior_imu_bias;  // zero bias

        prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
        prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
        prior_bias_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
        correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
        correction_noise2_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished());
        noise_model_between_bias_ = (gtsam::Vector(6) << imu_acc_bias_n_, imu_acc_bias_n_, imu_acc_bias_n_,
                                                          imu_gyr_bias_n_, imu_gyr_bias_n_, imu_gyr_bias_n_).finished();

        imu_integrator_opt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        imu_integrator_imu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        optimizer_ = gtsam::ISAM2(parameters);

        gtsam::NonlinearFactorGraph new_graph;
        graph_factors_ = new_graph;

        gtsam::Values new_values;
        graph_values_ = new_values;
    }

    void resetParams()
    {
        last_imu_t_imu_ = -1;
        done_first_opt_ = false;
        system_initialized_ = false;
    }

    double stamp2Sec(const builtin_interfaces::msg::Time& stamp)
    {
        return stamp.sec + stamp.nanosec * 1e-9;
    }

    sensor_msgs::msg::Imu transformIMU(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;

        // Transform acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                           imu_in.linear_acceleration.y,
                           imu_in.linear_acceleration.z);
        acc = ext_rot_ * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        // Transform angular velocity
        Eigen::Vector3d gyr(imu_in.angular_velocity.x,
                           imu_in.angular_velocity.y,
                           imu_in.angular_velocity.z);
        gyr = ext_rot_ * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        return imu_out;
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        sensor_msgs::msg::Imu this_imu = transformIMU(*imu_raw);

        imu_que_opt_.push_back(this_imu);
        imu_que_imu_.push_back(this_imu);

        if (!done_first_opt_)
            return;

        double imu_time = stamp2Sec(this_imu.header.stamp);
        double dt = (last_imu_t_imu_ < 0) ? (1.0 / 500.0) : (imu_time - last_imu_t_imu_);
        last_imu_t_imu_ = imu_time;

        // Integrate this single imu message
        imu_integrator_imu_->integrateMeasurement(
            gtsam::Vector3(this_imu.linear_acceleration.x, this_imu.linear_acceleration.y, this_imu.linear_acceleration.z),
            gtsam::Vector3(this_imu.angular_velocity.x, this_imu.angular_velocity.y, this_imu.angular_velocity.z), dt);

        // Predict odometry
        gtsam::NavState current_state = imu_integrator_imu_->predict(prev_state_odom_, prev_bias_odom_);

        // Publish odometry
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = this_imu.header.stamp;
        odometry.header.frame_id = odometry_frame_;
        odometry.child_frame_id = "odom_imu";

        gtsam::Pose3 imu_pose = gtsam::Pose3(current_state.quaternion(), current_state.position());

        odometry.pose.pose.position.x = imu_pose.translation().x();
        odometry.pose.pose.position.y = imu_pose.translation().y();
        odometry.pose.pose.position.z = imu_pose.translation().z();
        odometry.pose.pose.orientation.x = imu_pose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = imu_pose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = imu_pose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = imu_pose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = current_state.velocity().x();
        odometry.twist.twist.linear.y = current_state.velocity().y();
        odometry.twist.twist.linear.z = current_state.velocity().z();
        odometry.twist.twist.angular.x = this_imu.angular_velocity.x + prev_bias_odom_.gyroscope().x();
        odometry.twist.twist.angular.y = this_imu.angular_velocity.y + prev_bias_odom_.gyroscope().y();
        odometry.twist.twist.angular.z = this_imu.angular_velocity.z + prev_bias_odom_.gyroscope().z();

        imu_odom_pub_->publish(odometry);
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        double current_correction_time = stamp2Sec(odom_msg->header.stamp);

        // Make sure we have imu data to integrate
        if (imu_que_opt_.empty())
            return;

        float p_x = odom_msg->pose.pose.position.x;
        float p_y = odom_msg->pose.pose.position.y;
        float p_z = odom_msg->pose.pose.position.z;
        float r_x = odom_msg->pose.pose.orientation.x;
        float r_y = odom_msg->pose.pose.orientation.y;
        float r_z = odom_msg->pose.pose.orientation.z;
        float r_w = odom_msg->pose.pose.orientation.w;
        bool degenerate = (int)odom_msg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidar_pose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // 0. Initialize system
        if (!system_initialized_)
        {
            resetOptimization();

            // Pop old IMU messages
            while (!imu_que_opt_.empty())
            {
                if (stamp2Sec(imu_que_opt_.front().header.stamp) < current_correction_time)
                {
                    last_imu_t_opt_ = stamp2Sec(imu_que_opt_.front().header.stamp);
                    imu_que_opt_.pop_front();
                }
                else
                    break;
            }

            // Initial pose
            prev_pose_ = lidar_pose;
            gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_, prior_pose_noise_);
            graph_factors_.add(prior_pose);

            // Initial velocity
            prev_vel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_, prior_vel_noise_);
            graph_factors_.add(prior_vel);

            // Initial bias
            prev_bias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(B(0), prev_bias_, prior_bias_noise_);
            graph_factors_.add(prior_bias);

            // Add values
            graph_values_.insert(X(0), prev_pose_);
            graph_values_.insert(V(0), prev_vel_);
            graph_values_.insert(B(0), prev_bias_);

            // Optimize once
            optimizer_.update(graph_factors_, graph_values_);
            graph_factors_.resize(0);
            graph_values_.clear();

            imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_);
            imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

            key_ = 1;
            system_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "IMU preintegration system initialized");
            return;
        }

        // Reset graph for speed
        if (key_ == 100)
        {
            // Get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updated_pose_noise =
                gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(key_ - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updated_vel_noise =
                gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(key_ - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updated_bias_noise =
                gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(key_ - 1)));

            // Reset graph
            resetOptimization();

            // Add pose
            gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_, updated_pose_noise);
            graph_factors_.add(prior_pose);

            // Add velocity
            gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_, updated_vel_noise);
            graph_factors_.add(prior_vel);

            // Add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(B(0), prev_bias_, updated_bias_noise);
            graph_factors_.add(prior_bias);

            // Add values
            graph_values_.insert(X(0), prev_pose_);
            graph_values_.insert(V(0), prev_vel_);
            graph_values_.insert(B(0), prev_bias_);

            // Optimize once
            optimizer_.update(graph_factors_, graph_values_);
            graph_factors_.resize(0);
            graph_values_.clear();

            key_ = 1;
        }

        // 1. Integrate IMU data and optimize
        while (!imu_que_opt_.empty())
        {
            sensor_msgs::msg::Imu* this_imu = &imu_que_opt_.front();
            double imu_time = stamp2Sec(this_imu->header.stamp);
            if (imu_time < current_correction_time)
            {
                double dt = (last_imu_t_opt_ < 0) ? (1.0 / 500.0) : (imu_time - last_imu_t_opt_);
                imu_integrator_opt_->integrateMeasurement(
                    gtsam::Vector3(this_imu->linear_acceleration.x, this_imu->linear_acceleration.y, this_imu->linear_acceleration.z),
                    gtsam::Vector3(this_imu->angular_velocity.x, this_imu->angular_velocity.y, this_imu->angular_velocity.z), dt);

                last_imu_t_opt_ = imu_time;
                imu_que_opt_.pop_front();
            }
            else
                break;
        }

        // Add IMU factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu =
            dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_opt_);
        gtsam::ImuFactor imu_factor(X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preint_imu);
        graph_factors_.add(imu_factor);

        // Add IMU bias between factor
        graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
            B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_integrator_opt_->deltaTij()) * noise_model_between_bias_)));

        // Add pose factor
        gtsam::Pose3 cur_pose = lidar_pose;
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key_), cur_pose, degenerate ? correction_noise2_ : correction_noise_);
        graph_factors_.add(pose_factor);

        // Insert predicted values
        gtsam::NavState prop_state = imu_integrator_opt_->predict(prev_state_, prev_bias_);
        graph_values_.insert(X(key_), prop_state.pose());
        graph_values_.insert(V(key_), prop_state.v());
        graph_values_.insert(B(key_), prev_bias_);

        // Optimize
        optimizer_.update(graph_factors_, graph_values_);
        optimizer_.update();
        graph_factors_.resize(0);
        graph_values_.clear();

        // Overwrite the beginning of the preintegration for the next step
        gtsam::Values result = optimizer_.calculateEstimate();
        prev_pose_ = result.at<gtsam::Pose3>(X(key_));
        prev_vel_ = result.at<gtsam::Vector3>(V(key_));
        prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
        prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));

        // Reset the optimization preintegration object
        imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

        // Check optimization
        if (failureDetection(prev_vel_, prev_bias_))
        {
            resetParams();
            return;
        }

        // 2. After optimization, re-propagate imu odometry preintegration
        prev_state_odom_ = prev_state_;
        prev_bias_odom_ = prev_bias_;

        // First pop imu messages older than current correction data
        double last_imu_qt = -1;
        while (!imu_que_imu_.empty() && stamp2Sec(imu_que_imu_.front().header.stamp) < current_correction_time)
        {
            last_imu_qt = stamp2Sec(imu_que_imu_.front().header.stamp);
            imu_que_imu_.pop_front();
        }

        // Repropagate
        if (!imu_que_imu_.empty())
        {
            // Reset bias using the newly optimized bias
            imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_odom_);

            // Integrate imu messages from the beginning of this optimization
            for (size_t i = 0; i < imu_que_imu_.size(); ++i)
            {
                sensor_msgs::msg::Imu* this_imu = &imu_que_imu_[i];
                double imu_time = stamp2Sec(this_imu->header.stamp);
                double dt = (last_imu_qt < 0) ? (1.0 / 500.0) : (imu_time - last_imu_qt);

                imu_integrator_imu_->integrateMeasurement(
                    gtsam::Vector3(this_imu->linear_acceleration.x, this_imu->linear_acceleration.y, this_imu->linear_acceleration.z),
                    gtsam::Vector3(this_imu->angular_velocity.x, this_imu->angular_velocity.y, this_imu->angular_velocity.z), dt);
                last_imu_qt = imu_time;
            }
        }

        ++key_;
        done_first_opt_ = true;
    }

    bool failureDetection(const gtsam::Vector3& vel_cur, const gtsam::imuBias::ConstantBias& bias_cur)
    {
        Eigen::Vector3f vel(vel_cur.x(), vel_cur.y(), vel_cur.z());
        if (vel.norm() > 30)
        {
            RCLCPP_WARN(this->get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(bias_cur.accelerometer().x(), bias_cur.accelerometer().y(), bias_cur.accelerometer().z());
        Eigen::Vector3f bg(bias_cur.gyroscope().x(), bias_cur.gyroscope().y(), bias_cur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            RCLCPP_WARN(this->get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    // Parameters
    std::string imu_topic_;
    std::string odom_topic_;
    std::string lidar_frame_;
    std::string baselink_frame_;
    std::string odometry_frame_;
    std::string map_frame_;

    double imu_acc_noise_;
    double imu_gyr_noise_;
    double imu_acc_bias_n_;
    double imu_gyr_bias_n_;
    double imu_gravity_;

    Eigen::Matrix3d ext_rot_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ext_trans_ = Eigen::Vector3d::Zero();

    // GTSAM
    gtsam::ISAM2 optimizer_;
    gtsam::NonlinearFactorGraph graph_factors_;
    gtsam::Values graph_values_;

    gtsam::PreintegratedImuMeasurements* imu_integrator_opt_;
    gtsam::PreintegratedImuMeasurements* imu_integrator_imu_;

    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise2_;
    gtsam::Vector noise_model_between_bias_;

    gtsam::Pose3 prev_pose_;
    gtsam::Vector3 prev_vel_;
    gtsam::NavState prev_state_;
    gtsam::imuBias::ConstantBias prev_bias_;

    gtsam::NavState prev_state_odom_;
    gtsam::imuBias::ConstantBias prev_bias_odom_;

    std::deque<sensor_msgs::msg::Imu> imu_que_opt_;
    std::deque<sensor_msgs::msg::Imu> imu_que_imu_;

    bool done_first_opt_ = false;
    bool system_initialized_ = false;
    double last_imu_t_imu_ = -1;
    double last_imu_t_opt_ = -1;
    int key_ = 1;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub_;

    std::mutex mtx_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPreintegration>());
    rclcpp::shutdown();
    return 0;
}
