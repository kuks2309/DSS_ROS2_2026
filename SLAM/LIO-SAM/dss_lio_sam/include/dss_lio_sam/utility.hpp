#ifndef DSS_LIO_SAM_UTILITY_HPP
#define DSS_LIO_SAM_UTILITY_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/utilities.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using gtsam::symbol_shorthand::X;  // Pose3 (x, y, z, roll, pitch, yaw)
using gtsam::symbol_shorthand::V;  // Velocity (xdot, ydot, zdot)
using gtsam::symbol_shorthand::B;  // Bias (ax, ay, az, gx, gy, gz)
using gtsam::symbol_shorthand::G;  // GPS pose

// Velodyne point type
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time))

// Ouster point type
struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint32_t, t, t)(uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

// DSS Platform point type (standard PointXYZI with ring and time)
// DSS LiDAR outputs standard point cloud format
struct DSSPointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(DSSPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time))

// Use DSS point as default for DSS platform
using PointXYZIRT = DSSPointXYZIRT;

// Point type for LIO-SAM
struct PointTypePose {
    PCL_ADD_POINT4D;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointTypePose,
    (float, x, x)(float, y, y)(float, z, z)
    (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
    (double, time, time))

// Utility functions
inline float rad2deg(float radians) {
    return radians * 180.0 / M_PI;
}

inline float deg2rad(float degrees) {
    return degrees * M_PI / 180.0;
}

inline sensor_msgs::msg::PointCloud2 publishCloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const rclcpp::Time& stamp,
    const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    if (publisher->get_subscription_count() > 0) {
        publisher->publish(msg);
    }
    return msg;
}

inline Eigen::Affine3f pclPointToAffine3f(const PointTypePose& point) {
    return pcl::getTransformation(point.x, point.y, point.z, point.roll, point.pitch, point.yaw);
}

inline gtsam::Pose3 pclPointToGtsamPose3(const PointTypePose& point) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(point.roll, point.pitch, point.yaw),
        gtsam::Point3(point.x, point.y, point.z));
}

inline Eigen::Affine3f trans2Affine3f(float transformIn[]) {
    return pcl::getTransformation(
        transformIn[3], transformIn[4], transformIn[5],
        transformIn[0], transformIn[1], transformIn[2]);
}

inline gtsam::Pose3 trans2GtsamPose(float transformIn[]) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

inline Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    return yawAngle * pitchAngle * rollAngle;
}

#endif // DSS_LIO_SAM_UTILITY_HPP
