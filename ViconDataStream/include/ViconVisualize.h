#include <vector>
#include <string>
#include <iostream>
#include <shared_mutex>

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gpmp2/kinematics/ArmModel.h>

#include <open3d/Open3D.h>

#include "utils.h"


void visualizeStaticConfiguration(
    const Eigen::VectorXd& joint_angles,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose);

// void visualizeRealtime(
//     const std::vector<double>& left_arm_angles,
//     const std::vector<double>& right_arm_angles,
//     const gtsam::Pose3& left_base_pose,
//     const gtsam::Pose3& right_base_pose,
//     const std::string& dh_params_path,
//     const TubeInfo& tube_info,
//     const HumanInfo& human_info,
//     std::shared_mutex& joint_data_mutex,
//     std::shared_mutex& vicon_data_mutex);

// void visualizeRealtimePCL(
//     const std::vector<double>& left_arm_angles,
//     const std::vector<double>& right_arm_angles,
//     const gtsam::Pose3& left_base_pose,
//     const gtsam::Pose3& right_base_pose,
//     const std::string& dh_params_path,
//     const TubeInfo& tube_info,
//     const HumanInfo& human_info,
//     std::shared_mutex& joint_data_mutex,
//     std::shared_mutex& vicon_data_mutex);

// void visualizeRealtimeConsole(
//     const std::vector<double>& left_arm_angles,
//     const std::vector<double>& right_arm_angles,
//     const gtsam::Pose3& left_base_pose,
//     const gtsam::Pose3& right_base_pose,
//     const std::string& dh_params_path,
//     const TubeInfo& tube_info,
//     const HumanInfo& human_info,
//     std::shared_mutex& joint_data_mutex,
//     std::shared_mutex& vicon_data_mutex);