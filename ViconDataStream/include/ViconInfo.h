#pragma once

#define _USE_MATH_DEFINES

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include <thread>
#include <iomanip>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <tuple>

#include <Eigen/Dense>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/base/Vector.h>

#include <shared_mutex>

#include "ViconInterface.h"
#include "utils.h"

void updateTubeInfo(TubeInfo& tube_info, std::vector<MarkerData>& tube_tip, std::vector<MarkerData>& tube_end);

void updateHumanInfo(HumanInfo& human_info, std::vector<MarkerData>& human);

void updateTargetInfo(gtsam::Point3& target_info, std::vector<MarkerData>& target);

void updateViconInfo(ViconInterface& vicon, gtsam::Pose3& left_base, gtsam::Pose3& right_base, TubeInfo& tube_info, HumanInfo& human_info, gtsam::Point3& target_info, std::vector<double>& left_conf, std::vector<double>& right_conf, DHParameters& dh_params, std::shared_mutex& vicon_data_mutex, std::shared_mutex& joint_data_mutex);

gtsam::Pose3 updatePoseInfo1(std::vector<MarkerData>& vicon_data, DHParameters& dh_params, std::vector<double>& joint_conf);

gtsam::Pose3 updatePoseInfo2(std::vector<MarkerData>& vicon_data, MarkerData other_arm_base_1);

gtsam::Pose3 calculateFramePose(const Eigen::Vector3d& world_p1, 
                               const Eigen::Vector3d& world_p2,
                               const Eigen::Vector3d& world_p3, 
                               const Eigen::Vector3d& world_p4, 
                               double z_offset, 
                               bool p1_in_positive_x,
                               bool p4_in_positive_z);


