#pragma once

#include <vector>
#include <string>
#include <memory>
#include <array>
#include <unordered_map>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <queue>
#include <limits>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstdlib>

#include <yaml-cpp/yaml.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>  // Added missing include
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/kinematics/ArmModel.h>  // Added missing include
#include <gtsam/base/Vector.h>

#include "utils.h"


void saveTrajectoryResultToYAML(const TrajectoryResult& result, const std::string& method); 
void exportSDFToYAML(const gpmp2::SignedDistanceField& sdf);
void saveRecordToYAML(const TrajectoryRecord& record, const std::string& name);

void visualizeTrajectory(
    const std::vector<gtsam::Vector>& trajectory,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose
);

void visualizeTrajectory(
    const std::deque<Eigen::VectorXd>& trajectory,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose);

void storeEventToYaml(
    const std::deque<Eigen::VectorXd>& trajectory,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose);

void visualizeTrajectoryStatic(
    const std::vector<double>& configuration_degrees,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose);

void visualizeTaskTrajectory(
    const std::deque<Eigen::VectorXd>& trajectory,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose);

 