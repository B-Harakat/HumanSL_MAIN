#pragma once

#include <memory>
#include <vector>
#include <string>
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
#include <functional>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/base/Vector.h>
#include <gpmp2/kinematics/ArmModel.h>

#include "ezc3d/ezc3d.h"
#include "ezc3d/Header.h"
#include "ezc3d/Parameters.h"
#include "ezc3d/Data.h"


struct JointLimits {
    gtsam::Vector lower;
    gtsam::Vector upper;

    JointLimits() : lower(gtsam::Vector::Zero(7)), upper(gtsam::Vector::Zero(7)) {}
    
    JointLimits(size_t dof) : lower(gtsam::Vector::Zero(dof)), upper(gtsam::Vector::Zero(dof)) {}
};


struct DHParameters {
    gtsam::Vector a;
    gtsam::Vector alpha;
    gtsam::Vector d;
    gtsam::Vector theta;
};

struct HumanBoundingBox {
    double min_x, max_x, min_y, max_y, min_z, max_z;
};


struct C3D_Dataset {
    std::vector<gtsam::Point3> human_points;  // 3D points from C3D file
    gtsam::Point3 clav;
    gtsam::Point3 strn;  // CLAV and STRN points for arm base pose calculation
    HumanBoundingBox bounds;  // Bounding box of human points
};

struct HumanInfo {
    std::vector<gtsam::Point3> human_points;  // 3D points from vicon readings
    HumanBoundingBox bounds;  // Bounding box of human points
    gtsam::Point3 RFIN;
    gtsam::Point3 LFIN;
    gtsam::Point3 RFHD;
    gtsam::Point3 LFHD;
};

struct TubeInfo {
    std::vector<Eigen::Vector3d> tube_points;
    Eigen::Vector3d centroid;
    Eigen::Vector3d direction;  // Unit vector along tube axis
    double length;              // Extent along axis
};


struct TrajectoryResult {
    std::vector<gtsam::Vector> trajectory_pos;
    std::vector<gtsam::Vector> trajectory_vel;

    std::unordered_map<std::string, double> start_costs;
    std::unordered_map<std::string, double> final_costs;

    std::chrono::milliseconds optimization_duration;
    std::chrono::milliseconds initiation_duration;

    double start_error;
    double final_error;

    double dt;

    TrajectoryResult() : dt(1e-3) {}
};

struct JointTrajectory {
    std::deque<Eigen::VectorXd> pos;
    std::deque<Eigen::VectorXd> vel;
    std::deque<Eigen::VectorXd> acc;
};


struct TaskTrajectory {
    std::deque<Eigen::VectorXd> pos;
    std::deque<Eigen::VectorXd> vel;
    std::deque<Eigen::VectorXd> acc;
};


struct GPMP2_OccupancyGrid {
    size_t rows;
    size_t cols; 
    size_t z;
    double origin_x;
    double origin_y;
    double origin_z;
    double cell_size;
    std::vector<std::vector<std::vector<float>>> map;  // 3D occupancy grid [rows][cols][z]
    
    GPMP2_OccupancyGrid() = default;
    
    // Constructor from dimensions
    GPMP2_OccupancyGrid(size_t r, size_t c, size_t z_dim, double ox, double oy, double oz, double cs)
        : rows(r), cols(c), z(z_dim), origin_x(ox), origin_y(oy), origin_z(oz), cell_size(cs) {
        map.resize(rows);
        for (size_t i = 0; i < rows; ++i) {
            map[i].resize(cols);
            for (size_t j = 0; j < cols; ++j) {
                map[i][j].resize(z, 0.0f);
            }
        }
    }
};

struct TrajectoryRecord {
    std::vector<Eigen::VectorXd> target_trajectory;
    std::vector<Eigen::VectorXd> actual_trajectory;
};


gtsam::Pose3 createPoseFromConf(const gpmp2::ArmModel& arm_model, const gtsam::Vector& config, bool upright = false);

gtsam::Pose3 createPoseFromTube( const TubeInfo& tube_axis, double human_max_y, double offset_from_human_y, double offset_from_tube_z);

std::pair<gtsam::Pose3, gtsam::Pose3> createArmBasePoses(const gtsam::Point3& clav_point, const gtsam::Point3& strn_point);

std::deque<Eigen::VectorXd> convertToDeg(const std::vector<gtsam::Vector>& gtsam_trajectory);

std::deque<Eigen::VectorXd> computeAcceleration(
    const std::deque<Eigen::VectorXd>& velocity, 
    double dt);

template<typename T>
T convertTrajectory(const TrajectoryResult& result, double dt){
    T trajectory;
    trajectory.pos = convertToDeg(result.trajectory_pos);
    trajectory.vel = convertToDeg(result.trajectory_vel);
    trajectory.acc = computeAcceleration(trajectory.vel,dt);
    return trajectory;
}

void analyzeTrajectoryResults(
    const gpmp2::ArmModel& arm_model,
    const TrajectoryResult& trajectory,
    const gtsam::Pose3& target_pose
);

DHParameters createDHParams(const std::string& yaml_path);

std::pair<JointLimits, JointLimits> createJointLimits(const std::string& config_path);

gtsam::Matrix4 createDHTransform(double a, double alpha, double d, double theta);

gtsam::Pose3 computeBaseToEE(const DHParameters& dh, const gtsam::Vector& joint_angles);

gtsam::Pose3 forwardKinematics(const DHParameters& dh, 
                               const gtsam::Vector& joint_angles, 
                               const gtsam::Pose3& base_pose_in_world);
                               
gtsam::Pose3 inverseForwardKinematics(const DHParameters& dh, 
                              const gtsam::Vector& joint_angles, 
                              const gtsam::Pose3& ee_pose_in_world);

std::vector<double> shiftAngle(std::vector<double>& q_cur);

Eigen::VectorXd shiftAngle(Eigen::VectorXd& q_cur);

void shiftAngleInPlace(Eigen::VectorXd& q_cur);

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> pop_front(JointTrajectory& trajectory); 
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> pop_front(TaskTrajectory& trajectory);

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> world2base(
    const Eigen::VectorXd& p,
    const Eigen::VectorXd& dp, 
    const Eigen::VectorXd& ddp,
    const gtsam::Pose3& base_pose_);