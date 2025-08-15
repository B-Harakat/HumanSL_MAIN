//----------------------------------------------------------
// Functions of the forward kinematics
//----------------------------------------------------------
// Description: functions used to calculate the forward kinematics
// Copyright: Yihan Liu 2024
//----------------------------------------------------------

#define _USE_MATH_DEFINES

#include "Fwd_kinematics.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <tuple>

using namespace Eigen;
using namespace std;


//----------------------------------------------------------
// Function to compute euler angles from rotation matrix
//----------------------------------------------------------
// 4 inputs:
// R: rotation matrix
// roll: roll angle of end effector
// pitch: pitch angle of end effector
// yaw: yaw angle of end effector
//----------------------------------------------------------
void rot_2_zyx(const Matrix3d& R, double& roll, double& pitch, double& yaw) {
    // Extracting angles from the rotation matrix
    yaw = atan2(R(1, 0), R(0, 0));
//    double cos_theta = sgn(R(1,0)) * sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2));
    double sin_theta = (1 / 0.994) * (-R(2, 0));
    if (sin_theta > 1) {
        sin_theta  = 1;
    }
    if (sin_theta < -1) {
        sin_theta = -1;
    }
    pitch = asin(sin_theta);
    if ((R(1,0) < 0) && (-R(2, 0) > 0)){
        pitch = M_PI - pitch;
    }
    if ((R(1,0) < 0) && (-R(2, 0) < 0)){
        pitch = -M_PI - pitch;
    }
    roll = atan2(R(2, 1), R(2, 2));
}


//----------------------------------------------------------
// Function to execute the forward kinematics
//----------------------------------------------------------
// 1 input:
// q: joint angular position
//
// 2 output:
// p: 6-DoF positions of end effector
// T_B7: rotation matrix of end effector
//----------------------------------------------------------
tuple<VectorXd, MatrixXd> Fwd_kinematics::forward(const VectorXd& q) {
    // Define transformation matrices
    Matrix4d T_B1, T_12, T_23, T_34, T_45, T_56, T_67, T_7end;

    T_B1 << cos(q(0)), -sin(q(0)), 0, 0,
            -sin(q(0)), -cos(q(0)), 0, 0,
            0, 0, -1, 0.1564,
            0, 0, 0, 1;

    T_12 << cos(q(1)), -sin(q(1)), 0, 0,
            0, 0, -1, 0.0054,
            sin(q(1)), cos(q(1)), 0, -0.1284,
            0, 0, 0, 1;

    T_23 << cos(q(2)), -sin(q(2)), 0, 0,
            0, 0, 1, -0.2104,
            -sin(q(2)), -cos(q(2)), 0, -0.0064,
            0, 0, 0, 1;

    T_34 << cos(q(3)), -sin(q(3)), 0, 0,
            0, 0, -1, 0.0064,
            sin(q(3)), cos(q(3)), 0, -0.2104,
            0, 0, 0, 1;

    T_45 << cos(q(4)), -sin(q(4)), 0, 0,
            0, 0, 1, -0.2084,
            -sin(q(4)), -cos(q(4)), 0, -0.0064,
            0, 0, 0, 1;

    T_56 << cos(q(5)), -sin(q(5)), 0, 0,
            0, 0, -1, 0,
            sin(q(5)), cos(q(5)), 0, -0.1059,
            0, 0, 0, 1;

    T_67 << cos(q(6)), -sin(q(6)), 0, 0,
            0, 0, 1, -0.1059,
            -sin(q(6)), -cos(q(6)), 0, 0,
            0, 0, 0, 1;

    T_7end << 1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, -1, -0.1815, //-0.067, <- original but we added 14mm offset to get to where grasp site is
            0, 0, 0, 1;

    // Compute the forward kinematics
    Matrix4d T_B7 = T_B1 * T_12 * T_23 * T_34 * T_45 * T_56 * T_67 * T_7end;

    VectorXd p(6);
    double roll, pitch, yaw;

    // Compute the 6-DoF positions of end effector
    VectorXd pos_end = T_B7.block<3, 1>(0, 3);
    Matrix3d rot_end = T_B7.block<3, 3>(0, 0);
    rot_2_zyx(T_B7.block<3, 3>(0, 0), roll, pitch, yaw);
    p << pos_end(0), pos_end(1), pos_end(2), roll, pitch, yaw;

    return make_tuple(p, T_B7);
}



DHParameters_Eigen Fwd_kinematics::createDHParamsEigen(const std::string& yaml_path) {

    DHParameters_Eigen dh_params;
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // Initialize vectors for 7 DOF
    dh_params.a = Eigen::VectorXd::Zero(7);
    dh_params.alpha = Eigen::VectorXd::Zero(7);
    dh_params.d = Eigen::VectorXd::Zero(7);
    dh_params.theta = Eigen::VectorXd::Zero(7);
    
    // Load DH parameters from YAML
    if (config["dh_parameters"]) {
        for (const auto& joint : config["dh_parameters"]) {
            int joint_id = joint["joint_id"].as<int>();
            // Convert to 0-based indexing
            int index = joint_id - 1;
            
            if (index >= 0 && index < 7) {
                dh_params.a(index) = joint["a"].as<double>();
                dh_params.alpha(index) = joint["alpha"].as<double>();
                dh_params.d(index) = joint["d"].as<double>();
                dh_params.theta(index) = joint["theta_offset"].as<double>();
            }
        }
    }

    return dh_params;
}

static Eigen::Matrix4d createDHTransform(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
   
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
   
    // Standard DH transformation: Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    T(0, 0) = cos_theta;
    T(0, 1) = -sin_theta * cos_alpha;
    T(0, 2) = sin_theta * sin_alpha;
    T(0, 3) = a * cos_theta;
   
    T(1, 0) = sin_theta;
    T(1, 1) = cos_theta * cos_alpha;
    T(1, 2) = -cos_theta * sin_alpha;
    T(1, 3) = a * sin_theta;
   
    T(2, 0) = 0;
    T(2, 1) = sin_alpha;
    T(2, 2) = cos_alpha;
    T(2, 3) = d;
   
    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;
   
    return T;
}

std::tuple<Eigen::VectorXd, Eigen::Matrix4d> Fwd_kinematics::forward(const DHParameters_Eigen& dh,
                                                     const Eigen::VectorXd& joint_angles) {
    // Hard-coded base pose: identity rotation but rotated 180 degrees about x-axis
    Eigen::Matrix4d default_base_pose = Eigen::Matrix4d::Identity();
    default_base_pose(1, 1) = -1;  // 180 degree rotation about x-axis
    default_base_pose(2, 2) = -1;
    
    Eigen::Matrix4d T_base_to_ee = Eigen::Matrix4d::Identity();
   
    // Chain all DH transformations from base to end effector
    for (int i = 0; i < 7; i++) {
        // Actual theta = theta_offset + joint_angle
        double theta_i = dh.theta(i) + joint_angles(i);
       
        // Create DH transformation for joint i
        Eigen::Matrix4d T_i = createDHTransform(dh.a(i), dh.alpha(i), dh.d(i), theta_i);
       
        // Compose with previous transformations
        T_base_to_ee = T_base_to_ee * T_i;
    }
   
    // Transform to world frame: T_world_to_ee = T_world_to_base * T_base_to_ee
    Eigen::Matrix4d T_world_to_ee = default_base_pose * T_base_to_ee;
    
    // Extract position and orientation
    Eigen::VectorXd pos_end = T_world_to_ee.block<3, 1>(0, 3);
    Eigen::Matrix3d rot_end = T_world_to_ee.block<3, 3>(0, 0);
    
    // Convert rotation matrix to roll, pitch, yaw using GTSAM for consistency
    gtsam::Rot3 gtsam_rotation(rot_end);
    gtsam::Vector3 rpy = gtsam_rotation.rpy();
    
    // Create 6DOF vector [x, y, z, roll, pitch, yaw]
    Eigen::VectorXd p(6);
    p << pos_end(0), pos_end(1), pos_end(2), rpy(0), rpy(1), rpy(2);
    
    return std::make_tuple(p, T_world_to_ee);
}