#include "Dynamics.h"
#include <iostream>
#include <cassert>
#include <cmath>

Dynamics::Dynamics(const std::string& urdf_path) {
    // Load the URDF model
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    
    // Initialize default gravity (world frame: z-up)
    gravity_world_ << 0.0, 0.0, -9.81;
    
    // Initialize base orientation (default: identity but rotated 180 degree about x-axis)
    R_world_to_base_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // Set initial gravity in model
    model_.gravity.linear() = gravity_world_;
    
    std::cout << "Model loaded successfully!" << std::endl;
    std::cout << "Number of joints: " << model_.njoints << std::endl;
    std::cout << "Number of DOFs: " << model_.nv << std::endl;
}

Eigen::VectorXd Dynamics::convertJointAnglesToConfig(const Eigen::VectorXd& joint_angles) {
    assert(joint_angles.size() == 7 && "Joint angles must be 7-dimensional");
    
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);  // Size 11
    
    // Based on URDF joint order and types:
    // Actuator1: continuous -> cos, sin
    // Actuator2: revolute -> angle  
    // Actuator3: continuous -> cos, sin
    // Actuator4: revolute -> angle
    // Actuator5: continuous -> cos, sin
    // Actuator6: revolute -> angle
    // Actuator7: continuous -> cos, sin
    
    int q_idx = 0;
    
    // Actuator1 (continuous)
    q(q_idx++) = std::cos(joint_angles(0));
    q(q_idx++) = std::sin(joint_angles(0));
    
    // Actuator2 (revolute)
    q(q_idx++) = joint_angles(1);
    
    // Actuator3 (continuous)
    q(q_idx++) = std::cos(joint_angles(2));
    q(q_idx++) = std::sin(joint_angles(2));
    
    // Actuator4 (revolute)
    q(q_idx++) = joint_angles(3);
    
    // Actuator5 (continuous)
    q(q_idx++) = std::cos(joint_angles(4));
    q(q_idx++) = std::sin(joint_angles(4));
    
    // Actuator6 (revolute)
    q(q_idx++) = joint_angles(5);
    
    // Actuator7 (continuous)
    q(q_idx++) = std::cos(joint_angles(6));
    q(q_idx++) = std::sin(joint_angles(6));
    
    return q;
}

Eigen::VectorXd Dynamics::convertConfigToJointAngles(const Eigen::VectorXd& q) {
    assert(q.size() == model_.nq && "Configuration must match model.nq");
    
    Eigen::VectorXd joint_angles = Eigen::VectorXd::Zero(7);
    
    int q_idx = 0;
    
    // Actuator1 (continuous): atan2(sin, cos)
    joint_angles(0) = std::atan2(q(q_idx+1), q(q_idx));
    q_idx += 2;
    
    // Actuator2 (revolute): direct angle
    joint_angles(1) = q(q_idx++);
    
    // Actuator3 (continuous): atan2(sin, cos)
    joint_angles(2) = std::atan2(q(q_idx+1), q(q_idx));
    q_idx += 2;
    
    // Actuator4 (revolute): direct angle
    joint_angles(3) = q(q_idx++);
    
    // Actuator5 (continuous): atan2(sin, cos)
    joint_angles(4) = std::atan2(q(q_idx+1), q(q_idx));
    q_idx += 2;
    
    // Actuator6 (revolute): direct angle
    joint_angles(5) = q(q_idx++);
    
    // Actuator7 (continuous): atan2(sin, cos)
    joint_angles(6) = std::atan2(q(q_idx+1), q(q_idx));
    q_idx += 2;
    
    return joint_angles;
}

void Dynamics::setBaseOrientation(const Eigen::Matrix3d& R_world_to_base) {
    // Verify it's a valid rotation matrix
    double det = R_world_to_base.determinant();
    bool orthogonal = (R_world_to_base * R_world_to_base.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-6);
    
    // if (!std::abs(det - 1.0) < 1e-6 || !orthogonal) {
    //     std::cout << "Warning: Provided matrix is not a valid rotation matrix!" << std::endl;
    // }
    
    R_world_to_base_ = R_world_to_base;
    
    // Update gravity vector in the model
    Eigen::Vector3d gravity_base = -(R_world_to_base * gravity_world_);
    model_.gravity.linear() = gravity_base;
    
    // std::cout << "Base orientation updated. Gravity in base frame: [" 
    //           << gravity_base.transpose() << "]" << std::endl;
}

void Dynamics::setBaseOrientationRPY(double roll, double pitch, double yaw) {
    // Convert RPY to rotation matrix (ZYX convention)
    Eigen::Matrix3d R_x, R_y, R_z;
    
    R_x << 1, 0, 0,
           0, std::cos(roll), -std::sin(roll),
           0, std::sin(roll), std::cos(roll);
    
    R_y << std::cos(pitch), 0, std::sin(pitch),
           0, 1, 0,
           -std::sin(pitch), 0, std::cos(pitch);
    
    R_z << std::cos(yaw), -std::sin(yaw), 0,
           std::sin(yaw), std::cos(yaw), 0,
           0, 0, 1;
    
    Eigen::Matrix3d R_world_to_base = R_x * R_y * R_z;
    setBaseOrientation(R_world_to_base);
}

Eigen::Vector3d Dynamics::getCurrentGravityBase() const {
    return R_world_to_base_ * gravity_world_;
}

Eigen::Matrix3d Dynamics::getBaseOrientation() const {
    return R_world_to_base_;
}

Eigen::MatrixXd Dynamics::mass_m(const Eigen::VectorXd& q) {
    // Ensure q has correct size (should be model.nq for continuous joints)
    assert(q.size() == model_.nq && "Joint configuration size must match model.nq");
    
    // Compute mass matrix using Composite Rigid Body Algorithm (CRBA)
    pinocchio::crba(model_, data_, q);
    
    // Extract upper triangular part and make it symmetric
    data_.M.triangularView<Eigen::StrictlyLower>() = 
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    
    return data_.M;
}

Eigen::VectorXd Dynamics::gravity_m(const Eigen::VectorXd& q) {
    // Ensure q has correct size
    assert(q.size() == model_.nq && "Joint configuration size must match model.nq");
    
    // Zero velocity and acceleration for gravity computation
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
    
    // Compute gravity using RNEA with zero velocity and acceleration
    // The gravity is already set in the model via setBaseOrientation()
    return pinocchio::rnea(model_, data_, q, v, a);
}

Eigen::MatrixXd Dynamics::coriolis_m_simplified(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    // Ensure correct sizes
    assert(q.size() == model_.nq && v.size() == model_.nv && "q must match model.nq and v must match model.nv");
    
    // Method 1: Using computeCoriolisMatrix (if available)
    // pinocchio::computeCoriolisMatrix(model_, data_, q, v);
    // return data_.C;
    
    // Method 2: Numerical computation of Coriolis matrix
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
    
    // Compute mass matrix
    pinocchio::crba(model_, data_, q);
    data_.M.triangularView<Eigen::StrictlyLower>() = 
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M = data_.M;
    
    // Compute Coriolis and centrifugal forces
    Eigen::VectorXd zero_acc = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd coriolis_forces = pinocchio::rnea(model_, data_, q, v, zero_acc);
    
    // Subtract gravity to get pure Coriolis forces
    Eigen::VectorXd gravity = gravity_m(q);
    coriolis_forces -= gravity;
    
    // Alternative: Use the fact that C*v = h - g where h is computed by RNEA
    // For the full Coriolis matrix, we can use finite differences or the christoffel symbols
    
    // Method 3: Using christoffel symbols (more accurate)
    pinocchio::computeJointJacobians(model_, data_, q);
    
    // This is a simplified version - for full implementation you'd need to compute
    // the partial derivatives of the mass matrix
    for (int i = 0; i < model_.nv; ++i) {
        for (int j = 0; j < model_.nv; ++j) {
            C(i, j) = 0.0; // Placeholder - actual computation involves christoffel symbols
        }
    }
    
    return C;
}

Eigen::MatrixXd Dynamics::coriolis_m(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    assert(q.size() == model_.nq && v.size() == model_.nv);
    
    // Use Pinocchio's built-in Coriolis matrix computation
    pinocchio::computeCoriolisMatrix(model_, data_, q, v);
    
    return data_.C;
}

void Dynamics::computeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    pinocchio::crba(model_, data_, q);
    data_.M.triangularView<Eigen::StrictlyLower>() = 
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::computeGeneralizedGravity(model_, data_, q);
    pinocchio::computeCoriolisMatrix(model_, data_, q, v);
}

Eigen::VectorXd Dynamics::getNeutralConfiguration() {
    Eigen::VectorXd q_full = pinocchio::neutral(model_);
    
    // For models with continuous joints, q has size model.nq (not model.nv)
    // model.nq accounts for the 2-component representation of continuous joints
    // while model.nv is the actual degrees of freedom
    
    std::cout << "Full configuration size: " << q_full.size() << " (model.nq = " << model_.nq << ")" << std::endl;
    
    // Return the full configuration vector - it should be size model.nq
    return q_full;
}

Eigen::VectorXd Dynamics::getRandomConfiguration() {
    Eigen::VectorXd q_full = pinocchio::randomConfiguration(model_);
    
    // Extract only the actuated DOFs
    Eigen::VectorXd q_actuated = Eigen::VectorXd::Zero(model_.nv);
    
    if (model_.nv == 7 && q_full.size() >= 7) {
        q_actuated = q_full.head(7);
    } else {
        // Fallback: create random configuration directly
        q_actuated = Eigen::VectorXd::Random(model_.nv);
    }
    
    return q_actuated;
}

void Dynamics::printModelInfo() {
    std::cout << "=== Kinova Gen3 Model Information ===" << std::endl;
    std::cout << "Number of joints: " << model_.njoints << std::endl;
    std::cout << "Number of DOFs: " << model_.nv << std::endl;
    std::cout << "Joint names:" << std::endl;
    for (int i = 1; i < model_.njoints; ++i) {
        std::cout << "  " << i << ": " << model_.names[i] << std::endl;
    }
    
    // Print current gravity and base orientation
    Eigen::Vector3d gravity_base = getCurrentGravityBase();
    std::cout << "Current gravity in base frame: [" << gravity_base.transpose() << "] m/s²" << std::endl;
    std::cout << "Base orientation matrix (world to base):" << std::endl;
    std::cout << R_world_to_base_ << std::endl;
}