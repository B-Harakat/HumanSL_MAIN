//----------------------------------------------------------
// Functions of the impedance controller
//----------------------------------------------------------
// Description: functions used in the impedance controller
// Copyright: Yihan Liu 2024
//----------------------------------------------------------

#define _USE_MATH_DEFINES

#include "Controller.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <tuple>

#include <Jacobian.h>
#include <Fwd_kinematics.h>
#include <Dynamics.h>
#include <Filter.h>

using namespace Eigen;
using namespace std;
using namespace Jacobian;
using namespace Fwd_kinematics;
using namespace Filter;

// Buffers to store the last variables
Vector<double, 6> last_dp = Vector<double, 6>::Zero();
Matrix<double, 6, 7> last_jaco = Matrix<double, 6, 7>::Zero();
Matrix3d last_rot = Matrix<double, 3, 3>::Zero();
Vector3d last_pos = Vector<double, 3>::Zero();
Vector<double, 6> last_vel = Vector<double, 6>::Zero();
Vector<double, 6> acc_factor_buffer = Vector<double, 6>::Zero();

//----------------------------------------------------------------
// Function to transform the euler angle to the rotation matrix
//----------------------------------------------------------------
// 3 inputs
// roll: X-axis rotation angle
// pitch: Y-axis rotation angle
// yaw: Z-axis rotation angle
//
// 1 output
// R: rotation matrix
//----------------------------------------------------------------
Matrix3d euler_to_rotation_matrix(double roll, double pitch, double yaw) {
    // Rotation matrix for roll (X-axis rotation)
    Matrix3d R_x;
    R_x << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);

    // Rotation matrix for pitch (Y-axis rotation)
    Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);

    // Rotation matrix for yaw (Z-axis rotation)
    Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

    // Combined rotation matrix (with ZYX rotation)
    Matrix3d R = R_z * R_y * R_x;

    return R;
}


//--------------------------------------------------------------
// Function to convert the Euler angle velocity in the end frame
// into the angular velocity in the base frame
//--------------------------------------------------------------
// 2 inputs:
// euler_ang: 3 Euler angles in the end frame
// euler_vel: 3 Euler velocities in the end frame
//
// 1 output:
// omega_base: 3 angular velocities in the base frame
//--------------------------------------------------------------
Vector3d euler_vel_to_base_angular_vel(
        const Vector3d& euler_ang,  // [theta_x, theta_y, theta_z]
        const Vector3d& euler_vel  // [dtheta_x, dtheta_y, dtheta_z]
) {
    // Extract Euler angles
    double theta_x = euler_ang[0];
    double theta_y = euler_ang[1];
    double theta_z = euler_ang[2];

    // Compute the transformation matrix T_Euler^ZYX
    Matrix3d T_euler_ZYX;
    T_euler_ZYX <<
                1, 0, -sin(theta_y),
            0, cos(theta_x), cos(theta_y) * sin(theta_x),
            0, -sin(theta_x), cos(theta_y) * cos(theta_x);

    // Compute angular velocity in the end frame
    Vector3d omega_end = T_euler_ZYX * euler_vel;

    Matrix3d R = euler_to_rotation_matrix(theta_x, theta_y, theta_z);

    // Transform angular velocity from body frame to world frame
    Vector3d omega_base = R * omega_end;

    return omega_base;
}


//------------------------------------------------------------------------
// Function to compute the time derivative of the rotation matrix R
//------------------------------------------------------------------------
// 2 inputs:
// euler_ang: 3 Euler angles in the end frame
// euler_vel: 3 Euler velocities in the end frame
//
// 1 output:
// R_dot: time derivative of the rotation matrix
//------------------------------------------------------------------------
Matrix3d rot_matrix_dev(const Vector3d& euler_ang, const Vector3d& euler_vel) {
    // Euler angles
    double theta_x = euler_ang[0];
    double theta_y = euler_ang[1];
    double theta_z = euler_ang[2];

    // Euler velocities
    double dtheta_x = euler_vel[0];
    double dtheta_y = euler_vel[1];
    double dtheta_z = euler_vel[2];

    // Precompute sines and cosines
    double cx = cos(theta_x), sx = sin(theta_x);
    double cy = cos(theta_y), sy = sin(theta_y);
    double cz = cos(theta_z), sz = sin(theta_z);

    // Compute the time derivative of the rotation matrix R
    Matrix3d R_dot;
    R_dot << -sz * cy * dtheta_z - cz * sy * dtheta_y,
            -sz * sy * sx * dtheta_z - cz * (cy * sx * dtheta_y + sy * cx * dtheta_x) - cz * cx * dtheta_z,
            -sz * sy * cx * dtheta_z - cz * (cy * cx * dtheta_y - sy * sx * dtheta_x) + cz * sx * dtheta_z,

            cz * cy * dtheta_z - sz * sy * sx * dtheta_z,
            cz * sy * sx * dtheta_z - sz * (cy * sx * dtheta_y + sy * cx * dtheta_x) + sz * cx * dtheta_z,
            cz * sy * cx * dtheta_z - sz * (cy * cx * dtheta_y - sy * sx * dtheta_x) - sz * sx * dtheta_z,

            0, -cy * sx * dtheta_y - sy * cx * dtheta_x, -cy * cx * dtheta_y + sy * sx * dtheta_x;

    return R_dot;
}


//----------------------------------------------------------------
// Function to compute angular acceleration in the base frame
//----------------------------------------------------------------
// 3 inputs:
// euler_ang: 3 Euler angles in the end frame
// euler_vel: 3 Euler velocities in the end frame
// euler_acc: 3 Euler accelerations in the end frame
//
// 1 output:
// alpha_base: angular accelerations in the base frame
//-----------------------------------------------------------------
Vector3d euler_acc_to_base_angular_acc(
        const Vector3d& euler_ang,       // [theta_x, theta_y, theta_z]
        const Vector3d& euler_vel,       // [dtheta_x, dtheta_y, dtheta_z]
        const Vector3d& euler_acc        // [ddtheta_x, ddtheta_y, ddtheta_z]
) {
    // Compute the transformation matrix T_Euler^ZYX
    Matrix3d T_euler_ZYX;
    double theta_x = euler_ang[0];
    double theta_y = euler_ang[1];

    T_euler_ZYX <<
                1, 0, -sin(theta_y),
            0, cos(theta_x), cos(theta_y) * sin(theta_x),
            0, -sin(theta_x), cos(theta_y) * cos(theta_x);

    // Angular velocity in the end frame
    Vector3d omega_base = T_euler_ZYX * euler_vel;

    // Compute the time derivative of the angular velocity in the end frame
    Vector3d alpha_end = T_euler_ZYX * euler_acc; // First part

    // Compute the time derivative of the rotation matrix
    Matrix3d R_dot = rot_matrix_dev(euler_ang, euler_vel);

    // Compute the rotation matrix from ZYX Euler angles
    Matrix3d R = euler_to_rotation_matrix(euler_ang[0], euler_ang[1], euler_ang[2]);

    // Compute angular acceleration in the base frame
    Vector3d alpha_base = R * alpha_end + R_dot * omega_base;

    return alpha_base;
}



//------------------------------------------------------------
// Scale function for the friction compensation (to avoid rotation with a small velocity fluctuation around 0)
//------------------------------------------------------------
// 2 inputs:
// k: function slope
// dq: joint angular velocity
//
// output: scale factor for firction compensation
//----------------------------------------------------------
double scale_fric(const double k, double dq) {
    return 2.0 / (1.0 + exp(-k * dq)) - 1.0;
}


//-----------------------------------------------------------
// Function to calculate velocity gains for friction compensation
//-----------------------------------------------------------
// 2 inputs:
// dq: joint angular velocity
// index: number of joint
//
// output: joint velocity gain to compensate friction
//-----------------------------------------------------------
double vel_gain_fric(double dq, int index){
    VectorXd gain_posi(7), gain_nega(7);
    // Gains for 7 joints with different signs (positive or negative)
    gain_posi << 9, 8, 9, 8.5, 9, 8, 9.5;
    gain_nega << 9, 8, 9, 8.5, 9, 8, 9;

    if (dq >= 0){
        return gain_posi(index);
    }
    else{
        return gain_nega(index);
    }
}

//-----------------------------------------------------------
// Function to compute position and orientation errors
//-----------------------------------------------------------
// 5 inputs:
// pos_end: XYZ positions of end effector
// pos_d: desired XYZ position of end effector
// rot_end: rotation matrix of end effector
// rot_d: desired rotation matrix of end effector
// e: 6-DoF errors used in the controller
//----------------------------------------------------------
void pos_difference(Vector3d& pos_end, Vector3d& pos_d, Matrix3d& rot_end, Matrix3d& rot_d, VectorXd& e){

    // 3-DoF XYZ position errors
    Vector3d pos_e;
    pos_e = pos_end - pos_d;
    e.head<3>() = pos_e;

    // 3-DoF orientation errors
    Quaterniond orientation(rot_end);
    Quaterniond orientation_d(rot_d);
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // Quaternion difference
    Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    e.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    e.tail(3) << 2 * (-rot_end * e.tail(3));
}

void friction_compensation(VectorXd& u, VectorXd& dq){
    VectorXd offset(7), gain(7);
    offset << 0.5, 0.6, 0.45, 0.4, 0.4, 0.3, 0.5;

    u(0) += scale_fric(20, dq(0)) * (offset(0) + vel_gain_fric(dq(0), 0) * abs(dq(0)));
    u(1) += scale_fric(20, dq(1)) * (offset(1) + vel_gain_fric(dq(1),1) * abs(dq(1)));
    u(2) += scale_fric(20, dq(2)) * (offset(2) + vel_gain_fric(dq(2),2) * abs(dq(2)));
    u(3) += scale_fric(20, dq(3)) * (offset(3) + vel_gain_fric(dq(3),3) * abs(dq(3)));
    u(4) += 1.1 + scale_fric(40, dq(4)) * (offset(4) + vel_gain_fric(dq(4),4) * abs(dq(4)));
    u(5) += scale_fric(40, dq(5)) * (offset(5) + vel_gain_fric(dq(5),5) * abs(dq(5)));
    u(6) += 1.2 + scale_fric(70, dq(6)) * (offset(6) + vel_gain_fric(dq(6),6) * abs(dq(6)));
}


//-----------------------------------------------------------
// Function to compute control input
//-----------------------------------------------------------
// 10 inputs:
// q,dq,ddq: joint angular positions, velocities and accelerations
// T_B7: rotation matrix of end effector
// p_d,dp_d, ddp_d: desire positions, velocities and accelerations
// K_d_diag: diagonal values of desired stiffness
// K_n_diag: diagonal values of nullspace stiffness
// c_f: control frequency
// time period: time difference between two control loop
//
// 3 outputs:
// u: control input
// dp: velocities of end effector
// ddp: accelerations of end effector
//----------------------------------------------------------
tuple<VectorXd, VectorXd, VectorXd, VectorXd> Controller::task_impedance_controller(Dynamics &robot, VectorXd& q, VectorXd& dq, VectorXd& ddq, MatrixXd& T_B7,
                                                           VectorXd& p_d, VectorXd& dp_d, VectorXd& ddp_d,
                                                           VectorXd& K_d_diag, VectorXd& K_n_diag,
                                                           int c_f, double& time_period){
    // Outputs
    VectorXd u(7), dp(6), ddp(6);

    // Compute the stiffnesses and dampings
    Vector<double, 6> D_d_diag = 2 * K_d_diag.array().sqrt();
    Vector<double, 7> D_n_diag = 2 * K_n_diag.array().sqrt();
    DiagonalMatrix<double, 6> D_d = D_d_diag.asDiagonal();
    DiagonalMatrix<double, 6> K_d = K_d_diag.asDiagonal();
    DiagonalMatrix<double, 7> D_n = D_n_diag.asDiagonal();
    DiagonalMatrix<double, 7> K_n = K_n_diag.asDiagonal();

    // Define the acceleration amplification factor
    VectorXd acc_amp_factor_diag(6);
    DiagonalMatrix<double, Dynamic> acc_amp_factor(6);
    acc_amp_factor.setZero(); // Set all diagonal elements to zero

    // Parameters for Null space
    MatrixXd Iden = MatrixXd::Identity(7, 7);
    MatrixXd Null(7, 7);   // Nullspace projection matrix
    VectorXd e_q(7);          // Joint position errors in nullspace
    VectorXd q_d(7); // null space desired joint angles

    // Joint torques limits
    VectorXd gen3_JointTorquesLimits(7);
    gen3_JointTorquesLimits << 50, 50, 50, 50, 25, 25, 25;

    // Define the model matrices used in controller
    // MatrixXd mass_matrix(7,7);
    // VectorXd gravity_matrix(7);
    // MatrixXd coriolis_matrix(7,7);
    MatrixXd jacobian(6,7);
    MatrixXd jacobian_dot(6, 7);
    MatrixXd pinv_jacobian(7, 6);
    MatrixXd jacobian_T(7, 6);
    MatrixXd pinv_jacobian_T(6, 7);
    MatrixXd M_x(6,6), C_x(6, 6);
    Matrix3d rot_end(3,3), rot_d(3,3);
    Vector3d pos_d(3), pos_end(3), pos_e(3), rot_e(3);
    VectorXd e(6), e_dot(6), g_x(6);
    VectorXd force(6), tau1(7), tau2(7), tau3(7);
    VectorXd model_e(7);
    VectorXd D_f_diag(6), K_f_diag(6);
    VectorXd acc_amplification(6);
    VectorXd torque_amp_diag(6);

    // Dynamic matrices calculations
    robot.computeDynamics(robot.convertJointAnglesToConfig(q), dq);
    const auto& mass_matrix = robot.data_.M;
    const auto& gravity_matrix = robot.data_.g;  
    const auto& coriolis_matrix = robot.data_.C;
    
    // // mass_matrix = robot.mass_m(robot.convertJointAnglesToConfig(q));
    // std::cout << "=========== Dynamics Info ==========" << std::endl;
    // std::cout << q << std::endl;
    // std::cout << mass_matrix << std::endl;
    // // gravity_matrix = robot.gravity_m(robot.convertJointAnglesToConfig(q));
    // std::cout << gravity_matrix << std::endl;
    // // coriolis_matrix = robot.coriolis_m(robot.convertJointAnglesToConfig(q),dq);
    // std::cout << coriolis_matrix << std::endl;
    // std::cout << "============= End Info =============" << std::endl;
    jacobian = jaco_m(q);
    jacobian_dot = (jacobian - last_jaco) / time_period;
    last_jaco = jacobian;
    pinv_jacobian = (jacobian).completeOrthogonalDecomposition().pseudoInverse();
    jacobian_T = jacobian.transpose();
    pinv_jacobian_T = pinv_jacobian.transpose();

    // Dynamic matrices in task space
    M_x = pinv_jacobian_T * mass_matrix * pinv_jacobian;
    C_x = pinv_jacobian_T * (coriolis_matrix - mass_matrix * pinv_jacobian * jacobian_dot) * pinv_jacobian;
    g_x = pinv_jacobian_T * gravity_matrix;

    // Transformation from joint to task space
    dp = jacobian * dq;
    pos_end = T_B7.block<3, 1>(0, 3);
    rot_end = T_B7.block<3, 3>(0, 0);

    // Position error
    pos_d = p_d.head<3>();
    rot_d = euler_to_rotation_matrix(p_d(3), p_d(4), p_d(5));
    pos_difference(pos_end, pos_d, rot_end, rot_d, e);

    // Compute acceleration
    VectorXd pos_error(6), current_vel(6);
    pos_difference(pos_end, last_pos, rot_end, last_rot, pos_error);
    last_pos = pos_end;
    last_rot = rot_end;
    current_vel = pos_error * c_f;
    ddp = (current_vel - last_vel) * c_f;
    last_vel = current_vel;
    ddp = butterworth_filter(ddp);

    // Transform the desired velocity and acceleration into world frame
    Vector3d ang_rate_world = euler_vel_to_base_angular_vel(p_d.tail<3>(),dp_d.tail<3>());
    dp_d.tail<3>() = ang_rate_world;
    Vector3d ang_acc_world = euler_acc_to_base_angular_acc(p_d.tail<3>(), dp_d.tail<3>(), ddp_d.tail<3>());
    ddp_d.tail<3>() = ang_acc_world;

    // Velocity error
    e_dot = dp - dp_d;

    // Tool gravity compensation (change with tool gravity)
    // g_x(2) += 0.58*9.81; // tool handled by urdf instead now

    VectorXd dq_d(7);
    force = M_x*ddp_d + C_x*dp + g_x - K_d * e - D_d * e_dot;

    // Control law 1
    tau1 = jacobian_T * force;

    // Nullspace (Control law 2)
    dq_d = pinv_jacobian * dp_d;
    q_d = q + dq_d * (1 / c_f);
    Null = Iden - jacobian_T*pinv_jacobian_T;
    e_q = q_d - q;
    tau2 = Null*(K_n*e_q - D_n*dq);

    // Control input
    u = tau1 + tau2;

    friction_compensation(u, dq);

    // Set the torque saturation
    for (int i = 0; i < 7; i++)
    {
        if(u[i] > 0.9*gen3_JointTorquesLimits[i]){
            u[i] = 0.9*gen3_JointTorquesLimits[i];
        }else if(u[i] < -0.9 * gen3_JointTorquesLimits[i]){
            u[i] = -0.9 * gen3_JointTorquesLimits[i];
        }
    }

    return make_tuple(u, dp, ddp, acc_factor_buffer);
}


//-----------------------------------------------------------
// Function to initilize the controller
//-----------------------------------------------------------
// 2 inputs:
// pos: initial position
// T_B7: initial rotation matrix
//-----------------------------------------------------------
void Controller::ini_controller(Vector3d& pos, MatrixXd& T_B7){
    last_pos << pos;
    last_rot = T_B7.block<3, 3>(0, 0);
    last_vel = Vector<double, 6>::Zero();
}

VectorXd Controller::joint_impedance_controller(Dynamics &robot, VectorXd& q, VectorXd& dq, VectorXd& ddq,
                                                      VectorXd& q_d, VectorXd& dq_d, VectorXd& ddq_d,
                                                      VectorXd& K_joint_diag, VectorXd& K_integral_diag,
                                                      int c_f, double& time_period){
    // Output
    VectorXd u(7);

    // Thread-local variable to accumulate integral error
    static thread_local VectorXd integral_error = VectorXd::Zero(7);

    // Compute the joint stiffness, damping, and integral matrices
    Vector<double, 7> D_joint_diag = 2 * K_joint_diag.array().sqrt();
    DiagonalMatrix<double, 7> D_joint = D_joint_diag.asDiagonal();
    DiagonalMatrix<double, 7> K_joint = K_joint_diag.asDiagonal();
    DiagonalMatrix<double, 7> K_integral = K_integral_diag.asDiagonal();

    // Joint torques limits
    VectorXd gen3_JointTorquesLimits(7);
    gen3_JointTorquesLimits << 50,50,50,50,25,25,25;

    // Define the model matrices used in controller
    // MatrixXd mass_matrix(7,7);
    // VectorXd gravity_matrix(7);
    // MatrixXd coriolis_matrix(7,7);

    robot.computeDynamics(robot.convertJointAnglesToConfig(q), dq);
    const auto& mass_matrix = robot.data_.M;
    const auto& gravity_matrix = robot.data_.g;  
    const auto& coriolis_matrix = robot.data_.C;

    // // Dynamic matrices calculations
    // // mass_matrix = robot.mass_m(robot.convertJointAnglesToConfig(q));
    // std::cout << "=========== Dynamics Info ==========" << std::endl;
    // std::cout << q << std::endl;
    // std::cout << mass_matrix << std::endl;
    // // gravity_matrix = robot.gravity_m(robot.convertJointAnglesToConfig(q));
    // std::cout << gravity_matrix << std::endl;
    // // coriolis_matrix = robot.coriolis_m(robot.convertJointAnglesToConfig(q),dq);
    // std::cout << coriolis_matrix << std::endl;
    // std::cout << "============= End Info =============" << std::endl;

    // Joint space errors
    VectorXd e_q = q_d - q;           // Position error
    VectorXd e_dq = dq_d - dq;        // Velocity error
    
    // Apply error saturation filter (±5 degrees = ±0.0873 radians)
    const double error_saturation = 1.0 * M_PI / 180.0;  // 5 degrees in radians
    for (int i = 0; i < e_q.size(); i++) {
        if (e_q[i] > error_saturation) {
            e_q[i] = error_saturation;
        } else if (e_q[i] < -error_saturation) {
            e_q[i] = -error_saturation;
        }
    }
    
    // Apply deadband filter (0.2 degrees = 0.00349 radians)
    // const double deadband = 0.01 * M_PI / 180.0;  // 0.2 degrees in radians
    // for (int i = 0; i < e_q.size(); i++) {
    //     if (abs(e_q[i]) < deadband) {
    //         e_q[i] = 0.0;
    //     }
    // }
    
    // Update integral error (accumulate position error over time)
    integral_error += e_q * time_period;

    // Control input
    // u = mass_matrix * ddq_d + coriolis_matrix * dq + gravity_matrix + K_joint * e_q + D_joint * e_dq + K_integral * integral_error;
    // u = mass_matrix * ddq_d + coriolis_matrix * dq + gravity_matrix + K_joint * e_q + D_joint * e_dq + K_integral * integral_error;

    std::cout << "gravity force: ";
    for(auto& i : gravity_matrix){std::cout << i << ", ";}
    std::cout << "\n";

    u = (K_joint * e_q) + (D_joint * e_dq) + (K_integral * integral_error);
    // u[6] = K_joint.diagonal()[6] * e_q[6] + D_joint.diagonal()[6] * e_dq[6] + K_integral.diagonal()[6] * integral_error[6];
    // u = gravity + K_diag * eq + K_I * integral error (in position)

    // Add friction compensation (// remove for KI)
    // friction_compensation(u, dq);

    // Set the torque saturation
    for (int i = 0; i < 7; i++)
    {
        if(u[i] > 0.9*gen3_JointTorquesLimits[i]){
            u[i] = 0.9* gen3_JointTorquesLimits[i];
        }else if(u[i] < -0.9*gen3_JointTorquesLimits[i]){
            u[i] = -0.9*gen3_JointTorquesLimits[i];
        }
    }

    // std::cout << "Joint torques: ";
    // for (int i = 0; i < u.size(); i++){
    //     std::cout << std::fixed << std::setprecision(3) << u[i];
    //     if (i < u.size() - 1) std::cout << ", ";
    // }
    // std::cout << std::endl;

    return u;
}


VectorXd Controller::chicken_head_impedance_controller(Dynamics& robot, 
                              VectorXd& q, VectorXd& dq, MatrixXd& T_B7,
                              VectorXd& p_d, VectorXd& K_d_diag, 
                              double dt, bool& first_call) {
    
    // Initialize outputs and variables
    VectorXd u(7);
    VectorXd dp(6), e(6), e_dot(6);
    VectorXd q_prev(7);
    
    
    // Compute stiffness and damping matrices
    Vector<double, 6> D_d_diag = 2 * K_d_diag.array().sqrt();  // Critical damping
    DiagonalMatrix<double, 6> D_d = D_d_diag.asDiagonal();
    DiagonalMatrix<double, 6> K_d = K_d_diag.asDiagonal();
    
    // Compute robot dynamics
    robot.computeDynamics(robot.convertJointAnglesToConfig(q), dq);
    const auto& mass_matrix = robot.data_.M;
    const auto& gravity_matrix = robot.data_.g;
    const auto& coriolis_matrix = robot.data_.C;
    
    // Compute Jacobian and related matrices
    
    static thread_local MatrixXd last_jaco(6,7);

    if(first_call){
        q_prev = q - dq*dt;
        last_jaco = jaco_m(q_prev);
    }
    
    MatrixXd jacobian = jaco_m(q);  // 6x7 Jacobian
    MatrixXd jacobian_dot = (jacobian - last_jaco) / dt;
    last_jaco = jacobian;
    
    MatrixXd pinv_jacobian = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    MatrixXd jacobian_T = jacobian.transpose();
    MatrixXd pinv_jacobian_T = pinv_jacobian.transpose();
    
    // Task space dynamics
    MatrixXd M_x = pinv_jacobian_T * mass_matrix * pinv_jacobian;
    MatrixXd C_x = pinv_jacobian_T * (coriolis_matrix - mass_matrix * pinv_jacobian * jacobian_dot) * pinv_jacobian;
    VectorXd g_x = pinv_jacobian_T * gravity_matrix;
    
    // Current end-effector state
    Vector3d pos_end = T_B7.block<3, 1>(0, 3);
    Matrix3d rot_end = T_B7.block<3, 3>(0, 0);
    dp = jacobian * dq;  // Current Cartesian velocity

    // dp = butterworth_filter_pose(dp);
    
    // Desired pose (stationary target)
    Vector3d pos_d = p_d.head<3>();
    Matrix3d rot_d = euler_to_rotation_matrix(p_d(3), p_d(4), p_d(5));
    
    // Compute 6-DOF pose error
    pos_difference(pos_end, pos_d, rot_end, rot_d, e);


    // Apply deadband to error
    // const double deadband_pos = 0.02;  // 1cm for position, 0.01 rad for orientation
    // const double deadband_rot = 0.05;
    // for (int i = 0; i < e.size(); i++) {
    //     if(i < 3){
    //         if (abs(e(i)) < deadband_pos) {
    //             e(i) = 0.0;
    //         }
    //     }
    //     else{
    //         if (abs(e(i)) < deadband_rot) {
    //             e(i) = 0.0;
    //         }
    //     }
    // }
    
    // Thread-local variable to store integral error
    static thread_local VectorXd e_integral = VectorXd::Zero(6);
    
    // Create integral gain matrix
    Vector<double, 6> K_i_diag = 0.008 * K_d_diag.array();  // Integral gain (5% of proportional)
    DiagonalMatrix<double, 6> K_i = K_i_diag.asDiagonal();
    
    // Accumulate integral error
    e_integral += e * dt;
    
    // PI Control Law (no velocity term for mobile platform)
    // For stationary target: ddp_d = 0, dp_d = 0
    VectorXd force = -(K_d*e) + (K_i*e_integral);
    
    // Map Cartesian forces to joint torques
    u = jacobian_T * force;
    
    // Apply friction compensation (reuse from original code)
    friction_compensation(u, dq);
    
    // Torque saturation
    VectorXd gen3_JointTorquesLimits(7);
    gen3_JointTorquesLimits << 50, 50, 50, 50, 25, 25, 25;
    
    for (int i = 0; i < 7; i++) {
        if(u[i] > 0.9*gen3_JointTorquesLimits[i]){
            u[i] = 0.9*gen3_JointTorquesLimits[i];
        }else if(u[i] < -0.9 * gen3_JointTorquesLimits[i]){
            u[i] = -0.9 * gen3_JointTorquesLimits[i];
        }
    }
    
    return u;
}

VectorXd Controller::chicken_head_velocity_controller(Dynamics& robot,
    VectorXd& q, VectorXd& dq, MatrixXd& T_B7,
    VectorXd& p_d, VectorXd& K_p_diag,
    double dt) {
    
    // Initialize outputs and variables
    VectorXd dp(6), e(6);
    
    // Thread-local variable to store integral error
    static thread_local VectorXd e_buffer = VectorXd::Zero(6);
    
    // Create diagonal gain matrices
    Vector<double, 6> K_i_diag = 0.05 * K_p_diag.array();  // Integral gain
    
    DiagonalMatrix<double, 6> K_p = K_p_diag.asDiagonal();
    DiagonalMatrix<double, 6> K_i = K_i_diag.asDiagonal();
    
    // Compute Jacobian
    MatrixXd jacobian = jaco_m(q);
    MatrixXd pinv_jacobian = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    
    // Current end-effector state
    Vector3d pos_end = T_B7.block<3, 1>(0, 3);
    Matrix3d rot_end = T_B7.block<3, 3>(0, 0);
    // dp = jacobian * dq;
    
    // Desired pose
    Vector3d pos_d = p_d.head<3>();
    Matrix3d rot_d = euler_to_rotation_matrix(p_d(3), p_d(4), p_d(5));
    
    // Compute pose error
    pos_difference(pos_end, pos_d, rot_end, rot_d, e);
    
    // Apply deadband to error
    const double deadband_pos = 0.02;  // 1cm for position, 0.01 rad for orientation
    const double deadband_rot = 0.05;
    for (int i = 0; i < e.size(); i++) {
        if(i < 3){
            if (abs(e(i)) < deadband_pos) {
                e(i) = 0.0;
            }
        }
        else{
            if (abs(e(i)) < deadband_rot) {
                e(i) = 0.0;
            }
        }
    }
    
    // Accumulate integral error
    e_buffer += e * dt;
    
    // PI control to get desired Cartesian velocity
    VectorXd dp_desired = -(K_p * e) - (K_i * e_buffer);
    
    // Apply Butterworth filter to smooth the Cartesian velocity command
    dp_desired = butterworth_filter_pose(dp_desired);
    
    // Convert to joint velocities
    VectorXd dq_desired = pinv_jacobian * dp_desired;
    // VectorXd dq_desired_deg = dq_desired * (180.0/M_PI);

    std::cout << "Pose error in base frame coord: ";
    for(auto& i : e){std::cout << i << ", ";}
    std::cout<<"\n";
    
    return dq_desired;   
}


VectorXd Controller::chicken_head_velocity_torque_controller(Dynamics& robot,
    VectorXd& q, VectorXd& dq, MatrixXd& T_B7,
    VectorXd& q_desired, VectorXd& dp_measured, VectorXd& K_joint_diag,
    double dt) {
    
    // Calculate dq_d using finite difference
    static thread_local VectorXd q_desired_prev = VectorXd::Zero(7);
    static thread_local bool first_call = true;
    
    VectorXd dq_d(7);
    if (first_call) {
        q_desired_prev = q_desired;
        dq_d = VectorXd::Zero(7);
        first_call = false;
    } else {
        dq_d = (q_desired - q_desired_prev) / dt;
        q_desired_prev = q_desired;
    }
    
    // Create dummy variables for unused parameters
    VectorXd ddq = VectorXd::Zero(7);
    VectorXd ddq_d = VectorXd::Zero(7);
    
    // Create integral gain matrix
    VectorXd K_integral_diag = K_joint_diag * 0.02;
    
    // Use joint impedance controller
    VectorXd u = joint_impedance_controller(robot, q, dq, ddq, q_desired, dq_d, ddq_d, 
                                           K_joint_diag, K_integral_diag, 1.0/dt, dt);
    
    return u;   
}