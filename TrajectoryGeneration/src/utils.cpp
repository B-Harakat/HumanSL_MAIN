#include "utils.h"
#include <tuple>

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> pop_front(JointTrajectory& trajectory) {
    if (trajectory.pos.empty() || trajectory.vel.empty() || trajectory.acc.empty()) {
        throw std::runtime_error("JointTrajectory is empty");
    }
    
    // Get the first elements
    Eigen::VectorXd first_pos = trajectory.pos.front();
    Eigen::VectorXd first_vel = trajectory.vel.front();
    Eigen::VectorXd first_acc = trajectory.acc.front();
    
    // Remove the first elements except when there's only 1 left
    if (trajectory.pos.size() > 1 && trajectory.vel.size() > 1 && trajectory.acc.size() > 1) {
        trajectory.pos.pop_front();
        trajectory.vel.pop_front();
        trajectory.acc.pop_front();
    }
    
    return std::make_tuple(first_pos, first_vel, first_acc);
}


std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> pop_front(TaskTrajectory& trajectory) {
    if (trajectory.pos.empty() || trajectory.vel.empty() || trajectory.acc.empty()) {
        throw std::runtime_error("JointTrajectory is empty");
    }
    
    // Get the first elements
    Eigen::VectorXd first_pos = trajectory.pos.front();
    Eigen::VectorXd first_vel = trajectory.vel.front();
    Eigen::VectorXd first_acc = trajectory.acc.front();
    
    // Remove the first elements except when there's only 1 left
    if (trajectory.pos.size() > 1 && trajectory.vel.size() > 1 && trajectory.acc.size() > 1) {
        trajectory.pos.pop_front();
        trajectory.vel.pop_front();
        trajectory.acc.pop_front();
    }
    
    return std::make_tuple(first_pos, first_vel, first_acc);
}

gtsam::Pose3 createPoseFromConf(const gpmp2::ArmModel& arm_model, const gtsam::Vector& config, bool upright) {
    // Use the direct forwardKinematics method instead of matrix wrapper
    std::vector<gtsam::Pose3> joint_poses;
    arm_model.fk_model().forwardKinematics(config, {}, joint_poses);
     
    // The end-effector is the last pose (index 6 for 7-DOF arm)
    gtsam::Pose3 ee_pose = joint_poses.back();

    if (upright) {
        // Keep only the position, set rotation to identity
        return gtsam::Pose3(gtsam::Rot3(), ee_pose.translation());
    } else {
        // Return full pose with actual rotation
        return ee_pose;
    }
}

gtsam::Pose3 createPoseFromTube( const TubeInfo& tube_axis, double human_max_y, double offset_from_human_y, double offset_from_tube_z) 
{
    // Calculate target y position
    double target_y = human_max_y + offset_from_human_y;
    
    // Find point on tube axis where y = target_y
    double t = (target_y - tube_axis.centroid.y()) / tube_axis.direction.y();
    
    // Calculate position on tube axis
    Eigen::Vector3d tube_point = tube_axis.centroid + t * tube_axis.direction;
    
    // Set gripper position: at tube x,y but 15cm below tube z
    gtsam::Point3 target_position(
        tube_point.x(),
        target_y,
        tube_point.z() + offset_from_tube_z
    );

    gtsam::Rot3 rotation;
    if (offset_from_tube_z < 0) {
        // Negative offset: upright orientation (z-axis pointing up)
        rotation = gtsam::Rot3();  // Identity rotation
    } else {
        // Positive offset: downward orientation (z-axis pointing down)
        rotation = gtsam::Rot3::Rx(M_PI);  // 180 degree rotation around X-axis
    }

    // Create pose with identity rotation (z-axis pointing up)
    return gtsam::Pose3(rotation, target_position);
}
   
std::pair<gtsam::Pose3, gtsam::Pose3> createArmBasePoses(const gtsam::Point3& clav_point, const gtsam::Point3& strn_point) {
    
    gtsam::Point3 human_torso_centre = (clav_point + strn_point) / 2.0;

    // Calculate positions: 25cm behind human back, at 1.2m height, with fixed relative spacing

    double arm_y = human_torso_centre.y() + 0.60;  // 25cm behind
    double arm_z = human_torso_centre.z();  // Fixed middle torso height
    
    // Fixed relative spacing from XACRO
    double left_x_offset = 0.06029;
    double right_x_offset = -0.06029;
    
    // Position arms relative to human center

    double left_x = human_torso_centre.x() + left_x_offset;
    double right_x = human_torso_centre.x() + right_x_offset;
    
    // Create poses with same orientation as original Python code
    gtsam::Rot3 left_rot = gtsam::Rot3::RzRyRx(2.35619449039, 0, 4.71238898038);
    gtsam::Point3 left_trans(left_x, arm_y, arm_z);
    gtsam::Pose3 left_base_pose(left_rot, left_trans);
    
    gtsam::Rot3 right_rot = gtsam::Rot3::RzRyRx(-2.35619449039, 0, 4.71238898038);
    gtsam::Point3 right_trans(right_x, arm_y, arm_z);
    gtsam::Pose3 right_base_pose(right_rot, right_trans);
    
    return std::make_pair(left_base_pose, right_base_pose);
}

std::deque<Eigen::VectorXd> convertToDeg(const std::vector<gtsam::Vector>& gtsam_trajectory) {
        std::deque<Eigen::VectorXd> trajectory;
        
        for(const auto& config : gtsam_trajectory) {
            Eigen::VectorXd joint_angles(7);
            for(int i = 0; i < 7; i++) {
                // Convert from radians to degrees
                joint_angles[i] = config(i) * 180.0 / M_PI;
            }
            trajectory.push_back(joint_angles);
        }
        
        return trajectory;
    }


std::deque<Eigen::VectorXd> computeAcceleration(
    const std::deque<Eigen::VectorXd>& velocity, 
    double dt) {
    
    // Handle edge cases
    if (velocity.empty() || dt <= 0) {
        return {};
    }
    
    size_t num_timesteps = velocity.size();
    size_t num_joints = velocity[0].size();
    
    std::deque<Eigen::VectorXd> acceleration(num_timesteps, 
                                                  Eigen::VectorXd::Zero(num_joints));
    
    
    // For each joint
    for (size_t j = 0; j < num_joints; ++j) {
        
        // First point: forward extrapolation
        // Use forward difference: (v[1] - v[0]) / dt
        acceleration[0][j] = (velocity[1][j] - velocity[0][j]) / dt;
        
        // Interior points: centered finite difference
        // a[i] = (v[i+1] - v[i-1]) / (2*dt)
        for (size_t i = 1; i < num_timesteps - 1; ++i) {
            acceleration[i][j] = (velocity[i + 1][j] - velocity[i - 1][j]) / (2.0 * dt);
        }
        
        // Last point: backward extrapolation
        // Use backward difference: (v[n-1] - v[n-2]) / dt
        acceleration[num_timesteps - 1][j] = (velocity[num_timesteps - 1][j] - 
                                              velocity[num_timesteps - 2][j]) / dt;
    }
    
    return acceleration;
}


JointTrajectory convertTrajectory(const TrajectoryResult& result, double dt){

    JointTrajectory trajectory;
    trajectory.pos = convertToDeg(result.trajectory_pos);
    trajectory.vel = convertToDeg(result.trajectory_vel);
    trajectory.acc = computeAcceleration(trajectory.vel,dt);
    return trajectory;
}

void analyzeTrajectoryResults(
    const gpmp2::ArmModel& arm_model,
    const TrajectoryResult& trajectory,
    const gtsam::Pose3& target_pose) {
    
    std::cout << "\n=== Trajectory Analysis ===" << std::endl;
    if (!trajectory.trajectory_pos.empty()) {
        // Check final pose
        gtsam::Vector final_config = trajectory.trajectory_pos.back();
        gtsam::Pose3 final_pose = createPoseFromConf(arm_model, final_config, false);
        
        // Extract final position
        gtsam::Point3 final_position = final_pose.translation();
        gtsam::Point3 target_position = target_pose.translation();
        double position_error = (final_position - target_position).norm();
        
        std::cout << "Final position: [" << final_position.x() << ", " 
                  << final_position.y() << ", " << final_position.z() << "]" << std::endl;
        std::cout << "Target position: [" << target_position.x() << ", " 
                  << target_position.y() << ", " << target_position.z() << "]" << std::endl;
        std::cout << "Position error: " << position_error << " m" << std::endl;
        
        // Check joint limits
        bool within_limits = true;
        for (const auto& config : trajectory.trajectory_pos) {
            for (int i = 0; i < config.size(); ++i) {
                if (config(i) < -3.14 || config(i) > 3.14) {  // Simple joint limit check
                    within_limits = false;
                    break;
                }
            }
            if (!within_limits) break;
        }
        
        std::cout << "All waypoints within joint limits: " << (within_limits ? "Yes" : "No") << std::endl;
    }
    else{
        std::cout << "No trajectory data available for analysis." << std::endl;
    }
    
    std::cout << "========================\n" << std::endl;
}


std::pair<JointLimits, JointLimits> createJointLimits(const std::string& config_path) {
    
    JointLimits pos_limits(7);
    JointLimits vel_limits(7);

    YAML::Node config = YAML::LoadFile(config_path);
        
        // Extract position limits
        
    for (int i = 1; i <= 7; ++i) {
        std::string actuator_key = "actuator_" + std::to_string(i);
        if (config["position_limits"][actuator_key]) {
            pos_limits.lower(i-1) = config["position_limits"][actuator_key]["lower_limit"].as<double>();
            pos_limits.upper(i-1) = config["position_limits"][actuator_key]["upper_limit"].as<double>();
        }
    }
            
    // Extract velocity limits    
    for (int i = 1; i <= 7; ++i) {
        std::string actuator_key = "actuator_" + std::to_string(i);
        if (config["velocity_limits"][actuator_key]) {
            vel_limits.lower(i-1) = config["velocity_limits"][actuator_key]["lower_limit"].as<double>();
            vel_limits.upper(i-1) = config["velocity_limits"][actuator_key]["upper_limit"].as<double>();
        }
    }
            
  
    return std::make_pair(pos_limits, vel_limits);
}

DHParameters createDHParams(const std::string& yaml_path) {

    DHParameters dh_params;
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // Initialize vectors for 7 DOF
    dh_params.a = gtsam::Vector::Zero(7);
    dh_params.alpha = gtsam::Vector::Zero(7);
    dh_params.d = gtsam::Vector::Zero(7);
    dh_params.theta = gtsam::Vector::Zero(7);
    
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

// Helper function to create DH transformation matrix
gtsam::Matrix4 createDHTransform(double a, double alpha, double d, double theta) {
    gtsam::Matrix4 T = gtsam::Matrix4::Identity();
    
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

// Helper function to compute forward kinematics from base to end effector
gtsam::Pose3 computeBaseToEE(const DHParameters& dh, const gtsam::Vector& joint_angles) {
    // Start with identity transformation
    gtsam::Pose3 T_base_to_ee = gtsam::Pose3::Identity();
    
    // Chain all DH transformations from base to end effector
    for (int i = 0; i < 7; i++) {
        // Actual theta = theta_offset + joint_angle
        double theta_i = dh.theta(i) + joint_angles(i);
        
        // Create DH transformation for joint i
        gtsam::Matrix4 T_i = createDHTransform(dh.a(i), dh.alpha(i), dh.d(i), theta_i);
        gtsam::Pose3 pose_i(T_i);
        
        // Compose with previous transformations
        T_base_to_ee = T_base_to_ee * pose_i;
    }
    
    return T_base_to_ee;
}



// Forward kinematics: base_pose_in_world * T_base_to_ee -> ee_pose_in_world
gtsam::Pose3 forwardKinematics(const DHParameters& dh, 
                               const gtsam::Vector& joint_angles, 
                               const gtsam::Pose3& base_pose_in_world) {
    // Compute transformation from base to end effector
    gtsam::Pose3 T_base_to_ee = computeBaseToEE(dh, joint_angles);
    
    // Transform to world frame: T_world_to_ee = T_world_to_base * T_base_to_ee
    return base_pose_in_world * T_base_to_ee;
}

// Inverse kinematics: ee_pose_in_world * inverse(T_base_to_ee) -> base_pose_in_world
gtsam::Pose3 inverseForwardKinematics(const DHParameters& dh, 
                              const gtsam::Vector& joint_angles, 
                              const gtsam::Pose3& ee_pose_in_world) {
    // Compute transformation from base to end effector
    gtsam::Pose3 T_base_to_ee = computeBaseToEE(dh, joint_angles);
    
    // Solve for base pose: T_world_to_base = T_world_to_ee * T_ee_to_base
    // where T_ee_to_base = inverse(T_base_to_ee)
    return ee_pose_in_world * T_base_to_ee.inverse();
}

std::vector<double> shiftAngle(std::vector<double>& q_cur) {
    std::vector<double> shifted_angles;
    shifted_angles.reserve(q_cur.size());
    
    for (const double& angle : q_cur) {
        if (angle > 180.0) {
            shifted_angles.push_back(angle - 360.0);
        } else {
            shifted_angles.push_back(angle);
        }
    }
    
    return shifted_angles;
}

void shiftAngleInPlace(Eigen::VectorXd& q_cur) {
    
    for (int i = 0; i < q_cur.size(); ++i) {
        if (q_cur(i) > 180.0) {
            q_cur(i) = q_cur(i) - 360.0;
        } 
    }
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> world2base(
    const Eigen::VectorXd& p,
    const Eigen::VectorXd& dp, 
    const Eigen::VectorXd& ddp,
    const gtsam::Pose3& base_pose_) {

    
    gtsam::Pose3 base_pose = base_pose_ *
                              gtsam::Pose3(gtsam::Rot3::Rx(M_PI),
                              gtsam::Point3::Zero());

    // Extract position and orientation from input vectors (x,y,z,roll,pitch,yaw)
    gtsam::Point3 world_position(p(0), p(1), p(2));
    gtsam::Rot3 world_rotation = gtsam::Rot3::Ypr(p(5), p(4), p(3)); // yaw, pitch, roll
    gtsam::Pose3 world_pose(world_rotation, world_position);
    
    // Transform pose from world frame to base frame
    gtsam::Pose3 base_frame_pose = base_pose.inverse() * world_pose;
    
    // Extract transformed position and rotation
    gtsam::Point3 base_position = base_frame_pose.translation();
    gtsam::Vector3 base_rpy = base_frame_pose.rotation().rpy(); // roll, pitch, yaw
    
    // Create transformed pose vector
    Eigen::VectorXd p_base(6);
    p_base << base_position.x(), base_position.y(), base_position.z(),
              base_rpy(0), base_rpy(1), base_rpy(2); // roll, pitch, yaw
    
    // Transform velocity vectors
    // For linear velocities, transform by rotation only (no translation component)
    gtsam::Rot3 base_rotation_inv = base_pose.rotation().inverse();
    gtsam::Vector3 world_linear_vel(dp(0), dp(1), dp(2));
    gtsam::Vector3 world_angular_vel(dp(3), dp(4), dp(5));
    
    gtsam::Vector3 base_linear_vel = base_rotation_inv * world_linear_vel;
    gtsam::Vector3 base_angular_vel = base_rotation_inv * world_angular_vel;
    
    Eigen::VectorXd dp_base(6);
    dp_base << base_linear_vel(0), base_linear_vel(1), base_linear_vel(2),
               base_angular_vel(0), base_angular_vel(1), base_angular_vel(2);
    
    // Transform acceleration vectors
    gtsam::Vector3 world_linear_acc(ddp(0), ddp(1), ddp(2));
    gtsam::Vector3 world_angular_acc(ddp(3), ddp(4), ddp(5));
    
    gtsam::Vector3 base_linear_acc = base_rotation_inv * world_linear_acc;
    gtsam::Vector3 base_angular_acc = base_rotation_inv * world_angular_acc;
    
    Eigen::VectorXd ddp_base(6);
    ddp_base << base_linear_acc(0), base_linear_acc(1), base_linear_acc(2),
                base_angular_acc(0), base_angular_acc(1), base_angular_acc(2);
    
    return std::make_tuple(p_base, dp_base, ddp_base);
}