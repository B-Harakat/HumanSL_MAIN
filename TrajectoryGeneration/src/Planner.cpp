
#include "Planner.h"
#include <yaml-cpp/yaml.h>


void Planner::updateArmModel(const gtsam::Pose3& left_base_pose, const gtsam::Pose3& right_base_pose) {
            left_arm = createArmModel(left_base_pose, dh_params_);
            right_arm = createArmModel(right_base_pose, dh_params_);
        };

Planner::Planner(const std::string& dh_params_path,
                const std::string& joint_limits_path) : ArmModel(), InitializeTrajectory(createDHParams(dh_params_path)), OptimizeTrajectory()
                {
                    
    dh_params_ = createDHParams(dh_params_path); 
    auto [pos_limits, vel_limits] = createJointLimits(joint_limits_path);
    pos_limits_ = pos_limits;
    vel_limits_ = vel_limits;
};

TrajectoryResult Planner::plan(
                   const gpmp2::ArmModel& arm_model,
                   const gtsam::Vector& start_conf,
                   const gtsam::Pose3& end_pose,
                   const gtsam::Pose3& base_pose,
                   double total_time_sec,
                   size_t total_time_step,
                   const gpmp2::SignedDistanceField sdf) {

            // Create initial trajectory
            gtsam::Values init_values = createInitialTrajectory(start_conf, end_pose, base_pose, total_time_step);

            // Optimize trajectory
            TrajectoryResult result = optimizeJointTrajectory(
                                    arm_model, sdf, init_values, end_pose, 
                                    start_conf, pos_limits_, vel_limits_, 
                                    total_time_step, total_time_sec);

            return result;
        };





