#pragma once

#include "utils.h"
#include <unordered_map>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Symbol.h> 

#include <gtsam/linear/NoiseModel.h>

#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/SelfCollisionArm.h>

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePositionArm.h>

#include <gpmp2/planner/TrajUtils.h>

#include "JerkPenaltyFactor.h"

class OptimizeTrajectory {
private:

public:

    OptimizeTrajectory();

    TrajectoryResult optimizeJointTrajectory(
        const gpmp2::ArmModel& arm_model,
        const gpmp2::SignedDistanceField& sdf,
        const gtsam::Values& init_values,
        const gtsam::Pose3& target_pose,
        const gtsam::Vector& start_config,
        const gtsam::Vector& start_vel,
        const JointLimits& pos_limits,
        const JointLimits& vel_limits,
        const size_t total_time_step,
        const double total_time_sec,
        const double target_dt = 0.001
    );

    TrajectoryResult optimizeTaskTrajectory(
        const gpmp2::ArmModel& arm_model,
        const gpmp2::SignedDistanceField& sdf,
        const gtsam::Values& init_values,
        const std::deque<gtsam::Pose3>& pose_trajectory,
        const gtsam::Vector& start_config,
        const JointLimits& pos_limits,
        const JointLimits& vel_limits,
        const size_t total_time_step,
        const double total_time_sec,
        const double target_dt = 0.001
    );

    TrajectoryResult reOptimizeJointTrajectory(
        const gpmp2::ArmModel& arm_model,
        const gpmp2::SignedDistanceField& sdf,
        const gtsam::Values& init_values,
        const gtsam::Pose3& target_pose,
        const std::vector<gtsam::Vector>& start_configs,
        const std::vector<gtsam::Vector>& start_velocities,
        const JointLimits& pos_limits,
        const JointLimits& vel_limits,
        const size_t total_time_step,
        const double total_time_sec,
        const double target_dt = 0.001
    );


    std::pair<std::vector<gtsam::Vector>, std::vector<gtsam::Vector>> densifyTrajectory(
        const gtsam::Values& optimized_values,
        const gtsam::SharedNoiseModel& Qc_model,
        double delta_t,
        double total_time_sec,
        double target_dt);

};