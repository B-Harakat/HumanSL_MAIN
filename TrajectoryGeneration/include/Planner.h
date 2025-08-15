#pragma once

#include "utils.h"
#include "GenerateArmModel.h"
#include "TrajectoryOptimization.h"
#include "TrajectoryInitiation.h"


class Planner : public ArmModel, public InitializeTrajectory, public OptimizeTrajectory {

    private:

    public:

        DHParameters dh_params_;

        JointLimits pos_limits_;
        JointLimits vel_limits_;

        std::unique_ptr<gpmp2::ArmModel> left_arm;
        std::unique_ptr<gpmp2::ArmModel> right_arm;

        Planner(const std::string& dh_params_path,
                const std::string& joint_limits_path);

        void updateArmModel(const gtsam::Pose3& left_base_pose, const gtsam::Pose3& right_base_pose);
                
        
        TrajectoryResult plan(
                   const gpmp2::ArmModel& arm_model,
                   const gtsam::Vector& start_conf,
                   const gtsam::Pose3& end_pose,
                   const gtsam::Pose3& base_pose,
                   double total_time_sec,
                   size_t total_time_step,
                   const gpmp2::SignedDistanceField sdf
                );
};
