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

#include <yaml-cpp/yaml.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/base/Vector.h>
#include <gpmp2/kinematics/ArmModel.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
    #include <Windows.h>
#else
    #include <unistd.h>
#endif

#include <time.h>

#include "utils.h"

#include "GenerateArmModel.h"
#include "TrajectoryOptimization.h"
#include "TrajectoryInitiation.h"

#include "ViconInterface.h"

#include "move.h"
#include "sdf.h"


class Gen3Arm : public ViconSDF, public InitializeTrajectory, public OptimizeTrajectory {
private:
    std::string ip_;
    
    JointLimits pos_limits_;
    JointLimits vel_limits_;
    std::unique_ptr<gpmp2::SignedDistanceField> sdf;

public:
    DHParameters dh_params_;
    GPMP2_OccupancyGrid dataset_logs;
    gpmp2::ArmModel arm_model_logs;
    TrajectoryResult result_logs;
    gtsam::Pose3 target_pose_logs;


    // Constructor
    Gen3Arm(const std::string& ip_addr, 
        const std::string& robot_urdf_path, 
        const std::string& dh_params_path, 
        const std::string& joint_limits_path);

    // Create signed distance field
    void make_sdf( TubeInfo& tube_info, 
                   HumanInfo& human_info, 
                   bool include_tube,
                   const gtsam::Pose3& other_arm_base = gtsam::Pose3(), 
                   const std::vector<double>& other_arm_config_vec = std::vector<double>(7,0.0));

    // Get current pose from joint positions
    gtsam::Pose3 forward_kinematics(gtsam::Pose3& base_pose, 
                                                  std::vector<double>& current_joint_pos);

    // Create target pose relative to tube and human
    gtsam::Pose3 create_target_pose(const HumanInfo& human_info,
                                         const double& offset_from_human_y,
                                         const gtsam::Pose3& start_pose,
                                         double offset);

    gtsam::Pose3 over_head_pose(const Eigen::Vector3d& head_info,
                                const gtsam::Pose3& base_pose,             
                                const gtsam::Pose3& start_pose,
                                const double& offset_from_base_y,
                                const double& offset_from_head_x,
                                const double& offset_from_head_z);

    gtsam::Pose3 over_head_pipe_pose(const gtsam::Pose3& other_arm_info,             
                            HumanInfo& human_info,
                            const double& offset_from_human_max_y,
                            const double& offset_from_tube_z);
    
    gtsam::Pose3 installtion_pose(const gtsam::Point3& target_info,             
                            const gtsam::Pose3& start_pose);

    // Plan joint space trajectory
    void plan_joint(JointTrajectory& trajectory, 
                    std::vector<double>& current_joint_pos, 
                    const gtsam::Pose3& base_pose, 
                    const TubeInfo& tube_info,
                    double offset_from_base_y,
                    double offset_from_tube_z, 
                    double total_time_sec, 
                    size_t total_time_step,
                    int control_frequency, bool tune_pose = true, 
                    double y_pos_tolerance = 0.01,
                    double y_rot_tolerance = 0.01);
    
    void plan_joint(JointTrajectory& trajectory, 
                     std::vector<double>& current_joint_pos, 
                     const gtsam::Pose3& base_pose, 
                     const gtsam::Pose3& target_pose, 
                     double total_time_sec,
                     size_t total_time_step,
                     int control_frequency,
                    double y_pos_tolerance = 0.01,
                    double y_rot_tolerance = 0.01);

    void plan_joint(JointTrajectory& trajectory, 
                     std::vector<double>& current_joint_pos,  
                     std::vector<double>& target_joint_pos, 
                     const gtsam::Pose3& base_pose, 
                     double total_time_sec, 
                     size_t total_time_step,
                     int control_frequency,
                    double y_pos_tolerance = 0.01,
                    double y_rot_tolerance = 0.01);
    

    // Replan joint space trajectory starting from 200ms future state

    bool replan_joint(
                     const JointTrajectory& old_trajectory,
                     JointTrajectory& new_trajectory, 
                     const gtsam::Pose3& base_pose, 
                     const TubeInfo& tube_info,
                     double offset_from_base_y,
                     double offset_from_tube_z,
                     double total_time_sec, 
                     size_t total_time_step,
                     int control_frequency);
                    

    // Check if replanning is needed by monitoring end pose deviation
    void check_replan(const Eigen::VectorXd& trajectory_last_pos,
                          const gtsam::Pose3& base_pose,
                          const TubeInfo& tube_info, 
                          std::shared_mutex& vicon_data_mutex,
                          std::atomic<bool>& check_replan_flag, std::atomic<bool>& execution_ongoing_flag);

                          
    // Thread-safe replanning method for continuous monitoring and trajectory replacement
    void replan(JointTrajectory& current_trajectory,
                    JointTrajectory& new_trajectory,
                    gtsam::Pose3& base_pose,
                    std::shared_mutex& vicon_data_mutex,
                    std::shared_mutex& joint_data_mutex,
                    std::mutex& trajectory_mutex,
                    std::atomic<bool>& replan_triggered,
                    std::atomic<bool>& new_trajectory_ready,
                    std::atomic<bool>& execution_ongoing_flag,
                    HumanInfo& human_info, TubeInfo& tube_info,
                    gtsam::Pose3& other_base_pose,
                    std::vector<double>& other_conf,
                    size_t total_time_step, int control_frequency
                );

    // Plan task space trajectory
    void plan_task(TaskTrajectory& trajectory, 
                   const gtsam::Pose3& start_pose,
                   const gtsam::Pose3& end_pose,
                   const double duration_sec,
                   const double percentage,
                   const double height,
                   const int control_frequency = 500);
    
    void plan_task(JointTrajectory& trajectory, 
                    const gtsam::Pose3& start_pose,
                    const gtsam::Pose3& end_pose,
                    const gtsam::Pose3& base_pose,
                    const std::vector<double>& current_joint_pos,
                    const double total_time_sec,
                    const int total_time_step,
                    const double percentage,
                    const double height,
                    const int control_frequency,
                    bool target_pose_only = false,
                    double y_pos_tolerance = 0.1,
                    double y_rot_tolerance = 0.01);

    void plan_cartesian_z(JointTrajectory& trajectory, 
                    std::vector<double>& current_joint_pos, 
                    const gtsam::Pose3& base_pose, 
                    const TubeInfo& tube_info,
                    double total_time_sec, 
                    int control_frequency,
                    bool to_tube);
};

