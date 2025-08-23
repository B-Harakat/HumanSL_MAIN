#pragma once

#include "plan.h"
#include "move.h"


bool plan_action(
    int trigger_id,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& head_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex,
    bool mirrored = false
);

void state_transition(
    std::atomic<int>& state_idx, 
    std::atomic<int>& prev_state_idx,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    std::atomic<bool>& replan_triggered,
    std::atomic<bool>& new_trajectory_ready,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& head_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    k_api::BaseCyclic::BaseCyclicClient* left_base_cyclic,
    k_api::BaseCyclic::BaseCyclicClient* right_base_cyclic,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    JointTrajectory& new_joint_trajectory,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex);


void state_transition_mirrored(
    std::atomic<int>& state_idx, 
    std::atomic<int>& prev_state_idx,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    std::atomic<bool>& replan_triggered,
    std::atomic<bool>& new_trajectory_ready,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& head_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    k_api::BaseCyclic::BaseCyclicClient* left_base_cyclic,
    k_api::BaseCyclic::BaseCyclicClient* right_base_cyclic,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    JointTrajectory& new_joint_trajectory,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex);


void mirror_trajectory(JointTrajectory& original_trajectory, JointTrajectory& mirrored_trajectory, bool target_only = false);



