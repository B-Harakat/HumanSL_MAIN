#pragma once

#include "KinovaTrajectory.h"
#include "utils.h"
#include <shared_mutex>

namespace k_api = Kinova::Api;




bool joint_position_control_single(k_api::Base::BaseClient* base,
                            k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                            k_api::BaseCyclic::Feedback& base_feedback, 
                            k_api::BaseCyclic::Command& base_command,
                            VectorXd& q_d, VectorXd& q_cur);

bool joint_impedance_control_single(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command, 
                                    Dynamics &robot,
                                    VectorXd& q_d, VectorXd& dq_d, VectorXd& ddq_d, 
                                    VectorXd& K_joint_diag, VectorXd& q_cur, int control_frequency);

void joint_control_execution(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command,
                                Dynamics &robot,
                                JointTrajectory& trajectory, gtsam::Pose3& base_frame,
                                int control_frequency, std::atomic<bool>& flag, 
                                std::atomic<int>& replan_counter, std::atomic<bool>& replan_triggered,
                                std::atomic<bool>& new_trajectory_ready, JointTrajectory& new_trajectory,
                                std::mutex& trajectory_mutex, TrajectoryRecord& record);

// Overloaded function for backward compatibility (without replanning support)
void joint_control_execution(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command,
                                Dynamics &robot,
                                JointTrajectory& trajectory, gtsam::Pose3& base_frame,
                                int control_frequency, std::atomic<bool>& flag);


bool task_impedance_control_single(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                       k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command, Dynamics &robot,
                       VectorXd& p_d, VectorXd& dp_d, VectorXd& ddp_d, VectorXd& K_d_diag, int control_frequency, bool& first_call, VectorXd& last_dq, std::chrono::time_point<std::chrono::high_resolution_clock>& start_measure);
                
void task_control_execution(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command,
                                Dynamics &robot, TaskTrajectory& trajectory, gtsam::Pose3& base_frame, int control_frequency, std::string& dh_parameters_path,
                                std::atomic<bool>& flag);

void updateJointInfo(k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                       std::vector<double>& q_cur,
                       std::shared_mutex& joint_mutex);

bool move_gripper(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          float target_position, 
                          float proportional_gain = 2.2f,
                          float force_limit = 100.0f);

