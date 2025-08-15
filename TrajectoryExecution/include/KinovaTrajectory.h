/*
* Kinova Gen3 Trajectory Executor
* Executes a densified 1000Hz trajectory on Kinova Gen3 7DoF arm
*/

#ifndef KINOVA_TRAJECTORY_H
#define KINOVA_TRAJECTORY_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <atomic>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifndef _OS_UNIX
#define _OS_UNIX
#endif

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/Dense>
#include "Jacobian.h"
#include "Fwd_kinematics.h"
#include "Dynamics.h"
#include "Controller.h"
#include "Filter.h"

#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7
#define CONTROL_FREQUENCY 500



std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(promise<k_api::Base::ActionEvent>& finish_promise);

// Get current joint positions from the robot
std::vector<double> get_current_joint_pos(k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

// Move to trajectory start position, require the robot to be in single level
void move_single_level(k_api::Base::BaseClient* base, std::vector<double> q_d);

// Main trajectory execution function
bool joint_position_control(
                    k_api::Base::BaseClient* base,
                    k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                    const std::vector<VectorXd>& trajectory);

// Impedance control functions
bool task_impedance_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                       k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                       std::vector<VectorXd>& p_d, std::vector<VectorXd>& dp_d, std::vector<VectorXd>& ddp_d, VectorXd& K_d_diag);

bool joint_impedance_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                            k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                            std::vector<VectorXd>& q_d, std::vector<VectorXd>& dq_d, std::vector<VectorXd>& ddq_d, 
                            VectorXd& K_joint_diag);

bool joint_impedance_control_stabilized(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                            k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                            std::vector<VectorXd>& q_d, std::vector<VectorXd>& dq_d, std::vector<VectorXd>& ddq_d, 
                            VectorXd& K_joint_diag, VectorXd& K_task_diag);

// Example usage function
bool executeTrajectory(const std::string& ip_address, std::vector<VectorXd>& pos, std::vector<VectorXd>& vel, std::vector<VectorXd>& acc, std::string control_mode);
                       

#endif // KINOVA_TRAJECTORY_H