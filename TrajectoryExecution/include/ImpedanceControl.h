#pragma once

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

#include <Eigen/Dense>
#include "Jacobian.h"
#include <cmath>
#include "Fwd_kinematics.h"
#include "Dynamics.h"
#include "Controller.h"
#include "Filter.h"

namespace k_api = Kinova::Api;

bool task_impedance_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                       k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                       std::vector<VectorXd>& p_d, std::vector<VectorXd>& dp_d, std::vector<VectorXd>& ddp_d, VectorXd& K_d_diag);

bool joint_impedance_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                            k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                            std::vector<VectorXd>& q_d, std::vector<VectorXd>& dq_d, std::vector<VectorXd>& ddq_d, 
                            VectorXd& K_joint_diag);