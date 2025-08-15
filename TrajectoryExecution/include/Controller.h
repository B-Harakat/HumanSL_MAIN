//-------------------------------------------------
// Header file for Controller.cpp
//-------------------------------------------------

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Controller.h"
#include "Dynamics.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <tuple>

using namespace Eigen;
using namespace std;

//----------------------------------------------
// Functions used in the main codes
//----------------------------------------------
// impedance_controller: compute the controlled system input, u
// ini_controller: initialize the controller
//----------------------------------------------
namespace Controller {
    tuple<VectorXd, VectorXd, VectorXd, VectorXd> task_impedance_controller(Dynamics &robot, VectorXd& q, VectorXd& dq, VectorXd& ddq, MatrixXd& T_B7,
                                                             VectorXd& p_d, VectorXd& dp_d, VectorXd& ddp_d,
                                                             VectorXd& K_d_diag, VectorXd& K_n_diag,
                                                             int c_f, double& time_period);
    void ini_controller(Vector3d& pos, MatrixXd& T_B7);

    tuple<VectorXd> joint_impedance_controller(Dynamics &robot, VectorXd& q, VectorXd& dq, VectorXd& ddq,
                                                      VectorXd& q_d, VectorXd& dq_d, VectorXd& ddq_d,
                                                      VectorXd& K_joint_diag, 
                                                      int c_f, double& time_period);
}


#endif //CONTROLLER_H
