//-------------------------------------------------
// Header file for Fwd_kinematics.cpp
//-------------------------------------------------

#ifndef EXPERIMENTALCODES_FWD_KINEMATICS_H
#define EXPERIMENTALCODES_FWD_KINEMATICS_H

#include <Eigen/Dense>
#include <iostream>
#include <tuple>
#include <yaml-cpp/yaml.h>
#include <gtsam/geometry/Rot3.h>


using namespace Eigen;

struct DHParameters_Eigen {
    VectorXd a;
    VectorXd alpha;
    VectorXd d;
    VectorXd theta;
};

//----------------------------------------------
// Functions used in the main codes
//----------------------------------------------
// forward: executing forward kinematics
//----------------------------------------------
namespace Fwd_kinematics {
    std::tuple<VectorXd, MatrixXd> forward(const VectorXd& q);
    std::tuple<VectorXd, Matrix4d> forward(const DHParameters_Eigen& dh, const VectorXd& joint_angles);
    DHParameters_Eigen createDHParamsEigen(const std::string& yaml_path);
}



#endif //EXPERIMENTALCODES_FWD_KINEMATICS_H
