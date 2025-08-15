//-------------------------------------------------
// Header file for Jacobian.cpp
//-------------------------------------------------

#ifndef EXPERIMENTALCODES_JACOBIAN_H
#define EXPERIMENTALCODES_JACOBIAN_H
#include <Eigen/Dense>
using namespace Eigen;


//----------------------------------------------
// Functions used in the main codes
//----------------------------------------------
// jaco_m: compute the jacobian matrix
//----------------------------------------------
namespace Jacobian {
    MatrixXd jaco_m(const VectorXd &q);
}


#endif //EXPERIMENTALCODES_JACOBIAN_H
