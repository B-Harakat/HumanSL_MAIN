//-------------------------------------------------
// Header file for Filter.cpp
//-------------------------------------------------

#ifndef EXPERIMENTALCODES_FILTER_H
#define EXPERIMENTALCODES_FILTER_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
using namespace Eigen;
using namespace std;

//----------------------------------------------
// Functions used in the main codes
//----------------------------------------------
// butterworth_filter: butterworth_filter to filter noise in measured accelerations
// ini_butterworth: initialize the butterworth filter
//----------------------------------------------
namespace Filter {
    VectorXd butterworth_filter(const VectorXd& current_dq);
    void ini_butterworth();

    // function for pose error filtering
    void ini_butterworth_pose();
    VectorXd butterworth_filter_pose(const VectorXd& current_pose_error);

    // function for velocity filtering  
    void ini_butterworth_velocity();
    VectorXd butterworth_filter_velocity(const VectorXd& current_velocity);

};


#endif //EXPERIMENTALCODES_FILTER_H
