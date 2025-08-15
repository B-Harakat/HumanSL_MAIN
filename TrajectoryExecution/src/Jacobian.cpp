//----------------------------------------------------------
// Function of computation of jacobian matrix
//----------------------------------------------------------
// Description: computation of jacobian matrix
// Copyright: Yihan Liu 2024
//----------------------------------------------------------

#include "Jacobian.h"
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


//----------------------------------------------------------
// Computation of jacobian matrix
//----------------------------------------------------------
// 1 input:
// q: joint angular positions
//
// 1 output:
// jaco: jacobian matrix
//----------------------------------------------------------
MatrixXd Jacobian::jaco_m(const VectorXd& q) {
    Matrix4d T_B1, T_12, T_23, T_34, T_45, T_56, T_67, T_7Tool;

    Eigen::Vector3d z0, z1, z2, z3, z4, z5, z6, z7;
    Eigen::Vector4d p0, p1, p2, p3, p4, p5, p6, p7, pE;
    Eigen::Vector3d z1_p1, z2_p2, z3_p3, z4_p4, z5_p5, z6_p6, z7_p7;

    Eigen::MatrixXd jaco(6,7);

    // Transformation matrices
    T_B1 << cos(q(0)), -sin(q(0)), 0, 0,
            -sin(q(0)), -cos(q(0)), 0, 0,
            0, 0, -1, 0.1564,
            0, 0, 0, 1;

    T_12 << cos(q(1)), -sin(q(1)), 0, 0,
            0, 0, -1, 0.0054,
            sin(q(1)), cos(q(1)), 0, -0.1284,
            0, 0, 0, 1;

    T_23 << cos(q(2)), -sin(q(2)), 0, 0,
            0, 0, 1, -0.2104,
            -sin(q(2)), -cos(q(2)), 0, -0.0064,
            0, 0, 0, 1;

    T_34 << cos(q(3)), -sin(q(3)), 0, 0,
            0, 0, -1, 0.0064,
            sin(q(3)), cos(q(3)), 0, -0.2104,
            0, 0, 0, 1;

    T_45 << cos(q(4)), -sin(q(4)), 0, 0,
            0, 0, 1, -0.2084,
            -sin(q(4)), -cos(q(4)), 0, -0.0064,
            0, 0, 0, 1;

    T_56 << cos(q(5)), -sin(q(5)), 0, 0,
            0, 0, -1, 0,
            sin(q(5)), cos(q(5)), 0, -0.1059,
            0, 0, 0, 1;

    T_67 << cos(q(6)), -sin(q(6)), 0, 0,
            0, 0, 1, -0.1059,
            -sin(q(6)), -cos(q(6)), 0, 0,
            0, 0, 0, 1;

    T_7Tool << 1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, -1, -0.067,
            0, 0, 0, 1;

    z0 << 0, 0, 1;
    z1 = T_B1.block<3,3>(0,0)*z0;
    z2 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*z0;
    z3 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*z0;
    z4 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*z0;
    z5 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*z0;
    z6 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*T_56.block<3,3>(0,0)*z0;
    z7 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*T_56.block<3,3>(0,0)*T_67.block<3,3>(0,0)*z0;

    p0 << 0, 0, 0, 1;
    p1 = T_B1*p0;
    p2 = T_B1*T_12*p0;
    p3 = T_B1*T_12*T_23*p0;
    p4 = T_B1*T_12*T_23*T_34*p0;
    p5 = T_B1*T_12*T_23*T_34*T_45*p0;
    p6 = T_B1*T_12*T_23*T_34*T_45*T_56*p0;
    p7 = T_B1*T_12*T_23*T_34*T_45*T_56*T_67*p0;
    pE = T_B1*T_12*T_23*T_34*T_45*T_56*T_67*T_7Tool*p0;

    z1_p1 = z1.cross(pE.head<3>()-p1.head<3>());
    z2_p2 = z2.cross(pE.head<3>()-p2.head<3>());
    z3_p3 = z3.cross(pE.head<3>()-p3.head<3>());
    z4_p4 = z4.cross(pE.head<3>()-p4.head<3>());
    z5_p5 = z5.cross(pE.head<3>()-p5.head<3>());
    z6_p6 = z6.cross(pE.head<3>()-p6.head<3>());
    z7_p7 = z7.cross(pE.head<3>()-p7.head<3>());

    jaco << z1_p1, z2_p2, z3_p3, z4_p4, z5_p5, z6_p6, z7_p7,
            z1,    z2,    z3,    z4,    z5,    z6,    z7;

    return jaco;
}