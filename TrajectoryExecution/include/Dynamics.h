//-------------------------------------------------
// Header file for Dynamics.cpp
//-------------------------------------------------

#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <Eigen/Dense>

class Dynamics {
private:
    
    
    
public:

    Eigen::Vector3d gravity_world_;
    Eigen::Matrix3d R_world_to_base_;
    pinocchio::Data data_;
    pinocchio::Model model_;
    
    // Constructor
    Dynamics(const std::string& urdf_path);
    
    // Configuration conversion methods
    Eigen::VectorXd convertJointAnglesToConfig(const Eigen::VectorXd& joint_angles);
    Eigen::VectorXd convertConfigToJointAngles(const Eigen::VectorXd& q);
    
    // Base orientation methods
    void setBaseOrientation(const Eigen::Matrix3d& R_world_to_base);
    void setBaseOrientationRPY(double roll, double pitch, double yaw);
    Eigen::Vector3d getCurrentGravityBase() const;
    Eigen::Matrix3d getBaseOrientation() const;
    
    // Dynamics computation methods
    Eigen::MatrixXd mass_m(const Eigen::VectorXd& q);
    Eigen::VectorXd gravity_m(const Eigen::VectorXd& q);
    Eigen::MatrixXd coriolis_m_simplified(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
    Eigen::MatrixXd coriolis_m(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
    
    void computeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
    // Utility methods
    Eigen::VectorXd getNeutralConfiguration();
    Eigen::VectorXd getRandomConfiguration();
    void printModelInfo();
};

#endif // DYNAMICS_H