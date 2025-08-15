#pragma once

#include "utils.h"
#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>


class ArmModel{
    private:

        gpmp2::BodySphereVector generateArmSpheres(
            size_t arm_id_offset = 0, 
            double sphere_radius = 0.05);
        

    public:
        ArmModel();
        
        std::unique_ptr<gpmp2::ArmModel> createArmModel(
            const gtsam::Pose3& base_pose, 
            const DHParameters& dh_params);
};



