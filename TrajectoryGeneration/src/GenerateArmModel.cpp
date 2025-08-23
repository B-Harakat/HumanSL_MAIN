#include "GenerateArmModel.h"


ArmModel::ArmModel(){};


std::unique_ptr<gpmp2::ArmModel> ArmModel::createArmModel(const gtsam::Pose3& base_pose, const DHParameters& dh_params) {
    
    // Create 7-DOF arms
    auto arm = std::make_unique<gpmp2::Arm>(7, dh_params.a, dh_params.alpha, dh_params.d, 
                                                base_pose, dh_params.theta);
 
    auto arm_spheres = generateArmSpheres(0, 0.05);
    
    // Create ArmModel instances
    auto arm_model = std::make_unique<gpmp2::ArmModel>(*arm, arm_spheres);

    
    return arm_model;
}


gpmp2::BodySphereVector ArmModel::generateArmSpheres(
    size_t arm_id_offset, 
    double sphere_radius) {
    
    gpmp2::BodySphereVector spheres;
    
    // Joint 0: Base to Joint 1 (d[0] = 0.2848, ~28.5cm link)
    spheres.emplace_back(arm_id_offset + 0, sphere_radius, gtsam::Point3(0.0, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 0, sphere_radius * 0.7, gtsam::Point3(0.0, 0.07, 0.0));
    spheres.emplace_back(arm_id_offset + 0, sphere_radius * 0.6, gtsam::Point3(0.0, 0.12, 0.0));
    spheres.emplace_back(arm_id_offset + 0, sphere_radius * 0.5, gtsam::Point3(0.0, 0.17, 0.0));

    
    // Joint 1: Short connector (d[1] = -0.0118, ~1.2cm) - SKIP
    
    // Joint 2: Upper arm long link (d[2] = -0.4208, ~42.1cm link)
    spheres.emplace_back(arm_id_offset + 2, sphere_radius * 1.3, gtsam::Point3(0.0, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.05, 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.10, 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.18, 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.26, 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.34, 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.42, 0.0));
    
    // Joint 3: Short connector (d[3] = -0.0128, ~1.3cm) - SKIP
    
    // Joint 4: Forearm long link (d[4] = -0.3143, ~31.4cm link)
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1.3, gtsam::Point3(0.0, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.08, 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.16, 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.24, 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.31, 0.0));

    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(0.0, 0.0, 0.08));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(0.0, 0.0, -0.08));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(0.08, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(-0.08, 0.0, 0.0));
    
    // Joint 5: No link (d[5] = 0.0) - SKIP
    
    // Joint 6: Wrist to interface (d[6] = -0.1674, ~16.7cm link)
    spheres.emplace_back(arm_id_offset + 6, sphere_radius, gtsam::Point3(0.0, 0.0, -0.14));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius, gtsam::Point3(0.0, 0.0, -0.20));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius, gtsam::Point3(0.0, 0.0, -0.25));


    // Gripper/End-effector spheres 
    // spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.4, gtsam::Point3(0.07, 0.0, 0.04));
    // spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.4, gtsam::Point3(-0.07, 0.0, 0.04));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.05, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.05, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.05, 0.0, -0.04));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.05, 0.0, -0.04));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.07, 0.0, -0.08));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.07, 0.0, -0.08));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.07, 0.0, -0.12));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.07, 0.0, -0.12));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.05, 0.0, -0.14));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.05, 0.0, -0.14)); 
    spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.8, gtsam::Point3(0.0, 0.05, -0.15));    

    
    return spheres;
}

