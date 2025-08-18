



gtsam::Pose3 over_head_pose(const HumanInfo& human_info,             
                            const gtsam::Pose3& start_pose,
                            const double& offset_from_human_max_y,
                            const double& offset_from_human_mid_x,
                            const double& offset_from_human_max_z) {
    
    // Calculate target y position
    gtsam::Point3 start_pos = start_pose.translation();
    
    double target_y = human_info.bounds.max_y + offset_from_human_max_y;
    double target_x = human_info.bounds.min_x + (human_info.bounds.max_x - human_info.bounds.min_x)/2 + offset_from_human_mid_x;
    double target_z = human_info.bounds.max_z + offset_from_human_max_z;

    gtsam::Rot3 base_orientation = gtsam::Rot3::Rz(M_PI); // 180 degrees about z-axis
    gtsam::Rot3 target_orientation;
    if (start_pos.x() - target_x < 0) {
        target_orientation = base_orientation * gtsam::Rot3::Ry(-M_PI/4); // -30 degrees about new y-axis
    } else {
        target_orientation = base_orientation * gtsam::Rot3::Ry(M_PI/4);  // 30 degrees about new y-axis
    }
    
    gtsam::Point3 target_position(target_x, target_y,
    target_z);

    gtsam::Pose3 target_pose(target_orientation, target_position);
    
    return target_pose;
}


gtsam::Pose3 installtion_pose(const gtsam::Point3& target_info,             
                            const gtsam::Pose3& start_pose) {
    
    // Calculate target y position
    gtsam::Point3 start_pos = start_pose.translation();
    
    double target_y = start_pos.y();
    double target_x = target_info.x();
    double target_z = target_info.z();

    gtsam::Rot3 base_orientation = gtsam::Rot3::Rz(M_PI); // 180 degrees about z-axis
    
    gtsam::Point3 target_position(target_x, target_y,
    target_z);

    gtsam::Pose3 target_pose(target_orientation, target_position);
    
    return target_pose;
}

