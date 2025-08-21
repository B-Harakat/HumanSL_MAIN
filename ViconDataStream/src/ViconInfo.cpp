#include "ViconInfo.h"


void updateTubeInfo(TubeInfo& tube_info, std::vector<MarkerData>& tube_tip, std::vector<MarkerData>& tube_end, std::vector<MarkerData>& tube_mid){
    std::vector<Eigen::Vector3d> tube_tip_pts;
    std::vector<Eigen::Vector3d> tube_end_pts;
    std::vector<Eigen::Vector3d> tube_mid_pts;
    

    bool tube_tip_occluded = false;
    bool tube_end_occluded = false;
    
    for(auto marker : tube_tip){
        if(marker.occluded){
            tube_tip_occluded = true;
        } else {
            double x = marker.x/1000; 
            double y = marker.y/1000;
            double z = marker.z/1000;
            tube_tip_pts.push_back(Eigen::Vector3d(x,y,z));
        }
    }

    for(auto marker : tube_end){
        if(marker.occluded){
            tube_end_occluded = true;
        } else {
            double x = marker.x/1000; 
            double y = marker.y/1000;
            double z = marker.z/1000;
            tube_end_pts.push_back(Eigen::Vector3d(x,y,z));
        }
    }

    for(auto marker : tube_mid){
        if(!marker.occluded){
            double x = marker.x/1000; 
            double y = marker.y/1000;
            double z = marker.z/1000;
            tube_mid_pts.push_back(Eigen::Vector3d(x,y,z));
        }
    }

    // Average tube tip points
    Eigen::Vector3d avg_tube_tip = Eigen::Vector3d::Zero();
    if (!tube_tip_pts.empty()) {
        for (const auto& pt : tube_tip_pts) {
            avg_tube_tip += pt;
        }
        avg_tube_tip /= tube_tip_pts.size();
    }
    
    // Average tube end points
    Eigen::Vector3d avg_tube_end = Eigen::Vector3d::Zero();
    if (!tube_end_pts.empty()) {
        for (const auto& pt : tube_end_pts) {
            avg_tube_end += pt;
        }
        avg_tube_end /= tube_end_pts.size();
    }

    // Average tube mid points
    Eigen::Vector3d avg_tube_mid = Eigen::Vector3d::Zero();
    if (!tube_mid_pts.empty()) {
        for (const auto& pt : tube_mid_pts) {
            avg_tube_mid += pt;
        }
        avg_tube_mid /= tube_mid_pts.size();
    }
    
    TubeInfo result;
    
    // Calculate centroid as midpoint between averaged tip and end points
    result.centroid = ((avg_tube_tip + avg_tube_end) / 2.0 );
    
    // Calculate direction vector based on occlusion status
    if(tube_end_occluded && !tube_mid_pts.empty()){
        // If tube_end is occluded, use tube_mid and tube_tip
        result.direction = (avg_tube_mid - avg_tube_tip).normalized();
    } else if(tube_tip_occluded && !tube_mid_pts.empty()){
        // If tube_tip is occluded, use tube_mid and tube_end
        result.direction = (avg_tube_end - avg_tube_mid).normalized();
    } else {
        // Default: calculate direction from averaged end to averaged tip
        result.direction = (avg_tube_end - avg_tube_tip).normalized();
    }
    
    // Ensure y-axis is always positive
    if(result.direction.y() < 0){
        result.direction = -result.direction;
    }
    
    // Calculate tube length as distance between averaged points
    result.length = (avg_tube_tip - avg_tube_end).norm();

    // remove when camera is fixed
    result.centroid[2] += 0.0;
    result.centroid[0] -= 0.0;

    tube_info = result;
    
}


void updateHumanInfo(HumanInfo& human_info, std::vector<MarkerData>& human){
    
    std::vector<gtsam::Point3> human_points;

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for(auto marker : human){
        double x = marker.x/1000;
        double y = marker.y/1000;
        double z = marker.z/1000;



        human_points.push_back(gtsam::Point3(x, y, z));
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);

        if(marker.name == "human_RFIN"){
            human_info.RFIN = gtsam::Point3(x, y, z);
        }

        if(marker.name == "human_LFIN"){
            human_info.LFIN = gtsam::Point3(x, y, z);
        }

        if(marker.name == "human_RFHD"){
            human_info.RFHD = gtsam::Point3(x, y, z);
        }

        if(marker.name == "human_LFHD"){
            human_info.LFHD = gtsam::Point3(x, y, z);
        }
    }
    
    human_info.human_points = human_points;
    human_info.bounds = HumanBoundingBox{min_x, max_x, min_y, max_y, min_z, max_z};
}

void updateTargetInfo(gtsam::Point3& target_info, std::vector<MarkerData>& target){

    double x_total = 0.0;
    double y_total = 0.0;
    double z_total = 0.0;

    for(auto marker : target){
        x_total += marker.x/1000;
        y_total += marker.y/1000;
        z_total += marker.z/1000;
    }

    x_total = x_total / target.size();
    y_total = y_total / target.size();
    z_total = z_total / target.size();

    target_info = gtsam::Point3(x_total, y_total, z_total);
}


void updateViconInfo(ViconInterface& vicon, gtsam::Pose3& left_base, gtsam::Pose3& right_base, TubeInfo& tube_info, HumanInfo& human_info, gtsam::Point3& target_info, std::vector<double>& left_conf, std::vector<double>& right_conf, Eigen::Vector3d& lfin, Eigen::Vector3d& rfin, Eigen::Vector3d& head, DHParameters& dh_params, std::shared_mutex& vicon_data_mutex, std::shared_mutex& joint_data_mutex){
    
    static int counter = 0;
    static std::deque<TubeInfo> tube_info_array;
    static std::deque<gtsam::Pose3> left_base_array;
    static std::deque<gtsam::Pose3> right_base_array;

    static std::deque<Eigen::Vector3d> left_finger_pos_array;
    static std::deque<Eigen::Vector3d> right_finger_pos_array;

    static std::deque<Eigen::Vector3d> forehead_pos_array;
    static std::deque<Eigen::Vector3d> rfin_head_delta_array;

    std::unique_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);



    std::vector<MarkerData> right_base_data;
    std::vector<MarkerData> left_base_data;

    std::vector<MarkerData> right_tip_data;
    std::vector<MarkerData> left_tip_data;

    right_base_data.push_back(vicon.getMarkerPosition("right_base1"));
    right_base_data.push_back(vicon.getMarkerPosition("right_base2"));
    right_base_data.push_back(vicon.getMarkerPosition("right_base3"));
    // right_base_data.push_back(vicon.getMarkerPosition("right_base4"));

    left_base_data.push_back(vicon.getMarkerPosition("left_base1"));
    left_base_data.push_back(vicon.getMarkerPosition("left_base2"));
    left_base_data.push_back(vicon.getMarkerPosition("left_base3"));
    // left_base_data.push_back(vicon.getMarkerPosition("left_base4"));

    right_tip_data.push_back(vicon.getMarkerPosition("right_tip1"));
    right_tip_data.push_back(vicon.getMarkerPosition("right_tip2"));
    right_tip_data.push_back(vicon.getMarkerPosition("right_tip3"));
    right_tip_data.push_back(vicon.getMarkerPosition("right_tip4"));

    left_tip_data.push_back(vicon.getMarkerPosition("left_tip1"));
    left_tip_data.push_back(vicon.getMarkerPosition("left_tip2"));
    left_tip_data.push_back(vicon.getMarkerPosition("left_tip3"));
    left_tip_data.push_back(vicon.getMarkerPosition("left_tip4"));


    std::vector<MarkerData> tube_tip;
    tube_tip.push_back(vicon.getMarkerPosition("tube_tip1"));
    tube_tip.push_back(vicon.getMarkerPosition("tube_tip2"));
    tube_tip.push_back(vicon.getMarkerPosition("tube_tip3"));


    std::vector<MarkerData> tube_end;
    tube_end.push_back(vicon.getMarkerPosition("tube_end1"));
    tube_end.push_back(vicon.getMarkerPosition("tube_end2"));
    tube_end.push_back(vicon.getMarkerPosition("tube_end3"));

    std::vector<MarkerData> tube_mid;
    tube_mid.push_back(vicon.getMarkerPosition("tube_mid1"));
    tube_mid.push_back(vicon.getMarkerPosition("tube_mid2"));
    tube_mid.push_back(vicon.getMarkerPosition("tube_mid3"));
    
    std::vector<MarkerData> human = vicon.getMarkerPositions("human");
    std::vector<MarkerData> target= vicon.getMarkerPositions("target");
    
    TubeInfo tube_info_snapshot;

    updateTubeInfo(tube_info_snapshot, tube_tip, tube_end, tube_mid);
    updateHumanInfo(human_info, human);
    updateTargetInfo(target_info, target);

    // Only push tube_info_snapshot if y-component is the dominant axis
    if (std::abs(tube_info_snapshot.direction.y()) > std::abs(tube_info_snapshot.direction.x()) &&
        std::abs(tube_info_snapshot.direction.y()) > std::abs(tube_info_snapshot.direction.z())) {
        tube_info_array.push_back(tube_info_snapshot);
    }

    if(tube_info_array.size() >= 100){
        tube_info_array.pop_front();
        counter++;
        
        if(counter == 10){
            counter = 0;
        

            // Calculate average tube properties
            Eigen::Vector3d avg_centroid = Eigen::Vector3d::Zero();
            Eigen::Vector3d avg_direction = Eigen::Vector3d::Zero();
            double avg_length = 0.0;
            
            
            for(const auto& tube_sample : tube_info_array) {
                avg_centroid += tube_sample.centroid;
                avg_direction += tube_sample.direction;
                avg_length += tube_sample.length;
            }
        
            avg_centroid /= 100;
            avg_direction /= 100;
            avg_direction.normalize();  // Ensure unit vector
            avg_length /= 100;
            
            // Update tube_info with averaged values
            tube_info.centroid = avg_centroid;
            tube_info.direction = avg_direction;
            tube_info.length = avg_length;
        }
    }


    // Check for occlusion in right_base_data

    bool right_base_occluded = false;
    bool left_base_occluded = false;

    bool right_tip_occluded = false;
    bool left_tip_occluded = false;

    for (const auto& marker : right_base_data) {
        if (marker.occluded) {
            right_base_occluded = true;
            break;
        }
    }

    for (const auto& marker : left_base_data) {
        if (marker.occluded) {
            left_base_occluded = true;
            break;
        }
    }

    for (const auto& marker : right_tip_data) {
        if (marker.occluded) {
            right_tip_occluded = true;
            break;
        }
    }

    for (const auto& marker : left_tip_data) {
        if (marker.occluded) {
            left_tip_occluded = true;
            break;
        }
    }


    // Estimate left base pose if not occluded
    gtsam::Pose3 left_base_current, right_base_current;
    bool pose_updated = false;

    if (!left_base_occluded && !right_base_occluded){
        gtsam::Pose3 left_base_guess2 = updatePoseInfo2(left_base_data, right_base_data.front());
        gtsam::Pose3 right_base_guess2 = updatePoseInfo2(right_base_data, left_base_data.front());

        left_base_current = left_base_guess2;
        right_base_current = right_base_guess2;
        pose_updated = true;

    }
    
    else if (!left_base_occluded || !right_base_occluded) {

        gtsam::Pose3 right_base_guess2;
        gtsam::Pose3 left_base_guess2;

        if(left_base_occluded){

            // Find any non-occluded marker in left_base_data
            MarkerData non_occluded_left_marker = left_base_data.front(); // fallback
            for (const auto& marker : left_base_data) {
                if (!marker.occluded) {
                    non_occluded_left_marker = marker;
                    break;
                }
            }
            
            right_base_guess2 = updatePoseInfo2(right_base_data, non_occluded_left_marker);

            // Calculate left_base_guess2 using fixed transformation between left and right base
            // The transformation is fixed, so we can derive left from right
            gtsam::Pose3 left_to_right_transform = right_base.compose(left_base.inverse());
            left_base_guess2 = left_to_right_transform.inverse().compose(right_base_guess2);

        }
        if(right_base_occluded){
            // Find any non-occluded marker in right_base_data
            MarkerData non_occluded_right_marker = right_base_data.front(); // fallback
            for (const auto& marker : right_base_data) {
                if (!marker.occluded) {
                    non_occluded_right_marker = marker;
                    break;
                }
            }
            
            left_base_guess2 = updatePoseInfo2(left_base_data, non_occluded_right_marker);

            // Calculate right_base_guess2 using fixed transformation between left and right base
            // The transformation is fixed, so we can derive right from left
            gtsam::Pose3 left_to_right_transform = right_base.compose(left_base.inverse());
            right_base_guess2 = left_to_right_transform.compose(left_base_guess2);
        }

        left_base_current = left_base_guess2;
        right_base_current = right_base_guess2;
        pose_updated = true;
    }
    else if(!left_tip_occluded && !right_tip_occluded){
        gtsam::Pose3 left_base_guess1 = updatePoseInfo1(left_tip_data, dh_params, left_conf);
        gtsam::Pose3 right_base_guess1 = updatePoseInfo1(right_tip_data, dh_params, right_conf);

        left_base_current = left_base_guess1;
        right_base_current = right_base_guess1;
        pose_updated = true;
    }

    // Add poses to moving average arrays if pose was updated and passes filter
    if (pose_updated) {
        bool add_left = true;
        bool add_right = true;
        
        // Filter based on rotation difference from current average
        if (!left_base_array.empty()) {
            // Calculate current average rotation for left base
            gtsam::Vector3 avg_left_rotation_vector = gtsam::Vector3::Zero();
            for (const auto& pose : left_base_array) {
                gtsam::Vector3 rotation_vector = gtsam::Rot3::Logmap(pose.rotation());
                avg_left_rotation_vector += rotation_vector;
            }
            avg_left_rotation_vector /= left_base_array.size();
            gtsam::Rot3 avg_left_rotation = gtsam::Rot3::Expmap(avg_left_rotation_vector);
            
            // Check rotation difference
            gtsam::Rot3 rotation_diff = avg_left_rotation.inverse() * left_base_current.rotation();
            double rotation_angle = gtsam::Rot3::Logmap(rotation_diff).norm();
            
            // Don't add if rotation differs by more than 0.3 radians (~17 degrees)
            if (rotation_angle > 0.3) {
                add_left = false;
            }
        }
        
        if (!right_base_array.empty()) {
            // Calculate current average rotation for right base
            gtsam::Vector3 avg_right_rotation_vector = gtsam::Vector3::Zero();
            for (const auto& pose : right_base_array) {
                gtsam::Vector3 rotation_vector = gtsam::Rot3::Logmap(pose.rotation());
                avg_right_rotation_vector += rotation_vector;
            }
            avg_right_rotation_vector /= right_base_array.size();
            gtsam::Rot3 avg_right_rotation = gtsam::Rot3::Expmap(avg_right_rotation_vector);
            
            // Check rotation difference
            gtsam::Rot3 rotation_diff = avg_right_rotation.inverse() * right_base_current.rotation();
            double rotation_angle = gtsam::Rot3::Logmap(rotation_diff).norm();
            
            // Don't add if rotation differs by more than 0.3 radians (~17 degrees)
            if (rotation_angle > 0.3) {
                add_right = false;
            }
        }
        
        if (add_left) {
            left_base_array.push_back(left_base_current);
        }
        if (add_right) {
            right_base_array.push_back(right_base_current);
        }
    }

    // Maintain array size for moving average (similar to tube_info_array)
    if (left_base_array.size() >= 100) {
        left_base_array.pop_front();
    }
    if (right_base_array.size() >= 100) {
        right_base_array.pop_front();
    }

    // Calculate moving average poses every 10 counter cycles (similar to tube_info)
    if (left_base_array.size() >= 100 && counter == 0) {
        // Calculate average left_base pose
        gtsam::Point3 avg_left_translation = gtsam::Point3(0, 0, 0);
        gtsam::Vector3 avg_left_rotation_vector = gtsam::Vector3::Zero();
        
        for (const auto& pose : left_base_array) {
            avg_left_translation = avg_left_translation + pose.translation();
            gtsam::Vector3 rotation_vector = gtsam::Rot3::Logmap(pose.rotation());
            avg_left_rotation_vector += rotation_vector;
        }
        
        avg_left_translation = avg_left_translation / 100.0;
        avg_left_rotation_vector /= 100.0;
        gtsam::Rot3 avg_left_rotation = gtsam::Rot3::Expmap(avg_left_rotation_vector);
        
        left_base = gtsam::Pose3(avg_left_rotation, avg_left_translation);

        // Calculate average right_base pose
        gtsam::Point3 avg_right_translation = gtsam::Point3(0, 0, 0);
        gtsam::Vector3 avg_right_rotation_vector = gtsam::Vector3::Zero();
        
        for (const auto& pose : right_base_array) {
            avg_right_translation = avg_right_translation + pose.translation();
            gtsam::Vector3 rotation_vector = gtsam::Rot3::Logmap(pose.rotation());
            avg_right_rotation_vector += rotation_vector;
        }
        
        avg_right_translation = avg_right_translation / 100.0;
        avg_right_rotation_vector /= 100.0;
        gtsam::Rot3 avg_right_rotation = gtsam::Rot3::Expmap(avg_right_rotation_vector);
        
        right_base = gtsam::Pose3(avg_right_rotation, avg_right_translation);
    }
    else if (pose_updated) {
        // Use current poses if we don't have enough samples yet
        left_base = left_base_current;
        right_base = right_base_current;
    }
    
    int max_size = 30;

    if(human_info.LFIN.occluded == false){
        double x = human_info.LFIN.x() / 1000;
        double y = human_info.LFIN.y() / 1000;
        double z = human_info.LFIN.z() / 1000;

        left_finger_pos_array.push_back(Eigen::Vector3d(x,y,z));

        if(left_finger_pos_array.size() > max_size) left_finger_pos_array.pop_front();
    }

    if(human_info.RFIN.occluded == false){
        double x = human_info.RFIN.x() / 1000;
        double y = human_info.RFIN.y() / 1000;
        double z = human_info.RFIN.z() / 1000;

        right_finger_pos_array.push_back(Eigen::Vector3d(x,y,z));

        if(right_finger_pos_array.size() > max_size) right_finger_pos_array.pop_front();
    }

    if(human_info.RFHD.occluded == false && human_info.LFHD.occluded){

        double xr = human_info.RFHD.x() / 1000;
        double yr = human_info.RFHD.y() / 1000;
        double zr = human_info.RFHD.z() / 1000;

        double xl = human_info.LFHD.x() / 1000;
        double yl = human_info.LFHD.y() / 1000;
        double zl = human_info.LFHD.z() / 1000;

        double x = (xr+xl) /2;
        double y = (yr+yl) /2;
        double z = (zr+zl) /2;

        forehead_pos_array.push_back(Eigen::Vector3d(x,y,z));

        if(forehead_pos_array.size() > max_size) forehead_pos_array.pop_front();
    }

    Eigen::Vector3d avg_lfin = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_rfin = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_head = Eigen::Vector3d::Zero();

    for(const auto& pos : left_finger_pos_array) {
        avg_lfin += pos;
    }
    avg_lfin /= left_finger_pos_array.size();

    // Calculate average right finger position
    for(const auto& pos : right_finger_pos_array) {
        avg_rfin += pos;
    }
    avg_rfin /= right_finger_pos_array.size();

    for(const auto& pos : forehead_pos_array) {
        avg_head += pos;
    }
    avg_head /= forehead_pos_array.size();

    lfin = avg_lfin;
    rfin = avg_rfin;
    head = avg_head;
}


gtsam::Pose3 updatePoseInfo1(std::vector<MarkerData>& vicon_data, DHParameters& dh_params, std::vector<double>& joint_conf){

    std::vector<Eigen::Vector3d> ee_positions;

    for (const auto& marker : vicon_data) {
        ee_positions.push_back(Eigen::Vector3d(
    marker.x/1000, marker.y/1000, marker.z/1000));
    }

    gtsam::Pose3 ee =
        calculateFramePose(ee_positions[0],
        ee_positions[1], ee_positions[2], 
        ee_positions[3], 0.12, false, false);

    // std::cout << "EE pose in world" << ee << "\n";

    // Convert degrees to radians and std::vector<double> to Eigen::VectorXd
    Eigen::VectorXd joint_conf_rad(joint_conf.size());
    
    for (size_t i = 0; i < joint_conf.size(); ++i) {
        joint_conf_rad(i) = joint_conf[i] * M_PI / 180.0;
    }
    

    // Compute base poses using inverse forward kinematics
    gtsam::Pose3 base_guess = inverseForwardKinematics(dh_params, joint_conf_rad, ee);

    return base_guess;
}

gtsam::Pose3 updatePoseInfo2(std::vector<MarkerData>& vicon_data, MarkerData other_arm_base_1){

    std::vector<Eigen::Vector3d> base_positions;

    for (const auto& marker : vicon_data) {
        base_positions.push_back(Eigen::Vector3d(
    marker.x/1000, marker.y/1000, marker.z/1000));
    }
    Eigen::Vector3d other_arm_point(other_arm_base_1.x/1000, other_arm_base_1.y/1000, other_arm_base_1.z/1000);

    gtsam::Pose3 base_guess =
        calculateFramePose(base_positions[0],
        base_positions[1], base_positions[2], 
        other_arm_point, 0.133, false, true);

    return base_guess;
}


// world_p1 needs to be in positive X-axis of the local frame
// world_p4 needs to be in negative Z-aixs of the local frame
// See Kinova Gen3 Documentation DH parameter section
gtsam::Pose3 calculateFramePose(const Eigen::Vector3d& world_p1, 
                               const Eigen::Vector3d& world_p2,
                               const Eigen::Vector3d& world_p3, 
                               const Eigen::Vector3d& world_p4, double z_offset, bool p1_in_positive_x, bool p4_in_positive_z) {
    
    // Step 1: Find circle center (circumcenter of triangle P1,P2,P3)
    // For 3D circumcenter calculation, work in the plane containing the three points
    Eigen::Vector3d v1 = world_p2 - world_p1;
    Eigen::Vector3d v2 = world_p3 - world_p1;
    Eigen::Vector3d normal = v1.cross(v2);
    
    // Create orthonormal basis for the plane
    Eigen::Vector3d u1 = v1.normalized();
    Eigen::Vector3d u2 = (v2 - v2.dot(u1) * u1).normalized();
    
    // Project points onto 2D plane coordinates
    Eigen::Vector2d a(0, 0);  // p1 at origin
    Eigen::Vector2d b(v1.norm(), 0);  // p2 on u1 axis
    Eigen::Vector2d c(v2.dot(u1), v2.dot(u2));  // p3 in plane
    
    // Calculate 2D circumcenter
    double d = 2 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));
    
    Eigen::Vector3d world_center;
    if (std::abs(d) < 1e-10) {
        // Points are collinear, return centroid
        world_center = (world_p1 + world_p2 + world_p3) / 3.0;
    } else {
        double ux = ((a.x()*a.x() + a.y()*a.y()) * (b.y() - c.y()) + 
                     (b.x()*b.x() + b.y()*b.y()) * (c.y() - a.y()) + 
                     (c.x()*c.x() + c.y()*c.y()) * (a.y() - b.y())) / d;
        
        double uy = ((a.x()*a.x() + a.y()*a.y()) * (c.x() - b.x()) + 
                     (b.x()*b.x() + b.y()*b.y()) * (a.x() - c.x()) + 
                     (c.x()*c.x() + c.y()*c.y()) * (b.x() - a.x())) / d;
        
        // Convert back to 3D
        world_center = world_p1 + ux * u1 + uy * u2;
    }
    
    // Step 2: Calculate plane normal (two possible directions)
    Eigen::Vector3d plane_normal = v1.cross(v2).normalized();
    
    // Step 3: Use P4 to determine correct z-axis direction
    Eigen::Vector3d center_to_p4 = world_p4 - world_center;
    double dot_product = center_to_p4.dot(plane_normal);
    
    // If P4 is on the positive side of the plane, flip the normal
    Eigen::Vector3d world_z_axis;

    if(p4_in_positive_z){
        world_z_axis = (dot_product > 0) ? plane_normal : -plane_normal;
    }
    else{
        world_z_axis = (dot_product > 0) ? -plane_normal : plane_normal;
    }
    
    // Step 4: Calculate world x-axis (from center to P1)
    Eigen::Vector3d world_x_axis;
    if (p1_in_positive_x) {
        world_x_axis = (world_p1 - world_center).normalized();
    } else {
        // If P1 is not in positive X, flip the x-axis direction
        world_x_axis = -(world_p1 - world_center).normalized();
    }
    
    // Step 5: Calculate world y-axis (z cross x for right-handed system)
    Eigen::Vector3d world_y_axis = world_z_axis.cross(world_x_axis).normalized();
    
    // Step 6: Build rotation matrix for GTSAM
    // GTSAM expects rotation matrix as 3x3 matrix
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = world_x_axis;
    rotation_matrix.col(1) = world_y_axis;
    rotation_matrix.col(2) = world_z_axis;
    
    // Create GTSAM objects
    gtsam::Rot3 rotation(rotation_matrix);
    Eigen::Vector3d translation_with_z_offset = world_center + (z_offset * world_z_axis);
    gtsam::Point3 translation(translation_with_z_offset);
    
    return gtsam::Pose3(rotation, translation);
}


void state_monitor(Eigen::Vector3d& avg_lfin, Eigen::Vector3d& avg_rfin, Eigen::Vector3d& avg_head, TubeInfo& avg_tube_info, std::atomic<int> state_idx){

    static std::deque<double> rfin_head_delta_array;
    double avg_delta = 0;

    bool rfin_on_tube = false;
    bool lfin_on_tube = false;
    bool tube_on_head = false;
    bool r_thumbs_up  = false;

    // Calculate closest distance between avg_rfin and tube axis
    double lfin_tube_distance = std::numeric_limits<double>::max();
    double rfin_tube_distance = std::numeric_limits<double>::max();

    
    {
    Eigen::Vector3d to_point = avg_lfin - avg_tube_info.centroid;
    double t = to_point.dot(avg_tube_info.direction);
    Eigen::Vector3d closest_point_on_axis = avg_tube_info.centroid + t * avg_tube_info.direction;
    lfin_tube_distance = (avg_lfin - closest_point_on_axis).norm();
    }

    {
    Eigen::Vector3d to_point = avg_rfin - avg_tube_info.centroid;
    double t = to_point.dot(avg_tube_info.direction);
    Eigen::Vector3d closest_point_on_axis = avg_tube_info.centroid + t * avg_tube_info.direction;
    rfin_tube_distance = (avg_rfin - closest_point_on_axis).norm();
    }

    double tube_head_distance = avg_tube_info.centroid.z() - avg_head.z();

    {
        double rfin_head_delta = abs(avg_rfin.x()-avg_head.x());
        rfin_head_delta_array.push_back(rfin_head_delta);

        for(const auto& pos : rfin_head_delta_array) {
            avg_delta += pos;
        }
        avg_delta /= rfin_head_delta_array.size();

        if(rfin_head_delta_array.size() > 200){
            rfin_head_delta_array.pop_front();
        }
    }


    if(rfin_tube_distance < 0.05){  // 5cm threshold
        rfin_on_tube = true;
    }

    if(lfin_tube_distance < 0.05){
        lfin_on_tube = true;
    }

    if(tube_head_distance > 0){
        tube_on_head = true;
    }

    if(lfin_on_tube && avg_delta > 0.45){
        r_thumbs_up = true;
    }


    if(rfin_on_tube && !lfin_on_tube && !tube_on_head){
        state_idx.store(1);
    }
    if(rfin_on_tube && lfin_on_tube && tube_on_head){
        state_idx.store(2);
    }
    if(!rfin_on_tube && lfin_on_tube && r_thumbs_up){
        state_idx.store(3);
    }

}
