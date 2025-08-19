#include "ViconInfo.h"


void updateTubeInfo(TubeInfo& tube_info, std::vector<MarkerData>& tube){
    std::vector<Eigen::Vector3d> tubePoints;
    

    for(auto marker : tube){
        double x = marker.x/1000; 
        double y = marker.y/1000;
        double z = marker.z/1000;
        tubePoints.push_back(Eigen::Vector3d(x,y,z));
    }

    tube_info.tube_points = tubePoints;

    // Perform PCA analysis
    
    TubeInfo result;

    int n = tubePoints.size();

    result.centroid = Eigen::Vector3d::Zero();
    for (const auto& p : tubePoints) {
        result.centroid += p;
    }
    result.centroid /= n;


    
    
    Eigen::MatrixXd centered(n, 3);
    for (int i = 0; i < n; ++i) {
        centered.row(i) = (tubePoints[i] - result.centroid).transpose();
    }

    // Compute covariance matrix
    Eigen::Matrix3d covariance = centered.transpose() * centered / (n - 1);

    // Eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

    // The eigenvector with largest eigenvalue is the principal axis
    int maxEigenIndex = 0;
    for (int i = 1; i < 3; ++i) {
        if (solver.eigenvalues()(i) > solver.eigenvalues()(maxEigenIndex)) {
            maxEigenIndex = i;
        }
    }

    result.direction = solver.eigenvectors().col(maxEigenIndex);

    // If y-component is negative, flip the axis orientation about z-axis by 180 degrees
    if (result.direction.y() < 0) {
        result.direction.x() = -result.direction.x();
        result.direction.y() = -result.direction.y();
        // z-component remains unchanged
    }

    // Calculate tube length by projecting points onto axis
    double minProj = std::numeric_limits<double>::max();
    double maxProj = std::numeric_limits<double>::lowest();
    
    for (const auto& p : tubePoints) {
        Eigen::Vector3d centered_point = p - result.centroid;
        double projection = centered_point.dot(result.direction);
        minProj = std::min(minProj, projection);
        maxProj = std::max(maxProj, projection);
    }
    
    result.length = maxProj - minProj;

    // remove when camera is fixed
    result.centroid[2] += 0.04;

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

        if(marker.name == "human_RSHO"){
            human_info.RSHO = gtsam::Point3(x, y, z);
        }

        if(marker.name == "human_LSHO"){
            human_info.LSHO = gtsam::Point3(x, y, z);
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


void updateViconInfo(ViconInterface& vicon, gtsam::Pose3& left_base, gtsam::Pose3& right_base, TubeInfo& tube_info, HumanInfo& human_info, gtsam::Point3& target_info, std::vector<double>& left_conf, std::vector<double>& right_conf, std::string& dh_params_path, std::shared_mutex& vicon_data_mutex, std::shared_mutex& joint_data_mutex){
    
    static int counter = 0;
    static std::deque<TubeInfo> tube_info_array;

    std::unique_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
    

    std::vector<MarkerData> right_base_data;
    std::vector<MarkerData> left_base_data;

    right_base_data.push_back(vicon.getMarkerPosition("right_base1"));
    right_base_data.push_back(vicon.getMarkerPosition("right_base2"));
    right_base_data.push_back(vicon.getMarkerPosition("right_base3"));
    // right_base_data.push_back(vicon.getMarkerPosition("right_base4"));

    left_base_data.push_back(vicon.getMarkerPosition("left_base1"));
    left_base_data.push_back(vicon.getMarkerPosition("left_base2"));
    left_base_data.push_back(vicon.getMarkerPosition("left_base3"));
    // left_base_data.push_back(vicon.getMarkerPosition("left_base4"));


    std::vector<MarkerData> tube  = vicon.getMarkerPositions("tube");
    std::vector<MarkerData> human = vicon.getMarkerPositions("human");
    std::vector<MarkerData> target= vicon.getMarkerPositions("target");
    
    TubeInfo tube_info_snapshot;

    updateTubeInfo(tube_info_snapshot, tube);
    updateHumanInfo(human_info, human);
    updateTargetInfo(target_info, target);

    tube_info_array.push_back(tube_info_snapshot);

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


    // Estimate left base pose if not occluded
    if (!left_base_occluded && !right_base_occluded) {

        gtsam::Pose3 left_base_guess2 = updatePoseInfo2(left_base_data, right_base_data.front());
        gtsam::Pose3 right_base_guess2 = updatePoseInfo2(right_base_data, left_base_data.front());

        left_base = left_base_guess2;
        right_base = right_base_guess2;
    }

    // // Estimate right base pose if not occluded
    // if (!right_base_occluded) {

    //     gtsam::Pose3 right_base_guess2 = updatePoseInfo2(right_base_data, left_base_data.front());

    //     right_base = right_base_guess2;
    // }

    // for (auto& marker : tube){
    //     std::cout << "Tube data: \n";
    //     std::cout << marker.x << " " << marker.y << " " << marker.z << "\n";
    // }

    // for (auto& marker : human){
    //     std::cout << "human data: \n";
    //     std::cout << marker.x << " " << marker.y << " " << marker.z << "\n";
    // }
    
}


gtsam::Pose3 updatePoseInfo1(std::vector<MarkerData>& vicon_data, std::string& dh_params_path, std::vector<double>& joint_conf){

    std::vector<Eigen::Vector3d> ee_positions;

    for (const auto& marker : vicon_data) {
        ee_positions.push_back(Eigen::Vector3d(
    marker.x/1000, marker.y/1000, marker.z/1000));
    }

    gtsam::Pose3 ee =
        calculateFramePose(ee_positions[0],
        ee_positions[1], ee_positions[2], 
        ee_positions[3], 0.14, true);


    DHParameters dh_params = createDHParams(dh_params_path);

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
        other_arm_point, 0.133, false);

    return base_guess;
}


// world_p1 needs to be in positive X-axis of the local frame
// world_p4 needs to be in negative Z-aixs of the local frame
// See Kinova Gen3 Documentation DH parameter section
gtsam::Pose3 calculateFramePose(const Eigen::Vector3d& world_p1, 
                               const Eigen::Vector3d& world_p2,
                               const Eigen::Vector3d& world_p3, 
                               const Eigen::Vector3d& world_p4, double z_offset, bool p1_in_positive_x) {
    
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
    Eigen::Vector3d world_z_axis = (dot_product > 0) ? plane_normal : -plane_normal;
    
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