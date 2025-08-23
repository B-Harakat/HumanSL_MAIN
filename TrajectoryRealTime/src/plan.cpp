#include "plan.h"
#include "GenerateLogs.h"
#include "Jacobian.h"
#include <thread>
#include <chrono>
#include <deque>
#include <shared_mutex>

// Constructor
Gen3Arm::Gen3Arm(const std::string& ip_addr, 
         const std::string& robot_urdf_path, 
         const std::string& dh_params_path, 
         const std::string& joint_limits_path) 
    : ViconSDF(), InitializeTrajectory(createDHParams(dh_params_path)), OptimizeTrajectory() {
    
    dh_params_ = createDHParams(dh_params_path);
    std::tie(pos_limits_, vel_limits_) = createJointLimits(joint_limits_path);
    
    ip_ = ip_addr;
}

void Gen3Arm::make_sdf(
                   TubeInfo& tube_info, 
                   HumanInfo& human_info, 
                   bool include_tube,
                   const gtsam::Pose3& other_arm_base, 
                   const std::vector<double>& other_arm_config_vec) {
    
    gtsam::Vector other_arm_config = Eigen::Map<const Eigen::VectorXd>(other_arm_config_vec.data(), other_arm_config_vec.size()) * M_PI/180;

    double cell_size = 0.06; // 6 cm workspace grid resolution
    
    std::array<double, 3> workspace_size = {2.0, 3.5, 2.5};  // Relative to MUVE platform origin, the workspace spans [-1.0, -1.5, 0.0] to [1.0, 2.0, 2.0]
    std::array<double, 3> workspace_origin = {-1.0, -1.5, 0.0}; // x is running mill width direction, y is running mill length direction, z is height

    std::unique_ptr<gpmp2::ArmModel> other_arm_model = ArmModel::createArmModel(other_arm_base, dh_params_);
    
    auto grid_with_man = HumanObstacleFromVicon(human_info, cell_size, workspace_size, workspace_origin);
    auto grid_with_arm_and_man = ArmObstacleFromVicon(*grid_with_man, other_arm_model, other_arm_config, 0.01);
    auto grid_with_tube_and_arm_and_man = TubeObstacleFromVicon(*grid_with_arm_and_man, tube_info, 0.03);
    
    if (include_tube == true) { 
        sdf = createSDFFromOccupancyGrid(*grid_with_tube_and_arm_and_man);
        dataset_logs = *grid_with_tube_and_arm_and_man;
    }
    else {
        sdf = createSDFFromOccupancyGrid(*grid_with_arm_and_man);
        dataset_logs = *grid_with_arm_and_man;
    }
}

gtsam::Pose3 Gen3Arm::forward_kinematics(gtsam::Pose3& base_pose, 
                                                   std::vector<double>& current_joint_pos) {
    
    gtsam::Pose3 ee_pose;

    std::unique_ptr<gpmp2::ArmModel> arm_model = createArmModel(base_pose, dh_params_);

    gtsam::Vector start_conf = Eigen::Map<const Eigen::VectorXd>(
                    current_joint_pos.data(), current_joint_pos.size()) * M_PI / 180.0;

    // ee_pose = createPoseFromConf(*arm_model, start_conf, false);
    ee_pose = forwardKinematics(dh_params_, start_conf, base_pose);
    return ee_pose;
}

gtsam::Pose3 Gen3Arm::create_target_pose(const HumanInfo& human_info,
                                         const double& offset_from_human_y,
                                         const gtsam::Pose3& start_pose,
                                         double offset) {
    
    // Calculate target y position
    gtsam::Point3 start_pos = start_pose.translation();
    
    double target_y = human_info.bounds.max_y + offset_from_human_y + 0.4;
    double target_x = human_info.bounds.min_x + (human_info.bounds.max_x - human_info.bounds.min_x)/2 - 0.1;
    double target_z = human_info.bounds.max_z + offset;

    gtsam::Rot3 target_orientation = gtsam::Rot3::Rz(M_PI);
    
    gtsam::Point3 target_position(target_x, target_y,
    target_z);

    gtsam::Pose3 target_pose(target_orientation, target_position);
    
    return target_pose;
}

gtsam::Pose3 Gen3Arm::over_head_pose(const Eigen::Vector3d& head_info,
                            const gtsam::Pose3& base_pose,             
                            const gtsam::Pose3& start_pose,
                            const double& offset_from_base_y,
                            const double& offset_from_head_x,
                            const double& offset_from_head_z) {
    
    // Calculate target y position
    gtsam::Point3 start_pos = start_pose.translation();
    
    
    double target_y = base_pose.translation().y() + offset_from_base_y;
    double target_x = head_info.x() + offset_from_head_x;
    double target_z = head_info.z() + offset_from_head_z;

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


gtsam::Pose3 Gen3Arm::installtion_pose(const gtsam::Point3& target_info,             
                            const gtsam::Pose3& start_pose) {
    
    // Calculate target y position
    gtsam::Point3 start_pos = start_pose.translation();
    
    double target_y = start_pos.y();
    double target_x = target_info.x();
    double target_z = target_info.z();

    gtsam::Rot3 target_orientation = gtsam::Rot3::Rz(M_PI); // 180 degrees about z-axis
    
    gtsam::Point3 target_position(target_x, target_y, target_z);

    gtsam::Pose3 target_pose(target_orientation, target_position);
    
    return target_pose;
}

gtsam::Pose3 Gen3Arm::over_head_pipe_pose(const gtsam::Pose3& other_arm_info,             
                            HumanInfo& human_info,
                            const double& offset_from_human_max_y,
                            const double& offset_from_tube_z) {
    
    gtsam::Point3 other_position = other_arm_info.translation(); 
    double target_y = human_info.bounds.max_y + offset_from_human_max_y;
    
    auto angle = 0.0; // rad
    double offset_x = sin(angle) * offset_from_tube_z;
    double offset_z = cos(angle) * offset_from_tube_z;

    double target_x = other_position.x() + offset_x;
    double target_z = other_position.z() - offset_z;

    
    gtsam::Rot3 base_orientation = gtsam::Rot3::Rz(M_PI); // 180 degrees about z-axis
    gtsam::Rot3 target_orientation;
    
    target_orientation = base_orientation * gtsam::Rot3::Ry(angle);  // 30 degrees about new y-axis
    
    
    gtsam::Point3 target_position(target_x, target_y,
    target_z);

    gtsam::Pose3 target_pose(target_orientation, target_position);
    
    return target_pose;
}



void Gen3Arm::plan_joint(JointTrajectory& trajectory, 
                     std::vector<double>& current_joint_pos, 
                     const gtsam::Pose3& base_pose, 
                     const TubeInfo& tube_info,
                     double offset_from_base_y,
                     double offset_from_tube_z, 
                     double total_time_sec, 
                     size_t total_time_step,
                     int control_frequency, bool tune_pose, 
                     double y_pos_tolerance, double y_rot_tolerance) {
    
    std::unique_ptr<gpmp2::ArmModel> arm_model = createArmModel(base_pose, dh_params_);
    arm_model_logs  = *arm_model;

    visualizeTrajectoryStatic(current_joint_pos, arm_model_logs, dataset_logs, base_pose);
    // std::cout << "Verify starting condition, press ENTER to proceed with IK solving. \n";
    // std::cin.get();

    gtsam::Vector start_conf = Eigen::Map<const Eigen::VectorXd>(
        current_joint_pos.data(), current_joint_pos.size()) * M_PI / 180.0;

    gtsam::Vector start_vel = gtsam::Vector::Zero(7);


    gtsam::Pose3 best_end_pose;  // Will be filled with the best pose found by IK search
    
    
    double dt = 1.0 / control_frequency;

    TrajectoryResult result;
    
    int counter = 0;
    double best_final_error = 50000;

    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Initialize cumulative timing variables
    std::chrono::milliseconds total_initiation_duration(0);
    std::chrono::milliseconds total_optimization_duration(0);

    while(true){

            auto init_start_time = std::chrono::high_resolution_clock::now();

            gtsam::Values init_values = initJointTrajectoryFromVicon(start_conf, tube_info,
                                                                    offset_from_base_y, offset_from_tube_z,
                                                                    base_pose, total_time_step, best_end_pose, tune_pose);
            
            auto init_end_time = std::chrono::high_resolution_clock::now();
            auto initiation_duration = std::chrono::duration_cast<std::chrono::milliseconds>(init_end_time - init_start_time);
            total_initiation_duration += initiation_duration;
                                                
            std::cout << "Best Pose: " << best_end_pose << "\n";
            // Use the actual best pose found by our IK search (not a temporary pose)
            
            auto optimization_start_time = std::chrono::high_resolution_clock::now();
            result = optimizeJointTrajectory(
                                    *arm_model, *sdf, init_values, best_end_pose, 
                                    start_conf, start_vel, pos_limits_, vel_limits_, 
                                    total_time_step, total_time_sec, dt, 
                                    y_pos_tolerance, y_rot_tolerance);
            
            auto optimization_end_time = std::chrono::high_resolution_clock::now();
            auto optimization_duration = std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end_time - optimization_start_time);
            total_optimization_duration += optimization_duration;

            if(result.final_error < 50.0) break;

            counter++;
            best_final_error = (best_final_error > result.final_error) ? result.final_error : best_final_error;
            if(counter > 10) break;
        
    }

    
    result_logs = result;
    target_pose_logs = best_end_pose;

    result.initiation_duration = total_initiation_duration;
    result.optimization_duration = total_optimization_duration;

    std::cout << "Final Error: " << result.final_error << "\n";

    trajectory = convertTrajectory<JointTrajectory>(result, dt); 

    if(counter > 10){ 
            visualizeTrajectory(trajectory.pos, *arm_model, dataset_logs, base_pose);
            saveTrajectoryResultToYAML(result,"failed_plan");
            throw std::runtime_error((std::stringstream{} << "Could not generate good enough trajectory, best final error: " << best_final_error << "\n").str());
    }
}


void Gen3Arm::plan_joint(JointTrajectory& trajectory, 
                     std::vector<double>& current_joint_pos, 
                     const gtsam::Pose3& base_pose, 
                     const gtsam::Pose3& target_pose, 
                     double total_time_sec,
                     size_t total_time_step,
                     int control_frequency,
                    double y_pos_tolerance, double y_rot_tolerance) {
    
    std::unique_ptr<gpmp2::ArmModel> arm_model = createArmModel(base_pose, dh_params_);
    arm_model_logs  = *arm_model;

    visualizeTrajectoryStatic(current_joint_pos, arm_model_logs, dataset_logs, base_pose);
    // std::cout << "Verify starting condition, press ENTER to proceed with IK solving. \n";
    // std::cin.get();

    gtsam::Vector start_conf = Eigen::Map<const Eigen::VectorXd>(
        current_joint_pos.data(), current_joint_pos.size()) * M_PI / 180.0;

    gtsam::Vector start_vel = gtsam::Vector::Zero(7);


    double dt = 1.0 / control_frequency;

    TrajectoryResult result;
    
    int counter = 0;
    double best_final_error = 50000;

    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Initialize cumulative timing variables
    std::chrono::milliseconds total_initiation_duration(0);
    std::chrono::milliseconds total_optimization_duration(0);

    while(true){

            auto init_start_time = std::chrono::high_resolution_clock::now();

            gtsam::Values init_values = initJointTrajectoryFromTarget(start_conf, target_pose, base_pose, 
                                                                     total_time_step);
            
            auto init_end_time = std::chrono::high_resolution_clock::now();
            auto initiation_duration = std::chrono::duration_cast<std::chrono::milliseconds>(init_end_time - init_start_time);
            total_initiation_duration += initiation_duration;
                                                
            std::cout << "Target Pose: " << target_pose << "\n";
            // Use the actual best pose found by our IK search (not a temporary pose)
            
            auto optimization_start_time = std::chrono::high_resolution_clock::now();
            result = optimizeJointTrajectory(
                                    *arm_model, *sdf, init_values, target_pose, 
                                    start_conf, start_vel, pos_limits_, vel_limits_, 
                                    total_time_step, total_time_sec, dt, 
                                    y_pos_tolerance, y_rot_tolerance);
            
            auto optimization_end_time = std::chrono::high_resolution_clock::now();
            auto optimization_duration = std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end_time - optimization_start_time);
            total_optimization_duration += optimization_duration;

            if(result.final_error < 50.0) break;

            counter++;
            best_final_error = (best_final_error > result.final_error) ? result.final_error : best_final_error;
            if(counter > 10) break;
        
    }

    
    result_logs = result;
    target_pose_logs = target_pose;

    result.initiation_duration = total_initiation_duration;
    result.optimization_duration = total_optimization_duration;

    std::cout << "Final Error: " << result.final_error << "\n";

    trajectory = convertTrajectory<JointTrajectory>(result, dt); 

    if(counter > 10){ 
            visualizeTrajectory(trajectory.pos, *arm_model, dataset_logs, base_pose);
            saveTrajectoryResultToYAML(result,"failed_plan");
            throw std::runtime_error((std::stringstream{} << "Could not generate good enough trajectory, best final error: " << best_final_error << "\n").str());
    }
}


void Gen3Arm::plan_joint(JointTrajectory& trajectory, 
                     std::vector<double>& current_joint_pos,  
                     std::vector<double>& target_joint_pos, 
                     const gtsam::Pose3& base_pose, 
                     double total_time_sec, 
                     size_t total_time_step,
                     int control_frequency,
                    double y_pos_tolerance, double y_rot_tolerance) {
    
    std::unique_ptr<gpmp2::ArmModel> arm_model = createArmModel(base_pose, dh_params_);
    arm_model_logs  = *arm_model;

    // visualizeTrajectoryStatic(current_joint_pos, arm_model_logs, dataset_logs, base_pose);
    // std::cout << "Verify starting condition, press ENTER to proceed with IK solving. \n";
    // std::cin.get();
    
    gtsam::Vector start_conf = Eigen::Map<const Eigen::VectorXd>(
        current_joint_pos.data(), current_joint_pos.size()) * M_PI / 180.0;

    gtsam::Vector dummy_conf = Eigen::Map<const Eigen::VectorXd>(
        target_joint_pos.data(), target_joint_pos.size()) * M_PI / 180.0;

    gtsam::Vector start_vel = gtsam::Vector::Zero(7);

    std::vector<gtsam::Pose3> joint_poses;
    arm_model->fk_model().forwardKinematics(dummy_conf, {}, joint_poses);
    gtsam::Pose3 target_pose = joint_poses.back();
    

    gtsam::Values init_values = initJointTrajectoryFromTarget(start_conf, target_pose,
                                                               base_pose, total_time_step);
    
    double dt = 1.0/control_frequency;
    TrajectoryResult result = optimizeJointTrajectory(
                            *arm_model, *sdf, init_values, target_pose, 
                            start_conf, start_vel, pos_limits_, vel_limits_, 
                            total_time_step, total_time_sec, dt, y_pos_tolerance, y_rot_tolerance);
    
    result_logs = result;
    target_pose_logs = target_pose;
    
    trajectory = convertTrajectory<JointTrajectory>(result, dt); 
}

bool Gen3Arm::replan_joint(
                     const JointTrajectory& old_trajectory,
                     JointTrajectory& new_trajectory, 
                     const gtsam::Pose3& base_pose, 
                     const TubeInfo& tube_info,
                     double offset_from_base_y,
                     double offset_from_tube_z,
                     double total_time_sec, 
                     size_t total_time_step,
                     int control_frequency) {
    
    std::cout << "Starting replan_joint - extracting future states from trajectory..." << std::endl;
    
    // Step 1: Extract 3 consecutive states starting from timestep 100 (200ms)
    const size_t future_timestep = static_cast<size_t>(1000/(1000.0/control_frequency));  // 250ms / 1ms = 250 timesteps
    Eigen::VectorXd pos_deg;
    Eigen::VectorXd vel_deg;

    if (future_timestep > old_trajectory.pos.size()-1){
        pos_deg = old_trajectory.pos[old_trajectory.pos.size()-1];
        vel_deg = old_trajectory.vel[old_trajectory.pos.size()-1];
    }
    else{
        pos_deg = old_trajectory.pos[future_timestep];
        vel_deg = old_trajectory.vel[future_timestep];
    }
    
    
    std::cout<<"original old_trajectory at change: ";
    for(auto& k : pos_deg){ std::cout << std::round(k*100)/100 << ", ";}
    std::cout<< "\n";

    gtsam::Vector extracted_pos = pos_deg * M_PI / 180.0;
    gtsam::Vector extracted_vel = vel_deg * M_PI / 180.0;
    
    std::unique_ptr<gpmp2::ArmModel> arm_model = createArmModel(base_pose, dh_params_);
    arm_model_logs  = *arm_model;

    // Visualize the starting condition (from extracted state)
    // std::vector<double> extracted_start_deg;
    // for (int i = 0; i < extracted_pos[0].size(); ++i) {
    //     extracted_start_deg.push_back(extracted_pos[0](i) * 180.0 / M_PI);
    // }
    // visualizeTrajectoryStatic(extracted_start_deg, arm_model_logs, dataset_logs, base_pose);
    // std::cout << "Verify starting condition from 200ms future state, press ENTER to proceed with re-planning. \n";
    // std::cin.get();

    double dt = 1.0/control_frequency;

    TrajectoryResult result;
    
    int counter = 0;
    double best_final_error = 50000;

    gtsam::Pose3 target_pose;

    while(true){

        // Step 2: Use re_initializeTrajectory with the extracted states
        gtsam::Values init_values = initJointTrajectoryFromVicon(extracted_pos, tube_info,
                                                         offset_from_base_y, offset_from_tube_z,
                                                         base_pose, total_time_step, target_pose);
        
        double dt = 1.0 / control_frequency;
        // Step 3: Use re_optimizeTrajectory with the extracted states
        result = optimizeJointTrajectory(
                                *arm_model, *sdf, init_values, target_pose, 
                                extracted_pos, extracted_vel, pos_limits_, vel_limits_, 
                                total_time_step, total_time_sec, dt);

        if(result.final_error < 1000.0) break;

        
        best_final_error = (best_final_error > result.final_error) ? result.final_error : best_final_error;
        
        if(counter > 10) break;
        
        counter++;
    }

    std::cout << "Best Pose: " << target_pose << std::endl;

    result_logs = result;

    saveTrajectoryResultToYAML(result_logs,"replanned");

    if(counter > 10) return false;
            
    
    // Step 4: Convert to trajectory starting from t=0 (the trajectory begins with the 200ms future state)
    new_trajectory = convertTrajectory<JointTrajectory>(result, dt); 
    
    std::cout << "Replan completed. New trajectory starts from 200ms future state but at t=0." << std::endl;

    return true;
}

void Gen3Arm::check_replan(const Eigen::VectorXd& trajectory_last_pos,
                          const gtsam::Pose3& base_pose,
                          const TubeInfo& tube_info, 
                          std::shared_mutex& vicon_data_mutex,
                          std::atomic<bool>& check_replan_flag, std::atomic<bool>& execution_ongoing_flag) {
    
    std::cout << "Starting check_replan() monitoring..." << std::endl;
    
    // Rolling average parameters
    const size_t rolling_window_size = 50;  // Number of samples for rolling average (500ms at 10ms intervals)
    double position_threshold = 1000;  
    double normal_threshold = 1000;   
    
    // Static deques for rolling averages
    static std::deque<double> position_errors;
    static std::deque<double> normal_errors;
    
    // Clear deques at start of monitoring
    position_errors.clear();
    normal_errors.clear();
    
    while (!check_replan_flag.load() && execution_ongoing_flag.load()) {
        // Check if trajectory has data
        
        // Convert to radians for forward kinematics
        std::vector<double> final_joint_pos_vec;
        for (int i = 0; i < trajectory_last_pos.size(); ++i) {
            final_joint_pos_vec.push_back(trajectory_last_pos(i));
        }
        
        // Calculate end pose using forward kinematics
        gtsam::Pose3 current_end_pose = forward_kinematics(
            const_cast<gtsam::Pose3&>(base_pose), final_joint_pos_vec);

        // Thread-safe extraction of tube_info to snapshot
        TubeInfo tube_info_snapshot;
        {
            std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
            tube_info_snapshot = tube_info;
        }
        
        // Extract y-position of current end pose
        double current_y = current_end_pose.translation().y();
        
        // Find point on tube with same y-position using centroid and direction
        // Point on tube axis: P = centroid + t * direction
        // For same y-position: centroid.y + t * direction.y = current_y
        // Solve for t: t = (current_y - centroid.y) / direction.y
        double t = (current_y - tube_info_snapshot.centroid.y()) / tube_info_snapshot.direction.y();
        Eigen::Vector3d closest_tube_point = tube_info_snapshot.centroid + t * tube_info_snapshot.direction;
        
        // Calculate distance between current_end_pose and closest point on tube
        gtsam::Point3 current_pos = current_end_pose.translation();
        Eigen::Vector3d current_pos_eigen(current_pos.x(), current_pos.y(), current_pos.z());
        double position_error = (current_pos_eigen - closest_tube_point).norm();
        
        // Calculate normal distance between z-axis and tube axis
        // Get z-axis of current end pose
        gtsam::Point3 z_axis_gtsam = current_end_pose.rotation().r3();
        Eigen::Vector3d z_axis_current(z_axis_gtsam.x(), z_axis_gtsam.y(), z_axis_gtsam.z());

        
        // DEBUG: Print z-axis and tube direction
        // std::cout << "Z-point: [" << current_pos.x()<< ", " << current_pos.y() << ", " << current_pos.z() << "]" << std::endl;
        // std::cout << "tube-point: [" << closest_tube_point.x()<< ", " << closest_tube_point.y() << ", " << closest_tube_point.z() << "]" << std::endl;

        // std::cout << "Z-axis: [" << z_axis_current.x() << ", " << z_axis_current.y() << ", " << z_axis_current.z() << "]" << std::endl;
        // std::cout << "Tube direction: [" << tube_info_snapshot.direction.x() << ", " << tube_info_snapshot.direction.y() << ", " << tube_info_snapshot.direction.z() << "]" << std::endl;
        

        // Calculate normal distance between the two lines (z-axis and tube axis)
        // For two lines with directions d1, d2 and points p1, p2:
        // normal_distance = |(p2-p1) · (d1 × d2)| / ||d1 × d2||
        Eigen::Vector3d pos_diff = current_pos_eigen - closest_tube_point;
        Eigen::Vector3d cross_product = z_axis_current.cross(tube_info_snapshot.direction);
        double normal_error = std::abs(pos_diff.dot(cross_product)) / cross_product.norm();
        
        // Add current errors to rolling average deques
        position_errors.push_back(position_error);
        normal_errors.push_back(normal_error);

        position_threshold = 0.25; // 20cm average position difference
        normal_threshold = 0.10;  // 5cm average normal distance between axes
    
        // Maintain rolling window size
        if (position_errors.size() > rolling_window_size) {
            position_errors.pop_front();
        }
        if (normal_errors.size() > rolling_window_size) {
            normal_errors.pop_front();
        }
        
        // Calculate rolling averages (only after we have some samples)
        if (position_errors.size() >= 30) {  // Wait for at least 10 samples before checking
            double avg_position_error = 0.0;
            double avg_normal_error = 0.0;
            
            for (const auto& error : position_errors) {
                avg_position_error += error;
            }
            avg_position_error /= position_errors.size();
            
            for (const auto& error : normal_errors) {
                avg_normal_error += error;
            }
            avg_normal_error /= normal_errors.size();

            // std::cout << "Average position error: " << avg_position_error << "\n"; 

            std::cout << "Average normal error: " << avg_normal_error <<"\n";
            
            // Check if average thresholds are exceeded
            if (avg_position_error > position_threshold || avg_normal_error > normal_threshold) {
                std::cout << "Replanning threshold exceeded (rolling average)!" << std::endl;
                std::cout << "Average position error: " << avg_position_error 
                         << " (threshold: " << position_threshold << ")" << std::endl;
                std::cout << "Average rotation error: " << avg_normal_error 
                         << " (threshold: " << normal_threshold << ")" << std::endl;
                std::cout << "Current samples in rolling average: " << position_errors.size() << std::endl;
                
                check_replan_flag.store(true);
                position_errors.clear();
                normal_errors.clear();
                break;
            }
        }
        
        // Sleep for a short duration before next check
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Check every 10ms
    }
    
    std::cout << "check_replan() monitoring stopped." << std::endl;
}

void Gen3Arm::replan(JointTrajectory& current_trajectory,
                    JointTrajectory& new_trajectory,
                    gtsam::Pose3& base_pose,
                    std::shared_mutex& vicon_data_mutex,
                    std::shared_mutex& joint_data_mutex,
                    std::mutex& trajectory_mutex,
                    std::atomic<bool>& replan_triggered,
                    std::atomic<bool>& new_trajectory_ready,
                    std::atomic<bool>& execution_ongoing_flag,
                    HumanInfo& human_info, TubeInfo& tube_info,
                    gtsam::Pose3& other_base_pose,
                    std::vector<double>& other_conf,
                    size_t total_time_step, int control_frequency
                ) {
    
    std::cout << "Replan thread started." << std::endl;
    
    // Static flag to prevent multiple concurrent replanning
    static std::atomic<bool> replanning_in_progress{false};
  
    while (execution_ongoing_flag.load()) {
        // Only proceed if not already replanning and replan hasn't been triggered
        if (!replanning_in_progress.load() && !replan_triggered.load()) {
            
            // Thread-safe copy of current trajectory for monitoring
            Eigen::VectorXd trajectory_last_pos;
            {
                std::lock_guard<std::mutex> lock(trajectory_mutex);
                trajectory_last_pos = current_trajectory.pos.back();
            }
         
            // Start check_replan in a separate thread for this iteration
            std::atomic<bool> check_replan_flag{false};
   
            std::cout << "last pos: ";
            for(auto& k : trajectory_last_pos){ std::cout << std::round(k*100)/100 << ", ";}
            std::cout << "\n";
            check_replan(trajectory_last_pos, base_pose, tube_info, vicon_data_mutex, check_replan_flag, execution_ongoing_flag);
              
            
            // If replanning is needed
            if (check_replan_flag.load()) {

                std::cout << "Error threshold exceeded, starting replanning process..." << std::endl;
                
                // Set flag to prevent concurrent replanning
                replanning_in_progress.store(true);
                
                // Set replan triggered flag
                replan_triggered.store(true);
                
                // Take snapshots of vicon data
                TubeInfo tube_info_snapshot;
                HumanInfo human_info_snapshot;
                gtsam::Pose3 base_pose_snapshot;
                gtsam::Pose3 other_base_pose_snapshot;
                std::vector<double> other_conf_snapshot;

                {
                    std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
                    tube_info_snapshot = tube_info;
                    human_info_snapshot = human_info;
                    base_pose_snapshot = base_pose;
                    other_base_pose_snapshot = other_base_pose;
                }

                {
                    std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
                    other_conf_snapshot = other_conf;
                }

                JointTrajectory trajectory_snapshot;
                double remaining_time_sec;
                {
                    std::lock_guard<std::mutex> lock(trajectory_mutex);
                    trajectory_snapshot = current_trajectory;
                    size_t remaining_pos = current_trajectory.pos.size();
                    size_t size_to_skip = static_cast<size_t>(1000/(1000.0/control_frequency)); 
                    remaining_time_sec = (remaining_pos > size_to_skip && ((remaining_pos-size_to_skip)*(1.0/control_frequency)) > 0.5) ? ((static_cast<double>(remaining_pos-size_to_skip))*(1.0/control_frequency)) : 0.5;
                }
                
                std::cout << "Calling replan_joint() with snapshot data..." << std::endl;

                make_sdf(tube_info_snapshot, human_info_snapshot, true, other_base_pose_snapshot, other_conf);
              
                // Generate new trajectory (this takes ~200ms)
                if(replan_joint(
                    trajectory_snapshot,
                    new_trajectory,
                    base_pose_snapshot,
                    tube_info_snapshot,
                    0.3, 0.12,  // offset parameters
                    remaining_time_sec, total_time_step, control_frequency    // time and timestep parameters
                )){
                    std::cout << "New trajectory generated successfully." << std::endl;
                    
                    // Signal that new trajectory is ready
                    new_trajectory_ready.store(true);
                    
                    std::cout << "Replanning cycle completed, ready for next replan." << std::endl;
                    replanning_in_progress.store(false);
                    
                    visualizeTrajectory(new_trajectory.pos, arm_model_logs, dataset_logs, base_pose_snapshot);
                }
            } 
        }
        
        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "Replan thread terminated." << std::endl;
}


void Gen3Arm::plan_task(TaskTrajectory& trajectory, 
                    const gtsam::Pose3& start_pose,
                    const gtsam::Pose3& end_pose,
                    const double duration_sec,
                    const double percentage,
                    const double height,
                    const int control_frequency) {
    
    double dt = 1.0/control_frequency;

    auto [pos, vel, acc] = initTaskSpaceTrajectory(start_pose, end_pose, duration_sec, percentage, height, dt);
    
    trajectory.pos = pos;
    trajectory.vel = vel;
    trajectory.acc = acc;
}


void Gen3Arm::plan_task(JointTrajectory& trajectory, 
                    const gtsam::Pose3& start_pose,
                    const gtsam::Pose3& end_pose,
                    const gtsam::Pose3& base_pose,
                    const std::vector<double>& current_joint_pos,
                    const double total_time_sec,
                    const int total_time_step,
                    const double percentage,
                    const double height,
                    const int control_frequency,
                    bool target_pose_only,
                    double y_pos_tolerance,
                    double y_rot_tolerance) {
    
    std::unique_ptr<gpmp2::ArmModel> arm_model = createArmModel(base_pose, dh_params_);
    arm_model_logs  = *arm_model;

    visualizeTrajectoryStatic(current_joint_pos, arm_model_logs, dataset_logs, base_pose);
    // std::cout << "Verify starting condition, press ENTER to proceed with IK solving. \n";
    // std::cin.get();

    gtsam::Vector start_conf = Eigen::Map<const Eigen::VectorXd>(
        current_joint_pos.data(), current_joint_pos.size()) * M_PI / 180.0;


    gtsam::Pose3 best_end_pose;  // Will be filled with the best pose found by IK search
    
    
    double dt = 1.0 / control_frequency;

    TrajectoryResult result;
    
    int counter = 0;
    double best_final_error = 50000;

    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Initialize cumulative timing variables
    std::chrono::milliseconds total_initiation_duration(0);
    std::chrono::milliseconds total_optimization_duration(0);

    while(true){

        std::deque<gtsam::Pose3> pose_trajectory;
        gtsam::Values init_values;
        auto init_start_time = std::chrono::high_resolution_clock::now();

        if(target_pose_only){
            try{
                init_values = 
                        initJointTrajectoryFromTarget(start_conf, end_pose, 
                                base_pose, total_time_step);
                
                // Fill pose_trajectory with identity poses up to total_time_step + 1 size
                pose_trajectory.resize(total_time_step + 1);
                for (int i = 0; i < total_time_step; ++i) {
                    pose_trajectory[i] = gtsam::Pose3();
                }
                // Set the last entry to end_pose
                pose_trajectory[total_time_step] = end_pose;
                
            }
            catch(const std::exception& e){
                continue;
            }
        }
        else{
            try{
                init_values = 
                        initTaskSpaceTrajectory(start_pose, end_pose, 
                                base_pose, start_conf, pose_trajectory,
                                    percentage, height, total_time_step); 
                                                                    
            }
            catch(const std::exception& e){
                continue;
            }
        }

        auto init_end_time = std::chrono::high_resolution_clock::now();
        auto initiation_duration = std::chrono::duration_cast<std::chrono::milliseconds>(init_end_time - init_start_time);
        total_initiation_duration += initiation_duration;
                                            
        std::cout << "Final Pose: " << pose_trajectory.back() << "\n";
        // Use the actual best pose found by our IK search (not a temporary pose)
        
        auto optimization_start_time = std::chrono::high_resolution_clock::now();
        result = optimizeTaskTrajectory(
                                *arm_model, *sdf, init_values, pose_trajectory, 
                                start_conf, pos_limits_, vel_limits_, 
                                total_time_step, total_time_sec, dt, target_pose_only, 
                                y_pos_tolerance, y_rot_tolerance);
        
        auto optimization_end_time = std::chrono::high_resolution_clock::now();
        auto optimization_duration = std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end_time - optimization_start_time);
        total_optimization_duration += optimization_duration;

        if(result.final_error < 50.0) break;

        counter++;
        best_final_error = (best_final_error > result.final_error) ? result.final_error : best_final_error;
        if(counter > 10) break;
    }

    
    result_logs = result;
    target_pose_logs = best_end_pose;

    result.initiation_duration = total_initiation_duration;
    result.optimization_duration = total_optimization_duration;

    std::cout << "Final Error: " << result.final_error << "\n";

    trajectory = convertTrajectory<JointTrajectory>(result, dt); 

    if(counter > 10){ 
            visualizeTrajectory(trajectory.pos, *arm_model, dataset_logs, base_pose);
            
            throw std::runtime_error((std::stringstream{} << "Could not generate good enough trajectory, best final error: " << best_final_error << "\n").str());
    }
}

void Gen3Arm::plan_cartesian_z(JointTrajectory& trajectory, 
    std::vector<double>& current_joint_pos, 
    const gtsam::Pose3& base_pose, 
    const TubeInfo& tube_info,
    double total_time_sec, 
    int control_frequency,
    bool to_tube) {

    visualizeTrajectoryStatic(current_joint_pos, arm_model_logs, dataset_logs, base_pose);

    size_t total_time_step = (total_time_sec * control_frequency) + 1;

    gtsam::Vector start_conf = Eigen::Map<const Eigen::VectorXd>(
        current_joint_pos.data(), current_joint_pos.size()) * M_PI / 180.0;

    // Get current end-effector pose
    gtsam::Pose3 current_ee_pose = forward_kinematics(const_cast<gtsam::Pose3&>(base_pose), current_joint_pos);
    Eigen::Vector3d current_ee_pos = current_ee_pose.translation();
    gtsam::Point3 z_axis_gtsam = current_ee_pose.rotation().r3();
    Eigen::Vector3d ee_z_axis(z_axis_gtsam.x(), z_axis_gtsam.y(), z_axis_gtsam.z());
    
    // Calculate shortest distance from current EE position to tube axis
    Eigen::Vector3d tube_direction = tube_info.direction.normalized();
    Eigen::Vector3d to_ee = current_ee_pos - tube_info.centroid;
    
    // Project vector to EE onto tube direction to find closest point on tube axis
    double projection_length = to_ee.dot(tube_direction);
    Eigen::Vector3d closest_point_on_tube = tube_info.centroid + projection_length * tube_direction;
    
    // Distance from EE to tube axis
    double distance_to_tube; 
    if(to_tube){
        distance_to_tube = (current_ee_pos - closest_point_on_tube).norm();
    }
    else{
        distance_to_tube = 0.2;
    }
    
    // Generate Gaussian velocity profile
    std::vector<double> velocities(total_time_step);
    double dt = 1.0/control_frequency;
    double sigma = total_time_sec / 6.0; // 6-sigma gives nice bell shape
    double mu = total_time_sec / 2.0;    // Peak at middle
    
    // Calculate Gaussian and normalize to achieve desired distance
    double total_distance = 0.0;
    for (size_t i = 0; i < total_time_step; ++i) {
        double t = i * dt;
        double gaussian = exp(-0.5 * pow((t - mu) / sigma, 2));
        velocities[i] = gaussian;
        total_distance += gaussian * dt;
    }
    
    // Scale velocities to achieve exact distance
    double scale_factor = distance_to_tube / total_distance;
    for (auto& vel : velocities) {
        vel *= scale_factor;
    }
    
    
    // Initialize trajectory
    trajectory.pos.clear();
    trajectory.vel.clear();

    Eigen::VectorXd joint_acc_deg(7);
    joint_acc_deg << 0,0,0,0,0,0,0;
    
    gtsam::Vector current_joint_conf = start_conf;
    
    for (size_t i = 0; i < total_time_step; ++i) {
        // Transform EE z-axis from world frame to base frame
        Eigen::Matrix3d R_base_inv = base_pose.rotation().matrix().transpose(); // Inverse rotation
        Eigen::Vector3d ee_z_axis_base_frame = R_base_inv * ee_z_axis;
        
        // Desired end-effector velocity along EE z-axis in base frame
        Eigen::Vector3d desired_ee_vel;
        if(to_tube){
            desired_ee_vel = - velocities[i] * ee_z_axis_base_frame;
        }
        else{
            desired_ee_vel = velocities[i] * ee_z_axis_base_frame;
        }
        
        // Convert to 6D twist (linear + angular velocity)
        Eigen::VectorXd desired_twist(6);
        desired_twist.head(3) = desired_ee_vel;
        desired_twist.tail(3) = Eigen::Vector3d::Zero(); // No angular velocity
        

        // Get Jacobian at current configuration
        MatrixXd jacobian = Jacobian::jaco_m(current_joint_conf);
        MatrixXd pinv_jacobian = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        
        
        // Calculate joint velocities
        gtsam::Vector joint_vel = pinv_jacobian * desired_twist;
        
        
        // Store joint velocity
        Eigen::VectorXd joint_vel_deg = joint_vel*(180.0/M_PI);

        trajectory.vel.push_back(joint_vel_deg);
        trajectory.acc.push_back(joint_acc_deg);
        
        // Integrate to get joint position
        if (i == 0) {
            Eigen::VectorXd joint_pos_deg = current_joint_conf*(180.0/M_PI);
            trajectory.pos.push_back(joint_pos_deg);
        } else {
            current_joint_conf += joint_vel * dt;
            Eigen::VectorXd joint_pos_deg = current_joint_conf*(180.0/M_PI);
            trajectory.pos.push_back(joint_pos_deg);
        }
        
    }
    
}












