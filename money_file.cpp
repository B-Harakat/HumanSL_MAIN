#include <iostream>
#include "Dynamics.h"
// Include headers from TrajectoryGeneration
#include "GenerateTrajectory.h"
#include "Planner.h"
#include "TrajectoryInitiation.h"
#include "TrajectoryOptimization.h"
#include "utils.h"

// Include headers from TrajectoryExecution
#include "KinovaTrajectory.h"
#include "Controller.h"
#include "Filter.h"
#include "Fwd_kinematics.h"
#include "Jacobian.h"

#include <iostream>
#include <memory>

#include "utils.h"
#include "Planner.h"
#include "Obstacles.h"
#include "Tube.h"

#include "GenerateArmModel.h"
#include "GenerateLogs.h"

#include "TrajectoryInitiation.h"
#include "TrajectoryOptimization.h"
// #include "RRTStarPlanner.h"


int main(int argc, char* argv[]) {

    int frame_number = 2; // default 
    size_t total_time_step = 10; // default
    double offset_from_human_y = 0.8; // default offset from human y
    double offset_from_tube_z = -0.2; // default offset from tube z
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        size_t pos = arg.find('=');
        if (pos != std::string::npos) {
            std::string key = arg.substr(0, pos);
            std::string value = arg.substr(pos + 1);
            
            if (key == "frame_number") { // ./main frame_number=2
                frame_number = std::stoi(value);
            }

            if (key == "total_time_step") { // ./main total_time_step=10
                total_time_step = std::stoi(value);
            }

            if (key == "offset_from_human_y") { // ./main offset_from_human_y=0.8
                offset_from_human_y = std::stod(value);
            }
            if (key == "offset_from_tube_z") { // ./main offset_from_tube_z=-0.2
                offset_from_tube_z = std::stod(value);
            }
            // Add more parameters here 
        }
    }
    std::cout << "=== Dual-Arm Trajectory Optimization Pipeline ===" << std::endl;
    
    // Configuration

    double total_time_sec = 3.5;

    double cell_size = 0.04; // 4 cm workspace grid resolution
    
    std::array<double, 3> workspace_size = {2.0, 3.5, 3.0};  // Realtive to MUVE platform origin, the workspace spans [-1.0, -1.5, 0.0] to [1.0, 2.0, 2.0]
    std::array<double, 3> workspace_origin = {-1.0, -1.5, 0.0}; // x is running mill width direction, y is running mill length direction, z is height

    // gtsam::Vector start_conf_left = (gtsam::Vector(7) << 
    //     4.7123889804, 1.5707963268, 0.0523598776, 0.1570796327,
    //     0.0523598776, 0.0523598776, 0.0523598776).finished();

    gtsam::Vector start_conf_left = gtsam::Vector::Zero(7);

    // gtsam::Vector start_conf_right = (gtsam::Vector(7) << 
    //     1.5707963268, 1.5707963268, 0.0523598776, 0.1570796327, 
    //     0.0523598776, 0.0523598776, 0.0523598776).finished();

    gtsam::Vector start_conf_right = gtsam::Vector::Zero(7);


    std::string c3d_file_path = "../config/left_01_03.c3d";
    std::string joint_limit_path = "../config/joint_limits.yaml";
    std::string dh_params_path = "../config/dh_params.yaml";
    Dynamics robot("../config/GEN3_With_GRIPPER_DYNAMICS.urdf");

    

    Obstacle obstacle;

    C3D_Dataset c3d_dataset = obstacle.createC3DDataset(c3d_file_path, "rossana", frame_number);
    
    TubeInfo tube_axis_info = extractTubeInfoFromC3D(c3d_file_path, frame_number);

    gtsam::Pose3 right_target_pose = createPoseFromTube(tube_axis_info, c3d_dataset.bounds.max_y, offset_from_human_y, offset_from_tube_z);
    
    gtsam::Pose3 unity_pose = gtsam::Pose3::Identity();

    auto [left_base_pose, right_base_pose] = createArmBasePoses(c3d_dataset.clav, c3d_dataset.strn);
    
    Planner plan(dh_params_path, joint_limit_path);
    // plan.updateArmModel(left_base_pose, right_base_pose);
    plan.updateArmModel(left_base_pose, unity_pose);

    DHParameters test_dh_params = createDHParams(dh_params_path);
    gtsam::Pose3 test_base_to_ee = forwardKinematics(
                               test_dh_params, 
                               start_conf_right, 
                               unity_pose);

    gtsam::Pose3 test_ee_to_base = inverseForwardKinematics(
                               test_dh_params, 
                               start_conf_right, 
                               test_base_to_ee);
    std::cout << "Inverse Forward Kinematics Result: " << test_ee_to_base << "\n";

    // Forward Kinematics Comparison Test
    std::cout << "\n=== Forward Kinematics Comparison (Zero Joint Angles) ===" << std::endl;
    
    // Test vector: all joint angles = 0
    gtsam::Vector zero_joints = gtsam::Vector::Zero(7);
    std::cout << "Input joint angles: [0, 0, 0, 0, 0, 0, 0] (radians)" << std::endl;
    
    // Method 1: Custom DH-based implementation from utils.cpp
    std::cout << "\n1. Custom DH Implementation (utils.cpp):" << std::endl;
    gtsam::Pose3 identity_base = gtsam::Pose3::Identity();
    gtsam::Pose3 result1 = forwardKinematics(test_dh_params, zero_joints, identity_base);
    gtsam::Point3 pos1 = result1.translation();
    gtsam::Rot3 rot1 = result1.rotation();
    std::cout << "   Position: [" << pos1.x() << ", " << pos1.y() << ", " << pos1.z() << "]" << std::endl;
    std::cout << "   Rotation matrix:" << std::endl;
    std::cout << rot1.matrix() << std::endl;
    
    // Method 2: TrajectoryExecution implementation
    std::cout << "\n2. TrajectoryExecution Implementation (Fwd_kinematics.cpp):" << std::endl;
    Eigen::VectorXd zero_joints_eigen = Eigen::VectorXd::Zero(7);
    auto [pose_6dof, transform_matrix] = Fwd_kinematics::forward(zero_joints_eigen);
    std::cout << "   6-DOF pose [x, y, z, roll, pitch, yaw]: [";
    for (int i = 0; i < 6; i++) {
        std::cout << pose_6dof(i);
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "   Position: [" << pose_6dof(0) << ", " << pose_6dof(1) << ", " << pose_6dof(2) << "]" << std::endl;
    std::cout << "   Rotation (RPY): [" << pose_6dof(3) << ", " << pose_6dof(4) << ", " << pose_6dof(5) << "]" << std::endl;
    
    // Method 3: GPMP2 arm model implementation
    std::cout << "\n3. GPMP2 Arm Model Implementation:" << std::endl;
    std::vector<gtsam::Pose3> joint_poses;
    plan.right_arm->fk_model().forwardKinematics(zero_joints, {}, joint_poses);
    if (!joint_poses.empty()) {
        gtsam::Pose3 result3 = joint_poses.back(); // End effector is last pose
        gtsam::Point3 pos3 = result3.translation();
        gtsam::Rot3 rot3 = result3.rotation();
        std::cout << "   Position: [" << pos3.x() << ", " << pos3.y() << ", " << pos3.z() << "]" << std::endl;
        std::cout << "   Rotation matrix:" << std::endl;
        std::cout << rot3.matrix() << std::endl;
    } else {
        std::cout << "   Error: No joint poses returned from GPMP2" << std::endl;
    }
    
    // Comparison analysis
    std::cout << "\n=== Comparison Analysis ===" << std::endl;
    if (!joint_poses.empty()) {
        gtsam::Point3 pos3 = joint_poses.back().translation();
        
        // Compare positions
        double diff_1_2 = (pos1 - gtsam::Point3(pose_6dof(0), pose_6dof(1), pose_6dof(2))).norm();
        double diff_1_3 = (pos1 - pos3).norm();
        double diff_2_3 = (gtsam::Point3(pose_6dof(0), pose_6dof(1), pose_6dof(2)) - pos3).norm();
        
        std::cout << "Position differences (Euclidean distance):" << std::endl;
        std::cout << "   Method 1 vs Method 2: " << diff_1_2 << " meters" << std::endl;
        std::cout << "   Method 1 vs Method 3: " << diff_1_3 << " meters" << std::endl;
        std::cout << "   Method 2 vs Method 3: " << diff_2_3 << " meters" << std::endl;
        
        // Check if differences are significant
        const double tolerance = 1e-6; // 1 micrometer
        bool methods_agree = (diff_1_2 < tolerance) && (diff_1_3 < tolerance) && (diff_2_3 < tolerance);
        std::cout << "All methods agree within " << tolerance << " meters: " << (methods_agree ? "YES" : "NO") << std::endl;
    }
    std::cout << "========================================================\n" << std::endl;

    std::cout << "Press Enter to continue";
    std::cin.get();
    
    auto start_sdf_time = std::chrono::high_resolution_clock::now();

    auto grid_with_man = obstacle.createHumanFromC3D(c3d_dataset.human_points, cell_size, workspace_size, workspace_origin);
    auto grid_with_arm_and_man = obstacle.updateGridWithOtherArm(*grid_with_man, *plan.left_arm, start_conf_left);
    auto grid_with_tube_and_arm_and_man = obstacle.createTubeFromC3D(*grid_with_arm_and_man, tube_axis_info);
   
    auto sdf = obstacle.createSDFFromOccupancyGrid(*grid_with_tube_and_arm_and_man);
    auto end_sdf_time = std::chrono::high_resolution_clock::now();
    auto sdf_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_sdf_time - start_sdf_time);


    auto start_optimization_time = std::chrono::high_resolution_clock::now();

    auto trajectory = plan.plan(
         *plan.right_arm, start_conf_right, right_target_pose, right_base_pose,
         total_time_sec, total_time_step,*sdf);

    auto end_optimization_time = std::chrono::high_resolution_clock::now();
    auto optimization_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_optimization_time - start_optimization_time);

    trajectory.create_sdf_duration = sdf_duration;
    trajectory.optimization_duration = optimization_duration;

    // analyzeTrajectoryResults(*plan.right_arm, trajectory, right_target_pose);
    // visualizeTrajectory(trajectory.trajectory_pos, *plan.right_arm, *grid_with_tube_and_arm_and_man, right_base_pose);
    saveTrajectoryResultToYAML(trajectory,"gpmp2");
    
    // Add RRT* codes here
    
    // std::cout << "\n=== RRT* Trajectory Planning ===" << std::endl;
    
    // // Create RRT* planner with same parameters as GPMP2
    // RRTStarPlanner rrt_planner("config/dh_params.yaml", "config/joint_limits.yaml");
    
    // // Configure RRT* parameters for comparison
    // RRTStarParams rrt_params;
    // rrt_params.step_size = 0.15;
    // rrt_params.goal_tolerance = 0.05;
    // rrt_params.rewire_radius = 0.25;  // Reduced from 0.4 (fewer nodes to check for rewiring)
    // rrt_params.max_iterations = 5000;
    // rrt_params.goal_bias = 0.15;
    // rrt_params.collision_check_resolution = 0.15;  // Increased from 0.05 (3x faster edge checking)
    // rrt_params.optimize_path = true;
    // rrt_planner.setParams(rrt_params);
    
    // auto start_rrt_optimization_time = std::chrono::high_resolution_clock::now();
    
    // // Plan using RRT* with same inputs as GPMP2
    // auto rrt_trajectory = rrt_planner.plan(
    //     *plan.right_arm, start_conf_right, right_target_pose, right_base_pose,
    //     total_time_sec, total_time_step, *sdf);
    
    // auto end_rrt_optimization_time = std::chrono::high_resolution_clock::now();
    // auto rrt_optimization_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     end_rrt_optimization_time - start_rrt_optimization_time);
    
    // rrt_trajectory.create_sdf_duration = sdf_duration; // Reuse same SDF creation time
    // rrt_trajectory.optimization_duration = rrt_optimization_duration;
    
    // // Save RRT* results
    // saveTrajectoryResultToYAML(rrt_trajectory, "rrt_star");
    // End add RRT* code here




    // exportSDFToYAML(*sdf);

    // auto[q,dq,ddq,dummy_func1] = convertTrajectory(trajectory, 0.001);
    
    // std::cout << "Press Enter to execute trajectory through Kinova API" << std::endl;
    // std::cin.get(); // Wait for user input before executing trajectory

    // std::cout << "Executing trajectory..." << std::endl;
    // executeTrajectory("192.168.1.10", q, dq, ddq, "joint_position");

    // auto sdf_no_pipe = obstacle.createSDFFromOccupancyGrid(*grid_with_arm_and_man);
    // gtsam::Pose3 right_target_pose_pipe = createPoseFromTube(tube_axis_info, c3d_dataset.bounds.max_y, offset_from_human_y, 0);

    // auto trajectory_to_pipe = plan.plan(
    //      *plan.right_arm, trajectory.trajectory_pos[trajectory.trajectory_pos.size()-1], right_target_pose_pipe, right_base_pose,
    //      total_time_sec, total_time_step,*sdf_no_pipe);

    // visualizeTrajectory(trajectory_to_pipe.trajectory_pos, *plan.right_arm, *grid_with_tube_and_arm_and_man, right_base_pose);

    // trajectory_deg = convertTrajectory(trajectory_to_pipe.trajectory_pos);
    
    // std::cout << "Press Enter to execute trajectory through Kinova API" << std::endl;
    // std::cin.get(); // Wait for user input before executing trajectory

    // std::cout << "Executing trajectory..." << std::endl;
    // runTrajectory("192.168.1.10", trajectory_deg);

    // setGripperPosition("192.168.10",0);

    return 0;
}
