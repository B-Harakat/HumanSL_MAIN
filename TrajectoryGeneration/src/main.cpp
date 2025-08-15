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
    
    std::array<double, 3> workspace_size = {2.0, 3.5, 2.0};  // Realtive to MUVE platform origin, the workspace spans [-1.0, -1.5, 0.0] to [1.0, 2.0, 2.0]
    std::array<double, 3> workspace_origin = {-1.0, -1.5, 0.0}; // x is running mill width direction, y is running mill length direction, z is height

    // gtsam::Vector start_conf_left = (gtsam::Vector(7) << 
    //     4.7123889804, 1.5707963268, 0.0523598776, 0.1570796327,
    //     0.0523598776, 0.0523598776, 0.0523598776).finished();

    gtsam::Vector start_conf_left = gtsam::Vector::Zero(7);

    // gtsam::Vector start_conf_right = (gtsam::Vector(7) << 
    //     1.5707963268, 1.5707963268, 0.0523598776, 0.1570796327, 
    //     0.0523598776, 0.0523598776, 0.0523598776).finished();

    gtsam::Vector start_conf_right = gtsam::Vector::Zero(7);


    std::string c3d_file_path = "/home/bjorn/Code/Kinova3/c3d/left_01_03.c3d";
    std::string joint_limit_path = "/home/bjorn/Code/gpmp2/config/joint_limits.yaml";
    std::string dh_params_path = "/home/bjorn/Code/gpmp2/config/dh_params.yaml";


    Obstacle obstacle;

    C3D_Dataset c3d_dataset = obstacle.createC3DDataset(c3d_file_path, "rossana", frame_number);
    
    TubeInfo tube_axis_info = extractTubeInfoFromC3D(c3d_file_path, frame_number);

    gtsam::Pose3 right_target_pose = createPoseFromTube(tube_axis_info, c3d_dataset.bounds.max_y, offset_from_human_y, offset_from_tube_z);

    auto [left_base_pose, right_base_pose] = createArmBasePoses(c3d_dataset.clav, c3d_dataset.strn);
    
    Planner plan(dh_params_path, joint_limit_path);
    plan.updateArmModel(left_base_pose, right_base_pose);
    
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

    analyzeTrajectoryResults(*plan.right_arm, trajectory, right_target_pose);
    visualizeTrajectory(trajectory.trajectory_pos, *plan.right_arm, *grid_with_tube_and_arm_and_man, right_base_pose);
    saveTrajectoryResultToYAML(trajectory);
    exportSDFToYAML(*sdf);

    // std::vector<std::vector<double>> trajectory_deg = convertTrajectory(trajectory.trajectory);
    
    // std::cout << "Press Enter to execute trajectory through Kinova API" << std::endl;
    // std::cin.get(); // Wait for user input before executing trajectory

    // std::cout << "Executing trajectory..." << std::endl;
    // runTrajectory("192.168.1.10", trajectory_deg);

    
    auto sdf_no_pipe = obstacle.createSDFFromOccupancyGrid(*grid_with_arm_and_man);
    gtsam::Pose3 right_target_pose_pipe = createPoseFromTube(tube_axis_info, c3d_dataset.bounds.max_y, offset_from_human_y, 0);

    auto trajectory_to_pipe = plan.plan(
         *plan.right_arm, trajectory.trajectory_pos[trajectory.trajectory_pos.size()-1], right_target_pose_pipe, right_base_pose,
         total_time_sec, total_time_step,*sdf_no_pipe);

    visualizeTrajectory(trajectory_to_pipe.trajectory_pos, *plan.right_arm, *grid_with_tube_and_arm_and_man, right_base_pose);

    // trajectory_deg = convertTrajectory(trajectory_to_pipe.trajectory);
    
    // std::cout << "Press Enter to execute trajectory through Kinova API" << std::endl;
    // std::cin.get(); // Wait for user input before executing trajectory

    // std::cout << "Executing trajectory..." << std::endl;
    // runTrajectory("192.168.1.10", trajectory_deg);

    // setGripperPosition("192.168.10",0);

    return 0;
}
