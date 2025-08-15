//---------------------------------------------------------
// The main function for impedance controller
//---------------------------------------------------------
// Description: The main function for the impedance controller used in the PHRI project
// Copyright: Yihan Liu 2024
//---------------------------------------------------------

#define _USE_MATH_DEFINES


#include "plan.h"
#include "ViconInterface.h"
#include "Obstacles.h"
#include "GenerateLogs.h"
#include <thread>
#include <mutex>
#include <shared_mutex> 
#include "GenerateLogs.h"
#include "ViconVisualize.h"

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7
#define CONTROL_FREQUENCY 500


namespace k_api = Kinova::Api;
using namespace Jacobian;
using namespace Fwd_kinematics;
using namespace Controller;
using namespace Filter;


// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = chrono::seconds{20};

//------------------------------------------
// Function of high-level movement
//-----------------------------------------
// 2 inputs:
// base: communication variables
// q_d: desired joint angular position
//-----------------------------------------



//----------------------------------------------------
// Main function of impedance control
//----------------------------------------------------
int main(int argc, char **argv)
{   
    std::string ip_addr = IP_ADDRESS;
    std::string c3d_file_path = "../config/left_01_03.c3d";
    std::string joint_limit_path = "../config/joint_limits.yaml";
    std::string dh_params_path = "../config/dh_params.yaml";
    std::string robot_urdf_path = "../config/GEN3_With_GRIPPER_DYNAMICS.urdf";
    
    Dynamics robot(robot_urdf_path);
    TubeInfo tube_info;
    HumanInfo human_info;

    JointTrajectory joint_trajectory;

    Obstacle obstacle;

    std::vector<double> q_init_left(7); 
    q_init_left = {270,90,15,15,5,5,5};  // in deg
    std::vector<double> q_init_right(7);
    q_init_right= {90,90,15,15,5,5,5}; // in deg

    // Test forward kinematics seciton:

    gtsam::Vector start_conf = Eigen::Map<const Eigen::VectorXd>(
        q_init_right.data(), q_init_right.size()) * M_PI / 180.0;
    

    C3D_Dataset c3d_dataset = obstacle.createC3DDataset(c3d_file_path, "rossana", 251);
    auto [left_base_frame, right_base_frame] = createArmBasePoses(c3d_dataset.clav, c3d_dataset.strn);
    
    // Hardcoded base pose for right arm
    // gtsam::Matrix3 base_R;


    // base_R << -0.164464, 0.303844, -0.938419,
    //           -0.976664, 0.0830603, 0.198061,
    //           0.138125, 0.949094, 0.283093;
    // gtsam::Point3 base_t(-0.0342403,0.00315071,1.43671);
    // // gtsam::Pose3 right_base_frame(gtsam::Rot3(base_R), base_t);
    // gtsam::Pose3 right_base_frame = gtsam::Pose3(gtsam::Rot3(base_R), base_t);
    // gtsam::Pose3 left_base_frame = right_base_frame; // Keep left_base_frame for compatibility
    

    // gtsam::Pose3 identity_pose;

    // DHParameters dh_parameters = createDHParams(dh_params_path);

    // gtsam::Pose3 end_pose = forwardKinematics(dh_parameters, start_conf, right_base_frame);


    // std::cout << "End pose: " << end_pose <<"\n";

    std::cout << "\n Right base pose: " << right_base_frame << "\n\n";

    tube_info = obstacle.extractTubeInfoFromC3D(c3d_file_path, 251);
    human_info.human_points = c3d_dataset.human_points;
    human_info.bounds = c3d_dataset.bounds;

    Gen3Arm right_arm(ip_addr, robot_urdf_path, dh_params_path, joint_limit_path);

    auto start_sdf_time = std::chrono::high_resolution_clock::now();

    // right_arm.make_sdf(left_base_frame, q_init_left, tube_info, human_info, true);
    right_arm.make_sdf(tube_info, human_info, true, left_base_frame, q_init_left);
    
    auto end_sdf_time = std::chrono::high_resolution_clock::now();
    auto sdf_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_sdf_time - start_sdf_time);

    auto start_traj_time = std::chrono::high_resolution_clock::now();
    // Use new tube-aware trajectory planning with orientation variation
    
    right_arm.plan_joint(joint_trajectory, q_init_right, right_base_frame, 
                         tube_info, human_info, 0.4, 0.15, 3.0, 5, 500);

    
    auto end_traj_time = std::chrono::high_resolution_clock::now();
    auto traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_traj_time - start_traj_time);

    right_arm.result_logs.initiation_duration = sdf_duration;
    right_arm.result_logs.optimization_duration = traj_duration;

    saveTrajectoryResultToYAML(right_arm.result_logs,"test_generation");

    visualizeTrajectory(joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame);
    // visualizeStaticConfiguration(joint_trajectory.pos.front(), right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame);
    // std::vector<double> right_config(joint_trajectory.pos.back().begin(), joint_trajectory.pos.back().end());
    // right_arm.plan_joint(joint_trajectory, right_config, right_base_frame, 
    //                      tube_info, human_info, 0.4, 0.001, 3.0, 10);

    // visualizeTrajectory(joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame);
    
    // === REAL-TIME VISUALIZATION TEST ===
    std::cout << "Press Enter to start real-time visualization test" << std::endl;
    std::cin.get();


    
    // // Create shared variables for real-time visualization
    // std::vector<double> left_angles = q_init_left;
    // std::vector<double> right_angles = q_init_right;
    // std::shared_mutex joint_mutex;
    // std::shared_mutex vicon_mutex;
    
    // // Copy base frames to avoid capture issues
    // gtsam::Pose3 left_base_copy = left_base_frame;
    // gtsam::Pose3 right_base_copy = right_base_frame;
    
    // // Debug output
    // std::cout << "Starting visualization with:" << std::endl;
    // std::cout << "Left angles: ";
    // for (auto a : left_angles) std::cout << a << " ";
    // std::cout << std::endl;
    // std::cout << "Right angles: ";
    // for (auto a : right_angles) std::cout << a << " ";
    // std::cout << std::endl;
    // std::cout << "Tube points: " << tube_info.tube_points.size() << std::endl;
    // std::cout << "Human points: " << human_info.human_points.size() << std::endl;
    
    // // Start visualization thread
    // std::thread viz_thread([&]() {
    //     visualizeRealtimePCL(left_angles, right_angles, left_base_copy, right_base_copy,
    //                      dh_params_path, tube_info, human_info, 
    //                      joint_mutex, vicon_mutex);
    // });
    
    // // Animation thread to change joint angles
    // std::thread animation_thread([&]() {
    //     double time = 0.0;
    //     const double dt = 0.1; // 10Hz animation
        
    //     while (true) {
    //         {
    //             std::unique_lock<std::shared_mutex> lock(joint_mutex);
    //             // Animate first joint of both arms with sine waves
    //             right_angles[0] = q_init_right[0] + 20.0 * sin(time);        // ±20 degrees
    //             right_angles[1] = q_init_right[1] + 15.0 * sin(time * 1.5);  // ±15 degrees
    //             left_angles[0] = q_init_left[0] + 15.0 * sin(time * 0.8);    // ±15 degrees
    //             left_angles[2] = q_init_left[2] + 10.0 * cos(time);          // ±10 degrees
    //         }
            
    //         time += dt;
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // });
    
    // animation_thread.detach();
    // viz_thread.join(); // Wait for visualization window to close
    
    // std::cout << "Press Enter to execute trajectory through Kinova API" << std::endl;
    // std::cin.get(); // Wait for user input before executing trajectory
}