#include "plan.h"
#include <thread>
#include <mutex>
#include <shared_mutex> 
#include "GenerateLogs.h"
#include "Obstacles.h"
#include <atomic>
#include <yaml-cpp/yaml.h>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7
#define JOINT_CONTROL_FREQUENCY 500
#define TASK_CONTROL_FREQUENCY 500
#define GPMP2_TIMESTEPS 10

int main(){

    std::string joint_limit_path = "../config/joint_limits.yaml";
    std::string dh_params_path = "../config/dh_params.yaml";
    std::string robot_urdf_path = "../config/GEN3_With_GRIPPER_DYNAMICS.urdf";
    std::string c3d_file_path = "../config/left_01_03.c3d";
    std::string parameters_path = "../config/parameters.yaml";
    
    // IP address for right arm
    std::string right_ip_address = "192.168.1.10";

    // Create API objects
    auto error_callback = [](k_api::KError err){
        std::cout << "API Error: " << err.toString() << std::endl;
    };

    // RIGHT ARM - TCP connection for configuration
    auto right_transport = new k_api::TransportClientTcp();
    auto right_router = new k_api::RouterClient(right_transport, error_callback);
    right_transport->connect(right_ip_address, PORT);

    // RIGHT ARM - UDP connection for real-time control
    auto right_transport_real_time = new k_api::TransportClientUdp();
    auto right_router_real_time = new k_api::RouterClient(right_transport_real_time, error_callback);
    right_transport_real_time->connect(right_ip_address, PORT_REAL_TIME);

    // RIGHT ARM - UDP connection for joint monitoring
    auto right_transport_monitor = new k_api::TransportClientUdp();
    auto right_router_monitor = new k_api::RouterClient(right_transport_monitor, error_callback);
    right_transport_monitor->connect(right_ip_address, PORT_REAL_TIME);

    // Session setup
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);
    create_session_info.set_connection_inactivity_timeout(2000);

    // RIGHT ARM - Session managers
    auto right_session_manager = new k_api::SessionManager(right_router);
    right_session_manager->CreateSession(create_session_info);
    auto right_session_manager_real_time = new k_api::SessionManager(right_router_real_time);
    right_session_manager_real_time->CreateSession(create_session_info);

    // RIGHT ARM - Session manager for monitoring
    auto right_session_manager_monitor = new k_api::SessionManager(right_router_monitor);
    right_session_manager_monitor->CreateSession(create_session_info);

    // Create service clients for RIGHT ARM
    auto right_base = new k_api::Base::BaseClient(right_router);
    auto right_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(right_router_real_time);
    auto right_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(right_router);

    // Create monitoring service clients
    auto right_base_cyclic_monitor = new k_api::BaseCyclic::BaseCyclicClient(right_router_monitor);

    k_api::BaseCyclic::Feedback right_base_feedback;
    k_api::BaseCyclic::Command right_base_command;

    try {
        right_base->ClearFaults();
    } catch(...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    TubeInfo tube_info;  
    HumanInfo human_info;  

    TubeInfo tube_info_snapshot;  
    HumanInfo human_info_snapshot;  
    
    gtsam::Pose3 right_base_frame_snapshot = gtsam::Pose3(gtsam::Rot3::Rx(M_PI), gtsam::Point3(0,0,0));
    
    gtsam::Point3 left_T = gtsam::Point3(1.0,1.0,0.0);
    gtsam::Rot3 left_R = gtsam::Rot3::Rx(M_PI);
    gtsam::Pose3 left_base_frame_snapshot = gtsam::Pose3(left_R, left_T);

    std::shared_mutex joint_data_mutex;
    std::shared_mutex vicon_data_mutex;
    std::mutex base_frame_mutex;
    
    // Thread-safe replanning variables
    std::atomic<bool> replan_triggered{false};
    std::atomic<bool> new_trajectory_ready{false};
    std::atomic<int> replan_counter{0};
    std::mutex trajectory_mutex;  // For thread-safe trajectory replacement
    
    JointTrajectory joint_trajectory;
    JointTrajectory new_joint_trajectory;  // Buffer for new trajectory
    TaskTrajectory task_trajectory;
    TrajectoryRecord trajectory_record;

    Dynamics right_robot(robot_urdf_path);

    std::vector<double> q_cur_right(7);
    std::vector<double> q_cur_right_snapshot; 

    std::vector<double> q_init_right(7);
    q_init_right= {0,0,0,0,0,0,0}; // in deg

    Gen3Arm right_arm(right_ip_address, robot_urdf_path, dh_params_path, joint_limit_path);
    
    gtsam::Pose3 right_base_frame;
    double right_approach_time_sec = 2.0;

    move_single_level(right_base, q_init_right);
    
    std::vector<float> commands_right;

    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);

    right_base->SetServoingMode(servoing_mode);
    right_base_feedback = right_base_cyclic->RefreshFeedback();

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        commands_right.push_back(right_base_feedback.actuators(i).position());
        right_base_command.add_actuators()->set_position(right_base_feedback.actuators(i).position());
    }

    right_base_feedback = right_base_cyclic->Refresh(right_base_command);

    // Start recording current joint pos
    std::thread joint_info_thread([&](){
        while(true){
        updateJointInfo(right_base_cyclic_monitor, q_cur_right, joint_data_mutex);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } 
    });

    joint_info_thread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 

    // Set robot base frame (fixed since no Vicon) - rotated 180° about x-axis
    right_base_frame = gtsam::Pose3(
        gtsam::Rot3::Rx(M_PI),
        gtsam::Point3(0.0, 0.0, 0.0)
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    // Set actuators in position mode
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        right_actuator_config->SetControlMode(control_mode_message, id);
    }

    move_gripper(right_base_cyclic, 0);

    {   
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        q_cur_right_snapshot = shiftAngle(q_cur_right);
    }

    // Create simplified environment (no tube_info, human_info)
    std::vector<double> q_cur_left_snapshot(7, 0.0);  // Dummy

    std::vector<double> q_target_right;
    q_target_right = {0,60,0,0,0,-60,0};

    right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);

    right_arm.plan_joint(joint_trajectory, q_init_right, q_target_right, right_base_frame,  
                         right_approach_time_sec, GPMP2_TIMESTEPS, 500);

    visualizeTrajectory(joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame);

    std::cout << "Test trajectory planned, press Enter to continue to execution.";
    std::cin.get();
    
    std::atomic<bool> right_arm_flag_1{true};

    // std::thread check_replan_thread;
    // check_replan_thread = std::thread([&]() {
    //         gtsam::Pose3 target_pose = right_arm.target_pose_logs;
    //         right_arm.replan(
    //             joint_trajectory, new_joint_trajectory, right_base_frame, target_pose,
    //             vicon_data_mutex, joint_data_mutex, trajectory_mutex,
    //             replan_triggered, new_trajectory_ready, right_arm_flag_1,
    //             tube_info_snapshot, human_info_snapshot, q_cur_right,
    //             right_approach_offset_y, right_approach_offset_z, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
    //         );
    //     });
    

    // // Thread to translate right_base_frame by 0.3m in negative x-direction after 2 seconds
    // std::thread translate_frame_thread([&]() {
    //     std::this_thread::sleep_for(std::chrono::seconds(2));
    //     std::cout << "Translating base frame ... \n"; 
    //     std::lock_guard<std::mutex> lock(base_frame_mutex);
    //     gtsam::Point3 current_translation = right_base_frame.translation();
    //     gtsam::Point3 new_translation(current_translation.x() - 0.2, current_translation.y(), current_translation.z());
    //     right_base_frame = gtsam::Pose3(right_base_frame.rotation(), new_translation);
    // });
    // translate_frame_thread.detach();
    
    std::thread joint_execution_thread;
    joint_execution_thread = std::thread([&]() {
        joint_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, 
            right_base_command, right_robot, joint_trajectory, 
            right_base_frame, JOINT_CONTROL_FREQUENCY, 
            std::ref(right_arm_flag_1), std::ref(replan_counter), 
            std::ref(replan_triggered), std::ref(new_trajectory_ready), 
            std::ref(new_joint_trajectory), std::ref(trajectory_mutex), std::ref(trajectory_record));
    });
      
    joint_execution_thread.detach();
    // check_replan_thread.join();


    // ##### Move Gripper #####
    std::cout << "Moving Gripper \n";
    move_gripper(right_base_cyclic, 50);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); 






    // ############## Task EXECUTION ####################

    {   
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        q_cur_right_snapshot = shiftAngle(q_cur_right);
    }

    gtsam::Vector start_config = Eigen::Map<const Eigen::VectorXd>(
        q_cur_right_snapshot.data(), q_cur_right_snapshot.size()) * M_PI / 180.0;

    gtsam::Vector target_config = Eigen::Map<const Eigen::VectorXd>(
        q_init_right.data(), q_init_right.size()) * M_PI / 180.0;

    std::cout << " Start Joint: ";
    for (auto& elem : q_cur_right_snapshot) std::cout << elem << ", ";

    std::cout << "\n End Pose: ";
    for (auto& elem : q_init_right) std::cout << elem << ", ";
    
    std::cout << "\n";

    DHParameters dh = createDHParams(dh_params_path);

    gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot, q_cur_right_snapshot);
    gtsam::Pose3 target_pose = right_arm.forward_kinematics(right_base_frame_snapshot, q_init_right);

    std::cout << " Start Pose: " << start_pose <<"\n";

    std::cout << " End Pose: " << target_pose << "\n";

    right_arm.plan_task(new_joint_trajectory, start_pose, target_pose, right_base_frame_snapshot, q_cur_right_snapshot,
                         3.0, 5, 0.35, 0.23, 500);
    
    std::cout << " Start conf: ";
    for (auto& elem : new_joint_trajectory.pos[0]) std::cout << elem << ", ";

    std::cout << "\n End conf: ";
    for (auto& elem : new_joint_trajectory.pos[new_joint_trajectory.pos.size()-1]) std::cout << elem << ", ";
    

    visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);

    std::cout << "Test trajectory planned, press Enter to continue to execution.";
    std::cin.get();

    {
        std::lock_guard<std::mutex> lock(trajectory_mutex);
        joint_trajectory = std::move(new_joint_trajectory);
    }

    // ####### Chicken Head test #########
    
    std::cout << "Test trajectory planned, press Enter to continue to execution.";
    std::cin.get();

    Eigen::VectorXd p_d_world(6);
    gtsam::Vector3 pos = start_pose.translation();
    gtsam::Vector3 rpy = start_pose.rotation().rpy();
    p_d_world << pos.x(), pos.y(), pos.z(), rpy.x(), rpy.y(), rpy.z();

    // Thread to translate right_base_frame by 0.3m in negative x-direction after 2 seconds
    std::thread translate_frame_thread([&]() {

        for(int i =0; i < 10; i++){
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cout << "Translating base frame ... \n"; 
            std::lock_guard<std::shared_mutex> lock(vicon_data_mutex);
            gtsam::Point3 current_translation = right_base_frame.translation();
            gtsam::Point3 new_translation(current_translation.x() - 0.02, current_translation.y(), current_translation.z());
            right_base_frame = gtsam::Pose3(right_base_frame.rotation(), new_translation);
        }
    });
    translate_frame_thread.detach();

    right_arm_flag_1.store(false);

    std::atomic<bool> chicken_flag{true};

    std::thread chicken_thread;
    chicken_thread = std::thread([&]() {
        chicken_head_control_execution(right_base, right_base_cyclic, right_actuator_config, 
                                    right_base_feedback, right_base_command, right_robot, p_d_world,
                                right_base_frame, 500, dh_params_path, std::ref(vicon_data_mutex),
                            chicken_flag);
    });

    chicken_thread.detach();










    // My Code ends here
    
    // Cleanup - RIGHT ARM
    right_session_manager->CloseSession();
    right_session_manager_real_time->CloseSession();
    right_session_manager_monitor->CloseSession();

    right_router->SetActivationStatus(false);
    right_transport->disconnect();
    right_router_real_time->SetActivationStatus(false);
    right_transport_real_time->disconnect();
    right_router_monitor->SetActivationStatus(false);
    right_transport_monitor->disconnect();

    // Delete RIGHT ARM objects
    delete right_base;
    delete right_base_cyclic;
    delete right_base_cyclic_monitor;
    delete right_actuator_config;
    delete right_session_manager;
    delete right_session_manager_real_time;
    delete right_session_manager_monitor;
    delete right_router;
    delete right_router_real_time;
    delete right_router_monitor;
    delete right_transport;
    delete right_transport_real_time;
    delete right_transport_monitor;

    return 0;
}