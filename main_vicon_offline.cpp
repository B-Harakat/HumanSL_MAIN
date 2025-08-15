#include "plan.h"
#include "ViconInterface.h"
#include "ViconInfo.h"
#include <thread>
#include <mutex>
#include <shared_mutex> 
#include "GenerateLogs.h"
#include <atomic>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7
#define CONTROL_FREQUENCY 500

int main(){

    std::string joint_limit_path = "../config/joint_limits.yaml";
    std::string dh_params_path = "../config/dh_params.yaml";
    std::string robot_urdf_path = "../config/GEN3_With_GRIPPER_DYNAMICS.urdf";
    
    
    // IP addresses for each arm
    // std::string left_ip_address = "192.168.1.9";
    std::string right_ip_address = "192.168.1.10";
   

    // Create API objects
    auto error_callback = [](k_api::KError err){
        std::cout << "API Error: " << err.toString() << std::endl;
    };
   
    // // LEFT ARM - TCP connection for configuration
    // auto left_transport = new k_api::TransportClientTcp();
    // auto left_router = new k_api::RouterClient(left_transport, error_callback);
    // left_transport->connect(left_ip_address, PORT);
   
    // // LEFT ARM - UDP connection for real-time control
    // auto left_transport_real_time = new k_api::TransportClientUdp();
    // auto left_router_real_time = new k_api::RouterClient(left_transport_real_time, error_callback);
    // left_transport_real_time->connect(left_ip_address, PORT_REAL_TIME);
   
    // RIGHT ARM - TCP connection for configuration
    auto right_transport = new k_api::TransportClientTcp();
    auto right_router = new k_api::RouterClient(right_transport, error_callback);
    right_transport->connect(right_ip_address, PORT);
   
    // RIGHT ARM - UDP connection for real-time control
    auto right_transport_real_time = new k_api::TransportClientUdp();
    auto right_router_real_time = new k_api::RouterClient(right_transport_real_time, error_callback);
    right_transport_real_time->connect(right_ip_address, PORT_REAL_TIME);
   
    // // LEFT ARM - UDP connection for joint monitoring
    // auto left_transport_monitor = new k_api::TransportClientUdp();
    // auto left_router_monitor = new k_api::RouterClient(left_transport_monitor, error_callback);
    // left_transport_monitor->connect(left_ip_address, PORT_REAL_TIME);
   
    // RIGHT ARM - UDP connection for joint monitoring
    auto right_transport_monitor = new k_api::TransportClientUdp();
    auto right_router_monitor = new k_api::RouterClient(right_transport_monitor, error_callback);
    right_transport_monitor->connect(right_ip_address, PORT_REAL_TIME);
   
    // Session setup for both arms
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);
    create_session_info.set_connection_inactivity_timeout(2000);
   
    // // LEFT ARM - Session managers
    // auto left_session_manager = new k_api::SessionManager(left_router);
    // left_session_manager->CreateSession(create_session_info);
    // auto left_session_manager_real_time = new k_api::SessionManager(left_router_real_time);
    // left_session_manager_real_time->CreateSession(create_session_info);
   
    // RIGHT ARM - Session managers
    auto right_session_manager = new k_api::SessionManager(right_router);
    right_session_manager->CreateSession(create_session_info);
    auto right_session_manager_real_time = new k_api::SessionManager(right_router_real_time);
    right_session_manager_real_time->CreateSession(create_session_info);
   
    // // LEFT ARM - Session manager for monitoring
    // auto left_session_manager_monitor = new k_api::SessionManager(left_router_monitor);
    // left_session_manager_monitor->CreateSession(create_session_info);
   
    // RIGHT ARM - Session manager for monitoring
    auto right_session_manager_monitor = new k_api::SessionManager(right_router_monitor);
    right_session_manager_monitor->CreateSession(create_session_info);
 
    // // Create service clients for LEFT ARM
    // auto left_base = new k_api::Base::BaseClient(left_router);
    // auto left_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(left_router_real_time);
    // auto left_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(left_router);
   
    // Create service clients for RIGHT ARM
    auto right_base = new k_api::Base::BaseClient(right_router);
    auto right_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(right_router_real_time);
    auto right_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(right_router);
   
    // Create monitoring service clients
    // auto left_base_cyclic_monitor = new k_api::BaseCyclic::BaseCyclicClient(left_router_monitor);
    auto right_base_cyclic_monitor = new k_api::BaseCyclic::BaseCyclicClient(right_router_monitor);

    // k_api::BaseCyclic::Feedback left_base_feedback;
    // k_api::BaseCyclic::Command left_base_command;

    k_api::BaseCyclic::Feedback right_base_feedback;
    k_api::BaseCyclic::Command right_base_command;
   
    try {
        // left_base->ClearFaults();
        right_base->ClearFaults();
    } catch(...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    // My code starts here

    std::shared_mutex vicon_data_mutex;
    std::shared_mutex joint_data_mutex;
    
    // Thread-safe replanning variables
    std::atomic<bool> replan_triggered{false};
    std::atomic<bool> new_trajectory_ready{false};
    std::atomic<int> replan_counter{0};
    std::mutex trajectory_mutex;  // For thread-safe trajectory replacement
    
    JointTrajectory joint_trajectory;
    JointTrajectory new_joint_trajectory;  // Buffer for new trajectory
    TaskTrajectory task_trajectory;

    gtsam::Pose3 left_base_frame;
    gtsam::Pose3 right_base_frame; 

    // Dynamics left_robot(robot_urdf_path);
    Dynamics right_robot(robot_urdf_path);

    TubeInfo tube_info;
    HumanInfo human_info;

    std::vector<double> q_cur_right(7);
    std::vector<double> q_cur_left(7);
    q_cur_left = {0,0,0,0,0,0,0};  // in deg

    gtsam::Pose3 left_base_frame_snapshot; 
    gtsam::Pose3 right_base_frame_snapshot;
    TubeInfo tube_info_snapshot;
    HumanInfo human_info_snapshot;
    std::vector<double> q_cur_right_snapshot; 
    std::vector<double> q_cur_left_snapshot;


    std::vector<double> q_init_left(7); 
    q_init_left = {270,90,15,15,5,5,5};  // in deg
    std::vector<double> q_init_right(7);
    q_init_right= {90,90,15,15,5,5,5}; // in deg


    // Gen3Arm left_arm(left_ip_address, robot_urdf_path, dh_params_path, joint_limit_path);
    Gen3Arm right_arm(right_ip_address, robot_urdf_path, dh_params_path, joint_limit_path);

    // move_single_level(left_base, q_init_left);
    move_single_level(right_base, q_init_right);
    
    // std::vector<float> commands_left;
    std::vector<float> commands_right;

    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    
    // left_base->SetServoingMode(servoing_mode);
    // left_base_feedback = left_base_cyclic->RefreshFeedback();

    // // Initialize each actuator to their current position
    // for (int i = 0; i < ACTUATOR_COUNT; i++)
    // {
    //     commands_left.push_back(left_base_feedback.actuators(i).position());
    //     left_base_command.add_actuators()->set_position(left_base_feedback.actuators(i).position());
    // }
    // // Send a first frame
    // left_base_feedback = left_base_cyclic->Refresh(left_base_command);

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
        updateJointInfo(right_base_cyclic_monitor, q_cur_right, joint_data_mutex);
    });

    joint_info_thread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 


    // Vicon interface set up
    ViconInterface vicon;

    if (!vicon.connect("192.168.128.206")) {
        std::cerr << "Failed to connect to Vicon system. Exiting." << std::endl;
        return -1;
    }

    std::thread vicon_info_thread([&]() {
            updateViconInfo(vicon, left_base_frame, right_base_frame, tube_info, human_info, q_cur_left, q_cur_right, dh_params_path, vicon_data_mutex, joint_data_mutex);
        });

    vicon_info_thread.detach();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    // Set actuators in position mode
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        // left_actuator_config->SetControlMode(control_mode_message, id);
        right_actuator_config->SetControlMode(control_mode_message, id);
    }

    std::atomic<bool> right_arm_flag_1 = true;
    std::atomic<bool> right_arm_flag_2 = false;
    std::atomic<bool> right_arm_flag_3 = false;

    {
        std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
        auto left_R = right_base_frame.rotation();
        auto left_T = right_base_frame.translation();
        gtsam::Point3 left_T_snap = gtsam::Point3(left_T.x() - 0.4, left_T.y(), left_T.z());
        left_base_frame_snapshot = gtsam::Pose3(left_R, left_T_snap);
        right_base_frame_snapshot = right_base_frame; 
        tube_info_snapshot = tube_info;
        human_info_snapshot = human_info;
    }

    {   
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        q_cur_left_snapshot = q_cur_left;
        q_cur_right_snapshot = q_cur_right;

    }

    // right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left);
    right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
    std::cout << right_base_frame_snapshot; 
    right_arm.plan_joint(joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                         tube_info_snapshot, human_info_snapshot, 0.6, 0.15, 3.0, 10);

    visualizeTrajectory(joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);

    std::cout << "Pipe approach planned, press Enter to conitnue to execution.";
    std::cin.get();
    
    
    // Launch replanning monitoring thread
    std::thread check_replan_thread;
    std::thread right_approach_pipe_thread;
    std::cout << "JointPos Size: " << joint_trajectory.pos.size() << "\n";
    try {
        check_replan_thread = std::thread([&]() {
            gtsam::Pose3 target_pose = right_arm.target_pose_logs;
            right_arm.replan(
                joint_trajectory, new_joint_trajectory, right_base_frame_snapshot, target_pose,
                vicon_data_mutex, joint_data_mutex, trajectory_mutex,
                replan_triggered, new_trajectory_ready, right_arm_flag_1,
                tube_info_snapshot, human_info_snapshot, q_cur_right,
                0.6, 0.15, 2.0, 10
            );
        });

        // Launch joint control execution thread with replanning support
        right_approach_pipe_thread = std::thread([&]() {
            try {
                joint_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, right_base_command, right_robot, joint_trajectory, right_base_frame, CONTROL_FREQUENCY, std::ref(right_arm_flag_1), std::ref(replan_counter), std::ref(replan_triggered), std::ref(new_trajectory_ready), std::ref(new_joint_trajectory), std::ref(trajectory_mutex));
                // joint_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, right_base_command, right_robot, joint_trajectory, right_base_frame, CONTROL_FREQUENCY, std::ref(right_arm_flag_1));
            } catch (const std::exception& e) {
                std::cerr << "Error in joint_control_execution: " << e.what() << std::endl;
            }
        });
        
        // Wait for joint control to complete
        
        right_approach_pipe_thread.join();
 
        
        // Stop replanning thread
        if (check_replan_thread.joinable()) {
            check_replan_thread.join();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error launching threads: " << e.what() << std::endl;
        
        // Cleanup threads if they were started
        if (check_replan_thread.joinable()) {
            right_arm_flag_1.store(false);  // Signal threads to stop
            check_replan_thread.join();
        }
        if (right_approach_pipe_thread.joinable()) {
            right_approach_pipe_thread.join();
        }
    }

    while(!right_arm_flag_1);
    
    if(right_arm_flag_1 == false){

        {
            std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
            left_base_frame_snapshot = left_base_frame;
            right_base_frame_snapshot = right_base_frame; 
            tube_info_snapshot = tube_info;
            human_info_snapshot = human_info;
        }

        {   std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
            q_cur_left_snapshot = q_cur_left;
            q_cur_right_snapshot = q_cur_right;
        }

        // right_arm.make_sdf(left_base_frame_snapshot, q_cur_left_snapshot, tube_info_snapshot, human_info_snapshot, false);
        right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
        right_arm.plan_joint(joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                             tube_info_snapshot, human_info_snapshot, 0.4, 0.005, 2.0, 10);
        
        right_arm_flag_2 = true;

        joint_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, right_base_command, right_robot, joint_trajectory, right_base_frame, CONTROL_FREQUENCY, std::ref(right_arm_flag_2));
        move_gripper(right_base_cyclic, right_base_feedback, right_base_command, 60);
    }

    if(right_arm_flag_2 == false){

        {
            std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
            left_base_frame_snapshot = left_base_frame;
            right_base_frame_snapshot = right_base_frame; 
            tube_info_snapshot = tube_info;
            human_info_snapshot = human_info;
        }

        {
            std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
            q_cur_left_snapshot = q_cur_left;
            q_cur_right_snapshot = q_cur_right;
        }

        auto start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
        auto target_pose_to_above_head = right_arm.create_target_pose(human_info_snapshot, start_pose, 0.1);
        right_arm.plan_task(task_trajectory, start_pose, target_pose_to_above_head, 4.0, 0.35, 0.23);

        right_arm_flag_3 = true;
    }

    // control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    // for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    // {
    //     right_actuator_config->SetControlMode(control_mode_message, id);
    // }

    // std::thread right_reach_above_head_thread([&]() {
    //         task_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, right_base_command, right_robot, task_trajectory, right_base_frame, CONTROL_FREQUENCY, std::ref(right_arm_flag_3));
    //     });
    // right_reach_above_head_thread.join();


    

    // Insert execution codes here















    // End Execution code
    


    // Instruction for Claude: ignore this part
    // instead of grabbing future 200, grab the singular 200th end point as start.
    // INSTEAD OF A BUFFER, USE atomic bool flag to as a check inbetween timing controls, reset counter, and use new trajectory
    // Target. 1st option admittance
    // 2nd option, compensation by calculating human frame change = (prioritize)->backpack frmae change
    // End ignore






    // My Code ends here
    // Cleanup - LEFT ARM
    // left_session_manager->CloseSession();
    // left_session_manager_real_time->CloseSession();
    // left_session_manager_monitor->CloseSession();
   
    // left_router->SetActivationStatus(false);
    // left_transport->disconnect();
    // left_router_real_time->SetActivationStatus(false);
    // left_transport_real_time->disconnect();
    // left_router_monitor->SetActivationStatus(false);
    // left_transport_monitor->disconnect();
   
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
   
    // Delete LEFT ARM objects
    // delete left_base;
    // delete left_base_cyclic;
    // delete left_base_cyclic_monitor;
    // delete left_actuator_config;
    // delete left_session_manager;
    // delete left_session_manager_real_time;
    // delete left_session_manager_monitor;
    // delete left_router;
    // delete left_router_real_time;
    // delete left_router_monitor;
    // delete left_transport;
    // delete left_transport_real_time;
    // delete left_transport_monitor;
   
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