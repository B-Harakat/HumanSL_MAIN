#include "plan.h"
#include "ViconInterface.h"
#include "ViconInfo.h"
#include "ViconTrigger.h"
#include <thread>
#include <mutex>
#include <shared_mutex> 
#include "GenerateLogs.h"
#include <atomic>
#include <signal.h>
#include <cstdlib>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7
#define JOINT_CONTROL_FREQUENCY 1000
#define TASK_CONTROL_FREQUENCY 300
#define GPMP2_TIMESTEPS 50

// Global variables for cleanup
k_api::Base::BaseClient* g_right_base = nullptr;
k_api::BaseCyclic::BaseCyclicClient* g_right_base_cyclic = nullptr;
k_api::Base::BaseClient* g_left_base = nullptr;
k_api::BaseCyclic::BaseCyclicClient* g_left_base_cyclic = nullptr;

void cleanup_and_exit() {
    std::cout << "\nPerforming emergency shutdown..." << std::endl;
    
    // Open gripper
    if (g_right_base_cyclic) {
        try {
            move_gripper(g_right_base_cyclic, 0);
            std::cout << "Gripper opened" << std::endl;
        } catch (...) {
            std::cout << "Failed to open gripper" << std::endl;
        }
    }
    
    // Reset control mode to single level
    if (g_right_base) {
        try {
            auto servoing_mode_single = k_api::Base::ServoingModeInformation();
            servoing_mode_single.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            g_right_base->SetServoingMode(servoing_mode_single);
            std::cout << "Control mode reset to single level" << std::endl;
        } catch (...) {
            std::cout << "Failed to reset control mode" << std::endl;
        }
    }

    if (g_left_base_cyclic) {
        try {
            move_gripper(g_left_base_cyclic, 0);
            std::cout << "Gripper opened" << std::endl;
        } catch (...) {
            std::cout << "Failed to open gripper" << std::endl;
        }
    }
    
    // Reset control mode to single level
    if (g_left_base) {
        try {
            auto servoing_mode_single = k_api::Base::ServoingModeInformation();
            servoing_mode_single.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
            g_left_base->SetServoingMode(servoing_mode_single);
            std::cout << "Control mode reset to single level" << std::endl;
        } catch (...) {
            std::cout << "Failed to reset control mode" << std::endl;
        }
    }
    
    std::cout << "Emergency shutdown complete" << std::endl;
}

void signal_handler(int signal) {
    std::cout << "\nSignal " << signal << " received" << std::endl;
    cleanup_and_exit();
    exit(signal);
}

int main(){

    // Register signal handlers for cleanup
    signal(SIGINT, signal_handler);   // Ctrl+C
    signal(SIGTERM, signal_handler);  // Termination signal
    signal(SIGABRT, signal_handler);  // Abort signal
    std::atexit(cleanup_and_exit);    // Normal exit cleanup

    std::string joint_limit_path = "../config/joint_limits.yaml";
    std::string dh_params_path = "../config/dh_params.yaml";
    std::string robot_urdf_path = "../config/GEN3_With_GRIPPER_DYNAMICS.urdf";
    std::string parameters_path = "../config/parameters.yaml";

    
    // IP addresses for each arm
    std::string left_ip_address = "192.168.1.9";
    std::string right_ip_address = "192.168.1.10";
   

    // Create API objects
    auto error_callback = [](k_api::KError err){
        std::cout << "API Error: " << err.toString() << std::endl;
    };
   
    // LEFT ARM - TCP connection for configuration
    auto left_transport = new k_api::TransportClientTcp();
    auto left_router = new k_api::RouterClient(left_transport, error_callback);
    left_transport->connect(left_ip_address, PORT);
   
    // LEFT ARM - UDP connection for real-time control
    auto left_transport_real_time = new k_api::TransportClientUdp();
    auto left_router_real_time = new k_api::RouterClient(left_transport_real_time, error_callback);
    left_transport_real_time->connect(left_ip_address, PORT_REAL_TIME);
   
    // RIGHT ARM - TCP connection for configuration
    auto right_transport = new k_api::TransportClientTcp();
    auto right_router = new k_api::RouterClient(right_transport, error_callback);
    right_transport->connect(right_ip_address, PORT);
   
    // RIGHT ARM - UDP connection for real-time control
    auto right_transport_real_time = new k_api::TransportClientUdp();
    auto right_router_real_time = new k_api::RouterClient(right_transport_real_time, error_callback);
    right_transport_real_time->connect(right_ip_address, PORT_REAL_TIME);
   
    // LEFT ARM - UDP connection for joint monitoring
    auto left_transport_monitor = new k_api::TransportClientUdp();
    auto left_router_monitor = new k_api::RouterClient(left_transport_monitor, error_callback);
    left_transport_monitor->connect(left_ip_address, PORT_REAL_TIME);
   
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
   
    // LEFT ARM - Session managers
    auto left_session_manager = new k_api::SessionManager(left_router);
    left_session_manager->CreateSession(create_session_info);
    auto left_session_manager_real_time = new k_api::SessionManager(left_router_real_time);
    left_session_manager_real_time->CreateSession(create_session_info);
   
    // RIGHT ARM - Session managers
    auto right_session_manager = new k_api::SessionManager(right_router);
    right_session_manager->CreateSession(create_session_info);
    auto right_session_manager_real_time = new k_api::SessionManager(right_router_real_time);
    right_session_manager_real_time->CreateSession(create_session_info);
   
    // LEFT ARM - Session manager for monitoring
    auto left_session_manager_monitor = new k_api::SessionManager(left_router_monitor);
    left_session_manager_monitor->CreateSession(create_session_info);
   
    // RIGHT ARM - Session manager for monitoring
    auto right_session_manager_monitor = new k_api::SessionManager(right_router_monitor);
    right_session_manager_monitor->CreateSession(create_session_info);
 
    // Create service clients for LEFT ARM
    auto left_base = new k_api::Base::BaseClient(left_router);
    auto left_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(left_router_real_time);
    auto left_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(left_router);
   
    // Create service clients for RIGHT ARM
    auto right_base = new k_api::Base::BaseClient(right_router);
    auto right_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(right_router_real_time);
    auto right_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(right_router);
    
    // Set global pointers for cleanup functions
    g_right_base = right_base;
    g_right_base_cyclic = right_base_cyclic;

    g_left_base = left_base;
    g_left_base_cyclic = left_base_cyclic;
   
    // Create monitoring service clients
    auto left_base_cyclic_monitor = new k_api::BaseCyclic::BaseCyclicClient(left_router_monitor);
    auto right_base_cyclic_monitor = new k_api::BaseCyclic::BaseCyclicClient(right_router_monitor);

    k_api::BaseCyclic::Feedback left_base_feedback;
    k_api::BaseCyclic::Command left_base_command;

    k_api::BaseCyclic::Feedback right_base_feedback;
    k_api::BaseCyclic::Command right_base_command;
   
    try {
        left_base->ClearFaults();
        right_base->ClearFaults();
    } catch(...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        cleanup_and_exit();
        return false;
    }





    // My code starts here
    //
    //
    // My code starts here


    YAML::Node config = YAML::LoadFile(parameters_path);
    auto traj_params = config["trajectory_parameters"];
    
    // Right approach phase parameters
    double right_approach_offset_y = traj_params["right_approach_phase"]["offset_from_human_y"].as<double>();
    double right_approach_offset_z = traj_params["right_approach_phase"]["offset_from_tube_z"].as<double>();
    double right_approach_time_sec = traj_params["right_approach_phase"]["time_sec"].as<double>();

    // Right grasp phase parameters
    double right_grasp_offset_y = traj_params["right_grasp_phase"]["offset_from_human_y"].as<double>();
    double right_grasp_offset_z = traj_params["right_grasp_phase"]["offset_from_tube_z"].as<double>();
    double right_grasp_time_sec = traj_params["right_grasp_phase"]["time_sec"].as<double>();
    
    // Right shoulder phase parameters
    double right_shoulder_time_sec = traj_params["right_shoulder_phase"]["time_sec"].as<double>();
    double right_shoulder_percentage = traj_params["right_shoulder_phase"]["percentage"].as<double>();
    double right_shoulder_height = traj_params["right_shoulder_phase"]["height"].as<double>();

    double left_approach_offset_y = traj_params["left_approach_phase"]["offset_from_human_y"].as<double>();
    double left_approach_offset_z = traj_params["left_approach_phase"]["offset_from_tube_z"].as<double>();
    double left_approach_time_sec = traj_params["left_approach_phase"]["time_sec"].as<double>();

    // Right grasp phase parameters
    double left_grasp_offset_y = traj_params["left_grasp_phase"]["offset_from_human_y"].as<double>();
    double left_grasp_offset_z = traj_params["left_grasp_phase"]["offset_from_tube_z"].as<double>();
    double left_grasp_time_sec = traj_params["left_grasp_phase"]["time_sec"].as<double>();
    
    // Right disengage phase parameters
    double right_disengage_offset_y = traj_params["right_disengage_phase_1"]["offset_from_human_y"].as<double>();
    double right_disengage_offset_z = traj_params["right_disengage_phase_1"]["offset_from_tube_z"].as<double>();
    double right_disengage_time_sec = traj_params["right_disengage_phase_1"]["time_sec"].as<double>();

    std::shared_mutex vicon_data_mutex;
    std::shared_mutex joint_data_mutex;
    
    // Thread-safe replanning variables
    std::atomic<bool> replan_triggered{false};
    std::atomic<bool> new_trajectory_ready{false};
    std::atomic<int> replan_counter{0};
    std::mutex trajectory_mutex;  // For thread-safe trajectory replacement
    
    // Thread-safe phase trigger
    std::atomic<int> phase_idx{0};

    JointTrajectory joint_trajectory;
    JointTrajectory new_joint_trajectory;  // Buffer for new trajectory
    TaskTrajectory task_trajectory;

    gtsam::Pose3 left_base_frame;
    gtsam::Pose3 right_base_frame; 

    Dynamics left_robot(robot_urdf_path);
    Dynamics right_robot(robot_urdf_path);

    TubeInfo tube_info;
    HumanInfo human_info;

    std::vector<double> q_cur_right(7);
    std::vector<double> q_cur_left(7);
    

    gtsam::Pose3 left_base_frame_snapshot; 
    gtsam::Pose3 right_base_frame_snapshot;
    TubeInfo tube_info_snapshot;
    HumanInfo human_info_snapshot;
    std::vector<double> q_cur_right_snapshot; 
    std::vector<double> q_cur_left_snapshot;


    std::vector<double> q_init_left(7); 
    q_init_left = {-90,90,-15,45,5,5,-175};  // in deg
    std::vector<double> q_init_right(7);
    q_init_right= {90,90,15,45,5,5,5}; // in deg


    Gen3Arm left_arm(left_ip_address, robot_urdf_path, dh_params_path, joint_limit_path);
    Gen3Arm right_arm(right_ip_address, robot_urdf_path, dh_params_path, joint_limit_path);


    TrajectoryRecord right_grasp_phase_record;


    move_single_level(left_base, q_init_left);
    move_single_level(right_base, q_init_right);
    
    std::vector<float> commands_left;
    std::vector<float> commands_right;

    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    
    left_base->SetServoingMode(servoing_mode);
    left_base_feedback = left_base_cyclic->RefreshFeedback();

    // Initialize each actuator to their current position
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        commands_left.push_back(left_base_feedback.actuators(i).position());
        left_base_command.add_actuators()->set_position(left_base_feedback.actuators(i).position());
    }
    // Send a first frame
    left_base_feedback = left_base_cyclic->Refresh(left_base_command);

    right_base->SetServoingMode(servoing_mode);
    right_base_feedback = right_base_cyclic->RefreshFeedback();

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        commands_right.push_back(right_base_feedback.actuators(i).position());
        right_base_command.add_actuators()->set_position(right_base_feedback.actuators(i).position());
    }

    right_base_feedback = right_base_cyclic->Refresh(right_base_command);

    // Vicon interface set up
    ViconInterface vicon;

    if (!vicon.connect("192.168.128.206")) {
        std::cerr << "Failed to connect to Vicon system. Exiting." << std::endl;
        return -1;
    }

    std::thread info_thread([&]() {

        thread_local int prev_frame_number = -1;

        while (true) {
            
            int cur_frame_number = vicon.getFrameNumber();
            // std::cout << "Vicon frame number: " << cur_frame_number <<"\n";

            if (cur_frame_number == prev_frame_number){
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            else{
                prev_frame_number = cur_frame_number;
            }

            if (!vicon.getFrame()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            updateViconInfo(vicon, left_base_frame, right_base_frame, tube_info, human_info, q_cur_left, q_cur_right, dh_params_path, vicon_data_mutex, joint_data_mutex);
            updateJointInfo(right_base_cyclic_monitor, q_cur_right, joint_data_mutex);
            updateJointInfo(left_base_cyclic_monitor, q_cur_left, joint_data_mutex);

            triggerRightGrasp(human_info, tube_info, std::ref(phase_idx));
        }
    });

    info_thread.detach();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

    // Set actuators in position mode
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        left_actuator_config->SetControlMode(control_mode_message, id);
        right_actuator_config->SetControlMode(control_mode_message, id);
    }

    move_gripper(right_base_cyclic, 0);
    move_gripper(left_base_cyclic,  0);

    std::atomic<bool> right_arm_flag_1 = true;
    std::atomic<bool> right_arm_flag_2 = false;
    std::atomic<bool> right_arm_flag_3 = false;

    // while(!(phase_idx.load()==1)); // replace with trigger condition
    while(!right_arm_flag_1.load());
    
    {
        std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
        left_base_frame_snapshot = left_base_frame;
        right_base_frame_snapshot = right_base_frame; 
        tube_info_snapshot = tube_info;
        human_info_snapshot = human_info;
    }

    {   
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        q_cur_left_snapshot = shiftAngle(q_cur_left);
        q_cur_right_snapshot = shiftAngle(q_cur_right);
    }

    std::cout << "Left angle snap shot: ";
    for(auto& k : q_cur_left_snapshot){std::cout << k << ", ";}
    std::cout << "\n";

    std::cout << "Right angle snap shot: ";
    for(auto& k : q_cur_right_snapshot){std::cout << k << ", ";}
    std::cout << "\n";

    std::cout << "Right Base pose: " << right_base_frame_snapshot << "\n";

    // TubeInfo tube_info_dummy;
    // HumanInfo human_info_dummy;
    right_arm.make_sdf(tube_info, human_info, true, left_base_frame_snapshot, q_cur_left_snapshot);
    // std::cout << right_base_frame_snapshot; 
    right_arm.plan_joint(joint_trajectory, q_init_right, right_base_frame_snapshot, 
                         tube_info_snapshot, human_info_snapshot, 
                         right_approach_offset_y, right_approach_offset_z, 
                         right_approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);

    visualizeTrajectory(joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
    saveTrajectoryResultToYAML(right_arm.result_logs,"gpmp2");
    std::cout << "Pipe approach planned, press Enter to conitnue to execution.";
    std::cin.get();

    std::atomic<int> phase_idx_dummy1{1};
    std::thread check_replan_thread;
    std::thread joint_execution_thread;
    check_replan_thread = std::thread([&]() {
            gtsam::Pose3 target_pose = right_arm.target_pose_logs;
            right_arm.replan(
                joint_trajectory, new_joint_trajectory, right_base_frame, target_pose,
                std::ref(vicon_data_mutex), std::ref(joint_data_mutex), 
                std::ref(trajectory_mutex),
                std::ref(replan_triggered), std::ref(new_trajectory_ready), 
                std::ref(right_arm_flag_1), std::ref(phase_idx_dummy1),
                tube_info_snapshot, human_info_snapshot, q_cur_right,
                right_approach_offset_y, right_approach_offset_z, 
                GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
            );
        });

    joint_execution_thread = std::thread([&]() {
        joint_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, 
            right_base_command, right_robot, joint_trajectory, 
            right_base_frame, JOINT_CONTROL_FREQUENCY, 
            std::ref(right_arm_flag_1), std::ref(replan_counter), 
            std::ref(replan_triggered), std::ref(new_trajectory_ready), 
            std::ref(new_joint_trajectory), std::ref(trajectory_mutex), right_grasp_phase_record);
    });
      
    joint_execution_thread.join();
    check_replan_thread.join();

    std::cout << "Pipe approach finished \n";

    {
        std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
        left_base_frame_snapshot = left_base_frame;
        right_base_frame_snapshot = right_base_frame; 
        tube_info_snapshot = tube_info;
        human_info_snapshot = human_info;
    }

    {   
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        q_cur_left_snapshot = shiftAngle(q_cur_left);
        // q_cur_right_snapshot = shiftAngle(q_cur_right);
        
        std::vector<double> std_vec1(joint_trajectory.pos.back().data(), joint_trajectory.pos.back().data() + joint_trajectory.pos.back().size());
        q_cur_right_snapshot = std_vec1;
    }


    // right_arm.make_sdf(left_base_frame_snapshot, q_cur_left_snapshot, tube_info_snapshot, human_info_snapshot, false);
    right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
    right_arm.plan_joint(joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                            tube_info_snapshot, human_info_snapshot, right_grasp_offset_y, 
                            right_grasp_offset_z, right_grasp_time_sec, 
                            GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
    
    visualizeTrajectory(joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
    // std::cout << "Pipe grasp planned, press Enter to conitnue to execution.";
    // std::cin.get();


    right_arm_flag_2.store(true);


    joint_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, 
                right_base_command, right_robot, joint_trajectory, 
                right_base_frame, JOINT_CONTROL_FREQUENCY, 
                std::ref(right_arm_flag_2), std::ref(replan_counter), 
                std::ref(replan_triggered), std::ref(new_trajectory_ready), 
                std::ref(new_joint_trajectory), std::ref(trajectory_mutex), right_grasp_phase_record); 
    
    saveRecordToYAML(right_grasp_phase_record, "right_grasp_phase_record");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    move_gripper(right_base_cyclic, 50);


    while(right_arm_flag_2.load());

    {
        std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
        left_base_frame_snapshot = left_base_frame;
        right_base_frame_snapshot = right_base_frame; 
        tube_info_snapshot = tube_info;
        human_info_snapshot = human_info;
    }

    {
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        q_cur_left_snapshot = shiftAngle(q_cur_left);
        q_cur_right_snapshot = shiftAngle(q_cur_right);
    }

    auto start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
    auto target_pose_to_above_head = right_arm.create_target_pose(human_info_snapshot, right_grasp_offset_y, start_pose, 0.2);
    right_arm.plan_task(task_trajectory, start_pose, target_pose_to_above_head, right_shoulder_time_sec, right_shoulder_percentage, right_shoulder_height);

    std::cout << "Task Start Pose: " << start_pose << "\n";
    std::cout << "Task target Pose: " << target_pose_to_above_head << "\n";

    right_arm_flag_3 = true;

    Eigen::VectorXd vec = task_trajectory.pos.front();
    gtsam::Point3 translation(vec(0), vec(1), vec(2));

    // Extract rotation angles
    double roll = vec(3);
    double pitch = vec(4);
    double yaw = vec(5);

    // Create rotation matrix from RPY angles
    gtsam::Rot3 rotation = gtsam::Rot3::Ypr(yaw, pitch, roll);

    gtsam::Pose3 start_pose_planned = gtsam::Pose3(rotation, translation);

    std::cout << "Planned Task Start Pose: " << start_pose_planned << "\n";

    visualizeTaskTrajectory(task_trajectory.pos, right_arm.dataset_logs, right_base_frame);

    std::cout << "Press Enter To Continue: \n";
    std::cin.get(); 

    std::this_thread::sleep_for(std::chrono::seconds(10));

    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        right_actuator_config->SetControlMode(control_mode_message, id);
    }

    task_control_execution(right_base,right_base_cyclic,right_actuator_config,
            right_base_feedback, right_base_command, right_robot, 
            task_trajectory, right_base_frame, TASK_CONTROL_FREQUENCY, dh_params_path,
            std::ref(right_arm_flag_3));


    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        right_actuator_config->SetControlMode(control_mode_message, id);
    }

    

    std::atomic<bool> left_arm_flag_1 = true;
    std::atomic<bool> left_arm_flag_2 = false;
    std::atomic<bool> left_arm_flag_3 = false;

    while(!left_arm_flag_1.load()); // replace with trigger condition

    {
        std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
        left_base_frame_snapshot = left_base_frame;
        right_base_frame_snapshot = right_base_frame; 
        tube_info_snapshot = tube_info;
        human_info_snapshot = human_info;
    }

    std::cout << "Left base frame: " << left_base_frame_snapshot << "\n";
    std::cout << "Right base frame: " << right_base_frame_snapshot << "\n";

    {   
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        q_cur_left_snapshot = q_cur_left;
        q_cur_right_snapshot = q_cur_right;
    }

    
    left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, right_base_frame_snapshot, q_cur_right_snapshot);
    // std::cout << right_base_frame_snapshot; 
    left_arm.plan_joint(joint_trajectory, q_init_left, left_base_frame_snapshot, 
                         tube_info_snapshot, human_info_snapshot, 
                         right_approach_offset_y+0.1, right_approach_offset_z, 
                         right_approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY,false);

    visualizeTrajectory(joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
    saveTrajectoryResultToYAML(left_arm.result_logs,"left_approach");
    std::cout << "Pipe approach planned, press Enter to conitnue to execution.";
    std::cin.get();

    std::atomic<int> phase_idx_dummy3{3};
    
    check_replan_thread = std::thread([&]() {
            gtsam::Pose3 target_pose = left_arm.target_pose_logs;
            left_arm.replan(
                joint_trajectory, new_joint_trajectory, left_base_frame, target_pose,
                std::ref(vicon_data_mutex), std::ref(joint_data_mutex), 
                std::ref(trajectory_mutex),
                std::ref(replan_triggered), std::ref(new_trajectory_ready), 
                std::ref(left_arm_flag_1), std::ref(phase_idx_dummy3),
                tube_info_snapshot, human_info_snapshot, q_cur_left,
                right_approach_offset_y+0.1, right_approach_offset_z, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
            );
        });

    joint_execution_thread = std::thread([&]() {
        joint_control_execution(left_base,left_base_cyclic,left_actuator_config, left_base_feedback, 
            left_base_command, left_robot, joint_trajectory, 
            left_base_frame, JOINT_CONTROL_FREQUENCY, 
            std::ref(left_arm_flag_1), std::ref(replan_counter), 
            std::ref(replan_triggered), std::ref(new_trajectory_ready), 
            std::ref(new_joint_trajectory), std::ref(trajectory_mutex), right_grasp_phase_record);
    });
      
    joint_execution_thread.join();
    check_replan_thread.join();

    std::cout << "Pipe approach finished \n";


    {
        std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
        left_base_frame_snapshot = left_base_frame;
        right_base_frame_snapshot = right_base_frame; 
        tube_info_snapshot = tube_info;
        human_info_snapshot = human_info;
    }

    {   
        std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
        // q_cur_left_snapshot = shiftAngle(q_cur_left);
        q_cur_right_snapshot = shiftAngle(q_cur_right);
        
        std::vector<double> std_vec1(joint_trajectory.pos.back().data(), joint_trajectory.pos.back().data() + joint_trajectory.pos.back().size());
        q_cur_left_snapshot = std_vec1;
    }


    left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
    left_arm.plan_joint(joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, 
                            tube_info_snapshot, human_info_snapshot, right_grasp_offset_y+0.1, 
                            right_grasp_offset_z, right_grasp_time_sec, 
                            GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
    
    visualizeTrajectory(joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
    
    std::cout << "Pipe grasp planned, press Enter to conitnue to execution.";
    std::cin.get();

    left_arm_flag_2.store(true);


    joint_control_execution(left_base,left_base_cyclic,left_actuator_config, left_base_feedback, 
                left_base_command, left_robot, joint_trajectory, 
                left_base_frame, JOINT_CONTROL_FREQUENCY, 
                std::ref(left_arm_flag_2), std::ref(replan_counter), 
                std::ref(replan_triggered), std::ref(new_trajectory_ready), 
                std::ref(new_joint_trajectory), std::ref(trajectory_mutex), right_grasp_phase_record); 

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    move_gripper(right_base_cyclic, 50);


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
    left_session_manager->CloseSession();
    left_session_manager_real_time->CloseSession();
    left_session_manager_monitor->CloseSession();
   
    left_router->SetActivationStatus(false);
    left_transport->disconnect();
    left_router_real_time->SetActivationStatus(false);
    left_transport_real_time->disconnect();
    left_router_monitor->SetActivationStatus(false);
    left_transport_monitor->disconnect();
   
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
    delete left_base;
    delete left_base_cyclic;
    delete left_base_cyclic_monitor;
    delete left_actuator_config;
    delete left_session_manager;
    delete left_session_manager_real_time;
    delete left_session_manager_monitor;
    delete left_router;
    delete left_router_real_time;
    delete left_router_monitor;
    delete left_transport;
    delete left_transport_real_time;
    delete left_transport_monitor;
   
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