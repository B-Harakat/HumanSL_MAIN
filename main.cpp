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
#define JOINT_CONTROL_FREQUENCY 500
#define TASK_CONTROL_FREQUENCY 300
#define GPMP2_TIMESTEPS 50

// Global variables for cleanup
k_api::Base::BaseClient* g_right_base = nullptr;
k_api::BaseCyclic::BaseCyclicClient* g_right_base_cyclic = nullptr;
k_api::Base::BaseClient* g_left_base = nullptr;
k_api::BaseCyclic::BaseCyclicClient* g_left_base_cyclic = nullptr;
std::atomic<bool> motion_flag{true};

void cleanup_and_exit() {
    std::cout << "\nPerforming emergency shutdown..." << std::endl;
    motion_flag.store(false);

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


bool plan_action(
    std::atomic<int>& trigger_id,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    JointTrajectory& new_joint_trajectory,
    gtsam::Pose3& cur_pose_target,
    std::atomic<bool>& execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex
) {

        gtsam::Pose3 left_base_frame_snapshot;
        gtsam::Pose3 right_base_frame_snapshot;
        TubeInfo tube_info_snapshot;
        HumanInfo human_info_snapshot;
        gtsam::Point3 target_info_snapshot;
        std::vector<double> q_cur_left_snapshot;
        std::vector<double> q_cur_right_snapshot;
        
        {
            std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
            left_base_frame_snapshot = left_base_frame;
            right_base_frame_snapshot = right_base_frame; 
            tube_info_snapshot = tube_info;
            human_info_snapshot = human_info;
            target_info_snapshot = target_info;
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

        std::cout << "left Base pose: " << left_base_frame_snapshot << "\n";
        
        switch(trigger_id.load()){
            case 1: // Right arm engages pipe
            {
                double approach_offset_y = 0.4;
                double approach_offset_z = 0.1;
                double approach_time_sec = 3.0;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                                    tube_info_snapshot, human_info_snapshot, 
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_1");
                std::cout << "tube pose: ";
                for(auto& k : tube_info_snapshot.centroid) {std::cout<< k << ", ";}
                std::cout << "\n";

                std::cout << "fiunal joint pos: ";
                for(auto& k : new_joint_trajectory.pos.back()) {std::cout<< k << ", ";}
                std::cout << "\n";
                gtsam::Vector dummy_vec = new_joint_trajectory.pos.back() * (M_PI/180);
                gtsam::Pose3 final_pose = forwardKinematics(right_arm.dh_params_, dummy_vec, right_base_frame_snapshot);
                std::cout << "Final pose: " << final_pose << "\n";

                break;
            }
            case 2: // Right arm grasps pipe
            {
                double grasp_offset_y = 0.5;
                double grasp_offset_z = 0.001;
                double grasp_time_sec = 1.5;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                            tube_info_snapshot, human_info_snapshot, grasp_offset_y, 
                            grasp_offset_z, grasp_time_sec, 
                            GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
            
                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_2");
                
                break;
            }

            case 3: // Right arm moves pipe task space controlled
            {
                double move_time_sec = 3;
                double move_time_step = 10;
                double intermediate_point_percentage = 0.35;
                double intermediate_point_height = 0.25;

                double move_offset_from_human_max_y = 0.6; // position the gripper furter behind human
                double move_offset_from_human_mid_x = 0.15; // positive means moving the end pose towards human right
                double move_offset_from_human_max_z = 0.1;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;
                q_cur_right_snapshot[3] = -85;
                q_cur_right_snapshot[5] = -60;
                
                gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
                gtsam::Pose3 target_pose = right_arm.over_head_pose(human_info_snapshot, start_pose, 
                                                                    move_offset_from_human_max_y, 
                                                                    move_offset_from_human_mid_x, 
                                                                    move_offset_from_human_max_z);
    

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_task(new_joint_trajectory, start_pose, target_pose, 
                                    right_base_frame_snapshot, q_cur_right_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_3");
                
                break;
            }

            case 4: // Left arm approaches pipe
            {
                double approach_offset_y = 0.5;
                double approach_offset_z = 0.12;
                double approach_time_sec = 3.0;

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_joint(new_joint_trajectory, q_init_left, left_base_frame_snapshot, 
                                    tube_info_snapshot, human_info_snapshot, 
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_4");

                break;
            }

            case 5: // Left arm grasps pipe
            {
                double grasp_offset_y = 0.5;
                double grasp_offset_z = 0.001;
                double grasp_time_sec = 1.5;

                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_joint(new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, 
                            tube_info_snapshot, human_info_snapshot, grasp_offset_y, 
                            grasp_offset_z, grasp_time_sec, 
                            GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
                visualizeTrajectory(new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_5");
                
                break;
            }

            case 6: // Right arm disengages pipe
            {
                
                gtsam::Pose3 cur_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
                double grasp_offset_y = cur_pose.translation().y();

                double grasp_offset_z = 0.2;
                double grasp_time_sec = 1.5;

                bool tune_pose = false;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                            tube_info_snapshot, human_info_snapshot, grasp_offset_y, 
                            grasp_offset_z, grasp_time_sec, 
                            GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY, tune_pose);
                
                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_6");
                
                break;
            }

            case 7: // Right arm moves back to default position
            {
                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                double disengage_time_sec = 3.0;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(new_joint_trajectory, q_cur_right_snapshot, q_init_right, 
                                    right_base_frame_snapshot, disengage_time_sec, 
                                    GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_7");
                
                break;
            }

            case 8: // Left arm moves to target
            {
                gtsam::Pose3 start_pose = left_arm.forward_kinematics(left_base_frame_snapshot,q_cur_left_snapshot);
                gtsam::Pose3 target_pose = left_arm.installtion_pose(target_info, start_pose);
                double move_time_sec = 3.0; 
                int move_time_step = 5;
                double intermediate_point_height = 0.0;
                double intermediate_point_percentage = 0.5;

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_task(new_joint_trajectory, start_pose, target_pose, 
                                    left_base_frame_snapshot, q_cur_left_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_8");
                
                break;
            }

            case 9: // Left arm moves to target
            {
                left_chicken_flag.store(true);
                break;
            }
        }
        

        std::cout << "Pipe approach planned, press Enter to conitnue to execution.";
        std::cin.get();
        

        if(trigger_id.load() == 1 || trigger_id.load() == 2 || trigger_id.load() == 3 || trigger_id.load() == 6 || trigger_id.load() == 7){
            std::vector<double> cur_conf(new_joint_trajectory.pos.back().data(), new_joint_trajectory.pos.back().data() + new_joint_trajectory.pos.back().size());
            cur_pose_target = right_arm.forward_kinematics(right_base_frame_snapshot, cur_conf);

            right_chicken_flag.store(false);

            std::lock_guard<std::mutex> lock(trajectory_mutex);
            right_joint_trajectory = std::move(new_joint_trajectory);
        }

        if(trigger_id.load() == 4 || trigger_id.load() == 5 || trigger_id.load() == 8){
            std::vector<double> cur_conf(new_joint_trajectory.pos.back().data(), new_joint_trajectory.pos.back().data() + new_joint_trajectory.pos.back().size());
            cur_pose_target = left_arm.forward_kinematics(right_base_frame_snapshot, cur_conf);

            left_chicken_flag.store(false);

            std::lock_guard<std::mutex> lock(trajectory_mutex);
            left_joint_trajectory = std::move(new_joint_trajectory);
        }

        execution_ongoing_flag.store(true);
        
        
        return true;  // Success
}


int main(){

    // Kinova connection set up (Hidden)
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
    std::shared_mutex vicon_data_mutex;
    std::shared_mutex joint_data_mutex;
    
    // Thread-safe replanning variables
    std::atomic<bool> execution_ongoing_flag{false};
    std::atomic<bool> check_for_replan{false}; 
    std::atomic<bool> replan_triggered{false};
    std::atomic<bool> new_trajectory_ready{false};
    std::atomic<int> replan_counter{0};
    std::mutex trajectory_mutex;  // For thread-safe trajectory replacement
    
    // Thread-safe phase trigger
    std::atomic<int> phase_idx{0};

    JointTrajectory right_joint_trajectory;
    JointTrajectory left_joint_trajectory;

    JointTrajectory new_joint_trajectory;  // Buffer for new trajectory
    TaskTrajectory task_trajectory;

    gtsam::Pose3 left_base_frame;
    gtsam::Pose3 right_base_frame; 

    Dynamics left_robot(robot_urdf_path);
    Dynamics right_robot(robot_urdf_path);

    TubeInfo tube_info;
    HumanInfo human_info;
    gtsam::Point3 target_info;

    std::vector<double> q_cur_right(7);
    std::vector<double> q_cur_left(7);
    

    gtsam::Pose3 left_base_frame_snapshot; 
    gtsam::Pose3 right_base_frame_snapshot;
    TubeInfo tube_info_snapshot;
    HumanInfo human_info_snapshot;
    std::vector<double> q_cur_right_snapshot; 
    std::vector<double> q_cur_left_snapshot;

    gtsam::Pose3 target_pose_snapshot; 

    std::atomic<bool> right_chicken_flag{false};
    std::atomic<bool> left_chicken_flag{false};

    std::vector<double> q_init_left(7); 
    q_init_left = {-90,90,-15,45,5,5,-175};  // in deg
    std::vector<double> q_init_right(7);
    q_init_right= {90,90,15,45,5,5,5}; // in deg


    Gen3Arm left_arm(left_ip_address, robot_urdf_path, dh_params_path, joint_limit_path);
    Gen3Arm right_arm(right_ip_address, robot_urdf_path, dh_params_path, joint_limit_path);


    TrajectoryRecord right_record;
    TrajectoryRecord left_record;


    move_single_level(left_base, q_init_left);
    move_single_level(right_base, q_init_right);
    
    std::vector<float> commands_left;
    std::vector<float> commands_right;

    // Initialize actuator to low level control
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
            
            updateViconInfo(vicon, left_base_frame, right_base_frame, tube_info, human_info, target_info, q_cur_left, q_cur_right, dh_params_path, vicon_data_mutex, joint_data_mutex);
            updateJointInfo(right_base_cyclic_monitor, q_cur_right, joint_data_mutex);
            updateJointInfo(left_base_cyclic_monitor, q_cur_left, joint_data_mutex);
        }
    });

    info_thread.detach();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 
    
    // auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    // control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    // for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    // {
    //     left_actuator_config->SetControlMode(control_mode_message, id);
    //     right_actuator_config->SetControlMode(control_mode_message, id);
    // }

    std::cout << "Resetting gripper pos\n";
    move_gripper(right_base_cyclic, 0);
    move_gripper(left_base_cyclic,  0);

    Eigen::VectorXd q_init_right_eigen = Eigen::Map<Eigen::VectorXd>(q_init_right.data(), q_init_right.size());
    Eigen::VectorXd q_init_left_eigen = Eigen::Map<Eigen::VectorXd>(q_init_left.data(), q_init_left.size());
    
    Eigen::VectorXd q_init_vel(7); Eigen::VectorXd q_init_acc(7);
    q_init_vel << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
    q_init_acc << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;

    right_joint_trajectory.pos.push_back(q_init_right_eigen);
    right_joint_trajectory.vel.push_back(q_init_vel);
    right_joint_trajectory.acc.push_back(q_init_acc);

    left_joint_trajectory.pos.push_back(q_init_left_eigen);
    left_joint_trajectory.vel.push_back(q_init_vel);
    left_joint_trajectory.acc.push_back(q_init_acc);

    // Set actuators in torque mode
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        left_actuator_config->SetControlMode(control_mode_message, id);
        right_actuator_config->SetControlMode(control_mode_message, id);
    }

    std::cout << "Switching to torque mode and initializing each arm \n";

    std::thread right_robot_execution_thread;
    right_robot_execution_thread = std::thread([&]() {
        joint_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, 
            right_base_command, right_robot, right_joint_trajectory, 
            right_base_frame, JOINT_CONTROL_FREQUENCY,
            std::ref(motion_flag), std::ref(execution_ongoing_flag),
            std::ref(right_chicken_flag), std::ref(vicon_data_mutex), dh_params_path,
            std::ref(replan_counter), 
            std::ref(replan_triggered), std::ref(new_trajectory_ready), 
            std::ref(new_joint_trajectory), std::ref(trajectory_mutex), std::ref(right_record));
    });

    std::thread left_robot_execution_thread;
    left_robot_execution_thread = std::thread([&]() {
        joint_control_execution(left_base,left_base_cyclic,left_actuator_config, left_base_feedback, 
            left_base_command, left_robot, left_joint_trajectory, 
            left_base_frame, JOINT_CONTROL_FREQUENCY, 
            std::ref(motion_flag), std::ref(execution_ongoing_flag),
            std::ref(left_chicken_flag), std::ref(vicon_data_mutex), dh_params_path,
            std::ref(replan_counter), 
            std::ref(replan_triggered), std::ref(new_trajectory_ready), 
            std::ref(new_joint_trajectory), std::ref(trajectory_mutex), std::ref(left_record));
    });

      
    right_robot_execution_thread.detach();
    left_robot_execution_thread.detach();

    std::thread replan_thread;

    phase_idx.store(1);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    replan_thread =std::thread([&]() {
        right_arm.replan(
                    right_joint_trajectory, new_joint_trajectory, right_base_frame, target_pose_snapshot,
                    std::ref(vicon_data_mutex),
                    std::ref(trajectory_mutex), std::ref(replan_triggered), 
                    std::ref(new_trajectory_ready), std::ref(execution_ongoing_flag),
                    human_info, tube_info, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
                );
    });
    replan_thread.detach();

    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    
    std::cout << "target pos: ";
     for(auto& k : right_joint_trajectory.pos.back()){std::cout << k << ", ";} 
    std::cout << "\n";

    std::cout << "measured pos: ";
     for(auto& k : q_cur_right){std::cout << k << ", ";} 
    std::cout << "\n";

    phase_idx.store(2);

    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    
    move_gripper(right_base_cyclic, 50);
    

    phase_idx.store(3);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    phase_idx.store(4);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    replan_thread =std::thread([&]() {
        left_arm.replan(
                    left_joint_trajectory, new_joint_trajectory, left_base_frame, target_pose_snapshot,
                    std::ref(vicon_data_mutex),
                    std::ref(trajectory_mutex), std::ref(replan_triggered), 
                    std::ref(new_trajectory_ready), std::ref(execution_ongoing_flag),
                    human_info, tube_info, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
                );
    });
    replan_thread.detach();

    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    phase_idx.store(5);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    phase_idx.store(6);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    phase_idx.store(7);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    phase_idx.store(8);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    while(execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

    
    phase_idx.store(9);
    
    plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                tube_info, human_info, target_info, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
    
    while(true);











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