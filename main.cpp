#include "core.h"
#include "move.h"
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
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7
#define JOINT_CONTROL_FREQUENCY 500
#define TASK_CONTROL_FREQUENCY 300
#define GPMP2_TIMESTEPS 10

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
    std::atomic<bool> left_execution_ongoing_flag{false};
    std::atomic<bool> right_execution_ongoing_flag{false};
    std::atomic<bool> check_for_replan{false}; 
    std::atomic<bool> replan_triggered{false};
    std::atomic<bool> new_trajectory_ready{false};
    std::atomic<int> replan_counter{0};
    std::mutex trajectory_mutex;  // For thread-safe trajectory replacement
    
    // Thread-safe state idx
    std::atomic<int> state_idx{0};
    std::atomic<int> prev_state_idx{0};

    JointTrajectory right_joint_trajectory;
    JointTrajectory left_joint_trajectory;

    JointTrajectory new_joint_trajectory;  // Buffer for new trajectory

    gtsam::Pose3 left_base_frame;
    gtsam::Pose3 right_base_frame; 

    Dynamics left_robot(robot_urdf_path);
    Dynamics right_robot(robot_urdf_path);

    TubeInfo tube_info;
    HumanInfo human_info;
    gtsam::Point3 target_info;
    Eigen::Vector3d head_info;
    Eigen::Vector3d lfin_info;
    Eigen::Vector3d rfin_info;

    Eigen::Vector3d init_tube_pos;

    std::vector<double> q_cur_right(7);
    std::vector<double> q_cur_left(7);
    std::vector<double> u_cur_left(7);
    std::vector<double> u_cur_right(7);
    

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

        DHParameters dh_params = createDHParams(dh_params_path);

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
            
            updateViconInfo(vicon, left_base_frame, right_base_frame, tube_info, human_info, target_info, q_cur_left, q_cur_right, lfin_info, rfin_info, head_info, dh_params, vicon_data_mutex, joint_data_mutex);
            updateJointInfo(right_base_cyclic_monitor, q_cur_right, u_cur_right, joint_data_mutex);
            updateJointInfo(left_base_cyclic_monitor, q_cur_left, u_cur_left, joint_data_mutex);


        }
    });

    info_thread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 

    std::thread state_monitor_thread([&]() {
        while(true){
            state_monitor(lfin_info, rfin_info, head_info, tube_info, std::ref(state_idx), std::ref(vicon_data_mutex));
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        }
    });

    state_monitor_thread.detach();

    {
        std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
        init_tube_pos = tube_info.centroid;

        std::cout << "initial tube_info: \n";
        std::cout << "Tube centroid: " << tube_info.centroid.x() <<", " << tube_info.centroid.y() << ", "<<tube_info.centroid.z() << "\n";
        std::cout << "Tube direction: " << tube_info.direction.x() <<", " << tube_info.direction.y() << ", "<<tube_info.direction.z() << "\n";

        std::cout << "Initial Right base: \n";
        std::cout << right_base_frame << "\n";

        std::cout << "Initial Left base: \n";
        std::cout << left_base_frame << "\n";
    }

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


    std::thread record_thread([&](){

        thread_local int prev_frame_number = -1;
        
        // Create timestamped CSV file
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm* tm = std::localtime(&time_t);
        
        std::stringstream filename;
        filename << "../logs/experimental_results/" 
                 << std::setfill('0') << std::setw(2) << (tm->tm_mon + 1) << "_"
                 << std::setfill('0') << std::setw(2) << tm->tm_mday << "_"
                 << std::setfill('0') << std::setw(2) << tm->tm_hour << "_"
                 << std::setfill('0') << std::setw(2) << tm->tm_min << ".csv";
        
        std::ofstream csv_file(filename.str());
        
        // Write CSV header
        csv_file << "frame_number,";
        // Joint data headers
        for(int i = 0; i < 7; i++) csv_file << "q_cur_left_" << i << ",";
        for(int i = 0; i < 7; i++) csv_file << "q_cur_right_" << i << ",";
        for(int i = 0; i < 7; i++) csv_file << "q_target_left_" << i << ",";
        for(int i = 0; i < 7; i++) csv_file << "q_target_right_" << i << ",";
        for(int i = 0; i < 7; i++) csv_file << "u_cur_left_" << i << ",";
        for(int i = 0; i < 7; i++) csv_file << "u_cur_right_" << i << ",";
        // Pose3 headers
        csv_file << "left_base_x,left_base_y,left_base_z,left_base_rx,left_base_ry,left_base_rz,";
        csv_file << "right_base_x,right_base_y,right_base_z,right_base_rx,right_base_ry,right_base_rz,";
        // Tube headers
        csv_file << "tube_centroid_x,tube_centroid_y,tube_centroid_z,tube_direction_x,tube_direction_y,tube_direction_z,";
        // Human info headers
        csv_file << "human_RFIN_x,human_RFIN_y,human_RFIN_z,human_LFIN_x,human_LFIN_y,human_LFIN_z,";
        csv_file << "human_RFHD_x,human_RFHD_y,human_RFHD_z,human_LFHD_x,human_LFHD_y,human_LFHD_z,";
        // Individual marker headers
        csv_file << "right_base1_x,right_base1_y,right_base1_z,right_base2_x,right_base2_y,right_base2_z,right_base3_x,right_base3_y,right_base3_z,";
        csv_file << "left_base1_x,left_base1_y,left_base1_z,left_base2_x,left_base2_y,left_base2_z,left_base3_x,left_base3_y,left_base3_z,";
        csv_file << "right_tip1_x,right_tip1_y,right_tip1_z,right_tip2_x,right_tip2_y,right_tip2_z,right_tip3_x,right_tip3_y,right_tip3_z,right_tip4_x,right_tip4_y,right_tip4_z,";
        csv_file << "left_tip1_x,left_tip1_y,left_tip1_z,left_tip2_x,left_tip2_y,left_tip2_z,left_tip3_x,left_tip3_y,left_tip3_z,left_tip4_x,left_tip4_y,left_tip4_z,";
        csv_file << "tube_tip1_x,tube_tip1_y,tube_tip1_z,tube_tip2_x,tube_tip2_y,tube_tip2_z,tube_tip3_x,tube_tip3_y,tube_tip3_z,";
        csv_file << "tube_end1_x,tube_end1_y,tube_end1_z,tube_end2_x,tube_end2_y,tube_end2_z,tube_end3_x,tube_end3_y,tube_end3_z,";
        csv_file << "tube_mid1_x,tube_mid1_y,tube_mid1_z,tube_mid2_x,tube_mid2_y,tube_mid2_z,tube_mid3_x,tube_mid3_y,tube_mid3_z,";
        // Head and target headers
        csv_file << "head_x,head_y,head_z,target_x,target_y,target_z,state_idx" << std::endl;

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

            Eigen::VectorXd q_target_left_snapshot;
            Eigen::VectorXd q_target_right_snapshot;

            gtsam::Pose3 left_base_frame_snapshot;
            gtsam::Pose3 right_base_frame_snapshot;
            TubeInfo tube_info_snapshot;
            HumanInfo human_info_snapshot;
            Eigen::Vector3d head_info_snapshot;
            gtsam::Point3 target_info_snapshot;
            std::vector<double> q_cur_left_snapshot;
            std::vector<double> q_cur_right_snapshot;
            std::vector<double> u_cur_left_snapshot;
            std::vector<double> u_cur_right_snapshot;
            
            // Individual marker data snapshots
            std::vector<MarkerData> right_base_data_snapshot;
            std::vector<MarkerData> left_base_data_snapshot;
            std::vector<MarkerData> right_tip_data_snapshot;
            std::vector<MarkerData> left_tip_data_snapshot;
            std::vector<MarkerData> tube_tip_snapshot;
            std::vector<MarkerData> tube_end_snapshot;
            std::vector<MarkerData> tube_mid_snapshot;
            std::vector<MarkerData> human_markers_snapshot;
            std::vector<MarkerData> target_markers_snapshot;
        
            {
                std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
                left_base_frame_snapshot = left_base_frame;
                right_base_frame_snapshot = right_base_frame; 
                tube_info_snapshot = tube_info;
                human_info_snapshot = human_info;
                target_info_snapshot = target_info;
                head_info_snapshot = head_info;
                
                // Helper function to safely get marker data
                auto safeGetMarker = [&vicon](const std::string& name) -> MarkerData {
                    try {
                        MarkerData marker = vicon.getMarkerPosition(name);
                        if (marker.occluded || std::isnan(marker.x) || std::isnan(marker.y) || std::isnan(marker.z)) {
                            return {name, 0.0, 0.0, 0.0, true};
                        }
                        return marker;
                    } catch (...) {
                        return {name, 0.0, 0.0, 0.0, true};
                    }
                };

                auto safeGetMarkers = [&vicon](const std::string& subject) -> std::vector<MarkerData> {
                    try {
                        std::vector<MarkerData> markers = vicon.getMarkerPositions(subject);
                        for (auto& marker : markers) {
                            if (marker.occluded || std::isnan(marker.x) || std::isnan(marker.y) || std::isnan(marker.z)) {
                                marker.x = marker.y = marker.z = 0.0;
                                marker.occluded = true;
                            }
                        }
                        return markers;
                    } catch (...) {
                        return std::vector<MarkerData>();
                    }
                };
                
                // Collect individual marker data
                right_base_data_snapshot.clear();
                right_base_data_snapshot.push_back(safeGetMarker("right_base1"));
                right_base_data_snapshot.push_back(safeGetMarker("right_base2"));
                right_base_data_snapshot.push_back(safeGetMarker("right_base3"));
                
                left_base_data_snapshot.clear();
                left_base_data_snapshot.push_back(safeGetMarker("left_base1"));
                left_base_data_snapshot.push_back(safeGetMarker("left_base2"));
                left_base_data_snapshot.push_back(safeGetMarker("left_base3"));
                
                right_tip_data_snapshot.clear();
                right_tip_data_snapshot.push_back(safeGetMarker("right_tip1"));
                right_tip_data_snapshot.push_back(safeGetMarker("right_tip2"));
                right_tip_data_snapshot.push_back(safeGetMarker("right_tip3"));
                right_tip_data_snapshot.push_back(safeGetMarker("right_tip4"));
                
                left_tip_data_snapshot.clear();
                left_tip_data_snapshot.push_back(safeGetMarker("left_tip1"));
                left_tip_data_snapshot.push_back(safeGetMarker("left_tip2"));
                left_tip_data_snapshot.push_back(safeGetMarker("left_tip3"));
                left_tip_data_snapshot.push_back(safeGetMarker("left_tip4"));
                
                tube_tip_snapshot.clear();
                tube_tip_snapshot.push_back(safeGetMarker("tube_tip1"));
                tube_tip_snapshot.push_back(safeGetMarker("tube_tip2"));
                tube_tip_snapshot.push_back(safeGetMarker("tube_tip3"));
                
                tube_end_snapshot.clear();
                tube_end_snapshot.push_back(safeGetMarker("tube_end1"));
                tube_end_snapshot.push_back(safeGetMarker("tube_end2"));
                tube_end_snapshot.push_back(safeGetMarker("tube_end3"));
                
                tube_mid_snapshot.clear();
                tube_mid_snapshot.push_back(safeGetMarker("tube_mid1"));
                tube_mid_snapshot.push_back(safeGetMarker("tube_mid2"));
                tube_mid_snapshot.push_back(safeGetMarker("tube_mid3"));
                
                human_markers_snapshot = safeGetMarkers("human");
                target_markers_snapshot = safeGetMarkers("target");
            }

            {   
                std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
                q_cur_left_snapshot = shiftAngle(q_cur_left);
                q_cur_right_snapshot = shiftAngle(q_cur_right);
                u_cur_left_snapshot = u_cur_left;
                u_cur_right_snapshot = u_cur_right;
            }
   
            {
                std::lock_guard<std::mutex> lock(trajectory_mutex);
                q_target_left_snapshot = left_joint_trajectory.pos.front(); 
                q_target_right_snapshot = right_joint_trajectory.pos.front(); 
            }

            // Write CSV row
            csv_file << cur_frame_number << ",";
            
            // Joint current data
            for(int i = 0; i < 7; i++) csv_file << q_cur_left_snapshot[i] << ",";
            for(int i = 0; i < 7; i++) csv_file << q_cur_right_snapshot[i] << ",";
            
            // Joint target data
            for(int i = 0; i < 7; i++) {
                if(i < q_target_left_snapshot.size()) {
                    csv_file << q_target_left_snapshot[i] << ",";
                } else {
                    csv_file << "0.0,";
                }
            }
            for(int i = 0; i < 7; i++) {
                if(i < q_target_right_snapshot.size()) {
                    csv_file << q_target_right_snapshot[i] << ",";
                } else {
                    csv_file << "0.0,";
                }
            }
            
            // Joint control input data
            for(int i = 0; i < 7; i++) csv_file << u_cur_left_snapshot[i] << ",";
            for(int i = 0; i < 7; i++) csv_file << u_cur_right_snapshot[i] << ",";
            
            // Pose3 data - left base
            gtsam::Point3 left_translation = left_base_frame_snapshot.translation();
            gtsam::Vector3 left_rpy = left_base_frame_snapshot.rotation().rpy();
            csv_file << left_translation.x() << "," << left_translation.y() << "," << left_translation.z() << ","
                     << left_rpy(0) << "," << left_rpy(1) << "," << left_rpy(2) << ",";
            
            // Pose3 data - right base
            gtsam::Point3 right_translation = right_base_frame_snapshot.translation();
            gtsam::Vector3 right_rpy = right_base_frame_snapshot.rotation().rpy();
            csv_file << right_translation.x() << "," << right_translation.y() << "," << right_translation.z() << ","
                     << right_rpy(0) << "," << right_rpy(1) << "," << right_rpy(2) << ",";
            
            // Tube data
            csv_file << tube_info_snapshot.centroid.x() << "," << tube_info_snapshot.centroid.y() << "," << tube_info_snapshot.centroid.z() << ","
                     << tube_info_snapshot.direction.x() << "," << tube_info_snapshot.direction.y() << "," << tube_info_snapshot.direction.z() << ",";
            
            // Human info - specific markers
            csv_file << human_info_snapshot.RFIN.x() << "," << human_info_snapshot.RFIN.y() << "," << human_info_snapshot.RFIN.z() << ","
                     << human_info_snapshot.LFIN.x() << "," << human_info_snapshot.LFIN.y() << "," << human_info_snapshot.LFIN.z() << ","
                     << human_info_snapshot.RFHD.x() << "," << human_info_snapshot.RFHD.y() << "," << human_info_snapshot.RFHD.z() << ","
                     << human_info_snapshot.LFHD.x() << "," << human_info_snapshot.LFHD.y() << "," << human_info_snapshot.LFHD.z() << ",";
            
            // Individual marker data - right base
            for(const auto& marker : right_base_data_snapshot) {
                csv_file << marker.x << "," << marker.y << "," << marker.z << ",";
            }
            // Individual marker data - left base
            for(const auto& marker : left_base_data_snapshot) {
                csv_file << marker.x << "," << marker.y << "," << marker.z << ",";
            }
            // Individual marker data - right tip
            for(const auto& marker : right_tip_data_snapshot) {
                csv_file << marker.x << "," << marker.y << "," << marker.z << ",";
            }
            // Individual marker data - left tip
            for(const auto& marker : left_tip_data_snapshot) {
                csv_file << marker.x << "," << marker.y << "," << marker.z << ",";
            }
            // Individual marker data - tube tip
            for(const auto& marker : tube_tip_snapshot) {
                csv_file << marker.x << "," << marker.y << "," << marker.z << ",";
            }
            // Individual marker data - tube end
            for(const auto& marker : tube_end_snapshot) {
                csv_file << marker.x << "," << marker.y << "," << marker.z << ",";
            }
            // Individual marker data - tube mid
            for(const auto& marker : tube_mid_snapshot) {
                csv_file << marker.x << "," << marker.y << "," << marker.z << ",";
            }
            
            // Head and target data
            csv_file << head_info_snapshot.x() << "," << head_info_snapshot.y() << "," << head_info_snapshot.z() << ","
                     << target_info_snapshot.x() << "," << target_info_snapshot.y() << "," << target_info_snapshot.z() << ","
                     << state_idx << std::endl;
            
            csv_file.flush();

        }

    });
    
    record_thread.detach();

    // std::cin.get();


    std::cout << "Resetting gripper pos\n";
    move_gripper(right_base_cyclic, 0);
    move_gripper(left_base_cyclic,  0);

    // std::cin.get();

    // Set actuators in torque mode
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        left_actuator_config->SetControlMode(control_mode_message, id);
        right_actuator_config->SetControlMode(control_mode_message, id);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10)); 

    std::cout << "Switching to torque mode and initializing each arm \n";

    std::thread right_robot_execution_thread;
    right_robot_execution_thread = std::thread([&]() {
        joint_position_control_execution(right_base,right_base_cyclic,right_actuator_config, right_base_feedback, 
            right_base_command, right_robot, right_joint_trajectory, 
            right_base_frame, JOINT_CONTROL_FREQUENCY,
            std::ref(motion_flag), std::ref(right_execution_ongoing_flag),
            std::ref(right_chicken_flag), std::ref(vicon_data_mutex), dh_params_path,
            std::ref(replan_counter), 
            std::ref(replan_triggered), std::ref(new_trajectory_ready), 
            std::ref(new_joint_trajectory), std::ref(trajectory_mutex), std::ref(right_record));
    });

    std::thread left_robot_execution_thread;
    left_robot_execution_thread = std::thread([&]() {
        joint_position_control_execution(left_base,left_base_cyclic,left_actuator_config, left_base_feedback, 
            left_base_command, left_robot, left_joint_trajectory, 
            left_base_frame, JOINT_CONTROL_FREQUENCY, 
            std::ref(motion_flag), std::ref(left_execution_ongoing_flag),
            std::ref(left_chicken_flag), std::ref(vicon_data_mutex), dh_params_path,
            std::ref(replan_counter), 
            std::ref(replan_triggered), std::ref(new_trajectory_ready), 
            std::ref(new_joint_trajectory), std::ref(trajectory_mutex), std::ref(left_record));
    });

      
    right_robot_execution_thread.detach();
    left_robot_execution_thread.detach();


    while(true){
        state_transition(
            std::ref(state_idx), 
            std::ref(prev_state_idx),
            std::ref(vicon_data_mutex),
            std::ref(joint_data_mutex),
            std::ref(replan_triggered),
            std::ref(new_trajectory_ready),
            left_base_frame,
            right_base_frame,
            tube_info,
            human_info,
            target_info,
            head_info,
            init_tube_pos,
            q_cur_left,
            q_cur_right,
            q_init_left,
            q_init_right,
            right_arm,
            left_arm,
            left_base_cyclic,
            right_base_cyclic,
            left_joint_trajectory,
            right_joint_trajectory,
            new_joint_trajectory,
            std::ref(left_execution_ongoing_flag),
            std::ref(right_execution_ongoing_flag),
            std::ref(left_chicken_flag),
            std::ref(right_chicken_flag),
            std::ref(trajectory_mutex)
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }

    while(true);

    std::cout << "SOmehow bypassed?";





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