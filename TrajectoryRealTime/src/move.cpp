#include "move.h"
#include "KinovaTrajectory.h"
#include "utils.h"
#include <shared_mutex>

using namespace Jacobian;
using namespace Fwd_kinematics;
using namespace Controller;
using namespace Filter;

bool joint_position_control_single(k_api::Base::BaseClient* base,
                            k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                            k_api::BaseCyclic::Feedback& base_feedback, 
                            k_api::BaseCyclic::Command& base_command,
                            VectorXd& q_d, VectorXd& q_cur) {

    int num_joints = q_d.size();

    // Initialize cyclic communication
    base_feedback = base_cyclic->RefreshFeedback();

    for(int i = 0; i < num_joints; i++) {
        q_cur[i] = base_feedback.actuators(i).position();
    }

    shiftAngleInPlace(q_cur);  

    for(int i = 0; i < num_joints; i++) {
        base_command.mutable_actuators(i)->set_position(q_d[i]);
    }
 
    // Update frame ID
    base_command.set_frame_id(base_command.frame_id() + 1);
    if(base_command.frame_id() > 65535) {
        base_command.set_frame_id(0);
    }

    // Set command IDs
    for(int i = 0; i < num_joints; i++) {
        base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
    }
    // std::cout<<"here 7 \n";
    try {
        base_feedback = base_cyclic->Refresh(base_command, 0);
    } catch(k_api::KDetailedException& ex) {
        std::cout << "Kortex exception during trajectory execution: " << ex.what() << std::endl;
        return false;
    } catch(std::runtime_error& ex) {
        std::cout << "Runtime error during trajectory execution: " << ex.what() << std::endl;
        return false;
    } catch(...) {
        std::cout << "Unknown error during trajectory execution" << std::endl;
        return false;
    }
    
    return true;
}


bool joint_impedance_control_single(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command,
                                    Dynamics &robot,
                                    VectorXd& q_d, VectorXd& dq_d, VectorXd& ddq_d, VectorXd& last_dq,
                                    VectorXd& K_joint_diag, VectorXd& q_cur, int control_frequency) {


    // KINOVA feedback (joint space variables)
    VectorXd q(7), dq(7), ddq(7), u(7);

    // Time for one control iteration
    double dt = 1.0 / control_frequency;

    // k_api::BaseCyclic::Feedback base_feedback;
    // k_api::BaseCyclic::Command base_command;

    try
    {
        // Get current feedback
        base_feedback = base_cyclic->RefreshFeedback();

        // KINOVA Feedback: Get actual joint positions, velocities, torques & current
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            q_cur[i] = base_feedback.actuators(i).position();
            // q[i] = (M_PI/180) * base_feedback.actuators(i).position();
            dq[i] = (M_PI/180) * base_feedback.actuators(i).velocity();
        }
        
        ddq = (dq - last_dq) / dt;
        last_dq = dq;

        shiftAngleInPlace(q_cur);
        q = q_cur * (M_PI/180);
        
        // Joint space impedance controller
        std::tie(u) = joint_impedance_controller(robot, q, dq, ddq, q_d, dq_d, ddq_d, 
                                        K_joint_diag, control_frequency, dt);

        // Prepare command
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
            base_command.mutable_actuators(i)->set_torque_joint(u[i]);
        }

        // Set frame ID
        base_command.set_frame_id(base_feedback.frame_id() + 1);
        if (base_command.frame_id() > 65535)
            base_command.set_frame_id(0);

        for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
        {
            base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
        }

        // Send single command to robot
        base_feedback = base_cyclic->Refresh(base_command, 0);
    

        return true;
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex exception: " << ex.what() << std::endl;
        std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        return false;
    }
    catch (runtime_error& ex2)
    {
        std::cout << "runtime error: " << ex2.what() << std::endl;
        return false;
    }
    catch(...)
    {
        std::cout << "Unknown error." << std::endl;
        return false;
    }
}


void joint_control_execution(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command, 
                                Dynamics &robot,
                                JointTrajectory& trajectory,
                                gtsam::Pose3& base_frame, 
                                int control_frequency, std::atomic<bool>& flag, 
                                std::atomic<int>& replan_counter, std::atomic<bool>& replan_triggered,
                                std::atomic<bool>& new_trajectory_ready, JointTrajectory& new_trajectory,
                                std::mutex& trajectory_mutex, TrajectoryRecord& record){
    static thread_local int local_counter = 0;
    Eigen::VectorXd K_joint_diag(7);
    K_joint_diag << 100,100,100,100,100,100,100;

    // Monitor replan flag state
    static bool previous_replan_state = false;

    const double iteration_time = (1.0 / control_frequency) * 1000;
    std::cout << "Iteration time: " << iteration_time << "\n";

    VectorXd q_cur(7);
    
    while(flag){

        auto start_control = chrono::high_resolution_clock::now();

        // Thread-safe trajectory access
        Eigen::VectorXd q_d, dq_d, ddq_d, last_dq;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex);

            // if(trajectory.pos.size()%100 == 0) std::cout << trajectory.pos.size() << "\n";
            
            auto [q, dq, ddq] = pop_front(trajectory);
            q_d = q; dq_d = dq; ddq_d = ddq; last_dq = dq;


        }
        
        // auto start_control_test_1 = chrono::high_resolution_clock::now();

        // if(!joint_position_control_single(base,base_cyclic,base_feedback, base_command, q_d, q_cur)){
        //     throw std::runtime_error("ERROR: joint_position_control_single");
        // }


        if(!joint_impedance_control_single(base, base_cyclic, actuator_config, base_feedback, base_command, robot, q_d, dq_d, ddq_d, last_dq, K_joint_diag, q_cur, control_frequency)){
            throw std::runtime_error("ERROR: joint_impedance_control_single");
        };

        record.target_trajectory.push_back(q_d);
        record.actual_trajectory.push_back(q_cur);

        // auto end_control_test_1 = chrono::high_resolution_clock::now();
        // std::cout << "Control frequency: "<< chrono::duration<double, milli>(end_control_test_1 - start_control_test_1).count() << "\n";


        // Check if replan flag was triggered (transition from false to true)
        bool current_replan_state = replan_triggered.load();
        if (current_replan_state && !previous_replan_state) {
            // Replan flag just became true, reset the counter
            replan_counter.store(0);
            std::cout << "Replan triggered! Counter reset to 0." << std::endl;
        }
        previous_replan_state = current_replan_state;
        
        // If replan flag is active, increment the counter every iteration
        if (current_replan_state) {
            replan_counter.fetch_add(1);
            
            // Check for trajectory replacement at 200ms (100 iterations at 500Hz)
            int current_counter = replan_counter.load();
            size_t size_to_skip = static_cast<size_t>(240/(1000.0/control_frequency));
            if (current_counter >= size_to_skip && new_trajectory_ready.load()) {
                std::cout << "Replacing trajectory at counter: " << current_counter << std::endl;
                
                // Thread-safe trajectory replacement
                {
                    std::lock_guard<std::mutex> lock(trajectory_mutex);

                    std::cout << "old joint config at change: ";
                    for(auto& dummy: q_d){std::cout << std::round(dummy * 100.0) / 100.0 << ", ";}
                    std::cout << "\n";
                    std::cout << "new joint config at change: ";
                    for(auto& dummy: new_trajectory.pos.front()){std::cout << std::round(dummy * 100.0) / 100.0  << ", ";}

                    trajectory = std::move(new_trajectory);
                }
                
                // Reset all flags
                replan_triggered.store(false);
                new_trajectory_ready.store(false);
                replan_counter.store(0);
                previous_replan_state = false;
                
                std::cout << "Trajectory replacement completed. Continuing with new trajectory." << std::endl;
            }
        }
               
        // Original trajectory completion logic
        size_t trajectory_size = trajectory.pos.size();
        
        if(trajectory_size == 1){
            local_counter += 1;
            // std::cout << local_counter << "\n";
        }
        else{
            local_counter = 0;
        }

        if(local_counter >= 1000){
            flag.store(false);
            local_counter = 0;
            break;
        }

        auto end_control = chrono::high_resolution_clock::now();
        auto run_time = chrono::duration<double, milli>(end_control - start_control).count();
        int diff = 0;
        // std::cout << "Run time: " << run_time << "\n";
        if (run_time < iteration_time) {
            auto start_delay = chrono::high_resolution_clock::now();
            diff = (iteration_time - run_time) * 1000;
            chrono::microseconds delay(diff);
            while (chrono::high_resolution_clock::now() - start_delay < delay);
        }
    }

    std::cout << "Joint execution done, exiting... \n";
}

// Overloaded function for backward compatibility (without replanning support)
void joint_control_execution(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command,
                                Dynamics &robot,
                                JointTrajectory& trajectory, gtsam::Pose3& base_frame,
                                int control_frequency, std::atomic<bool>& flag) {

    static thread_local int local_counter = 0;
    Eigen::VectorXd K_joint_diag(7);
    K_joint_diag << 100,100,100,100,100,100,100;

    // Monitor replan flag state
    static bool previous_replan_state = false;

    VectorXd q_cur(7);

    while(flag){

        auto start_control = chrono::high_resolution_clock::now();
        auto [q, dq, ddq] = pop_front(trajectory);

        const double iteration_time = (1.0 / control_frequency) * 1000;
        
        if(!joint_position_control_single(base,base_cyclic,base_feedback, base_command, q, q_cur)){
            throw std::runtime_error("ERROR: joint_position_control_single");
        }

        if(trajectory.pos.size() == 1){
            local_counter += 1;
        }
        else{
            local_counter = 0;
        }
        std::cout << local_counter << "\n";
        if(local_counter > 1000){
            local_counter = 0;
            flag = false;
        }

        auto end_control = chrono::high_resolution_clock::now();
        auto run_time = chrono::duration<double, milli>(end_control - start_control).count();
        int diff = 0;
        if (run_time < iteration_time) {
            auto start_delay = chrono::high_resolution_clock::now();
            diff = (iteration_time - run_time) * 1000;
            chrono::microseconds delay(diff);
            while (chrono::high_resolution_clock::now() - start_delay < delay);
        }
    }
}

bool task_impedance_control_single(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                       k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command, Dynamics &robot,
                       VectorXd& p_d, VectorXd& dp_d, VectorXd& ddp_d, VectorXd& K_d_diag, int control_frequency, bool& first_call, VectorXd& last_dq, std::chrono::time_point<std::chrono::high_resolution_clock>& start_measure, DHParameters_Eigen& dh_eigen) {
    

    // KINOVA feedback (joint space variables)
    VectorXd q(7), dq(7), ddq(7);

    // Task space variables
    VectorXd u(7), p(6), dp(6), ddp(6);
    MatrixXd T_B7(4,4);  // Rotation matrix

    // Define the D_n and K_n for nullspace
    VectorXd K_n_diag(7);
    K_n_diag << 4, 4, 4, 2, 2, 2, 2;

    // Time for one control iterative
    double dt = 1.0 / control_frequency;

 
    base_feedback = base_cyclic->RefreshFeedback();

    // KINOVA Feedback: Obtaining gen3 ACTUAL joint positions, velocities, torques & current
    for (int i = 0; i < 7; i++) {
      q[i] = base_feedback.actuators(i).position();  
      dq[i] = base_feedback.actuators(i).velocity();
    }

    shiftAngleInPlace(q);  // Apply angle wrapping in degrees


    for (int i = 0; i < 7; i++) {
        q[i] = (M_PI/180) * q[i];   // Convert to radians
        dq[i] = (M_PI/180) * dq[i]; // Convert to radians
    }

    // Apply the forward kinematics
    std::tie(p, T_B7) = forward(dh_eigen,q);

    // std::cout << "p_d relative to base_frame: ";
    // for(auto& k : p_d){std::cout<< k <<", ";}
    // std::cout << "\n";

    // std::cout << "fwd measured pose: ";
    // for(auto& k : p){std::cout<< k <<", ";}
    // std::cout << "\n";

    // std::cout << "Blocked ...";
    // std::cin.get();



    // initilize the controller and filter
    if(first_call == true){
        Vector<double, 3> pos = p.head(3);

        ini_controller(pos, T_B7);

        ini_butterworth();
    }

    auto end_measure = chrono::high_resolution_clock::now();
    
    
    if (first_call == true){
        dt = 1.0 / control_frequency;
        first_call = false;
    }
    else{
        chrono::duration<double> measure_dur = end_measure - start_measure;
        dt = measure_dur.count();
    }

    // std::cout << "printing dq \n";
    // for (auto& dq_dummy : dq){
    //     std::cout << dq_dummy <<"\n";
    // }

    // std::cout << "now printing last_dq \n";
    // for (auto& last_dq_dummy : last_dq){
    //     std::cout << last_dq_dummy <<"\n";
    // }

    ddq = (dq - last_dq) / dt;
    last_dq = dq;

    // std::cout << "now printing ddq \n";
    // for (auto& last_ddq_dummy : ddq){
    //     std::cout << last_ddq_dummy <<"\n";
    // }


    start_measure = chrono::high_resolution_clock::now();

    VectorXd acc_factor(6);
    // Impedance controller
    std::tie(u, dp, ddp, acc_factor) = task_impedance_controller(robot, q, dq, ddq, T_B7, p_d, dp_d,
                                                        ddp_d, K_d_diag, K_n_diag, control_frequency, dt);
    
    for (int i = 0; i < 7; i++)
    {
        // -- Position
        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
        // -- Torque
        base_command.mutable_actuators(i)->set_torque_joint(u[i]);
    }

    // Incrementing identifier ensures actuators can reject out of time frames
    base_command.set_frame_id(base_command.frame_id() + 1);
    if (base_command.frame_id() > 65535)
        base_command.set_frame_id(0);

    for (int idx = 0; idx < 7; idx++)
    {
        base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
    }
  
    try
    {
        base_feedback = base_cyclic->Refresh(base_command, 0);

        return true;
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex exception: " << ex.what() << std::endl;
        std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        return false;
    }
    catch (runtime_error& ex2)
    {
        std::cout << "runtime error: " << ex2.what() << std::endl;
        return false;
    }
    catch(...)
    {
        std::cout << "Unknown error." << std::endl;
        return false;
    }
}


void task_control_execution(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, 
                                k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, k_api::BaseCyclic::Feedback& base_feedback, k_api::BaseCyclic::Command& base_command, Dynamics &robot,
                                TaskTrajectory& trajectory, gtsam::Pose3& base_frame, int control_frequency, std::string& dh_parameters_path,
                                std::atomic<bool>& flag){
    
    static thread_local int counter = 0;
    auto start_measure = chrono::high_resolution_clock::now();

    Eigen::VectorXd last_dq(7);
    last_dq << 0,0,0,0,0,0,0;

    Eigen::VectorXd K_d_diag(6);
    K_d_diag << 1000, 1000, 1000, 100, 100, 1;

    DHParameters_Eigen dh_eigen = createDHParamsEigen(dh_parameters_path);

    bool first_call = true;

    while(flag){

        auto start_control = chrono::high_resolution_clock::now();

        auto[p_d_world, dp_d_world, ddp_d_world] = pop_front(trajectory);
        
        // Transform poses from world_frame to base_frame
        auto[p_d, dp_d, ddp_d] = world2base(p_d_world, dp_d_world, ddp_d_world, base_frame);
        const double iteration_time = (1.0 / control_frequency) * 1000;

        robot.setBaseOrientation(base_frame.rotation().matrix());
        if(!task_impedance_control_single(base, base_cyclic, actuator_config, base_feedback, base_command, robot, p_d, dp_d, ddp_d, K_d_diag, control_frequency, first_call, last_dq, start_measure, dh_eigen)){
            throw std::runtime_error("ERROR: task_impedance_control_single");
        }

        auto end_control = chrono::high_resolution_clock::now();
        auto run_time = chrono::duration<double, milli>(end_control - start_control).count();
        int diff = 0;
        if (run_time < iteration_time) {
            auto start_delay = chrono::high_resolution_clock::now();
            diff = (iteration_time - run_time) * 1000;
            chrono::microseconds delay(diff);
            while (chrono::high_resolution_clock::now() - start_delay < delay);
        }
    
        if(trajectory.pos.size() == 1){
            counter += 1;
        }
        else{
            counter = 0;
        }

        if(counter > 1000){
            counter = 0;
            flag = false;
        }
    }
}

void updateJointInfo(  k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                       std::vector<double>& q_cur, 
                       std::shared_mutex& joint_mutex) {
    
    std::vector<double> joints(7);
    
    try {
        // Read joint positions from both arms
        k_api::BaseCyclic::Feedback base_feedback = base_cyclic->RefreshFeedback();
        for (int i = 0; i < 7; i++) {
            joints[i] = base_feedback.actuators(i).position();
        }

        // Update shared variables with mutex protection
        {
            std::unique_lock<std::shared_mutex> lock(joint_mutex);
            q_cur = joints;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error reading joint states: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
}

bool move_gripper(k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          float target_position, 
                          float proportional_gain,
                          float force_limit) {
    
    const float MINIMAL_POSITION_ERROR = 1.5f;

    k_api::BaseCyclic::Feedback base_feedback; 
    k_api::BaseCyclic::Command base_command;
    
    // Clamp target position to valid range (0-100%)
    if (target_position > 100.0f) {
        target_position = 100.0f;
    }
    if (target_position < 0.0f) {
        target_position = 0.0f;
    }

    try {
        // Get initial gripper feedback
        base_feedback = base_cyclic->RefreshFeedback();
        
        // Initialize gripper command with current position
        float gripper_initial_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
    
        // Initialize base command with current actuator positions
        base_command.clear_actuators();
        for (auto actuator : base_feedback.actuators()) {
            k_api::BaseCyclic::ActuatorCommand* actuator_command;
            actuator_command = base_command.mutable_actuators()->Add();
            actuator_command->set_position(actuator.position());
            actuator_command->set_velocity(0.0);
            actuator_command->set_torque_joint(0.0);
            actuator_command->set_command_id(0);
        }


        // Initialize interconnect command
        base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
        auto gripper_motor_command = base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
        gripper_motor_command->set_position(gripper_initial_position);
        gripper_motor_command->set_velocity(0.0);
        gripper_motor_command->set_force(force_limit);


        // Control loop
        while (true) {
            // Refresh cyclic data
            base_feedback = base_cyclic->Refresh(base_command, 0);
            float actual_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();

            float position_error = target_position - actual_position;

            // Check if target position is reached
            if (std::abs(position_error) < MINIMAL_POSITION_ERROR) {
                gripper_motor_command->set_velocity(0.0);
                base_cyclic->Refresh(base_command, 0);
                break;
            }

            // Calculate velocity using proportional control
            float velocity = proportional_gain * std::abs(position_error);
            if (velocity > 100.0f) {
                velocity = 100.0f;
            }
            
            gripper_motor_command->set_position(target_position);
            gripper_motor_command->set_velocity(velocity);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        return true;
        
    } catch(const k_api::KDetailedException& ex) {
        std::cerr << "Kortex exception during gripper movement: " << ex.what() << std::endl;
        return false;
    } catch(const std::runtime_error& ex) {
        std::cerr << "Runtime error during gripper movement: " << ex.what() << std::endl;
        return false;
    } catch(...) {
        std::cerr << "Unknown error during gripper movement" << std::endl;
        return false;
    }
}
