/*
* Kinova Gen3 Trajectory Executor - Implementation
* Executes a densified 1000Hz trajectory on Kinova Gen3 7DoF arm
*/

#define _USE_MATH_DEFINES

#include "KinovaTrajectory.h"

using namespace Jacobian;
using namespace Fwd_kinematics;
using namespace Controller;
using namespace Filter;

constexpr auto TIMEOUT_PROMISE_DURATION = chrono::seconds{20};


// Internal helper function - Get current time in microseconds
static int64_t GetTickUs() {
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

// Internal helper function - Create an event listener that will set the promise action event to the exit value
// Will set to either END or ABORT
static std::function<void(k_api::Base::ActionNotification)> 
    create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

std::vector<double> get_current_joint_pos(k_api::BaseCyclic::BaseCyclicClient* base_cyclic) {
    std::vector<double> q(7);
    k_api::BaseCyclic::Feedback base_feedback;
    base_feedback = base_cyclic->RefreshFeedback();
    for (int i = 0; i < 7; i++){
        q[i] = base_feedback.actuators(i).position();
    }

    return q;
}

std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

void move_single_level(k_api::Base::BaseClient* base, std::vector<double> q_d)
{

    auto action = k_api::Base::Action();

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

    // Arm straight up
    for (size_t i = 0; i < actuator_count.count(); ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(q_d[i]);
    }

    promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
    );

    base->ExecuteAction(action);

    const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if(status != future_status::ready)
    {
        cout << "Timeout on action notification wait" << endl;
    }
    const auto promise_event = finish_future.get();
}

// Main trajectory execution function
bool joint_position_control(k_api::Base::BaseClient* base,
                            k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                            const std::vector<VectorXd>& trajectory) {

    std::cout << "Starting trajectory execution..." << std::endl;
    std::cout << "Trajectory length: " << trajectory.size() << " points" << std::endl;
    
    int num_joints = trajectory[0].size();

    // Move to start position
    std::vector<double> trajectory_vec(trajectory[0].data(), trajectory[0].data() + trajectory[0].size());
    move_single_level(base, trajectory_vec);

    // Clear any faults
    try {
        base->ClearFaults();
    } catch(...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }
    
    // Set to low-level servoing mode
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    
    // Initialize cyclic communication
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;
    
    base_feedback = base_cyclic->RefreshFeedback();
    
    // Initialize command with current positions
    for(int i = 0; i < num_joints; i++) {
        base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
    }
    
    // Send initial frame
    base_feedback = base_cyclic->Refresh(base_command);
    
    std::cout << "Starting trajectory execution..." << std::endl;
    
    // Real-time execution loop - using official Kinova timing pattern
    int trajectory_idx = 0;
    int64_t now = 0;
    int64_t last = 0;
    
    while(trajectory_idx < trajectory.size()) {
        now = GetTickUs();
        
        // Check if 1ms has passed (1000 microseconds) - using official pattern
        if(now - last > 1000) {
            
            // Set joint positions from trajectory
            for(int i = 0; i < num_joints; i++) {
                base_command.mutable_actuators(i)->set_position(trajectory[trajectory_idx][i]);
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
            
            try {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            } catch(k_api::KDetailedException& ex) {
                std::cout << "Kortex exception during trajectory execution: " << ex.what() << std::endl;
                break;
            } catch(std::runtime_error& ex) {
                std::cout << "Runtime error during trajectory execution: " << ex.what() << std::endl;
                break;
            } catch(...) {
                std::cout << "Unknown error during trajectory execution" << std::endl;
                break;
            }
            
            trajectory_idx++;
            last = GetTickUs();  // CRITICAL: Set AFTER communication, not before
            
            // Progress indicator
            if(trajectory_idx % 100 == 0) {
                double progress = (double)trajectory_idx / trajectory.size() * 100.0;
                std::cout << "Progress: " << std::fixed << std::setprecision(1) << progress << "%" << std::endl;
            }
        }
    }
    
    std::cout << "Trajectory execution completed!" << std::endl;
    
    // Return to high-level servoing
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    
    return true;
}

bool task_impedance_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                       k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                       std::vector<VectorXd>& p_d, std::vector<VectorXd>& dp_d, std::vector<VectorXd>& ddp_d, VectorXd& K_d_diag) {
    bool return_status = true;

    // Clearing faults
    try {
        base->ClearFaults();
    }
    catch (...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;

    // KINOVA feedback (joint space variables)
    VectorXd q(7), dq(7), last_dq(7), ddq(7), torque(7), current(7);

    // Task space variables
    VectorXd u(7), p(6), dp(6), ddp(6);
    MatrixXd T_B7(4,4);  // Rotation matrix

    // Define the D_n and K_n for nullspace
    VectorXd K_n_diag(7);
    K_n_diag << 4, 4, 4, 2, 2, 2, 2;

    // Time for one control iterative
    double dt = 1.0 / CONTROL_FREQUENCY;
    const double iteration_time = (1.0 / CONTROL_FREQUENCY) * 1000;

    // Buffer to save the last velocity
    last_dq << 0,0,0,0,0,0,0;

    std::cout << "Initializing the arm for torque control ^^!" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        std::cout << "1" <<std::endl;

        // Initialize each actuator to their current position
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        std::cout << "2" <<std::endl;

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set actuatorS in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }
        std::cout << "3" <<std::endl;
        this_thread::sleep_for(chrono::milliseconds(40));

        std::cout << "4" <<std::endl;

        // Clock to record the time period between two control/measure loop
        auto start_measure = chrono::high_resolution_clock::now();
        auto start_control = chrono::high_resolution_clock::now();

        std::cout << "5" <<std::endl;
        // Control loop
        for(size_t k = 0; k < p_d.size(); k++) // This is an example code
        {

            // KINOVA Feedback: Obtaining gen3 ACTUAL joint positions, velocities, torques & current
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                q[i] = (M_PI/180)*base_feedback.actuators(i).position();
                dq[i] = (M_PI/180)*base_feedback.actuators(i).velocity();
                torque[i] = base_feedback.actuators(i).torque();
                current[i] = base_feedback.actuators(i).current_motor();
            }
            std::cout << "6" <<std::endl;

            // Apply the forward kinematics
            std::tie(p, T_B7) = forward(q);

            std::cout << "7" <<std::endl;
            // initilize the controller and filter
            if(timer_count == 0){
                Vector<double, 3> pos = p.head(3);
                std::cout << "8" <<std::endl;
                ini_controller(pos, T_B7);
                std::cout << "9" <<std::endl;
                ini_butterworth();
            }
            std::cout << "10" <<std::endl;

            // Compute the joint accelerations
            auto end_measure = chrono::high_resolution_clock::now();
            chrono::duration<double> measure_dur = end_measure - start_measure;
            dt = measure_dur.count();
            if (timer_count == 0){
                dt = 1.0 / CONTROL_FREQUENCY;
            }
            ddq = (dq - last_dq) / dt;
            start_measure = end_measure;
            last_dq = dq;


            VectorXd acc_factor(6);
            // Impedance controller
            std::tie(u, dp, ddp, acc_factor) = task_impedance_controller(robot, q, dq, ddq, T_B7, p_d[k], dp_d[k],
                                                               ddp_d[k], K_d_diag, K_n_diag,CONTROL_FREQUENCY, dt);
            std::cout << "11" <<std::endl;
            // Set the control frequency
            auto end_control = chrono::high_resolution_clock::now();
            auto run_time = chrono::duration<double, milli>(end_control - start_control).count();
            int diff = 0;
            if (run_time < iteration_time) { // if real control loop time < desired iteration time
                auto start_delay = chrono::high_resolution_clock::now();
                diff = (iteration_time - run_time) * 1000;
                chrono::microseconds delay(diff);
                while (chrono::high_resolution_clock::now() - start_delay < delay); // delay the time difference
            }
            auto last_control_time = start_control;
            start_control = chrono::high_resolution_clock::now();
            auto control_duration = chrono::duration<double, milli>(start_control - last_control_time).count();

            for (int i = 0; i < ACTUATOR_COUNT; i++)
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

            for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
            }

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;

                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (runtime_error& ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch(...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            timer_count++;
        }

        while(1){

            // KINOVA Feedback: Obtaining gen3 ACTUAL joint positions, velocities, torques & current
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                q[i] = (M_PI/180)*base_feedback.actuators(i).position();
                dq[i] = (M_PI/180)*base_feedback.actuators(i).velocity();
                torque[i] = base_feedback.actuators(i).torque();
                current[i] = base_feedback.actuators(i).current_motor();
            }
            std::cout << "6" <<std::endl;

            // Apply the forward kinematics
            std::tie(p, T_B7) = forward(q);
            std::cout << "7" <<std::endl;
            // initilize the controller and filter
            if(timer_count == 0){
                Vector<double, 3> pos = p.head(3);
                std::cout << "8" <<std::endl;
                ini_controller(pos, T_B7);
                std::cout << "9" <<std::endl;
                ini_butterworth();
            }
            std::cout << "10" <<std::endl;

            // Compute the joint accelerations
            auto end_measure = chrono::high_resolution_clock::now();
            chrono::duration<double> measure_dur = end_measure - start_measure;
            dt = measure_dur.count();
            if (timer_count == 0){
                dt = 1.0 / CONTROL_FREQUENCY;
            }
            ddq = (dq - last_dq) / dt;
            start_measure = end_measure;
            last_dq = dq;


            VectorXd acc_factor(6);
            int last_idx = p_d.size()-1;
            // Impedance controller
            std::tie(u, dp, ddp, acc_factor) = task_impedance_controller(robot, q, dq, ddq, T_B7, p_d[last_idx], dp_d[last_idx],
                                                               ddp_d[last_idx], K_d_diag, K_n_diag,CONTROL_FREQUENCY, dt);
            std::cout << "11" <<std::endl;
            // Set the control frequency
            auto end_control = chrono::high_resolution_clock::now();
            auto run_time = chrono::duration<double, milli>(end_control - start_control).count();
            int diff = 0;
            if (run_time < iteration_time) { // if real control loop time < desired iteration time
                auto start_delay = chrono::high_resolution_clock::now();
                diff = (iteration_time - run_time) * 1000;
                chrono::microseconds delay(diff);
                while (chrono::high_resolution_clock::now() - start_delay < delay); // delay the time difference
            }
            auto last_control_time = start_control;
            start_control = chrono::high_resolution_clock::now();
            auto control_duration = chrono::duration<double, milli>(start_control - last_control_time).count();

            for (int i = 0; i < ACTUATOR_COUNT; i++)
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

            for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
            }

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;

                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (runtime_error& ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch(...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            timer_count++;
        }


        std::cout << "Torque control ^^ completed" << std::endl;

        // Set actuators back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }

        std::cout << "Torque control ^^ clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    this_thread::sleep_for(chrono::milliseconds(2000));

    return return_status;
}


bool joint_impedance_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                            k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                            std::vector<VectorXd>& q_d, std::vector<VectorXd>& dq_d, std::vector<VectorXd>& ddq_d, 
                            VectorXd& K_joint_diag) {
    bool return_status = true;

    // Clearing faults
    try {
        base->ClearFaults();
    }
    catch (...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    vector<float> commands;
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    int timer_count = 0;

    // KINOVA feedback (joint space variables)
    VectorXd q(7), dq(7), last_dq(7), ddq(7), torque(7), current(7);

    // Joint space control output
    VectorXd u(7);

    // Time for one control iteration
    double dt = 1.0 / CONTROL_FREQUENCY;
    const double iteration_time = (1.0 / CONTROL_FREQUENCY) * 1000;

    // Buffer to save the last velocity
    last_dq << 0,0,0,0,0,0,0;

    std::cout << "Initializing the arm for joint space torque control!" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set actuators in torque mode
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }
        
        this_thread::sleep_for(chrono::milliseconds(40));

        // Clock to record the time period between control loops
        auto start_measure = chrono::high_resolution_clock::now();
        auto start_control = chrono::high_resolution_clock::now();

        // Control loop - execute trajectory
        for(size_t k = 0; k < q_d.size(); k++) 
        {
            // KINOVA Feedback: Get actual joint positions, velocities, torques & current
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                q[i] = (M_PI/180) * base_feedback.actuators(i).position();
                dq[i] = (M_PI/180) * base_feedback.actuators(i).velocity();
                torque[i] = base_feedback.actuators(i).torque();
                current[i] = base_feedback.actuators(i).current_motor();
            }

            // Compute the joint accelerations
            auto end_measure = chrono::high_resolution_clock::now();
            chrono::duration<double> measure_dur = end_measure - start_measure;
            dt = measure_dur.count();
            if (timer_count == 0){
                dt = 1.0 / CONTROL_FREQUENCY;
            }
            ddq = (dq - last_dq) / dt;
            start_measure = end_measure;
            last_dq = dq;

            // Joint space impedance controller
            std::tie(u) = joint_impedance_controller(robot, q, dq, ddq, q_d[k], dq_d[k], ddq_d[k], 
                                               K_joint_diag, CONTROL_FREQUENCY, dt);

            // Set the control frequency
            auto end_control = chrono::high_resolution_clock::now();
            auto run_time = chrono::duration<double, milli>(end_control - start_control).count();
            int diff = 0;
            if (run_time < iteration_time) {
                auto start_delay = chrono::high_resolution_clock::now();
                diff = (iteration_time - run_time) * 1000;
                chrono::microseconds delay(diff);
                while (chrono::high_resolution_clock::now() - start_delay < delay);
            }
            auto last_control_time = start_control;
            start_control = chrono::high_resolution_clock::now();

            // Send commands to robot
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                base_command.mutable_actuators(i)->set_torque_joint(u[i]);
            }

            // Increment frame ID
            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535)
                base_command.set_frame_id(0);

            for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
            }

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;
                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (runtime_error& ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch(...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            timer_count++;
        }

        // Hold at final position
        while(1){
            // Get current joint states
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                q[i] = (M_PI/180) * base_feedback.actuators(i).position();
                dq[i] = (M_PI/180) * base_feedback.actuators(i).velocity();
                torque[i] = base_feedback.actuators(i).torque();
                current[i] = base_feedback.actuators(i).current_motor();
            }

            // Compute joint accelerations
            auto end_measure = chrono::high_resolution_clock::now();
            chrono::duration<double> measure_dur = end_measure - start_measure;
            dt = measure_dur.count();
            ddq = (dq - last_dq) / dt;
            start_measure = end_measure;
            last_dq = dq;

            // Use final trajectory point for steady-state
            int last_idx = q_d.size()-1;
            std::tie(u) = joint_impedance_controller(robot, q, dq, ddq, q_d[last_idx], dq_d[last_idx], ddq_d[last_idx], 
                                               K_joint_diag, CONTROL_FREQUENCY, dt);

            // Control frequency management
            auto end_control = chrono::high_resolution_clock::now();
            auto run_time = chrono::duration<double, milli>(end_control - start_control).count();
            if (run_time < iteration_time) {
                auto start_delay = chrono::high_resolution_clock::now();
                int diff = (iteration_time - run_time) * 1000;
                chrono::microseconds delay(diff);
                while (chrono::high_resolution_clock::now() - start_delay < delay);
            }
            start_control = chrono::high_resolution_clock::now();

            // Send commands
            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                base_command.mutable_actuators(i)->set_torque_joint(u[i]);
            }

            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535)
                base_command.set_frame_id(0);

            for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
            }

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception: " << ex.what() << std::endl;
                std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
            }
            catch (runtime_error& ex2)
            {
                std::cout << "runtime error: " << ex2.what() << std::endl;
            }
            catch(...)
            {
                std::cout << "Unknown error." << std::endl;
            }

            timer_count++;
        }

        std::cout << "Joint space torque control completed" << std::endl;

        // Set actuators back to position mode
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }

        std::cout << "Joint space torque control clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set servoing mode back to single level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    this_thread::sleep_for(chrono::milliseconds(100));

    return return_status;
}


// Example usage function
bool executeTrajectory(const std::string& ip_address, std::vector<VectorXd>& pos, std::vector<VectorXd>& vel, std::vector<VectorXd>& acc, std::string control_mode) {
    
    // Create API objects 
    auto error_callback = [](k_api::KError err){ 
        std::cout << "API Error: " << err.toString() << std::endl; 
    };
    
    // TCP connection for configuration
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip_address, PORT);
    
    // UDP connection for real-time control
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(ip_address, PORT_REAL_TIME);
    
    // Session setup
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);
    create_session_info.set_connection_inactivity_timeout(2000);
    
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
 
    // Create service clients
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    
    // Execute trajectory
    Dynamics robot("../config/GEN3_With_GRIPPER_DYNAMICS.urdf");
    std::cout << "KinovaTrajectory - executeTrajectory() loaded with static robot dynamics, remember to update this" << std::endl;
    bool success; 
    if(control_mode == "joint_position"){
            success = joint_position_control(base, base_cyclic, pos);
    }
    else if (control_mode == "joint_impedance"){
            VectorXd K_joint_diag(7);
            K_joint_diag << 100, 100, 80, 80, 50, 50, 50; 
            success = joint_impedance_control(base,base_cyclic,actuator_config,robot,pos,vel,acc,K_joint_diag);
    }
    else if (control_mode == "task_impedance"){
            VectorXd K_task_diag(6);
            K_task_diag << 1200, 1200, 1200, 50, 50, 1;
            success = task_impedance_control(base,base_cyclic,actuator_config,robot,pos,vel,acc,K_task_diag);
     
    }
    else{
        throw std::invalid_argument("Invalid control mode");
    }
    
    // Cleanup
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();
    
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();
    
    // Delete objects
    delete base;
    delete base_cyclic;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
    
    return success;
}

