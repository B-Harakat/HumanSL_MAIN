#define _USE_MATH_DEFINES

#include "ImpedanceControl.h"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include <thread>
#include <iomanip>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <tuple>


#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>


namespace k_api = Kinova::Api;
using namespace Jacobian;
using namespace Fwd_kinematics;
using namespace Controller;
using namespace Filter;

#define CONTROL_FREQUENCY 400

bool task_impedance_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                       k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, Dynamics &robot,
                       std::vector<VectorXd>& p_d, std::vector<VectorXd>& dp_d, std::vector<VectorXd>& ddp_d, VectorXd& K_d_diag) {
    bool return_status = true;

    // Clearing faults
    try {
        base->ClearFaults();
    }
    catch (...) {
        cout << "Unable to clear robot faults" << endl;
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

    cout << "Initializing the arm for torque control ^^!" << endl;
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
            tie(p, T_B7) = forward(q);
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
            tie(u, dp, ddp, acc_factor) = task_impedance_controller(robot, q, dq, ddq, T_B7, p_d[k], dp_d[k],
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
                cout << "Kortex exception: " << ex.what() << endl;

                cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << endl;
            }
            catch (runtime_error& ex2)
            {
                cout << "runtime error: " << ex2.what() << endl;
            }
            catch(...)
            {
                cout << "Unknown error." << endl;
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
            tie(p, T_B7) = forward(q);
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
            tie(u, dp, ddp, acc_factor) = task_impedance_controller(robot, q, dq, ddq, T_B7, p_d[last_idx], dp_d[last_idx],
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
                cout << "Kortex exception: " << ex.what() << endl;

                cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << endl;
            }
            catch (runtime_error& ex2)
            {
                cout << "runtime error: " << ex2.what() << endl;
            }
            catch(...)
            {
                cout << "Unknown error." << endl;
            }

            timer_count++;
        }


        cout << "Torque control ^^ completed" << endl;

        // Set actuators back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }

        cout << "Torque control ^^ clean exit" << endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        cout << "API error: " << ex.what() << endl;
        return_status = false;
    }
    catch (runtime_error& ex2)
    {
        cout << "Error: " << ex2.what() << endl;
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
        cout << "Unable to clear robot faults" << endl;
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

    cout << "Initializing the arm for joint space torque control!" << endl;
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
            tie(u) = joint_impedance_controller(robot, q, dq, ddq, q_d[k], dq_d[k], ddq_d[k], 
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
                cout << "Kortex exception: " << ex.what() << endl;
                cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << endl;
            }
            catch (runtime_error& ex2)
            {
                cout << "runtime error: " << ex2.what() << endl;
            }
            catch(...)
            {
                cout << "Unknown error." << endl;
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
            tie(u) = joint_impedance_controller(robot, q, dq, ddq, q_d[last_idx], dq_d[last_idx], ddq_d[last_idx], 
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
                cout << "Kortex exception: " << ex.what() << endl;
                cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << endl;
            }
            catch (runtime_error& ex2)
            {
                cout << "runtime error: " << ex2.what() << endl;
            }
            catch(...)
            {
                cout << "Unknown error." << endl;
            }

            timer_count++;
        }

        cout << "Joint space torque control completed" << endl;

        // Set actuators back to position mode
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }

        cout << "Joint space torque control clean exit" << endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        cout << "API error: " << ex.what() << endl;
        return_status = false;
    }
    catch (runtime_error& ex2)
    {
        cout << "Error: " << ex2.what() << endl;
        return_status = false;
    }

    // Set servoing mode back to single level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    this_thread::sleep_for(chrono::milliseconds(100));

    return return_status;
}
