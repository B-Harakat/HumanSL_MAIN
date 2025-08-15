//---------------------------------------------------------
// The main function for impedance controller
//---------------------------------------------------------
// Description: The main function for the impedance controller used in the PHRI project
// Copyright: Yihan Liu 2024
//---------------------------------------------------------

#define _USE_MATH_DEFINES


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

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>


#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#include <Eigen/Dense>
#include "Jacobian.h"
#include <cmath>
#include "Fwd_kinematics.h"
#include "Dynamics.h"
#include "Controller.h"
#include "Filter.h"
#include "KinovaTrajectory.h"

#include "move.h"

#include <boost/lockfree/spsc_queue.hpp>


namespace k_api = Kinova::Api;
using namespace Jacobian;
using namespace Fwd_kinematics;
using namespace Controller;
using namespace Filter;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7
#define CONTROL_FREQUENCY 500

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = chrono::seconds{20};


// // Create an event listener that will set the promise action event to the exit value
// // Will set promise to either END or ABORT
// // Use finish_promise.get_future.get() to wait and get the value
// function<void(k_api::Base::ActionNotification)>
// create_event_listener_by_promise(promise<k_api::Base::ActionEvent>& finish_promise)
// {
//     return [&finish_promise] (k_api::Base::ActionNotification notification)
//     {
//         const auto action_event = notification.action_event();
//         switch(action_event)
//         {
//             case k_api::Base::ActionEvent::ACTION_END:
//             case k_api::Base::ActionEvent::ACTION_ABORT:
//                 finish_promise.set_value(action_event);
//                 break;
//             default:
//                 break;
//         }
//     };
// }


std::tuple<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> 
interpolateTrajectory(const Eigen::VectorXd& start_pose,
                      const Eigen::VectorXd& end_pose,
                      double duration_sec) {
    // Calculate number of points (including start and end)
    int num_points = static_cast<int>(duration_sec * CONTROL_FREQUENCY) + 1;
   
    std::vector<Eigen::VectorXd> trajectory;
    trajectory.reserve(num_points);
   
    // Handle edge case where duration is 0 or very small
    if (num_points <= 1) {
        trajectory.push_back(start_pose);
        std::vector<Eigen::VectorXd> velocity, acceleration;
        velocity.push_back(Eigen::VectorXd::Zero(6));
        acceleration.push_back(Eigen::VectorXd::Zero(6));
        return std::make_tuple(trajectory, velocity, acceleration);
    }
   
    // Pre-compute angular differences for RPY (handles wrapping)
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    };
   
    auto angularDifference = [&normalizeAngle](double end_angle, double start_angle) {
        return normalizeAngle(end_angle - start_angle);
    };
   
    double roll_diff = angularDifference(end_pose(3), start_pose(3));
    double pitch_diff = angularDifference(end_pose(4), start_pose(4));
    double yaw_diff = angularDifference(end_pose(5), start_pose(5));
   
    // Generate interpolated trajectory
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);  // Parameter from 0 to 1
       
        Eigen::VectorXd waypoint(6);
       
        // Linear interpolation for position (x, y, z)
        waypoint(0) = start_pose(0) + t * (end_pose(0) - start_pose(0));
        waypoint(1) = start_pose(1) + t * (end_pose(1) - start_pose(1));
        waypoint(2) = start_pose(2) + t * (end_pose(2) - start_pose(2));
       
        // Linear interpolation for orientation (roll, pitch, yaw) with wrapping
        waypoint(3) = normalizeAngle(start_pose(3) + t * roll_diff);
        waypoint(4) = normalizeAngle(start_pose(4) + t * pitch_diff);
        waypoint(5) = normalizeAngle(start_pose(5) + t * yaw_diff);
       
        trajectory.push_back(waypoint);
    }
   
    // Calculate time step
    double dt = duration_sec / (num_points - 1);
   
    // Calculate velocity trajectory using finite differences
    std::vector<Eigen::VectorXd> velocity;
    velocity.reserve(num_points);
   
    for (int i = 0; i < num_points; ++i) {
        Eigen::VectorXd vel(6);
        
        if (i == 0 || i == num_points - 1) {
            // Zero velocity at start and end
            vel = Eigen::VectorXd::Zero(6);
        } else {
            // Central difference for interior points
            for (int j = 0; j < 6; ++j) {
                if (j >= 3) {
                    // Handle angular velocities with wrapping
                    double angle_diff = angularDifference(trajectory[i+1](j), trajectory[i-1](j));
                    vel(j) = angle_diff / (2.0 * dt);
                } else {
                    // Linear velocities
                    vel(j) = (trajectory[i+1](j) - trajectory[i-1](j)) / (2.0 * dt);
                }
            }
        }
        velocity.push_back(vel);
    }
   
    // Calculate acceleration trajectory using finite differences
    std::vector<Eigen::VectorXd> acceleration;
    acceleration.reserve(num_points);
   
    for (int i = 0; i < num_points; ++i) {
        Eigen::VectorXd acc(6);
        
        if (i == 0 || i == num_points - 1) {
            // Zero acceleration at start and end
            acc = Eigen::VectorXd::Zero(6);
        } else {
            // Central difference for interior points
            for (int j = 0; j < 6; ++j) {
                if (j >= 3) {
                    // Handle angular accelerations with wrapping
                    double vel_diff = angularDifference(velocity[i+1](j), velocity[i-1](j));
                    acc(j) = vel_diff / (2.0 * dt);
                } else {
                    // Linear accelerations
                    acc(j) = (velocity[i+1](j) - velocity[i-1](j)) / (2.0 * dt);
                }
            }
        }
        acceleration.push_back(acc);
    }
   
    return std::make_tuple(trajectory, velocity, acceleration);
}

//------------------------------------------
// Function of high-level movement
//-----------------------------------------
// 2 inputs:
// base: communication variables
// q_d: desired joint angular position
//-----------------------------------------
void Move_high_level(k_api::Base::BaseClient* base, Eigen::VectorXd q_d)
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


//----------------------------------------------------
// Main function of impedance control
//----------------------------------------------------
int main(int argc, char **argv)
{
    Dynamics robot("../config/GEN3_With_GRIPPER_DYNAMICS.urdf");

    Eigen::VectorXd K_d_diag(6), p_f(6), q_0(7);
    // Define the desired position, velocity, acceleration and stiffness here.

    q_0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    auto [p_0, _] = Fwd_kinematics::forward(q_0);
    p_f << 0.2, 0.0, 1.0, 0.0, 0.0, 0.0;
    K_d_diag << 1200, 1200, 1200, 50, 50, 1;

    std::vector<Eigen::VectorXd> p_d, dp_d, ddp_d;

    tie(p_d,dp_d, ddp_d) = interpolateTrajectory(p_0,p_f,3);
    std::deque<Eigen::VectorXd> pos(p_d.begin(), p_d.end());
    std::deque<Eigen::VectorXd> vel(dp_d.begin(), dp_d.end());
    std::deque<Eigen::VectorXd> acc(ddp_d.begin(), ddp_d.end());

    TaskTrajectory task_trajectory;
    task_trajectory.pos = pos;
    task_trajectory.vel = vel;
    task_trajectory.acc = acc;

    gtsam::Pose3 base_frame;

    // Experiment loop for different sets
    this_thread::sleep_for(chrono::milliseconds(1000));

    // Create API objects
    auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };

    cout << "Creating transport objects" << endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    cout << "Creating transport real time objects" << endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    cout << "Creating sessions for communication" << endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    cout << "Sessions created" << endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    
    // Clear any faults
    try {
        base->ClearFaults();
    } catch(...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    // Move to the initial configuration
    Eigen::VectorXd q_d(7);
    q_d << 0, 0, 0, 0, 0, 0, 0;
    q_d = q_d * (180 / M_PI);
    Move_high_level(base, q_d);

    std::cout << "Press Enter to execute trajectory through Kinova API" << std::endl;
    std::cin.get(); // Wait for user input before executing trajectory

    // Atomic flag to control the loop termination for drawScene
    atomic<bool> is_impedance_control_running(true);

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;
    std::cout << "0\n";
    std::vector<float> commands;
    std::cout << "1\n";
    auto servoing_mode = k_api::Base::ServoingModeInformation();
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    base_feedback = base_cyclic->RefreshFeedback();
    std::cout << "2\n";

    // Initialize each actuator to their current position
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        
        commands.push_back(base_feedback.actuators(i).position());
        std::cout << "1\n";
        // Save the current actuator position, to avoid a following error
        base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
    }

    base_feedback = base_cyclic->Refresh(base_command);

    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    for (int id = 1; id < ACTUATOR_COUNT+1; id++)
    {
        actuator_config->SetControlMode(control_mode_message, id);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    std::thread impedance_thread([&]() {
            std::cout << base_frame << "\n";
            task_control_execution(base,base_cyclic, actuator_config, base_feedback, base_command, robot, task_trajectory, base_frame, CONTROL_FREQUENCY,  std::ref(is_impedance_control_running));
        });

    // Thread for impedance_control function
    // thread impedance_thread([&]() {
    //     auto isOk = task_impedance_control(base, base_cyclic, actuator_config, robot, p_d, dp_d, ddp_d, K_d_diag);
    //     if (!isOk) {
    //         cout << "There has been an unexpected error in impedance_control() function." << endl;
    //     }
    //     // Signal that the impedance control has finished
    //     is_impedance_control_running = false;
    // });



    // Join the threads back to the main thread
    impedance_thread.join();

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}