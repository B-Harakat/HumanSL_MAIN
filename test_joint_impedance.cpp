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


#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <cmath>

std::tuple<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> 
interpolateTrajectory(const Eigen::VectorXd& start_pos, 
                      const Eigen::VectorXd& end_pos, 
                      double duration) {
    
    // Calculate number of waypoints
    int num_points = static_cast<int>(duration * CONTROL_FREQUENCY) + 1;
    
    // Initialize output vectors
    std::vector<Eigen::VectorXd> positions, velocities, accelerations;
    positions.reserve(num_points);
    velocities.reserve(num_points);
    accelerations.reserve(num_points);
    
    int num_joints = start_pos.size();
    
    // Calculate total displacement for each joint
    Eigen::VectorXd total_displacement = end_pos - start_pos;
    
    // Generate linear trajectory points
    for (int point = 0; point < num_points; ++point) {
        double t = static_cast<double>(point) / (num_points - 1);  // Parameter from 0 to 1
        
        // Linear interpolation for position
        Eigen::VectorXd pos = start_pos + t * total_displacement;
        positions.push_back(pos);
        
        // Constant velocity (except at endpoints)
        Eigen::VectorXd vel(num_joints);
        if (point == 0 || point == num_points - 1) {
            vel = Eigen::VectorXd::Zero(num_joints);  // Zero velocity at start and end
        } else {
            vel = total_displacement / duration;  // Constant velocity
        }
        velocities.push_back(vel); 
        
        // Zero acceleration (linear motion)
        accelerations.push_back(Eigen::VectorXd::Zero(num_joints));
    }
    
    return std::make_tuple(positions, velocities, accelerations);
}

//------------------------------------------
// Function of high-level movement
//-----------------------------------------
// 2 inputs:
// base: communication variables
// q_d: desired joint angular position
//-----------------------------------------
void Move_high_level(k_api::Base::BaseClient* base, VectorXd q_d)
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

    VectorXd K_d_diag(7), q_f(7), q_0(7);
    // Define the desired position, velocity, acceleration and stiffness here.

    q_0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    q_f << 0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    K_d_diag << 1000, 1000, 1000, 1000, 500, 200, 200;

    JointTrajectory joint_trajectory;
    std::vector<VectorXd> q_d, dq_d, ddq_d;

    tie(q_d,dq_d, ddq_d) = interpolateTrajectory(q_0,q_f,3);
    std::deque<VectorXd> pos(q_d.begin(), q_d.end());
    std::deque<VectorXd> vel(dq_d.begin(), dq_d.end());
    std::deque<VectorXd> acc(ddq_d.begin(), ddq_d.end());
    joint_trajectory.pos = pos;
    joint_trajectory.vel = vel;
    joint_trajectory.acc = acc;

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
    VectorXd q_init(7);
    q_init << 0, 0, 0, 0, 0, 0, 0;
    Move_high_level(base, q_init);

    // Atomic flag to control the loop termination for drawScene
    atomic<bool> is_impedance_control_running(true);

    // Thread for impedance_control function
    // thread impedance_thread([&]() {
    //     auto isOk = joint_impedance_control(base, base_cyclic, actuator_config, robot, q_d, dq_d, ddq_d, K_d_diag);
    //     if (!isOk) {
    //         cout << "There has been an unexpected error in impedance_control() function." << endl;
    //     }
    //     // Signal that the impedance control has finished
    //     is_impedance_control_running = false;
    // });

    std::thread impedance_thread([&]() {
        joint_control_execution(base,base_cyclic, actuator_config, base_feedback, base_command, robot, joint_trajectory, CONTROL_FREQUENCY, std::ref(is_impedance_control_running));
    });

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