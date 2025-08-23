#include "TrajectoryInitiation.h"
#include <cmath>
#include <random>
#include "quik_solveIK.h"
#include "analytical_ik.h"


InitializeTrajectory::InitializeTrajectory(DHParameters dh_params) : dh_params_(dh_params)
{}


bool InitializeTrajectory::solveIK(const gtsam::Pose3& target_pose, 
                    const gtsam::Pose3& base_pose,
                    const gtsam::Vector& seed_config,
                    gtsam::Vector& result_config,
                    int max_attempts,
                    double guess_range) {
    
    // === DAMPED LEAST SQUARES IK SOLVER ===
    // Convert gtsam types to Eigen for DLS solver
    Eigen::Matrix4d target_transform = target_pose.matrix();
    Eigen::Matrix4d base_transform = base_pose.matrix();
    
    Eigen::Vector<double, 7> seed_eigen;
    for(int i = 0; i < 7; i++) {
        seed_eigen(i) = seed_config(i);
    }
    
    // Use Damped Least Squares IK solver with multiple random seeds
    auto solution = analytical_ik::AnalyticalIKSolver::solveBestIK(
        target_transform, base_transform, seed_eigen, max_attempts);
    
    if(solution.is_valid) {
        // Convert result to gtsam::Vector
        result_config = gtsam::Vector(7);
        for(int i = 0; i < 7; i++) {
            result_config(i) = solution.joint_angles(i);
        }
        return true;
    } else {
        return false;
    }  
}

bool InitializeTrajectory::solveQuik(const gtsam::Pose3& target_pose, 
                    const gtsam::Pose3& base_pose,
                    const gtsam::Vector& seed_config,
                    gtsam::Vector& result_config,
                    int max_attempts,
                    double guess_range) {
    // === QUIK SOLVER  ===
    // Define manipulator (same as in quik_solveIK.h)
    auto Kinova_Gen3 = std::make_shared<quik::Robot<7>>(
        (Eigen::Matrix<double, 7, 4>() << 
            0.0,      -M_PI/2,   0.2848,      0,
            0.0,       M_PI/2,  -0.0118,      M_PI,
            0.0,       M_PI/2,  -0.4208,      M_PI,
            0.0,       M_PI/2,  -0.0128,      M_PI,
            0.0,       M_PI/2,  -0.3143,      M_PI,
            0.0,       M_PI/2,   0.0,         M_PI,
            0.0,       M_PI,    -0.3074,      M_PI).finished(),
                        
        (Eigen::Vector<quik::JOINTTYPE_t,7>() << 
            quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, 
            quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, 
            quik::JOINT_REVOLUTE
        ).finished(),

        (Eigen::Vector<double,7>() << 1, 1, 1, 1, 1, 1, 1).finished(),
        base_pose.matrix(),
        Eigen::Matrix4d::Identity()
    );

    quik::IKSolver<7> IKS(
        Kinova_Gen3,
        200, // max iterations
        quik::ALGORITHM_QUIK,
        1e-6, // exit tolerance
        1e-14, // minimum step tolerance
        0.05, // improvement tolerance
        10, // max consecutive grad fails
        80, // max grad fails
        1e-10, // lambda2
        0.34, // max linear error step
        1 // max angular error step
    );

    Eigen::Matrix4d target_transform = target_pose.matrix();
    
    Eigen::Vector<double,7> best_solution;
    double best_quality = std::numeric_limits<double>::max();
    bool found_valid_solution = false;

    Eigen::Vector<double,7> Q0_init;
    for(int i = 0; i < 7; i++) {
                Q0_init(i) = seed_config(i);
            }
    
    for(int attempt = 0; attempt < max_attempts; attempt++) {
        Eigen::Vector<double,7> Q0;
        try {
            if(attempt == 0) {
                // First attempt: use provided seed
                for(int i = 0; i < 7; i++) {
                    Q0(i) = seed_config(i);
                }
            } else {
                // Subsequent attempts: random initial guesses
                Q0 = generateRandomInitialGuess();
            }
            
            Eigen::Vector<double,7> Q_star;
            Eigen::Vector<double,6> e_star;
            int iter;
            quik::BREAKREASON_t breakReason;
            
            IKS.IK(target_transform, Q0, Q_star, e_star, iter, breakReason);
            
            wrapAngles(Q_star,Q0_init);
            double quality = computeSolutionQuality(e_star, Q_star, Q0_init);
            
            if(quality < best_quality) {
                best_quality = quality;
                best_solution = Q_star;
                
                if(isWithinJointLimits(Q_star) && breakReason == quik::BREAKREASON_TOLERANCE) {
                    found_valid_solution = true;
                    break;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "IK error: " << e.what() << std::endl;
        }

    }
    
    std::cout << "found_valid: " << found_valid_solution << "  best quality: " << best_quality << "\n";
    // Convert result to gtsam::Vector
    if(found_valid_solution || best_quality < 1e-1) { // Accept "good enough" solutions
        result_config = gtsam::Vector(7);
        for(int i = 0; i < 7; i++) {
            result_config(i) = best_solution(i);
        }
        return true;
    }
    
    return false;
    
}


void InitializeTrajectory::wrapAngles(gtsam::Vector& angles, const gtsam::Vector& reference) {
    assert(angles.size() == reference.size());
    
    for(size_t i = 0; i < angles.size(); i++) {
        // First wrap the reference angle to [-π, π]
        double wrapped_ref = std::remainder(reference(i), 2*M_PI);
        
        // Then wrap the target angle relative to the wrapped reference
        double diff = angles(i) - wrapped_ref;
        diff = std::remainder(diff, 2*M_PI);
        angles(i) = wrapped_ref + diff;
    }
}

void InitializeTrajectory::wrapAngles(Eigen::Vector<double,7>& angles, const Eigen::Vector<double,7>& reference) {
    for(int i = 0; i < 7; i++) {
        // First wrap the reference angle to [-π, π]
        double wrapped_ref = std::remainder(reference(i), 2*M_PI);
        
        // Then wrap the target angle relative to the wrapped reference
        double diff = angles(i) - wrapped_ref;
        diff = std::remainder(diff, 2*M_PI);
        angles(i) = wrapped_ref + diff;
    }
}


gtsam::Values InitializeTrajectory::initJointTrajectoryFromTarget(
                                const gtsam::Vector& start_conf,
                                const gtsam::Pose3& end_pose,
                                const gtsam::Pose3& base_pose,
                                const size_t total_time_step) {
    
    std::vector<gtsam::Vector> end_confs;
    int best_idx = -1;
    double best_quality = 1e20;
    int target_count = 0;
        
    int success_count = 0;
    for(int i = 0; i < 10; i++){
        gtsam::Vector end_conf; 
        std::cout << "  Attempt " << (i+1) << "/10: ";
        if(solveIK(end_pose, base_pose, start_conf, end_conf, 100, 0.25)){
            wrapAngles(end_conf, start_conf);
            end_confs.push_back(end_conf);
            std::cout << "SUCCESS\n";
            success_count++;
            double magnitude = (end_conf - start_conf).norm();

            if(magnitude < best_quality){
                best_quality = magnitude;
                best_idx = end_confs.size()-1;
            }
        } else {
            std::cout << "FAILED\n";
        }
        
    }
    // std::cout << "SSSSSSSStart Joint Conf: ";
    // for (auto& f : start_conf){
    //     std::cout << f <<", ";
    // }
    // std::cout << "\n";
    // std::cout << "BBBBBBBBBBBBBBBBest Joint Conf: ";
    // for (auto& d : end_confs[best_idx]){
    //     std::cout << d <<", ";
    // }
   
    
    // std::cout << "\nTotal targets tried: " << target_count << ", solutions found: " << end_confs.size() << "\n";
    
    // Removed old diagnostic code - using new tube-aware approach
    
    if(end_confs.size() > 0){
        gtsam::Values init_values;

        for (size_t i = 0; i <= total_time_step; i++) {
            gtsam::Vector conf;
            if (i == 0)
            conf = start_conf;
            else if (i == total_time_step) 
            conf = end_confs[best_idx];
            else
            conf =
                static_cast<double>(i) / static_cast<double>(total_time_step) * end_confs[best_idx] +
                (1.0 - static_cast<double>(i) / static_cast<double>(total_time_step)) *
                    start_conf;

            init_values.insert(gtsam::Symbol('x', i), conf);
        }
        // init vel as avg vel
        gtsam::Vector avg_vel = (end_confs[best_idx] - start_conf) / static_cast<double>(total_time_step);
        for (size_t i = 0; i <= total_time_step; i++)
            init_values.insert(gtsam::Symbol('v', i), avg_vel);

        // std::cout << "start conf: ";
        // for (auto& data : start_conf){
        //     std::cout << data <<" ";
        // }
        // std::cout <<"\n";

        // std::cout << "end pose: " << end_pose <<"\n";
        // std::cout << "base_pose: " << base_pose <<"\n";

        return init_values;
        }
    
    else {

        // std::cout << "start conf: ";
        // for (auto& data : start_conf){
        //     std::cout << data <<" ";
        // }
        // std::cout <<"\n";

        // std::cout << "end pose: " << end_pose <<"\n";
        // std::cout << "base_pose: " << base_pose <<"\n";
        throw std::runtime_error("Failed to solve IK for end pose");
    }

}



gtsam::Values InitializeTrajectory::initJointTrajectoryFromVicon(
                                const gtsam::Vector& start_conf,
                                const TubeInfo& tube_info,
                                double offset_from_base_y,
                                double offset_from_tube_z,
                                const gtsam::Pose3& base_pose,
                                const size_t total_time_step,
                                gtsam::Pose3& best_end_pose, bool tune_pose) {
    
    std::vector<gtsam::Vector> end_confs;
    int best_idx = -1;
    double best_quality = 1e20;

    // Calculate target y position
    double target_y = base_pose.translation().y() + offset_from_base_y;

    std::cout << "Base info: " << base_pose.translation().x() << ", " << base_pose.translation().y() << ", " << base_pose.translation().z() <<"\n";
    std::cout << "Target y: " << target_y << "\n";
    
    // Find point on tube axis where y = target_y
    double t = (target_y - tube_info.centroid.y()) / tube_info.direction.y();
    Eigen::Vector3d tube_point = tube_info.centroid + t * tube_info.direction;
    
    // Base position for orientation calculations
    gtsam::Point3 base_point = base_pose.translation();
    Eigen::Vector3d base_pos(base_point.x(), base_point.y(), base_point.z());
    
    gtsam::Pose3 start_pose = forwardKinematics(dh_params_, start_conf, base_pose);
    gtsam::Point3 start_point = start_pose.translation();
    Eigen::Vector3d start_pos(start_point.x(), start_point.y(), start_point.z());
    
    // Position sweep: y-axis variation (-0.1 to +0.3, steps 0.02) 
    // for(double y_compensation = 0.0; y_compensation <= 0.2; y_compensation += 0.05) {

    for(int j =0; j < 4; j++){

        double y_compensation = 0.0;
        if(tune_pose){
            if(j == 0) y_compensation = 0.0;
            else y_compensation = (rand() % 15) / 100.0;
        }

        // Calculate modified tube point
        Eigen::Vector3d compensated_tube_point = tube_point;
        compensated_tube_point.y() += y_compensation;
        Eigen::Vector3d ee_position = compensated_tube_point;

        int success_count = 0;
        bool found_solution_for_this_position = false;

        if(y_compensation == 0.0){
                std::cout << "============================\n";
                std::cout << "    || 0   6  12  18  24  30\n";
        }

        printf("%.3f", y_compensation);
        std::cout << "||";
        
        // Orientation sweep: 0° to 45° around tube axis (steps 10°)  
        // for(int angle_deg = 0; angle_deg <= 30; angle_deg += 6) {
        for (int i = 0; i < 5; i++){
            double angle_deg = 0.0;

            if(tune_pose){
                angle_deg = (rand() % 41);
            }
            
            double angle_rad = angle_deg * M_PI / 180.0;
            
            ee_position.z() = compensated_tube_point.z() - cos(angle_rad) * offset_from_tube_z;

            if ((ee_position.x() - start_pos.x()) > 0) {
                ee_position.x() = compensated_tube_point.x() - sin(angle_rad) * offset_from_tube_z;
            } else {
                ee_position.x() = compensated_tube_point.x() + sin(angle_rad) * offset_from_tube_z;
            }

            // Calculate rotation axis (tube direction)
            Eigen::Vector3d tube_axis = tube_info.direction.normalized();
            
            // Apply rotation - choose direction that brings EE closer to base
            Eigen::Matrix3d rotated_orientation;
            
            // Ensure z-axis points toward tube axis (radially inward)
            Eigen::Vector3d to_tube = (compensated_tube_point - ee_position).normalized();
            rotated_orientation.col(2) = to_tube;
            
            // Y-axis in negative tube axis direction
            rotated_orientation.col(1) = -tube_axis;
            
            // X-axis via right-hand rule: x = y × z
            Eigen::Vector3d y_axis = -tube_axis;
            Eigen::Vector3d x_axis = y_axis.cross(to_tube).normalized();
            rotated_orientation.col(0) = x_axis;
            
            // Create target pose
            gtsam::Rot3 target_rotation(rotated_orientation);
            gtsam::Point3 target_position(ee_position);
            gtsam::Pose3 target_pose(target_rotation, target_position);

            // std::cout << "Y: "<< y_compensation << "Angle: " << angle_deg << "\n" << target_pose <<"\n\n";
            
            gtsam::Vector end_conf;
            if(solveIK(target_pose, base_pose, start_conf, end_conf, 50, 0.25)) {
                wrapAngles(end_conf, start_conf);
                end_confs.push_back(end_conf);
            
                success_count++;
                double magnitude = (end_conf - start_conf).norm();

                if(magnitude < best_quality) {
                    best_quality = magnitude;
                    best_idx = end_confs.size()-1;
                    best_end_pose = target_pose;  // Store the pose that gave us the best solution
                }
                found_solution_for_this_position = true;
                std::cout << " !  ";
            } else {
                std::cout << " .  ";
            }
        
        }
        std::cout << "\n";
    }
    std::cout << "=====================\n";
    

    if(end_confs.size() > 0) {
        gtsam::Values init_values;

        for (size_t i = 0; i <= total_time_step; i++) {
            gtsam::Vector conf;
            if (i == 0)
                conf = start_conf;
            else if (i == total_time_step) 
                conf = end_confs[best_idx];
            else
                conf =
                    static_cast<double>(i) / static_cast<double>(total_time_step) * end_confs[best_idx] +
                    (1.0 - static_cast<double>(i) / static_cast<double>(total_time_step)) *
                        start_conf;
            
            // std::cout << "\ninitial confs, i = " << i << ": ";
            // for(auto& k : conf){ std::cout << k << ", ";}
            // std::cout << "\n";

            init_values.insert(gtsam::Symbol('x', i), conf);
        }
        // init vel as avg vel
        gtsam::Vector avg_vel = (end_confs[best_idx] - start_conf) / static_cast<double>(total_time_step);
        for (size_t i = 0; i <= total_time_step; i++)
            init_values.insert(gtsam::Symbol('v', i), avg_vel);

        return init_values;
    } else {
        throw std::runtime_error("Failed to solve IK for any target pose with tube orientations");
    }
}


std::tuple<std::deque<Eigen::VectorXd>, std::deque<Eigen::VectorXd>, std::deque<Eigen::VectorXd>> 
InitializeTrajectory::initTaskSpaceTrajectory(const gtsam::Pose3& start_pose,
                      const gtsam::Pose3& end_pose,
                      const double& duration_sec,
                      const double& percentage,
                      const double& height,
                      double dt) {
    
    // Calculate number of points (including start and end)
    int num_points = static_cast<int>(duration_sec * (1/dt)) + 1;
    
    std::deque<Eigen::VectorXd> trajectory;
    
    // Handle edge case where duration is 0 or very small
    if (num_points <= 1) {
        Eigen::VectorXd pose_vec(6);
        gtsam::Vector3 pos = start_pose.translation();
        gtsam::Vector3 rpy = start_pose.rotation().rpy();
        pose_vec << pos.x(), pos.y(), pos.z(), rpy.x(), rpy.y(), rpy.z();
        trajectory.push_back(pose_vec);
        
        std::deque<Eigen::VectorXd> velocity, acceleration;
        velocity.push_back(Eigen::VectorXd::Zero(6));
        acceleration.push_back(Eigen::VectorXd::Zero(6));
        return std::make_tuple(trajectory, velocity, acceleration);
    }
    
    // Extract positions and orientations
    gtsam::Vector3 start_pos = start_pose.translation();
    gtsam::Vector3 end_pos = end_pose.translation();
    gtsam::Vector3 start_rpy = start_pose.rotation().rpy();
    gtsam::Vector3 end_rpy = end_pose.rotation().rpy();
    
    // Calculate the middle point for position spline
    gtsam::Vector3 line_point = start_pos + percentage * (end_pos - start_pos);
    gtsam::Vector3 middle_point = line_point;
    middle_point.z() += height;  // Add height offset in z-direction
    
    // Create cubic spline coefficients for each dimension (x, y, z)
    // Using natural cubic spline through 3 points: t = [0, 0.5, 1]
    // Control points: [start_pos, middle_point, end_pos]
    
    auto computeCubicSplineCoeffs = [](double p0, double p1, double p2) -> std::array<double, 4> {
        // For natural cubic spline through 3 points at t = [0, 0.5, 1]
        // Solving the system with natural boundary conditions (second derivative = 0 at ends)
        double a = p0;
        double b = -3.0 * p0 + 4.0 * p1 - p2;
        double c = 2.0 * p0 - 4.0 * p1 + 2.0 * p2;
        double d = 0.0;  // Natural boundary condition
        return {a, b, c, d};
    };
    
    // Get coefficients for each dimension
    auto coeffs_x = computeCubicSplineCoeffs(start_pos.x(), middle_point.x(), end_pos.x());
    auto coeffs_y = computeCubicSplineCoeffs(start_pos.y(), middle_point.y(), end_pos.y());
    auto coeffs_z = computeCubicSplineCoeffs(start_pos.z(), middle_point.z(), end_pos.z());
    
    // Helper functions for angular interpolation (same as original function)
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    };
    
    auto angularDifference = [&normalizeAngle](double end_angle, double start_angle) {
        return normalizeAngle(end_angle - start_angle);
    };
    
    // Pre-compute angular differences for RPY (handles wrapping)
    double roll_diff = angularDifference(end_rpy.x(), start_rpy.x());
    double pitch_diff = angularDifference(end_rpy.y(), start_rpy.y());
    double yaw_diff = angularDifference(end_rpy.z(), start_rpy.z());
    
    // Generate interpolated trajectory
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);  // Parameter from 0 to 1
        
        Eigen::VectorXd waypoint(6);
        
        if (i == 0) {
            // For the first point, use the same interpolation formula as other points (t=0)
            // to ensure consistency in RPY computation
            waypoint(0) = coeffs_x[0]; // t=0, so only the constant term
            waypoint(1) = coeffs_y[0];
            waypoint(2) = coeffs_z[0];
            
            // Use same angle computation as interpolation (t=0 case)
            waypoint(3) = normalizeAngle(start_rpy.x() + 0 * roll_diff);  // = start_rpy.x()
            waypoint(4) = normalizeAngle(start_rpy.y() + 0 * pitch_diff); // = start_rpy.y()
            waypoint(5) = normalizeAngle(start_rpy.z() + 0 * yaw_diff);   // = start_rpy.z()
        } else {
            // Cubic spline interpolation for position (x, y, z)
            waypoint(0) = coeffs_x[0] + coeffs_x[1] * t + coeffs_x[2] * t * t + coeffs_x[3] * t * t * t;
            waypoint(1) = coeffs_y[0] + coeffs_y[1] * t + coeffs_y[2] * t * t + coeffs_y[3] * t * t * t;
            waypoint(2) = coeffs_z[0] + coeffs_z[1] * t + coeffs_z[2] * t * t + coeffs_z[3] * t * t * t;
            
            // Linear interpolation for orientation (roll, pitch, yaw) with wrapping
            waypoint(3) = normalizeAngle(start_rpy.x() + t * roll_diff);
            waypoint(4) = normalizeAngle(start_rpy.y() + t * pitch_diff);
            waypoint(5) = normalizeAngle(start_rpy.z() + t * yaw_diff);
        }
        
        trajectory.push_back(waypoint);
    }
    
    
    // Calculate velocity trajectory using finite differences
    std::deque<Eigen::VectorXd> velocity;
    
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
                    // Linear velocities (position derivatives)
                    vel(j) = (trajectory[i+1](j) - trajectory[i-1](j)) / (2.0 * dt);
                }
            }
        }
        velocity.push_back(vel);
    }
    
    // Calculate acceleration trajectory using finite differences
    std::deque<Eigen::VectorXd> acceleration;
    
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

gtsam::Values
InitializeTrajectory::initTaskSpaceTrajectory(const gtsam::Pose3& start_pose,
                      const gtsam::Pose3& end_pose,
                      const gtsam::Pose3& base_pose,
                      const gtsam::Vector& start_conf,
                      std::deque<gtsam::Pose3>& pose_trajectory,
                      double percentage,
                      double height,
                      int num_points) {
    
    // Handle edge case where num_points is 1 or less
    if (num_points <= 1) {
        throw std::invalid_argument("num_points must be bigger than 1");
    }
    
    // Extract positions and orientations
    gtsam::Vector3 start_pos = start_pose.translation();
    gtsam::Vector3 end_pos = end_pose.translation();
    gtsam::Vector3 start_rpy = start_pose.rotation().rpy();
    gtsam::Vector3 end_rpy = end_pose.rotation().rpy();
    
    // Calculate the middle point for position spline
    gtsam::Vector3 line_point = start_pos + percentage * (end_pos - start_pos);
    gtsam::Vector3 middle_point = line_point;
    middle_point.z() += height;  // Add height offset in z-direction
    
    // Create cubic spline coefficients for each dimension (x, y, z)
    // Using natural cubic spline through 3 points: t = [0, 0.5, 1]
    // Control points: [start_pos, middle_point, end_pos]
    
    auto computeCubicSplineCoeffs = [](double p0, double p1, double p2) -> std::array<double, 4> {
        // For natural cubic spline through 3 points at t = [0, 0.5, 1]
        // Solving the system with natural boundary conditions (second derivative = 0 at ends)
        double a = p0;
        double b = -3.0 * p0 + 4.0 * p1 - p2;
        double c = 2.0 * p0 - 4.0 * p1 + 2.0 * p2;
        double d = 0.0;  // Natural boundary condition
        return {a, b, c, d};
    };
    
    // Get coefficients for each dimension
    auto coeffs_x = computeCubicSplineCoeffs(start_pos.x(), middle_point.x(), end_pos.x());
    auto coeffs_y = computeCubicSplineCoeffs(start_pos.y(), middle_point.y(), end_pos.y());
    auto coeffs_z = computeCubicSplineCoeffs(start_pos.z(), middle_point.z(), end_pos.z());
    
    // Helper functions for angular interpolation
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    };
    
    auto angularDifference = [&normalizeAngle](double end_angle, double start_angle) {
        return normalizeAngle(end_angle - start_angle);
    };
    
    // Pre-compute angular differences for RPY (handles wrapping)
    double roll_diff = angularDifference(end_rpy.x(), start_rpy.x());
    double pitch_diff = angularDifference(end_rpy.y(), start_rpy.y());
    double yaw_diff = angularDifference(end_rpy.z(), start_rpy.z());
    
    // Generate interpolated trajectory
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / (num_points);  // Parameter from 0 to 1
        
        // Calculate position using cubic spline
        gtsam::Vector3 position;
        if (i == 0) {
            position.x() = coeffs_x[0]; // t=0, so only the constant term
            position.y() = coeffs_y[0];
            position.z() = coeffs_z[0];
        } else {
            // Cubic spline interpolation for position (x, y, z)
            position.x() = coeffs_x[0] + coeffs_x[1] * t + coeffs_x[2] * t * t + coeffs_x[3] * t * t * t;
            position.y() = coeffs_y[0] + coeffs_y[1] * t + coeffs_y[2] * t * t + coeffs_y[3] * t * t * t;
            position.z() = coeffs_z[0] + coeffs_z[1] * t + coeffs_z[2] * t * t + coeffs_z[3] * t * t * t;
        }
        
        // Calculate orientation using linear interpolation with wrapping
        double roll = normalizeAngle(start_rpy.x() + t * roll_diff);
        double pitch = normalizeAngle(start_rpy.y() + t * pitch_diff);
        double yaw = normalizeAngle(start_rpy.z() + t * yaw_diff);
        
        // Create rotation from RPY
        gtsam::Rot3 rotation = gtsam::Rot3::RzRyRx(roll, pitch, yaw);
        
        // Create pose and add to trajectory
        gtsam::Pose3 pose(rotation, position);
        pose_trajectory.push_back(pose);
    }

    std::vector<gtsam::Vector> end_confs;
    end_confs.push_back(start_conf);
    gtsam::Vector end_conf;

    for(int i = 1; i <= num_points; i++){
        for(int j = 0; j < 4; j++){
            if(solveIK(pose_trajectory[i], base_pose, end_confs[i-1], end_conf, 50, 0.25)) {
                wrapAngles(end_conf, end_confs[i-1]);
                end_confs.push_back(end_conf);
                break;
            }

            if(j == 3){throw std::runtime_error("IK failed during task space initiation");}
        }
    }

    
    gtsam::Values init_values;

    for (size_t i = 0; i <= num_points; i++) {
        gtsam::Vector conf;
       
        conf = end_confs[i];

        init_values.insert(gtsam::Symbol('x', i), conf);
    }
    // init vel as avg vel
    gtsam::Vector avg_vel = gtsam::Vector::Zero(7);
    for (size_t i = 0; i <= num_points; i++)
        init_values.insert(gtsam::Symbol('v', i), avg_vel);

    return init_values;
}

