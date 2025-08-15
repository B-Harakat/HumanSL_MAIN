#include "TrajectoryOptimization.h"

OptimizeTrajectory::OptimizeTrajectory() {}

TrajectoryResult OptimizeTrajectory::optimizeJointTrajectory(
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& init_values,
    const gtsam::Pose3& target_pose,
    const gtsam::Vector& start_config,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    const size_t total_time_step,
    const double total_time_sec,
    const double target_dt) {
    
    std::cout << "Creating arm trajectory..." << std::endl;
    
    std::vector<std::string> factor_keys;
    std::unordered_map<std::string, double> init_factor_costs;
    std::unordered_map<std::string, double> final_factor_costs;

    // Trajectory parameters
    
    double delta_t = total_time_sec / total_time_step;
    
    // GP and optimization parameters
    gtsam::Matrix Qc = gtsam::Matrix::Identity(7, 7) * 0.5;
    auto Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);
    double collision_sigma = 0.0005;
    double epsilon_dist = 0.05;
    auto pose_fix_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.0005);
    auto vel_fix_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
    
    gtsam::Vector start_vel = gtsam::Vector::Zero(7);
    gtsam::Vector end_vel = gtsam::Vector::Zero(7);

    gtsam::Matrix self_collision_data(3, 4);  // 3 checks, 4 columns each
    self_collision_data << 
        0, 4, 0.03, collision_sigma,  // sphere 0 vs sphere 4
        0, 6, 0.03, collision_sigma,  // sphere 0 vs sphere 6  
        2, 6, 0.03, collision_sigma;  // sphere 2 vs sphere 6
    
    
    gtsam::NonlinearFactorGraph graph;
    
    for (size_t i = 0; i <= total_time_step; ++i) {
        gtsam::Symbol key_pos('x', i);
        gtsam::Symbol key_vel('v', i);
        
        // Start/end priors
        if (i == 0) {
            graph.add(gtsam::PriorFactor<gtsam::Vector>(key_pos, start_config, pose_fix_model));
                    factor_keys.push_back("StartPosPrior");
            graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, start_vel, vel_fix_model));
                    factor_keys.push_back("StartVelPrior");
        
        } else if (i == total_time_step) {
            graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, end_vel, vel_fix_model));
                    factor_keys.push_back("EndVelPrior");
        }           
        
        // Joint limits
        auto pos_limit_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
        gtsam::Vector limit_thresh = gtsam::Vector::Constant(7, 0.1);
        graph.add(gpmp2::JointLimitFactorVector(
            key_pos, pos_limit_model, pos_limits.lower, pos_limits.upper, limit_thresh));
        factor_keys.push_back("JointPosLimits");

        // Velocity limits
        auto vel_limit_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
        gtsam::Vector vel_limit_thresh = gtsam::Vector::Constant(7, 0.005);
        graph.add(gpmp2::VelocityLimitFactorVector(
            key_vel, vel_limit_model, vel_limits.upper, vel_limit_thresh));
        factor_keys.push_back("JointVelLimits");

        // Obstacle collision avoidance
        graph.add(gpmp2::ObstacleSDFFactorArm(
            key_pos, arm_model, sdf, collision_sigma, epsilon_dist));
        factor_keys.push_back("ObstacleFactor");

        graph.add(gpmp2::SelfCollisionArm(
            key_pos, arm_model, self_collision_data));
        factor_keys.push_back("SelfCollisionFactor");

        // GP priors
        if (i > 0) {
            gtsam::Symbol key_pos1('x', i - 1);
            gtsam::Symbol key_pos2('x', i);
            gtsam::Symbol key_vel1('v', i - 1);
            gtsam::Symbol key_vel2('v', i);
            
            graph.add(gpmp2::GaussianProcessPriorLinear(
                key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
            factor_keys.push_back("GP_Prior");
        }
    }

    auto jerk_noise_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.01);
        
    // Add jerk penalty factors for consecutive position triplets
    for (size_t i = 1; i < total_time_step; ++i) {
        gtsam::Symbol key_pos1('x', i - 1);
        gtsam::Symbol key_pos2('x', i);
        gtsam::Symbol key_pos3('x', i + 1);
        
        graph.add(JerkPenaltyFactor(
            key_pos1, key_pos2, key_pos3, jerk_noise_model, delta_t));
        factor_keys.push_back("JerkFactor");
    }
    // Add workspace constraints for final waypoint if specified
    gtsam::Symbol final_key('x', total_time_step);
    
    auto workspace_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    graph.add(gpmp2::GaussianPriorWorkspacePoseArm(
        final_key, arm_model, 6, target_pose, workspace_model));
    factor_keys.push_back("PoseFactor");
    
    for (size_t i = 0; i < graph.size(); ++i) {
      
        double constraint_error = graph.at(i)->error(init_values);

        init_factor_costs[factor_keys[i]] += constraint_error;    
    }
        
    // Setup optimizer
    gtsam::LevenbergMarquardtParams parameters;
    parameters.setVerbosity("none");
    parameters.setRelativeErrorTol(1e-6);
    parameters.setAbsoluteErrorTol(1e-6);
    parameters.setMaxIterations(1000);
    parameters.setlambdaInitial(1e-5);
    parameters.setlambdaFactor(10.0);
    parameters.setlambdaUpperBound(1e6);
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
    
    // Optimize
    gtsam::Values result = optimizer.optimize();
    
    for (size_t i = 0; i < graph.size(); ++i) {
  
        double constraint_error = graph.at(i)->error(result);
   
        final_factor_costs[factor_keys[i]] += constraint_error;
    }

    auto [densified_pos, densified_vel] = densifyTrajectory(result, Qc_model, delta_t, total_time_sec, target_dt);    
    
    // Formulate result
    TrajectoryResult trajectory_result;

    trajectory_result.dt = target_dt;
    trajectory_result.start_error = graph.error(init_values);
    trajectory_result.final_error = graph.error(result);
    trajectory_result.trajectory_pos = densified_pos;
    trajectory_result.trajectory_vel = densified_vel;
    trajectory_result.start_costs = init_factor_costs;
    trajectory_result.final_costs = final_factor_costs;

    return trajectory_result;
}


std::pair<std::vector<gtsam::Vector>, std::vector<gtsam::Vector>> OptimizeTrajectory::densifyTrajectory(
    const gtsam::Values& optimized_values,
    const gtsam::SharedNoiseModel& Qc_model,
    double delta_t,
    double total_time_sec,
    double target_dt) {
    
    std::vector<gtsam::Vector> dense_trajectory;
    std::vector<gtsam::Vector> dense_velocities;
    
    // Calculate total points needed for 1000Hz
    size_t total_dense_points = static_cast<size_t>(total_time_sec / target_dt) + 1;
    
    // Calculate number of interpolation points between each pair of waypoints
    size_t inter_step = static_cast<size_t>(delta_t / target_dt) - 1;
    
    // std::cout << "Densifying trajectory to" << target_dt <<"s interval:" << std::endl;
    // std::cout << "Original delta_t: " << delta_t << " seconds" << std::endl;
    // std::cout << "Target delta_t: " << target_dt << " seconds" << std::endl;
    // std::cout << "Interpolation points between waypoints: " << inter_step << std::endl;
    // std::cout << "Total dense points: " << total_dense_points << std::endl;
    
    // Find the maximum waypoint index in the optimized values
    size_t max_waypoint_idx = 0;
    gtsam::KeyVector key_vec = optimized_values.keys();
    for (const auto& key : key_vec) {
        if (gtsam::Symbol(key).chr() == 'x') {
            max_waypoint_idx = std::max(max_waypoint_idx, gtsam::Symbol(key).index());
        }
    }
    
    // Use GPMP2's interpolateArmTraj function to get densified values
    gtsam::Values dense_values = gpmp2::interpolateArmTraj(
        optimized_values,
        Qc_model,
        delta_t,
        inter_step,
        0,                    // start_index
        max_waypoint_idx      // end_index
    );
    
    // Convert gtsam::Values to std::vector<gtsam::Vector> for both positions and velocities
    // The interpolated values will have sequential indices starting from 0
    size_t dense_idx = 0;
    while (dense_values.exists(gtsam::Symbol('x', dense_idx))) {
        // Extract position
        gtsam::Vector config = dense_values.at<gtsam::Vector>(gtsam::Symbol('x', dense_idx));
        dense_trajectory.push_back(config);
        
        // Extract velocity
        if (dense_values.exists(gtsam::Symbol('v', dense_idx))) {
            gtsam::Vector velocity = dense_values.at<gtsam::Vector>(gtsam::Symbol('v', dense_idx));
            dense_velocities.push_back(velocity);
        }
        
        dense_idx++;
    }
    
    std::cout << "Generated " << dense_trajectory.size() << " dense position waypoints" << std::endl;
    std::cout << "Generated " << dense_velocities.size() << " dense velocity waypoints" << std::endl;
    
    // Verify timing
    double actual_dt = total_time_sec / (dense_trajectory.size() - 1);
    double actual_freq = 1.0 / actual_dt;
    std::cout << "Actual frequency: " << actual_freq << " Hz" << std::endl;
    
    return std::make_pair(dense_trajectory, dense_velocities);
}

TrajectoryResult OptimizeTrajectory::reOptimizeJointTrajectory(
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& init_values,
    const gtsam::Pose3& target_pose,
    const std::vector<gtsam::Vector>& start_configs,
    const std::vector<gtsam::Vector>& start_velocities,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    const size_t total_time_step,
    const double total_time_sec,
    const double target_dt) {
    
    std::cout << "Creating re-optimized arm trajectory..." << std::endl;
    
    std::vector<std::string> factor_keys;
    std::unordered_map<std::string, double> init_factor_costs;
    std::unordered_map<std::string, double> final_factor_costs;

    // Trajectory parameters
    double delta_t = total_time_sec / total_time_step;
    
    // GP and optimization parameters
    gtsam::Matrix Qc = gtsam::Matrix::Identity(7, 7);
    auto Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);
    double collision_sigma = 0.0005;
    double epsilon_dist = 0.05;
    auto pose_fix_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.0005);
    auto vel_fix_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
    
    gtsam::Matrix self_collision_data(3, 4);
    self_collision_data << 
        0, 4, 0.03, collision_sigma,
        0, 6, 0.03, collision_sigma,
        2, 6, 0.03, collision_sigma;
    
    gtsam::NonlinearFactorGraph graph;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (size_t i = 0; i <= total_time_step; ++i) {
        gtsam::Symbol key_pos('x', i);
        gtsam::Symbol key_vel('v', i);
        
        // GP priors
        if (i > 0) {
            gtsam::Symbol key_pos1('x', i-1);
            gtsam::Symbol key_pos2('x', i);
            gtsam::Symbol key_vel1('v', i-1);
            gtsam::Symbol key_vel2('v', i);
            
            graph.add(gpmp2::GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
            factor_keys.push_back("GPPrior");
        }
        
        // Start/end priors - Modified to use extracted states for first 3 timesteps
        // if (i == 0 || i == 1 || i == 2) {
        if (i == 0) {
            if (i < start_configs.size()) {
                graph.add(gtsam::PriorFactor<gtsam::Vector>(key_pos, start_configs[i], pose_fix_model));
                factor_keys.push_back("StartPosPrior");
            }
            if (i < start_velocities.size()) {
                graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, start_velocities[i], vel_fix_model));
                factor_keys.push_back("StartVelPrior");
            }
        }
        
        // End constraints (zero velocity at final timestep)
        if (i == total_time_step) {
            gtsam::Vector end_vel = gtsam::Vector::Zero(7);
            graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, end_vel, vel_fix_model));
            factor_keys.push_back("EndVelPrior");
        }
        
        // Obstacle avoidance
        graph.add(gpmp2::ObstacleSDFFactorArm(key_pos, arm_model, sdf, collision_sigma, epsilon_dist));
        factor_keys.push_back("ObstacleAvoidance");
        
        // Self collision avoidance
        graph.add(gpmp2::SelfCollisionArm(key_pos, arm_model, self_collision_data));
        factor_keys.push_back("SelfCollisionFactor");
        
        // Velocity limits
        auto vel_limit_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
        gtsam::Vector vel_limit_thresh = gtsam::Vector::Constant(7, 0.05);
        graph.add(gpmp2::VelocityLimitFactorVector(
            key_vel, vel_limit_model, vel_limits.upper, vel_limit_thresh));
        factor_keys.push_back("JointVelLimits");
    }
    
    // Add jerk penalty factors for consecutive position triplets
    auto jerk_noise_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.0001);
    for (size_t i = 1; i < total_time_step; ++i) {

        // if(i == 1 || i == total_time_step - 1){}
        gtsam::Symbol key_pos1('x', i - 1);
        gtsam::Symbol key_pos2('x', i);
        gtsam::Symbol key_pos3('x', i + 1);
        
        graph.add(JerkPenaltyFactor(
            key_pos1, key_pos2, key_pos3, jerk_noise_model, delta_t));
        factor_keys.push_back("JerkFactor");
    }
    
    // Target pose constraint at end
    auto workspace_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-4);
    graph.add(gpmp2::GaussianPriorWorkspacePoseArm(
        gtsam::Symbol('x', total_time_step), arm_model, 6, target_pose, workspace_model));
    factor_keys.push_back("PoseFactor");
    
    // Calculate initial error
    double init_error = graph.error(init_values);

    for (size_t i = 0; i < graph.size(); ++i) {
      
        double constraint_error = graph.at(i)->error(init_values);

        init_factor_costs[factor_keys[i]] += constraint_error;    
    }
    
    // Optimization
    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(1000);
    lm_params.setRelativeErrorTol(1e-8);
    lm_params.setAbsoluteErrorTol(1e-8);
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, lm_params);
    gtsam::Values result = optimizer.optimize();
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto optimization_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    

    for (size_t i = 0; i < graph.size(); ++i) {
  
        double constraint_error = graph.at(i)->error(result);
   
        final_factor_costs[factor_keys[i]] += constraint_error;
    }

    double final_error = graph.error(result);

    auto [densified_pos, densified_vel] = densifyTrajectory(result, Qc_model, delta_t, total_time_sec, target_dt);    
    
    // Formulate result
    TrajectoryResult trajectory_result;

    trajectory_result.dt = target_dt;
    trajectory_result.start_error = graph.error(init_values);
    trajectory_result.final_error = graph.error(result);
    trajectory_result.trajectory_pos = densified_pos;
    trajectory_result.trajectory_vel = densified_vel;
    trajectory_result.start_costs = init_factor_costs;
    trajectory_result.final_costs = final_factor_costs;

    std::cout << "Re-optimization completed. Error: " << init_error << " -> " << final_error << std::endl;
    
    return trajectory_result;
}

