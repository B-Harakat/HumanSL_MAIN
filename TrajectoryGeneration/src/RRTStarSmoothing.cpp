#include "RRTStarSmoothing.h"
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <algorithm>
#include <cmath>

gtsam::Vector RRTStarSmoother::SmoothnessFactor::evaluateError(
    const gtsam::Vector& q1, const gtsam::Vector& q2, const gtsam::Vector& q3,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3) const {
    
    // Compute acceleration: (q3 - q2) / dt - (q2 - q1) / dt = (q3 - 2*q2 + q1) / dt^2
    gtsam::Vector acceleration = (q3 - 2.0 * q2 + q1) / (dt_ * dt_);
    
    // Jacobians
    if (H1) *H1 = gtsam::Matrix::Identity(q1.size(), q1.size()) / (dt_ * dt_);
    if (H2) *H2 = -2.0 * gtsam::Matrix::Identity(q2.size(), q2.size()) / (dt_ * dt_);
    if (H3) *H3 = gtsam::Matrix::Identity(q3.size(), q3.size()) / (dt_ * dt_);
    
    return acceleration;
}

gtsam::Vector RRTStarSmoother::SimpleObstacleFactor::evaluateError(
    const gtsam::Vector& config,
    boost::optional<gtsam::Matrix&> H) const {
    
    gtsam::Vector error = gtsam::Vector::Zero(1);
    
    // Check all robot links for collision
    double min_distance = std::numeric_limits<double>::max();
    
    std::vector<gtsam::Point3> sphere_centers;
    arm_model_.sphereCenters(config, sphere_centers);
    
    for (size_t i = 0; i < arm_model_.nr_body_spheres(); ++i) {
        double distance = sdf_.getSignedDistance(sphere_centers[i]);
        min_distance = std::min(min_distance, distance - arm_model_.sphere_radius(i));
    }
    
    // Penalty if too close to obstacles
    if (min_distance < safety_distance_) {
        error[0] = safety_distance_ - min_distance;
    }
    
    // Simple finite difference for Jacobian
    if (H) {
        gtsam::Matrix jacobian(1, config.size());
        double epsilon = 1e-6;
        
        for (int i = 0; i < config.size(); ++i) {
            gtsam::Vector config_plus = config;
            config_plus[i] += epsilon;
            
            gtsam::Vector config_minus = config;
            config_minus[i] -= epsilon;
            
            gtsam::Vector error_plus = gtsam::Vector::Zero(1);
            gtsam::Vector error_minus = gtsam::Vector::Zero(1);
            
            // Compute error for perturbed configurations
            double min_dist_plus = std::numeric_limits<double>::max();
            double min_dist_minus = std::numeric_limits<double>::max();
            
            std::vector<gtsam::Point3> sphere_centers_plus, sphere_centers_minus;
            arm_model_.sphereCenters(config_plus, sphere_centers_plus);
            arm_model_.sphereCenters(config_minus, sphere_centers_minus);
            
            for (size_t j = 0; j < arm_model_.nr_body_spheres(); ++j) {
                double dist_plus = sdf_.getSignedDistance(sphere_centers_plus[j]);
                double dist_minus = sdf_.getSignedDistance(sphere_centers_minus[j]);
                
                min_dist_plus = std::min(min_dist_plus, dist_plus - arm_model_.sphere_radius(j));
                min_dist_minus = std::min(min_dist_minus, dist_minus - arm_model_.sphere_radius(j));
            }
            
            if (min_dist_plus < safety_distance_) error_plus[0] = safety_distance_ - min_dist_plus;
            if (min_dist_minus < safety_distance_) error_minus[0] = safety_distance_ - min_dist_minus;
            
            jacobian(0, i) = (error_plus[0] - error_minus[0]) / (2.0 * epsilon);
        }
        
        *H = jacobian;
    }
    
    return error;
}

gtsam::Vector RRTStarSmoother::JointLimitFactor::evaluateError(
    const gtsam::Vector& config,
    boost::optional<gtsam::Matrix&> H) const {
    
    gtsam::Vector error = gtsam::Vector::Zero(config.size());
    
    for (int i = 0; i < config.size(); ++i) {
        double lower_violation = limits_.lower[i] + margin_ - config[i];
        double upper_violation = config[i] - (limits_.upper[i] - margin_);
        
        if (lower_violation > 0) {
            error[i] = lower_violation;
        } else if (upper_violation > 0) {
            error[i] = upper_violation;
        }
    }
    
    if (H) {
        gtsam::Matrix jacobian = gtsam::Matrix::Zero(config.size(), config.size());
        
        for (int i = 0; i < config.size(); ++i) {
            double lower_violation = limits_.lower[i] + margin_ - config[i];
            double upper_violation = config[i] - (limits_.upper[i] - margin_);
            
            if (lower_violation > 0) {
                jacobian(i, i) = -1.0;
            } else if (upper_violation > 0) {
                jacobian(i, i) = 1.0;
            }
        }
        
        *H = jacobian;
    }
    
    return error;
}

std::vector<gtsam::Vector> RRTStarSmoother::smoothTrajectory(
    const std::vector<gtsam::Vector>& path,
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    double dt) {
    
    if (path.size() < 3) return path;
    
    // Create factor graph
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;
    
    // Noise models
    auto smoothness_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector::Constant(path[0].size(), 1.0 / params_.smoothness_weight));
    auto obstacle_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector::Constant(1, 1.0 / params_.obstacle_weight));
    auto endpoint_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector::Constant(path[0].size(), 1.0 / params_.endpoint_weight));
    auto joint_limit_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector::Constant(path[0].size(), 1.0 / params_.joint_limit_weight));
    
    // Add waypoints to initial estimate
    for (size_t i = 0; i < path.size(); ++i) {
        gtsam::Symbol symbol('q', i);
        initial_estimate.insert(symbol, path[i]);
        
        // Add obstacle avoidance factors - simplified for now
        // graph.add(boost::make_shared<SimpleObstacleFactor>(symbol, obstacle_noise, arm_model, sdf));
        
        // Add joint limit factors - simplified for now
        // graph.add(boost::make_shared<JointLimitFactor>(symbol, joint_limit_noise, pos_limits));
    }
    
    // Add smoothness factors
    for (size_t i = 1; i < path.size() - 1; ++i) {
        gtsam::Symbol sym1('q', i - 1);
        gtsam::Symbol sym2('q', i);
        gtsam::Symbol sym3('q', i + 1);
        
        // graph.add(boost::make_shared<SmoothnessFactor>(sym1, sym2, sym3, smoothness_noise, dt));
    }
    
    // Add endpoint constraints if enabled
    if (params_.maintain_endpoints) {
        graph.add(gtsam::PriorFactor<gtsam::Vector>(gtsam::Symbol('q', 0), path[0], endpoint_noise));
        graph.add(gtsam::PriorFactor<gtsam::Vector>(gtsam::Symbol('q', path.size() - 1), path.back(), endpoint_noise));
    }
    
    // Optimize
    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(params_.max_iterations);
    lm_params.setRelativeErrorTol(params_.convergence_threshold);
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, lm_params);
    gtsam::Values result = optimizer.optimize();
    
    // Extract optimized path
    std::vector<gtsam::Vector> optimized_path;
    optimized_path.reserve(path.size());
    
    for (size_t i = 0; i < path.size(); ++i) {
        gtsam::Symbol symbol('q', i);
        optimized_path.push_back(result.at<gtsam::Vector>(symbol));
    }
    
    return optimized_path;
}

std::vector<gtsam::Vector> RRTStarSmoother::geometricSmoothing(
    const std::vector<gtsam::Vector>& path,
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    int iterations) {
    
    if (path.size() < 3) return path;
    
    std::vector<gtsam::Vector> smoothed_path = path;
    
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<gtsam::Vector> new_path = smoothed_path;
        
        // Apply smoothing to interior points
        for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
            gtsam::Vector smoothed_config = 0.25 * smoothed_path[i - 1] + 
                                          0.5 * smoothed_path[i] + 
                                          0.25 * smoothed_path[i + 1];
            
            // Check if smoothed configuration is collision-free
            bool valid = true;
            std::vector<gtsam::Point3> sphere_centers;
            arm_model.sphereCenters(smoothed_config, sphere_centers);
            
            for (size_t j = 0; j < arm_model.nr_body_spheres(); ++j) {
                double distance = sdf.getSignedDistance(sphere_centers[j]);
                if (distance <= arm_model.sphere_radius(j)) {
                    valid = false;
                    break;
                }
            }
            
            if (valid) {
                new_path[i] = smoothed_config;
            }
        }
        
        smoothed_path = new_path;
    }
    
    return smoothed_path;
}

std::vector<gtsam::Vector> RRTStarSmoother::shortcutSmoothing(
    const std::vector<gtsam::Vector>& path,
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    int max_iterations) {
    
    if (path.size() < 3) return path;
    
    std::vector<gtsam::Vector> shortened_path = path;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        bool improvement = false;
        
        for (size_t i = 0; i < shortened_path.size() - 2; ++i) {
            for (size_t j = i + 2; j < shortened_path.size(); ++j) {
                // Check if direct connection is possible
                bool valid_connection = true;
                gtsam::Vector start = shortened_path[i];
                gtsam::Vector end = shortened_path[j];
                gtsam::Vector direction = end - start;
                double distance = direction.norm();
                
                int num_checks = static_cast<int>(std::ceil(distance / 0.05));
                
                for (int k = 1; k < num_checks; ++k) {
                    double t = static_cast<double>(k) / static_cast<double>(num_checks);
                    gtsam::Vector intermediate = start + t * direction;
                    
                    // Check collision
                    std::vector<gtsam::Point3> sphere_centers;
                    arm_model.sphereCenters(intermediate, sphere_centers);
                    
                    for (size_t l = 0; l < arm_model.nr_body_spheres(); ++l) {
                        double distance = sdf.getSignedDistance(sphere_centers[l]);
                        if (distance <= arm_model.sphere_radius(l)) {
                            valid_connection = false;
                            break;
                        }
                    }
                    
                    if (!valid_connection) break;
                }
                
                if (valid_connection) {
                    // Remove intermediate waypoints
                    shortened_path.erase(shortened_path.begin() + i + 1, shortened_path.begin() + j);
                    improvement = true;
                    break;
                }
            }
            if (improvement) break;
        }
        
        if (!improvement) break;
    }
    
    return shortened_path;
}

std::vector<gtsam::Vector> RRTStarSmoother::combinedSmoothing(
    const std::vector<gtsam::Vector>& path,
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    double total_time_sec,
    double dt) {
    
    if (path.size() < 2) return path;
    
    // Step 1: Shortcut smoothing to remove redundant waypoints
    std::vector<gtsam::Vector> shortened_path = shortcutSmoothing(path, arm_model, sdf, 50);
    
    // Step 2: Geometric smoothing for initial refinement
    std::vector<gtsam::Vector> geometric_path = geometricSmoothing(shortened_path, arm_model, sdf, 5);
    
    // Step 3: Optimization-based smoothing for final refinement
    std::vector<gtsam::Vector> final_path = smoothTrajectory(geometric_path, arm_model, sdf, pos_limits, vel_limits, dt);
    
    return final_path;
}

double RRTStarSmoother::computePathLength(const std::vector<gtsam::Vector>& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i-1]).norm();
    }
    return length;
}

double RRTStarSmoother::computePathSmoothness(const std::vector<gtsam::Vector>& path) {
    if (path.size() < 3) return 0.0;
    
    double smoothness = 0.0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
        gtsam::Vector acceleration = path[i+1] - 2.0 * path[i] + path[i-1];
        smoothness += acceleration.squaredNorm();
    }
    return smoothness;
}

bool RRTStarSmoother::isValidPath(const std::vector<gtsam::Vector>& path,
                                 const gpmp2::ArmModel& arm_model,
                                 const gpmp2::SignedDistanceField& sdf) {
    for (const auto& config : path) {
        std::vector<gtsam::Point3> sphere_centers;
        arm_model.sphereCenters(config, sphere_centers);
        
        for (size_t i = 0; i < arm_model.nr_body_spheres(); ++i) {
            double distance = sdf.getSignedDistance(sphere_centers[i]);
            if (distance <= arm_model.sphere_radius(i)) {
                return false;
            }
        }
    }
    return true;
}