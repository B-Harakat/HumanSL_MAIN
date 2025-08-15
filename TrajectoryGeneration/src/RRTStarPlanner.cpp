#include "RRTStarPlanner.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <queue>
#include <cmath>
#include <iomanip>
#include <gpmp2/obstacle/SDFexception.h>

RRTStarPlanner::RRTStarPlanner(const std::string& dh_params_path,
                               const std::string& joint_limits_path,
                               const RRTStarParams& params)
    : ArmModel(), params_(params), gen_(rd_()) {
    
    dh_params_ = createDHParams(dh_params_path);
    auto [pos_limits, vel_limits] = createJointLimits(joint_limits_path);
    pos_limits_ = pos_limits;
    vel_limits_ = vel_limits;
    
    // Initialize random distributions for each joint
    joint_distributions_.reserve(pos_limits_.lower.size());
    for (int i = 0; i < pos_limits_.lower.size(); ++i) {
        joint_distributions_.emplace_back(pos_limits_.lower[i], pos_limits_.upper[i]);
    }
}

TrajectoryResult RRTStarPlanner::plan(const gpmp2::ArmModel& arm_model,
                                     const gtsam::Vector& start_conf,
                                     const gtsam::Pose3& end_pose,
                                     const gtsam::Pose3& base_pose,
                                     double total_time_sec,
                                     size_t total_time_step,
                                     const gpmp2::SignedDistanceField& sdf) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Solve IK for goal configuration
    std::cout << "RRT*: Solving IK for target pose..." << std::endl;
    gtsam::Vector goal_config = solveGoalIK(end_pose, base_pose, start_conf);
    std::cout << "RRT*: Goal IK solved. Goal config: [";
    for (int i = 0; i < goal_config.size(); ++i) {
        std::cout << goal_config[i];
        if (i < goal_config.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // Initialize tree with start configuration
    std::vector<std::shared_ptr<RRTNode>> tree;
    auto start_node = std::make_shared<RRTNode>(start_conf);
    tree.push_back(start_node);
    
    std::shared_ptr<RRTNode> goal_node = nullptr;
    
    // Add progress reporting and timeout
    auto iteration_start_time = std::chrono::high_resolution_clock::now();
    const auto timeout_duration = std::chrono::minutes(5); // 5 minute timeout
    
    // Statistics tracking
    int invalid_config_count = 0;
    int invalid_edge_count = 0;
    int valid_additions = 0;
    
    for (int iteration = 0; iteration < params_.max_iterations; ++iteration) {
        // Check timeout
        auto current_time = std::chrono::high_resolution_clock::now();
        if (current_time - iteration_start_time > timeout_duration) {
            std::cout << "RRT*: Timeout after " << iteration << " iterations" << std::endl;
            break;
        }
        
        // Progress reporting (reduced frequency)
        if (iteration % 1000 == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - iteration_start_time);
            double success_rate = (double)valid_additions / (iteration + 1) * 100.0;
            std::cout << "RRT*: Iteration " << iteration << ", tree size: " << tree.size() 
                      << ", elapsed: " << elapsed.count() << "s, success rate: " 
                      << std::fixed << std::setprecision(1) << success_rate << "%" << std::endl;
            std::cout << "  Invalid configs: " << invalid_config_count 
                      << ", invalid edges: " << invalid_edge_count << std::endl;
        }
        
        // Sample configuration - FUNDAMENTAL CHANGE: Task-space sampling
        gtsam::Vector sample_config;
        if (iteration % 10 == 0) {  // 10% task-space sampling
            sample_config = sampleTaskSpaceBiased(end_pose, base_pose, goal_config);
        } else {
            sample_config = sampleGoalBiased(goal_config);  // Original joint-space sampling
        }
        
        // Find nearest node
        auto nearest_node = findNearestNode(tree, sample_config);
        
        // Steer towards sample
        gtsam::Vector new_config = steer(nearest_node->config, sample_config);
        
        // Check if new configuration is valid
        bool config_valid = isValidConfig(new_config, arm_model, sdf);
        if (!config_valid) {
            invalid_config_count++;
            if (iteration % 1000 == 0) {
                std::cout << "  Config invalid (joint limits or collision)" << std::endl;
            }
            continue;
        }
        
        bool edge_valid = isValidEdge(nearest_node->config, new_config, arm_model, sdf);
        if (!edge_valid) {
            invalid_edge_count++;
            if (iteration % 1000 == 0) {
                std::cout << "  Edge invalid (collision along path)" << std::endl;
            }
            continue;
        }
        
        valid_additions++;
        if (iteration % 1000 == 0) {
            std::cout << "  Valid config and edge found!" << std::endl;
        }
        
        // Find nearby nodes for potential rewiring
        std::vector<std::shared_ptr<RRTNode>> nearby_nodes = 
            findNearbyNodes(tree, new_config, params_.rewire_radius);
        
        // Choose best parent
        auto best_parent = chooseBestParent(nearby_nodes, new_config, arm_model, sdf);
        if (!best_parent) best_parent = nearest_node;
        
        // Create new node
        double cost = best_parent->cost + computeConfigDistance(best_parent->config, new_config);
        auto new_node = std::make_shared<RRTNode>(new_config, best_parent, cost);
        
        // Add to tree
        tree.push_back(new_node);
        best_parent->children.push_back(new_node);
        
        // Rewire tree
        rewireTree(tree, new_node, nearby_nodes, arm_model, sdf);
        
        // Check if goal is reached
        double distance_to_goal = computeConfigDistance(new_config, goal_config);
        if (iteration % 1000 == 0) {
            std::cout << "  Distance to goal: " << distance_to_goal 
                      << " (tolerance: " << params_.goal_tolerance << ")" << std::endl;
        }
        
        if (isGoalReached(new_config, goal_config)) {
            std::cout << "RRT*: GOAL REACHED at iteration " << iteration << "!" << std::endl;
            goal_node = new_node;
            break;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    TrajectoryResult result;
    result.optimization_duration = duration;
    result.dt = total_time_sec / static_cast<double>(total_time_step);
    
    if (!goal_node) {
        std::cerr << "RRT*: Failed to find path to goal" << std::endl;
        return result;
    }
    
    // Extract path
    std::vector<gtsam::Vector> path = extractPath(goal_node);
    
    // Optimize path for smoothness if enabled
    if (params_.optimize_path) {
        path = optimizePathSmoothness(path, arm_model, sdf);
    }
    
    // Convert path to trajectory
    result = pathToTrajectory(path, total_time_sec);
    result.optimization_duration = duration;
    
    return result;
}

TrajectoryResult RRTStarPlanner::planConfigToConfig(const gpmp2::ArmModel& arm_model,
                                                   const gtsam::Vector& start_conf,
                                                   const gtsam::Vector& goal_conf,
                                                   double total_time_sec,
                                                   const gpmp2::SignedDistanceField& sdf) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Initialize tree with start configuration
    std::vector<std::shared_ptr<RRTNode>> tree;
    auto start_node = std::make_shared<RRTNode>(start_conf);
    tree.push_back(start_node);
    
    std::shared_ptr<RRTNode> goal_node = nullptr;
    
    for (int iteration = 0; iteration < params_.max_iterations; ++iteration) {
        // Sample configuration
        gtsam::Vector sample_config = sampleGoalBiased(goal_conf);
        
        // Find nearest node
        auto nearest_node = findNearestNode(tree, sample_config);
        
        // Steer towards sample
        gtsam::Vector new_config = steer(nearest_node->config, sample_config);
        
        // Check if new configuration is valid
        if (!isValidConfig(new_config, arm_model, sdf)) continue;
        if (!isValidEdge(nearest_node->config, new_config, arm_model, sdf)) continue;
        
        // Find nearby nodes for potential rewiring
        std::vector<std::shared_ptr<RRTNode>> nearby_nodes = 
            findNearbyNodes(tree, new_config, params_.rewire_radius);
        
        // Choose best parent
        auto best_parent = chooseBestParent(nearby_nodes, new_config, arm_model, sdf);
        if (!best_parent) best_parent = nearest_node;
        
        // Create new node
        double cost = best_parent->cost + computeConfigDistance(best_parent->config, new_config);
        auto new_node = std::make_shared<RRTNode>(new_config, best_parent, cost);
        
        // Add to tree
        tree.push_back(new_node);
        best_parent->children.push_back(new_node);
        
        // Rewire tree
        rewireTree(tree, new_node, nearby_nodes, arm_model, sdf);
        
        // Check if goal is reached
        if (isGoalReached(new_config, goal_conf)) {
            goal_node = new_node;
            break;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    TrajectoryResult result;
    result.optimization_duration = duration;
    
    if (!goal_node) {
        std::cerr << "RRT*: Failed to find path to goal" << std::endl;
        return result;
    }
    
    // Extract path
    std::vector<gtsam::Vector> path = extractPath(goal_node);
    
    // Optimize path for smoothness if enabled
    if (params_.optimize_path) {
        path = optimizePathSmoothness(path, arm_model, sdf);
    }
    
    // Convert path to trajectory
    result = pathToTrajectory(path, total_time_sec);
    result.optimization_duration = duration;
    
    return result;
}

gtsam::Vector RRTStarPlanner::sampleRandomConfig() {
    gtsam::Vector config(joint_distributions_.size());
    for (size_t i = 0; i < joint_distributions_.size(); ++i) {
        config[i] = joint_distributions_[i](gen_);
    }
    return config;
}

gtsam::Vector RRTStarPlanner::sampleGoalBiased(const gtsam::Vector& goal_config) {
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    if (bias_dist(gen_) < params_.goal_bias) {
        return goal_config;
    }
    return sampleRandomConfig();
}

std::shared_ptr<RRTNode> RRTStarPlanner::findNearestNode(const std::vector<std::shared_ptr<RRTNode>>& tree,
                                                         const gtsam::Vector& config) {
    double min_distance = std::numeric_limits<double>::max();
    std::shared_ptr<RRTNode> nearest_node = nullptr;
    
    for (const auto& node : tree) {
        double distance = computeConfigDistance(node->config, config);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node = node;
        }
    }
    
    return nearest_node;
}

gtsam::Vector RRTStarPlanner::steer(const gtsam::Vector& from, const gtsam::Vector& to) {
    gtsam::Vector direction = to - from;
    double distance = direction.norm();
    
    if (distance <= params_.step_size) {
        return to;
    }
    
    direction = direction / distance * params_.step_size;
    return from + direction;
}

bool RRTStarPlanner::isValidConfig(const gtsam::Vector& config, const gpmp2::ArmModel& arm_model,
                                  const gpmp2::SignedDistanceField& sdf) {
    // Check joint limits
    for (int i = 0; i < config.size(); ++i) {
        if (config[i] < pos_limits_.lower[i] || config[i] > pos_limits_.upper[i]) {
            return false;
        }
    }
    
    // Check collision using SDF - optimized for early termination
    std::vector<gtsam::Point3> sphere_centers;
    arm_model.sphereCenters(config, sphere_centers);
    
    // Check larger spheres first (more likely to collide)
    for (size_t i = 0; i < arm_model.nr_body_spheres(); ++i) {
        try {
            double distance = sdf.getSignedDistance(sphere_centers[i]);
            double total_eps = arm_model.sphere_radius(i);
            if (distance <= total_eps) {  // In collision - early return
                return false;
            }
        } catch (const gpmp2::SDFQueryOutOfRange&) {
            // Sphere center is outside SDF bounds - early return
            return false;
        }
    }
    
    return true;
}

bool RRTStarPlanner::isValidEdge(const gtsam::Vector& from, const gtsam::Vector& to,
                                const gpmp2::ArmModel& arm_model, const gpmp2::SignedDistanceField& sdf) {
    gtsam::Vector direction = to - from;
    double distance = direction.norm();
    
    // Fast rejection: if distance is very small, likely valid
    if (distance < 0.01) return true;
    
    int num_checks = static_cast<int>(std::ceil(distance / params_.collision_check_resolution));
    
    for (int i = 1; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(num_checks);
        gtsam::Vector intermediate_config = from + t * direction;
        
        if (!isValidConfig(intermediate_config, arm_model, sdf)) {
            return false;
        }
    }
    
    return true;
}

std::vector<std::shared_ptr<RRTNode>> RRTStarPlanner::findNearbyNodes(
    const std::vector<std::shared_ptr<RRTNode>>& tree,
    const gtsam::Vector& config, double radius) {
    
    std::vector<std::shared_ptr<RRTNode>> nearby_nodes;
    
    for (const auto& node : tree) {
        double distance = computeConfigDistance(node->config, config);
        if (distance <= radius) {
            nearby_nodes.push_back(node);
        }
    }
    
    return nearby_nodes;
}

std::shared_ptr<RRTNode> RRTStarPlanner::chooseBestParent(
    const std::vector<std::shared_ptr<RRTNode>>& nearby_nodes,
    const gtsam::Vector& new_config, const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf) {
    
    std::shared_ptr<RRTNode> best_parent = nullptr;
    double best_cost = std::numeric_limits<double>::max();
    
    for (const auto& node : nearby_nodes) {
        if (!isValidEdge(node->config, new_config, arm_model, sdf)) continue;
        
        double cost = node->cost + computeConfigDistance(node->config, new_config);
        if (cost < best_cost) {
            best_cost = cost;
            best_parent = node;
        }
    }
    
    return best_parent;
}

void RRTStarPlanner::rewireTree(std::vector<std::shared_ptr<RRTNode>>& tree,
                               std::shared_ptr<RRTNode> new_node,
                               const std::vector<std::shared_ptr<RRTNode>>& nearby_nodes,
                               const gpmp2::ArmModel& arm_model, const gpmp2::SignedDistanceField& sdf) {
    
    for (const auto& node : nearby_nodes) {
        if (node == new_node || node == new_node->parent) continue;
        
        if (!isValidEdge(new_node->config, node->config, arm_model, sdf)) continue;
        
        double new_cost = new_node->cost + computeConfigDistance(new_node->config, node->config);
        if (new_cost < node->cost) {
            // Remove node from old parent's children
            if (node->parent) {
                auto& children = node->parent->children;
                children.erase(std::remove(children.begin(), children.end(), node), children.end());
            }
            
            // Set new parent
            node->parent = new_node;
            node->cost = new_cost;
            new_node->children.push_back(node);
            
            // Update costs of all descendants
            std::function<void(std::shared_ptr<RRTNode>)> updateCosts = 
                [&](std::shared_ptr<RRTNode> n) {
                    for (auto& child : n->children) {
                        child->cost = n->cost + computeConfigDistance(n->config, child->config);
                        updateCosts(child);
                    }
                };
            updateCosts(node);
        }
    }
}

double RRTStarPlanner::computeConfigDistance(const gtsam::Vector& q1, const gtsam::Vector& q2) {
    gtsam::Vector diff = q1 - q2;
    return diff.norm();
}

double RRTStarPlanner::computePathCost(std::shared_ptr<RRTNode> node) {
    return node->cost;
}

double RRTStarPlanner::computeSmoothnessCost(const gtsam::Vector& q1, const gtsam::Vector& q2, const gtsam::Vector& q3) {
    gtsam::Vector acceleration = q3 - 2.0 * q2 + q1;
    return acceleration.squaredNorm();
}

std::vector<gtsam::Vector> RRTStarPlanner::extractPath(std::shared_ptr<RRTNode> goal_node) {
    std::vector<gtsam::Vector> path;
    std::shared_ptr<RRTNode> current = goal_node;
    
    while (current) {
        path.push_back(current->config);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<gtsam::Vector> RRTStarPlanner::optimizePathSmoothness(const std::vector<gtsam::Vector>& path,
                                                                 const gpmp2::ArmModel& arm_model,
                                                                 const gpmp2::SignedDistanceField& sdf) {
    if (path.size() < 3) return path;
    
    std::vector<gtsam::Vector> optimized_path = path;
    
    // Simple smoothing: try to shortcut path segments
    for (size_t i = 0; i < optimized_path.size() - 2; ++i) {
        for (size_t j = i + 2; j < optimized_path.size(); ++j) {
            if (isValidEdge(optimized_path[i], optimized_path[j], arm_model, sdf)) {
                // Remove intermediate waypoints
                optimized_path.erase(optimized_path.begin() + i + 1, optimized_path.begin() + j);
                break;
            }
        }
    }
    
    return optimized_path;
}

TrajectoryResult RRTStarPlanner::pathToTrajectory(const std::vector<gtsam::Vector>& path,
                                                 double total_time_sec, double dt) {
    TrajectoryResult result;
    result.dt = dt;
    
    if (path.empty()) return result;
    
    size_t total_points = static_cast<size_t>(total_time_sec / dt);
    result.trajectory_pos.reserve(total_points);
    result.trajectory_vel.reserve(total_points);
    
    // Simple linear interpolation between waypoints
    double segment_time = total_time_sec / static_cast<double>(path.size() - 1);
    size_t points_per_segment = static_cast<size_t>(segment_time / dt);
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        gtsam::Vector start_config = path[i];
        gtsam::Vector end_config = path[i + 1];
        gtsam::Vector segment_vel = (end_config - start_config) / segment_time;
        
        for (size_t j = 0; j < points_per_segment; ++j) {
            double t = static_cast<double>(j) / static_cast<double>(points_per_segment);
            gtsam::Vector interpolated_config = start_config + t * (end_config - start_config);
            
            result.trajectory_pos.push_back(interpolated_config);
            result.trajectory_vel.push_back(segment_vel);
        }
    }
    
    // Add final point
    result.trajectory_pos.push_back(path.back());
    result.trajectory_vel.push_back(gtsam::Vector::Zero(path.back().size()));
    
    return result;
}

bool RRTStarPlanner::isGoalReached(const gtsam::Vector& config, const gtsam::Vector& goal_config) {
    return computeConfigDistance(config, goal_config) <= params_.goal_tolerance;
}

gtsam::Vector RRTStarPlanner::solveGoalIK(const gtsam::Pose3& target_pose, const gtsam::Pose3& base_pose,
                                         const gtsam::Vector& seed_config) {
    // Simple IK solver - could be improved with better algorithm
    gtsam::Vector current_config = seed_config;
    
    for (int iteration = 0; iteration < 100; ++iteration) {
        gtsam::Pose3 current_pose = forwardKinematics(dh_params_, current_config, base_pose);
        gtsam::Pose3 error_pose = target_pose.between(current_pose);
        
        // Check if converged
        if (error_pose.translation().norm() < 0.01 && 
            error_pose.rotation().rpy().norm() < 0.1) {
            break;
        }
        
        // Simple gradient descent update
        gtsam::Vector6 error;
        error << error_pose.translation(), error_pose.rotation().rpy();
        
        // Compute Jacobian numerically
        gtsam::Matrix J(6, current_config.size());
        double epsilon = 1e-6;
        
        for (int i = 0; i < current_config.size(); ++i) {
            gtsam::Vector config_plus = current_config;
            config_plus[i] += epsilon;
            gtsam::Pose3 pose_plus = forwardKinematics(dh_params_, config_plus, base_pose);
            
            gtsam::Vector config_minus = current_config;
            config_minus[i] -= epsilon;
            gtsam::Pose3 pose_minus = forwardKinematics(dh_params_, config_minus, base_pose);
            
            gtsam::Vector6 diff;
            diff << (pose_plus.translation() - pose_minus.translation()) / (2 * epsilon),
                    (pose_plus.rotation().rpy() - pose_minus.rotation().rpy()) / (2 * epsilon);
            
            J.col(i) = diff;
        }
        
        // Update configuration
        gtsam::Vector delta_q = J.transpose() * error * 0.1;
        current_config += delta_q;
        
        // Clamp to joint limits
        for (int i = 0; i < current_config.size(); ++i) {
            current_config[i] = std::max(pos_limits_.lower[i], 
                                       std::min(pos_limits_.upper[i], current_config[i]));
        }
    }
    
    return current_config;
}