#pragma once

#include "utils.h"
#include "GenerateArmModel.h"
#include <vector>
#include <memory>
#include <random>
#include <chrono>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/SignedDistanceField.h>

struct RRTNode {
    gtsam::Vector config;
    std::shared_ptr<RRTNode> parent;
    std::vector<std::shared_ptr<RRTNode>> children;
    double cost;
    
    RRTNode(const gtsam::Vector& q) : config(q), parent(nullptr), cost(0.0) {}
    RRTNode(const gtsam::Vector& q, std::shared_ptr<RRTNode> p, double c) 
        : config(q), parent(p), cost(c) {}
};

struct RRTStarParams {
    double step_size = 0.2;           // Maximum step size in configuration space
    double goal_tolerance = 0.05;     // Distance tolerance to goal
    double rewire_radius = 0.5;       // Radius for rewiring nearby nodes
    int max_iterations = 5000;        // Maximum iterations
    double goal_bias = 0.1;           // Probability of sampling goal
    double collision_check_resolution = 0.05; // Resolution for collision checking along edges
    double smoothness_weight = 1.0;   // Weight for smoothness in cost function
    bool optimize_path = true;        // Whether to optimize final path for smoothness
    
    RRTStarParams() = default;
};

class RRTStarPlanner : public ArmModel {
private:
    DHParameters dh_params_;
    JointLimits pos_limits_;
    JointLimits vel_limits_;
    RRTStarParams params_;
    
    std::random_device rd_;
    std::mt19937 gen_;
    std::vector<std::uniform_real_distribution<double>> joint_distributions_;
    
    // Core RRT* functions
    gtsam::Vector sampleRandomConfig();
    gtsam::Vector sampleGoalBiased(const gtsam::Vector& goal_config);
    gtsam::Vector sampleTaskSpaceBiased(const gtsam::Pose3& target_pose, const gtsam::Pose3& base_pose, const gtsam::Vector& goal_config);
    std::shared_ptr<RRTNode> findNearestNode(const std::vector<std::shared_ptr<RRTNode>>& tree, 
                                             const gtsam::Vector& config);
    gtsam::Vector steer(const gtsam::Vector& from, const gtsam::Vector& to);
    bool isValidConfig(const gtsam::Vector& config, const gpmp2::ArmModel& arm_model, 
                      const gpmp2::SignedDistanceField& sdf);
    bool isValidEdge(const gtsam::Vector& from, const gtsam::Vector& to, 
                    const gpmp2::ArmModel& arm_model, const gpmp2::SignedDistanceField& sdf);
    
    std::vector<std::shared_ptr<RRTNode>> findNearbyNodes(
        const std::vector<std::shared_ptr<RRTNode>>& tree, 
        const gtsam::Vector& config, double radius);
    
    std::shared_ptr<RRTNode> chooseBestParent(
        const std::vector<std::shared_ptr<RRTNode>>& nearby_nodes,
        const gtsam::Vector& new_config, const gpmp2::ArmModel& arm_model, 
        const gpmp2::SignedDistanceField& sdf);
    
    void rewireTree(std::vector<std::shared_ptr<RRTNode>>& tree,
                   std::shared_ptr<RRTNode> new_node,
                   const std::vector<std::shared_ptr<RRTNode>>& nearby_nodes,
                   const gpmp2::ArmModel& arm_model, const gpmp2::SignedDistanceField& sdf);
    
    // Cost and smoothness functions
    double computeConfigDistance(const gtsam::Vector& q1, const gtsam::Vector& q2);
    double computePathCost(std::shared_ptr<RRTNode> node);
    double computeSmoothnessCost(const gtsam::Vector& q1, const gtsam::Vector& q2, const gtsam::Vector& q3);
    
    // Path extraction and optimization
    std::vector<gtsam::Vector> extractPath(std::shared_ptr<RRTNode> goal_node);
    std::vector<gtsam::Vector> optimizePathSmoothness(const std::vector<gtsam::Vector>& path,
                                                     const gpmp2::ArmModel& arm_model,
                                                     const gpmp2::SignedDistanceField& sdf);
    
    // Trajectory generation from path
    TrajectoryResult pathToTrajectory(const std::vector<gtsam::Vector>& path,
                                     double total_time_sec, double dt = 0.001);
    
    // Goal reaching check
    bool isGoalReached(const gtsam::Vector& config, const gtsam::Vector& goal_config);
    gtsam::Vector solveGoalIK(const gtsam::Pose3& target_pose, const gtsam::Pose3& base_pose,
                             const gtsam::Vector& seed_config);

public:
    RRTStarPlanner(const std::string& dh_params_path,
                   const std::string& joint_limits_path,
                   const RRTStarParams& params = RRTStarParams());
    
    // Main planning function
    TrajectoryResult plan(const gpmp2::ArmModel& arm_model,
                         const gtsam::Vector& start_conf,
                         const gtsam::Pose3& end_pose,
                         const gtsam::Pose3& base_pose,
                         double total_time_sec,
                         size_t total_time_step,
                         const gpmp2::SignedDistanceField& sdf);
    
    // Configuration-to-configuration planning
    TrajectoryResult planConfigToConfig(const gpmp2::ArmModel& arm_model,
                                       const gtsam::Vector& start_conf,
                                       const gtsam::Vector& goal_conf,
                                       double total_time_sec,
                                       const gpmp2::SignedDistanceField& sdf);
    
    // Parameter setters
    void setParams(const RRTStarParams& params) { params_ = params; }
    RRTStarParams getParams() const { return params_; }
};