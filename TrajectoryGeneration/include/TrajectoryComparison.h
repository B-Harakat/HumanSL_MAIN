#pragma once

#include "utils.h"
#include "Planner.h"
#include "RRTStarPlanner.h"
#include "RRTStarSmoothing.h"
#include <chrono>
#include <string>
#include <vector>
#include <unordered_map>

struct PlannerMetrics {
    // Trajectory quality metrics
    double path_length;
    double smoothness_cost;
    double max_velocity;
    double max_acceleration;
    double max_jerk;
    
    // Success metrics
    bool planning_success;
    double final_pose_error;
    double goal_tolerance_achieved;
    
    // Performance metrics
    std::chrono::milliseconds planning_time;
    std::chrono::milliseconds optimization_time;
    std::chrono::milliseconds total_time;
    
    // Algorithm-specific metrics
    int iterations_used;
    double convergence_error;
    
    // Collision metrics
    double min_obstacle_distance;
    bool collision_free;
    
    // Energy/effort metrics
    double total_joint_effort;
    double max_joint_velocity;
    
    PlannerMetrics() : path_length(0.0), smoothness_cost(0.0), max_velocity(0.0), max_acceleration(0.0),
                      max_jerk(0.0), planning_success(false), final_pose_error(0.0), goal_tolerance_achieved(0.0),
                      planning_time(0), optimization_time(0), total_time(0), iterations_used(0),
                      convergence_error(0.0), min_obstacle_distance(0.0), collision_free(false),
                      total_joint_effort(0.0), max_joint_velocity(0.0) {}
};

struct ComparisonResult {
    PlannerMetrics gpmp2_metrics;
    PlannerMetrics rrt_star_metrics;
    
    // Comparative metrics
    double path_length_ratio;      // RRT* / GPMP2
    double smoothness_ratio;       // RRT* / GPMP2
    double time_ratio;             // RRT* / GPMP2
    double accuracy_comparison;    // Pose error difference
    
    std::string winner_overall;
    std::string winner_speed;
    std::string winner_smoothness;
    std::string winner_accuracy;
    
    ComparisonResult() : path_length_ratio(1.0), smoothness_ratio(1.0), time_ratio(1.0), 
                        accuracy_comparison(0.0) {}
};

struct TestScenario {
    std::string name;
    std::string description;
    
    gtsam::Vector start_config;
    gtsam::Pose3 target_pose;
    gtsam::Pose3 base_pose;
    
    double total_time_sec;
    size_t total_time_step;
    
    // Test-specific parameters
    RRTStarParams rrt_params;
    SmoothingParams smoothing_params;
    
    TestScenario() : total_time_sec(5.0), total_time_step(100) {}
};

class TrajectoryComparison {
private:
    std::unique_ptr<Planner> gpmp2_planner_;
    std::unique_ptr<RRTStarPlanner> rrt_star_planner_;
    std::unique_ptr<RRTStarSmoother> smoother_;
    
    // Metric computation functions
    PlannerMetrics computeMetrics(const TrajectoryResult& result,
                                 const gtsam::Pose3& target_pose,
                                 const gtsam::Pose3& base_pose,
                                 const gpmp2::ArmModel& arm_model,
                                 const gpmp2::SignedDistanceField& sdf,
                                 const std::string& planner_name);
    
    double computePathLength(const std::vector<gtsam::Vector>& trajectory);
    double computeSmoothnessCost(const std::vector<gtsam::Vector>& trajectory);
    double computeMaxVelocity(const std::vector<gtsam::Vector>& velocity);
    double computeMaxAcceleration(const std::vector<gtsam::Vector>& trajectory, double dt);
    double computeMaxJerk(const std::vector<gtsam::Vector>& trajectory, double dt);
    double computeFinalPoseError(const TrajectoryResult& result, 
                                const gtsam::Pose3& target_pose,
                                const gtsam::Pose3& base_pose,
                                const DHParameters& dh_params);
    double computeMinObstacleDistance(const std::vector<gtsam::Vector>& trajectory,
                                     const gpmp2::ArmModel& arm_model,
                                     const gpmp2::SignedDistanceField& sdf);
    double computeTotalJointEffort(const std::vector<gtsam::Vector>& trajectory,
                                  const std::vector<gtsam::Vector>& velocity);
    
    // Analysis functions
    ComparisonResult analyzeResults(const PlannerMetrics& gpmp2_metrics,
                                   const PlannerMetrics& rrt_metrics);
    
public:
    TrajectoryComparison(const std::string& dh_params_path,
                        const std::string& joint_limits_path);
    
    // Single scenario comparison
    ComparisonResult comparePlanners(const TestScenario& scenario,
                                    const gpmp2::ArmModel& arm_model,
                                    const gpmp2::SignedDistanceField& sdf);
    
    // Batch comparison across multiple scenarios
    std::vector<ComparisonResult> batchComparison(const std::vector<TestScenario>& scenarios,
                                                 const gpmp2::ArmModel& arm_model,
                                                 const gpmp2::SignedDistanceField& sdf);
    
    // Statistical analysis across multiple runs
    ComparisonResult monteCarloComparison(const TestScenario& base_scenario,
                                         const gpmp2::ArmModel& arm_model,
                                         const gpmp2::SignedDistanceField& sdf,
                                         int num_runs = 10,
                                         double noise_level = 0.1);
    
    // Scenario generators
    std::vector<TestScenario> generateReachingScenarios(const gpmp2::ArmModel& arm_model,
                                                        const gtsam::Pose3& base_pose,
                                                        int num_scenarios = 10);
    
    std::vector<TestScenario> generateObstacleAvoidanceScenarios(const gpmp2::ArmModel& arm_model,
                                                                const gtsam::Pose3& base_pose,
                                                                int num_scenarios = 5);
    
    // Report generation
    void generateReport(const std::vector<ComparisonResult>& results,
                       const std::string& output_file);
    
    void generateDetailedReport(const std::vector<ComparisonResult>& results,
                               const std::vector<TestScenario>& scenarios,
                               const std::string& output_file);
    
    // Visualization support
    void exportTrajectoriesForVisualization(const TrajectoryResult& gpmp2_result,
                                           const TrajectoryResult& rrt_result,
                                           const TestScenario& scenario,
                                           const std::string& output_file);
    
    // Parameter tuning support
    RRTStarParams tuneRRTStarParameters(const std::vector<TestScenario>& scenarios,
                                       const gpmp2::ArmModel& arm_model,
                                       const gpmp2::SignedDistanceField& sdf);
    
    SmoothingParams tuneSmoothingParameters(const std::vector<TestScenario>& scenarios,
                                           const gpmp2::ArmModel& arm_model,
                                           const gpmp2::SignedDistanceField& sdf);
};