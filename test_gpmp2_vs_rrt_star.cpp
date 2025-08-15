#include "RRTStarPlanner.h"
#include "TrajectoryComparison.h"
#include "Planner.h"
#include "GenerateArmModel.h"
#include "Obstacles.h"
#include <iostream>

int main() {
    try {
        // Configuration paths
        std::string dh_params_path = "config/dh_params.yaml";
        std::string joint_limits_path = "config/joint_limits.yaml";
        
        // Create arm model and obstacles
        ArmModel arm_model_generator;
        DHParameters dh_params = createDHParams(dh_params_path);
        gtsam::Pose3 base_pose = gtsam::Pose3();
        auto arm_model = arm_model_generator.createArmModel(base_pose, dh_params);
        
        // Create simple obstacle environment
        Obstacle obstacle_gen;
        GPMP2_OccupancyGrid occupancy_grid(50, 50, 50, -1.0, -1.0, 0.0, 0.04);
        auto sdf_ptr = obstacle_gen.createSDFFromOccupancyGrid(occupancy_grid);
        gpmp2::SignedDistanceField sdf = *sdf_ptr;
        
        // === Example 1: Basic RRT* Planning ===
        std::cout << "=== Example 1: Basic RRT* Planning ===" << std::endl;
        
        RRTStarPlanner rrt_planner(dh_params_path, joint_limits_path);
        
        // Set start configuration and target pose
        gtsam::Vector start_config = gtsam::Vector::Zero(7);
        gtsam::Pose3 target_pose(gtsam::Rot3(), gtsam::Point3(0.5, 0.3, 0.4));
        
        // Plan trajectory
        TrajectoryResult rrt_result = rrt_planner.plan(
            *arm_model, start_config, target_pose, base_pose,
            5.0, 100, sdf);
        
        if (rrt_result.trajectory_pos.empty()) {
            std::cout << "RRT* planning failed!" << std::endl;
        } else {
            std::cout << "RRT* planning succeeded!" << std::endl;
            std::cout << "Trajectory length: " << rrt_result.trajectory_pos.size() << " points" << std::endl;
            std::cout << "Planning time: " << rrt_result.optimization_duration.count() << " ms" << std::endl;
        }
        
        // === Example 2: Parameter Tuning ===
        std::cout << "\n=== Example 2: RRT* Parameter Tuning ===" << std::endl;
        
        RRTStarParams custom_params;
        custom_params.step_size = 0.1;              // Smaller steps for precision
        custom_params.goal_tolerance = 0.02;        // Tighter goal tolerance
        custom_params.rewire_radius = 0.3;          // Smaller rewiring radius
        custom_params.max_iterations = 10000;       // More iterations
        custom_params.goal_bias = 0.2;              // Higher goal bias
        custom_params.optimize_path = true;         // Enable path optimization
        
        rrt_planner.setParams(custom_params);
        
        TrajectoryResult tuned_result = rrt_planner.plan(
            *arm_model, start_config, target_pose, base_pose,
            5.0, 100, sdf);
        
        if (!tuned_result.trajectory_pos.empty()) {
            std::cout << "Tuned RRT* planning succeeded!" << std::endl;
            std::cout << "Planning time: " << tuned_result.optimization_duration.count() << " ms" << std::endl;
        }
        
        // === Example 3: Smoothness Optimization ===
        std::cout << "\n=== Example 3: Advanced Smoothness Optimization ===" << std::endl;
        
        if (!rrt_result.trajectory_pos.empty()) {
            RRTStarSmoother smoother;
            
            SmoothingParams smooth_params;
            smooth_params.smoothness_weight = 50.0;
            smooth_params.obstacle_weight = 100.0;
            // smooth_params.optimize_path = true;  // Not available in this struct
            smoother.setParams(smooth_params);
            
            auto [pos_limits, vel_limits] = createJointLimits(joint_limits_path);
            
            // Apply different smoothing methods
            std::vector<gtsam::Vector> geometric_smooth = smoother.geometricSmoothing(
                rrt_result.trajectory_pos, *arm_model, sdf, 10);
            
            std::vector<gtsam::Vector> shortcut_smooth = smoother.shortcutSmoothing(
                rrt_result.trajectory_pos, *arm_model, sdf, 100);
            
            std::vector<gtsam::Vector> combined_smooth = smoother.combinedSmoothing(
                rrt_result.trajectory_pos, *arm_model, sdf, pos_limits, vel_limits, 5.0, 0.01);
            
            std::cout << "Original path length: " << smoother.computePathLength(rrt_result.trajectory_pos) << std::endl;
            std::cout << "Geometric smoothed length: " << smoother.computePathLength(geometric_smooth) << std::endl;
            std::cout << "Shortcut smoothed length: " << smoother.computePathLength(shortcut_smooth) << std::endl;
            std::cout << "Combined smoothed length: " << smoother.computePathLength(combined_smooth) << std::endl;
            
            std::cout << "Original smoothness cost: " << smoother.computePathSmoothness(rrt_result.trajectory_pos) << std::endl;
            std::cout << "Combined smoothness cost: " << smoother.computePathSmoothness(combined_smooth) << std::endl;
        }
        
        // === Example 4: Comparison with GPMP2 ===
        std::cout << "\n=== Example 4: RRT* vs GPMP2 Comparison ===" << std::endl;
        
        TrajectoryComparison comparator(dh_params_path, joint_limits_path);
        
        // Create test scenario
        TestScenario scenario;
        scenario.name = "Basic Reaching Test";
        scenario.description = "Compare RRT* and GPMP2 for simple reaching task";
        scenario.start_config = start_config;
        scenario.target_pose = target_pose;
        scenario.base_pose = base_pose;
        scenario.total_time_sec = 5.0;
        scenario.total_time_step = 100;
        scenario.rrt_params = custom_params;
        
        ComparisonResult comparison = comparator.comparePlanners(scenario, *arm_model, sdf);
        
        std::cout << "Comparison Results:" << std::endl;
        std::cout << "Overall Winner: " << comparison.winner_overall << std::endl;
        std::cout << "Speed Winner: " << comparison.winner_speed << std::endl;
        std::cout << "Smoothness Winner: " << comparison.winner_smoothness << std::endl;
        std::cout << "Accuracy Winner: " << comparison.winner_accuracy << std::endl;
        
        std::cout << "GPMP2 - Time: " << comparison.gpmp2_metrics.total_time.count() 
                  << "ms, Path Length: " << comparison.gpmp2_metrics.path_length 
                  << ", Smoothness: " << comparison.gpmp2_metrics.smoothness_cost << std::endl;
        std::cout << "RRT* - Time: " << comparison.rrt_star_metrics.total_time.count() 
                  << "ms, Path Length: " << comparison.rrt_star_metrics.path_length 
                  << ", Smoothness: " << comparison.rrt_star_metrics.smoothness_cost << std::endl;
        
        // === Example 5: Batch Testing ===
        std::cout << "\n=== Example 5: Batch Testing Multiple Scenarios ===" << std::endl;
        
        std::vector<TestScenario> scenarios = comparator.generateReachingScenarios(*arm_model, base_pose, 5);
        std::vector<ComparisonResult> batch_results = comparator.batchComparison(scenarios, *arm_model, sdf);
        
        // Generate report
        comparator.generateReport(batch_results, "rrt_vs_gpmp2_comparison_report.md");
        std::cout << "Generated comparison report: rrt_vs_gpmp2_comparison_report.md" << std::endl;
        
        // === Example 6: Configuration-to-Configuration Planning ===
        std::cout << "\n=== Example 6: Configuration-to-Configuration Planning ===" << std::endl;
        
        gtsam::Vector goal_config = gtsam::Vector::Random(7) * 0.5;
        
        TrajectoryResult config_result = rrt_planner.planConfigToConfig(
            *arm_model, start_config, goal_config, 3.0, sdf);
        
        if (!config_result.trajectory_pos.empty()) {
            std::cout << "Config-to-config planning succeeded!" << std::endl;
            std::cout << "Trajectory points: " << config_result.trajectory_pos.size() << std::endl;
            std::cout << "Planning time: " << config_result.optimization_duration.count() << " ms" << std::endl;
        }
        
        std::cout << "\n=== RRT* Implementation Complete! ===" << std::endl;
        std::cout << "Key Features Implemented:" << std::endl;
        std::cout << "✓ Core RRT* algorithm with obstacle avoidance" << std::endl;
        std::cout << "✓ Multiple smoothness optimization methods" << std::endl;
        std::cout << "✓ Compatible with existing GPMP2 interfaces" << std::endl;
        std::cout << "✓ Comprehensive comparison tools" << std::endl;
        std::cout << "✓ Configurable parameters for different scenarios" << std::endl;
        std::cout << "✓ Both pose-based and configuration-based planning" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}