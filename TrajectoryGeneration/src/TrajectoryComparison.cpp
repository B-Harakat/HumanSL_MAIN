#include "TrajectoryComparison.h"
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <random>
#include <yaml-cpp/yaml.h>

TrajectoryComparison::TrajectoryComparison(const std::string& dh_params_path,
                                          const std::string& joint_limits_path) {
    gpmp2_planner_ = std::make_unique<Planner>(dh_params_path, joint_limits_path);
    rrt_star_planner_ = std::make_unique<RRTStarPlanner>(dh_params_path, joint_limits_path);
    smoother_ = std::make_unique<RRTStarSmoother>();
}

ComparisonResult TrajectoryComparison::comparePlanners(const TestScenario& scenario,
                                                      const gpmp2::ArmModel& arm_model,
                                                      const gpmp2::SignedDistanceField& sdf) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Plan with GPMP2
    TrajectoryResult gpmp2_result = gpmp2_planner_->plan(
        arm_model, scenario.start_config, scenario.target_pose, scenario.base_pose,
        scenario.total_time_sec, scenario.total_time_step, sdf);
    
    // Plan with RRT*
    rrt_star_planner_->setParams(scenario.rrt_params);
    TrajectoryResult rrt_result = rrt_star_planner_->plan(
        arm_model, scenario.start_config, scenario.target_pose, scenario.base_pose,
        scenario.total_time_sec, scenario.total_time_step, sdf);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_comparison_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Compute metrics for both planners
    PlannerMetrics gpmp2_metrics = computeMetrics(gpmp2_result, scenario.target_pose, scenario.base_pose,
                                                 arm_model, sdf, "GPMP2");
    PlannerMetrics rrt_metrics = computeMetrics(rrt_result, scenario.target_pose, scenario.base_pose,
                                               arm_model, sdf, "RRT*");
    
    // Analyze and compare results
    ComparisonResult comparison = analyzeResults(gpmp2_metrics, rrt_metrics);
    
    return comparison;
}

std::vector<ComparisonResult> TrajectoryComparison::batchComparison(
    const std::vector<TestScenario>& scenarios,
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf) {
    
    std::vector<ComparisonResult> results;
    results.reserve(scenarios.size());
    
    for (const auto& scenario : scenarios) {
        std::cout << "Running comparison for scenario: " << scenario.name << std::endl;
        
        ComparisonResult result = comparePlanners(scenario, arm_model, sdf);
        results.push_back(result);
        
        // Print brief summary
        std::cout << "  GPMP2 time: " << result.gpmp2_metrics.total_time.count() << "ms"
                  << ", RRT* time: " << result.rrt_star_metrics.total_time.count() << "ms"
                  << ", Winner: " << result.winner_overall << std::endl;
    }
    
    return results;
}

ComparisonResult TrajectoryComparison::monteCarloComparison(
    const TestScenario& base_scenario,
    const gpmp2::ArmModel& arm_model,
    const gpmp2::SignedDistanceField& sdf,
    int num_runs,
    double noise_level) {
    
    std::vector<ComparisonResult> results;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> noise_dist(0.0, noise_level);
    
    for (int run = 0; run < num_runs; ++run) {
        TestScenario noisy_scenario = base_scenario;
        
        // Add noise to start configuration
        for (int i = 0; i < noisy_scenario.start_config.size(); ++i) {
            noisy_scenario.start_config[i] += noise_dist(gen);
        }
        
        // Add noise to target pose
        gtsam::Vector3 pos_noise(noise_dist(gen), noise_dist(gen), noise_dist(gen));
        gtsam::Vector3 rot_noise(noise_dist(gen) * 0.1, noise_dist(gen) * 0.1, noise_dist(gen) * 0.1);
        
        gtsam::Point3 new_position = noisy_scenario.target_pose.translation() + pos_noise;
        gtsam::Rot3 new_rotation = noisy_scenario.target_pose.rotation() * gtsam::Rot3::RzRyRx(rot_noise);
        noisy_scenario.target_pose = gtsam::Pose3(new_rotation, new_position);
        
        ComparisonResult result = comparePlanners(noisy_scenario, arm_model, sdf);
        results.push_back(result);
    }
    
    // Compute average metrics
    ComparisonResult average_result;
    
    for (const auto& result : results) {
        average_result.gpmp2_metrics.planning_time += result.gpmp2_metrics.planning_time;
        average_result.gpmp2_metrics.path_length += result.gpmp2_metrics.path_length;
        average_result.gpmp2_metrics.smoothness_cost += result.gpmp2_metrics.smoothness_cost;
        average_result.gpmp2_metrics.final_pose_error += result.gpmp2_metrics.final_pose_error;
        
        average_result.rrt_star_metrics.planning_time += result.rrt_star_metrics.planning_time;
        average_result.rrt_star_metrics.path_length += result.rrt_star_metrics.path_length;
        average_result.rrt_star_metrics.smoothness_cost += result.rrt_star_metrics.smoothness_cost;
        average_result.rrt_star_metrics.final_pose_error += result.rrt_star_metrics.final_pose_error;
    }
    
    // Divide by number of runs
    average_result.gpmp2_metrics.planning_time /= num_runs;
    average_result.gpmp2_metrics.path_length /= num_runs;
    average_result.gpmp2_metrics.smoothness_cost /= num_runs;
    average_result.gpmp2_metrics.final_pose_error /= num_runs;
    
    average_result.rrt_star_metrics.planning_time /= num_runs;
    average_result.rrt_star_metrics.path_length /= num_runs;
    average_result.rrt_star_metrics.smoothness_cost /= num_runs;
    average_result.rrt_star_metrics.final_pose_error /= num_runs;
    
    return analyzeResults(average_result.gpmp2_metrics, average_result.rrt_star_metrics);
}

PlannerMetrics TrajectoryComparison::computeMetrics(const TrajectoryResult& result,
                                                   const gtsam::Pose3& target_pose,
                                                   const gtsam::Pose3& base_pose,
                                                   const gpmp2::ArmModel& arm_model,
                                                   const gpmp2::SignedDistanceField& sdf,
                                                   const std::string& planner_name) {
    PlannerMetrics metrics;
    
    if (result.trajectory_pos.empty()) {
        metrics.planning_success = false;
        return metrics;
    }
    
    metrics.planning_success = true;
    metrics.planning_time = result.optimization_duration;
    metrics.total_time = result.optimization_duration;
    
    // Path length
    metrics.path_length = computePathLength(result.trajectory_pos);
    
    // Smoothness
    metrics.smoothness_cost = computeSmoothnessCost(result.trajectory_pos);
    
    // Velocity and acceleration metrics
    metrics.max_velocity = computeMaxVelocity(result.trajectory_vel);
    metrics.max_acceleration = computeMaxAcceleration(result.trajectory_pos, result.dt);
    metrics.max_jerk = computeMaxJerk(result.trajectory_pos, result.dt);
    
    // Final pose error
    metrics.final_pose_error = computeFinalPoseError(result, target_pose, base_pose, gpmp2_planner_->dh_params_);
    
    // Obstacle distance
    metrics.min_obstacle_distance = computeMinObstacleDistance(result.trajectory_pos, arm_model, sdf);
    metrics.collision_free = metrics.min_obstacle_distance > 0.0;
    
    // Joint effort
    metrics.total_joint_effort = computeTotalJointEffort(result.trajectory_pos, result.trajectory_vel);
    metrics.max_joint_velocity = computeMaxVelocity(result.trajectory_vel);
    
    return metrics;
}

double TrajectoryComparison::computePathLength(const std::vector<gtsam::Vector>& trajectory) {
    double length = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        length += (trajectory[i] - trajectory[i-1]).norm();
    }
    return length;
}

double TrajectoryComparison::computeSmoothnessCost(const std::vector<gtsam::Vector>& trajectory) {
    if (trajectory.size() < 3) return 0.0;
    
    double cost = 0.0;
    for (size_t i = 1; i < trajectory.size() - 1; ++i) {
        gtsam::Vector acceleration = trajectory[i+1] - 2.0 * trajectory[i] + trajectory[i-1];
        cost += acceleration.squaredNorm();
    }
    return cost;
}

double TrajectoryComparison::computeMaxVelocity(const std::vector<gtsam::Vector>& velocity) {
    double max_vel = 0.0;
    for (const auto& vel : velocity) {
        max_vel = std::max(max_vel, vel.norm());
    }
    return max_vel;
}

double TrajectoryComparison::computeMaxAcceleration(const std::vector<gtsam::Vector>& trajectory, double dt) {
    if (trajectory.size() < 2) return 0.0;
    
    double max_acc = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        gtsam::Vector acc = (trajectory[i] - trajectory[i-1]) / dt;
        max_acc = std::max(max_acc, acc.norm());
    }
    return max_acc;
}

double TrajectoryComparison::computeMaxJerk(const std::vector<gtsam::Vector>& trajectory, double dt) {
    if (trajectory.size() < 3) return 0.0;
    
    double max_jerk = 0.0;
    for (size_t i = 2; i < trajectory.size(); ++i) {
        gtsam::Vector jerk = (trajectory[i] - 2.0 * trajectory[i-1] + trajectory[i-2]) / (dt * dt);
        max_jerk = std::max(max_jerk, jerk.norm());
    }
    return max_jerk;
}

double TrajectoryComparison::computeFinalPoseError(const TrajectoryResult& result,
                                                  const gtsam::Pose3& target_pose,
                                                  const gtsam::Pose3& base_pose,
                                                  const DHParameters& dh_params) {
    if (result.trajectory_pos.empty()) return std::numeric_limits<double>::max();
    
    gtsam::Vector final_config = result.trajectory_pos.back();
    gtsam::Pose3 final_pose = forwardKinematics(dh_params, final_config, base_pose);
    
    gtsam::Vector6 error;
    error << (target_pose.translation() - final_pose.translation()),
             target_pose.rotation().between(final_pose.rotation()).rpy();
    
    return error.norm();
}

double TrajectoryComparison::computeMinObstacleDistance(const std::vector<gtsam::Vector>& trajectory,
                                                       const gpmp2::ArmModel& arm_model,
                                                       const gpmp2::SignedDistanceField& sdf) {
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& config : trajectory) {
        std::vector<gtsam::Point3> sphere_centers;
        arm_model.sphereCenters(config, sphere_centers);
        
        for (size_t i = 0; i < arm_model.nr_body_spheres(); ++i) {
            double distance = sdf.getSignedDistance(sphere_centers[i]);
            min_distance = std::min(min_distance, distance - arm_model.sphere_radius(i));
        }
    }
    
    return min_distance;
}

double TrajectoryComparison::computeTotalJointEffort(const std::vector<gtsam::Vector>& trajectory,
                                                    const std::vector<gtsam::Vector>& velocity) {
    double total_effort = 0.0;
    for (size_t i = 0; i < std::min(trajectory.size(), velocity.size()); ++i) {
        total_effort += velocity[i].squaredNorm();
    }
    return total_effort;
}

ComparisonResult TrajectoryComparison::analyzeResults(const PlannerMetrics& gpmp2_metrics,
                                                     const PlannerMetrics& rrt_metrics) {
    ComparisonResult result;
    result.gpmp2_metrics = gpmp2_metrics;
    result.rrt_star_metrics = rrt_metrics;
    
    // Compute ratios
    if (gpmp2_metrics.path_length > 0) {
        result.path_length_ratio = rrt_metrics.path_length / gpmp2_metrics.path_length;
    }
    
    if (gpmp2_metrics.smoothness_cost > 0) {
        result.smoothness_ratio = rrt_metrics.smoothness_cost / gpmp2_metrics.smoothness_cost;
    }
    
    if (gpmp2_metrics.total_time.count() > 0) {
        result.time_ratio = static_cast<double>(rrt_metrics.total_time.count()) / 
                           static_cast<double>(gpmp2_metrics.total_time.count());
    }
    
    result.accuracy_comparison = rrt_metrics.final_pose_error - gpmp2_metrics.final_pose_error;
    
    // Determine winners
    result.winner_speed = (rrt_metrics.total_time < gpmp2_metrics.total_time) ? "RRT*" : "GPMP2";
    result.winner_smoothness = (rrt_metrics.smoothness_cost < gpmp2_metrics.smoothness_cost) ? "RRT*" : "GPMP2";
    result.winner_accuracy = (rrt_metrics.final_pose_error < gpmp2_metrics.final_pose_error) ? "RRT*" : "GPMP2";
    
    // Overall winner (weighted combination)
    double gpmp2_score = 0.0;
    double rrt_score = 0.0;
    
    if (gpmp2_metrics.planning_success) gpmp2_score += 1.0;
    if (rrt_metrics.planning_success) rrt_score += 1.0;
    
    if (result.winner_speed == "GPMP2") gpmp2_score += 0.3;
    else rrt_score += 0.3;
    
    if (result.winner_smoothness == "GPMP2") gpmp2_score += 0.4;
    else rrt_score += 0.4;
    
    if (result.winner_accuracy == "GPMP2") gpmp2_score += 0.3;
    else rrt_score += 0.3;
    
    result.winner_overall = (gpmp2_score > rrt_score) ? "GPMP2" : "RRT*";
    
    return result;
}

void TrajectoryComparison::generateReport(const std::vector<ComparisonResult>& results,
                                         const std::string& output_file) {
    std::ofstream file(output_file);
    
    file << "# Trajectory Planning Comparison Report\n\n";
    file << "## Summary Statistics\n\n";
    
    int gpmp2_wins = 0, rrt_wins = 0;
    double avg_time_ratio = 0.0, avg_smoothness_ratio = 0.0, avg_path_ratio = 0.0;
    
    for (const auto& result : results) {
        if (result.winner_overall == "GPMP2") gpmp2_wins++;
        else rrt_wins++;
        
        avg_time_ratio += result.time_ratio;
        avg_smoothness_ratio += result.smoothness_ratio;
        avg_path_ratio += result.path_length_ratio;
    }
    
    avg_time_ratio /= results.size();
    avg_smoothness_ratio /= results.size();
    avg_path_ratio /= results.size();
    
    file << "Total Scenarios: " << results.size() << "\n";
    file << "GPMP2 Wins: " << gpmp2_wins << " (" << 100.0 * gpmp2_wins / results.size() << "%)\n";
    file << "RRT* Wins: " << rrt_wins << " (" << 100.0 * rrt_wins / results.size() << "%)\n\n";
    
    file << "Average Time Ratio (RRT*/GPMP2): " << std::fixed << std::setprecision(3) << avg_time_ratio << "\n";
    file << "Average Smoothness Ratio (RRT*/GPMP2): " << avg_smoothness_ratio << "\n";
    file << "Average Path Length Ratio (RRT*/GPMP2): " << avg_path_ratio << "\n\n";
    
    file << "## Detailed Results\n\n";
    file << "| Scenario | Winner | Time Ratio | Smoothness Ratio | Path Ratio |\n";
    file << "|----------|--------|------------|------------------|------------|\n";
    
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& result = results[i];
        file << "| " << i+1 << " | " << result.winner_overall 
             << " | " << std::setprecision(3) << result.time_ratio
             << " | " << result.smoothness_ratio
             << " | " << result.path_length_ratio << " |\n";
    }
    
    file.close();
}

std::vector<TestScenario> TrajectoryComparison::generateReachingScenarios(
    const gpmp2::ArmModel& arm_model,
    const gtsam::Pose3& base_pose,
    int num_scenarios) {
    
    std::vector<TestScenario> scenarios;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Generate random configurations within joint limits
    for (int i = 0; i < num_scenarios; ++i) {
        TestScenario scenario;
        scenario.name = "Reaching_" + std::to_string(i);
        scenario.description = "Random reaching task " + std::to_string(i);
        scenario.base_pose = base_pose;
        
        // Random start configuration
        scenario.start_config = gtsam::Vector(gpmp2_planner_->pos_limits_.lower.size());
        for (int j = 0; j < scenario.start_config.size(); ++j) {
            std::uniform_real_distribution<double> dist(
                gpmp2_planner_->pos_limits_.lower[j], 
                gpmp2_planner_->pos_limits_.upper[j]);
            scenario.start_config[j] = dist(gen);
        }
        
        // Random target pose within workspace
        std::uniform_real_distribution<double> x_dist(0.3, 0.8);
        std::uniform_real_distribution<double> y_dist(-0.5, 0.5);
        std::uniform_real_distribution<double> z_dist(0.1, 0.8);
        std::uniform_real_distribution<double> rot_dist(-M_PI, M_PI);
        
        gtsam::Point3 target_position(x_dist(gen), y_dist(gen), z_dist(gen));
        gtsam::Rot3 target_rotation = gtsam::Rot3::RzRyRx(rot_dist(gen) * 0.3, 
                                                          rot_dist(gen) * 0.3, 
                                                          rot_dist(gen));
        scenario.target_pose = gtsam::Pose3(target_rotation, target_position);
        
        scenarios.push_back(scenario);
    }
    
    return scenarios;
}

void TrajectoryComparison::exportTrajectoriesForVisualization(
    const TrajectoryResult& gpmp2_result,
    const TrajectoryResult& rrt_result,
    const TestScenario& scenario,
    const std::string& output_file) {
    
    YAML::Node data;
    
    // Scenario info
    data["scenario"]["name"] = scenario.name;
    data["scenario"]["description"] = scenario.description;
    
    // GPMP2 trajectory
    for (size_t i = 0; i < gpmp2_result.trajectory_pos.size(); ++i) {
        YAML::Node point;
        for (int j = 0; j < gpmp2_result.trajectory_pos[i].size(); ++j) {
            point.push_back(gpmp2_result.trajectory_pos[i][j]);
        }
        data["gpmp2_trajectory"].push_back(point);
    }
    
    // RRT* trajectory
    for (size_t i = 0; i < rrt_result.trajectory_pos.size(); ++i) {
        YAML::Node point;
        for (int j = 0; j < rrt_result.trajectory_pos[i].size(); ++j) {
            point.push_back(rrt_result.trajectory_pos[i][j]);
        }
        data["rrt_trajectory"].push_back(point);
    }
    
    std::ofstream file(output_file);
    file << data;
}