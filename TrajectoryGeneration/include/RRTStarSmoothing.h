#pragma once

#include "utils.h"
#include <vector>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/SignedDistanceField.h>

struct SmoothingParams {
    double smoothness_weight = 10.0;      // Weight for smoothness term
    double obstacle_weight = 100.0;       // Weight for obstacle avoidance
    double endpoint_weight = 1000.0;      // Weight for maintaining endpoints
    double joint_limit_weight = 50.0;     // Weight for joint limits
    double velocity_limit_weight = 50.0;  // Weight for velocity limits
    int max_iterations = 100;             // Maximum optimization iterations
    double convergence_threshold = 1e-6;  // Convergence threshold
    bool maintain_endpoints = true;       // Whether to keep start/end fixed
    
    SmoothingParams() = default;
};

class RRTStarSmoother {
private:
    SmoothingParams params_;
    
    // Smoothness factor for trajectory optimization
    class SmoothnessFactor : public gtsam::NoiseModelFactor3<gtsam::Vector, gtsam::Vector, gtsam::Vector> {
    private:
        double dt_;
        
    public:
        SmoothnessFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3,
                        const gtsam::SharedNoiseModel& model, double dt)
            : NoiseModelFactor3<gtsam::Vector, gtsam::Vector, gtsam::Vector>(model, key1, key2, key3), dt_(dt) {}
        
        gtsam::Vector evaluateError(const gtsam::Vector& q1, const gtsam::Vector& q2, const gtsam::Vector& q3,
                                   boost::optional<gtsam::Matrix&> H1 = boost::none,
                                   boost::optional<gtsam::Matrix&> H2 = boost::none,
                                   boost::optional<gtsam::Matrix&> H3 = boost::none) const;
    };
    
    // Obstacle avoidance factor
    class SimpleObstacleFactor : public gtsam::NoiseModelFactor1<gtsam::Vector> {
    private:
        const gpmp2::ArmModel& arm_model_;
        const gpmp2::SignedDistanceField& sdf_;
        double safety_distance_;
        
    public:
        SimpleObstacleFactor(gtsam::Key key, const gtsam::SharedNoiseModel& model,
                           const gpmp2::ArmModel& arm_model, const gpmp2::SignedDistanceField& sdf,
                           double safety_distance = 0.05)
            : NoiseModelFactor1<gtsam::Vector>(model, key), arm_model_(arm_model), sdf_(sdf), safety_distance_(safety_distance) {}
        
        gtsam::Vector evaluateError(const gtsam::Vector& config,
                                   boost::optional<gtsam::Matrix&> H = boost::none) const;
    };
    
    // Joint limit factor
    class JointLimitFactor : public gtsam::NoiseModelFactor1<gtsam::Vector> {
    private:
        JointLimits limits_;
        double margin_;
        
    public:
        JointLimitFactor(gtsam::Key key, const gtsam::SharedNoiseModel& model,
                        const JointLimits& limits, double margin = 0.1)
            : NoiseModelFactor1<gtsam::Vector>(model, key), limits_(limits), margin_(margin) {}
        
        gtsam::Vector evaluateError(const gtsam::Vector& config,
                                   boost::optional<gtsam::Matrix&> H = boost::none) const;
    };

public:
    RRTStarSmoother(const SmoothingParams& params = SmoothingParams()) : params_(params) {}
    
    // Main smoothing function using optimization
    std::vector<gtsam::Vector> smoothTrajectory(const std::vector<gtsam::Vector>& path,
                                               const gpmp2::ArmModel& arm_model,
                                               const gpmp2::SignedDistanceField& sdf,
                                               const JointLimits& pos_limits,
                                               const JointLimits& vel_limits,
                                               double dt = 0.01);
    
    // Simple geometric smoothing (faster but less optimal)
    std::vector<gtsam::Vector> geometricSmoothing(const std::vector<gtsam::Vector>& path,
                                                 const gpmp2::ArmModel& arm_model,
                                                 const gpmp2::SignedDistanceField& sdf,
                                                 int iterations = 10);
    
    // Spline-based smoothing
    std::vector<gtsam::Vector> splineSmoothing(const std::vector<gtsam::Vector>& path,
                                              const gpmp2::ArmModel& arm_model,
                                              const gpmp2::SignedDistanceField& sdf,
                                              double total_time_sec,
                                              double dt = 0.01);
    
    // Shortcut smoothing (removes unnecessary waypoints)
    std::vector<gtsam::Vector> shortcutSmoothing(const std::vector<gtsam::Vector>& path,
                                                const gpmp2::ArmModel& arm_model,
                                                const gpmp2::SignedDistanceField& sdf,
                                                int max_iterations = 100);
    
    // Combined smoothing approach
    std::vector<gtsam::Vector> combinedSmoothing(const std::vector<gtsam::Vector>& path,
                                                const gpmp2::ArmModel& arm_model,
                                                const gpmp2::SignedDistanceField& sdf,
                                                const JointLimits& pos_limits,
                                                const JointLimits& vel_limits,
                                                double total_time_sec,
                                                double dt = 0.01);
    
    // Utility functions
    double computePathLength(const std::vector<gtsam::Vector>& path);
    double computePathSmoothness(const std::vector<gtsam::Vector>& path);
    bool isValidPath(const std::vector<gtsam::Vector>& path,
                    const gpmp2::ArmModel& arm_model,
                    const gpmp2::SignedDistanceField& sdf);
    
    void setParams(const SmoothingParams& params) { params_ = params; }
    SmoothingParams getParams() const { return params_; }
};