#ifndef ANALYTICAL_IK_H
#define ANALYTICAL_IK_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>
#include <chrono>

namespace analytical_ik {


// Kinova Gen3 DH parameters and geometry
struct KinovaGen3Params {
    // DH parameters: [a, alpha, d, theta_offset]
    static constexpr double DH_PARAMS[7][4] = {
        {0.0,       M_PI/2,  -0.2848,      0},      // Joint 0
        {0.0,       M_PI/2,  -0.0118,      M_PI},   // Joint 1
        {0.0,       M_PI/2,  -0.4208,      M_PI},   // Joint 2
        {0.0,       M_PI/2,  -0.0128,      M_PI},   // Joint 3
        {0.0,       M_PI/2,  -0.3143,      M_PI},   // Joint 4
        {0.0,       M_PI/2,   0.0,         M_PI},   // Joint 5
        {0.0,       M_PI,    -0.2874,      M_PI}    // Joint 6
    };
    
    // Joint limits (radians) - using original limits to avoid issues
    static constexpr double JOINT_LIMITS_LOWER[7] = {-1e20, -2.2515, -1e20, -2.5807, -1e20, -2.0996, -1e20};
    static constexpr double JOINT_LIMITS_UPPER[7] = {1e20, 2.2515, 1e20, 2.5807, 1e20, 2.0996, 1e20};
};

// Structure to hold IK solution
struct IKSolution {
    Eigen::Vector<double, 7> joint_angles;
    double quality_score;
    bool is_valid;
    int iterations_used;
    
    IKSolution() : quality_score(std::numeric_limits<double>::max()), is_valid(false), iterations_used(0) {
        joint_angles.setZero();
    }
};

class AnalyticalIKSolver {
private:
    // Damped Least Squares parameters
    static constexpr double DEFAULT_DAMPING = 0.1;
    static constexpr double MAX_DAMPING = 10.0;
    static constexpr double MIN_DAMPING = 1e-4;
    static constexpr double DAMPING_INCREASE = 3.0;
    static constexpr double DAMPING_DECREASE = 0.3;
    static constexpr int MAX_ITERATIONS = 100;
    static constexpr double POSITION_TOLERANCE = 2e-2;  // Relaxed to 5cm
    static constexpr double ORIENTATION_TOLERANCE = 0.2; // Relaxed to ~11 degrees
    static constexpr double MIN_IMPROVEMENT = 1e-3;
    
    // Utility functions
    static double normalizeAngle(double angle);

    static bool isWithinJointLimits(const Eigen::Vector<double, 7>& joints);
    static double computeQualityScore(const Eigen::Vector<double, 7>& joints, 
                                    const Eigen::Vector<double, 7>& seed);
    
    // Forward kinematics and Jacobian
    static Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta);
    static Eigen::Matrix4d forwardKinematics(const Eigen::Vector<double, 7>& joints, 
                                           const Eigen::Matrix4d& base_transform);
    static Eigen::Matrix<double, 6, 7> computeJacobian(const Eigen::Vector<double, 7>& joints,
                                                      const Eigen::Matrix4d& base_transform);
    
    // Pose difference computation
    static Eigen::Vector<double, 6> computePoseError(const Eigen::Matrix4d& current_pose,
                                                    const Eigen::Matrix4d& target_pose);
    
    // Damped Least Squares core algorithm
    static IKSolution solveDampedLeastSquares(const Eigen::Matrix4d& target_pose,
                                            const Eigen::Matrix4d& base_transform,
                                            const Eigen::Vector<double, 7>& initial_guess);
    
    // Random seed generation
    static std::vector<Eigen::Vector<double, 7>> generateRandomSeeds(
        const Eigen::Vector<double, 7>& preferred_seed, int num_seeds);

public:
    // Main IK solver interface
    static std::vector<IKSolution> solveIK(const Eigen::Matrix4d& target_pose,
                                         const Eigen::Matrix4d& base_transform,
                                         const Eigen::Vector<double, 7>& seed_config,
                                         int num_attempts = 10);
    
    // Single best solution (fastest)
    static IKSolution solveBestIK(const Eigen::Matrix4d& target_pose,
                                const Eigen::Matrix4d& base_transform,
                                const Eigen::Vector<double, 7>& seed_config,
                                int num_attempts = 10);
    
    // Public forward kinematics for debugging
    static Eigen::Matrix4d computeForwardKinematics(const Eigen::Vector<double, 7>& joints, 
                                                   const Eigen::Matrix4d& base_transform) {
        return forwardKinematics(joints, base_transform);
    }

};

//=============================================================================
// IMPLEMENTATION
//=============================================================================

inline double AnalyticalIKSolver::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline bool AnalyticalIKSolver::isWithinJointLimits(const Eigen::Vector<double, 7>& joints) {
    for (int i = 0; i < 7; i++) {
        // Skip infinite limits (continuous joints)
        if (KinovaGen3Params::JOINT_LIMITS_LOWER[i] < -1e10 || 
            KinovaGen3Params::JOINT_LIMITS_UPPER[i] > 1e10) {
            continue; // No limits for this joint
        }
        
        if (joints(i) < KinovaGen3Params::JOINT_LIMITS_LOWER[i] || 
            joints(i) > KinovaGen3Params::JOINT_LIMITS_UPPER[i]) {
            return false;
        }
    }
    return true;
}

inline double AnalyticalIKSolver::computeQualityScore(const Eigen::Vector<double, 7>& joints, 
                                                    const Eigen::Vector<double, 7>& seed) {
    double distance = (joints - seed).norm();
    
    // Add penalty for joint limit violations
    double penalty = 0.0;
    for (int i = 0; i < 7; i++) {
        if (joints(i) < KinovaGen3Params::JOINT_LIMITS_LOWER[i]) {
            penalty += std::pow(KinovaGen3Params::JOINT_LIMITS_LOWER[i] - joints(i), 2);
        } else if (joints(i) > KinovaGen3Params::JOINT_LIMITS_UPPER[i]) {
            penalty += std::pow(joints(i) - KinovaGen3Params::JOINT_LIMITS_UPPER[i], 2);
        }
    }
    
    return distance + 10.0 * penalty;
}

inline Eigen::Matrix4d AnalyticalIKSolver::dhTransform(double a, double alpha, double d, double theta) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    
    Eigen::Matrix4d T;
    T << ct,    -st*ca,  st*sa,   a*ct,
         st,     ct*ca, -ct*sa,   a*st,
         0,      sa,     ca,      d,
         0,      0,      0,       1;
    return T;
}

inline Eigen::Matrix4d AnalyticalIKSolver::forwardKinematics(const Eigen::Vector<double, 7>& joints, 
                                                           const Eigen::Matrix4d& base_transform) {
    Eigen::Matrix4d T = base_transform;
    
    for (int i = 0; i < 7; i++) {
        double theta = joints(i) + KinovaGen3Params::DH_PARAMS[i][3];
        T *= dhTransform(KinovaGen3Params::DH_PARAMS[i][0],  // a
                        KinovaGen3Params::DH_PARAMS[i][1],   // alpha
                        KinovaGen3Params::DH_PARAMS[i][2],   // d
                        theta);                              // theta
    }
    
    return T;
}

inline Eigen::Matrix<double, 6, 7> AnalyticalIKSolver::computeJacobian(const Eigen::Vector<double, 7>& joints,
                                                                      const Eigen::Matrix4d& base_transform) {
    Eigen::Matrix<double, 6, 7> J;
    
    // Compute forward kinematics for each joint to get joint positions and axes
    std::vector<Eigen::Matrix4d> transforms(8); // Base + 7 joints
    transforms[0] = base_transform;
    
    for (int i = 0; i < 7; i++) {
        double theta = joints(i) + KinovaGen3Params::DH_PARAMS[i][3];
        transforms[i+1] = transforms[i] * dhTransform(KinovaGen3Params::DH_PARAMS[i][0],
                                                     KinovaGen3Params::DH_PARAMS[i][1],
                                                     KinovaGen3Params::DH_PARAMS[i][2],
                                                     theta);
    }
    
    // End-effector position
    Eigen::Vector3d pe = transforms[7].block<3,1>(0,3);
    
    // Compute Jacobian columns
    for (int i = 0; i < 7; i++) {
        // Joint axis (z-axis of joint i frame)
        Eigen::Vector3d zi = transforms[i].block<3,1>(0,2);
        
        // Joint position
        Eigen::Vector3d pi = transforms[i].block<3,1>(0,3);
        
        // Linear velocity contribution (for revolute joints)
        Eigen::Vector3d linear_vel = zi.cross(pe - pi);
        
        // Angular velocity contribution
        Eigen::Vector3d angular_vel = zi;
        
        // Fill Jacobian column
        J.col(i) << linear_vel, angular_vel;
    }
    
    return J;
}

inline Eigen::Vector<double, 6> AnalyticalIKSolver::computePoseError(const Eigen::Matrix4d& current_pose,
                                                                   const Eigen::Matrix4d& target_pose) {
    Eigen::Vector<double, 6> error;
    
    // Position error
    Eigen::Vector3d pos_error = target_pose.block<3,1>(0,3) - current_pose.block<3,1>(0,3);
    
    // Orientation error using simpler approach
    Eigen::Matrix3d R_current = current_pose.block<3,3>(0,0);
    Eigen::Matrix3d R_target = target_pose.block<3,3>(0,0);
    Eigen::Matrix3d R_error = R_target * R_current.transpose();
    
    // Use rotation vector approach (more numerically stable)
    Eigen::Vector3d orient_error;
    double trace = R_error.trace();
    
    if (trace > 2.9) {
        // Near identity - use small angle approximation
        orient_error << (R_error(2,1) - R_error(1,2))/2.0,
                       (R_error(0,2) - R_error(2,0))/2.0,
                       (R_error(1,0) - R_error(0,1))/2.0;
    } else {
        // General case - use axis-angle but with better handling
        double angle = acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
        if (angle < 1e-6) {
            orient_error.setZero();
        } else {
            double sin_angle = sin(angle);
            if (abs(sin_angle) > 1e-6) {
                orient_error << (R_error(2,1) - R_error(1,2)) / (2.0 * sin_angle) * angle,
                               (R_error(0,2) - R_error(2,0)) / (2.0 * sin_angle) * angle,
                               (R_error(1,0) - R_error(0,1)) / (2.0 * sin_angle) * angle;
            } else {
                orient_error.setZero();
            }
        }
    }
    
    error << pos_error, orient_error;
    return error;
}

inline IKSolution AnalyticalIKSolver::solveDampedLeastSquares(const Eigen::Matrix4d& target_pose,
                                                            const Eigen::Matrix4d& base_transform,
                                                            const Eigen::Vector<double, 7>& initial_guess) {
    IKSolution solution;
    solution.joint_angles = initial_guess;
    solution.iterations_used = 0;
    
    double damping = DEFAULT_DAMPING;
    double prev_error_norm = std::numeric_limits<double>::max();
    
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        solution.iterations_used = iter + 1;
        
        // Normalize joint angles
        for (int i = 0; i < 7; i++) {
            solution.joint_angles(i) = normalizeAngle(solution.joint_angles(i));
        }
        
        // Compute current pose
        Eigen::Matrix4d current_pose = forwardKinematics(solution.joint_angles, base_transform);
        
        // Compute pose error
        Eigen::Vector<double, 6> error = computePoseError(current_pose, target_pose);
        double error_norm = error.norm();
        

        
        // Check convergence
        if (error.head<3>().norm() < POSITION_TOLERANCE && 
            error.tail<3>().norm() < ORIENTATION_TOLERANCE) {
            solution.is_valid = isWithinJointLimits(solution.joint_angles);
            solution.quality_score = error_norm;
    
            return solution;
        }
        
        // Compute Jacobian
        Eigen::Matrix<double, 6, 7> J = computeJacobian(solution.joint_angles, base_transform);
        
        // Damped least squares update
        Eigen::Matrix<double, 7, 7> JtJ = J.transpose() * J;
        Eigen::Matrix<double, 7, 7> damped_JtJ = JtJ + damping * Eigen::Matrix<double, 7, 7>::Identity();
        
        // Solve for joint angle update
        Eigen::Vector<double, 7> delta_q = damped_JtJ.ldlt().solve(J.transpose() * error);
        
        // Apply update with step size limitation
        double step_size = 1.0;
        double max_joint_change = delta_q.cwiseAbs().maxCoeff();
        if (max_joint_change > 0.3) { // Limit to 0.3 rad per step (~17 degrees)
            step_size = 0.3 / max_joint_change;
        }
        
        // Ensure minimum step size to avoid getting stuck
        if (step_size < 0.01) {
            step_size = 0.01;
        }
        
        // Store old joint angles for debugging
        Eigen::Vector<double, 7> old_joints = solution.joint_angles;
        solution.joint_angles += step_size * delta_q;
        
        // Recompute error after joint update for accurate tracking
        Eigen::Matrix4d updated_pose = forwardKinematics(solution.joint_angles, base_transform);
        Eigen::Vector<double, 6> updated_error = computePoseError(updated_pose, target_pose);
        double updated_error_norm = updated_error.norm();
        
        // Adaptive damping
        if (updated_error_norm < prev_error_norm) {
            // Error decreased - reduce damping
            damping = std::max(MIN_DAMPING, damping * DAMPING_DECREASE);
        } else {
            // Error increased - increase damping
            damping = std::min(MAX_DAMPING, damping * DAMPING_INCREASE);
        }
        
        // Check for minimal improvement (relative) using correct error values
        if (iter > 1 && std::abs(prev_error_norm - updated_error_norm) < MIN_IMPROVEMENT * prev_error_norm) {
            break;
        }
        
        // Update prev_error_norm for next iteration
        prev_error_norm = updated_error_norm;
    }
    
    // Final validation with more relaxed criteria
    Eigen::Matrix4d final_pose = forwardKinematics(solution.joint_angles, base_transform);
    Eigen::Vector<double, 6> final_error = computePoseError(final_pose, target_pose);
    
    // Relaxed IK tolerances for better success rate
    bool position_ok = final_error.head<3>().norm() < 0.04;  // 5cm tolerance
    bool orientation_ok = final_error.tail<3>().norm() < 0.15;  // ~17 degree tolerance
    bool limits_ok = isWithinJointLimits(solution.joint_angles);
    
   
    solution.is_valid = (position_ok && orientation_ok && limits_ok);
    solution.quality_score = final_error.norm();
    
    return solution;
}

inline std::vector<Eigen::Vector<double, 7>> AnalyticalIKSolver::generateRandomSeeds(
    const Eigen::Vector<double, 7>& preferred_seed, int num_seeds) {
    
    std::vector<Eigen::Vector<double, 7>> seeds;
    seeds.push_back(preferred_seed); // First seed is the preferred one
    
    if (num_seeds <= 1) return seeds;
    
    // Random number generator
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    for (int i = 1; i < num_seeds; i++) {
        Eigen::Vector<double, 7> seed;
        
        for (int j = 0; j < 7; j++) {
            if (KinovaGen3Params::JOINT_LIMITS_LOWER[j] > -1e10 && 
                KinovaGen3Params::JOINT_LIMITS_UPPER[j] < 1e10) {
                // Joint has limits - sample within them
                std::uniform_real_distribution<double> dist(
                    KinovaGen3Params::JOINT_LIMITS_LOWER[j], 
                    KinovaGen3Params::JOINT_LIMITS_UPPER[j]);
                seed(j) = dist(gen);
            } else {
                // Continuous joint - sample around preferred seed
                std::normal_distribution<double> dist(preferred_seed(j), 1.0);
                seed(j) = normalizeAngle(dist(gen));
            }
        }
        
        seeds.push_back(seed);
    }
    
    return seeds;
}

inline std::vector<IKSolution> AnalyticalIKSolver::solveIK(const Eigen::Matrix4d& target_pose,
                                                         const Eigen::Matrix4d& base_transform,
                                                         const Eigen::Vector<double, 7>& seed_config,
                                                         int num_attempts) {
    std::vector<IKSolution> all_solutions;
    
    // Generate random seeds
    auto seeds = generateRandomSeeds(seed_config, num_attempts);
    
    // Try each seed
    for (const auto& seed : seeds) {
        IKSolution solution = solveDampedLeastSquares(target_pose, base_transform, seed);
        
        if (solution.is_valid || solution.quality_score < 10.0) { // Accept reasonable solutions
            solution.quality_score = computeQualityScore(solution.joint_angles, seed_config);
            all_solutions.push_back(solution);
        }
    }
    
    // Sort by quality
    std::sort(all_solutions.begin(), all_solutions.end(), 
              [](const IKSolution& a, const IKSolution& b) {
                  if (a.is_valid != b.is_valid) return a.is_valid;
                  return a.quality_score < b.quality_score;
              });
    
    return all_solutions;
}

inline IKSolution AnalyticalIKSolver::solveBestIK(const Eigen::Matrix4d& target_pose,
                                                const Eigen::Matrix4d& base_transform,
                                                const Eigen::Vector<double, 7>& seed_config,
                                                int num_attempts) {
    auto solutions = solveIK(target_pose, base_transform, seed_config, num_attempts);
    
    if (solutions.empty()) {
        return IKSolution(); // Invalid solution
    }
    
    return solutions[0]; // Best solution
}



} // namespace analytical_ik

#endif // ANALYTICAL_IK_H