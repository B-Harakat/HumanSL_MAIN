#ifndef QUIK_SOLVEIK_H
#define QUIK_SOLVEIK_H

#include "quik/IKSolver.h"
#include "quik/Robot.h"
#include "quik/geometry.h"

#include <iostream>
#include <chrono>
#include <memory>
#include <cmath>
#include <random>
#include <limits>
#include "Eigen/Dense"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/base/Vector.h"
#include <gpmp2/kinematics/ArmModel.h>

using Eigen::Matrix4d;

// Joint limits for Kinova Gen3 
const double JOINT_LIMITS_LOWER[7] = {-1e20, -2.2515, -1e20, -2.5807, -1e20, -2.0996, -1e20};
const double JOINT_LIMITS_UPPER[7] = {1e20, 2.2515, 1e20, 2.5807, 1e20, 2.0996, 1e20};

// Function to check if configuration is within joint limits
bool isWithinJointLimits(const Eigen::Vector<double,7>& config) {
    for(int i = 0; i < 7; i++) {
        if(config(i) < JOINT_LIMITS_LOWER[i] || config(i) > JOINT_LIMITS_UPPER[i]) {
            return false;
        }
    }
    return true;
}

// Function to generate random initial guess within joint limits
Eigen::Vector<double,7> generateRandomInitialGuess() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    Eigen::Vector<double,7> guess;
    for(int i = 0; i < 7; i++) {
        if(JOINT_LIMITS_LOWER[i] > -1e10 && JOINT_LIMITS_UPPER[i] < 1e10) {
            // Joint has limits - sample within them
            std::uniform_real_distribution<double> dist(JOINT_LIMITS_LOWER[i], JOINT_LIMITS_UPPER[i]);
            guess(i) = dist(gen);
        } else {
            // Continuous joint - sample in [-π, π]
            std::uniform_real_distribution<double> dist(-M_PI, M_PI);
            guess(i) = dist(gen);
        }
    }
    return guess;
}

Eigen::Vector<double,7> generateRandomInitialGuess(const Eigen::Vector<double,7>& Q0, 
                                                  double guess_range) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // Fixed perturbation range of ±0.15 radians
    std::uniform_real_distribution<double> perturbation_dist(-guess_range, guess_range);
    
    Eigen::Vector<double,7> guess;
    for(int i = 0; i < 7; i++) {
        // Add random perturbation to Q0
        double perturbation = perturbation_dist(gen);
        guess(i) = Q0(i) + perturbation;
        
        // Clamp to joint limits if joint has limits
        if(JOINT_LIMITS_LOWER[i] > -1e10 && JOINT_LIMITS_UPPER[i] < 1e10) {
            guess(i) = std::max(JOINT_LIMITS_LOWER[i], std::min(guess(i), JOINT_LIMITS_UPPER[i]));
        }
        // For continuous joints, no clamping needed - just use the perturbed value
    }
    return guess;
}


// Function to compute solution quality (lower is better)
double computeSolutionQuality(const Eigen::Vector<double,6>& error, const Eigen::Vector<double,7>& config, const Eigen::Vector<double,7>& Q0_init) {
    double pose_error = error.norm();
    
    // Add penalty for violating joint limits
    double limit_penalty = 0.0;
    for(int i = 0; i < 7; i++) {
        if(config(i) < JOINT_LIMITS_LOWER[i]) {
            limit_penalty += std::pow(JOINT_LIMITS_LOWER[i] - config(i), 2);
        } else if(config(i) > JOINT_LIMITS_UPPER[i]) {
            limit_penalty += std::pow(config(i) - JOINT_LIMITS_UPPER[i], 2);
        }
    }
    
    return pose_error + 10.0 * limit_penalty; // Weight limit violations heavily, minimize distance from Q0_init
}

#endif // QUIK_SOLVEIK_H