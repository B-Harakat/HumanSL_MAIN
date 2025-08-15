#include "utils.h"
#include <fstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

struct WorkspaceEntry {
    gtsam::Vector joint_config;  // 7 DOF joint angles
    gtsam::Pose3 end_effector_pose;
    
    WorkspaceEntry() : joint_config(gtsam::Vector::Zero(7)) {}
    
    WorkspaceEntry(const gtsam::Vector& config, const gtsam::Pose3& pose) 
        : joint_config(config), end_effector_pose(pose) {}
};

// Function to round limit down to nearest 0.5 increment
double roundDownToDiscretization(double limit, double discretization = 0.5) {
    return std::floor(limit / discretization) * discretization;
}

// Function to generate discrete joint values for a given joint
std::vector<double> generateJointValues(double lower_limit, double upper_limit, double discretization = 0.5) {
    std::vector<double> values;
    
    // Handle continuous joints with large limits
    if (lower_limit <= -1e10 || upper_limit >= 1e10) {
        // Use ±2π range for continuous joints (more reasonable for workspace analysis)
        lower_limit = -1.0 * M_PI;
        upper_limit = 1.0 * M_PI;
    }
    
    // Round limits to nearest discretization boundaries
    double start = std::ceil(lower_limit / discretization) * discretization;
    double end = std::floor(upper_limit / discretization) * discretization;
    
    // Generate values within the actual joint limits
    for (double val = start; val <= end; val += discretization) {
        values.push_back(val);
    }
    
    return values;
}

// Function to write a single workspace entry to binary file
void writeWorkspaceEntry(std::ofstream& file, const WorkspaceEntry& entry) {
    // Write joint configuration (7 doubles)
    for (int i = 0; i < 7; i++) {
        double joint_val = entry.joint_config(i);
        file.write(reinterpret_cast<const char*>(&joint_val), sizeof(double));
    }
    
    // Write pose translation (3 doubles)
    gtsam::Point3 translation = entry.end_effector_pose.translation();
    file.write(reinterpret_cast<const char*>(&translation.x()), sizeof(double));
    file.write(reinterpret_cast<const char*>(&translation.y()), sizeof(double));
    file.write(reinterpret_cast<const char*>(&translation.z()), sizeof(double));
    
    // Write pose rotation matrix (9 doubles)
    gtsam::Matrix3 rotation = entry.end_effector_pose.rotation().matrix();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double rot_val = rotation(i, j);
            file.write(reinterpret_cast<const char*>(&rot_val), sizeof(double));
        }
    }
}

// Function to read a single workspace entry from binary file
WorkspaceEntry readWorkspaceEntry(std::ifstream& file) {
    WorkspaceEntry entry;
    
    // Read joint configuration (7 doubles)
    entry.joint_config = gtsam::Vector::Zero(7);
    for (int i = 0; i < 7; i++) {
        double joint_val;
        file.read(reinterpret_cast<char*>(&joint_val), sizeof(double));
        entry.joint_config(i) = joint_val;
    }
    
    // Read pose translation (3 doubles)
    double x, y, z;
    file.read(reinterpret_cast<char*>(&x), sizeof(double));
    file.read(reinterpret_cast<char*>(&y), sizeof(double));
    file.read(reinterpret_cast<char*>(&z), sizeof(double));
    gtsam::Point3 translation(x, y, z);
    
    // Read pose rotation matrix (9 doubles)
    gtsam::Matrix3 rotation;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double rot_val;
            file.read(reinterpret_cast<char*>(&rot_val), sizeof(double));
            rotation(i, j) = rot_val;
        }
    }
    
    // Construct the pose
    entry.end_effector_pose = gtsam::Pose3(gtsam::Rot3(rotation), translation);
    
    return entry;
}

// Function to find joint configuration closest to target pose
gtsam::Vector findClosestJointConfig(const std::string& workspace_file, const gtsam::Pose3& target_pose) {
    std::ifstream file(workspace_file, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open workspace file: " + workspace_file);
    }
    
    // Read number of entries
    size_t num_entries;
    file.read(reinterpret_cast<char*>(&num_entries), sizeof(size_t));
    
    if (num_entries == 0) {
        throw std::runtime_error("Workspace file contains no entries");
    }
    
    double min_distance = std::numeric_limits<double>::max();
    gtsam::Vector best_config;
    bool found_any = false;
    
    // Iterate through all entries to find the closest one
    for (size_t i = 0; i < num_entries; i++) {
        WorkspaceEntry entry = readWorkspaceEntry(file);
        
        // Calculate pose distance (translation + rotation)
        double trans_distance = target_pose.translation().distance(entry.end_effector_pose.translation());
        double rot_distance = target_pose.rotation().logmap(entry.end_effector_pose.rotation()).norm();
        
        // Combined distance metric (translation in meters + rotation in radians)
        double total_distance = trans_distance + rot_distance;
        
        if (!found_any || total_distance < min_distance) {
            min_distance = total_distance;
            best_config = entry.joint_config;
            found_any = true;
        }
    }
    
    file.close();
    
    if (!found_any) {
        throw std::runtime_error("No valid configurations found in workspace file");
    }
    
    return best_config;
}

int main() {
    try {
        std::cout << "Starting workspace analysis..." << std::endl;
        
        // Load configuration files
        std::string config_dir = "../config/";
        std::string joint_limits_path = config_dir + "joint_limits.yaml";
        std::string dh_params_path = config_dir + "dh_params.yaml";
        
        // Load joint limits and DH parameters
        auto [pos_limits, vel_limits] = createJointLimits(joint_limits_path);
        DHParameters dh_params = createDHParams(dh_params_path);
        
        std::cout << "Configuration files loaded successfully." << std::endl;
        
        // Generate discrete values for each joint
        std::vector<std::vector<double>> joint_values(7);
        
        for (int i = 0; i < 7; i++) {
            std::cout << "Joint " << (i+1) << " limits: [" << pos_limits.lower(i) 
                      << ", " << pos_limits.upper(i) << "]" << std::endl;
            joint_values[i] = generateJointValues(pos_limits.lower(i), pos_limits.upper(i), 0.2*M_PI);
            std::cout << "Joint " << (i+1) << ": " << joint_values[i].size() 
                      << " values from " << joint_values[i].front() 
                      << " to " << joint_values[i].back() << " rad" << std::endl;
        }
        
        // Calculate total number of combinations
        size_t total_combinations = 1;
        for (int i = 0; i < 7; i++) {
            total_combinations *= joint_values[i].size();
        }
        std::cout << "Total combinations to evaluate: " << total_combinations << std::endl;
        
        // Open output file for streaming write
        std::string output_file = config_dir + "workspace_lookup_table.bin";
        std::ofstream file(output_file, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + output_file);
        }
        
        // Write number of entries at the beginning (we'll update this later)
        file.write(reinterpret_cast<const char*>(&total_combinations), sizeof(size_t));
        
        gtsam::Pose3 base_pose = gtsam::Pose3::Identity();  // Identity base pose as specified
        
        size_t processed = 0;
        size_t report_interval = total_combinations / 100;  // Report every 1%
        if (report_interval == 0) report_interval = 1000;
        
        // Variables for workspace statistics
        bool first_entry = true;
        double min_x, max_x, min_y, max_y, min_z, max_z;
        
        // Nested loops to iterate through all joint combinations
        for (double j1 : joint_values[0]) {
            for (double j2 : joint_values[1]) {
                for (double j3 : joint_values[2]) {
                    for (double j4 : joint_values[3]) {
                        for (double j5 : joint_values[4]) {
                            for (double j6 : joint_values[5]) {
                                for (double j7 : joint_values[6]) {
                                    // Create joint configuration
                                    gtsam::Vector config(7);
                                    config << j1, j2, j3, j4, j5, j6, j7;
                                    
                                    // Compute forward kinematics
                                    gtsam::Pose3 ee_pose = forwardKinematics(dh_params, config, base_pose);
                                    
                                    // Create and write the entry immediately
                                    WorkspaceEntry entry(config, ee_pose);
                                    writeWorkspaceEntry(file, entry);
                                    
                                    // Update statistics
                                    gtsam::Point3 pos = ee_pose.translation();
                                    if (first_entry) {
                                        min_x = max_x = pos.x();
                                        min_y = max_y = pos.y();
                                        min_z = max_z = pos.z();
                                        first_entry = false;
                                    } else {
                                        min_x = std::min(min_x, pos.x());
                                        max_x = std::max(max_x, pos.x());
                                        min_y = std::min(min_y, pos.y());
                                        max_y = std::max(max_y, pos.y());
                                        min_z = std::min(min_z, pos.z());
                                        max_z = std::max(max_z, pos.z());
                                    }
                                    
                                    processed++;
                                    
                                    // Progress reporting
                                    if (processed % report_interval == 0) {
                                        double progress = (double)processed / total_combinations * 100.0;
                                        std::cout << "Progress: " << std::fixed << std::setprecision(1) 
                                                  << progress << "% (" << processed << "/" 
                                                  << total_combinations << ")" << std::endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        file.close();
        
        std::cout << "Workspace generation completed!" << std::endl;
        std::cout << "Generated " << processed << " workspace entries." << std::endl;
        std::cout << "Workspace data written to: " << output_file << std::endl;
        
        // Print workspace statistics
        std::cout << "\n=== Workspace Statistics ===" << std::endl;
        std::cout << "Workspace bounds:" << std::endl;
        std::cout << "  X: [" << min_x << ", " << max_x << "] (range: " << (max_x - min_x) << ")" << std::endl;
        std::cout << "  Y: [" << min_y << ", " << max_y << "] (range: " << (max_y - min_y) << ")" << std::endl;
        std::cout << "  Z: [" << min_z << ", " << max_z << "] (range: " << (max_z - min_z) << ")" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Workspace analysis completed successfully!" << std::endl;
    return 0;
}