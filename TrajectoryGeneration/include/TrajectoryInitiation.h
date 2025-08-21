#ifndef TRAJECTORY_INITIATION_H
#define TRAJECTORY_INITIATION_H
#include "spline.h"

#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Dense>
#include <memory>
#include "utils.h"


class InitializeTrajectory {
private:

    DHParameters dh_params_;

    bool solveIK(const gtsam::Pose3& target_pose, 
                const gtsam::Pose3& base_pose,
                const gtsam::Vector& seed_config,
                gtsam::Vector& result_config,
                int max_attempts = 10,
                double guess_range = 0.5);
    
    bool solveQuik(const gtsam::Pose3& target_pose, 
                    const gtsam::Pose3& base_pose,
                    const gtsam::Vector& seed_config,
                    gtsam::Vector& result_config,
                    int max_attempts = 10,
                    double guess_range = 0.5);
                                
    void wrapAngles(gtsam::Vector& angles, const gtsam::Vector& reference); 

    void wrapAngles(Eigen::Vector<double,7>& angles, const Eigen::Vector<double,7>& reference);

public:
    InitializeTrajectory(DHParameters dh_params);

    gtsam::Values initJointTrajectoryFromTarget(
                                        const gtsam::Vector& start_conf,
                                    const gtsam::Pose3& end_pose,
                                const gtsam::Pose3& base_pose,
                            const size_t total_time_step);
                            
    gtsam::Values initJointTrajectoryFromVicon(
                                        const gtsam::Vector& start_conf,
                                        const TubeInfo& tube_info,
                                        double offset_from_base_y,
                                        double offset_from_tube_z,
                                        const gtsam::Pose3& base_pose,
                                        const size_t total_time_step,
                                        gtsam::Pose3& best_end_pose, bool tune_pose = true);

    
    std::tuple<std::deque<Eigen::VectorXd>, std::deque<Eigen::VectorXd>, std::deque<Eigen::VectorXd>> 
        initTaskSpaceTrajectory(const gtsam::Pose3& start_pose,
                        const gtsam::Pose3& end_pose,
                    const double& duration_sec,
                const double& percentage = 0.25,
            const double& height = 0.23,
        double dt = 0.002);

    gtsam::Values initTaskSpaceTrajectory(const gtsam::Pose3& start_pose,
                                        const gtsam::Pose3& end_pose,
                                    const gtsam::Pose3& base_pose,
                                const gtsam::Vector& start_conf,
                            std::deque<gtsam::Pose3>& pose_trajectory,
                        double percentage,
                      double height,
                    int num_points);
};

#endif // TRAJECTORY_INITIATION_H