#pragma once

#include "utils.h"
#include "GenerateArmModel.h"

#include <unordered_map>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/kinematics/ArmModel.h> 
#include <gtsam/base/Vector.h>

class ViconSDF : public ArmModel {
public:
    // Empty constructor
    ViconSDF();
    
    // Destructor
    ~ViconSDF() = default;
    
    // Main methods for creating obstacles
    std::unique_ptr<GPMP2_OccupancyGrid> HumanObstacleFromVicon(
        const HumanInfo& subject,
        double cell_size,
        const std::array<double, 3>& workspace_size,
        const std::array<double, 3>& workspace_origin);
    
    std::unique_ptr<GPMP2_OccupancyGrid> ArmObstacleFromVicon(
        const GPMP2_OccupancyGrid& original_dataset,
        const std::unique_ptr<gpmp2::ArmModel>& arm_model,
        const gtsam::Vector& other_arm_config,
        double safety_margin);
    
    std::unique_ptr<GPMP2_OccupancyGrid> TubeObstacleFromVicon(
        const GPMP2_OccupancyGrid& original_dataset,
        const TubeInfo& tube_axis,
        double safety_margin);

    // SDF computation methods
    std::unique_ptr<gpmp2::SignedDistanceField> createSDFFromOccupancyGrid(const GPMP2_OccupancyGrid& dataset);

private:
    
    // Helper method for computing SDF
    std::vector<gtsam::Matrix> occupancyGridToSDF(const GPMP2_OccupancyGrid& dataset);
    
    // Helper method for computing 3D Euclidean Distance Transform
    std::vector<std::vector<std::vector<double>>> computeEDT3D(
        const std::vector<std::vector<std::vector<bool>>>& binary_grid);
    

};