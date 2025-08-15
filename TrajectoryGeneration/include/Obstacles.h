#pragma once

#include "utils.h"
#include "Tube.h"

#include <unordered_map>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>  // Added missing include
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/kinematics/ArmModel.h>  // Added missing include
#include <gtsam/base/Vector.h>


class ObstacleFromC3D{

    protected:

    double cell_size_;
    std::array<double, 3> workspace_size_;
    std::array<double, 3> workspace_origin_;

    public:

    C3D_Dataset createC3DDataset(
        const std::string& c3d_file_path,
        const std::string& subject_prefix,
        int frame_number);

    TubeInfo extractTubeInfoFromC3D(const std::string& c3dFilePath, const int& frame_idx);

    std::unique_ptr<GPMP2_OccupancyGrid> createHumanFromC3D(
        const std::vector<gtsam::Point3>& subject_points,
        double cell_size = 0.02,
        const std::array<double, 3>& workspace_size = {2.0, 3.0, 2.0},
        const std::array<double, 3>& workspace_origin = {-1.0, -1.5, 0.0}
    );

    std::unique_ptr<GPMP2_OccupancyGrid> createTubeFromC3D(
        const GPMP2_OccupancyGrid& original_dataset,
        const TubeInfo& tube_axis,
        double safety_margin = 0.02
    );

    double distancePointToCylinder(const gtsam::Point3& point, 
                              const gtsam::Point3& cylinder_start, 
                              const gtsam::Point3& cylinder_end, 
                              double cylinder_radius);
    
};


class Obstacle : public ObstacleFromC3D {

    private:

    std::vector<std::vector<std::vector<double>>> computeEDT3D(
    const std::vector<std::vector<std::vector<bool>>>& binary_grid);

    public:

    std::unique_ptr<GPMP2_OccupancyGrid> updateGridWithOtherArm(
        const GPMP2_OccupancyGrid& original_dataset,
        const gpmp2::ArmModel& arm_model,  // Fixed type
        const gtsam::Vector& arm_config,
        double safety_margin = 0.05
    );

    std::vector<gtsam::Matrix> occupancyGridToSDF(const GPMP2_OccupancyGrid& dataset);

    std::unique_ptr<gpmp2::SignedDistanceField> createSDFFromOccupancyGrid(const GPMP2_OccupancyGrid& dataset);

};



