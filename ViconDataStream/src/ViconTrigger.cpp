#include "ViconTrigger.h"

void triggerRightGrasp(HumanInfo& human_info, TubeInfo& tube_info, std::atomic<int>& phase_idx){
    static std::deque<gtsam::Point3> rfin_positions;
    
    if(phase_idx.load() == 0){
        rfin_positions.push_back(human_info.RFIN);
        
        if (rfin_positions.size() > 100) {
            rfin_positions.pop_front();
        }
        
        if (rfin_positions.size() == 100) {
            gtsam::Point3 sum(0.0, 0.0, 0.0);
            for (const auto& pos : rfin_positions) {
                sum = sum + pos;
            }
            gtsam::Point3 average = sum / 100.0;
            
            Eigen::Vector3d avg_pos(average.x(), average.y(), average.z());
            
            // Find the corresponding point on the tube at the same Y level as average position
            double y_offset = avg_pos.y() - tube_info.centroid.y();
            Eigen::Vector3d tube_point_at_y = tube_info.centroid + y_offset * tube_info.direction;
            
            // Calculate distance between averaged hand position and corresponding tube point
            double distance = (avg_pos - tube_point_at_y).norm();
            
            if (distance <= 0.05) {
                phase_idx.store(1);
            }
        }
    }
}


void triggerLeftGrasp(HumanInfo& human_info, TubeInfo& tube_info, std::atomic<int>& phase_idx){
    static std::deque<gtsam::Point3> lfin_positions;
    
    if(phase_idx.load() == 1){
        lfin_positions.push_back(human_info.LFIN);
        
        if (lfin_positions.size() > 100) {
            lfin_positions.pop_front();
        }
        
        if (lfin_positions.size() == 100) {
            gtsam::Point3 sum(0.0, 0.0, 0.0);
            for (const auto& pos : lfin_positions) {
                sum = sum + pos;
            }
            gtsam::Point3 average = sum / 100.0;
            
            Eigen::Vector3d avg_pos(average.x(), average.y(), average.z());
            
            // Find the corresponding point on the tube at the same Y level as average position
            double y_offset = avg_pos.y() - tube_info.centroid.y();
            Eigen::Vector3d tube_point_at_y = tube_info.centroid + y_offset * tube_info.direction;
            
            // Calculate distance between averaged hand position and corresponding tube point
            double distance = (avg_pos - tube_point_at_y).norm();
            
            if (distance <= 0.05) {
                phase_idx.store(2);
            }
        }
    }
}


void triggerLeftImpedanceToTarget(HumanInfo& human_info, TubeInfo& tube_info, std::atomic<int>& phase_idx){
    static std::deque<gtsam::Point3> lfin_positions;
    
    if(phase_idx.load() == 1){
        lfin_positions.push_back(human_info.LFIN);
        
        if (lfin_positions.size() > 100) {
            lfin_positions.pop_front();
        }
        
        if (lfin_positions.size() == 100) {
            gtsam::Point3 sum(0.0, 0.0, 0.0);
            for (const auto& pos : lfin_positions) {
                sum = sum + pos;
            }
            gtsam::Point3 average = sum / 100.0;
            
            Eigen::Vector3d avg_pos(average.x(), average.y(), average.z());
            
            // Find the corresponding point on the tube at the same Y level as average position
            double y_offset = avg_pos.y() - tube_info.centroid.y();
            Eigen::Vector3d tube_point_at_y = tube_info.centroid + y_offset * tube_info.direction;
            
            // Calculate distance between averaged hand position and corresponding tube point
            double distance = (avg_pos - tube_point_at_y).norm();
            
            if (distance <= 0.05) {
                phase_idx.store(2);
            }
        }
    }
}