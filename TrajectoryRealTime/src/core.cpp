#include "core.h"
#include "GenerateLogs.h"

#define GPMP2_TIMESTEPS 10
#define JOINT_CONTROL_FREQUENCY 500

bool plan_action(
    int trigger_id,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& head_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex,
    bool mirrored
) {

        JointTrajectory left_new_joint_trajectory;
        JointTrajectory right_new_joint_trajectory;

        gtsam::Pose3 left_base_frame_snapshot;
        gtsam::Pose3 right_base_frame_snapshot;
        TubeInfo tube_info_snapshot;
        HumanInfo human_info_snapshot;
        Eigen::Vector3d head_info_snapshot;
        gtsam::Point3 target_info_snapshot;
        std::vector<double> q_cur_left_snapshot;
        std::vector<double> q_cur_right_snapshot;
        
        {
            std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
            left_base_frame_snapshot = left_base_frame;
            right_base_frame_snapshot = right_base_frame; 
            tube_info_snapshot = tube_info;
            human_info_snapshot = human_info;
            target_info_snapshot = target_info;
            head_info_snapshot = head_info;
        }

        {   
            std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
            q_cur_left_snapshot = shiftAngle(q_cur_left);
            q_cur_right_snapshot = shiftAngle(q_cur_right);
        }

        std::cout << "Left angle snap shot: ";
        for(auto& k : q_cur_left_snapshot){std::cout << k << ", ";}
        std::cout << "\n";

        std::cout << "Right angle snap shot: ";
        for(auto& k : q_cur_right_snapshot){std::cout << k << ", ";}
        std::cout << "\n";

        std::cout << "Right Base pose: " << right_base_frame_snapshot << "\n";

        std::cout << "left Base pose: " << left_base_frame_snapshot << "\n";

        bool right_arm_in_motion = false;
        bool left_arm_in_motion = false;

        
        switch(trigger_id){
            case 1: // Right arm engages pipe
            {
                double approach_offset_y = 0.3;
                double approach_offset_z = 0.1;
                double approach_time_sec = 3.0;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                                    tube_info_snapshot,
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY, 0.01,0.01);

                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_1");
                std::cout << "tube pose: ";
                for(auto& k : tube_info_snapshot.centroid) {std::cout<< k << ", ";}
                std::cout << "\n";

                std::cout << "fiunal joint pos: ";
                for(auto& k : right_new_joint_trajectory.pos.back()) {std::cout<< k << ", ";}
                std::cout << "\n";
                gtsam::Vector dummy_vec = right_new_joint_trajectory.pos.back() * (M_PI/180);
                gtsam::Pose3 final_pose = forwardKinematics(right_arm.dh_params_, dummy_vec, right_base_frame_snapshot);
                std::cout << "Final pose: " << final_pose << "\n";
                right_arm_in_motion = true;

                break;
            }
            case 2: // Right arm grasps pipe
            {
                double grasp_time_sec = 1.5;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, grasp_time_sec, JOINT_CONTROL_FREQUENCY, true);
            
                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_2");

                right_arm_in_motion = true;
                
                break;
            }

            case 3: // Right arm moves pipe task space controlled
            {
                double move_time_sec = 6;
                double move_time_step = 5;
                double intermediate_point_percentage = 0.35;
                double intermediate_point_height = 0.25;

                double move_offset_from_base_y = 0.5; // position the gripper furter behind human
                double move_offset_from_head_x = -0.15; // positive means moving the end pose towards human left
                double move_offset_from_head_z = 0.25;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;
                // q_cur_right_snapshot[3] = -85;
                // q_cur_right_snapshot[5] = -60;

                std::cout << "here 1\n";
                
                gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
                gtsam::Pose3 target_pose = right_arm.over_head_pose(head_info_snapshot, right_base_frame_snapshot, start_pose, 
                                                                    move_offset_from_base_y, 
                                                                    move_offset_from_head_x, 
                                                                    move_offset_from_head_z);
    
                std::cout << "Over head target pose: \n";
                std::cout << target_pose << "\n";

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_task(right_new_joint_trajectory, start_pose, target_pose, 
                                    right_base_frame_snapshot, q_cur_right_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY, false, 0.1, 0.01);

                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_3");

                right_arm_in_motion = true;
                
                break;
            }

            case 4: // Left arm approaches pipe
            {
                double approach_offset_y = 0.30;
                double approach_offset_z = 0.15;
                double approach_time_sec = 3.0;

                gtsam::Pose3 right_ee_pose = right_arm.forward_kinematics(right_base_frame_snapshot, q_cur_right_snapshot); 

                tube_info_snapshot.centroid = right_ee_pose.translation();
                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, right_base_frame_snapshot, q_cur_right_snapshot);

                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                left_arm.plan_joint(left_new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, 
                                    tube_info_snapshot,
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY, false, 0.01, 0.05);

                visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_4");
                
                left_arm_in_motion = true;

                break;
            }

            case 5: // Left arm grasps pipe
            {

                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                left_arm.plan_cartesian_z(left_new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, true);
                visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_5");

                left_arm_in_motion = true;
                break;
            }

            case 6: // Right arm disengages pipe
            {
                
                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
                right_arm_in_motion = true;
                break;
            }

            case 7: // Right arm moves back to default position
            {
                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                double disengage_time_sec = 3.0;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(right_new_joint_trajectory, q_cur_right_snapshot, q_init_right, 
                                    right_base_frame_snapshot, disengage_time_sec, 
                                    GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_7");
                
                right_arm_in_motion = true;
                break;
            }

            case 8: // Left arm moves to target
            {
                gtsam::Pose3 start_pose = left_arm.forward_kinematics(left_base_frame_snapshot,q_cur_left_snapshot);
                gtsam::Pose3 target_pose = left_arm.installtion_pose(target_info, start_pose);
                double move_time_sec = 3.0; 
                int move_time_step = 5;
                double intermediate_point_height = 0.0;
                double intermediate_point_percentage = 0.5;

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_task(left_new_joint_trajectory, start_pose, target_pose, 
                                    left_base_frame_snapshot, q_cur_left_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY, true);

                visualizeTrajectory(left_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_8");
                
                left_arm_in_motion = true;
                break;
            }

            case 9: // Left arm moves to target
            {
                left_chicken_flag.store(true);
                break;
            }

            case 10: // left arm moves pipe task space controlled
            {
                double move_time_sec = 3;
                double move_time_step = 4;
                double intermediate_point_percentage = 0.35;
                double intermediate_point_height = 0.20;


                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                gtsam::Pose3 start_pose = left_arm.forward_kinematics(left_base_frame_snapshot,q_cur_left_snapshot);
                
                double move_offset_from_base_y = start_pose.translation().y() - left_base_frame_snapshot.translation().y() + 0.1; // position the gripper furter behind human
                double move_offset_from_head_x = 0.10; // positive means moving the end pose towards human left
                double move_offset_from_head_z = 0.20;
                
                gtsam::Pose3 target_pose = left_arm.over_head_pose(head_info_snapshot, left_base_frame_snapshot, start_pose, 
                                                                    move_offset_from_base_y, 
                                                                    move_offset_from_head_x, 
                                                                    move_offset_from_head_z);
    

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_task(left_new_joint_trajectory, start_pose, target_pose, 
                                    left_base_frame_snapshot, q_cur_left_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_10");

                left_arm_in_motion = true;
                break;
            }


            case 11: // Right arm engages pipe
            {
                double approach_offset_y = 0.5;
                double approach_offset_z = 0.1;
                double approach_time_sec = 3.0;

                gtsam::Pose3 left_ee_pose = left_arm.forward_kinematics(left_base_frame_snapshot, q_cur_left_snapshot); 

                tube_info_snapshot.centroid = left_ee_pose.translation();

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                                    tube_info_snapshot,
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_11");
                std::cout << "tube pose: ";
                for(auto& k : tube_info_snapshot.centroid) {std::cout<< k << ", ";}
                std::cout << "\n";

                std::cout << "fiunal joint pos: ";
                for(auto& k : right_new_joint_trajectory.pos.back()) {std::cout<< k << ", ";}
                std::cout << "\n";
                gtsam::Vector dummy_vec = right_new_joint_trajectory.pos.back() * (M_PI/180);
                gtsam::Pose3 final_pose = forwardKinematics(right_arm.dh_params_, dummy_vec, right_base_frame_snapshot);
                std::cout << "Final pose: " << final_pose << "\n";
                    
                right_arm_in_motion = true;
                break;
            }

            case 12: // Right arm grasps pipe
            {
                double grasp_offset_y = 0.4;
                double grasp_offset_z = 0.001;
                double grasp_time_sec = 1.5;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, true);
            
                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_12");

                right_arm_in_motion = true;
                
                break;
            }

            case 13: // Left arm disengages pipe
            {
                
                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                left_arm.plan_cartesian_z(left_new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
                left_arm_in_motion = true;

                break;
            }

            case 14: // left arm moves back to default position
            {
                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                double disengage_time_sec = 3.0;

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_joint(left_new_joint_trajectory, q_cur_left_snapshot, q_init_left, 
                                    left_base_frame_snapshot, disengage_time_sec, 
                                    GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
                visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_14");

                left_arm_in_motion = true;
                
                break;
            }

            case 15: // Right arm moves pipe task space controlled
            {
                double move_time_sec = 3;
                double move_time_step = 10;
                double intermediate_point_percentage = 0.65;
                double intermediate_point_height = 0.25;


                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;
                
                gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
                gtsam::Point3 target_pos(init_tube_pos.x(), start_pose.translation().y(), init_tube_pos.z());
            
                gtsam::Rot3 base_rot = gtsam::Rot3::Rz(M_PI); // 180 degrees about z-axis
                gtsam::Rot3 target_rot;
                target_rot = base_rot * gtsam::Rot3::Ry(M_PI/4);  // 30 degrees about new y-axis
                
                
                gtsam::Pose3 target_pose(target_rot, target_pos);
    

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_task(right_new_joint_trajectory, start_pose, target_pose, 
                                    right_base_frame_snapshot, q_cur_right_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_15");

                right_arm_in_motion = true;
                
                break;
            }

            case 16: // Right arm disengages pipe
            {
                
                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
                right_arm_in_motion = true;

                break;
            }

            case 17: // Right arm moves back to default position
            {
                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                double disengage_time_sec = 3.0;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(right_new_joint_trajectory, q_cur_right_snapshot, q_init_right, 
                                    right_base_frame_snapshot, disengage_time_sec, 
                                    GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
                visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_17");
                
                right_arm_in_motion = true;

                break;
            }
            
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));


        
        if(right_arm_in_motion){
   
            right_chicken_flag.store(false);
            right_execution_ongoing_flag.store(true);

            if(mirrored && (trigger_id == 1 || trigger_id == 2 || trigger_id == 3 || trigger_id == 15 || trigger_id == 16 || trigger_id == 17)){
                left_new_joint_trajectory = left_joint_trajectory;
                mirror_trajectory(right_new_joint_trajectory, left_new_joint_trajectory);

                if(trigger_id == 3){

                    left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                    std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                    q_cur_left_snapshot = std_vec1;
                    std::vector<double> std_vec2(left_new_joint_trajectory.pos.back().data(), left_new_joint_trajectory.pos.back().data() + left_new_joint_trajectory.pos.back().size());
                    left_arm.plan_joint(left_new_joint_trajectory, 
                        q_cur_left_snapshot, 
                        std_vec2,
                        left_base_frame_snapshot,
                        3.0, 
                        10,
                        JOINT_CONTROL_FREQUENCY); 
                    }

                visualizeTrajectory(left_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, left_base_frame_snapshot);

                std::lock_guard<std::mutex> lock(trajectory_mutex);
                right_joint_trajectory = std::move(right_new_joint_trajectory);
                left_joint_trajectory = std::move(left_new_joint_trajectory);
            }
            else{
                std::lock_guard<std::mutex> lock(trajectory_mutex);
                right_joint_trajectory = std::move(right_new_joint_trajectory);
            }
        }

        if(left_arm_in_motion){

            left_chicken_flag.store(false);
            left_execution_ongoing_flag.store(true);

            if(mirrored && (trigger_id == 8)){
                right_new_joint_trajectory = right_joint_trajectory;
                mirror_trajectory(left_new_joint_trajectory, right_new_joint_trajectory);

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                std::vector<double> std_vec2(right_new_joint_trajectory.pos.back().data(), right_new_joint_trajectory.pos.back().data() + right_new_joint_trajectory.pos.back().size());

                right_arm.plan_joint(right_new_joint_trajectory, 
                     q_cur_right_snapshot, 
                     std_vec2,
                     right_base_frame_snapshot,
                     3.0, 
                     10,
                     JOINT_CONTROL_FREQUENCY); 

                std::lock_guard<std::mutex> lock(trajectory_mutex);
                right_joint_trajectory = std::move(right_new_joint_trajectory);
                left_joint_trajectory = std::move(left_new_joint_trajectory);
            }
            else{
                std::lock_guard<std::mutex> lock(trajectory_mutex);
                left_joint_trajectory = std::move(left_new_joint_trajectory);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        return true;  // Success
}


// bool plan_action_mirror(
//     int trigger_id,
//     std::shared_mutex& vicon_data_mutex,
//     std::shared_mutex& joint_data_mutex,
//     gtsam::Pose3& left_base_frame,
//     gtsam::Pose3& right_base_frame,
//     TubeInfo& tube_info,
//     HumanInfo& human_info,
//     gtsam::Point3& target_info,
//     Eigen::Vector3d& head_info,
//     Eigen::Vector3d& init_tube_pos,
//     std::vector<double>& q_cur_left,
//     std::vector<double>& q_cur_right,
//     std::vector<double>& q_init_left,
//     std::vector<double>& q_init_right,
//     Gen3Arm& right_arm,
//     Gen3Arm& left_arm,
//     JointTrajectory& left_joint_trajectory,
//     JointTrajectory& right_joint_trajectory,
//     std::atomic<bool>& left_execution_ongoing_flag,
//     std::atomic<bool>& right_execution_ongoing_flag,
//     std::atomic<bool>& left_chicken_flag,
//     std::atomic<bool>& right_chicken_flag,
//     std::mutex& trajectory_mutex,
// ) {

//         JointTrajectory left_new_joint_trajectory;
//         JointTrajectory right_new_joint_trajectory;

//         gtsam::Pose3 left_base_frame_snapshot;
//         gtsam::Pose3 right_base_frame_snapshot;
//         TubeInfo tube_info_snapshot;
//         HumanInfo human_info_snapshot;
//         Eigen::Vector3d head_info_snapshot;
//         gtsam::Point3 target_info_snapshot;
//         std::vector<double> q_cur_left_snapshot;
//         std::vector<double> q_cur_right_snapshot;
        
//         {
//             std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
//             left_base_frame_snapshot = left_base_frame;
//             right_base_frame_snapshot = right_base_frame; 
//             tube_info_snapshot = tube_info;
//             human_info_snapshot = human_info;
//             target_info_snapshot = target_info;
//             head_info_snapshot = head_info;
//         }

//         {   
//             std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
//             q_cur_left_snapshot = shiftAngle(q_cur_left);
//             q_cur_right_snapshot = shiftAngle(q_cur_right);
//         }

//         std::cout << "Left angle snap shot: ";
//         for(auto& k : q_cur_left_snapshot){std::cout << k << ", ";}
//         std::cout << "\n";

//         std::cout << "Right angle snap shot: ";
//         for(auto& k : q_cur_right_snapshot){std::cout << k << ", ";}
//         std::cout << "\n";

//         std::cout << "Right Base pose: " << right_base_frame_snapshot << "\n";

//         std::cout << "left Base pose: " << left_base_frame_snapshot << "\n";

//         bool right_arm_in_motion = false;
//         bool left_arm_in_motion = false;

        
//         switch(trigger_id){
//             case 1: // Right arm engages pipe
//             {
//                 double approach_offset_y = 0.3;
//                 double approach_offset_z = 0.1;
//                 double approach_time_sec = 3.0;

//                 right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
//                 right_arm.plan_joint(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
//                                     tube_info_snapshot,
//                                     approach_offset_y, approach_offset_z, 
//                                     approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY, 0.01,0.01);

//                 visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_1");

//                 left_new_joint_trajectory = left_joint_trajectory;
//                 mirror_trajectory(right_new_joint_trajectory, left_new_joint_trajectory);

//                 std::cout << "tube pose: ";
//                 for(auto& k : tube_info_snapshot.centroid) {std::cout<< k << ", ";}
//                 std::cout << "\n";

//                 std::cout << "fiunal joint pos: ";
//                 for(auto& k : right_new_joint_trajectory.pos.back()) {std::cout<< k << ", ";}
//                 std::cout << "\n";
//                 gtsam::Vector dummy_vec = right_new_joint_trajectory.pos.back() * (M_PI/180);
//                 gtsam::Pose3 final_pose = forwardKinematics(right_arm.dh_params_, dummy_vec, right_base_frame_snapshot);
//                 std::cout << "Final pose: " << final_pose << "\n";

//                 right_arm_in_motion = true;
//                 left_arm_in_motion = true;

//                 break;
//             }
//             case 2: // Right arm grasps pipe
//             {
//                 double grasp_time_sec = 1.5;

//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;

//                 right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, grasp_time_sec, JOINT_CONTROL_FREQUENCY, true);
            
//                 visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_2");

//                 left_new_joint_trajectory = left_joint_trajectory;
//                 mirror_trajectory(right_new_joint_trajectory, left_new_joint_trajectory);

//                 right_arm_in_motion = true;
//                 left_arm_in_motion = true;
                
//                 break;
//             }

//             case 3: // Right arm moves pipe task space controlled
//             {
//                 double move_time_sec = 6;
//                 double move_time_step = 3.5;
//                 double intermediate_point_percentage = 0.35;
//                 double intermediate_point_height = 0.25;

//                 double move_offset_from_base_y = 0.5; // position the gripper furter behind human
//                 double move_offset_from_head_x = -0.15; // positive means moving the end pose towards human left
//                 double move_offset_from_head_z = 0.20;

//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;
//                 // q_cur_right_snapshot[3] = -85;
//                 // q_cur_right_snapshot[5] = -60;

//                 std::cout << "here 1\n";
                
//                 gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
//                 gtsam::Pose3 target_pose = right_arm.over_head_pose(head_info_snapshot, right_base_frame_snapshot, start_pose, 
//                                                                     move_offset_from_base_y, 
//                                                                     move_offset_from_head_x, 
//                                                                     move_offset_from_head_z);
    
//                 std::cout << "Over head target pose: \n";
//                 std::cout << target_pose << "\n";

//                 right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
//                 right_arm.plan_task(right_new_joint_trajectory, start_pose, target_pose, 
//                                     right_base_frame_snapshot, q_cur_right_snapshot, 
//                                     move_time_sec, move_time_step, intermediate_point_percentage, 
//                                     intermediate_point_height, JOINT_CONTROL_FREQUENCY, false, 0.1, 0.01);

//                 std::vector<double> final_right(right_new_joint_trajectory.pos.back().data(), 
//                                                 right_new_joint_trajectory.pos.back().data() + 
//                                                     right_new_joint_trajectory.pos.back().size());


//                 double approach_offset_y = 0.30;
//                 double approach_offset_z = 0.15;
//                 double approach_time_sec = 3.5;


//                 gtsam::Pose3 right_ee_pose = right_arm.forward_kinematics(right_base_frame_snapshot, final_right); 

//                 tube_info_snapshot.centroid = right_ee_pose.translation();
//                 left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, final_right);

//                 std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
//                 q_cur_left_snapshot = std_vec1;

//                 left_arm.plan_joint(left_new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, 
//                                     tube_info_snapshot,
//                                     approach_offset_y, approach_offset_z, 
//                                     approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY, false, 0.01, 0.05);
                
                

//                 visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_3");

//                 visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(left_arm.result_logs,"plan_4");
                
//                 left_arm_in_motion = true;
//                 right_arm_in_motion = true;
                
//                 break;
//             }


//             case 5: // Left arm grasps pipe
//             {

//                 std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
//                 q_cur_left_snapshot = std_vec1;

//                 left_arm.plan_cartesian_z(left_new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, true);
//                 visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(left_arm.result_logs,"plan_5");

//                 left_arm_in_motion = true;
//                 break;
//             }

//             case 6: // Right arm disengages pipe
//             {
                
//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;

//                 right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
//                 right_arm_in_motion = true;
//                 break;
//             }


//             case 8: // Left arm moves to target
//             {
//                 gtsam::Pose3 start_pose = left_arm.forward_kinematics(left_base_frame_snapshot,q_cur_left_snapshot);
//                 gtsam::Pose3 target_pose = left_arm.installtion_pose(target_info, start_pose);
//                 double move_time_sec = 3.0; 
//                 int move_time_step = 5;
//                 double intermediate_point_height = 0.0;
//                 double intermediate_point_percentage = 0.5;

//                 left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
//                 left_arm.plan_task(left_new_joint_trajectory, start_pose, target_pose, 
//                                     left_base_frame_snapshot, q_cur_left_snapshot, 
//                                     move_time_sec, move_time_step, intermediate_point_percentage, 
//                                     intermediate_point_height, JOINT_CONTROL_FREQUENCY, true);

//                 visualizeTrajectory(left_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_8");

//                 gtsam::Point3 right_target_pos = target_pose.translation();

//                 double dist_x = target_pose.translation().x() - head_info.x();
//                 double target_x = head_info.x() - dist_x;

//                 gtsam::Point3 right_target_pos(target_x, target_pose.translation().y(), target_pose.translation().z());
//                 gtsam::Point3 right_target_pose(target_pose.rotation(), right_target_pos);

//                 right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
//                 right_arm.plan_task(right_new_joint_trajectory, start_pose, target_pose, 
//                                     right_base_frame_snapshot, q_cur_right_snapshot, 
//                                     move_time_sec, move_time_step, intermediate_point_percentage, 
//                                     intermediate_point_height, JOINT_CONTROL_FREQUENCY, true);


                
//                 left_arm_in_motion = true;
//                 right_arm_in_motion = true;

//                 break;
//             }

//             case 9: // Left arm moves to target
//             {
//                 left_chicken_flag.store(true);
//                 break;
//             }

//             case 10: // left arm moves pipe task space controlled
//             {
//                 double move_time_sec = 3;
//                 double move_time_step = 4;
//                 double intermediate_point_percentage = 0.35;
//                 double intermediate_point_height = 0.20;


//                 std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
//                 q_cur_left_snapshot = std_vec1;

//                 gtsam::Pose3 start_pose = left_arm.forward_kinematics(left_base_frame_snapshot,q_cur_left_snapshot);
                
//                 double move_offset_from_base_y = start_pose.translation().y() - left_base_frame_snapshot.translation().y() + 0.1; // position the gripper furter behind human
//                 double move_offset_from_head_x = 0.10; // positive means moving the end pose towards human left
//                 double move_offset_from_head_z = 0.20;
                
//                 gtsam::Pose3 target_pose = left_arm.over_head_pose(head_info_snapshot, left_base_frame_snapshot, start_pose, 
//                                                                     move_offset_from_base_y, 
//                                                                     move_offset_from_head_x, 
//                                                                     move_offset_from_head_z);
    

//                 left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
//                 left_arm.plan_task(left_new_joint_trajectory, start_pose, target_pose, 
//                                     left_base_frame_snapshot, q_cur_left_snapshot, 
//                                     move_time_sec, move_time_step, intermediate_point_percentage, 
//                                     intermediate_point_height, JOINT_CONTROL_FREQUENCY);

//                 visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(left_arm.result_logs,"plan_10");

//                 std::vector<double> final_left(left_new_joint_trajectory.pos.back().data(), 
//                                                 left_new_joint_trajectory.pos.back().data() + 
//                                                     left_new_joint_trajectory.pos.back().size());


//                 double approach_offset_y = 0.30;
//                 double approach_offset_z = 0.15;
//                 double approach_time_sec = 3.5;


//                 gtsam::Pose3 left_ee_pose = left_arm.forward_kinematics(left_base_frame_snapshot, final_left); 

//                 tube_info_snapshot.centroid = left_ee_pose.translation();
//                 left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, final_left);

//                 std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
//                 q_cur_left_snapshot = std_vec1;

//                 left_arm.plan_joint(left_new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, 
//                                     tube_info_snapshot,
//                                     approach_offset_y, approach_offset_z, 
//                                     approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY, false, 0.01, 0.05);
                

//                 left_arm_in_motion = true;
//                 break;
//             }


//             case 11: // Right arm engages pipe
//             {
//                 double approach_offset_y = 0.5;
//                 double approach_offset_z = 0.1;
//                 double approach_time_sec = 3.0;

//                 gtsam::Pose3 left_ee_pose = left_arm.forward_kinematics(left_base_frame_snapshot, q_cur_left_snapshot); 

//                 tube_info_snapshot.centroid = left_ee_pose.translation();

//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;

//                 right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
//                 right_arm.plan_joint(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
//                                     tube_info_snapshot,
//                                     approach_offset_y, approach_offset_z, 
//                                     approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);

//                 visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_11");
//                 std::cout << "tube pose: ";
//                 for(auto& k : tube_info_snapshot.centroid) {std::cout<< k << ", ";}
//                 std::cout << "\n";

//                 std::cout << "fiunal joint pos: ";
//                 for(auto& k : right_new_joint_trajectory.pos.back()) {std::cout<< k << ", ";}
//                 std::cout << "\n";
//                 gtsam::Vector dummy_vec = right_new_joint_trajectory.pos.back() * (M_PI/180);
//                 gtsam::Pose3 final_pose = forwardKinematics(right_arm.dh_params_, dummy_vec, right_base_frame_snapshot);
//                 std::cout << "Final pose: " << final_pose << "\n";
                    
//                 right_arm_in_motion = true;
//                 break;
//             }

//             case 12: // Right arm grasps pipe
//             {
//                 double grasp_offset_y = 0.4;
//                 double grasp_offset_z = 0.001;
//                 double grasp_time_sec = 1.5;

//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;

//                 right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, true);
            
//                 visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_12");

//                 right_arm_in_motion = true;
                
//                 break;
//             }

//             case 13: // Left arm disengages pipe
//             {
                
//                 std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
//                 q_cur_left_snapshot = std_vec1;

//                 left_arm.plan_cartesian_z(left_new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
//                 left_arm_in_motion = true;

//                 break;
//             }

//             case 14: // left arm moves back to default position
//             {
//                 std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
//                 q_cur_left_snapshot = std_vec1;

//                 double disengage_time_sec = 3.0;

//                 left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
//                 left_arm.plan_joint(left_new_joint_trajectory, q_cur_left_snapshot, q_init_left, 
//                                     left_base_frame_snapshot, disengage_time_sec, 
//                                     GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
//                 visualizeTrajectory(left_new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(left_arm.result_logs,"plan_14");

//                 left_arm_in_motion = true;
                
//                 break;
//             }

//             case 15: // Right arm moves pipe task space controlled
//             {
//                 double move_time_sec = 3;
//                 double move_time_step = 10;
//                 double intermediate_point_percentage = 0.65;
//                 double intermediate_point_height = 0.25;


//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;
                
//                 gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
//                 gtsam::Point3 target_pos(init_tube_pos.x(), start_pose.translation().y(), init_tube_pos.z());
            
//                 gtsam::Rot3 base_rot = gtsam::Rot3::Rz(M_PI); // 180 degrees about z-axis
//                 gtsam::Rot3 target_rot;
//                 target_rot = base_rot * gtsam::Rot3::Ry(M_PI/4);  // 30 degrees about new y-axis
                
                
//                 gtsam::Pose3 target_pose(target_rot, target_pos);
    

//                 right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
//                 right_arm.plan_task(right_new_joint_trajectory, start_pose, target_pose, 
//                                     right_base_frame_snapshot, q_cur_right_snapshot, 
//                                     move_time_sec, move_time_step, intermediate_point_percentage, 
//                                     intermediate_point_height, JOINT_CONTROL_FREQUENCY);

//                 visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_15");

//                 right_arm_in_motion = true;
                
//                 break;
//             }

//             case 16: // Right arm disengages pipe
//             {
                
//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;

//                 right_arm.plan_cartesian_z(right_new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
//                 right_arm_in_motion = true;

//                 break;
//             }

//             case 17: // Right arm moves back to default position
//             {
//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;

//                 double disengage_time_sec = 3.0;

//                 right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
//                 right_arm.plan_joint(right_new_joint_trajectory, q_cur_right_snapshot, q_init_right, 
//                                     right_base_frame_snapshot, disengage_time_sec, 
//                                     GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
//                 visualizeTrajectory(right_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
//                 saveTrajectoryResultToYAML(right_arm.result_logs,"plan_17");
                
//                 right_arm_in_motion = true;

//                 break;
//             }
            
//         }
        
//         std::this_thread::sleep_for(std::chrono::milliseconds(1000));


        
//         if(right_arm_in_motion){
   
//             right_chicken_flag.store(false);
//             right_execution_ongoing_flag.store(true);

//             if(mirrored && (trigger_id == 1 || trigger_id == 2 || trigger_id == 3 || trigger_id == 15 || trigger_id == 16 || trigger_id == 17)){
//                 left_new_joint_trajectory = left_joint_trajectory;
//                 mirror_trajectory(right_new_joint_trajectory, left_new_joint_trajectory);

//                 if(trigger_id == 3){

//                     left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
//                     std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
//                     q_cur_left_snapshot = std_vec1;
//                     std::vector<double> std_vec2(left_new_joint_trajectory.pos.back().data(), left_new_joint_trajectory.pos.back().data() + left_new_joint_trajectory.pos.back().size());
//                     left_arm.plan_joint(left_new_joint_trajectory, 
//                         q_cur_left_snapshot, 
//                         std_vec2,
//                         left_base_frame_snapshot,
//                         3.0, 
//                         10,
//                         JOINT_CONTROL_FREQUENCY); 
//                     }

//                 visualizeTrajectory(left_new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, left_base_frame_snapshot);

//                 std::lock_guard<std::mutex> lock(trajectory_mutex);
//                 right_joint_trajectory = std::move(right_new_joint_trajectory);
//                 left_joint_trajectory = std::move(left_new_joint_trajectory);
//             }
//             else{
//                 std::lock_guard<std::mutex> lock(trajectory_mutex);
//                 right_joint_trajectory = std::move(right_new_joint_trajectory);
//             }
//         }

//         if(left_arm_in_motion){

//             left_chicken_flag.store(false);
//             left_execution_ongoing_flag.store(true);

//             if(mirrored && (trigger_id == 8)){
//                 right_new_joint_trajectory = right_joint_trajectory;
//                 mirror_trajectory(left_new_joint_trajectory, right_new_joint_trajectory);

//                 std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
//                 q_cur_right_snapshot = std_vec1;

//                 std::vector<double> std_vec2(right_new_joint_trajectory.pos.back().data(), right_new_joint_trajectory.pos.back().data() + right_new_joint_trajectory.pos.back().size());

//                 right_arm.plan_joint(right_new_joint_trajectory, 
//                      q_cur_right_snapshot, 
//                      std_vec2,
//                      right_base_frame_snapshot,
//                      3.0, 
//                      10,
//                      JOINT_CONTROL_FREQUENCY); 

//                 std::lock_guard<std::mutex> lock(trajectory_mutex);
//                 right_joint_trajectory = std::move(right_new_joint_trajectory);
//                 left_joint_trajectory = std::move(left_new_joint_trajectory);
//             }
//             else{
//                 std::lock_guard<std::mutex> lock(trajectory_mutex);
//                 left_joint_trajectory = std::move(left_new_joint_trajectory);
//             }
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(10));

//         return true;  // Success
// }


void state_transition(
    std::atomic<int>& state_idx, 
    std::atomic<int>& prev_state_idx,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    std::atomic<bool>& replan_triggered,
    std::atomic<bool>& new_trajectory_ready,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& head_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    k_api::BaseCyclic::BaseCyclicClient* left_base_cyclic,
    k_api::BaseCyclic::BaseCyclicClient* right_base_cyclic,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    JointTrajectory& new_joint_trajectory,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex){

    int phase_idx;
    std::thread replan_thread;

    if(prev_state_idx.load() == 0 && state_idx.load() == 1){

        phase_idx = 1;
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        replan_thread =std::thread([&]() {
            right_arm.replan(
                        right_joint_trajectory, new_joint_trajectory, right_base_frame,
                        std::ref(vicon_data_mutex), std::ref(joint_data_mutex),
                        std::ref(trajectory_mutex), std::ref(replan_triggered), 
                        std::ref(new_trajectory_ready), std::ref(right_execution_ongoing_flag),
                        human_info, tube_info, left_base_frame, q_cur_left, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
                    );
        });
        replan_thread.join();

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        phase_idx = 2;
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        move_gripper(right_base_cyclic, 50);

        phase_idx = 3;

        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        prev_state_idx.store(1);
    }

    if(prev_state_idx.load() == 1 && state_idx.load() == 2){

        phase_idx = 4;
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        replan_thread =std::thread([&]() {
            left_arm.replan(
                        left_joint_trajectory, new_joint_trajectory, left_base_frame,
                        std::ref(vicon_data_mutex), std::ref(joint_data_mutex),
                        std::ref(trajectory_mutex), std::ref(replan_triggered), 
                        std::ref(new_trajectory_ready), std::ref(left_execution_ongoing_flag),
                        human_info, tube_info, right_base_frame, q_cur_right, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
                    );
        });
        replan_thread.detach();

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}


        phase_idx = 5;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        move_gripper(right_base_cyclic, 0);

        phase_idx = 6;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        move_gripper(left_base_cyclic, 45);
        

        phase_idx = 7;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        phase_idx = 8;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}


        phase_idx = 9;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        prev_state_idx.store(2);
    }


    if(prev_state_idx.load() == 2 && state_idx.load() == 3){
        
        phase_idx = 10;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
            
        prev_state_idx.store(3);
    }

    if(prev_state_idx.load() == 3 && state_idx.load() == 2){

        phase_idx = 11;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);

        replan_thread =std::thread([&]() {
            right_arm.replan(
                        right_joint_trajectory, new_joint_trajectory, right_base_frame,
                        std::ref(vicon_data_mutex), std::ref(joint_data_mutex),
                        std::ref(trajectory_mutex), std::ref(replan_triggered), 
                        std::ref(new_trajectory_ready), std::ref(right_execution_ongoing_flag),
                        human_info, tube_info, left_base_frame, q_cur_left, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
                    );
        });
        replan_thread.join();
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        phase_idx = 12;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        move_gripper(left_base_cyclic, 0);

        phase_idx = 13;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        move_gripper(right_base_cyclic, 45);

        phase_idx = 14;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        phase_idx = 15;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        move_gripper(right_base_cyclic, 0);

        phase_idx = 16;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        phase_idx = 17;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        prev_state_idx.store(2);
    }

}

void state_transition_mirrored(
    std::atomic<int>& state_idx, 
    std::atomic<int>& prev_state_idx,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    std::atomic<bool>& replan_triggered,
    std::atomic<bool>& new_trajectory_ready,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& head_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    k_api::BaseCyclic::BaseCyclicClient* left_base_cyclic,
    k_api::BaseCyclic::BaseCyclicClient* right_base_cyclic,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    JointTrajectory& new_joint_trajectory,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex){

    int phase_idx;
    std::thread replan_thread;

    if(prev_state_idx.load() == 0 && state_idx.load() == 1){

        phase_idx = 1;
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
        
        // replan_thread =std::thread([&]() {
        //     right_arm.replan(
        //                 right_joint_trajectory, new_joint_trajectory, right_base_frame,
        //                 std::ref(vicon_data_mutex), std::ref(joint_data_mutex),
        //                 std::ref(trajectory_mutex), std::ref(replan_triggered), 
        //                 std::ref(new_trajectory_ready), std::ref(right_execution_ongoing_flag),
        //                 human_info, tube_info, left_base_frame, q_cur_left, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
        //             );
        // });
        // replan_thread.join();
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10)); std::cout << "left: " << left_execution_ongoing_flag.load() <<" right: " << right_execution_ongoing_flag.load() <<"\n";}
        
        std::cout << "left: " << left_execution_ongoing_flag.load() <<" right: " << right_execution_ongoing_flag.load() <<"\n";
        phase_idx = 2;
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        move_gripper(right_base_cyclic, 50);

        phase_idx = 3;

        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        prev_state_idx.store(1);
    }

    if(prev_state_idx.load() == 1 && state_idx.load() == 2){

        phase_idx = 4;
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        replan_thread =std::thread([&]() {
            left_arm.replan(
                        left_joint_trajectory, new_joint_trajectory, left_base_frame,
                        std::ref(vicon_data_mutex), std::ref(joint_data_mutex),
                        std::ref(trajectory_mutex), std::ref(replan_triggered), 
                        std::ref(new_trajectory_ready), std::ref(left_execution_ongoing_flag),
                        human_info, tube_info, right_base_frame, q_cur_right, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
                    );
        });
        replan_thread.detach();

        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}


        phase_idx = 5;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        move_gripper(right_base_cyclic, 0);

        phase_idx = 6;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        move_gripper(left_base_cyclic, 45);
        
        phase_idx = 8;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}


        phase_idx = 9;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        prev_state_idx.store(2);
    }


    if(prev_state_idx.load() == 2 && state_idx.load() == 3){
        
        phase_idx = 10;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
            
        prev_state_idx.store(3);
    }

    if(prev_state_idx.load() == 3 && state_idx.load() == 2){

        phase_idx = 11;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        replan_thread =std::thread([&]() {
            right_arm.replan(
                        right_joint_trajectory, new_joint_trajectory, right_base_frame,
                        std::ref(vicon_data_mutex), std::ref(joint_data_mutex),
                        std::ref(trajectory_mutex), std::ref(replan_triggered), 
                        std::ref(new_trajectory_ready), std::ref(right_execution_ongoing_flag),
                        human_info, tube_info, left_base_frame, q_cur_left, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY
                    );
        });
        replan_thread.join();
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        phase_idx = 12;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        move_gripper(left_base_cyclic, 0);

        phase_idx = 13;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        move_gripper(right_base_cyclic, 45);

        phase_idx = 15;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        move_gripper(right_base_cyclic, 0);

        phase_idx = 16;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        phase_idx = 17;
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, head_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex, true);
                    
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        prev_state_idx.store(2);
    }

}


void mirror_trajectory(JointTrajectory& original_trajectory, JointTrajectory& mirrored_trajectory, bool target_only){
    if(original_trajectory.pos.empty() || mirrored_trajectory.pos.empty()) {
        return;
    }

    JointTrajectory trajectory_to_be_mirrored = mirrored_trajectory;
    
    // Store actual starting conditions from trajectory_to_be_mirrored
    Eigen::VectorXd actual_start_pos = trajectory_to_be_mirrored.pos.front();
    Eigen::VectorXd actual_start_vel = trajectory_to_be_mirrored.vel.empty() ? 
        Eigen::VectorXd::Zero(actual_start_pos.size()) : trajectory_to_be_mirrored.vel.front();
    
    size_t num_points = original_trajectory.pos.size();
    size_t num_joints = actual_start_pos.size();
    
    // Create mirrored reference trajectory (original with first joint negated)
    std::vector<Eigen::VectorXd> mirrored_ref_pos, mirrored_ref_vel;
    
    for(size_t i = 0; i < num_points; ++i) {
        Eigen::VectorXd ref_pos = original_trajectory.pos[i];
        Eigen::VectorXd ref_vel = original_trajectory.vel.empty() ? 
            Eigen::VectorXd::Zero(ref_pos.size()) : original_trajectory.vel[i];
        
        // Mirror continuous joints (0, 2, 4) - joint 6 not reversed
        for(int j = 0; j < ref_pos.size(); ++j) {
            if(j == 0 || j == 2 || j == 4) {
                ref_pos(j) = -ref_pos(j);
            }
        }
        for(int j = 0; j < ref_vel.size(); ++j) {
            if(j == 0 || j == 2 || j == 4) {
                ref_vel(j) = -ref_vel(j);
            }
        }
        
        mirrored_ref_pos.push_back(ref_pos);
        mirrored_ref_vel.push_back(ref_vel);
    }
    
    // Clear and rebuild the output trajectory
    trajectory_to_be_mirrored.pos.clear();
    trajectory_to_be_mirrored.vel.clear();
    trajectory_to_be_mirrored.acc.clear();
    
    // Define cubic Hermite spline control points with angle wrapping
    Eigen::VectorXd p0 = actual_start_pos;                    // Start position (actual)
    Eigen::VectorXd p1 = mirrored_ref_pos.back();           // End position (mirrored reference end)
    Eigen::VectorXd v0 = actual_start_vel;                   // Start velocity (actual)
    Eigen::VectorXd v1 = mirrored_ref_vel.back();           // End velocity (mirrored reference end)
    
    // Debug prints
    std::cout << "=== MIRROR TRAJECTORY DEBUG ===" << std::endl;
    std::cout << "Initial joint pos (trajectory_to_be_mirrored): ";
    for(int i = 0; i < actual_start_pos.size(); ++i) {
        std::cout << actual_start_pos(i) << " ";
    }
    std::cout << std::endl;
    
    std::cout << "Initial joint pos (original trajectory): ";
    for(int i = 0; i < original_trajectory.pos.front().size(); ++i) {
        std::cout << original_trajectory.pos.front()(i) << " ";
    }
    std::cout << std::endl;
    
    std::cout << "Final joint pos (original trajectory): ";
    for(int i = 0; i < original_trajectory.pos.back().size(); ++i) {
        std::cout << original_trajectory.pos.back()(i) << " ";
    }
    std::cout << std::endl;
    
    std::cout << "Final joint pos (reversed original): ";
    for(int i = 0; i < original_trajectory.pos.back().size(); ++i) {
        if(i == 0 || i == 2 || i == 4) {
            std::cout << -original_trajectory.pos.back()(i) << " ";
        } else {
            std::cout << original_trajectory.pos.back()(i) << " ";
        }
    }
    std::cout << std::endl;
    
    std::cout << "Final joint pos (mirrored trajectory target): ";
    for(int i = 0; i < p1.size(); ++i) {
        std::cout << p1(i) << " ";
    }
    std::cout << std::endl;
    
    if(target_only) {
        // Target-only mode: simple linear interpolation from start to mirrored target
        std::cout << "Using target_only mode - linear interpolation to mirrored target" << std::endl;
        
        for(size_t i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1); // Normalized time [0, 1]
            
            // Linear interpolation: pos = start + t * (end - start)
            Eigen::VectorXd interp_pos = p0 + t * (p1 - p0);
            Eigen::VectorXd interp_vel = v0 + t * (v1 - v0);
            
            mirrored_trajectory.pos.push_back(interp_pos);
            mirrored_trajectory.vel.push_back(interp_vel);
        }
        
        // Compute acceleration using finite differences
        for(size_t i = 0; i < mirrored_trajectory.vel.size(); ++i) {
            Eigen::VectorXd acc;
            
            if(i == 0) {
                // Forward difference for first point
                acc = mirrored_trajectory.vel[1] - mirrored_trajectory.vel[0];
            } else if(i == mirrored_trajectory.vel.size() - 1) {
                // Backward difference for last point
                acc = mirrored_trajectory.vel[i] - mirrored_trajectory.vel[i-1];
            } else {
                // Central difference for middle points
                acc = (mirrored_trajectory.vel[i+1] - mirrored_trajectory.vel[i-1]) * 0.5;
            }
            
            mirrored_trajectory.acc.push_back(acc);
        }
        
        return; // Exit early in target_only mode
    }
    
    // Handle angle wrapping for continuous joints (0, 2, 4, 6)
    auto wrap_angle_diff = [](double target, double start) -> double {
        double diff = target - start;
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        return diff;
    };
    
    // No angle wrapping - use raw mirrored values for all joints
    
    // Generate new trajectory using cubic Hermite spline with reference pattern influence
    for(size_t i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1); // Normalized time [0, 1]
        
        // Standard cubic Hermite basis functions
        double h00 = 2*t*t*t - 3*t*t + 1;       // Start position weight
        double h10 = t*t*t - 2*t*t + t;         // Start velocity weight  
        double h01 = -2*t*t*t + 3*t*t;          // End position weight
        double h11 = t*t*t - t*t;               // End velocity weight
        
        // Base spline interpolation between actual start and mirrored end
        Eigen::VectorXd base_pos = h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1;
        
        // Calculate reference trajectory offset at this time step with angle wrapping
        Eigen::VectorXd ref_offset = mirrored_ref_pos[i] - mirrored_ref_pos[0];
        
        // No angle wrapping - use raw differences for all joints
        
        // Blend the base spline with reference pattern
        // Use exponential decay to transition from following reference pattern to following spline
        double pattern_influence = std::exp(-2.0 * t);  // Starts at 1, decays to ~0
        
        Eigen::VectorXd final_pos = base_pos + pattern_influence * ref_offset;
        
        // No angle wrapping - use raw final positions for all joints
        
        // For velocity: derivative of Hermite spline plus scaled reference velocity pattern
        double dt = 1.0 / (num_points - 1);
        double dh00_dt = (6*t*t - 6*t) / dt;
        double dh10_dt = (3*t*t - 4*t + 1) / dt;
        double dh01_dt = (-6*t*t + 6*t) / dt;
        double dh11_dt = (3*t*t - 2*t) / dt;
        
        Eigen::VectorXd base_vel = dh00_dt * p0 + dh10_dt * v0 + dh01_dt * p1 + dh11_dt * v1;
        Eigen::VectorXd ref_vel_offset = mirrored_ref_vel[i] - mirrored_ref_vel[0];
        
        Eigen::VectorXd final_vel = base_vel + pattern_influence * ref_vel_offset;
        
        trajectory_to_be_mirrored.pos.push_back(final_pos);
        trajectory_to_be_mirrored.vel.push_back(final_vel);
    }
    
    // Compute acceleration using finite differences
    for(size_t i = 0; i < trajectory_to_be_mirrored.vel.size(); ++i) {
        Eigen::VectorXd acc;
        
        if(i == 0) {
            // Forward difference for first point
            acc = trajectory_to_be_mirrored.vel[1] - trajectory_to_be_mirrored.vel[0];
        } else if(i == trajectory_to_be_mirrored.vel.size() - 1) {
            // Backward difference for last point
            acc = trajectory_to_be_mirrored.vel[i] - trajectory_to_be_mirrored.vel[i-1];
        } else {
            // Central difference for middle points
            acc = (trajectory_to_be_mirrored.vel[i+1] - trajectory_to_be_mirrored.vel[i-1]) * 0.5;
        }
        
        trajectory_to_be_mirrored.acc.push_back(acc);
    }
    mirrored_trajectory = std::move(trajectory_to_be_mirrored);
}