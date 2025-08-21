#include "core.h"
#include "plan.h"

bool plan_action(
    std::atomic<int>& trigger_id,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    JointTrajectory& new_joint_trajectory,
    gtsam::Pose3& cur_pose_target,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex
) {

        gtsam::Pose3 left_base_frame_snapshot;
        gtsam::Pose3 right_base_frame_snapshot;
        TubeInfo tube_info_snapshot;
        HumanInfo human_info_snapshot;
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
        
        switch(trigger_id.load()){
            case 1: // Right arm engages pipe
            {
                double approach_offset_y = 0.4;
                double approach_offset_z = 0.1;
                double approach_time_sec = 3.0;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                                    tube_info_snapshot,
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_1");
                std::cout << "tube pose: ";
                for(auto& k : tube_info_snapshot.centroid) {std::cout<< k << ", ";}
                std::cout << "\n";

                std::cout << "fiunal joint pos: ";
                for(auto& k : new_joint_trajectory.pos.back()) {std::cout<< k << ", ";}
                std::cout << "\n";
                gtsam::Vector dummy_vec = new_joint_trajectory.pos.back() * (M_PI/180);
                gtsam::Pose3 final_pose = forwardKinematics(right_arm.dh_params_, dummy_vec, right_base_frame_snapshot);
                std::cout << "Final pose: " << final_pose << "\n";

                break;
            }
            case 2: // Right arm grasps pipe
            {
                double grasp_offset_y = 0.4;
                double grasp_offset_z = 0.001;
                double grasp_time_sec = 1.5;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.plan_cartesian_z(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, true);

                // right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                // right_arm.plan_joint(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, 
                //             tube_info_snapshot, human_info_snapshot, grasp_offset_y, 
                //             grasp_offset_z, grasp_time_sec, 
                //             GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
            
                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_2");
                
                break;
            }

            case 3: // Right arm moves pipe task space controlled
            {
                double move_time_sec = 3;
                double move_time_step = 10;
                double intermediate_point_percentage = 0.35;
                double intermediate_point_height = 0.25;

                double move_offset_from_base_y = 0.5; // position the gripper furter behind human
                double move_offset_from_human_mid_x = -0.15; // positive means moving the end pose towards human left
                double move_offset_from_human_max_z = 0.25;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;
                // q_cur_right_snapshot[3] = -85;
                // q_cur_right_snapshot[5] = -60;
                
                gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
                gtsam::Pose3 target_pose = right_arm.over_head_pose(human_info_snapshot, right_base_frame_snapshot, start_pose, 
                                                                    move_offset_from_base_y, 
                                                                    move_offset_from_human_mid_x, 
                                                                    move_offset_from_human_max_z);
    

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_task(new_joint_trajectory, start_pose, target_pose, 
                                    right_base_frame_snapshot, q_cur_right_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_3");
                
                break;
            }

            case 4: // Left arm approaches pipe
            {
                double approach_offset_y = 0.35;
                double approach_offset_z = 0.15;
                double approach_time_sec = 3.0;

                gtsam::Pose3 right_ee_pose = right_arm.forward_kinematics(right_base_frame_snapshot, q_cur_right_snapshot); 

                // gtsam::Pose3 target_pose = left_arm.over_head_pipe_pose(right_ee_pose, human_info_snapshot, approach_offset_y, approach_offset_z);

                tube_info_snapshot.centroid = right_ee_pose.translation();
                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, right_base_frame_snapshot, q_cur_right_snapshot);
                // left_arm.plan_joint(new_joint_trajectory, 
                //                     q_init_left, 
                //                     left_base_frame_snapshot, 
                //                     target_pose, 
                //                     approach_time_sec,
                //                     GPMP2_TIMESTEPS,
                //                     JOINT_CONTROL_FREQUENCY);

                left_arm.plan_joint(new_joint_trajectory, q_init_left, left_base_frame_snapshot, 
                                    tube_info_snapshot,
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY, false);

                visualizeTrajectory(new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_4");

                break;
            }

            case 5: // Left arm grasps pipe
            {

                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                left_arm.plan_cartesian_z(new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, true);
                visualizeTrajectory(new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_5");


                break;
            }

            case 6: // Right arm disengages pipe
            {
                
                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.plan_cartesian_z(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
                break;
            }

            case 7: // Right arm moves back to default position
            {
                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                double disengage_time_sec = 3.0;

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_joint(new_joint_trajectory, q_cur_right_snapshot, q_init_right, 
                                    right_base_frame_snapshot, disengage_time_sec, 
                                    GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_7");
                
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
                left_arm.plan_task(new_joint_trajectory, start_pose, target_pose, 
                                    left_base_frame_snapshot, q_cur_left_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY, true);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_8");
                
                break;
            }

            case 9: // Left arm moves to target
            {
                left_chicken_flag.store(true);
                break;
            }

            case 10: // Right arm moves pipe task space controlled
            {
                double move_time_sec = 3;
                double move_time_step = 4;
                double intermediate_point_percentage = 0.35;
                double intermediate_point_height = 0.20;


                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;
                // q_cur_right_snapshot[3] = -85;
                // q_cur_right_snapshot[5] = -60;
                
                gtsam::Pose3 start_pose = left_arm.forward_kinematics(left_base_frame_snapshot,q_cur_left_snapshot);
                
                double move_offset_from_base_y = start_pose.translation.y() - left_base_frame_snapshot.translation.y(); // position the gripper furter behind human
                double move_offset_from_human_mid_x = 0.15; // positive means moving the end pose towards human left
                double move_offset_from_human_max_z = 0.25;
                
                gtsam::Pose3 target_pose = left_arm.over_head_pose(human_info_snapshot, right_base_frame_snapshot, start_pose, 
                                                                    move_offset_from_base_y, 
                                                                    move_offset_from_human_mid_x, 
                                                                    move_offset_from_human_max_z);
    

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_task(new_joint_trajectory, start_pose, target_pose, 
                                    left_base_frame_snapshot, q_cur_left_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_10");
                
                break;
            }


            case 11: // Right arm engages pipe
            {
                double approach_offset_y = 0.4;
                double approach_offset_z = 0.1;
                double approach_time_sec = 3.0;

                gtsam::Pose3 left_ee_pose = left_arm.forward_kinematics(left_base_frame_snapshot, q_cur_left_snapshot); 

                // gtsam::Pose3 target_pose = left_arm.over_head_pipe_pose(right_ee_pose, human_info_snapshot, approach_offset_y, approach_offset_z);

                tube_info_snapshot.centroid = left_ee_pose.translation();

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, true, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_joint(new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, 
                                    tube_info_snapshot,
                                    approach_offset_y, approach_offset_z, 
                                    approach_time_sec, GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_11");
                std::cout << "tube pose: ";
                for(auto& k : tube_info_snapshot.centroid) {std::cout<< k << ", ";}
                std::cout << "\n";

                std::cout << "fiunal joint pos: ";
                for(auto& k : new_joint_trajectory.pos.back()) {std::cout<< k << ", ";}
                std::cout << "\n";
                gtsam::Vector dummy_vec = new_joint_trajectory.pos.back() * (M_PI/180);
                gtsam::Pose3 final_pose = forwardKinematics(right_arm.dh_params_, dummy_vec, right_base_frame_snapshot);
                std::cout << "Final pose: " << final_pose << "\n";

                break;
            }

            case 12: // Right arm grasps pipe
            {
                double grasp_offset_y = 0.4;
                double grasp_offset_z = 0.001;
                double grasp_time_sec = 1.5;

                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;

                right_arm.plan_cartesian_z(new_joint_trajectory, q_cur_right_snapshot, right_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, true);
            
                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_2");
                
                break;
            }

            case 13: // Left arm disengages pipe
            {
                
                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                left_arm.plan_cartesian_z(new_joint_trajectory, q_cur_left_snapshot, left_base_frame_snapshot, tube_info_snapshot, 1.5, JOINT_CONTROL_FREQUENCY, false);
                
                break;
            }

            case 14: // left arm moves back to default position
            {
                std::vector<double> std_vec1(left_joint_trajectory.pos.back().data(), left_joint_trajectory.pos.back().data() + left_joint_trajectory.pos.back().size());
                q_cur_left_snapshot = std_vec1;

                double disengage_time_sec = 3.0;

                left_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, right_base_frame_snapshot, q_cur_right_snapshot);
                left_arm.plan_joint(new_joint_trajectory, q_cur_left_snapshot, q_init_left, 
                                    left_base_frame_snapshot, disengage_time_sec, 
                                    GPMP2_TIMESTEPS, JOINT_CONTROL_FREQUENCY);
                
                visualizeTrajectory(new_joint_trajectory.pos, left_arm.arm_model_logs, left_arm.dataset_logs, left_base_frame_snapshot);
                saveTrajectoryResultToYAML(left_arm.result_logs,"plan_14");
                
                break;
            }

            case 3: // Right arm moves pipe task space controlled
            {
                double move_time_sec = 3;
                double move_time_step = 10;
                double intermediate_point_percentage = 0.35;
                double intermediate_point_height = 0.25;


                std::vector<double> std_vec1(right_joint_trajectory.pos.back().data(), right_joint_trajectory.pos.back().data() + right_joint_trajectory.pos.back().size());
                q_cur_right_snapshot = std_vec1;
                
                gtsam::Pose3 start_pose = right_arm.forward_kinematics(right_base_frame_snapshot,q_cur_right_snapshot);
                gtsam::Pose3 target_pose = right_arm.over_head_pose(human_info_snapshot, right_base_frame_snapshot, start_pose, 
                                                                    move_offset_from_base_y, 
                                                                    move_offset_from_human_mid_x, 
                                                                    move_offset_from_human_max_z);
    

                right_arm.make_sdf(tube_info_snapshot, human_info_snapshot, false, left_base_frame_snapshot, q_cur_left_snapshot);
                right_arm.plan_task(new_joint_trajectory, start_pose, target_pose, 
                                    right_base_frame_snapshot, q_cur_right_snapshot, 
                                    move_time_sec, move_time_step, intermediate_point_percentage, 
                                    intermediate_point_height, JOINT_CONTROL_FREQUENCY);

                visualizeTrajectory(new_joint_trajectory.pos, right_arm.arm_model_logs, right_arm.dataset_logs, right_base_frame_snapshot);
                saveTrajectoryResultToYAML(right_arm.result_logs,"plan_3");
                
                break;
            }
            

            
        }
        
        if(trigger_id.load() == 3){

            std::this_thread::sleep_for(std::chrono::milliseconds(8000));
            
        }
        else if(trigger_id.load() == 4){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else if(trigger_id.load() == 5){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else if(trigger_id.load() == 6){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else if(trigger_id.load() == 7){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else if(trigger_id.load() == 8){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else{
            std::cout << "Pipe approach planned, press Enter to conitnue to execution.";
            std::cin.get();
        }

        if(trigger_id.load() == 1 || trigger_id.load() == 2 || trigger_id.load() == 3 || trigger_id.load() == 6 || trigger_id.load() == 7){
            std::vector<double> cur_conf(new_joint_trajectory.pos.back().data(), new_joint_trajectory.pos.back().data() + new_joint_trajectory.pos.back().size());
            cur_pose_target = right_arm.forward_kinematics(right_base_frame_snapshot, cur_conf);

            right_chicken_flag.store(false);
            right_execution_ongoing_flag.store(true);

            std::lock_guard<std::mutex> lock(trajectory_mutex);
            right_joint_trajectory = std::move(new_joint_trajectory);
        }

        if(trigger_id.load() == 4 || trigger_id.load() == 5 || trigger_id.load() == 8){
            std::vector<double> cur_conf(new_joint_trajectory.pos.back().data(), new_joint_trajectory.pos.back().data() + new_joint_trajectory.pos.back().size());
            cur_pose_target = left_arm.forward_kinematics(right_base_frame_snapshot, cur_conf);

            left_chicken_flag.store(false);
            left_execution_ongoing_flag.store(true);

            std::lock_guard<std::mutex> lock(trajectory_mutex);
            left_joint_trajectory = std::move(new_joint_trajectory);
        }

        return true;  // Success
}


bool state_transition(
    std::atomic<int>& state_idx, 
    std::atomic<int>& prev_state_idx,
    std::shared_mutex& vicon_data_mutex,
    std::shared_mutex& joint_data_mutex,
    gtsam::Pose3& left_base_frame,
    gtsam::Pose3& right_base_frame,
    TubeInfo& tube_info,
    HumanInfo& human_info,
    gtsam::Point3& target_info,
    Eigen::Vector3d& init_tube_pos,
    std::vector<double>& q_cur_left,
    std::vector<double>& q_cur_right,
    std::vector<double>& q_init_left,
    std::vector<double>& q_init_right,
    Gen3Arm& right_arm,
    Gen3Arm& left_arm,
    JointTrajectory& left_joint_trajectory,
    JointTrajectory& right_joint_trajectory,
    JointTrajectory& new_joint_trajectory,
    gtsam::Pose3& cur_pose_target,
    std::atomic<bool>& left_execution_ongoing_flag,
    std::atomic<bool>& right_execution_ongoing_flag,
    std::atomic<bool>& left_chicken_flag,
    std::atomic<bool>& right_chicken_flag,
    std::mutex& trajectory_mutex){

    std::atomic<int> phase_idx;

    if(prev_state_idx.load() == 0 && state_idx.load() == 1){

        phase_idx.store(1);
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
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
        
        phase_idx.store(2);
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    
        phase_idx.store(3);

        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        prev_state_idx.store(1);
    }

    if(prev_state_idx.load() == 1 && state_idx.load() == 2){

        phase_idx.store(4);
    
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
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


        phase_idx.store(5);
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        move_gripper(right_base_cyclic, 0);

        phase_idx.store(6);
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        
        move_gripper(left_base_cyclic, 45);
        

        phase_idx.store(7);
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}

        phase_idx.store(8);
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        while(left_execution_ongoing_flag.load() || right_execution_ongoing_flag.load()){std::this_thread::sleep_for(std::chrono::milliseconds(10));}


        phase_idx.store(9);
        
        plan_action(phase_idx, vicon_data_mutex, joint_data_mutex, left_base_frame, right_base_frame,
                    tube_info, human_info, target_info, init_tube_pos, q_cur_left, q_cur_right, q_init_left, q_init_right, right_arm, left_arm, 
                    left_joint_trajectory, right_joint_trajectory, new_joint_trajectory, target_pose_snapshot, left_execution_ongoing_flag, right_execution_ongoing_flag, left_chicken_flag, right_chicken_flag, trajectory_mutex);
        
        prev_state_idx.store(2);
    }


    }

