#include "ViconVisualize.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include "GenerateArmModel.h"
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>

void visualizeStaticConfiguration(
    const Eigen::VectorXd& joint_angles,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose) {
    
    std::cout << "Creating static arm visualization with Open3D..." << std::endl;
    std::cout << "Configuration dimensions: " << joint_angles.size() << std::endl;
    std::cout << "Arm spheres: " << arm_model.nr_body_spheres() << std::endl;
    
    // Convert Eigen::VectorXd to gtsam::Vector
    gtsam::Vector configuration(joint_angles.size());
    for (int i = 0; i < joint_angles.size(); ++i) {
        configuration(i) = joint_angles(i)*(M_PI/180.0);
    }
    
    // Compute forward kinematics to get sphere positions
    gtsam::Matrix sphere_centers = arm_model.sphereCentersMat(configuration);
    
    // Get end-effector pose using forward kinematics
    std::vector<gtsam::Pose3> joint_poses;
    arm_model.fk_model().forwardKinematics(configuration, {}, joint_poses);
    gtsam::Pose3 ee_pose = joint_poses.back();
    
    // Get base position
    gtsam::Point3 base_position = base_pose.translation();
    gtsam::Matrix3 base_rotation = base_pose.rotation().matrix();
    
    // Get end-effector position and rotation
    gtsam::Point3 ee_position = ee_pose.translation();
    gtsam::Matrix3 ee_rotation = ee_pose.rotation().matrix();
    
    // Create geometries to visualize
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    
    // Create occupancy grid visualization
    auto occupied_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    size_t occupied_count = 0;
    
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                if (dataset.map[i][j][k] > 0.5f) {
                    double world_x = dataset.origin_x + (i + 0.5) * dataset.cell_size;
                    double world_y = dataset.origin_y + (j + 0.5) * dataset.cell_size;
                    double world_z = dataset.origin_z + (k + 0.5) * dataset.cell_size;
                    
                    // Create cube mesh for each occupied cell
                    auto cube = open3d::geometry::TriangleMesh::CreateBox(
                        dataset.cell_size, dataset.cell_size, dataset.cell_size);
                    cube->Translate(Eigen::Vector3d(world_x - dataset.cell_size/2, 
                                                   world_y - dataset.cell_size/2, 
                                                   world_z - dataset.cell_size/2));
                    cube->PaintUniformColor(Eigen::Vector3d(1.0, 0.2, 0.2)); // Red color
                    
                    *occupied_mesh += *cube;
                    occupied_count++;
                }
            }
        }
    }
    
    if (occupied_count > 0) {
        geometries.push_back(occupied_mesh);
    }
    
    // Create arm spheres
    for (size_t j = 0; j < arm_model.nr_body_spheres(); ++j) {
        auto sphere = open3d::geometry::TriangleMesh::CreateSphere(arm_model.sphere_radius(j));
        sphere->Translate(Eigen::Vector3d(sphere_centers(0, j), 
                                         sphere_centers(1, j), 
                                         sphere_centers(2, j)));
        sphere->PaintUniformColor(Eigen::Vector3d(0.2, 1.0, 0.2)); // Green color
        geometries.push_back(sphere);
    }
    
    // Create coordinate frames
    
    // World origin frame
    auto world_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.3);
    geometries.push_back(world_frame);
    
    // Robot base frame
    auto base_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.25);
    Eigen::Matrix4d base_transform = Eigen::Matrix4d::Identity();
    base_transform.block<3,3>(0,0) = base_rotation;
    base_transform.block<3,1>(0,3) = Eigen::Vector3d(base_position.x(), base_position.y(), base_position.z());
    base_frame->Transform(base_transform);
    geometries.push_back(base_frame);
    
    // End-effector frame
    auto ee_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.15);
    Eigen::Matrix4d ee_transform = Eigen::Matrix4d::Identity();
    ee_transform.block<3,3>(0,0) = ee_rotation;
    ee_transform.block<3,1>(0,3) = Eigen::Vector3d(ee_position.x(), ee_position.y(), ee_position.z());
    ee_frame->Transform(ee_transform);
    geometries.push_back(ee_frame);
    
    std::cout << "Visualization created successfully!" << std::endl;
    std::cout << "- Occupied cells: " << occupied_count << std::endl;
    std::cout << "- Arm spheres: " << arm_model.nr_body_spheres() << std::endl;
    std::cout << "Opening Open3D visualizer..." << std::endl;
    
    // Visualize using Open3D
    open3d::visualization::DrawGeometries(geometries, 
                                         "Static Arm Configuration", 
                                         1600, 900);
}

// void visualizeRealtime(
//     const std::vector<double>& left_arm_angles,
//     const std::vector<double>& right_arm_angles,
//     const gtsam::Pose3& left_base_pose,
//     const gtsam::Pose3& right_base_pose,
//     const std::string& dh_params_path,
//     const TubeInfo& tube_info,
//     const HumanInfo& human_info,
//     std::shared_mutex& joint_data_mutex,
//     std::shared_mutex& vicon_data_mutex) {
    
//     std::cout << "Starting real-time visualization using proper Open3D non-blocking pattern..." << std::endl;
    
//     setenv("DISPLAY", ":0", 1);
//     setenv("XDG_SESSION_TYPE", "x11", 1);
    
//     // Load DH parameters
//     DHParameters dh_params = createDHParams(dh_params_path);
//     ArmModel arm_model_generator;
    
//     // Get initial data snapshots
//     std::vector<double> left_angles_snapshot, right_angles_snapshot;
//     gtsam::Pose3 left_base_snapshot, right_base_snapshot;
//     TubeInfo tube_snapshot;
//     HumanInfo human_snapshot;
    
//     {
//         std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
//         left_angles_snapshot = left_arm_angles;
//         right_angles_snapshot = right_arm_angles;
//     }
//     {
//         std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
//         left_base_snapshot = left_base_pose;
//         right_base_snapshot = right_base_pose;
//         tube_snapshot = tube_info;
//         human_snapshot = human_info;
//     }
    
//     // Create arm models
//     auto left_arm_model = arm_model_generator.createArmModel(left_base_snapshot, dh_params);
//     auto right_arm_model = arm_model_generator.createArmModel(right_base_snapshot, dh_params);
    
//     if (!left_arm_model || !right_arm_model) {
//         std::cout << "Failed to create arm models!" << std::endl;
//         return;
//     }
    
//     std::cout << "Created arm models with " << left_arm_model->nr_body_spheres() 
//               << " left and " << right_arm_model->nr_body_spheres() << " right spheres" << std::endl;
    
//     // Create persistent visualizer - TRUE NON-BLOCKING PATTERN
//     open3d::visualization::Visualizer vis;
//     if (!vis.CreateVisualizerWindow("Real-time Robot Visualization", 1600, 900, 50, 50, true)) {
//         std::cerr << "Failed to create visualizer window!" << std::endl;
//         return;
//     }
    
//     std::cout << "Visualizer window created successfully" << std::endl;
    
//     // Set up camera view
//     auto& view_control = vis.GetViewControl();
//     Eigen::Vector3d look_at_point(
//         (left_base_snapshot.translation().x() + right_base_snapshot.translation().x()) / 2.0,
//         (left_base_snapshot.translation().y() + right_base_snapshot.translation().y()) / 2.0,
//         (left_base_snapshot.translation().z() + right_base_snapshot.translation().z()) / 2.0
//     );
//     view_control.SetFront(Eigen::Vector3d(1, 1, -0.5));
//     view_control.SetLookat(look_at_point);
//     view_control.SetUp(Eigen::Vector3d(0, 0, 1));
//     view_control.SetZoom(0.8); // Increased zoom to see objects better
    
//     std::cout << "Camera look_at_point: [" << look_at_point.x() << ", " 
//               << look_at_point.y() << ", " << look_at_point.z() << "]" << std::endl;
    
//     // Create persistent geometry containers for dynamic updating
//     std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> left_arm_spheres;
//     std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> right_arm_spheres;
    
//     // Calculate initial positions and create geometries
//     gtsam::Vector left_config_init(left_angles_snapshot.size());
//     gtsam::Vector right_config_init(right_angles_snapshot.size());
    
//     for (size_t i = 0; i < left_angles_snapshot.size(); ++i) {
//         left_config_init(i) = left_angles_snapshot[i] * (M_PI/180.0);
//     }
//     for (size_t i = 0; i < right_angles_snapshot.size(); ++i) {
//         right_config_init(i) = right_angles_snapshot[i] * (M_PI/180.0);
//     }
    
//     gtsam::Matrix left_sphere_centers_init = left_arm_model->sphereCentersMat(left_config_init);
//     gtsam::Matrix right_sphere_centers_init = right_arm_model->sphereCentersMat(right_config_init);
    
//     // Create and add left arm spheres to visualizer ONCE
//     for (size_t j = 0; j < left_arm_model->nr_body_spheres(); ++j) {
//         auto sphere = std::make_shared<open3d::geometry::TriangleMesh>(
//             *open3d::geometry::TriangleMesh::CreateSphere(left_arm_model->sphere_radius(j))
//         );
//         sphere->Translate(Eigen::Vector3d(left_sphere_centers_init(0, j),
//                                          left_sphere_centers_init(1, j),
//                                          left_sphere_centers_init(2, j)));
//         sphere->PaintUniformColor(Eigen::Vector3d(0.2, 1.0, 0.2)); // Green
//         left_arm_spheres.push_back(sphere);
//         vis.AddGeometry(sphere, false); // Add to visualizer
//     }
    
//     // Create and add right arm spheres to visualizer ONCE
//     for (size_t j = 0; j < right_arm_model->nr_body_spheres(); ++j) {
//         auto sphere = std::make_shared<open3d::geometry::TriangleMesh>(
//             *open3d::geometry::TriangleMesh::CreateSphere(right_arm_model->sphere_radius(j))
//         );
//         sphere->Translate(Eigen::Vector3d(right_sphere_centers_init(0, j),
//                                          right_sphere_centers_init(1, j),
//                                          right_sphere_centers_init(2, j)));
//         sphere->PaintUniformColor(Eigen::Vector3d(0.2, 0.2, 1.0)); // Blue
//         right_arm_spheres.push_back(sphere);
//         vis.AddGeometry(sphere, false); // Add to visualizer
//     }
    
//     // Add static elements ONCE
//     for (const auto& human_point : human_snapshot.human_points) {
//         auto sphere = std::make_shared<open3d::geometry::TriangleMesh>(
//             *open3d::geometry::TriangleMesh::CreateSphere(0.03)
//         );
//         sphere->Translate(Eigen::Vector3d(human_point.x(), human_point.y(), human_point.z()));
//         sphere->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red
//         vis.AddGeometry(sphere, false);
//     }
    
//     for (const auto& tube_point : tube_snapshot.tube_points) {
//         auto sphere = std::make_shared<open3d::geometry::TriangleMesh>(
//             *open3d::geometry::TriangleMesh::CreateSphere(0.02)
//         );
//         sphere->Translate(tube_point);
//         sphere->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5)); // Grey
//         vis.AddGeometry(sphere, false);
//     }
    
//     if (tube_snapshot.length > 0) {
//         Eigen::Vector3d start_point = tube_snapshot.centroid - (tube_snapshot.length / 2.0) * tube_snapshot.direction;
//         Eigen::Vector3d end_point = tube_snapshot.centroid + (tube_snapshot.length / 2.0) * tube_snapshot.direction;
        
//         auto line_set = std::make_shared<open3d::geometry::LineSet>();
//         line_set->points_.push_back(start_point);
//         line_set->points_.push_back(end_point);
//         line_set->lines_.push_back(Eigen::Vector2i(0, 1));
//         line_set->colors_.push_back(Eigen::Vector3d(0.2, 0.2, 0.2)); // Dark
//         vis.AddGeometry(line_set, false);
//     }
    
//     auto world_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.3);
//     vis.AddGeometry(world_frame, false);
    
//     std::cout << "Initial geometries added to persistent visualizer" << std::endl;
    
//     // Force initial render to ensure window shows content
//     vis.UpdateRender();
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
//     // TRUE NON-BLOCKING ANIMATION LOOP using Open3D UpdateGeometry pattern
//     const auto frame_duration = std::chrono::milliseconds(42); // 24Hz
    
//     int frame_counter = 0;
//     while (frame_counter < 120) { // 5 seconds at 24Hz
//         auto frame_start = std::chrono::steady_clock::now();
        
//         // Get current joint angles with thread-safe access
//         {
//             std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
//             left_angles_snapshot = left_arm_angles;
//             right_angles_snapshot = right_arm_angles;
//         }
        
//         // Convert to radians
//         gtsam::Vector left_config(left_angles_snapshot.size());
//         gtsam::Vector right_config(right_angles_snapshot.size());
        
//         for (size_t i = 0; i < left_angles_snapshot.size(); ++i) {
//             left_config(i) = left_angles_snapshot[i] * (M_PI/180.0);
//         }
//         for (size_t i = 0; i < right_angles_snapshot.size(); ++i) {
//             right_config(i) = right_angles_snapshot[i] * (M_PI/180.0);
//         }
        
//         // Calculate new sphere positions
//         gtsam::Matrix left_sphere_centers = left_arm_model->sphereCentersMat(left_config);
//         gtsam::Matrix right_sphere_centers = right_arm_model->sphereCentersMat(right_config);
        
//         // Update left arm spheres by modifying vertices directly
//         for (size_t j = 0; j < left_arm_spheres.size(); ++j) {
//             // Create sphere at origin with correct radius
//             auto new_sphere = open3d::geometry::TriangleMesh::CreateSphere(left_arm_model->sphere_radius(j));
            
//             // Position it at the new location
//             new_sphere->Translate(Eigen::Vector3d(left_sphere_centers(0, j),
//                                                  left_sphere_centers(1, j),
//                                                  left_sphere_centers(2, j)));
//             new_sphere->PaintUniformColor(Eigen::Vector3d(0.2, 1.0, 0.2)); // Green
            
//             // Update the existing geometry's vertices and colors
//             left_arm_spheres[j]->vertices_ = new_sphere->vertices_;
//             left_arm_spheres[j]->vertex_colors_ = new_sphere->vertex_colors_;
//         }
        
//         // Update right arm spheres by modifying vertices directly  
//         for (size_t j = 0; j < right_arm_spheres.size(); ++j) {
//             // Create sphere at origin with correct radius
//             auto new_sphere = open3d::geometry::TriangleMesh::CreateSphere(right_arm_model->sphere_radius(j));
            
//             // Position it at the new location
//             new_sphere->Translate(Eigen::Vector3d(right_sphere_centers(0, j),
//                                                  right_sphere_centers(1, j),
//                                                  right_sphere_centers(2, j)));
//             new_sphere->PaintUniformColor(Eigen::Vector3d(0.2, 0.2, 1.0)); // Blue
            
//             // Update the existing geometry's vertices and colors
//             right_arm_spheres[j]->vertices_ = new_sphere->vertices_;
//             right_arm_spheres[j]->vertex_colors_ = new_sphere->vertex_colors_;
//         }
        
//         // CRITICAL: Notify visualizer that geometries have been updated
//         for (auto& sphere : left_arm_spheres) {
//             vis.UpdateGeometry(sphere);
//         }
//         for (auto& sphere : right_arm_spheres) {
//             vis.UpdateGeometry(sphere);
//         }
        
//         // TRUE NON-BLOCKING Open3D pattern: PollEvents + UpdateRender
//         if (!vis.PollEvents()) {
//             std::cout << "Visualization window closed by user" << std::endl;
//             break;
//         }
//         vis.UpdateRender();
        
//         if (frame_counter % 24 == 0) {
//             std::cout << "Frame " << frame_counter << " - Right angles: [" 
//                       << (int)right_angles_snapshot[0] << ", " << (int)right_angles_snapshot[1] 
//                       << ", ...] - Non-blocking animation running!" << std::endl;
//         }
        
//         // Maintain 24Hz frame rate
//         auto frame_end = std::chrono::steady_clock::now();
//         auto elapsed = frame_end - frame_start;
//         if (elapsed < frame_duration) {
//             std::this_thread::sleep_for(frame_duration - elapsed);
//         }
        
//         frame_counter++;
//     }
    
//     // Clean shutdown
//     vis.DestroyVisualizerWindow();
//     std::cout << "Real-time non-blocking visualization finished!" << std::endl;
// }

// void visualizeRealtimeConsole(
//     const std::vector<double>& left_arm_angles,
//     const std::vector<double>& right_arm_angles,
//     const gtsam::Pose3& left_base_pose,
//     const gtsam::Pose3& right_base_pose,
//     const std::string& dh_params_path,
//     const TubeInfo& tube_info,
//     const HumanInfo& human_info,
//     std::shared_mutex& joint_data_mutex,
//     std::shared_mutex& vicon_data_mutex) {
    
//     std::cout << "Starting console real-time visualization (no VTK/OpenGL issues)..." << std::endl;
    
//     for (int i = 0; i < 100; ++i) { // 5 seconds at 50ms intervals
//         std::vector<double> left_angles_snapshot, right_angles_snapshot;
        
//         {
//             std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
//             left_angles_snapshot = left_arm_angles;
//             right_angles_snapshot = right_arm_angles;
//         }
        
//         std::cout << "Frame " << i << " - Left: [";
//         for (size_t j = 0; j < left_angles_snapshot.size(); ++j) {
//             std::cout << (int)left_angles_snapshot[j];
//             if (j < left_angles_snapshot.size() - 1) std::cout << ", ";
//         }
//         std::cout << "] Right: [";
//         for (size_t j = 0; j < right_angles_snapshot.size(); ++j) {
//             std::cout << (int)right_angles_snapshot[j];
//             if (j < right_angles_snapshot.size() - 1) std::cout << ", ";
//         }
//         std::cout << "]" << std::endl;
        
//         std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
    
//     std::cout << "Console visualization finished - no crashes!" << std::endl;
// }

// void visualizeRealtimePCL(
//     const std::vector<double>& left_arm_angles,
//     const std::vector<double>& right_arm_angles,
//     const gtsam::Pose3& left_base_pose,
//     const gtsam::Pose3& right_base_pose,
//     const std::string& dh_params_path,
//     const TubeInfo& tube_info,
//     const HumanInfo& human_info,
//     std::shared_mutex& joint_data_mutex,
//     std::shared_mutex& vicon_data_mutex) {
    
//     std::cout << "Starting PCL real-time visualization..." << std::endl;
    
//     // Load DH parameters and create arm models
//     DHParameters dh_params = createDHParams(dh_params_path);
//     ArmModel arm_model_generator;
    
//     auto left_arm_model = arm_model_generator.createArmModel(left_base_pose, dh_params);
//     auto right_arm_model = arm_model_generator.createArmModel(right_base_pose, dh_params);
    
//     if (!left_arm_model || !right_arm_model) {
//         std::cout << "Failed to create arm models!" << std::endl;
//         return;
//     }
    
//     // Create simple PCL cloud viewer (more stable than PCLVisualizer)
//     pcl::visualization::CloudViewer viewer("Real-time Robot Visualization");
    
//     // Create persistent point cloud for all data
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
//     std::cout << "PCL Visualizer created - starting real-time loop..." << std::endl;
    
//     // Initialize viewer once
//     static bool initialized = false;
    
//     // Real-time update loop
//     while (!viewer.wasStopped()) {
//         // Get current data (thread-safe)
//         std::vector<double> left_angles_snapshot, right_angles_snapshot;
//         TubeInfo tube_snapshot;
//         HumanInfo human_snapshot;
        
//         {
//             std::shared_lock<std::shared_mutex> joint_lock(joint_data_mutex);
//             left_angles_snapshot = left_arm_angles;
//             right_angles_snapshot = right_arm_angles;
//         }
//         {
//             std::shared_lock<std::shared_mutex> vicon_lock(vicon_data_mutex);
//             tube_snapshot = tube_info;
//             human_snapshot = human_info;
//         }
        
//         // Convert to radians and compute sphere positions
//         gtsam::Vector left_config(left_angles_snapshot.size());
//         gtsam::Vector right_config(right_angles_snapshot.size());
        
//         for (size_t i = 0; i < left_angles_snapshot.size(); ++i) {
//             left_config(i) = left_angles_snapshot[i] * (M_PI/180.0);
//         }
//         for (size_t i = 0; i < right_angles_snapshot.size(); ++i) {
//             right_config(i) = right_angles_snapshot[i] * (M_PI/180.0);
//         }
        
//         gtsam::Matrix left_sphere_centers = left_arm_model->sphereCentersMat(left_config);
//         gtsam::Matrix right_sphere_centers = right_arm_model->sphereCentersMat(right_config);
        
//         // Update combined cloud with all data
//         combined_cloud->clear();
        
//         // Add left arm spheres (Green)
//         for (size_t j = 0; j < left_arm_model->nr_body_spheres(); ++j) {
//             pcl::PointXYZRGB point;
//             point.x = left_sphere_centers(0, j);
//             point.y = left_sphere_centers(1, j);
//             point.z = left_sphere_centers(2, j);
//             point.r = 50; point.g = 255; point.b = 50; // Green
//             combined_cloud->push_back(point);
//         }
        
//         // Add right arm spheres (Blue)
//         for (size_t j = 0; j < right_arm_model->nr_body_spheres(); ++j) {
//             pcl::PointXYZRGB point;
//             point.x = right_sphere_centers(0, j);
//             point.y = right_sphere_centers(1, j);
//             point.z = right_sphere_centers(2, j);
//             point.r = 50; point.g = 50; point.b = 255; // Blue
//             combined_cloud->push_back(point);
//         }
        
//         // Add human points (Red)
//         for (const auto& human_point : human_snapshot.human_points) {
//             pcl::PointXYZRGB point;
//             point.x = human_point.x();
//             point.y = human_point.y();
//             point.z = human_point.z();
//             point.r = 255; point.g = 50; point.b = 50; // Red
//             combined_cloud->push_back(point);
//         }
        
//         // Add tube points (Gray)
//         for (const auto& tube_point : tube_snapshot.tube_points) {
//             pcl::PointXYZRGB point;
//             point.x = tube_point.x();
//             point.y = tube_point.y();
//             point.z = tube_point.z();
//             point.r = 128; point.g = 128; point.b = 128; // Gray
//             combined_cloud->push_back(point);
//         }
        
//         // Use safer update pattern to avoid segfault
//         if (!initialized) {
//             viewer.showCloud(combined_cloud);
//             initialized = true;
//         } else {
//             // For subsequent updates, use a safer approach
//             try {
//                 viewer.showCloud(combined_cloud);
//             } catch (...) {
//                 std::cout << "CloudViewer update failed, continuing..." << std::endl;
//             }
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Slower updates
//     }
    
//     std::cout << "PCL Visualizer finished" << std::endl;
// }