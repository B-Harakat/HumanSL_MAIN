#include "Obstacles.h"


C3D_Dataset ObstacleFromC3D::createC3DDataset(
    const std::string& c3d_file_path,
    const std::string& subject_prefix,
    int frame_number) {
   
    C3D_Dataset c3d_dataset;
    std::cout << "Loading C3D file: " << c3d_file_path << std::endl;
   
    // Load C3D file
    ezc3d::c3d c3d_file(c3d_file_path);
   
    // Get basic info
    auto header = c3d_file.header();
    auto data = c3d_file.data();
   
    size_t n_frames = header.nbFrames();
    size_t n_points = header.nb3dPoints();
   
    std::cout << "C3D Info: " << n_frames << " frames, " << n_points << " points per frame" << std::endl;
   
    // Find points matching subject prefix
    std::vector<size_t> subject_point_indices;
    std::vector<std::string> subject_point_names;
   
    size_t CLAV_idx;
    size_t STRN_idx;
    auto point_names = c3d_file.parameters().group("POINT").parameter("LABELS").valuesAsString();
    for (size_t i = 0; i < point_names.size(); ++i) {
        if (point_names[i].find(subject_prefix + ":") == 0) {
            subject_point_indices.push_back(i);
            subject_point_names.push_back(point_names[i]);
            if(point_names[i].find(subject_prefix + ":CLAV") == 0){CLAV_idx = i; std::cout<<"Found CLAV point"<< std::endl;}
            if(point_names[i].find(subject_prefix + ":STRN") == 0){STRN_idx = i;}
        }
    }
   
    if (subject_point_indices.empty()) {
        throw std::runtime_error("No points found with prefix '" + subject_prefix + ":'");
    }    
    std::cout << "Found " << subject_point_indices.size() << " points for subject '" << subject_prefix << "'" << std::endl;
   
    // Extract 3D points for the specified frame
    std::vector<gtsam::Point3> subject_points;
   
    for (size_t idx : subject_point_indices) {
        auto point = data.frame(frame_number).points().point(idx);
        double x = point.x() / 1000.0; // Convert mm to meters
        double y = point.y() / 1000.0;
        double z = point.z() / 1000.0;
       
        // Check if point is valid (not NaN/inf)
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            subject_points.emplace_back(x, y, z);
        }
        if(idx == CLAV_idx) {c3d_dataset.clav = gtsam::Point3(x, y, z);}
        if(idx == STRN_idx) {c3d_dataset.strn = gtsam::Point3(x, y, z);}
    }
   
    
    double min_x = subject_points[0].x();
    double max_x = subject_points[0].x();
    double min_y = subject_points[0].y();
    double max_y = subject_points[0].y();
    double min_z = subject_points[0].z();
    double max_z = subject_points[0].z();
    
    for (const auto& point : subject_points) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }

    c3d_dataset.human_points = subject_points;
    c3d_dataset.bounds = HumanBoundingBox{min_x, max_x, min_y, max_y, min_z, max_z};
    
    return c3d_dataset;
}

TubeInfo ObstacleFromC3D::extractTubeInfoFromC3D(const std::string& c3dFilePath, const int& frame_idx) {

    ezc3d::c3d c3dFile(c3dFilePath);
    

    auto pointNames = c3dFile.pointNames();

    std::vector<size_t> tubeIndices;
    for (size_t i = 0; i < pointNames.size(); ++i) {
        if (pointNames[i].find("tube:") != std::string::npos) {
            tubeIndices.push_back(i);
        }
    }

    auto tube_time3 = std::chrono::high_resolution_clock::now();

    const auto& frame = c3dFile.data().frames()[frame_idx];
    
    // Collect tube points for PCA
    std::vector<Eigen::Vector3d> tubePoints;
    
    for (size_t idx : tubeIndices) {
        const auto& pointData = frame.points().point(idx);

        tubePoints.push_back(Eigen::Vector3d(pointData.x()/1000, pointData.y()/1000, pointData.z()/1000));
    }

    // Perform PCA analysis
    
    TubeInfo result;

    int n = tubePoints.size();

    result.centroid = Eigen::Vector3d::Zero();
    for (const auto& p : tubePoints) {
        result.centroid += p;
    }
    result.centroid /= n;
    
    Eigen::MatrixXd centered(n, 3);
    for (int i = 0; i < n; ++i) {
        centered.row(i) = (tubePoints[i] - result.centroid).transpose();
    }

    // Compute covariance matrix
    Eigen::Matrix3d covariance = centered.transpose() * centered / (n - 1);

    // Eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

    // The eigenvector with largest eigenvalue is the principal axis
    int maxEigenIndex = 0;
    for (int i = 1; i < 3; ++i) {
        if (solver.eigenvalues()(i) > solver.eigenvalues()(maxEigenIndex)) {
            maxEigenIndex = i;
        }
    }

    result.direction = solver.eigenvectors().col(maxEigenIndex);


    // Calculate tube length by projecting points onto axis
    double minProj = std::numeric_limits<double>::max();
    double maxProj = std::numeric_limits<double>::lowest();
    
    for (const auto& p : tubePoints) {
        Eigen::Vector3d centered_point = p - result.centroid;
        double projection = centered_point.dot(result.direction);
        minProj = std::min(minProj, projection);
        maxProj = std::max(maxProj, projection);
    }
    
    result.length = maxProj - minProj;
    
    return result;
}


std::unique_ptr<GPMP2_OccupancyGrid> ObstacleFromC3D::createHumanFromC3D(
    const std::vector<gtsam::Point3>& subject_points,
    double cell_size,
    const std::array<double, 3>& workspace_size,
    const std::array<double, 3>& workspace_origin) {
    
    
    // Calculate bounding box of subject points
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    
    for (const auto& point : subject_points) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }
    
    // Add 5cm safety margin to bounding box
    const double safety_margin = 0.05; // 5cm
    min_x -= safety_margin;
    max_x += safety_margin;
    min_y -= safety_margin;
    max_y += safety_margin;
    min_z -= safety_margin;
    max_z += safety_margin;
    
    std::cout << "Human bounding box (with 5cm margin): " << std::endl;
    std::cout << "  X: [" << min_x << ", " << max_x << "]" << std::endl;
    std::cout << "  Y: [" << min_y << ", " << max_y << "]" << std::endl;
    std::cout << "  Z: [" << min_z << ", " << max_z << "]" << std::endl;
    
    // Create workspace grid
    size_t grid_x = static_cast<size_t>(std::ceil(workspace_size[0] / cell_size));
    size_t grid_y = static_cast<size_t>(std::ceil(workspace_size[1] / cell_size));
    size_t grid_z = static_cast<size_t>(std::ceil(workspace_size[2] / cell_size));
    
    std::cout << "Creating " << grid_x << "x" << grid_y << "x" << grid_z << " occupancy grid" << std::endl;
    
    // Create dataset
    auto dataset = std::make_unique<GPMP2_OccupancyGrid>(grid_x, grid_y, grid_z, 
                                           workspace_origin[0], workspace_origin[1], workspace_origin[2], 
                                           cell_size);
    
    std::cout << "Adding human obstacle to occupancy grid..." << std::endl;

    // Convert world bounds to grid bounds
    int grid_min_x = std::max(0, static_cast<int>((min_x - workspace_origin[0]) / cell_size));
    int grid_max_x = std::min(static_cast<int>(grid_x) - 1, 
                             static_cast<int>((max_x - workspace_origin[0]) / cell_size));
    int grid_min_y = std::max(0, static_cast<int>((min_y - workspace_origin[1]) / cell_size));
    int grid_max_y = std::min(static_cast<int>(grid_y) - 1, 
                             static_cast<int>((max_y - workspace_origin[1]) / cell_size));
    int grid_min_z = std::max(0, static_cast<int>((min_z - workspace_origin[2]) / cell_size));
    int grid_max_z = std::min(static_cast<int>(grid_z) - 1, 
                             static_cast<int>((max_z - workspace_origin[2]) / cell_size));

    std::cout << "Searching grid cells in bounds: " << std::endl;
    std::cout << "  X: [" << grid_min_x << ", " << grid_max_x << "]" << std::endl;
    std::cout << "  Y: [" << grid_min_y << ", " << grid_max_y << "]" << std::endl;
    std::cout << "  Z: [" << grid_min_z << ", " << grid_max_z << "]" << std::endl;

    // Simple distance heuristics method for determining occupancy (within bounding box only)
    if (!subject_points.empty()) {
        for (int i = grid_min_x; i <= grid_max_x; ++i) {
            for (int j = grid_min_y; j <= grid_max_y; ++j) {
                for (int k = grid_min_z; k <= grid_max_z; ++k) {
                    // Calculate world coordinates of grid cell center
                    double world_x = workspace_origin[0] + (i + 0.5) * cell_size;
                    double world_y = workspace_origin[1] + (j + 0.5) * cell_size;
                    double world_z = workspace_origin[2] + (k + 0.5) * cell_size;
                    
                    // Check if grid cell is within 10cm of any subject point
                    bool within_distance = false;
                    for (const auto& point : subject_points) {
                        double dx = world_x - point.x();
                        double dy = world_y - point.y();
                        double dz = world_z - point.z();
                        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                        
                        if (distance <= 0.1) { // 10cm threshold
                            within_distance = true;
                            break; // Found one close point, no need to check others
                        }
                    }
                    
                    if (within_distance) {
                        dataset->map[i][j][k] = 1.0f;
                    }
                }
            }
        }
    }
    
    // Count occupied cells
    size_t n_occupied = 0;
    for (size_t i = 0; i < grid_x; ++i) {
        for (size_t j = 0; j < grid_y; ++j) {
            for (size_t k = 0; k < grid_z; ++k) {
                if (dataset->map[i][j][k] > 0.5f) {
                    n_occupied++;
                }
            }
        }
    }
    
    size_t total_cells = grid_x * grid_y * grid_z;
    size_t searched_cells = (grid_max_x - grid_min_x + 1) * (grid_max_y - grid_min_y + 1) * (grid_max_z - grid_min_z + 1);
    
    std::cout << "Occupied cells: " << n_occupied << "/" << total_cells 
              << " (" << (100.0 * n_occupied / total_cells) << "%)" << std::endl;
    std::cout << "Searched cells: " << searched_cells << "/" << total_cells 
              << " (" << (100.0 * searched_cells / total_cells) << "% of total grid)" << std::endl;
    
    return dataset;
}

std::unique_ptr<GPMP2_OccupancyGrid> ObstacleFromC3D::createTubeFromC3D(
    const GPMP2_OccupancyGrid& original_dataset,
    const TubeInfo& tube_axis,
    double safety_margin) {
    
    // Hard-coded tube radius as requested
    const double tube_radius = 0.03; // 5cm
    
    std::cout << "Adding tube obstacle to occupancy grid..." << std::endl;
    std::cout << "Tube length: " << tube_axis.length << "m, radius: " << tube_radius << "m" << std::endl;
    std::cout << "Tube centroid: (" 
              << tube_axis.centroid.x() << ", " 
              << tube_axis.centroid.y() << ", " 
              << tube_axis.centroid.z() << ")" << std::endl;
    std::cout << "Tube direction: (" 
              << tube_axis.direction.x() << ", " 
              << tube_axis.direction.y() << ", " 
              << tube_axis.direction.z() << ")" << std::endl;

    // Create copy of original dataset
    auto updated_dataset = std::make_unique<GPMP2_OccupancyGrid>(original_dataset);
    
    // Convert Eigen to gtsam for consistency
    gtsam::Point3 tube_center(tube_axis.centroid.x(), tube_axis.centroid.y(), tube_axis.centroid.z());
    gtsam::Point3 tube_direction(tube_axis.direction.x(), tube_axis.direction.y(), tube_axis.direction.z());
    
    // Normalize direction vector (should already be normalized from PCA)
    gtsam::Point3 tube_axis_normalized = tube_direction / tube_direction.norm();
    
    // Calculate tube endpoints
    double half_length = tube_axis.length / 2.0;
    gtsam::Point3 tube_start = tube_center - half_length * tube_axis_normalized;
    gtsam::Point3 tube_end = tube_center + half_length * tube_axis_normalized;
    
    // Effective radius including safety margin
    double effective_radius = tube_radius + safety_margin;
    
    size_t occupied_cells_added = 0;
    
    // Calculate bounding box for efficiency
    double min_x = std::min(tube_start.x(), tube_end.x()) - effective_radius;
    double max_x = std::max(tube_start.x(), tube_end.x()) + effective_radius;
    double min_y = std::min(tube_start.y(), tube_end.y()) - effective_radius;
    double max_y = std::max(tube_start.y(), tube_end.y()) + effective_radius;
    double min_z = std::min(tube_start.z(), tube_end.z()) - effective_radius;
    double max_z = std::max(tube_start.z(), tube_end.z()) + effective_radius;
    
    // Convert world bounds to grid bounds
    int grid_min_x = std::max(0, static_cast<int>((min_x - original_dataset.origin_x) / original_dataset.cell_size));
    int grid_max_x = std::min(static_cast<int>(original_dataset.rows) - 1, 
                             static_cast<int>((max_x - original_dataset.origin_x) / original_dataset.cell_size));
    int grid_min_y = std::max(0, static_cast<int>((min_y - original_dataset.origin_y) / original_dataset.cell_size));
    int grid_max_y = std::min(static_cast<int>(original_dataset.cols) - 1, 
                             static_cast<int>((max_y - original_dataset.origin_y) / original_dataset.cell_size));
    int grid_min_z = std::max(0, static_cast<int>((min_z - original_dataset.origin_z) / original_dataset.cell_size));
    int grid_max_z = std::min(static_cast<int>(original_dataset.z) - 1, 
                             static_cast<int>((max_z - original_dataset.origin_z) / original_dataset.cell_size));
    
    // Iterate through grid cells in bounding box
    for (int i = grid_min_x; i <= grid_max_x; ++i) {
        for (int j = grid_min_y; j <= grid_max_y; ++j) {
            for (int k = grid_min_z; k <= grid_max_z; ++k) {
                // Calculate world coordinates of grid cell center
                double world_x = original_dataset.origin_x + (i + 0.5) * original_dataset.cell_size;
                double world_y = original_dataset.origin_y + (j + 0.5) * original_dataset.cell_size;
                double world_z = original_dataset.origin_z + (k + 0.5) * original_dataset.cell_size;
                
                gtsam::Point3 cell_center(world_x, world_y, world_z);
                
                // Calculate distance from cell center to tube axis (cylinder)
                double distance_to_tube = distancePointToCylinder(cell_center, tube_start, tube_end, effective_radius);
                
                // Mark as occupied if within tube radius
                if (distance_to_tube <= effective_radius) {
                    if (updated_dataset->map[i][j][k] < 0.5f) {  // Only count new occupations
                        occupied_cells_added++;
                    }
                    updated_dataset->map[i][j][k] = 1.0f;
                }
            }
        }
    }
    
    std::cout << "Added " << occupied_cells_added << " grid cells for tube obstacle" << std::endl;
    
    return updated_dataset;
}


std::unique_ptr<GPMP2_OccupancyGrid> Obstacle::updateGridWithOtherArm(
    const GPMP2_OccupancyGrid& original_dataset,
    const gpmp2::ArmModel& arm_model,
    const gtsam::Vector& arm_config,
    double safety_margin) {
    
    std::cout << "Adding arm with " << arm_model.nr_body_spheres() << " spheres to occupancy grid..." << std::endl;
    
    // Create copy of original dataset
    auto updated_dataset = std::make_unique<GPMP2_OccupancyGrid>(original_dataset);
    
    // Get sphere centers in world coordinates
    gtsam::Matrix sphere_centers_mat = arm_model.sphereCentersMat(arm_config);
    
    size_t occupied_cells_added = 0;
    
    // For each sphere, mark grid cells as occupied
    for (size_t i = 0; i < arm_model.nr_body_spheres(); ++i) {
        gtsam::Point3 sphere_center(sphere_centers_mat(0, i), sphere_centers_mat(1, i), sphere_centers_mat(2, i));
        double sphere_radius = arm_model.sphere_radius(i);
        
        // Convert sphere center to grid coordinates
        int grid_x = static_cast<int>((sphere_center.x() - original_dataset.origin_x) / original_dataset.cell_size);
        int grid_y = static_cast<int>((sphere_center.y() - original_dataset.origin_y) / original_dataset.cell_size);
        int grid_z = static_cast<int>((sphere_center.z() - original_dataset.origin_z) / original_dataset.cell_size);
        
        // Calculate grid radius
        int grid_radius = static_cast<int>(std::ceil(sphere_radius / original_dataset.cell_size));
        
        // Mark all grid cells within sphere radius as occupied
        for (int dx = -grid_radius; dx <= grid_radius; ++dx) {
            for (int dy = -grid_radius; dy <= grid_radius; ++dy) {
                for (int dz = -grid_radius; dz <= grid_radius; ++dz) {
                    int gx = grid_x + dx;
                    int gy = grid_y + dy;
                    int gz = grid_z + dz;
                    
                    // Check bounds
                    if (gx >= 0 && gx < static_cast<int>(original_dataset.rows) &&
                        gy >= 0 && gy < static_cast<int>(original_dataset.cols) &&
                        gz >= 0 && gz < static_cast<int>(original_dataset.z)) {
                        
                        // Calculate actual distance from sphere center
                        double cell_center_x = original_dataset.origin_x + (gx + 0.5) * original_dataset.cell_size;
                        double cell_center_y = original_dataset.origin_y + (gy + 0.5) * original_dataset.cell_size;
                        double cell_center_z = original_dataset.origin_z + (gz + 0.5) * original_dataset.cell_size;
                        
                        gtsam::Point3 cell_center(cell_center_x, cell_center_y, cell_center_z);
                        double distance = (sphere_center - cell_center).norm();
                        
                        // Mark as occupied if within sphere
                        if (distance <= sphere_radius) {
                            if (updated_dataset->map[gx][gy][gz] < 0.5f) {  // Only count new occupations
                                occupied_cells_added++;
                            }
                            updated_dataset->map[gx][gy][gz] = 1.0f;
                        }
                    }
                }
            }
        }
    }
    
    std::cout << "Added " << occupied_cells_added << " grid cells for arm obstacles" << std::endl;
    
    return updated_dataset;
}


std::vector<std::vector<std::vector<double>>> Obstacle::computeEDT3D(
    const std::vector<std::vector<std::vector<bool>>>& binary_grid) {
    
    size_t rows = binary_grid.size();
    size_t cols = binary_grid[0].size();
    size_t depth = binary_grid[0][0].size();
    
    // Initialize distance array with large values
    std::vector<std::vector<std::vector<double>>> dist(
        rows, std::vector<std::vector<double>>(cols, std::vector<double>(depth, std::numeric_limits<double>::max())));
    
    // Priority queue for Dijkstra: (distance, i, j, k)
    std::priority_queue<std::tuple<double, size_t, size_t, size_t>, 
                       std::vector<std::tuple<double, size_t, size_t, size_t>>,
                       std::greater<std::tuple<double, size_t, size_t, size_t>>> pq;
    
    // Initialize distances for foreground pixels (obstacles/sources)
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            for (size_t k = 0; k < depth; ++k) {
                if (binary_grid[i][j][k]) {
                    dist[i][j][k] = 0.0;
                    pq.push(std::make_tuple(0.0, i, j, k));
                }
            }
        }
    }
    
    // 26-connectivity neighbors for 3D
    std::vector<std::tuple<int, int, int>> neighbors = {
        {-1,-1,-1}, {-1,-1,0}, {-1,-1,1}, {-1,0,-1}, {-1,0,0}, {-1,0,1}, 
        {-1,1,-1}, {-1,1,0}, {-1,1,1}, {0,-1,-1}, {0,-1,0}, {0,-1,1},
        {0,0,-1}, {0,0,1}, {0,1,-1}, {0,1,0}, {0,1,1},
        {1,-1,-1}, {1,-1,0}, {1,-1,1}, {1,0,-1}, {1,0,0}, {1,0,1},
        {1,1,-1}, {1,1,0}, {1,1,1}
    };
    
    // Dijkstra's algorithm to compute distances
    while (!pq.empty()) {
        auto [current_dist, i, j, k] = pq.top();
        pq.pop();
        
        // Skip if we've already found a shorter path
        if (current_dist > dist[i][j][k]) {
            continue;
        }
        
        // Check all neighbors
        for (const auto& [di, dj, dk] : neighbors) {
            int ni = static_cast<int>(i) + di;
            int nj = static_cast<int>(j) + dj;
            int nk = static_cast<int>(k) + dk;
            
            if (ni >= 0 && ni < static_cast<int>(rows) &&
                nj >= 0 && nj < static_cast<int>(cols) &&
                nk >= 0 && nk < static_cast<int>(depth)) {
                
                double edge_weight = std::sqrt(di*di + dj*dj + dk*dk);
                double new_dist = current_dist + edge_weight;
                
                if (new_dist < dist[ni][nj][nk]) {
                    dist[ni][nj][nk] = new_dist;
                    pq.push(std::make_tuple(new_dist, ni, nj, nk));
                }
            }
        }
    }
    
    return dist;
}


std::vector<gtsam::Matrix> Obstacle::occupancyGridToSDF(const GPMP2_OccupancyGrid& dataset) {
    
    // Convert dataset.map to binary grids following Python implementation
    std::vector<std::vector<std::vector<bool>>> cur_map(
        dataset.rows, std::vector<std::vector<bool>>(dataset.cols, std::vector<bool>(dataset.z)));
    std::vector<std::vector<std::vector<bool>>> inv_map(
        dataset.rows, std::vector<std::vector<bool>>(dataset.cols, std::vector<bool>(dataset.z)));
    
    // DEBUG: Count obstacles and free space
    size_t obstacle_count = 0;
    size_t free_count = 0;
    
    // Regularize unknown area to open area and create inverse map
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                cur_map[i][j][k] = dataset.map[i][j][k] > 0.75f;
                inv_map[i][j][k] = !cur_map[i][j][k];
                
                if (cur_map[i][j][k]) obstacle_count++;
                else free_count++;
            }
        }
    }

    // Check if map is empty
    bool has_obstacles = obstacle_count > 0;
    
    std::vector<gtsam::Matrix> sdf_data(dataset.z);

    // Compute distance transforms
    auto map_dist = computeEDT3D(cur_map);  // Distance from obstacles (inverse map)
    auto inv_map_dist = computeEDT3D(inv_map);  // Distance from free space
    
    // DEBUG: Check EDT results
    double map_dist_min = std::numeric_limits<double>::max();
    double map_dist_max = std::numeric_limits<double>::lowest();
    double inv_map_dist_min = std::numeric_limits<double>::max();
    double inv_map_dist_max = std::numeric_limits<double>::lowest();
    
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                map_dist_min = std::min(map_dist_min, map_dist[i][j][k]);
                map_dist_max = std::max(map_dist_max, map_dist[i][j][k]);
                inv_map_dist_min = std::min(inv_map_dist_min, inv_map_dist[i][j][k]);
                inv_map_dist_max = std::max(inv_map_dist_max, inv_map_dist[i][j][k]);
            }
        }
    }
    

    // Initialize SDF matrices with proper coordinate mapping
    for (size_t z = 0; z < dataset.z; ++z) {
        sdf_data[z] = gtsam::Matrix::Zero(dataset.cols, dataset.rows);  // Note: cols x rows for Y x X
    }
    
    // Compute signed distance field: field = map_dist - inv_map_dist
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                double field_value = (map_dist[i][j][k] - inv_map_dist[i][j][k]) * dataset.cell_size;
                
                // Map dataset coordinates to SDF coordinates:
                // dataset.map[i][j][k] where i=X, j=Y, k=Z
                // -> sdf_data[k](j, i) where k=Z, j=Y(row), i=X(col)
                sdf_data[k](j, i) = field_value;
            }
        }
    }
    
    return sdf_data;
}


std::unique_ptr<gpmp2::SignedDistanceField> Obstacle::createSDFFromOccupancyGrid(const GPMP2_OccupancyGrid& dataset) {
    // Convert occupancy grid to SDF using proper 3D EDT
    auto sdf_data = occupancyGridToSDF(dataset);
    
    // Create origin point
    gtsam::Point3 origin(dataset.origin_x, dataset.origin_y, dataset.origin_z);
    
    // Create SignedDistanceField with proper dimensions
    // GPMP2 expects field_rows = Y dimension, field_cols = X dimension
    auto sdf = std::make_unique<gpmp2::SignedDistanceField>(
        origin, dataset.cell_size, dataset.cols, dataset.rows, dataset.z);
    
    // Initialize field data
    for (size_t z = 0; z < dataset.z; ++z) {
        sdf->initFieldData(z, sdf_data[z]);
    }
    
    return sdf;
}


// Helper function to calculate distance from point to cylinder
double ObstacleFromC3D::distancePointToCylinder(const gtsam::Point3& point, 
                              const gtsam::Point3& cylinder_start, 
                              const gtsam::Point3& cylinder_end, 
                              double cylinder_radius) {
    
    // Vector from start to end of cylinder
    gtsam::Point3 cylinder_axis = cylinder_end - cylinder_start;
    double cylinder_length = cylinder_axis.norm();
    
    if (cylinder_length < 1e-10) {
        // Degenerate case: cylinder is a point, treat as sphere
        return (point - cylinder_start).norm();
    }
    
    // Normalize axis
    gtsam::Point3 axis_normalized = cylinder_axis / cylinder_length;
    
    // Vector from cylinder start to point
    gtsam::Point3 start_to_point = point - cylinder_start;
    
    // Project point onto cylinder axis
    double projection_length = start_to_point.dot(axis_normalized);
    
    // Clamp projection to cylinder length (finite cylinder)
    projection_length = std::max(0.0, std::min(cylinder_length, projection_length));
    
    // Find closest point on cylinder axis
    gtsam::Point3 closest_on_axis = cylinder_start + projection_length * axis_normalized;
    
    // Distance from point to closest point on axis
    double radial_distance = (point - closest_on_axis).norm();
    
    return radial_distance;
}
