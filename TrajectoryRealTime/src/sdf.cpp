#include "sdf.h"


// Empty constructor
ViconSDF::ViconSDF() : ArmModel(){
    // Initialize any member variables if needed in the future
}

std::unique_ptr<GPMP2_OccupancyGrid> ViconSDF::HumanObstacleFromVicon(
    const HumanInfo& subject,
    double cell_size,
    const std::array<double, 3>& workspace_size,
    const std::array<double, 3>& workspace_origin) {
    
    auto [min_x, max_x, min_y, max_y, min_z, max_z] = subject.bounds;
    auto subject_points = subject.human_points;
    
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
    
    return dataset;
}

std::unique_ptr<GPMP2_OccupancyGrid> ViconSDF::ArmObstacleFromVicon(
    const GPMP2_OccupancyGrid& original_dataset,
    const std::unique_ptr<gpmp2::ArmModel>& arm_model,
    const gtsam::Vector& other_arm_config,
    double safety_margin) {
    
    // Create copy of original dataset
    auto updated_dataset = std::make_unique<GPMP2_OccupancyGrid>(original_dataset);
    
    // Get sphere centers in world coordinates
    gtsam::Matrix sphere_centers_mat = arm_model->sphereCentersMat(other_arm_config);
    
    // For each sphere, mark grid cells as occupied
    for (size_t i = 0; i < arm_model->nr_body_spheres(); ++i) {
        gtsam::Point3 sphere_center(sphere_centers_mat(0, i), sphere_centers_mat(1, i), sphere_centers_mat(2, i));
        double sphere_radius = arm_model->sphere_radius(i);
        
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
                            }
                            updated_dataset->map[gx][gy][gz] = 1.0f;
                        }
                    }
                }
            }
        }
    }
    
    return updated_dataset;
}

std::unique_ptr<GPMP2_OccupancyGrid> ViconSDF::TubeObstacleFromVicon(
    const GPMP2_OccupancyGrid& original_dataset,
    const TubeInfo& tube_axis,
    double safety_margin) {
    
    // Hard-coded tube radius 
    const double tube_radius = 0.02; // 3.5cm
    
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
                gtsam::Point3 cylinder_axis = tube_end - tube_start;
                double cylinder_length = cylinder_axis.norm();
                
                // Normalize axis
                gtsam::Point3 axis_normalized = cylinder_axis / cylinder_length;
                
                // Vector from cylinder start to point
                gtsam::Point3 start_to_point = cell_center - tube_start;
                
                // Project point onto cylinder axis
                double projection_length = start_to_point.dot(axis_normalized);
                
                // Clamp projection to cylinder length (finite cylinder)
                projection_length = std::max(0.0, std::min(cylinder_length, projection_length));
                
                // Find closest point on cylinder axis
                gtsam::Point3 closest_on_axis = tube_start + projection_length * axis_normalized;
                
                // Distance from point to closest point on axis
                double distance_to_tube = (cell_center - closest_on_axis).norm();
                
                // Mark as occupied if within tube radius
                if (distance_to_tube <= effective_radius) {
                    updated_dataset->map[i][j][k] = 1.0f;
                }
            }
        }
    }

    return updated_dataset;
}

std::vector<std::vector<std::vector<double>>> ViconSDF::computeEDT3D(
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

std::vector<gtsam::Matrix> ViconSDF::occupancyGridToSDF(const GPMP2_OccupancyGrid& dataset) {
    
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

std::unique_ptr<gpmp2::SignedDistanceField> ViconSDF::createSDFFromOccupancyGrid(const GPMP2_OccupancyGrid& dataset) {
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