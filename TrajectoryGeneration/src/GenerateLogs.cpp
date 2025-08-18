#include "GenerateLogs.h"


void saveTrajectoryResultToYAML(const TrajectoryResult& result, const std::string& method) {
    // Generate timestamp-based filename
    std::string filename;

    std::ostringstream oss;
    oss << "../logs/" << method <<"_results" 
        << ".yaml";
    filename = oss.str();
 

    // Create YAML structure
    YAML::Node root;
    YAML::Node trajectory_node;
    
    // Metadata
    YAML::Node metadata;
    metadata["start_error"] = result.start_error;
    metadata["final_error"] = result.final_error;
    metadata["optimization_duration_ms"] = result.optimization_duration.count();
    metadata["initiation_duration_ms"] = result.initiation_duration.count();
    metadata["dt"] = result.dt;
    metadata["original_trajectory_size"] = result.trajectory_pos.size();
    
    // Sample every 16th frame for ~60 FPS
    constexpr size_t sample_rate = 1;
    std::vector<gtsam::Vector> sampled_trajectory;
    
    for (size_t i = 0; i < result.trajectory_pos.size(); i += sample_rate) {
        sampled_trajectory.push_back(result.trajectory_pos[i]);
    }
    
    // Always include the final waypoint if it wasn't included
    if (!result.trajectory_pos.empty() && 
        (result.trajectory_pos.size() - 1) % sample_rate != 0) {
        sampled_trajectory.push_back(result.trajectory_pos.back());
    }
    
    metadata["sampled_trajectory_size"] = sampled_trajectory.size();
    metadata["sample_rate"] = sample_rate;
    metadata["effective_dt"] = result.dt * sample_rate;
    
    trajectory_node["metadata"] = metadata;
    
    // Costs
    YAML::Node costs;
    
    YAML::Node start_costs;
    for (const auto& cost : result.start_costs) {
        start_costs[cost.first] = cost.second;
    }
    costs["start_costs"] = start_costs;
    
    YAML::Node final_costs;
    for (const auto& cost : result.final_costs) {
        final_costs[cost.first] = cost.second;
    }
    costs["final_costs"] = final_costs;
    
    trajectory_node["costs"] = costs;
    
    // Trajectory waypoints
    YAML::Node trajectory_waypoints;
    for (size_t i = 0; i < sampled_trajectory.size(); ++i) {
        const gtsam::Vector& waypoint = sampled_trajectory[i];
        YAML::Node waypoint_node;
        
        // Convert gtsam::Vector to YAML sequence
        for (int j = 0; j < waypoint.size(); ++j) {
            waypoint_node.push_back(waypoint(j));
        }
        
        trajectory_waypoints.push_back(waypoint_node);
    }
    
    trajectory_node["trajectory"] = trajectory_waypoints;
    root["trajectory_result"] = trajectory_node;
    
    // Write to file
    std::ofstream file(filename);
    file << root;
    file.close();
    
    std::cout << "Trajectory saved to: " << filename << std::endl;
    std::cout << "Sampled " << sampled_trajectory.size() << " waypoints from " 
              << result.trajectory_pos.size() << " original waypoints" << std::endl;
}


void exportSDFToYAML(const gpmp2::SignedDistanceField& sdf) {
    std::ofstream file("../logs/sdf_visualization.yaml");
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open sdf_visualization.yaml for writing" << std::endl;
        return;
    }
    
    // Get SDF properties
    auto origin = sdf.origin();
    double cell_size = sdf.cell_size();
    size_t cols = sdf.x_count();  // X dimension
    size_t rows = sdf.y_count();  // Y dimension  
    size_t z_levels = sdf.z_count(); // Z dimension
    auto& data = sdf.raw_data();
    
    std::cout << "Exporting SDF to YAML:" << std::endl;
    std::cout << "  Origin: [" << origin.x() << ", " << origin.y() << ", " << origin.z() << "]" << std::endl;
    std::cout << "  Cell size: " << cell_size << std::endl;
    std::cout << "  Dimensions: [" << cols << ", " << rows << ", " << z_levels << "]" << std::endl;
    
    // Write YAML header
    file << "sdf:" << std::endl;
    file << "  origin: [" << origin.x() << ", " << origin.y() << ", " << origin.z() << "]" << std::endl;
    file << "  cell_size: " << cell_size << std::endl;
    file << "  dimensions: [" << cols << ", " << rows << ", " << z_levels << "]" << std::endl;
    file << "  data:" << std::endl;
    
    // Write data for each Z level
    for (size_t z = 0; z < z_levels; ++z) {
        file << "    z_level_" << z << ":" << std::endl;
        
        const auto& z_slice = data[z];  // This is a gtsam::Matrix of size (rows x cols)
        
        for (size_t row = 0; row < rows; ++row) {
            file << "      - [";
            for (size_t col = 0; col < cols; ++col) {
                file << z_slice(row, col);
                if (col < cols - 1) file << ", ";
            }
            file << "]" << std::endl;
        }
    }
    
    file.close();
    std::cout << "SDF exported to sdf_visualization.yaml" << std::endl;
}


void visualizeTrajectory(
    const std::vector<gtsam::Vector>& trajectory,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose) {
    
    if (trajectory.empty()) {
        ::std::cerr << "Empty trajectory provided!" << ::std::endl;
        return;
    }
    
    // Calculate timing parameters
    double total_time_sec = trajectory.size() / 1000.0;  // 1000Hz trajectory
    
    // Create sampled frame indices (every 33rd frame + last frame)
    ::std::vector<size_t> sampled_indices;
    for (size_t i = 0; i < trajectory.size(); i += 33) {
        sampled_indices.push_back(i);
    }
    // Ensure last frame is included
    if (sampled_indices.back() != trajectory.size() - 1) {
        sampled_indices.push_back(trajectory.size() - 1);
    }
    
    ::std::cout << "Creating animated trajectory visualization..." << ::std::endl;
    ::std::cout << "Total trajectory time: " << total_time_sec << " seconds" << ::std::endl;
    ::std::cout << "Original frames: " << trajectory.size() << " (1000Hz)" << ::std::endl;
    ::std::cout << "Sampled frames: " << sampled_indices.size() << " (~30Hz)" << ::std::endl;
    ::std::cout << "Arm spheres: " << arm_model.nr_body_spheres() << ::std::endl;
    
    // Get camera look-at point from base pose
    gtsam::Point3 base_position = base_pose.translation();
    
    // Create timestamped filename
    auto now = ::std::chrono::system_clock::now();
    auto time_t = ::std::chrono::system_clock::to_time_t(now);
    auto ms = ::std::chrono::duration_cast<::std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    ::std::stringstream ss;
    ss << ::std::put_time(::std::localtime(&time_t), "%Y%m%d_%H%M%S");
    ss << "_" << ::std::setfill('0') << ::std::setw(3) << ms.count();
    
    ::std::string timestamp = ss.str();
    ::std::string filename = "trajectory_animation_" + timestamp + ".html";
    
    // Save to local Videos directory
    ::std::string videos_dir = "./TrajectoryAnimation";
    ::std::string full_path = videos_dir + "/" + filename;
    
    // Create directory if it doesn't exist
    ::std::string mkdir_cmd = "mkdir -p " + videos_dir;
    ::std::system(mkdir_cmd.c_str());
    
    ::std::ofstream file(full_path);
    
    if (!file.is_open()) {
        ::std::cerr << "Could not create file: " << full_path << ::std::endl;
        ::std::cerr << "Make sure the directory exists and is accessible" << ::std::endl;
        return;
    }
    
    // Write HTML header
    file << "<!DOCTYPE html>\n"
         << "<html>\n"
         << "<head>\n"
         << "    <title>Animated Trajectory Visualization</title>\n"
         << "    <style>\n"
         << "        body { \n"
         << "            margin: 0; \n"
         << "            padding: 0; \n"
         << "            background: #111; \n"
         << "            overflow: hidden; \n"
         << "            font-family: Arial, sans-serif;\n"
         << "        }\n"
         << "        #controls { \n"
         << "            position: absolute; \n"
         << "            top: 10px; \n"
         << "            left: 10px; \n"
         << "            color: white; \n"
         << "            z-index: 100;\n"
         << "            background: rgba(0,0,0,0.7);\n"
         << "            padding: 15px;\n"
         << "            border-radius: 5px;\n"
         << "        }\n"
         << "        #controls button {\n"
         << "            margin: 5px;\n"
         << "            padding: 8px 12px;\n"
         << "            background: #333;\n"
         << "            color: white;\n"
         << "            border: none;\n"
         << "            border-radius: 3px;\n"
         << "            cursor: pointer;\n"
         << "        }\n"
         << "        #controls button:hover { background: #555; }\n"
         << "        #controls button:active { background: #777; }\n"
         << "        #speed-control {\n"
         << "            margin: 10px 0;\n"
         << "        }\n"
         << "        #speed-slider {\n"
         << "            width: 150px;\n"
         << "            margin: 0 10px;\n"
         << "        }\n"
         << "        #scrub-control {\n"
         << "            margin: 10px 0;\n"
         << "        }\n"
         << "        #scrub-slider {\n"
         << "            width: 200px;\n"
         << "            margin: 0 10px;\n"
         << "        }\n"
         << "        #info {\n"
         << "            margin-top: 10px;\n"
         << "            font-size: 12px;\n"
         << "            color: #ccc;\n"
         << "        }\n"
         << "        #rotation-control {\n"
         << "            position: absolute;\n"
         << "            bottom: 20px;\n"
         << "            left: 50%;\n"
         << "            transform: translateX(-50%);\n"
         << "            z-index: 100;\n"
         << "        }\n"
         << "        #rotation-slider {\n"
         << "            width: 300px;\n"
         << "            height: 20px;\n"
         << "        }\n"
         << "    </style>\n"
         << "</head>\n"
         << "<body>\n"
         << "    <div id=\"controls\">\n"
         << "        <h3>Trajectory Animation</h3>\n"
         << "        <div>\n"
         << "            <button id=\"playBtn\">Play</button>\n"
         << "            <button id=\"pauseBtn\">Pause</button>\n"
         << "            <button id=\"resetBtn\">Reset</button>\n"
         << "        </div>\n"
         << "        <div id=\"speed-control\">\n"
         << "            Speed: <input type=\"range\" id=\"speed-slider\" min=\"0.1\" max=\"1.0\" step=\"0.1\" value=\"1.0\">\n"
         << "            <span id=\"speed-value\">1.0x (" << total_time_sec << "s)</span>\n"
         << "        </div>\n"
         << "        <div id=\"scrub-control\">\n"
         << "            Time: <input type=\"range\" id=\"scrub-slider\" min=\"0\" max=\"100\" value=\"0\">\n"
         << "            <span id=\"time-value\">0%</span>\n"
         << "        </div>\n"
         << "        <div id=\"info\">\n"
         << "            <p>Frame: <span id=\"frame-counter\">0</span> / <span id=\"total-frames\">" << sampled_indices.size() << "</span></p>\n"
         << "            <p>Mouse: Wheel=Zoom | Right-click=Pan</p>\n"
         << "            <p>Red: Obstacles | Blue: Arm | Green: EE Path</p>\n"
         << "        </div>\n"
         << "    </div>\n"
         << "    \n"
         << "    <div id=\"rotation-control\">\n"
         << "        <input type=\"range\" id=\"rotation-slider\" min=\"0\" max=\"360\" value=\"0\">\n"
         << "    </div>\n"
         << "    \n"
         << "    <script src=\"https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js\"></script>\n"
         << "    <script>\n"
         << "        // Scene setup\n"
         << "        const scene = new THREE.Scene();\n"
         << "        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);\n"
         << "        const renderer = new THREE.WebGLRenderer({ antialias: true });\n"
         << "        renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "        renderer.setClearColor(0x111111);\n"
         << "        renderer.shadowMap.enabled = true;\n"
         << "        renderer.shadowMap.type = THREE.PCFSoftShadowMap;\n"
         << "        document.body.appendChild(renderer.domElement);\n"
         << "        \n"
         << "        // Camera parameters\n"
         << "        const lookAtPoint = new THREE.Vector3(" << base_position.x() << ", " << base_position.y() << ", " << base_position.z() << ");\n"
         << "        const cameraDistance = 3.0;\n"
         << "        const cameraElevation = Math.PI / 4; // 45 degrees\n"
         << "        let cameraAzimuth = Math.PI / 4; // 45 degrees, will be controlled by slider\n"
         << "        \n"
         << "        // Function to update camera position\n"
         << "        function updateCameraPosition() {\n"
         << "            const x = lookAtPoint.x + cameraDistance * Math.cos(cameraElevation) * Math.cos(cameraAzimuth);\n"
         << "            const y = lookAtPoint.y + cameraDistance * Math.cos(cameraElevation) * Math.sin(cameraAzimuth);\n"
         << "            const z = lookAtPoint.z + cameraDistance * Math.sin(cameraElevation);\n"
         << "            camera.position.set(x, y, z);\n"
         << "            camera.lookAt(lookAtPoint);\n"
         << "        }\n"
         << "        \n"
         << "        // Lighting\n"
         << "        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);\n"
         << "        scene.add(ambientLight);\n"
         << "        \n"
         << "        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);\n"
         << "        directionalLight.position.set(5, 5, 5);\n"
         << "        directionalLight.castShadow = true;\n"
         << "        directionalLight.shadow.camera.near = 0.1;\n"
         << "        directionalLight.shadow.camera.far = 50;\n"
         << "        directionalLight.shadow.camera.left = -10;\n"
         << "        directionalLight.shadow.camera.right = 10;\n"
         << "        directionalLight.shadow.camera.top = 10;\n"
         << "        directionalLight.shadow.camera.bottom = -10;\n"
         << "        scene.add(directionalLight);\n"
         << "        \n"
         << "        // Materials\n"
         << "        const occupiedMaterial = new THREE.MeshLambertMaterial({ color: 0xff4444 });\n"
         << "        const armMaterial = new THREE.MeshLambertMaterial({ color: 0x4444ff });\n"
         << "        const traceMaterial = new THREE.LineBasicMaterial({ color: 0x44ff44, linewidth: 3 });\n"
         << "        \n"
         << "        // Animation state\n"
         << "        let isPlaying = false;\n"
         << "        let currentFrame = 0;\n"
         << "        let animationSpeed = 1.0;\n"
         << "        let lastTime = 0;\n"
         << "        const totalTimeSeconds = " << total_time_sec << ";\n"
         << "        \n"
         << "        // Trajectory data (sampled frames)\n"
         << "        const trajectoryData = [";
    
    // Add trajectory data - only for sampled frames
    for (size_t i = 0; i < sampled_indices.size(); ++i) {
        size_t frame_idx = sampled_indices[i];
        const gtsam::Vector& config = trajectory[frame_idx];
        
        // Compute forward kinematics to get sphere positions
        gtsam::Matrix sphere_centers = arm_model.sphereCentersMat(config);
        
        if (i > 0) file << ",";
        file << "\n            ["; // Start frame
        
        for (size_t j = 0; j < arm_model.nr_body_spheres(); ++j) {
            if (j > 0) file << ", ";
            file << "{"
                 << "x: " << sphere_centers(0, j) << ", "
                 << "y: " << sphere_centers(1, j) << ", "
                 << "z: " << sphere_centers(2, j) << ", "
                 << "r: " << arm_model.sphere_radius(j)
                 << "}";
        }
        file << "]"; // End frame
    }
    
    file << "\n        ];\n"
         << "        \n"
         << "        // End-effector trajectory (last sphere of each frame)\n"
         << "        const eeTrajectory = trajectoryData.map(frame => {\n"
         << "            const eeSphere = frame[frame.length - 1];\n"
         << "            return new THREE.Vector3(eeSphere.x, eeSphere.y, eeSphere.z);\n"
         << "        });\n"
         << "        \n"
         << "        // Create occupancy grid\n"
         << "        const occupiedGroup = new THREE.Group();\n"
         << "        const occupiedGeometry = new THREE.BoxGeometry();\n"
         << "        \n"
         << "        // Add occupancy grid data\n"
         << "        const cellSize = " << dataset.cell_size << ";\n"
         << "        const origin = [" << dataset.origin_x << ", " << dataset.origin_y << ", " << dataset.origin_z << "];\n"
         << "        \n";
    
    // Add occupancy grid - count occupied cells
    size_t occupied_count = 0;
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                if (dataset.map[i][j][k] > 0.5f) {
                    double world_x = dataset.origin_x + (i + 0.5) * dataset.cell_size;
                    double world_y = dataset.origin_y + (j + 0.5) * dataset.cell_size;
                    double world_z = dataset.origin_z + (k + 0.5) * dataset.cell_size;
                    
                    file << "        {\n"
                         << "            const cube = new THREE.Mesh(occupiedGeometry, occupiedMaterial);\n"
                         << "            cube.position.set(" << world_x << ", " << world_y << ", " << world_z << ");\n"
                         << "            cube.scale.set(cellSize, cellSize, cellSize);\n"
                         << "            cube.castShadow = true;\n"
                         << "            occupiedGroup.add(cube);\n"
                         << "        }\n";
                    occupied_count++;
                }
            }
        }
    }
    
    // Continue with JavaScript for animation logic
    file << "        scene.add(occupiedGroup);\n"
         << "        \n"
         << "        // Create arm spheres group\n"
         << "        const armGroup = new THREE.Group();\n"
         << "        const armSpheres = [];\n"
         << "        \n"
         << "        // Initialize arm spheres\n"
         << "        for (let i = 0; i < trajectoryData[0].length; i++) {\n"
         << "            const sphere = trajectoryData[0][i];\n"
         << "            const geometry = new THREE.SphereGeometry(sphere.r, 16, 16);\n"
         << "            const mesh = new THREE.Mesh(geometry, armMaterial);\n"
         << "            mesh.position.set(sphere.x, sphere.y, sphere.z);\n"
         << "            mesh.castShadow = true;\n"
         << "            armSpheres.push(mesh);\n"
         << "            armGroup.add(mesh);\n"
         << "        }\n"
         << "        scene.add(armGroup);\n"
         << "        \n"
         << "        // Create end-effector trace\n"
         << "        const traceGeometry = new THREE.BufferGeometry();\n"
         << "        const tracePositions = [];\n"
         << "        const traceLine = new THREE.Line(traceGeometry, traceMaterial);\n"
         << "        scene.add(traceLine);\n"
         << "        \n"
         << "        // Initialize camera position\n"
         << "        updateCameraPosition();\n"
         << "        \n"
         << "        // Animation controls\n"
         << "        const playBtn = document.getElementById('playBtn');\n"
         << "        const pauseBtn = document.getElementById('pauseBtn');\n"
         << "        const resetBtn = document.getElementById('resetBtn');\n"
         << "        const speedSlider = document.getElementById('speed-slider');\n"
         << "        const speedValue = document.getElementById('speed-value');\n"
         << "        const scrubSlider = document.getElementById('scrub-slider');\n"
         << "        const timeValue = document.getElementById('time-value');\n"
         << "        const frameCounter = document.getElementById('frame-counter');\n"
         << "        const rotationSlider = document.getElementById('rotation-slider');\n"
         << "        \n"
         << "        playBtn.addEventListener('click', () => {\n"
         << "            isPlaying = true;\n"
         << "            lastTime = performance.now();\n"
         << "        });\n"
         << "        \n"
         << "        pauseBtn.addEventListener('click', () => {\n"
         << "            isPlaying = false;\n"
         << "        });\n"
         << "        \n"
         << "        resetBtn.addEventListener('click', () => {\n"
         << "            currentFrame = 0;\n"
         << "            isPlaying = false;\n"
         << "            updateVisualization();\n"
         << "        });\n"
         << "        \n"
         << "        speedSlider.addEventListener('input', (e) => {\n"
         << "            animationSpeed = parseFloat(e.target.value);\n"
         << "            const playbackTime = (totalTimeSeconds / animationSpeed).toFixed(1);\n"
         << "            speedValue.textContent = animationSpeed.toFixed(1) + 'x (' + playbackTime + 's)';\n"
         << "        });\n"
         << "        \n"
         << "        scrubSlider.addEventListener('input', (e) => {\n"
         << "            const percent = parseFloat(e.target.value);\n"
         << "            currentFrame = Math.floor((percent / 100) * (trajectoryData.length - 1));\n"
         << "            timeValue.textContent = percent.toFixed(1) + '%';\n"
         << "            updateVisualization();\n"
         << "        });\n"
         << "        \n"
         << "        rotationSlider.addEventListener('input', (e) => {\n"
         << "            const degrees = parseFloat(e.target.value);\n"
         << "            cameraAzimuth = (degrees * Math.PI) / 180; // Convert to radians\n"
         << "            updateCameraPosition();\n"
         << "        });\n"
         << "        \n"
         << "        // Update visualization for current frame\n"
         << "        function updateVisualization() {\n"
         << "            if (trajectoryData.length === 0) return;\n"
         << "            \n"
         << "            // Update arm spheres\n"
         << "            const frame = trajectoryData[currentFrame];\n"
         << "            for (let i = 0; i < armSpheres.length; i++) {\n"
         << "                const sphere = frame[i];\n"
         << "                armSpheres[i].position.set(sphere.x, sphere.y, sphere.z);\n"
         << "            }\n"
         << "            \n"
         << "            // Update end-effector trace\n"
         << "            const traceLength = currentFrame + 1;\n"
         << "            const positions = new Float32Array(traceLength * 3);\n"
         << "            \n"
         << "            for (let i = 0; i < traceLength; i++) {\n"
         << "                const point = eeTrajectory[i];\n"
         << "                positions[i * 3] = point.x;\n"
         << "                positions[i * 3 + 1] = point.y;\n"
         << "                positions[i * 3 + 2] = point.z;\n"
         << "            }\n"
         << "            \n"
         << "            traceGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));\n"
         << "            \n"
         << "            // Update UI\n"
         << "            frameCounter.textContent = currentFrame + 1;\n"
         << "            const percent = (currentFrame / (trajectoryData.length - 1)) * 100;\n"
         << "            scrubSlider.value = percent;\n"
         << "            timeValue.textContent = percent.toFixed(1) + '%';\n"
         << "        }\n"
         << "        \n"
         << "        // Animation loop\n"
         << "        function animate(currentTime) {\n"
         << "            requestAnimationFrame(animate);\n"
         << "            \n"
         << "            if (isPlaying && trajectoryData.length > 0) {\n"
         << "                const deltaTime = currentTime - lastTime;\n"
         << "                const frameTime = (1000 / 30) / animationSpeed; // 30 FPS base rate\n"
         << "                \n"
         << "                if (deltaTime >= frameTime) {\n"
         << "                    currentFrame++;\n"
         << "                    if (currentFrame >= trajectoryData.length) {\n"
         << "                        currentFrame = 0; // Loop animation\n"
         << "                    }\n"
         << "                    updateVisualization();\n"
         << "                    lastTime = currentTime;\n"
         << "                }\n"
         << "            }\n"
         << "            \n"
         << "            renderer.render(scene, camera);\n"
         << "        }\n"
         << "        \n"
         << "        // Mouse controls (zoom and pan only)\n"
         << "        let mouseX = 0, mouseY = 0;\n"
         << "        let mouseDown = false;\n"
         << "        let mouseButton = 0;\n"
         << "        \n"
         << "        document.addEventListener('mousedown', (event) => {\n"
         << "            if (event.button === 2) { // Right click only\n"
         << "                mouseDown = true;\n"
         << "                mouseButton = event.button;\n"
         << "                mouseX = event.clientX;\n"
         << "                mouseY = event.clientY;\n"
         << "            }\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mouseup', () => {\n"
         << "            mouseDown = false;\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mousemove', (event) => {\n"
         << "            if (mouseDown && mouseButton === 2) { // Right click - pan\n"
         << "                const deltaX = event.clientX - mouseX;\n"
         << "                const deltaY = event.clientY - mouseY;\n"
         << "                \n"
         << "                const panSpeed = 0.005;\n"
         << "                const right = new THREE.Vector3().crossVectors(camera.up, camera.position.clone().sub(lookAtPoint).normalize());\n"
         << "                const up = camera.up.clone();\n"
         << "                \n"
         << "                lookAtPoint.add(right.multiplyScalar(-deltaX * panSpeed));\n"
         << "                lookAtPoint.add(up.multiplyScalar(deltaY * panSpeed));\n"
         << "                \n"
         << "                updateCameraPosition();\n"
         << "                \n"
         << "                mouseX = event.clientX;\n"
         << "                mouseY = event.clientY;\n"
         << "            }\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('wheel', (event) => {\n"
         << "            const scale = event.deltaY > 0 ? 1.1 : 0.9;\n"
         << "            cameraDistance = Math.max(0.5, Math.min(20, cameraDistance * scale));\n"
         << "            updateCameraPosition();\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('contextmenu', (event) => event.preventDefault());\n"
         << "        \n"
         << "        // Handle window resize\n"
         << "        window.addEventListener('resize', () => {\n"
         << "            camera.aspect = window.innerWidth / window.innerHeight;\n"
         << "            camera.updateProjectionMatrix();\n"
         << "            renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "        });\n"
         << "        \n"
         << "        // Initialize\n"
         << "        updateVisualization();\n"
         << "        animate(0);\n"
         << "    </script>\n"
         << "</body>\n"
         << "</html>";
    
    file.close();
    
    ::std::cout << "Visualization created with:" << ::std::endl;
    ::std::cout << "- Total time: " << total_time_sec << " seconds" << ::std::endl;
    ::std::cout << "- Sampled frames: " << sampled_indices.size() << " (~30Hz)" << ::std::endl;
    ::std::cout << "- " << arm_model.nr_body_spheres() << " arm spheres per frame" << ::std::endl;
    ::std::cout << "- " << occupied_count << " occupied grid cells" << ::std::endl;
    ::std::cout << "- Saved to: " << full_path << ::std::endl;
    
    // Try to open in Chrome browser on Linux
    bool opened = false;
    
    // Method 1: Try google-chrome
    ::std::string chrome_cmd = "google-chrome \"" + full_path + "\" > /dev/null 2>&1 &";
    if (::std::system("which google-chrome > /dev/null 2>&1") == 0) {
        if (::std::system(chrome_cmd.c_str()) == 0) {
            opened = true;
            ::std::cout << "Opened in Chrome browser" << ::std::endl;
        }
    }
    
    // Method 2: Try chromium-browser if google-chrome not available
    if (!opened) {
        ::std::string chromium_cmd = "chromium-browser \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which chromium-browser > /dev/null 2>&1") == 0) {
            if (::std::system(chromium_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opened in Chromium browser" << ::std::endl;
            }
        }
    }
    
    // Method 3: Try xdg-open as fallback
    if (!opened) {
        ::std::string xdg_cmd = "xdg-open \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which xdg-open > /dev/null 2>&1") == 0) {
            if (::std::system(xdg_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opened with default browser" << ::std::endl;
            }
        }
    }
    
    if (!opened) {
        ::std::cout << "\n=== MANUAL OPENING REQUIRED ===" << ::std::endl;
        ::std::cout << "Could not open browser automatically." << ::std::endl;
        ::std::cout << "Please manually open the file:" << ::std::endl;
        ::std::cout << "File location: " << full_path << ::std::endl;
        ::std::cout << "\nAlternative methods:" << ::std::endl;
        ::std::cout << "1. Open file manager and navigate to: ./TrajectoryAnimation/" << ::std::endl;
        ::std::cout << "2. Double-click: " << filename << ::std::endl;
        ::std::cout << "3. Or from command line:" << ::std::endl;
        ::std::cout << "   google-chrome \"" << full_path << "\"" << ::std::endl;
    } else {
        ::std::cout << "Animation should now be opening in your browser!" << ::std::endl;
        ::std::cout << "Use the rotation slider at the bottom to rotate around the arm base!" << ::std::endl;
    }
}

void visualizeTrajectory(
    const std::deque<Eigen::VectorXd>& trajectory,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose) {
    
    if (trajectory.empty()) {
        ::std::cerr << "Empty trajectory provided!" << ::std::endl;
        return;
    }
    
    // Calculate timing parameters
    double total_time_sec = trajectory.size() / 1000.0;  // 1000Hz trajectory
    
    // Create sampled frame indices (every 33rd frame + last frame)
    ::std::vector<size_t> sampled_indices;
    for (size_t i = 0; i < trajectory.size(); i += 33) {
        sampled_indices.push_back(i);
    }
    // Ensure last frame is included
    if (sampled_indices.back() != trajectory.size() - 1) {
        sampled_indices.push_back(trajectory.size() - 1);
    }
    
    ::std::cout << "Creating animated trajectory visualization..." << ::std::endl;
    ::std::cout << "Total trajectory time: " << total_time_sec << " seconds" << ::std::endl;
    ::std::cout << "Original frames: " << trajectory.size() << " (1000Hz)" << ::std::endl;
    ::std::cout << "Sampled frames: " << sampled_indices.size() << " (~30Hz)" << ::std::endl;
    ::std::cout << "Arm spheres: " << arm_model.nr_body_spheres() << ::std::endl;
    
    // Get camera look-at point from base pose
    gtsam::Point3 base_position = base_pose.translation();
    
    // Create timestamped filename
    auto now = ::std::chrono::system_clock::now();
    auto time_t = ::std::chrono::system_clock::to_time_t(now);
    auto ms = ::std::chrono::duration_cast<::std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    ::std::stringstream ss;
    ss << ::std::put_time(::std::localtime(&time_t), "%Y%m%d_%H%M%S");
    ss << "_" << ::std::setfill('0') << ::std::setw(3) << ms.count();
    
    ::std::string timestamp = ss.str();
    ::std::string filename = "trajectory_animation_" + timestamp + ".html";
    
    // Save to local Videos directory
    ::std::string videos_dir = "./TrajectoryAnimation";
    ::std::string full_path = videos_dir + "/" + filename;
    
    // Create directory if it doesn't exist
    ::std::string mkdir_cmd = "mkdir -p " + videos_dir;
    ::std::system(mkdir_cmd.c_str());
    
    ::std::ofstream file(full_path);
    
    if (!file.is_open()) {
        ::std::cerr << "Could not create file: " << full_path << ::std::endl;
        ::std::cerr << "Make sure the directory exists and is accessible" << ::std::endl;
        return;
    }
    
    // Write HTML header
    file << "<!DOCTYPE html>\n"
         << "<html>\n"
         << "<head>\n"
         << "    <title>Animated Trajectory Visualization</title>\n"
         << "    <style>\n"
         << "        body { \n"
         << "            margin: 0; \n"
         << "            padding: 0; \n"
         << "            background: #111; \n"
         << "            overflow: hidden; \n"
         << "            font-family: Arial, sans-serif;\n"
         << "        }\n"
         << "        #controls { \n"
         << "            position: absolute; \n"
         << "            top: 10px; \n"
         << "            left: 10px; \n"
         << "            color: white; \n"
         << "            z-index: 100;\n"
         << "            background: rgba(0,0,0,0.7);\n"
         << "            padding: 15px;\n"
         << "            border-radius: 5px;\n"
         << "        }\n"
         << "        #controls button {\n"
         << "            margin: 5px;\n"
         << "            padding: 8px 12px;\n"
         << "            background: #333;\n"
         << "            color: white;\n"
         << "            border: none;\n"
         << "            border-radius: 3px;\n"
         << "            cursor: pointer;\n"
         << "        }\n"
         << "        #controls button:hover { background: #555; }\n"
         << "        #controls button:active { background: #777; }\n"
         << "        #speed-control {\n"
         << "            margin: 10px 0;\n"
         << "        }\n"
         << "        #speed-slider {\n"
         << "            width: 150px;\n"
         << "            margin: 0 10px;\n"
         << "        }\n"
         << "        #scrub-control {\n"
         << "            margin: 10px 0;\n"
         << "        }\n"
         << "        #scrub-slider {\n"
         << "            width: 200px;\n"
         << "            margin: 0 10px;\n"
         << "        }\n"
         << "        #info {\n"
         << "            margin-top: 10px;\n"
         << "            font-size: 12px;\n"
         << "            color: #ccc;\n"
         << "        }\n"
         << "        #rotation-control {\n"
         << "            position: absolute;\n"
         << "            bottom: 20px;\n"
         << "            left: 50%;\n"
         << "            transform: translateX(-50%);\n"
         << "            z-index: 100;\n"
         << "        }\n"
         << "        #rotation-slider {\n"
         << "            width: 300px;\n"
         << "            height: 20px;\n"
         << "        }\n"
         << "    </style>\n"
         << "</head>\n"
         << "<body>\n"
         << "    <div id=\"controls\">\n"
         << "        <h3>Trajectory Animation</h3>\n"
         << "        <div>\n"
         << "            <button id=\"playBtn\">Play</button>\n"
         << "            <button id=\"pauseBtn\">Pause</button>\n"
         << "            <button id=\"resetBtn\">Reset</button>\n"
         << "        </div>\n"
         << "        <div id=\"speed-control\">\n"
         << "            Speed: <input type=\"range\" id=\"speed-slider\" min=\"0.1\" max=\"1.0\" step=\"0.1\" value=\"1.0\">\n"
         << "            <span id=\"speed-value\">1.0x (" << total_time_sec << "s)</span>\n"
         << "        </div>\n"
         << "        <div id=\"scrub-control\">\n"
         << "            Time: <input type=\"range\" id=\"scrub-slider\" min=\"0\" max=\"100\" value=\"0\">\n"
         << "            <span id=\"time-value\">0%</span>\n"
         << "        </div>\n"
         << "        <div id=\"info\">\n"
         << "            <p>Frame: <span id=\"frame-counter\">0</span> / <span id=\"total-frames\">" << sampled_indices.size() << "</span></p>\n"
         << "            <p>Mouse: Wheel=Zoom | Right-click=Pan</p>\n"
         << "            <p>Red: Obstacles | Blue: Arm | Green: EE Path</p>\n"
         << "        </div>\n"
         << "    </div>\n"
         << "    \n"
         << "    <div id=\"rotation-control\">\n"
         << "        <input type=\"range\" id=\"rotation-slider\" min=\"0\" max=\"360\" value=\"0\">\n"
         << "    </div>\n"
         << "    \n"
         << "    <script src=\"https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js\"></script>\n"
         << "    <script>\n"
         << "        // Scene setup\n"
         << "        const scene = new THREE.Scene();\n"
         << "        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);\n"
         << "        const renderer = new THREE.WebGLRenderer({ antialias: true });\n"
         << "        renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "        renderer.setClearColor(0x111111);\n"
         << "        renderer.shadowMap.enabled = true;\n"
         << "        renderer.shadowMap.type = THREE.PCFSoftShadowMap;\n"
         << "        document.body.appendChild(renderer.domElement);\n"
         << "        \n"
         << "        // Camera parameters\n"
         << "        const lookAtPoint = new THREE.Vector3(" << base_position.x() << ", " << base_position.y() << ", " << base_position.z() << ");\n"
         << "        const cameraDistance = 3.0;\n"
         << "        const cameraElevation = Math.PI / 4; // 45 degrees\n"
         << "        let cameraAzimuth = Math.PI / 4; // 45 degrees, will be controlled by slider\n"
         << "        \n"
         << "        // Function to update camera position\n"
         << "        function updateCameraPosition() {\n"
         << "            const x = lookAtPoint.x + cameraDistance * Math.cos(cameraElevation) * Math.cos(cameraAzimuth);\n"
         << "            const y = lookAtPoint.y + cameraDistance * Math.cos(cameraElevation) * Math.sin(cameraAzimuth);\n"
         << "            const z = lookAtPoint.z + cameraDistance * Math.sin(cameraElevation);\n"
         << "            camera.position.set(x, y, z);\n"
         << "            camera.lookAt(lookAtPoint);\n"
         << "        }\n"
         << "        \n"
         << "        // Lighting\n"
         << "        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);\n"
         << "        scene.add(ambientLight);\n"
         << "        \n"
         << "        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);\n"
         << "        directionalLight.position.set(5, 5, 5);\n"
         << "        directionalLight.castShadow = true;\n"
         << "        directionalLight.shadow.camera.near = 0.1;\n"
         << "        directionalLight.shadow.camera.far = 50;\n"
         << "        directionalLight.shadow.camera.left = -10;\n"
         << "        directionalLight.shadow.camera.right = 10;\n"
         << "        directionalLight.shadow.camera.top = 10;\n"
         << "        directionalLight.shadow.camera.bottom = -10;\n"
         << "        scene.add(directionalLight);\n"
         << "        \n"
         << "        // Materials\n"
         << "        const occupiedMaterial = new THREE.MeshLambertMaterial({ color: 0xff4444 });\n"
         << "        const armMaterial = new THREE.MeshLambertMaterial({ color: 0x4444ff });\n"
         << "        const traceMaterial = new THREE.LineBasicMaterial({ color: 0x44ff44, linewidth: 3 });\n"
         << "        \n"
         << "        // Animation state\n"
         << "        let isPlaying = false;\n"
         << "        let currentFrame = 0;\n"
         << "        let animationSpeed = 1.0;\n"
         << "        let lastTime = 0;\n"
         << "        const totalTimeSeconds = " << total_time_sec << ";\n"
         << "        \n"
         << "        // Trajectory data (sampled frames)\n"
         << "        const trajectoryData = [";
    
    // Add trajectory data - only for sampled frames
    for (size_t i = 0; i < sampled_indices.size(); ++i) {
        size_t frame_idx = sampled_indices[i];
        const Eigen::VectorXd& config_degrees = trajectory[frame_idx];
        
        // Convert from degrees to radians for forward kinematics
        Eigen::VectorXd config_radians = config_degrees * (M_PI / 180.0);
        
        // Compute forward kinematics to get sphere positions
        gtsam::Matrix sphere_centers = arm_model.sphereCentersMat(config_radians);
        
        if (i > 0) file << ",";
        file << "\n            ["; // Start frame
        
        for (size_t j = 0; j < arm_model.nr_body_spheres(); ++j) {
            if (j > 0) file << ", ";
            file << "{"
                 << "x: " << sphere_centers(0, j) << ", "
                 << "y: " << sphere_centers(1, j) << ", "
                 << "z: " << sphere_centers(2, j) << ", "
                 << "r: " << arm_model.sphere_radius(j)
                 << "}";
        }
        file << "]"; // End frame
    }
    
    file << "\n        ];\n"
         << "        \n"
         << "        // End-effector trajectory (last sphere of each frame)\n"
         << "        const eeTrajectory = trajectoryData.map(frame => {\n"
         << "            const eeSphere = frame[frame.length - 1];\n"
         << "            return new THREE.Vector3(eeSphere.x, eeSphere.y, eeSphere.z);\n"
         << "        });\n"
         << "        \n"
         << "        // Create occupancy grid\n"
         << "        const occupiedGroup = new THREE.Group();\n"
         << "        const occupiedGeometry = new THREE.BoxGeometry();\n"
         << "        \n"
         << "        // Add occupancy grid data\n"
         << "        const cellSize = " << dataset.cell_size << ";\n"
         << "        const origin = [" << dataset.origin_x << ", " << dataset.origin_y << ", " << dataset.origin_z << "];\n"
         << "        \n";
    
    // Add occupancy grid - count occupied cells
    size_t occupied_count = 0;
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                if (dataset.map[i][j][k] > 0.5f) {
                    double world_x = dataset.origin_x + (i + 0.5) * dataset.cell_size;
                    double world_y = dataset.origin_y + (j + 0.5) * dataset.cell_size;
                    double world_z = dataset.origin_z + (k + 0.5) * dataset.cell_size;
                    
                    file << "        {\n"
                         << "            const cube = new THREE.Mesh(occupiedGeometry, occupiedMaterial);\n"
                         << "            cube.position.set(" << world_x << ", " << world_y << ", " << world_z << ");\n"
                         << "            cube.scale.set(cellSize, cellSize, cellSize);\n"
                         << "            cube.castShadow = true;\n"
                         << "            occupiedGroup.add(cube);\n"
                         << "        }\n";
                    occupied_count++;
                }
            }
        }
    }
    
    // Continue with JavaScript for animation logic
    file << "        scene.add(occupiedGroup);\n"
         << "        \n"
         << "        // Create arm spheres group\n"
         << "        const armGroup = new THREE.Group();\n"
         << "        const armSpheres = [];\n"
         << "        \n"
         << "        // Initialize arm spheres\n"
         << "        for (let i = 0; i < trajectoryData[0].length; i++) {\n"
         << "            const sphere = trajectoryData[0][i];\n"
         << "            const geometry = new THREE.SphereGeometry(sphere.r, 16, 16);\n"
         << "            const mesh = new THREE.Mesh(geometry, armMaterial);\n"
         << "            mesh.position.set(sphere.x, sphere.y, sphere.z);\n"
         << "            mesh.castShadow = true;\n"
         << "            armSpheres.push(mesh);\n"
         << "            armGroup.add(mesh);\n"
         << "        }\n"
         << "        scene.add(armGroup);\n"
         << "        \n"
         << "        // Create end-effector trace\n"
         << "        const traceGeometry = new THREE.BufferGeometry();\n"
         << "        const tracePositions = [];\n"
         << "        const traceLine = new THREE.Line(traceGeometry, traceMaterial);\n"
         << "        scene.add(traceLine);\n"
         << "        \n"
         << "        // Initialize camera position\n"
         << "        updateCameraPosition();\n"
         << "        \n"
         << "        // Animation controls\n"
         << "        const playBtn = document.getElementById('playBtn');\n"
         << "        const pauseBtn = document.getElementById('pauseBtn');\n"
         << "        const resetBtn = document.getElementById('resetBtn');\n"
         << "        const speedSlider = document.getElementById('speed-slider');\n"
         << "        const speedValue = document.getElementById('speed-value');\n"
         << "        const scrubSlider = document.getElementById('scrub-slider');\n"
         << "        const timeValue = document.getElementById('time-value');\n"
         << "        const frameCounter = document.getElementById('frame-counter');\n"
         << "        const rotationSlider = document.getElementById('rotation-slider');\n"
         << "        \n"
         << "        playBtn.addEventListener('click', () => {\n"
         << "            isPlaying = true;\n"
         << "            lastTime = performance.now();\n"
         << "        });\n"
         << "        \n"
         << "        pauseBtn.addEventListener('click', () => {\n"
         << "            isPlaying = false;\n"
         << "        });\n"
         << "        \n"
         << "        resetBtn.addEventListener('click', () => {\n"
         << "            currentFrame = 0;\n"
         << "            isPlaying = false;\n"
         << "            updateVisualization();\n"
         << "        });\n"
         << "        \n"
         << "        speedSlider.addEventListener('input', (e) => {\n"
         << "            animationSpeed = parseFloat(e.target.value);\n"
         << "            const playbackTime = (totalTimeSeconds / animationSpeed).toFixed(1);\n"
         << "            speedValue.textContent = animationSpeed.toFixed(1) + 'x (' + playbackTime + 's)';\n"
         << "        });\n"
         << "        \n"
         << "        scrubSlider.addEventListener('input', (e) => {\n"
         << "            const percent = parseFloat(e.target.value);\n"
         << "            currentFrame = Math.floor((percent / 100) * (trajectoryData.length - 1));\n"
         << "            timeValue.textContent = percent.toFixed(1) + '%';\n"
         << "            updateVisualization();\n"
         << "        });\n"
         << "        \n"
         << "        rotationSlider.addEventListener('input', (e) => {\n"
         << "            const degrees = parseFloat(e.target.value);\n"
         << "            cameraAzimuth = (degrees * Math.PI) / 180; // Convert to radians\n"
         << "            updateCameraPosition();\n"
         << "        });\n"
         << "        \n"
         << "        // Update visualization for current frame\n"
         << "        function updateVisualization() {\n"
         << "            if (trajectoryData.length === 0) return;\n"
         << "            \n"
         << "            // Update arm spheres\n"
         << "            const frame = trajectoryData[currentFrame];\n"
         << "            for (let i = 0; i < armSpheres.length; i++) {\n"
         << "                const sphere = frame[i];\n"
         << "                armSpheres[i].position.set(sphere.x, sphere.y, sphere.z);\n"
         << "            }\n"
         << "            \n"
         << "            // Update end-effector trace\n"
         << "            const traceLength = currentFrame + 1;\n"
         << "            const positions = new Float32Array(traceLength * 3);\n"
         << "            \n"
         << "            for (let i = 0; i < traceLength; i++) {\n"
         << "                const point = eeTrajectory[i];\n"
         << "                positions[i * 3] = point.x;\n"
         << "                positions[i * 3 + 1] = point.y;\n"
         << "                positions[i * 3 + 2] = point.z;\n"
         << "            }\n"
         << "            \n"
         << "            traceGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));\n"
         << "            \n"
         << "            // Update UI\n"
         << "            frameCounter.textContent = currentFrame + 1;\n"
         << "            const percent = (currentFrame / (trajectoryData.length - 1)) * 100;\n"
         << "            scrubSlider.value = percent;\n"
         << "            timeValue.textContent = percent.toFixed(1) + '%';\n"
         << "        }\n"
         << "        \n"
         << "        // Animation loop\n"
         << "        function animate(currentTime) {\n"
         << "            requestAnimationFrame(animate);\n"
         << "            \n"
         << "            if (isPlaying && trajectoryData.length > 0) {\n"
         << "                const deltaTime = currentTime - lastTime;\n"
         << "                const frameTime = (1000 / 30) / animationSpeed; // 30 FPS base rate\n"
         << "                \n"
         << "                if (deltaTime >= frameTime) {\n"
         << "                    currentFrame++;\n"
         << "                    if (currentFrame >= trajectoryData.length) {\n"
         << "                        currentFrame = 0; // Loop animation\n"
         << "                    }\n"
         << "                    updateVisualization();\n"
         << "                    lastTime = currentTime;\n"
         << "                }\n"
         << "            }\n"
         << "            \n"
         << "            renderer.render(scene, camera);\n"
         << "        }\n"
         << "        \n"
         << "        // Mouse controls (zoom and pan only)\n"
         << "        let mouseX = 0, mouseY = 0;\n"
         << "        let mouseDown = false;\n"
         << "        let mouseButton = 0;\n"
         << "        \n"
         << "        document.addEventListener('mousedown', (event) => {\n"
         << "            if (event.button === 2) { // Right click only\n"
         << "                mouseDown = true;\n"
         << "                mouseButton = event.button;\n"
         << "                mouseX = event.clientX;\n"
         << "                mouseY = event.clientY;\n"
         << "            }\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mouseup', () => {\n"
         << "            mouseDown = false;\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mousemove', (event) => {\n"
         << "            if (mouseDown && mouseButton === 2) { // Right click - pan\n"
         << "                const deltaX = event.clientX - mouseX;\n"
         << "                const deltaY = event.clientY - mouseY;\n"
         << "                \n"
         << "                const panSpeed = 0.005;\n"
         << "                const right = new THREE.Vector3().crossVectors(camera.up, camera.position.clone().sub(lookAtPoint).normalize());\n"
         << "                const up = camera.up.clone();\n"
         << "                \n"
         << "                lookAtPoint.add(right.multiplyScalar(-deltaX * panSpeed));\n"
         << "                lookAtPoint.add(up.multiplyScalar(deltaY * panSpeed));\n"
         << "                \n"
         << "                updateCameraPosition();\n"
         << "                \n"
         << "                mouseX = event.clientX;\n"
         << "                mouseY = event.clientY;\n"
         << "            }\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('wheel', (event) => {\n"
         << "            const scale = event.deltaY > 0 ? 1.1 : 0.9;\n"
         << "            cameraDistance = Math.max(0.5, Math.min(20, cameraDistance * scale));\n"
         << "            updateCameraPosition();\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('contextmenu', (event) => event.preventDefault());\n"
         << "        \n"
         << "        // Handle window resize\n"
         << "        window.addEventListener('resize', () => {\n"
         << "            camera.aspect = window.innerWidth / window.innerHeight;\n"
         << "            camera.updateProjectionMatrix();\n"
         << "            renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "        });\n"
         << "        \n"
         << "        // Initialize\n"
         << "        updateVisualization();\n"
         << "        animate(0);\n"
         << "    </script>\n"
         << "</body>\n"
         << "</html>";
    
    file.close();
    
    ::std::cout << "Visualization created with:" << ::std::endl;
    ::std::cout << "- Total time: " << total_time_sec << " seconds" << ::std::endl;
    ::std::cout << "- Sampled frames: " << sampled_indices.size() << " (~30Hz)" << ::std::endl;
    ::std::cout << "- " << arm_model.nr_body_spheres() << " arm spheres per frame" << ::std::endl;
    ::std::cout << "- " << occupied_count << " occupied grid cells" << ::std::endl;
    ::std::cout << "- Saved to: " << full_path << ::std::endl;
    
    // Try to open in Chrome browser on Linux
    bool opened = false;
    
    // Method 1: Try google-chrome
    ::std::string chrome_cmd = "google-chrome \"" + full_path + "\" > /dev/null 2>&1 &";
    if (::std::system("which google-chrome > /dev/null 2>&1") == 0) {
        if (::std::system(chrome_cmd.c_str()) == 0) {
            opened = true;
            ::std::cout << "Opened in Chrome browser" << ::std::endl;
        }
    }
    
    // Method 2: Try chromium-browser if google-chrome not available
    if (!opened) {
        ::std::string chromium_cmd = "chromium-browser \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which chromium-browser > /dev/null 2>&1") == 0) {
            if (::std::system(chromium_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opened in Chromium browser" << ::std::endl;
            }
        }
    }
    
    // Method 3: Try xdg-open as fallback
    if (!opened) {
        ::std::string xdg_cmd = "xdg-open \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which xdg-open > /dev/null 2>&1") == 0) {
            if (::std::system(xdg_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opened with default browser" << ::std::endl;
            }
        }
    }
    
    if (!opened) {
        ::std::cout << "\n=== MANUAL OPENING REQUIRED ===" << ::std::endl;
        ::std::cout << "Could not open browser automatically." << ::std::endl;
        ::std::cout << "Please manually open the file:" << ::std::endl;
        ::std::cout << "File location: " << full_path << ::std::endl;
        ::std::cout << "\nAlternative methods:" << ::std::endl;
        ::std::cout << "1. Open file manager and navigate to: ./TrajectoryAnimation/" << ::std::endl;
        ::std::cout << "2. Double-click: " << filename << ::std::endl;
        ::std::cout << "3. Or from command line:" << ::std::endl;
        ::std::cout << "   google-chrome \"" << full_path << "\"" << ::std::endl;
    }

}


void storeEventToYaml(
    const std::deque<Eigen::VectorXd>& trajectory,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose) {
    
    if (trajectory.empty()) {
        std::cerr << "Empty trajectory provided!" << std::endl;
        return;
    }
    
    // Calculate timing parameters
    double total_time_sec = trajectory.size() / 1000.0;  // 1000Hz trajectory
    
    // Create sampled frame indices (every 33rd frame + last frame)
    std::vector<size_t> sampled_indices;
    for (size_t i = 0; i < trajectory.size(); i += 33) {
        sampled_indices.push_back(i);
    }
    // Ensure last frame is included
    if (sampled_indices.back() != trajectory.size() - 1) {
        sampled_indices.push_back(trajectory.size() - 1);
    }
    
    std::cout << "Creating trajectory data export..." << std::endl;
    std::cout << "Total trajectory time: " << total_time_sec << " seconds" << std::endl;
    std::cout << "Original frames: " << trajectory.size() << " (1000Hz)" << std::endl;
    std::cout << "Sampled frames: " << sampled_indices.size() << " (~30Hz)" << std::endl;
    
    // Create timestamped filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
    
    std::string timestamp = ss.str();
    std::string filename = "trajectory_animation_" + timestamp + ".yaml";
    
    // Save to logs directory
    std::string logs_dir = "../logs";
    std::string full_path = logs_dir + "/" + filename;
    
    // Create directory if it doesn't exist
    std::string mkdir_cmd = "mkdir -p " + logs_dir;
    std::system(mkdir_cmd.c_str());
    
    std::ofstream file(full_path);
    
    if (!file.is_open()) {
        std::cerr << "Could not create file: " << full_path << std::endl;
        std::cerr << "Make sure the logs directory exists and is accessible" << std::endl;
        return;
    }
    
    // Extract base pose data
    gtsam::Point3 base_position = base_pose.translation();
    gtsam::Matrix3 base_rotation = base_pose.rotation().matrix();
    
    // Write YAML header and metadata
    file << "# Robot Trajectory Animation Data\n";
    file << "# Generated: " << timestamp << "\n\n";
    
    file << "metadata:\n";
    file << "  total_time_sec: " << total_time_sec << "\n";
    file << "  trajectory_frequency: 1000  # Hz\n";
    file << "  sampling_rate: 33  # every 33rd frame\n";
    file << "  total_frames: " << trajectory.size() << "\n";
    file << "  sampled_frames: " << sampled_indices.size() << "\n";
    
    // Write robot base frame
    file << "robot_base:\n";
    file << "  position: [" << base_position.x() << ", " << base_position.y() << ", " << base_position.z() << "]\n";
    file << "  rotation_matrix:\n";
    for (int i = 0; i < 3; ++i) {
        file << "    - [";
        for (int j = 0; j < 3; ++j) {
            file << base_rotation(i, j);
            if (j < 2) file << ", ";
        }
        file << "]\n";
    }
    file << "\n";
    
    // Write occupancy grid metadata
    file << "occupancy_grid:\n";
    file << "  dimensions: [" << dataset.rows << ", " << dataset.cols << ", " << dataset.z << "]\n";
    file << "  origin: [" << dataset.origin_x << ", " << dataset.origin_y << ", " << dataset.origin_z << "]\n";
    file << "  cell_size: " << dataset.cell_size << "\n";
    
    // Count occupied cells for info
    size_t occupied_count = 0;
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                if (dataset.map[i][j][k] > 0.5f) {
                    occupied_count++;
                }
            }
        }
    }
    file << "  occupied_cells: " << occupied_count << "\n";
    
    // Write occupancy grid data as 3D binary matrix
    file << "  data:\n";
    for (size_t i = 0; i < dataset.rows; ++i) {
        file << "    - # row " << i << "\n";
        for (size_t j = 0; j < dataset.cols; ++j) {
            file << "      - [";
            for (size_t k = 0; k < dataset.z; ++k) {
                file << (dataset.map[i][j][k] > 0.5f ? 1 : 0);
                if (k < dataset.z - 1) file << ", ";
            }
            file << "]\n";
        }
    }
    file << "\n";
    
    // Write trajectory data
    file << "trajectory:\n";
    for (size_t i = 0; i < sampled_indices.size(); ++i) {
        size_t frame_idx = sampled_indices[i];
        const gtsam::Vector& config = trajectory[frame_idx];
        
        file << "  - frame: " << frame_idx << "\n";
        file << "    joint_config: [";
        for (int j = 0; j < config.size(); ++j) {
            file << config(j);
            if (j < config.size() - 1) file << ", ";
        }
        file << "]\n";
    }
    
    file.close();
    
    std::cout << "Trajectory data exported successfully!" << std::endl;
    std::cout << "- Saved to: " << full_path << std::endl;
}

void visualizeTrajectoryStatic(
    const std::vector<double>& configuration_degrees,
    const gpmp2::ArmModel& arm_model,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose) {
    
    ::std::cout << "Creating static arm visualization..." << ::std::endl;
    ::std::cout << "Configuration dimensions: " << configuration_degrees.size() << ::std::endl;
    ::std::cout << "Arm spheres: " << arm_model.nr_body_spheres() << ::std::endl;
    
    // Convert std::vector<double> (degrees) to gtsam::Vector (radians)
    gtsam::Vector configuration(configuration_degrees.size());
    for (size_t i = 0; i < configuration_degrees.size(); ++i) {
        configuration(i) = configuration_degrees[i] * (M_PI / 180.0);
    }
    
    // Compute forward kinematics to get sphere positions
    gtsam::Matrix sphere_centers = arm_model.sphereCentersMat(configuration);
    
    // Get end-effector pose using forward kinematics
    std::vector<gtsam::Pose3> joint_poses;
    arm_model.fk_model().forwardKinematics(configuration, {}, joint_poses);
    gtsam::Pose3 ee_pose = joint_poses.back();
    
    // Get camera look-at point from base pose
    gtsam::Point3 base_position = base_pose.translation();
    
    // Create timestamped filename
    auto now = ::std::chrono::system_clock::now();
    auto time_t = ::std::chrono::system_clock::to_time_t(now);
    auto ms = ::std::chrono::duration_cast<::std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    ::std::stringstream ss;
    ss << ::std::put_time(::std::localtime(&time_t), "%Y%m%d_%H%M%S");
    ss << "_" << ::std::setfill('0') << ::std::setw(3) << ms.count();
    
    ::std::string timestamp = ss.str();
    ::std::string filename = "static_arm_visualization_" + timestamp + ".html";
    
    // Save to local Videos directory
    ::std::string videos_dir = "./TrajectoryAnimation";
    ::std::string full_path = videos_dir + "/" + filename;
    
    // Create directory if it doesn't exist
    ::std::string mkdir_cmd = "mkdir -p " + videos_dir;
    ::std::system(mkdir_cmd.c_str());
    
    ::std::ofstream file(full_path);
    
    if (!file.is_open()) {
        ::std::cerr << "Could not create file: " << full_path << ::std::endl;
        ::std::cerr << "Make sure the directory exists and is accessible" << ::std::endl;
        return;
    }
    
    // Write HTML header
    file << "<!DOCTYPE html>\n"
         << "<html>\n"
         << "<head>\n"
         << "    <title>Static Arm Visualization</title>\n"
         << "    <style>\n"
         << "        body { \n"
         << "            margin: 0; \n"
         << "            padding: 0; \n"
         << "            background: #111; \n"
         << "            overflow: hidden; \n"
         << "            font-family: Arial, sans-serif;\n"
         << "        }\n"
         << "        #info { \n"
         << "            position: absolute; \n"
         << "            top: 10px; \n"
         << "            left: 10px; \n"
         << "            color: white; \n"
         << "            z-index: 100;\n"
         << "            background: rgba(0,0,0,0.7);\n"
         << "            padding: 15px;\n"
         << "            border-radius: 5px;\n"
         << "            font-size: 12px;\n"
         << "        }\n"
         << "        #rotation-control {\n"
         << "            position: absolute;\n"
         << "            bottom: 20px;\n"
         << "            left: 50%;\n"
         << "            transform: translateX(-50%);\n"
         << "            z-index: 100;\n"
         << "        }\n"
         << "        #rotation-slider {\n"
         << "            width: 300px;\n"
         << "            height: 20px;\n"
         << "        }\n"
         << "    </style>\n"
         << "</head>\n"
         << "<body>\n"
         << "    <div id=\"info\">\n"
         << "        <h3>Static Arm Visualization</h3>\n"
         << "        <p>Mouse: Wheel=Zoom | Right-click=Pan</p>\n"
         << "        <p>Red: Obstacles | Green: Arm</p>\n"
         << "        <p>RGB Axes: X=Red, Y=Green, Z=Blue</p>\n"
         << "    </div>\n"
         << "    \n"
         << "    <div id=\"rotation-control\">\n"
         << "        <input type=\"range\" id=\"rotation-slider\" min=\"0\" max=\"360\" value=\"0\">\n"
         << "    </div>\n"
         << "    \n"
         << "    <script src=\"https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js\"></script>\n"
         << "    <script>\n"
         << "        // Scene setup\n"
         << "        const scene = new THREE.Scene();\n"
         << "        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);\n"
         << "        const renderer = new THREE.WebGLRenderer({ antialias: true });\n"
         << "        renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "        renderer.setClearColor(0x111111);\n"
         << "        renderer.shadowMap.enabled = true;\n"
         << "        renderer.shadowMap.type = THREE.PCFSoftShadowMap;\n"
         << "        document.body.appendChild(renderer.domElement);\n"
         << "        \n"
         << "        // Camera parameters\n"
         << "        const lookAtPoint = new THREE.Vector3(" << base_position.x() << ", " << base_position.y() << ", " << base_position.z() << ");\n"
         << "        const cameraDistance = 3.0;\n"
         << "        const cameraElevation = Math.PI / 4; // 45 degrees\n"
         << "        let cameraAzimuth = Math.PI / 4; // 45 degrees, will be controlled by slider\n"
         << "        \n"
         << "        // Function to update camera position\n"
         << "        function updateCameraPosition() {\n"
         << "            const x = lookAtPoint.x + cameraDistance * Math.cos(cameraElevation) * Math.cos(cameraAzimuth);\n"
         << "            const y = lookAtPoint.y + cameraDistance * Math.cos(cameraElevation) * Math.sin(cameraAzimuth);\n"
         << "            const z = lookAtPoint.z + cameraDistance * Math.sin(cameraElevation);\n"
         << "            camera.position.set(x, y, z);\n"
         << "            camera.lookAt(lookAtPoint);\n"
         << "        }\n"
         << "        \n"
         << "        // Function to create coordinate frame arrows\n"
         << "        function createCoordinateFrame(position, orientation, scale = 0.2) {\n"
         << "            const group = new THREE.Group();\n"
         << "            \n"
         << "            // Arrow geometry and materials\n"
         << "            const arrowLength = scale;\n"
         << "            const arrowHeadLength = scale * 0.3;\n"
         << "            const arrowHeadWidth = scale * 0.1;\n"
         << "            \n"
         << "            // X axis (red)\n"
         << "            const xArrow = new THREE.ArrowHelper(\n"
         << "                new THREE.Vector3(1, 0, 0),\n"
         << "                new THREE.Vector3(0, 0, 0),\n"
         << "                arrowLength, 0xff0000, arrowHeadLength, arrowHeadWidth\n"
         << "            );\n"
         << "            group.add(xArrow);\n"
         << "            \n"
         << "            // Y axis (green)\n"
         << "            const yArrow = new THREE.ArrowHelper(\n"
         << "                new THREE.Vector3(0, 1, 0),\n"
         << "                new THREE.Vector3(0, 0, 0),\n"
         << "                arrowLength, 0x00ff00, arrowHeadLength, arrowHeadWidth\n"
         << "            );\n"
         << "            group.add(yArrow);\n"
         << "            \n"
         << "            // Z axis (blue)\n"
         << "            const zArrow = new THREE.ArrowHelper(\n"
         << "                new THREE.Vector3(0, 0, 1),\n"
         << "                new THREE.Vector3(0, 0, 0),\n"
         << "                arrowLength, 0x0000ff, arrowHeadLength, arrowHeadWidth\n"
         << "            );\n"
         << "            group.add(zArrow);\n"
         << "            \n"
         << "            // Set position\n"
         << "            group.position.set(position.x, position.y, position.z);\n"
         << "            \n"
         << "            // Apply orientation if provided\n"
         << "            if (orientation) {\n"
         << "                group.setRotationFromMatrix(orientation);\n"
         << "            }\n"
         << "            \n"
         << "            return group;\n"
         << "        }\n"
         << "        \n"
         << "        // Lighting\n"
         << "        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);\n"
         << "        scene.add(ambientLight);\n"
         << "        \n"
         << "        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);\n"
         << "        directionalLight.position.set(5, 5, 5);\n"
         << "        directionalLight.castShadow = true;\n"
         << "        directionalLight.shadow.camera.near = 0.1;\n"
         << "        directionalLight.shadow.camera.far = 50;\n"
         << "        directionalLight.shadow.camera.left = -10;\n"
         << "        directionalLight.shadow.camera.right = 10;\n"
         << "        directionalLight.shadow.camera.top = 10;\n"
         << "        directionalLight.shadow.camera.bottom = -10;\n"
         << "        scene.add(directionalLight);\n"
         << "        \n"
         << "        // Materials\n"
         << "        const occupiedMaterial = new THREE.MeshLambertMaterial({ color: 0xff4444 });\n"
         << "        const armMaterial = new THREE.MeshLambertMaterial({ color: 0x44ff44 }); // Green color for arm\n"
         << "        \n"
         << "        // Create occupancy grid\n"
         << "        const occupiedGroup = new THREE.Group();\n"
         << "        const occupiedGeometry = new THREE.BoxGeometry();\n"
         << "        \n"
         << "        // Add occupancy grid data\n"
         << "        const cellSize = " << dataset.cell_size << ";\n"
         << "        const origin = [" << dataset.origin_x << ", " << dataset.origin_y << ", " << dataset.origin_z << "];\n"
         << "        \n";
    
    // Add occupancy grid - count occupied cells
    size_t occupied_count = 0;
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                if (dataset.map[i][j][k] > 0.5f) {
                    double world_x = dataset.origin_x + (i + 0.5) * dataset.cell_size;
                    double world_y = dataset.origin_y + (j + 0.5) * dataset.cell_size;
                    double world_z = dataset.origin_z + (k + 0.5) * dataset.cell_size;
                    
                    file << "        {\n"
                         << "            const cube = new THREE.Mesh(occupiedGeometry, occupiedMaterial);\n"
                         << "            cube.position.set(" << world_x << ", " << world_y << ", " << world_z << ");\n"
                         << "            cube.scale.set(cellSize, cellSize, cellSize);\n"
                         << "            cube.castShadow = true;\n"
                         << "            occupiedGroup.add(cube);\n"
                         << "        }\n";
                    occupied_count++;
                }
            }
        }
    }
    
    // Get base pose rotation matrix
    gtsam::Matrix3 base_rotation = base_pose.rotation().matrix();
    
    // Get end-effector position and rotation
    gtsam::Point3 ee_position = ee_pose.translation();
    gtsam::Matrix3 ee_rotation = ee_pose.rotation().matrix();
    
    // Continue with JavaScript
    file << "        scene.add(occupiedGroup);\n"
         << "        \n"
         << "        // Create arm spheres group\n"
         << "        const armGroup = new THREE.Group();\n"
         << "        \n"
         << "        // Add arm spheres at static configuration\n";
    
    // Add static arm spheres
    for (size_t j = 0; j < arm_model.nr_body_spheres(); ++j) {
        file << "        {\n"
             << "            const geometry = new THREE.SphereGeometry(" << arm_model.sphere_radius(j) << ", 16, 16);\n"
             << "            const mesh = new THREE.Mesh(geometry, armMaterial);\n"
             << "            mesh.position.set(" << sphere_centers(0, j) << ", " << sphere_centers(1, j) << ", " << sphere_centers(2, j) << ");\n"
             << "            mesh.castShadow = true;\n"
             << "            armGroup.add(mesh);\n"
             << "        }\n";
    }
    
    file << "        scene.add(armGroup);\n"
         << "        \n"
         << "        // Add coordinate frames\n"
         << "        \n"
         << "        // World origin frame\n"
         << "        const worldFrame = createCoordinateFrame(new THREE.Vector3(0, 0, 0), null, 0.3);\n"
         << "        scene.add(worldFrame);\n"
         << "        \n"
         << "        // Robot base frame\n"
         << "        const baseRotationMatrix = new THREE.Matrix4().set(\n"
         << "            " << base_rotation(0,0) << ", " << base_rotation(0,1) << ", " << base_rotation(0,2) << ", 0,\n"
         << "            " << base_rotation(1,0) << ", " << base_rotation(1,1) << ", " << base_rotation(1,2) << ", 0,\n"
         << "            " << base_rotation(2,0) << ", " << base_rotation(2,1) << ", " << base_rotation(2,2) << ", 0,\n"
         << "            0, 0, 0, 1\n"
         << "        );\n"
         << "        const baseFrame = createCoordinateFrame(\n"
         << "            new THREE.Vector3(" << base_position.x() << ", " << base_position.y() << ", " << base_position.z() << "),\n"
         << "            baseRotationMatrix,\n"
         << "            0.25\n"
         << "        );\n"
         << "        scene.add(baseFrame);\n"
         << "        \n"
         << "        // End-effector frame\n"
         << "        const eeRotationMatrix = new THREE.Matrix4().set(\n"
         << "            " << ee_rotation(0,0) << ", " << ee_rotation(1,0) << ", " << ee_rotation(2,0) << ", 0,\n"
         << "            " << ee_rotation(0,2) << ", " << ee_rotation(1,2) << ", " << ee_rotation(2,2) << ", 0,\n"
         << "            " << ee_rotation(0,1) << ", " << ee_rotation(1,1) << ", " << ee_rotation(2,1) << ", 0,\n"
         << "            0, 0, 0, 1\n"
         << "        );\n"
         << "        const eeFrame = createCoordinateFrame(\n"
         << "            new THREE.Vector3(" << ee_position.x() << ", " << ee_position.y() << ", " << ee_position.z() << "),\n"
         << "            eeRotationMatrix,\n"
         << "            0.15\n"
         << "        );\n"
         << "        scene.add(eeFrame);\n"
         << "        \n"
         << "        // Initialize camera position\n"
         << "        updateCameraPosition();\n"
         << "        \n"
         << "        // Rotation control\n"
         << "        const rotationSlider = document.getElementById('rotation-slider');\n"
         << "        \n"
         << "        rotationSlider.addEventListener('input', (e) => {\n"
         << "            const degrees = parseFloat(e.target.value);\n"
         << "            cameraAzimuth = (degrees * Math.PI) / 180; // Convert to radians\n"
         << "            updateCameraPosition();\n"
         << "        });\n"
         << "        \n"
         << "        // Mouse controls (zoom and pan only)\n"
         << "        let mouseX = 0, mouseY = 0;\n"
         << "        let mouseDown = false;\n"
         << "        let mouseButton = 0;\n"
         << "        \n"
         << "        document.addEventListener('mousedown', (event) => {\n"
         << "            if (event.button === 2) { // Right click only\n"
         << "                mouseDown = true;\n"
         << "                mouseButton = event.button;\n"
         << "                mouseX = event.clientX;\n"
         << "                mouseY = event.clientY;\n"
         << "            }\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mouseup', () => {\n"
         << "            mouseDown = false;\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mousemove', (event) => {\n"
         << "            if (mouseDown && mouseButton === 2) { // Right click - pan\n"
         << "                const deltaX = event.clientX - mouseX;\n"
         << "                const deltaY = event.clientY - mouseY;\n"
         << "                \n"
         << "                const panSpeed = 0.005;\n"
         << "                const right = new THREE.Vector3().crossVectors(camera.up, camera.position.clone().sub(lookAtPoint).normalize());\n"
         << "                const up = camera.up.clone();\n"
         << "                \n"
         << "                lookAtPoint.add(right.multiplyScalar(-deltaX * panSpeed));\n"
         << "                lookAtPoint.add(up.multiplyScalar(deltaY * panSpeed));\n"
         << "                \n"
         << "                updateCameraPosition();\n"
         << "                \n"
         << "                mouseX = event.clientX;\n"
         << "                mouseY = event.clientY;\n"
         << "            }\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('wheel', (event) => {\n"
         << "            const scale = event.deltaY > 0 ? 1.1 : 0.9;\n"
         << "            cameraDistance = Math.max(0.5, Math.min(20, cameraDistance * scale));\n"
         << "            updateCameraPosition();\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('contextmenu', (event) => event.preventDefault());\n"
         << "        \n"
         << "        // Handle window resize\n"
         << "        window.addEventListener('resize', () => {\n"
         << "            camera.aspect = window.innerWidth / window.innerHeight;\n"
         << "            camera.updateProjectionMatrix();\n"
         << "            renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "        });\n"
         << "        \n"
         << "        // Render loop\n"
         << "        function animate() {\n"
         << "            requestAnimationFrame(animate);\n"
         << "            renderer.render(scene, camera);\n"
         << "        }\n"
         << "        \n"
         << "        // Start rendering\n"
         << "        animate();\n"
         << "    </script>\n"
         << "</body>\n"
         << "</html>";
    
    file.close();
    
    ::std::cout << "Static visualization created with:" << ::std::endl;
    ::std::cout << "- " << arm_model.nr_body_spheres() << " arm spheres (green)" << ::std::endl;
    ::std::cout << "- " << occupied_count << " occupied grid cells (red)" << ::std::endl;
    ::std::cout << "- World origin frame (RGB axes)" << ::std::endl;
    ::std::cout << "- Robot base frame (RGB axes)" << ::std::endl;
    ::std::cout << "- End-effector frame (RGB axes)" << ::std::endl;
    ::std::cout << "- Saved to: " << full_path << ::std::endl;
    
    // Try to open in Chrome browser on Linux
    bool opened = false;
    
    // Method 1: Try google-chrome
    ::std::string chrome_cmd = "google-chrome \"" + full_path + "\" > /dev/null 2>&1 &";
    if (::std::system("which google-chrome > /dev/null 2>&1") == 0) {
        if (::std::system(chrome_cmd.c_str()) == 0) {
            opened = true;
            ::std::cout << "Opened in Chrome browser" << ::std::endl;
        }
    }
    
    // Method 2: Try chromium-browser if google-chrome not available
    if (!opened) {
        ::std::string chromium_cmd = "chromium-browser \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which chromium-browser > /dev/null 2>&1") == 0) {
            if (::std::system(chromium_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opened in Chromium browser" << ::std::endl;
            }
        }
    }
    
    // Method 3: Try xdg-open as fallback
    if (!opened) {
        ::std::string xdg_cmd = "xdg-open \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which xdg-open > /dev/null 2>&1") == 0) {
            if (::std::system(xdg_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opened with default browser" << ::std::endl;
            }
        }
    }
    
    if (!opened) {
        ::std::cout << "\n=== MANUAL OPENING REQUIRED ===" << ::std::endl;
        ::std::cout << "Could not open browser automatically." << ::std::endl;
        ::std::cout << "Please manually open the file:" << ::std::endl;
        ::std::cout << "File location: " << full_path << ::std::endl;
        ::std::cout << "\nAlternative methods:" << ::std::endl;
        ::std::cout << "1. Open file manager and navigate to: ./TrajectoryAnimation/" << ::std::endl;
        ::std::cout << "2. Double-click: " << filename << ::std::endl;
        ::std::cout << "3. Or from command line:" << ::std::endl;
        ::std::cout << "   google-chrome \"" << full_path << "\"" << ::std::endl;
    } else {
        ::std::cout << "Static arm visualization should now be opening in your browser!" << ::std::endl;
        ::std::cout << "Use the rotation slider at the bottom to rotate around the arm base!" << ::std::endl;
    }
}

void visualizeTaskTrajectory(
    const std::deque<Eigen::VectorXd>& trajectory,
    const GPMP2_OccupancyGrid& dataset,
    const gtsam::Pose3& base_pose) {
    
    if (trajectory.empty()) {
        ::std::cerr << "Empty task trajectory provided!" << ::std::endl;
        return;
    }
    
    ::std::cout << "Creating task trajectory visualization..." << ::std::endl;
    ::std::cout << "Trajectory points: " << trajectory.size() << ::std::endl;
    
    // Calculate timing parameters
    double total_time_sec = trajectory.size() / 1000.0;  // 1000Hz trajectory
    
    // Create sampled frame indices (every 33rd frame + last frame)
    ::std::vector<size_t> sampled_indices;
    for (size_t i = 0; i < trajectory.size(); i += 33) {
        sampled_indices.push_back(i);
    }
    // Ensure last frame is included
    if (sampled_indices.back() != trajectory.size() - 1) {
        sampled_indices.push_back(trajectory.size() - 1);
    }
    
    ::std::cout << "Sampled " << sampled_indices.size() << " frames for visualization" << ::std::endl;
    
    // Convert TaskTrajectory poses to gtsam::Pose3 objects
    ::std::vector<gtsam::Pose3> poses;
    for (size_t idx : sampled_indices) {
        const Eigen::VectorXd& pose_vec = trajectory[idx];
        if (pose_vec.size() != 6) {
            ::std::cerr << "Invalid pose dimension: " << pose_vec.size() << " (expected 6)" << ::std::endl;
            return;
        }
        
        // Extract position (x, y, z)
        gtsam::Point3 position(pose_vec[0], pose_vec[1], pose_vec[2]);
        
        // Extract orientation (roll, pitch, yaw)
        gtsam::Rot3 rotation = gtsam::Rot3::Ypr(pose_vec[5], pose_vec[4], pose_vec[3]); // yaw, pitch, roll
        
        poses.push_back(gtsam::Pose3(rotation, position));
    }
    
    // Get base pose components
    gtsam::Point3 base_position = base_pose.translation();
    gtsam::Matrix3 base_rotation = base_pose.rotation().matrix();
    
    // Create timestamped filename
    auto now = ::std::chrono::system_clock::now();
    auto time_t = ::std::chrono::system_clock::to_time_t(now);
    auto ms = ::std::chrono::duration_cast<::std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    ::std::stringstream ss;
    ss << ::std::put_time(::std::localtime(&time_t), "%Y%m%d_%H%M%S");
    ss << "_" << ::std::setfill('0') << ::std::setw(3) << ms.count();
    
    ::std::string timestamp = ss.str();
    ::std::string filename = "task_trajectory_" + timestamp + ".html";
    
    // Save to local Videos directory
    ::std::string videos_dir = "./TrajectoryAnimation";
    ::std::string full_path = videos_dir + "/" + filename;
    
    // Create directory if it doesn't exist
    ::std::string mkdir_cmd = "mkdir -p " + videos_dir;
    ::std::system(mkdir_cmd.c_str());
    
    ::std::ofstream file(full_path);
    
    if (!file.is_open()) {
        ::std::cerr << "Could not create file: " << full_path << ::std::endl;
        ::std::cerr << "Make sure the directory exists and is accessible" << ::std::endl;
        return;
    }
    
    // Write HTML header
    file << "<!DOCTYPE html>\n"
         << "<html>\n"
         << "<head>\n"
         << "    <title>Task Trajectory Animation</title>\n"
         << "    <style>\n"
         << "        body { \n"
         << "            margin: 0; \n"
         << "            padding: 0; \n"
         << "            background: #111; \n"
         << "            overflow: hidden; \n"
         << "            font-family: Arial, sans-serif;\n"
         << "        }\n"
         << "        #controls { \n"
         << "            position: absolute; \n"
         << "            top: 10px; \n"
         << "            left: 10px; \n"
         << "            color: white; \n"
         << "            z-index: 100;\n"
         << "            background: rgba(0,0,0,0.7);\n"
         << "            padding: 15px;\n"
         << "            border-radius: 5px;\n"
         << "        }\n"
         << "        #controls button {\n"
         << "            margin: 5px;\n"
         << "            padding: 8px 12px;\n"
         << "            background: #333;\n"
         << "            color: white;\n"
         << "            border: none;\n"
         << "            border-radius: 3px;\n"
         << "            cursor: pointer;\n"
         << "        }\n"
         << "        #controls button:hover { background: #555; }\n"
         << "        #controls button:active { background: #777; }\n"
         << "        #speed-control {\n"
         << "            margin: 10px 0;\n"
         << "        }\n"
         << "        #speed-slider {\n"
         << "            width: 150px;\n"
         << "            margin: 0 10px;\n"
         << "        }\n"
         << "        #scrub-control {\n"
         << "            margin: 10px 0;\n"
         << "        }\n"
         << "        #scrub-slider {\n"
         << "            width: 200px;\n"
         << "            margin: 0 10px;\n"
         << "        }\n"
         << "        #info {\n"
         << "            margin-top: 10px;\n"
         << "            font-size: 12px;\n"
         << "            color: #ccc;\n"
         << "        }\n"
         << "    </style>\n"
         << "</head>\n"
         << "<body>\n"
         << "    <div id=\"controls\">\n"
         << "        <h3>Task Trajectory Animation</h3>\n"
         << "        <button id=\"playBtn\">Play</button>\n"
         << "        <button id=\"pauseBtn\">Pause</button>\n"
         << "        <button id=\"resetBtn\">Reset</button>\n"
         << "        <div id=\"speed-control\">\n"
         << "            Speed: <input type=\"range\" id=\"speed-slider\" min=\"0.1\" max=\"1.0\" step=\"0.1\" value=\"1.0\">\n"
         << "            <span id=\"speed-value\">1.0x (" << total_time_sec << "s)</span>\n"
         << "        </div>\n"
         << "        <div id=\"scrub-control\">\n"
         << "            Time: <input type=\"range\" id=\"scrub-slider\" min=\"0\" max=\"100\" value=\"0\">\n"
         << "            <span id=\"time-value\">0%</span>\n"
         << "        </div>\n"
         << "        <div id=\"info\">\n"
         << "            <p>Frame: <span id=\"frame-counter\">0</span> / <span id=\"total-frames\">" << sampled_indices.size() << "</span></p>\n"
         << "            <p>Total Points: " << trajectory.size() << " | Duration: " << total_time_sec << "s</p>\n"
         << "            <p>Mouse: Left=Rotate | Right=Pan | Wheel=Zoom</p>\n"
         << "            <p>Cyan: EE Path | Red: Obstacles | RGB: Coordinate Frames</p>\n"
         << "        </div>\n"
         << "    </div>\n"
         << "    \n"
         << "    <script src=\"https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js\"></script>\n"
         << "    <script>\n"
         << "        // Scene setup\n"
         << "        let scene, camera, renderer;\n"
         << "        let trajectoryTrace;\n"
         << "        let currentFrame = 0;\n"
         << "        let isPlaying = false;\n"
         << "        let animationSpeed = 1.0;\n"
         << "        let lastTime = 0;\n"
         << "        const totalTimeSeconds = " << total_time_sec << ";\n"
         << "        \n"
         << "        // Camera controls\n"
         << "        let cameraTarget = new THREE.Vector3(" << base_position.x() << ", " << base_position.y() << ", " << base_position.z() << ");\n"
         << "        let cameraDistance = 3.0;\n"
         << "        let cameraAzimuth = 0;\n"
         << "        let cameraElevation = 0.3;\n"
         << "        \n"
         << "        // Mouse controls\n"
         << "        let mouseDown = false;\n"
         << "        let mouseButton = 0;\n"
         << "        let mouseX = 0, mouseY = 0;\n"
         << "        \n"
         << "        // Task trajectory data (positions and orientations)\n"
         << "        const trajectoryPoints = [\n";

    // Output trajectory points for JavaScript
    for (size_t i = 0; i < poses.size(); ++i) {
        gtsam::Point3 pos = poses[i].translation();
        gtsam::Matrix3 rot = poses[i].rotation().matrix();
        file << "            {\n"
             << "                x: " << pos.x() << ", y: " << pos.y() << ", z: " << pos.z() << ",\n"
             << "                rotMatrix: [\n"
             << "                    " << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << ",\n"
             << "                    " << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << ",\n"
             << "                    " << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << "\n"
             << "                ]\n"
             << "            }";
        if (i < poses.size() - 1) file << ",";
        file << "\n";
    }

    file << "        ];\n"
         << "        \n"
         << "        function init() {\n"
         << "            // Scene\n"
         << "            scene = new THREE.Scene();\n"
         << "            scene.background = new THREE.Color(0x222222);\n"
         << "            \n"
         << "            // Camera\n"
         << "            camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);\n"
         << "            \n"
         << "            // Renderer\n"
         << "            renderer = new THREE.WebGLRenderer({ antialias: true });\n"
         << "            renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "            renderer.shadowMap.enabled = true;\n"
         << "            renderer.shadowMap.type = THREE.PCFSoftShadowMap;\n"
         << "            document.body.appendChild(renderer.domElement);\n"
         << "            \n"
         << "            // Lighting\n"
         << "            const ambientLight = new THREE.AmbientLight(0x404040, 0.6);\n"
         << "            scene.add(ambientLight);\n"
         << "            \n"
         << "            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);\n"
         << "            directionalLight.position.set(5, 5, 5);\n"
         << "            directionalLight.castShadow = true;\n"
         << "            directionalLight.shadow.mapSize.width = 2048;\n"
         << "            directionalLight.shadow.mapSize.height = 2048;\n"
         << "            scene.add(directionalLight);\n"
         << "            \n"
         << "            // Initialize trajectory trace\n"
         << "            const traceGeometry = new THREE.BufferGeometry();\n"
         << "            const traceMaterial = new THREE.LineBasicMaterial({color: 0x00ffff, linewidth: 3});\n"
         << "            trajectoryTrace = new THREE.Line(traceGeometry, traceMaterial);\n"
         << "            scene.add(trajectoryTrace);\n"
         << "            \n"
         << "            // Initialize current end-effector frame (will be updated in animation)\n"
         << "            window.currentEEFrame = null;\n"
         << "        }\n"
         << "        \n"
         << "        function createCoordinateFrame(position, orientation, scale = 0.2) {\n"
         << "            const group = new THREE.Group();\n"
         << "            \n"
         << "            // Arrow geometry and materials\n"
         << "            const arrowLength = scale;\n"
         << "            const arrowHeadLength = scale * 0.3;\n"
         << "            const arrowHeadWidth = scale * 0.1;\n"
         << "            \n"
         << "            // X axis (red)\n"
         << "            const xArrow = new THREE.ArrowHelper(\n"
         << "                new THREE.Vector3(1, 0, 0),\n"
         << "                new THREE.Vector3(0, 0, 0),\n"
         << "                arrowLength, 0xff0000, arrowHeadLength, arrowHeadWidth\n"
         << "            );\n"
         << "            group.add(xArrow);\n"
         << "            \n"
         << "            // Y axis (green)\n"
         << "            const yArrow = new THREE.ArrowHelper(\n"
         << "                new THREE.Vector3(0, 1, 0),\n"
         << "                new THREE.Vector3(0, 0, 0),\n"
         << "                arrowLength, 0x00ff00, arrowHeadLength, arrowHeadWidth\n"
         << "            );\n"
         << "            group.add(yArrow);\n"
         << "            \n"
         << "            // Z axis (blue)\n"
         << "            const zArrow = new THREE.ArrowHelper(\n"
         << "                new THREE.Vector3(0, 0, 1),\n"
         << "                new THREE.Vector3(0, 0, 0),\n"
         << "                arrowLength, 0x0000ff, arrowHeadLength, arrowHeadWidth\n"
         << "            );\n"
         << "            group.add(zArrow);\n"
         << "            \n"
         << "            // Set position\n"
         << "            group.position.copy(position);\n"
         << "            \n"
         << "            // Apply orientation if provided (rotation only)\n"
         << "            if (orientation) {\n"
         << "                // Extract rotation from matrix and apply to group\n"
         << "                const rotationMatrix = new THREE.Matrix3().setFromMatrix4(orientation);\n"
         << "                group.setRotationFromMatrix(orientation);\n"
         << "            }\n"
         << "            \n"
         << "            return group;\n"
         << "        }\n"
         << "        \n"
         << "        function updateCameraPosition() {\n"
         << "            const x = cameraDistance * Math.cos(cameraElevation) * Math.cos(cameraAzimuth);\n"
         << "            const y = cameraDistance * Math.cos(cameraElevation) * Math.sin(cameraAzimuth);\n"
         << "            const z = cameraDistance * Math.sin(cameraElevation);\n"
         << "            \n"
         << "            camera.position.set(\n"
         << "                cameraTarget.x + x,\n"
         << "                cameraTarget.y + y,\n"
         << "                cameraTarget.z + z\n"
         << "            );\n"
         << "            camera.lookAt(cameraTarget);\n"
         << "        }\n"
         << "        \n"
         << "        function updateVisualization() {\n"
         << "            if (trajectoryPoints.length === 0) return;\n"
         << "            \n"
         << "            // Update trajectory trace up to current frame\n"
         << "            const traceLength = Math.min(currentFrame + 1, trajectoryPoints.length);\n"
         << "            const positions = new Float32Array(traceLength * 3);\n"
         << "            \n"
         << "            for (let i = 0; i < traceLength; i++) {\n"
         << "                const point = trajectoryPoints[i];\n"
         << "                positions[i * 3] = point.x;\n"
         << "                positions[i * 3 + 1] = point.y;\n"
         << "                positions[i * 3 + 2] = point.z;\n"
         << "            }\n"
         << "            \n"
         << "            trajectoryTrace.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));\n"
         << "            \n"
         << "            // Update current end-effector coordinate frame\n"
         << "            if (window.currentEEFrame) {\n"
         << "                scene.remove(window.currentEEFrame);\n"
         << "            }\n"
         << "            \n"
         << "            if (currentFrame < trajectoryPoints.length) {\n"
         << "                const currentPoint = trajectoryPoints[currentFrame];\n"
         << "                const rotMatrix = new THREE.Matrix4().set(\n"
         << "                    currentPoint.rotMatrix[0], currentPoint.rotMatrix[1], currentPoint.rotMatrix[2], 0,\n"
         << "                    currentPoint.rotMatrix[3], currentPoint.rotMatrix[4], currentPoint.rotMatrix[5], 0,\n"
         << "                    currentPoint.rotMatrix[6], currentPoint.rotMatrix[7], currentPoint.rotMatrix[8], 0,\n"
         << "                    0, 0, 0, 1\n"
         << "                );\n"
         << "                \n"
         << "                window.currentEEFrame = createCoordinateFrame(\n"
         << "                    new THREE.Vector3(currentPoint.x, currentPoint.y, currentPoint.z),\n"
         << "                    rotMatrix,\n"
         << "                    0.15\n"
         << "                );\n"
         << "                scene.add(window.currentEEFrame);\n"
         << "            }\n"
         << "            \n"
         << "            // Update UI\n"
         << "            const frameCounter = document.getElementById('frame-counter');\n"
         << "            const scrubSlider = document.getElementById('scrub-slider');\n"
         << "            const timeValue = document.getElementById('time-value');\n"
         << "            \n"
         << "            if (frameCounter) frameCounter.textContent = currentFrame;\n"
         << "            if (scrubSlider && !isPlaying) { // Don't update slider during playback to avoid conflicts\n"
         << "                const percent = (currentFrame / (trajectoryPoints.length - 1)) * 100;\n"
         << "                scrubSlider.value = percent;\n"
         << "                if (timeValue) timeValue.textContent = percent.toFixed(1) + '%';\n"
         << "            }\n"
         << "        }\n"
         << "        \n"
         << "        function addObstacles() {\n"
         << "            // Create shared geometry and material for occupied cells\n"
         << "            const cellSize = " << dataset.cell_size << ";\n"
         << "            const occupiedGeometry = new THREE.BoxGeometry();\n"
         << "            const occupiedMaterial = new THREE.MeshLambertMaterial({color: 0xff6b6b, transparent: true, opacity: 0.3});\n"
         << "            const occupiedGroup = new THREE.Group();\n"
         << "        \n";

    // Add occupancy grid - count occupied cells
    size_t occupied_count = 0;
    for (size_t i = 0; i < dataset.rows; ++i) {
        for (size_t j = 0; j < dataset.cols; ++j) {
            for (size_t k = 0; k < dataset.z; ++k) {
                if (dataset.map[i][j][k] > 0.5f) {
                    double world_x = dataset.origin_x + (i + 0.5) * dataset.cell_size;
                    double world_y = dataset.origin_y + (j + 0.5) * dataset.cell_size;
                    double world_z = dataset.origin_z + (k + 0.5) * dataset.cell_size;
                    
                    file << "        {\n"
                         << "            const cube = new THREE.Mesh(occupiedGeometry, occupiedMaterial);\n"
                         << "            cube.position.set(" << world_x << ", " << world_y << ", " << world_z << ");\n"
                         << "            cube.scale.set(cellSize, cellSize, cellSize);\n"
                         << "            cube.castShadow = true;\n"
                         << "            occupiedGroup.add(cube);\n"
                         << "        }\n";
                    occupied_count++;
                }
            }
        }
    }

    // Add the occupied group to scene
    file << "        scene.add(occupiedGroup);\n";

    file << "        }\n"
         << "        \n"
         << "        init();\n"
         << "        addObstacles();\n"
         << "        \n"
         << "        // Add coordinate frames\n"
         << "        \n"
         << "        // World origin frame\n"
         << "        const worldFrame = createCoordinateFrame(new THREE.Vector3(0, 0, 0), null, 0.3);\n"
         << "        scene.add(worldFrame);\n"
         << "        \n"
         << "        // Robot base frame\n"
         << "        const baseRotationMatrix = new THREE.Matrix4().set(\n"
         << "            " << base_rotation(0,0) << ", " << base_rotation(0,1) << ", " << base_rotation(0,2) << ", 0,\n"
         << "            " << base_rotation(1,0) << ", " << base_rotation(1,1) << ", " << base_rotation(1,2) << ", 0,\n"
         << "            " << base_rotation(2,0) << ", " << base_rotation(2,1) << ", " << base_rotation(2,2) << ", 0,\n"
         << "            0, 0, 0, 1\n"
         << "        );\n"
         << "        const baseFrame = createCoordinateFrame(\n"
         << "            new THREE.Vector3(" << base_position.x() << ", " << base_position.y() << ", " << base_position.z() << "),\n"
         << "            baseRotationMatrix,\n"
         << "            0.25\n"
         << "        );\n"
         << "        scene.add(baseFrame);\n"
         << "        \n"
         << "        // Initialize camera position\n"
         << "        updateCameraPosition();\n"
         << "        \n"
         << "        // Animation controls\n"
         << "        const playBtn = document.getElementById('playBtn');\n"
         << "        const pauseBtn = document.getElementById('pauseBtn');\n"
         << "        const resetBtn = document.getElementById('resetBtn');\n"
         << "        const speedSlider = document.getElementById('speed-slider');\n"
         << "        const speedValue = document.getElementById('speed-value');\n"
         << "        const scrubSlider = document.getElementById('scrub-slider');\n"
         << "        const timeValue = document.getElementById('time-value');\n"
         << "        \n"
         << "        playBtn.addEventListener('click', () => {\n"
         << "            isPlaying = true;\n"
         << "            lastTime = performance.now();\n"
         << "        });\n"
         << "        \n"
         << "        pauseBtn.addEventListener('click', () => {\n"
         << "            isPlaying = false;\n"
         << "        });\n"
         << "        \n"
         << "        resetBtn.addEventListener('click', () => {\n"
         << "            currentFrame = 0;\n"
         << "            isPlaying = false;\n"
         << "            updateVisualization();\n"
         << "        });\n"
         << "        \n"
         << "        speedSlider.addEventListener('input', (e) => {\n"
         << "            animationSpeed = parseFloat(e.target.value);\n"
         << "            const playbackTime = (totalTimeSeconds / animationSpeed).toFixed(1);\n"
         << "            speedValue.textContent = animationSpeed.toFixed(1) + 'x (' + playbackTime + 's)';\n"
         << "        });\n"
         << "        \n"
         << "        scrubSlider.addEventListener('input', (e) => {\n"
         << "            const percent = parseFloat(e.target.value);\n"
         << "            currentFrame = Math.floor((percent / 100) * (trajectoryPoints.length - 1));\n"
         << "            timeValue.textContent = percent.toFixed(1) + '%';\n"
         << "            updateVisualization();\n"
         << "        });\n"
         << "        \n"
         << "        // Mouse controls for camera (exclude UI elements)\n"
         << "        document.addEventListener('mousedown', (event) => {\n"
         << "            // Don't interfere with UI controls\n"
         << "            if (event.target.closest('#controls')) {\n"
         << "                return;\n"
         << "            }\n"
         << "            \n"
         << "            mouseDown = true;\n"
         << "            mouseButton = event.button;\n"
         << "            mouseX = event.clientX;\n"
         << "            mouseY = event.clientY;\n"
         << "            event.preventDefault();\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mouseup', () => {\n"
         << "            mouseDown = false;\n"
         << "        });\n"
         << "        \n"
         << "        document.addEventListener('mousemove', (event) => {\n"
         << "            if (!mouseDown) return;\n"
         << "            \n"
         << "            const deltaX = event.clientX - mouseX;\n"
         << "            const deltaY = event.clientY - mouseY;\n"
         << "            \n"
         << "            if (mouseButton === 0) { // Left button - rotate\n"
         << "                cameraAzimuth -= deltaX * 0.01;\n"
         << "                cameraElevation = Math.max(-Math.PI/2 + 0.1, Math.min(Math.PI/2 - 0.1, cameraElevation + deltaY * 0.01));\n"
         << "                updateCameraPosition();\n"
         << "            } else if (mouseButton === 2) { // Right button - pan\n"
         << "                const right = new THREE.Vector3();\n"
         << "                const up = new THREE.Vector3();\n"
         << "                camera.getWorldDirection(new THREE.Vector3());\n"
         << "                right.crossVectors(camera.getWorldDirection(new THREE.Vector3()), camera.up).normalize();\n"
         << "                up.crossVectors(right, camera.getWorldDirection(new THREE.Vector3())).normalize();\n"
         << "                \n"
         << "                const panSpeed = 0.003;\n"
         << "                cameraTarget.add(right.multiplyScalar(-deltaX * panSpeed));\n"
         << "                cameraTarget.add(up.multiplyScalar(deltaY * panSpeed));\n"
         << "                updateCameraPosition();\n"
         << "            }\n"
         << "            \n"
         << "            mouseX = event.clientX;\n"
         << "            mouseY = event.clientY;\n"
         << "        });\n"
         << "        \n"
         << "        // Mouse wheel - zoom\n"
         << "        document.addEventListener('wheel', (event) => {\n"
         << "            cameraDistance = Math.max(0.5, Math.min(10.0, cameraDistance + event.deltaY * 0.01));\n"
         << "            updateCameraPosition();\n"
         << "            event.preventDefault();\n"
         << "        });\n"
         << "        \n"
         << "        // Disable context menu\n"
         << "        document.addEventListener('contextmenu', (event) => {\n"
         << "            event.preventDefault();\n"
         << "        });\n"
         << "        \n"
         << "        // Animation loop\n"
         << "        function animate(currentTime) {\n"
         << "            requestAnimationFrame(animate);\n"
         << "            \n"
         << "            // Update animation frame\n"
         << "            if (isPlaying && trajectoryPoints.length > 0) {\n"
         << "                const frameTime = (1000 * totalTimeSeconds) / (trajectoryPoints.length * animationSpeed);\n"
         << "                const deltaTime = currentTime - lastTime;\n"
         << "                \n"
         << "                if (deltaTime >= frameTime) {\n"
         << "                    currentFrame++;\n"
         << "                    if (currentFrame >= trajectoryPoints.length) {\n"
         << "                        currentFrame = 0; // Loop animation\n"
         << "                    }\n"
         << "                    updateVisualization();\n"
         << "                    lastTime = currentTime;\n"
         << "                }\n"
         << "            }\n"
         << "            \n"
         << "            renderer.render(scene, camera);\n"
         << "        }\n"
         << "        \n"
         << "        // Handle window resize\n"
         << "        window.addEventListener('resize', function() {\n"
         << "            camera.aspect = window.innerWidth / window.innerHeight;\n"
         << "            camera.updateProjectionMatrix();\n"
         << "            renderer.setSize(window.innerWidth, window.innerHeight);\n"
         << "        });\n"
         << "        \n"
         << "        // Initialize\n"
         << "        updateVisualization();\n"
         << "        animate(0);\n"
         << "    </script>\n"
         << "</body>\n"
         << "</html>\n";
    
    file.close();
    
    ::std::cout << "Task trajectory visualization saved to: " << full_path << ::std::endl;
    
    // Try to open in Chrome browser on Linux
    bool opened = false;
    
    // Method 1: Try google-chrome
    ::std::string chrome_cmd = "google-chrome \"" + full_path + "\" > /dev/null 2>&1 &";
    if (::std::system("which google-chrome > /dev/null 2>&1") == 0) {
        if (::std::system(chrome_cmd.c_str()) == 0) {
            opened = true;
            ::std::cout << "Opening task trajectory visualization in Chrome browser..." << ::std::endl;
        }
    }
    
    // Method 2: Try chromium-browser if google-chrome not available
    if (!opened) {
        ::std::string chromium_cmd = "chromium-browser \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which chromium-browser > /dev/null 2>&1") == 0) {
            if (::std::system(chromium_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opening task trajectory visualization in Chromium browser..." << ::std::endl;
            }
        }
    }
    
    // Method 3: Try xdg-open as fallback
    if (!opened) {
        ::std::string xdg_cmd = "xdg-open \"" + full_path + "\" > /dev/null 2>&1 &";
        if (::std::system("which xdg-open > /dev/null 2>&1") == 0) {
            if (::std::system(xdg_cmd.c_str()) == 0) {
                opened = true;
                ::std::cout << "Opening task trajectory visualization with default browser..." << ::std::endl;
            }
        }
    }
    
    if (!opened) {
        ::std::cout << "\n=== MANUAL OPENING REQUIRED ===" << ::std::endl;
        ::std::cout << "Could not open browser automatically." << ::std::endl;
        ::std::cout << "Please manually open the file:" << ::std::endl;
        ::std::cout << "File location: " << full_path << ::std::endl;
        ::std::cout << "\nAlternative methods:" << ::std::endl;
        ::std::cout << "1. Open file manager and navigate to: ./TrajectoryAnimation/" << ::std::endl;
        ::std::cout << "2. Double-click: " << filename << ::std::endl;
        ::std::cout << "3. Or from command line:" << ::std::endl;
        ::std::cout << "   google-chrome \"" << full_path << "\"" << ::std::endl;
    } else {
        ::std::cout << "Task trajectory visualization should now be opening in your browser!" << ::std::endl;
        ::std::cout << "Use the rotation slider at the bottom to rotate around the robot base!" << ::std::endl;
    }
}


void saveRecordToYAML(const TrajectoryRecord& record, const std::string& name) {
    std::string filename = "../logs/" + name + ".yaml";
    
    YAML::Node root;
    
    // Save target trajectory
    YAML::Node target_traj;
    for (size_t i = 0; i < record.target_trajectory.size(); ++i) {
        YAML::Node point;
        const Eigen::VectorXd& vec = record.target_trajectory[i];
        for (int j = 0; j < vec.size(); ++j) {
            point.push_back(vec[j]);
        }
        target_traj.push_back(point);
    }
    root["target_trajectory"] = target_traj;
    
    // Save actual trajectory
    YAML::Node actual_traj;
    for (size_t i = 0; i < record.actual_trajectory.size(); ++i) {
        YAML::Node point;
        const Eigen::VectorXd& vec = record.actual_trajectory[i];
        for (int j = 0; j < vec.size(); ++j) {
            point.push_back(vec[j]);
        }
        actual_traj.push_back(point);
    }
    root["actual_trajectory"] = actual_traj;
    
    // Write to file
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Error: Could not create file " << filename << std::endl;
        return;
    }
    
    fout << root;
    fout.close();
    
    std::cout << "TrajectoryRecord saved to: " << filename << std::endl;
}