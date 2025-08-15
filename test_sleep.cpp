// test_open3d.cpp
#include <open3d/Open3D.h>
#include <iostream>

int main() {
    // Try different possible version function names
    #ifdef OPEN3D_VERSION
        std::cout << "Open3D version: " << OPEN3D_VERSION << std::endl;
    #else
        std::cout << "Open3D loaded successfully!" << std::endl;
    #endif
    
    // Create a simple point cloud to test core functionality
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.push_back({0, 0, 0});
    pcd->points_.push_back({1, 0, 0});
    pcd->points_.push_back({0, 1, 0});
    
    std::cout << "Point cloud created with " << pcd->points_.size() << " points" << std::endl;
    
    // Test basic geometry operations
    pcd->PaintUniformColor({1.0, 0.0, 0.0}); // Paint red
    std::cout << "Point cloud painted red" << std::endl;
    
    // Test bounding box
    auto bbox = pcd->GetAxisAlignedBoundingBox();
    auto center = bbox.GetCenter();
    std::cout << "Bounding box center: (" << center[0] << ", " << center[1] << ", " << center[2] << ")" << std::endl;
    
    std::cout << "Open3D test completed successfully!" << std::endl;
    return 0;
}