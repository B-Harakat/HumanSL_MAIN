#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

/**
 * Calculate frame pose from 4 reference points
 * 
 * Frame coordinates (given):
 * P1 = {88.2, 0, 0}      - Point on +X axis of circle
 * P2 = {0, -88.2, 0}     - Point on -Y axis of circle  
 * P3 = {-88.2, 0, 0}     - Point on -X axis of circle
 * P4 = {0, -15.9, 44.4}  - Point offset in +Z direction
 * 
 * @param world_p1 World coordinates of P1
 * @param world_p2 World coordinates of P2  
 * @param world_p3 World coordinates of P3
 * @param world_p4 World coordinates of P4
 * @return Pose3 representing the frame transformation
 */
gtsam::Pose3 calculateFramePose(const Eigen::Vector3d& world_p1, 
                               const Eigen::Vector3d& world_p2,
                               const Eigen::Vector3d& world_p3, 
                               const Eigen::Vector3d& world_p4) {
    
    // Step 1: Find circle center (circumcenter of triangle P1,P2,P3)
    // For 3D circumcenter calculation, work in the plane containing the three points
    Eigen::Vector3d v1 = world_p2 - world_p1;
    Eigen::Vector3d v2 = world_p3 - world_p1;
    Eigen::Vector3d normal = v1.cross(v2);
    
    // Create orthonormal basis for the plane
    Eigen::Vector3d u1 = v1.normalized();
    Eigen::Vector3d u2 = (v2 - v2.dot(u1) * u1).normalized();
    
    // Project points onto 2D plane coordinates
    Eigen::Vector2d a(0, 0);  // p1 at origin
    Eigen::Vector2d b(v1.norm(), 0);  // p2 on u1 axis
    Eigen::Vector2d c(v2.dot(u1), v2.dot(u2));  // p3 in plane
    
    // Calculate 2D circumcenter
    double d = 2 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));
    
    Eigen::Vector3d world_center;
    if (std::abs(d) < 1e-10) {
        // Points are collinear, return centroid
        world_center = (world_p1 + world_p2 + world_p3) / 3.0;
    } else {
        double ux = ((a.x()*a.x() + a.y()*a.y()) * (b.y() - c.y()) + 
                     (b.x()*b.x() + b.y()*b.y()) * (c.y() - a.y()) + 
                     (c.x()*c.x() + c.y()*c.y()) * (a.y() - b.y())) / d;
        
        double uy = ((a.x()*a.x() + a.y()*a.y()) * (c.x() - b.x()) + 
                     (b.x()*b.x() + b.y()*b.y()) * (a.x() - c.x()) + 
                     (c.x()*c.x() + c.y()*c.y()) * (b.x() - a.x())) / d;
        
        // Convert back to 3D
        world_center = world_p1 + ux * u1 + uy * u2;
    }
    
    // Step 2: Calculate plane normal (two possible directions)
    Eigen::Vector3d plane_normal = v1.cross(v2).normalized();
    
    // Step 3: Use P4 to determine correct z-axis direction
    Eigen::Vector3d center_to_p4 = world_p4 - world_center;
    double dot_product = center_to_p4.dot(plane_normal);
    
    // If P4 is on the negative side of the plane, flip the normal
    Eigen::Vector3d world_z_axis = (dot_product < 0) ? plane_normal : -plane_normal;  

    // Step 4: Calculate world x-axis (from center to P1)
    Eigen::Vector3d world_x_axis = (world_p1 - world_center).normalized();
    
    // Step 5: Calculate world y-axis (z cross x for right-handed system)
    Eigen::Vector3d world_y_axis = world_z_axis.cross(world_x_axis).normalized();
    
    // Step 6: Build rotation matrix for GTSAM
    // GTSAM expects rotation matrix as 3x3 matrix
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = world_x_axis;
    rotation_matrix.col(1) = world_y_axis;
    rotation_matrix.col(2) = world_z_axis;
    
    // Create GTSAM objects
    gtsam::Rot3 rotation(rotation_matrix);
    Eigen::Vector3d translation_with_grasp_site_offset = world_center + 0.14 * world_z_axis;
    gtsam::Point3 translation(translation_with_grasp_site_offset);
    
    return gtsam::Pose3(rotation, translation);
}

// Example usage and verification
int main() {
    // Frame coordinates (given)
    Eigen::Vector3d frame_p1(0.682, 0, 0);
    Eigen::Vector3d frame_p2(0, -0.682, 0);
    Eigen::Vector3d frame_p3(-0.682, 0, 0);
    Eigen::Vector3d frame_p4(0, -0.159, -0.444);
    
    // Example world coordinates (replace with your motion capture data)
    Eigen::Vector3d world_p1(0.682, 0, 0);
    Eigen::Vector3d world_p2(0, 0.682, 0);  
    Eigen::Vector3d world_p3(-0.682, 0, 0);
    Eigen::Vector3d world_p4(0, -0.159, -0.444);
    
    // Calculate the pose
    gtsam::Pose3 frame_pose = calculateFramePose(world_p1, world_p2, world_p3, world_p4);
    
    // Print results
    std::cout << "Frame Pose:\n";
    frame_pose.print("Frame");
    
    // Verification: Transform frame points to world coordinates
    std::cout << "\nVerification:\n";
    gtsam::Point3 check_p1 = frame_pose.transformFrom(gtsam::Point3(frame_p1));
    gtsam::Point3 check_p2 = frame_pose.transformFrom(gtsam::Point3(frame_p2));
    gtsam::Point3 check_p3 = frame_pose.transformFrom(gtsam::Point3(frame_p3));
    gtsam::Point3 check_p4 = frame_pose.transformFrom(gtsam::Point3(frame_p4));
    
    std::cout << "P1 should be: [" << world_p1.transpose() << "]\n";
    std::cout << "P1 calculated: " << check_p1 << "\n";
    
    std::cout << "P2 should be: [" << world_p2.transpose() << "]\n";
    std::cout << "P2 calculated: " << check_p2 << "\n";
    
    std::cout << "P3 should be: [" << world_p3.transpose() << "]\n";
    std::cout << "P3 calculated: " << check_p3 << "\n";

    
    std::cout << "P4 should be: [" << world_p4.transpose() << "]\n";
    std::cout << "P4 calculated: " << check_p4 << "\n";

    
    return 0;
}