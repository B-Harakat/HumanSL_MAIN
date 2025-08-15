#include "Tube.h"

TubeInfo extractTubeInfoFromC3D(const std::string& c3dFilePath, const int& frame_idx) {

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
