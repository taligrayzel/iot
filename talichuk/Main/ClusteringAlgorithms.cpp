// #include "ClusteringAlgorithms.h"
// #include "KDTree.h"
// #include <cmath>

// // Derivative-based clustering: group points based on the change in direction between consecutive points
// DerivativeClustering::DerivativeClustering(double thresh) : threshold(thresh) {}

// std::vector<std::vector<Point>> DerivativeClustering::clusterPoints(const std::vector<Point>& points) {
//     std::vector<std::vector<Point>> clusters;
//     if (points.size() < 2) return clusters;

//     std::vector<Point> current_cluster;
//     current_cluster.push_back(points[0]);

//     for (size_t i = 1; i < points.size(); ++i) {
//         // Compute the angle between adjacent points (simple 2D difference)
//         double dx1 = points[i].x - points[i-1].x;
//         double dy1 = points[i].y - points[i-1].y;
//         double angle1 = std::atan2(dy1, dx1);

//         if (i > 1) {
//             // Compute the angle between current point and the previous point's direction
//             double dx2 = points[i-1].x - points[i-2].x;
//             double dy2 = points[i-1].y - points[i-2].y;
//             double angle2 = std::atan2(dy2, dx2);

//             double angle_diff = std::fabs(angle1 - angle2);
//             // Normalize angle difference to the range [0, Pi]
//             if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;

//             // If angle difference exceeds threshold, start a new cluster
//             if (angle_diff > threshold) {
//                 clusters.push_back(current_cluster);  // Add the current cluster to the result
//                 current_cluster.clear();             // Clear current cluster for new points
//             }
//         }

//         // Add current point to the current cluster
//         current_cluster.push_back(points[i]);
//     }

//     // Add the last cluster
//     if (!current_cluster.empty()) {
//         clusters.push_back(current_cluster);
//     }

//     return clusters;
// }

// DBSCANClustering::DBSCANClustering(double epsilon, int min_points) : eps(epsilon), minPts(min_points) {}

// std::vector<std::vector<Point>> DBSCANClustering::clusterPoints(const std::vector<Point>& points) {
//     // Create a KD-Tree from the input points
//     KDTree kdTree(points);

//     std::vector<int> cluster_indices(points.size(), -1);  // -1 means unvisited
//     std::vector<bool> visited(points.size(), false);

//     int cluster_id = 0;

//     for (size_t i = 0; i < points.size(); ++i) {
//         if (visited[i]) continue;

//         visited[i] = true;

//         // Find neighbors within epsilon distance using the KD-Tree
//         std::vector<Point> neighbors = kdTree.search(points[i], eps);

//         if (neighbors.size() < minPts) {
//             cluster_indices[i] = -1;  // Mark as noise
//         } else {
//             cluster_indices[i] = cluster_id++;  // Start a new cluster
//         }
//     }

//     // Group the points based on their cluster indices
//     std::vector<std::vector<Point>> clusters(cluster_id);

//     for (size_t i = 0; i < points.size(); ++i) {
//         if (cluster_indices[i] != -1) {
//             clusters[cluster_indices[i]].push_back(points[i]);
//         }
//     }

//     return clusters;
// }
