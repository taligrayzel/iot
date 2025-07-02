// #ifndef CLUSTERINGALGORITHM_H
// #define CLUSTERINGALGORITHM_H

// #include <vector>

// // Base class for clustering algorithms
// class ClusteringAlgorithm {
// public:
//     virtual ~ClusteringAlgorithm() = default;
//     virtual std::vector<std::vector<Point>> clusterPoints(const std::vector<Point>& points) = 0;
// };

// class DerivativeClustering : public ClusteringAlgorithm {
// private:
//     double threshold;  // Threshold for the change in direction

// public:
//     explicit DerivativeClustering(double thresh);
//     std::vector<std::vector<Point>> clusterPoints(const std::vector<Point>& points) override;
// };

// // Forward declaration of KDTree class to avoid circular dependency
// class KDTree;

// class DBSCANClustering : public ClusteringAlgorithm {
// private:
//     double eps;  // Radius for DBSCAN neighborhood
//     int minPts;  // Minimum points to form a cluster

// public:
//     DBSCANClustering(double epsilon, int min_points);
//     std::vector<std::vector<Point>> clusterPoints(const std::vector<Point>& points) override;
// };

// #endif  // CLUSTERINGALGORITHM_H
