// #include "KDTree.h"

// // === KDTree Methods ===

// // Insert a point into the KD-tree recursively
// KDTree::KDNode* KDTree::insertRec(KDNode* node, Point p, int depth) {
//     if (!node) return new KDNode(p);

//     int axis = depth % 2;
//     if ((axis == 0 && p.x < node->point.x) || (axis == 1 && p.y < node->point.y))
//         node->left = insertRec(node->left, p, depth + 1);
//     else
//         node->right = insertRec(node->right, p, depth + 1);

//     return node;
// }

// // Search for points within the radius using recursion
// void KDTree::radiusSearchRec(KDNode* node, Point target, double radius, int depth, std::vector<Point>& result) {
//     if (!node) return;

//     double dist = std::hypot(target.x - node->point.x, target.y - node->point.y);
//     if (dist <= radius)
//         result.push_back(node->point);

//     int axis = depth % 2;
//     double delta = (axis == 0) ? target.x - node->point.x : target.y - node->point.y;

//     if (delta < 0 || std::fabs(delta) < radius) {
//         radiusSearchRec(node->left, target, radius, depth + 1, result);
//     }
//     if (delta > 0 || std::fabs(delta) < radius) {
//         radiusSearchRec(node->right, target, radius, depth + 1, result);
//     }
// }

// // Insert a point into the KD-tree
// void KDTree::insert(Point p) {
//     root = insertRec(root, p, 0);
// }

// // Public method to search for points within a radius
// std::vector<Point> KDTree::radiusSearch(Point target, double radius) {
//     std::vector<Point> result;
//     radiusSearchRec(root, target, radius, 0, result);
//     return result;
// }

// // === Clustering function ===

// // Cluster points using radius-based grouping (DBSCAN-like)
// std::vector<std::vector<Point>> clusterPoints(const std::vector<Point>& allPoints, double radius, int minPoints) {
//     std::unordered_set<Point> visited;
//     std::vector<std::vector<Point>> clusters;
//     KDTree kd;

//     // Insert all points into the KD-tree
//     for (const auto& p : allPoints) kd.insert(p);

//     // Iterate over each point and group them based on density
//     for (const auto& p : allPoints) {
//         if (visited.count(p)) continue;

//         // Find neighboring points within the radius
//         std::vector<Point> neighbors = kd.radiusSearch(p, radius);
//         if (neighbors.size() < minPoints) {
//             visited.insert(p);  // Mark as noise or sparse
//             continue;
//         }

//         // Create a new cluster and expand it by adding reachable neighbors
//         std::vector<Point> cluster;
//         std::vector<Point> toProcess = neighbors;
//         while (!toProcess.empty()) {
//             Point current = toProcess.back();
//             toProcess.pop_back();

//             if (visited.count(current)) continue;
//             visited.insert(current);
//             cluster.push_back(current);

//             // Find more neighbors for the current point
//             std::vector<Point> subNeighbors = kd.radiusSearch(current, radius);
//             if (subNeighbors.size() >= minPoints) {
//                 toProcess.insert(toProcess.end(), subNeighbors.begin(), subNeighbors.end());
//             }
//         }

//         clusters.push_back(cluster);
//     }

//     return clusters;
// }


// KDTree::~KDTree() {
//     // Implement a destructor to clean up dynamically allocated memory.
//     clearTree(root);
// }

// void KDTree::clearTree(KDNode* node) {
//     if (node) {
//         clearTree(node->left);
//         clearTree(node->right);
//         delete node;
//         node = nullptr;
//     }
// }
