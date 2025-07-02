// #ifndef KDTREE_H
// #define KDTREE_H

// #include <vector>
// #include <unordered_set>
// #include <cmath>


// // === Simplified KD-Tree for radius search ===
// class KDTree {
// private:
//     // Node structure for KD-tree
//     struct KDNode {
//         Point point;
//         KDNode* left = nullptr;
//         KDNode* right = nullptr;

//         KDNode(Point p) : point(p) {}
//     };

//     KDNode* root = nullptr;

//     // Recursively insert a point into the KD-tree
//     KDNode* insertRec(KDNode* node, Point p, int depth);

//     // Recursively search for points within a given radius
//     void radiusSearchRec(KDNode* node, Point target, double radius, int depth, std::vector<Point>& result);

// public:
//     ~KDTree();

//     void clearTree(KDNode* node);

//     // Public method to insert a point
//     void insert(Point p);

//     // Public method to search for points within a given radius
//     std::vector<Point> radiusSearch(Point target, double radius);
// };

// // === Clustering using radius-based grouping (similar to DBSCAN) ===
// std::vector<std::vector<Point>> clusterPoints(const std::vector<Point>& allPoints, double radius, int minPoints);

// #endif // KDTREE_H
