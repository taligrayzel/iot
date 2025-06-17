#ifndef POSITIONAL_STL
#define POSITIONAL_STL
#include <cmath>

// #include <unordered_map>
// #include <unordered_set>
// #include <vector>
// #include <memory>

struct Point {
    double x, y;

    Point(double x = 0.0, double y = 0.0);

    Point operator-(const Point& other) const;
    Point operator+(const Point& other) const;
    Point operator*(double scalar) const;
    bool operator==(const Point& other) const;
    Point& operator+=(const Point& other);

    double cross(const Point& other) const;
    double length() const;
    double dot(const Point& other) const;
    Point normalized() const;
};

inline Point operator*(double scalar, const Point& p) {
    return Point(p.x * scalar, p.y * scalar);
}

struct Pose {
  Point loc;
  double vLinear; // Linear velocity (m/s)
  double vAngular;// Angular velocity (rad/s)
  double theta; // orientation
};


Point toWorldFrame(const Pose& pose, const Point& localPoint);


// // === Hash function for Point to use in unordered_set ===
// namespace std {
//     template <>
//     struct hash<Point> {
//         size_t operator()(const Point& p) const {
//             return hash<double>()(p.x) ^ hash<double>()(p.y);
//         }
//     };
// }

// struct PoseNode {
//     int id;
//     Pose pose;
//     double timestamp;  // Optional
// };

// struct LandmarkNode {
//     int id;
//     Point position;
// };

// struct Edge {
//     int id;  // Optional
//     int from_id;
//     int to_id;
//     enum Type { ODOMETRY, OBSERVATION } type;
//     Pose relative_pose;         // For odometry
//     Point observation_offset;   // For observation
//     double info[3][3];          // Information matrix (covariance inverse)

//     static Edge makeOdometryEdge(int from, int to, const Pose& rel, double variance);
// };

// class Graph {
// public:
//     std::unordered_map<int, PoseNode> poses;
//     std::unordered_map<int, LandmarkNode> landmarks;
//     std::vector<Edge> edges;

//     // Fast lookup for optimization, pruning, etc.
//     std::unordered_map<int, std::vector<Edge>> edges_from;
//     std::unordered_map<int, std::vector<Edge>> edges_to;

//     void addPose(int id, const Pose& pose);
//     void addLandmark(int id, const Point& pos);
//     void addOdometryEdge(int from_id, int to_id, const Pose& rel_pose);
//     void addObservationEdge(int pose_id, int landmark_id, const Point& measurement);

//     void addEdge(const Edge& e);

//     std::vector<const Edge*> getOdometryEdgesFrom(int pose_id) const;

//     // Clone the graph (deep copy)
//     std::shared_ptr<Graph> clone() const;
// };

// // Particle structure containing robot pose, a graph of map (landmarks), and weight
// struct Particle {
//     std::shared_ptr<Graph> graph;
//     double weight;

//     Particle(std::shared_ptr<Graph> g, double w = 1.0);
// };

#endif // POSITIONAL_STL
