#include "positional_stl.h"
#include <iostream>

Point::Point(double x, double y) : x(x), y(y) {}

Point Point::operator-(const Point& other) const {
    return {x - other.x, y - other.y};
}

Point Point::operator+(const Point& other) const {
    return {x + other.x, y + other.y};
}

Point Point::operator*(double scalar) const {
    return {x * scalar, y * scalar};
}

bool Point::operator==(const Point& other) const {
    return (x == other.x && y == other.y);
}

Point& Point::operator+=(const Point& other) {
    x += other.x;
    y += other.y;
    return *this;
}

double Point::cross(const Point& other) const {
    return x * other.y - y * other.x;
}

double Point::length() const {
    return std::sqrt(x * x + y * y);
}

double Point::dot(const Point& other) const {
    return x * other.x + y * other.y;
}

Point Point::normalized() const {
    double len = length();
    return len > 0 ? (*this) * (1.0 / len) : Point(0, 0);
}


Point toWorldFrame(const Pose& pose, const Point& localPoint) {
    double cos_theta = std::cos(pose.theta);
    double sin_theta = std::sin(pose.theta);

    return {
        pose.loc.x + cos_theta * localPoint.x - sin_theta * localPoint.y,
        pose.loc.y + sin_theta * localPoint.x + cos_theta * localPoint.y
    };
}

// // Edge method to create an odometry edge
// Edge Edge::makeOdometryEdge(int from, int to, const Pose& rel, double variance) {
//     Edge e;
//     e.from_id = from;
//     e.to_id = to;
//     e.type = Edge::ODOMETRY;
//     e.relative_pose = rel;

//     double inv_var = 1.0 / variance;
//     e.info[0][0] = inv_var;
//     e.info[1][1] = inv_var;
//     e.info[2][2] = inv_var;
//     return e;
// }

// // Graph class methods

// void Graph::addPose(int id, const Pose& pose) {
//     PoseNode node;
//     node.id = id;
//     node.pose = pose;
//     poses[id] = node;
// }

// void Graph::addLandmark(int id, const Point& pos) {
//     LandmarkNode node;
//     node.id = id;
//     node.position = pos;
//     landmarks[id] = node;
// }

// void Graph::addOdometryEdge(int from_id, int to_id, const Pose& rel_pose) {
//     addOdometryEdge(from_id, to_id, rel_pose, 0.01);
// }

// void Graph::addObservationEdge(int pose_id, int landmark_id, const Point& measurement) {
//     Edge e;
//     e.from_id = pose_id;
//     e.to_id = landmark_id;
//     e.type = Edge::OBSERVATION;
//     e.observation_offset = measurement;
//     e.info[0][0] = 1.0;  // Assuming some information matrix (identity for now)
//     e.info[1][1] = 1.0;
//     e.info[2][2] = 1.0;
//     addEdge(e);
// }

// void Graph::addEdge(const Edge& e) {
//     edges.push_back(e);
//     edges_from[e.from_id].push_back(e);
//     edges_to[e.to_id].push_back(e);
// }

// std::vector<const Edge*> Graph::getOdometryEdgesFrom(int pose_id) const {
//     std::vector<const Edge*> result;
//     auto it = edges_from.find(pose_id);
//     if (it != edges_from.end()) {
//         for (const Edge& e : it->second) {
//             if (e.type == Edge::ODOMETRY)
//                 result.push_back(&e);
//         }
//     }
//     return result;
// }

// // Clone the graph (deep copy)
// std::shared_ptr<Graph> Graph::clone() const {
//     auto new_graph = std::make_shared<Graph>();
//     new_graph->poses = poses;
//     new_graph->landmarks = landmarks;
//     new_graph->edges = edges;
//     new_graph->edges_from = edges_from;
//     new_graph->edges_to = edges_to;
//     return new_graph;
// }

// // Particle constructor
// Particle::Particle(std::shared_ptr<Graph> g, double w) : graph(g), weight(w) {}

