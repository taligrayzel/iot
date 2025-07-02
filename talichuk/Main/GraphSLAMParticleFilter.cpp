// #include "GraphSLAMParticleFilter.h"
// #include <limits>
// #include <cmath>
// #include "KDTree.h"

// // Generate unique Pose, Landmark, and Edge IDs
// int GraphSLAM_particle_filter::generatePoseId() { return next_pose_id++; }
// int GraphSLAM_particle_filter::generateLandmarkId() { return next_landmark_id++; }
// int GraphSLAM_particle_filter::generateEdgeId() { return next_edge_id++; }

// GraphSLAM_particle_filter::GraphSLAM_particle_filter(int n, double bin_x, double bin_y, double bin_theta, 
//                                                      double eps, double z, int max_n)
//     : numParticles(n), kld_resampler(bin_x, bin_y, bin_theta, eps, z, max_n) {}

// // Initialize particles
// void GraphSLAM_particle_filter::initialize(double x_range, double y_range) {
//     std::uniform_real_distribution<double> dist_x(0.0, x_range);
//     std::uniform_real_distribution<double> dist_y(0.0, y_range);
//     std::uniform_real_distribution<double> dist_theta(0.0, 2 * M_PI);

//     particles.clear();
//     for (int i = 0; i < numParticles; ++i) {
//         Pose p = {dist_x(gen), dist_y(gen), dist_theta(gen)};
//         auto g = std::make_shared<Graph>();

//         int pose_id = generatePoseId();
//         PoseNode initial_pose_node = {pose_id, p};
//         g->addPose(pose_id, p);

//         particles.emplace_back(g, 1.0);
//     }
// }

// // Predict step for each particle (update pose)
// void GraphSLAM_particle_filter::predict(double delta_x, double delta_y, double delta_theta) {
//     for (auto& p : particles) {
//         p.graph->addOdometryEdge(p.graph->poses.size() - 1, p.graph->poses.size(), {delta_x, delta_y, delta_theta});
//     }
// }

// // Ensure the graph is unique for each particle
// void GraphSLAM_particle_filter::ensureUniqueGraph(std::shared_ptr<Graph>& g) {
//     if (!g.unique()) {
//         g = g->clone();  // Only copy if shared by others
//     }
// }

// // Update weights based on sensor data
// void GraphSLAM_particle_filter::updateWeights(const std::vector<Point>& observations, double sensor_range) {
//     for (auto& p : particles) {
//         double weight = 1.0;

//         // Construct a KD-tree for the landmarks in the particle's graph
//         std::vector<Point> particle_landmarks;
//         for (const auto& lm : p.graph->landmarks) {
//             particle_landmarks.push_back(lm.second.position);
//         }

//         // Create a KDTree for the particle's landmarks
//         KDTree landmarkTree;
//         for (const auto& lm : particle_landmarks) {
//             landmarkTree.insert(lm);
//         }

//         // For each observation, find the closest landmarks and update the weight
//         for (const auto& obs : observations) {
//             // Find nearby landmarks using the KD-tree's radius search
//             std::vector<Point> nearby_landmarks = landmarkTree.radiusSearch(obs, sensor_range);

//             if (!nearby_landmarks.empty()) {
//                 double min_dist = std::numeric_limits<double>::max();
//                 // Find the closest landmark in the particle's graph
//                 for (const auto& lm : nearby_landmarks) {
//                     double dist = std::hypot(lm.x - obs.x, lm.y - obs.y);
//                     if (dist < min_dist) {
//                         min_dist = dist;
//                     }
//                 }

//                 // Update weight based on sensor observation and Gaussian likelihood
//                 double sensor_weight = exp(-min_dist * min_dist / (2.0 * 0.1));  // Assuming a fixed variance of 0.1
//                 weight *= sensor_weight;
//             }
//         }
//         p.weight = weight;
//     }
// }

// // Resample particles using KLD resampler
// void GraphSLAM_particle_filter::resample() {
//     particles = kld_resampler.resample(particles);
// }

// // Estimate the position of the robot (average of particle poses)
// Pose GraphSLAM_particle_filter::estimatePosition() {
//     double x_sum = 0.0, y_sum = 0.0;
//     for (const auto& p : particles) {
//         x_sum += p.graph->poses.at(p.graph->poses.size() - 1).pose.x;
//         y_sum += p.graph->poses.at(p.graph->poses.size() - 1).pose.y;
//     }
//     return {x_sum / numParticles, y_sum / numParticles, 0.0};  // Ignoring theta for simplicity
// }

// // Add landmark to a particle’s map
// void GraphSLAM_particle_filter::addLandmarkToParticle(int particle_idx, int landmark_id, LandmarkNode lm) {
//     particles[particle_idx].graph->landmarks[landmark_id] = lm;
// }
