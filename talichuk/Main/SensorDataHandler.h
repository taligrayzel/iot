 #ifndef SENSOR_DATA_HANDLER_H
 #define SENSOR_DATA_HANDLER_H

// #include "ClusteringAlgorithms.h"
 #include "DistanceSensor.h"  // Include the DistanceSensor class
// #include <memory>
 #include <vector>
// #include "Point.h"  // Assuming you have a Point class definition

 class SensorDataHandler {
 private:
//     std::shared_ptr<ClusteringAlgorithm> clustering_algorithm;
  DistanceSensor leftDistanceSensor;  // Instance of DistanceSensor
  DistanceSensor frontDistanceSensor; 
  DistanceSensor rightDistanceSensor; 
  std::vector<float> raw_points_left;
  std::vector<float> raw_points_right;
  std::vector<float> raw_points_front;  // Store collected points directly
  int counter = 0;
//     bool is_burst;  // Flag for burst vs. continuous collection
//     size_t burst_size;  // Number of points to collect in burst mode

//     // Filtering Parameters
//     double min_distance;  // Minimum acceptable distance
//     double max_distance;  // Maximum acceptable distance
 // size_t smoothing_window_size;  // Size of the moving average filter

 public:
 //SensorDataHandler(std::shared_ptr<ClusteringAlgorithm> clustering_method);
  SensorDataHandler();
  bool begin();  // Initialize the sensor
  void collectData();  // Collect data from the sensor
  void setUpDistanceSensors();
  void fillSensorsVectors();
  std::vector<float> getSensorsData();
//     std::vector<std::vector<Point>> processData();  // Process collected data and return clusters
//     void clearData();  // Clear the collected data
//     void setBurstMode(bool burst, size_t size = 10);  // Set collection mode and burst size

//     // Filtering methods
//     void setFilteringParams(double min_dist, double max_dist, size_t smoothing_size);
//     std::vector<Point> filterData(const std::vector<Point>& points);
//     std::vector<Point> smoothData(const std::vector<Point>& points);
 };

 #endif // SENSOR_DATA_HANDLER_H
