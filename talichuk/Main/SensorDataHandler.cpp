#include "SensorDataHandler.h"
#include "Config.h"
#include <numeric>

// #include <iostream>

// Constructor
SensorDataHandler::SensorDataHandler()
    : leftDistanceSensor(LOX3_ADDRESS, SHT_LOX3),
      frontDistanceSensor(LOX2_ADDRESS, SHT_LOX2),
      rightDistanceSensor(LOX1_ADDRESS, SHT_LOX1){
        raw_points_left.assign(10, 0.0f); 
        raw_points_right.assign(10, 0.0f);
        raw_points_front.assign(10, 0.0f);
      }

bool SensorDataHandler::begin(){
  digitalWrite(leftDistanceSensor.loxSHT, LOW);
  digitalWrite(frontDistanceSensor.loxSHT, LOW);
  digitalWrite(rightDistanceSensor.loxSHT, LOW);
  
  setUpDistanceSensors();
}

void SensorDataHandler::setUpDistanceSensors(){
  Serial.println("Starting setUp for distance sensor...");
  // all reset
  digitalWrite(leftDistanceSensor.loxSHT, LOW);    
  digitalWrite(frontDistanceSensor.loxSHT, LOW);
  digitalWrite(rightDistanceSensor.loxSHT, LOW);
  delay(10);
  // all unreset
  digitalWrite(leftDistanceSensor.loxSHT, HIGH);
  digitalWrite(frontDistanceSensor.loxSHT, HIGH);
  digitalWrite(rightDistanceSensor.loxSHT, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(leftDistanceSensor.loxSHT, HIGH);
  digitalWrite(frontDistanceSensor.loxSHT, LOW);
  digitalWrite(rightDistanceSensor.loxSHT, LOW);

  // initing LOX1
  if(!leftDistanceSensor.lox.begin(leftDistanceSensor.loxAddress)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(frontDistanceSensor.loxSHT, HIGH);
  digitalWrite(rightDistanceSensor.loxSHT, LOW);
  delay(10);

  //initing LOX2
  if(!frontDistanceSensor.lox.begin(frontDistanceSensor.loxAddress)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }

  // activating LOX3
  digitalWrite(rightDistanceSensor.loxSHT, HIGH);
  delay(10);

  //initing LOX3
  if(!rightDistanceSensor.lox.begin(rightDistanceSensor.loxAddress)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }

  Serial.println("successfuly finished setUpDistanceSensors...");

}

void SensorDataHandler::fillSensorsVectors(){
  raw_points_left[counter%raw_points_left.size()] = leftDistanceSensor.readSensor();
  raw_points_right[counter%raw_points_right.size()] = rightDistanceSensor.readSensor();
  raw_points_front[counter%raw_points_front.size()] = frontDistanceSensor.readSensor();
  counter++;
}

std::vector<float> SensorDataHandler::getSensorsData(){
  std::vector<float> result(3, 0.0f);
  result[0] = accumulate(raw_points_left.begin(), raw_points_left.end(), 0.0f) / raw_points_left.size();
  result[1] = accumulate(raw_points_right.begin(), raw_points_right.end(), 0.0f) / raw_points_right.size();
  result[2] = accumulate(raw_points_front.begin(), raw_points_front.end(), 0.0f) / raw_points_front.size();
  return result;
}

// Collect data from the sensor (example logic, you may customize)
// void SensorDataHandler::collectData() {
//     // Example collection: get a point from the sensor
//     long distance = sensor.readDistance();
//     if (distance != -1) {
//         raw_points.push_back(Point{static_cast<double>(distance), 0.0});
//     }
// }

// // Process collected data and return the clusters
// std::vector<std::vector<Point>> SensorDataHandler::processData() {
//     // Optionally, apply data filtering and smoothing
//     std::vector<Point> filtered_points = filterData(raw_points);
//     std::vector<Point> smoothed_points = smoothData(filtered_points);

//     // Perform clustering based on the chosen algorithm
//     return clustering_algorithm->clusterPoints(smoothed_points);
// }

// // Clear collected data
// void SensorDataHandler::clearData() {
//     raw_points.clear();
// }

// // Set burst or continuous mode for data collection
// void SensorDataHandler::setBurstMode(bool burst, size_t size) {
//     is_burst = burst;
//     burst_size = size;
// }

// // Set filtering parameters
// void SensorDataHandler::setFilteringParams(double min_dist, double max_dist, size_t smoothing_size) {
//     min_distance = min_dist;
//     max_distance = max_dist;
//     smoothing_window_size = smoothing_size;
// }

// // Filter points based on distance range
// std::vector<Point> SensorDataHandler::filterData(const std::vector<Point>& points) {
//     std::vector<Point> filtered_points;
//     for (const auto& point : points) {
//         if (point.x >= min_distance && point.x <= max_distance && point.y >= min_distance && point.y <= max_distance) {
//             filtered_points.push_back(point);
//         }
//     }
//     return filtered_points;
// }

// // Apply smoothing to the points (e.g., moving average filter)
// std::vector<Point> SensorDataHandler::smoothData(const std::vector<Point>& points) {
//     std::vector<Point> smoothed_points;
//     if (points.empty()) return smoothed_points;

//     // Simple moving average filter
//     for (size_t i = 0; i < points.size(); ++i) {
//         double sum_x = 0.0, sum_y = 0.0;
//         size_t count = 0;

//         for (size_t j = (i >= smoothing_window_size ? i - smoothing_window_size : 0); j <= i; ++j) {
//             sum_x += points[j].x;
//             sum_y += points[j].y;
//             ++count;
//         }

//         smoothed_points.push_back(Point{sum_x / count, sum_y / count});
//     }
//     return smoothed_points;
// }
