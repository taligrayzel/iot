// #ifndef GRAPHSLAM_PARTICLE_FILTER_H
// #define GRAPHSLAM_PARTICLE_FILTER_H

// #include "GraphSLAM.h"  // Assuming this includes your Graph class and other necessary components
// #include "KLDResampler.h"  // Assuming this includes your KLD resampler class
// #include <vector>
// #include <memory>
// #include <random>
// #include <cmath>

// class GraphSLAM_particle_filter {
// private:
//     std::vector<Particle> particles;  // Particle set
//     int numParticles;
//     std::default_random_engine gen;
//     KLDResampler kld_resampler;  // KLD resampler instance

//     int next_pose_id = 0;
//     int next_landmark_id = 1000;
//     int next_edge_id = 0;

//     int generatePoseId();
//     int generateLandmarkId();
//     int generateEdgeId();

// public:
//     GraphSLAM_particle_filter(int n, double bin_x, double bin_y, double bin_theta, 
//                               double eps, double z, int max_n);

//     // Initialize particles
//     void initialize(double x_range, double y_range);

//     // Predict step for each particle (update pose)
//     void predict(double delta_x, double delta_y, double delta_theta);

//     // Ensure unique graph for particle
//     void ensureUniqueGraph(std::shared_ptr<Graph>& g);

//     // Update weights based on sensor data (GraphSLAM version)
//     void updateWeights(const std::vector<Point>& observations, double sensor_range);

//     // Resampling (using KLD resampling)
//     void resample();

//     // Display the estimated position of the robot
//     Pose estimatePosition();

//     // Add landmark to a particle’s map
//     void addLandmarkToParticle(int particle_idx, int landmark_id, LandmarkNode lm);
// };

// #endif // GRAPHSLAM_PARTICLE_FILTER_H

















/*
// ESP32-specific compression (e.g., heatshrink or miniz), with CRC and Firebase-safe upload
#include <Arduino.h>
#include <LittleFS.h>
#include "LogDir.h"
#include "DataBase.h"
#include "heatshrink_encoder.h"
#include "CRC32.h"
#include <base64.h>

#define MAX_BUFFER_RECORDS 100
#define COMPRESSED_BUFFER_SIZE 2048
#define HSE_WINDOW_SZ2 8
#define HSE_LOOKAHEAD_SZ2 4

bool LogDir::commitRecord(LogRecord* rec, bool block) {
  if (xSemaphoreTake(bufferMutex, block ? portMAX_DELAY : 0) != pdTRUE) {
    DPRINTLN("WARNING: Could not commit record, mutex busy");
    return false;
  }

  auto it = std::find(activeRecords.begin(), activeRecords.end(), rec);
  if (it != activeRecords.end()) {
    commitedRecords.push_back(*it);
    activeRecords.erase(it);
  }

  xSemaphoreGive(bufferMutex);

  if (commitedRecords.size() >= MAX_BUFFER_RECORDS) {
    flushToFile();
  }
  return true;
}

void LogDir::flushToFile() {
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  std::deque<LogRecord*> toWrite = std::move(commitedRecords);
  commitedRecords.clear();
  xSemaphoreGive(bufferMutex);

  if (toWrite.empty()) return;

  String combinedJson = "[";
  for (size_t i = 0; i < toWrite.size(); ++i) {
    combinedJson += toWrite[i]->getJson().raw();
    if (i != toWrite.size() - 1) combinedJson += ",";
    delete toWrite[i];
  }
  combinedJson += "]";

  // Compress using heatshrink
  uint8_t compOut[COMPRESSED_BUFFER_SIZE];
  heatshrink_encoder hse;
  heatshrink_encoder_reset(&hse);
  size_t inputSize = combinedJson.length();
  const uint8_t* input = (const uint8_t*)combinedJson.c_str();
  size_t sunk = 0, compLen = 0;

  while (sunk < inputSize) {
    size_t inLen = inputSize - sunk;
    heatshrink_encoder_sink(&hse, input + sunk, inLen, &inLen);
    sunk += inLen;

    size_t outLen;
    HSE_poll_res pres;
    do {
      pres = heatshrink_encoder_poll(&hse, compOut + compLen, COMPRESSED_BUFFER_SIZE - compLen, &outLen);
      compLen += outLen;
    } while (pres == HSER_POLL_MORE);

    heatshrink_encoder_finish(&hse);
  }

  size_t outLen;
  while (heatshrink_encoder_poll(&hse, compOut + compLen, COMPRESSED_BUFFER_SIZE - compLen, &outLen) == HSER_POLL_MORE) {
    compLen += outLen;
  }
  heatshrink_encoder_finish(&hse);

  // Calculate CRC
  CRC32 crc;
  crc.update(compOut, compLen);
  uint32_t crcValue = crc.finalize();

  String encoded = base64::encode(compOut, compLen);

  xSemaphoreTake(fileMutex, portMAX_DELAY);
  File f = LittleFS.open(fileName, "a");
  if (!f) {
    DPRINTLN("ERROR: Cannot open file for compressed flush");
    xSemaphoreGive(fileMutex);
    return;
  }

  f.printf("%08X:%s\n", crcValue, encoded.c_str());
  f.close();
  xSemaphoreGive(fileMutex);
  DPRINTLN("Compressed + CRC block written");
}

void LogDir::uploadFileToFirebase(FirebaseData* fbdo) {
  xSemaphoreTake(fileMutex, portMAX_DELAY);

  File file = LittleFS.open(fileName, "r");
  if (!file) {
    DPRINTLN("No file to upload for " + category);
    xSemaphoreGive(fileMutex);
    return;
  }

  FirebaseJson wrapper;
  FirebaseJsonArray uploadArray;
  size_t totalSize = 0;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (line.trim().isEmpty()) continue;
    uploadArray.add(line);
    totalSize += line.length();
  }
  file.close();

  if (uploadArray.size() == 0) {
    xSemaphoreGive(fileMutex);
    return;
  }

  wrapper.set("compressed", uploadArray);
  String path = "/" + category + "/" + LogDir::timeHMSms();

  if (Firebase.RTDB.setJSON(fbdo, path, &wrapper)) {
    DPRINTLN("Uploaded compressed log: " + path);
    LittleFS.remove(fileName);
  } else {
    DPRINTLN("Failed to upload compressed log: " + fbdo->errorReason());
  }

  xSemaphoreGive(fileMutex);
}
*/
