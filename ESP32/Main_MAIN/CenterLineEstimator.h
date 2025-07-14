#pragma once
#include <deque>
#include <vector>
#include "positional_stl.h"  // For Point and centerLine
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


#define MAX_HISTORY 15


class CL_Estimator {
protected:
    SemaphoreHandle_t historyMutex;
    Point history[MAX_HISTORY];
    size_t head;
    size_t size;

public:
    CL_Estimator() : historyMutex(xSemaphoreCreateMutex()), head(0), size(0){}

    virtual ~CL_Estimator() {
        if (historyMutex) vSemaphoreDelete(historyMutex);
    }

    void addMeasurement(const Point& leftWall, const Point& rightWall) {
        Point center((leftWall.x + rightWall.x) / 2.0f,
                     (leftWall.y + rightWall.y) / 2.0f);

        xSemaphoreTake(historyMutex, portMAX_DELAY);
        history[head] = center;
        head = (head + 1) % MAX_HISTORY;
        if (size < MAX_HISTORY) {
            ++size;
        }
        xSemaphoreGive(historyMutex);
    }

    virtual Line* getSmoothedCenterLine() = 0;

    virtual bool isReady() = 0;

    virtual void reset() {
        xSemaphoreTake(historyMutex, portMAX_DELAY);
        head = 0;
        size = 0;
        xSemaphoreGive(historyMutex);
    }

    int getHistory(Point dst[MAX_HISTORY]) {
        xSemaphoreTake(historyMutex, portMAX_DELAY);
        for (size_t i = 0; i < size; ++i) {
            dst[i] = history[(head + MAX_HISTORY - size + i) % MAX_HISTORY];
        }
        int res = size;
        xSemaphoreGive(historyMutex);
        return res;
    }
};

class PCA_CL_Estimator : public CL_Estimator {

public:
    using CL_Estimator::CL_Estimator;
    bool isReady() override{
      return size >= 3;
    }

    Line* getSmoothedCenterLine() override {
    double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    Point local_hist[MAX_HISTORY];
    int localSize = getHistory(local_hist);
    if (localSize < 3) return nullptr;

    for (int i = 0; i < localSize; ++i) {
        const Point& p = local_hist[i];
        sumX += p.x;
        sumY += p.y;
        sumXY += p.x * p.y;
        sumXX += p.x * p.x;
    }

    double meanX = sumX / localSize;
    double meanY = sumY / localSize;
    double numerator = sumXY - (localSize * meanX * meanY);
    double denominator = sumXX - (localSize * meanX * meanX);

    float heading;
    if (std::abs(denominator) < 1e-6) {
        heading = M_PI / 2.0f;
    } else {
        heading = std::atan(numerator / denominator);
    }

    // Ensure the heading matches the movement direction
    Point p0 = local_hist[0];
    Point pN = local_hist[localSize - 1];
    Point movement = pN - p0;
    float movementDir = std::atan2(movement.y, movement.x);

    // Flip heading if opposite direction
    float delta = std::fmod(heading - movementDir + 3*M_PI, 2*M_PI) - M_PI;  // Normalize to [-PI, PI]
    if (std::abs(delta) > M_PI / 2.0f) {
        heading += M_PI;  // Flip direction
    }

    heading = std::fmod(heading + 2*M_PI, 2*M_PI); // Normalize to [0, 2PI]
    return new Line{Point(meanX, meanY), heading};
}
};




// class Ransac_CL_Estimator : public CL_Estimator {

// public:
//     using CL_Estimator::CL_Estimator;

//     Line* getSmoothedCenterLine() const override {
//         if (history.size() < 2) {
//             return nullptr;  // Not enough points to define a line
//       }

//         const int iterations = 100;
//         const float distanceThreshold = 0.2f;
//         size_t bestInliers = 0;
//         Line bestModel;

//         std::default_random_engine rng{std::random_device{}()};
//         std::uniform_int_distribution<size_t> dist(0, history.size() - 1);

//         for (int i = 0; i < iterations; ++i) {
//             size_t i1 = dist(rng);
//             size_t i2 = dist(rng);
//             if (i1 == i2) continue;

//             const Point& p1 = history[i1].loc;
//             const Point& p2 = history[i2].loc;

//             float dx = p2.x - p1.x;
//             float dy = p2.y - p1.y;
//             float norm = std::sqrt(dx * dx + dy * dy);
//             if (norm < 1e-6) continue;

//             float a = dy;
//             float b = -dx;
//             float c = dx * p1.y - dy * p1.x;

//             size_t inliers = 0;
//             for (const auto& cl : history) {
//                 float d = std::fabs(a * cl.loc.x + b * cl.loc.y + c) / norm;
//                 if (d < distanceThreshold)
//                     ++inliers;
//             }

//             if (inliers > bestInliers) {
//                 bestInliers = inliers;
//                 bestModel = Line{
//                     {(p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0},
//                     static_cast<float>(std::atan2(dy, dx))
//                 };
//             }
//         }

//         // Refine center using inliers to bestModel
//         float bestA = sin(bestModel.theta);
//         float bestB = -cos(bestModel.theta);
//         float bestC = -bestA * bestModel.loc.x - bestB * bestModel.loc.y;

//         double sumX = 0, sumY = 0;
//         size_t count = 0;

//         for (const auto& cl : history) {
//             float d = std::fabs(bestA * cl.loc.x + bestB * cl.loc.y + bestC);
//             if (d < distanceThreshold) {
//                 sumX += cl.loc.x;
//                 sumY += cl.loc.y;
//                 ++count;
//             }
//         }

//         if (count == 0) {
//             return new Line(bestModel);  // no refinement possible
//         }

//         Point averagePoint(sumX / count, sumY / count);
//         return new Line{averagePoint, bestModel.theta};
//     }
// };