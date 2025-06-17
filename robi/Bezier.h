#ifndef BEZIER_H
#define BEZIER_H

#include <array>
#include <cmath>
#include "positional_stl.h" // Assumes Point is defined here

template <int Order>
class Bezier {
    static constexpr int NumPoints = Order + 1;
    std::array<Point, NumPoints> controlPoints;

    // Binomial coefficient
    static constexpr int binomial(int n, int k) {
        return (k == 0 || k == n) ? 1 : binomial(n - 1, k - 1) + binomial(n - 1, k);
    }

    // Compute basis function
    static double bernstein(int i, double t) {
        return binomial(Order, i) * std::pow(1 - t, Order - i) * std::pow(t, i);
    }

public:
    Bezier(const std::array<Point, NumPoints>& points)
        : controlPoints(points) {}

    Point at(double t) const {
        Point result;
        for (int i = 0; i < NumPoints; ++i) {
            result = result + controlPoints[i] * bernstein(i, t);
        }
        return result;
    }

    // First derivative (tangent)
    Point getTangent(double t) const {
        Point result;
        for (int i = 0; i < Order; ++i) {
            Point diff = controlPoints[i + 1] - controlPoints[i];
            result = result + diff * (Order * bernstein(i, t));
        }
        return result;
    }

    // Second derivative (curvature-related)
    Point getSecondDerivative(double t) const {
        Point result;
        for (int i = 0; i < Order - 1; ++i) {
            Point diff = controlPoints[i + 2] - 2 * controlPoints[i + 1] + controlPoints[i];
            result = result + diff * (Order * (Order - 1) * bernstein(i, t));
        }
        return result;
    }

    double curvature_at(double t) const {
        Point d1 = getTangent(t);
        Point d2 = getSecondDerivative(t);
        double num = std::abs(d1.x * d2.y - d1.y * d2.x);
        double denom = std::pow(d1.x * d1.x + d1.y * d1.y, 1.5);
        return denom != 0 ? num / denom : 0.0;
    }

    double getOffsetCurvature(double t, double offset) const {
        double k = curvature_at(t);
        return k / (1 - offset * k);
    }

    double findClosestPoint(const Point& target, int resolution) const {
    double minDist = -10000000000000.0;
    double closestT = 0.0;

    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        Point pt = at(t);
        double dist = (pt - target).length();
        if (dist < minDist) {
            minDist = dist;
            closestT = t;
        }
    }

    return closestT;
}
};

#endif // BEZIER_H
