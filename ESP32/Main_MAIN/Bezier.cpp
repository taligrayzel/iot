// #include "Bezier.h"

// // Get point on Bezier curve at t ∈ [0, 1]
// Point Bezier::getPoint(double t) const {
//     double u = 1 - t;
//     double tt = t * t;
//     double uu = u * u;
//     double uuu = uu * u;
//     double ttt = tt * t;

//     Point p;
//     p.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
//     p.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
//     return p;
// }

// // Get tangent vector at t
// Point Bezier::getTangent(double t) const {
//     double u = 1 - t;
//     Point tangent;
//     tangent.x =
//         3 * u * u * (p1.x - p0.x) +
//         6 * u * t * (p2.x - p1.x) +
//         3 * t * t * (p3.x - p2.x);
//     tangent.y =
//         3 * u * u * (p1.y - p0.y) +
//         6 * u * t * (p2.y - p1.y) +
//         3 * t * t * (p3.y - p2.y);
//     return tangent;
// }

// Point Bezier::getSecondDerivative(double t) const {
//     double u = 1 - t;
//     Point second_derivative;
//     second_derivative.x =
//         6 * u * (p2.x - 2 * p1.x + p0.x) +
//         6 * t * (p3.x - 2 * p2.x + p1.x);
//     second_derivative.y =
//         6 * u * (p2.y - 2 * p1.y + p0.y) +
//         6 * t * (p3.y - 2 * p2.y + p1.y);
//     return second_derivative;
// }

// // Calculate the curvature at a point on the Bezier curve
// double Bezier::getCurvature(double t) const {
//     // Get the first and second derivatives
//     Point tangent = getTangent(t);
//     Point second_derivative = getSecondDerivative(t);

//     // Numerator: |x'(t) * y''(t) - y'(t) * x''(t)|
//     double numerator = fabs(tangent.x * second_derivative.y - tangent.y * second_derivative.x);

//     // Denominator: (x'(t)^2 + y'(t)^2)^(3/2)
//     double denominator = pow(tangent.x * tangent.x + tangent.y * tangent.y, 1.5);

//     // Curvature formula
//     return numerator / denominator;
// }

// double Bezier::getOffsetCurvature(double t, double offset) const {
//     // Get the original curve curvature at point t
//     double originalCurvature = getCurvature(t);
    
//     // Calculate offset curvature using the formula
//     double offsetCurvature = originalCurvature / (1 + originalCurvature * offset);
    
//     return offsetCurvature;
// }

// double Bezier::findClosestPoint(const Point& target, int resolution) const {
//     double minDist = std::numeric_limits<double>::max();
//     double closestT = 0.0;

//     for (int i = 0; i <= resolution; ++i) {
//         double t = static_cast<double>(i) / resolution;
//         Point pt = getPoint(t);
//         double dist = (pt - target).length();
//         if (dist < minDist) {
//             minDist = dist;
//             closestT = t;
//         }
//     }

//     return closestT;
// }
