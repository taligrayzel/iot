#ifndef MOTORCONTROLUNIT_H
#define MOTORCONTROLUNIT_H

#include "Bezier.h"         // Bezier curve path definitions
#include "Motor.h"          // Motor class for controlling individual motors
#include "positional_stl.h" // Pose and Point definitions
#include <cmath>

enum class MovementType {
    IDLE,
    MOVING_DISTANCE,
    TURNING_ANGLE,
    FOLLOWING_PATH,
    MANUAL_CONTROL
};

class MotorControlUnit {
private:
    // ====== Path Planning and Tracking ======
    Bezier<2>* path;
    Bezier<4>* interPath;
    double interT;
    double lookaheadDistance;
    double planingCycleTime;

    // ====== Robot State ======
    Pose robotPos;
    Motor motor1, motor2;
    MovementType currentMovement;

    // ====== Motion Tracking ======
    double targetDistance;
    double distanceTraveled;
    double targetAngle;
    double angleTurned;
    PIDController syncPid; 

    // ====== Internal Helpers ======
    int ticksCounter;

    Bezier<4>* create_intermidiate_path();
    int control_point_search(Point& result, const Point& ref, const Bezier<2>* ref_path, double c);
    int planIntermediatePath();

    void updateOdometry();
    void setSpeed(double linearSpeed, double angularSpeed);  // Advanced: control velocity
    void sync_Motors(float dt);

    // ====== Pure Pursuit / Path Following ======
    int pathPursuitController(); // Executes pure pursuit control loop
    void clearMovementData();
    

public:
    // ====== Constructor ======
    MotorControlUnit();

    // ====== Initialization ======
    bool begin();

    // ====== High-Level Movement ======
    void moveDistance(double distance);     // Move forward/backward N meters
    void turnAngle(double degrees);         // Turn robot by N degrees (positive = left, negative = right)
    void driveManual(float normalizedSpeed);
    void drivePath(Bezier<2>* path);
    void stopMovement();

    // ====== Motor and Motion Management ======
    void calculateSpeed();
    bool isMovementComplete() const;        // Check if current motion command is complete

    // ====== Robot State Updates ======
    void updatePosition(const Pose& newPos);
    void updateMovement(float dt);  // Call periodically to progress motion
};

#endif // MOTORCONTROLUNIT_H
