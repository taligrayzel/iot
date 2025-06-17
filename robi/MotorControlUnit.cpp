#include "HardwareSerial.h"
#include "MotorControlUnit.h"
#include "Config.h"  // Include the combined config file

#define MS_TO_RPM(ms) ((ms) * 60.0) / (2.0 * 3.14159 * (WHEEL_RADIUS))
#define RPM_TO_MS(rpm) ((rpm) * 2.0 * 3.14159 * (WHEEL_RADIUS) / 60)

#define DEG_TO_DIST(degrees) ((degrees) * 3.141592653589793 / 180.0 * (WHEEL_RADIUS))
#define DIST_TO_DEG(dists) ((dists) * 180.0 * (WHEEL_RADIUS) / 3.141592653589793)

MotorControlUnit::MotorControlUnit() :
  path(NULL), interPath(NULL), interT(0), lookaheadDistance(0), planingCycleTime(0),
  motor1(IN_A_1, IN_A_2, ENC_A_1, ENC_A_2,PWM_A_1,PWM_A_2,GEAR_RATIO,TPR),
  motor2(IN_B_1, IN_B_2, ENC_B_1, ENC_B_2,PWM_B_1,PWM_B_2,GEAR_RATIO,TPR),
  currentMovement(MovementType::IDLE), targetDistance(0), distanceTraveled(0),
  targetAngle(0), angleTurned(0), syncPid(1.0f, 0.0f, 0.0f),ticksCounter(0){}

void MotorControlUnit::driveManual(float normalizedSpeed){
  currentMovement = MovementType::MANUAL_CONTROL;
  motor1.setSpeedTarget(normalizedSpeed);
  motor2.setSpeedTarget(normalizedSpeed);
}

void MotorControlUnit::moveDistance(double distance){
  currentMovement = MovementType::MOVING_DISTANCE;
  motor1.setPositionTarget(DIST_TO_DEG(distance));
  motor2.setPositionTarget(DIST_TO_DEG(distance));
}

void MotorControlUnit::turnAngle(double degrees){
    currentMovement = MovementType::TURNING_ANGLE;
    double theta_rad = degrees * M_PI / 180.0;
    double arcLength = (WHEEL_BASE / 2.0) * theta_rad;        // meters
    double wheelDegrees = DIST_TO_DEG(arcLength);             // degrees
    // motor1.setPositionTarget(wheelDegrees);
    motor2.setPositionTarget(-wheelDegrees);
  }

void MotorControlUnit::drivePath(Bezier<2>* path){
  if(!path)
    return;
  this->path = path;
  currentMovement = MovementType::FOLLOWING_PATH;
}

// Stop both motors
void MotorControlUnit::stopMovement() {
  currentMovement = MovementType::IDLE;
  motor1.stop();
  motor2.stop();
}

int MotorControlUnit::control_point_search(Point& res, const Point& ref, const Bezier<2>* ref_path , double c){
  // int flag = 3;
  // double t = ref_path->findClosestPoint(ref,100);
  // double ds = (ref_path->at(t) - ref).length();
  // if(ds  > lookaheadDistance){
  //   res = (lookaheadDistance / ds) * (ref_path->at(t) - ref);
  //   if(ds > 2*lookaheadDistance)
  //     return 2;
  //   else
  //     return 1;
  // }
  // double dp = 0;
  // double dt = (1-t)/100;
  // double t_i = t + dt;
  // for(; t_i < 1; t_i += dt){
  //   dp += (ref_path->at(t_i) - ref_path->at(t_i - dt)).length();
  //   double d = sqrt(dp*dp + ds*ds);
  //   double k = ref_path->curvature_at(t_i);
  //   double l_crit = min(lookaheadDistance, 1/(c*k));
  //   if(d >= l_crit){
  //     flag = 0;
  //     break;
  //   }
  // }
  // res = ref_path->at(t_i);
  // return flag;
  return 0;
}

Bezier<4>* MotorControlUnit::create_intermidiate_path(){
  std::array<Point, 5> controlPoints;
  // double tt_time; // total time of each planning cycle;
  // double max_v = RPM_TO_MS(min(motor1.getCurrentMaxRpm(), motor2.getCurrentMaxRpm()));
  // planingCycleTime = (4*lookaheadDistance) / max_v;
  // tt_time = planingCycleTime*planingCycleTime;
  // controlPoints[0] = robotPos.loc;
  // Point localP1 = Point(robotPos.vLinear*planingCycleTime/4, 0);
  // double p2y = robotPos.vAngular*robotPos.vLinear*tt_time / 12;
  // Point localP2 = Point(localP1.x  + sqrt(max(pow((max_v*planingCycleTime / 4),2) - (p2y*p2y),0.0)), p2y);
  // controlPoints[1] = toWorldFrame(robotPos, localP1);
  // controlPoints[2] = toWorldFrame(robotPos, localP2);
  // int flag = control_point_search(controlPoints[3] , controlPoints[2], path, 1);
  // if(flag == 0 ){
  //   control_point_search(controlPoints[4] , controlPoints[3], path, 1);
  // }
  // else if(flag == 1){
  //   controlPoints[3] += controlPoints[2];
  //   control_point_search(controlPoints[4] , controlPoints[3], path, 1);
  // }else if(flag == 2){
  //   controlPoints[4] = 2 * controlPoints[3];
  //   controlPoints[3] += controlPoints[2];
  //   controlPoints[4] += controlPoints[2];
  // }else{
  //   finishedPath = true;
  // }
  return new Bezier<4>(controlPoints);
}

int MotorControlUnit::planIntermediatePath() {
    // if(interPath) delete interPath;
    // if(!finishedPath){
    //   interPath = create_intermidiate_path();
    //   interPathStartTime = millis();
    //   return 1;
    // }
     return 0;
}

int MotorControlUnit::pathPursuitController() {
  // if(!path)
  //   return 1; // no path;
  // if(!interPath){
  //   planIntermediatePath();
  // }
  // double elapsedTime = (millis() - interPathStartTime) / 1000.0; // seconds
  // double t = elapsedTime / planingCycleTime;
  // if (t >= 1) {
  //   if(planIntermediatePath())
  //     t = 0;
  //   else
  //     return 1; 
  // }
  // sendMotorsCommands(t);
   return 0;  // Return success
}

void MotorControlUnit::calculateSpeed(){
  Serial.print("motor1 movement speed: ");
  Serial.println(motor1.getSmoothedRPM());
  Serial.print("motor2 movement speed: ");
  Serial.println(motor2.getSmoothedRPM());
}

void MotorControlUnit::sync_Motors(float dt){
  // float m1_target_speed = motor1.getTargetSpeed();
  // float m2_target_speed = motor2.getTargetSpeed();
  // float m1_curr_speed = motor1.getSmoothedRPM();
  // float m2_curr_speed = motor2.getSmoothedRPM();
  // float velocityError = m1_curr_speed - m2_curr_speed;
  // float correction = syncPid.update(velocityError, dt);
  // m1_target_speed -= correction;
  // m2_target_speed += correction;
  // motor1.setSpeedTarget(m1_target_speed);
  // motor2.setSpeedTarget(m2_target_speed);
}

// Turn the robot right
void MotorControlUnit::updateMovement(float dt) {
  updateOdometry();
  Serial.print("ACTION: ");
  switch(currentMovement){
    case MovementType::FOLLOWING_PATH:
      Serial.println("FOLLOWING_PATH");
      if(ticksCounter >= TICK_TO_CONTROL){
        planIntermediatePath();
        ticksCounter = 0;
      }
      pathPursuitController();
      ticksCounter++;
      break;
    case MovementType::MANUAL_CONTROL:
      Serial.println("MANUAL_CONTROL");
      sync_Motors(dt);
      break;
    case MovementType::MOVING_DISTANCE:
      Serial.println("MOVING_DISTANCE");
      Serial.print("Motor1: ");
      motor1.updatePositionControl(dt);
      Serial.print("Motor2: ");
      motor2.updatePositionControl(dt);
      // sync_Motors(dt);
      break;
      //add sync motor speeds
    case MovementType::TURNING_ANGLE:
      Serial.println("TURNING_ANGLE");
      motor1.updatePositionControl(dt);
      motor2.updatePositionControl(dt);
      break;
    case MovementType::IDLE:
      Serial.println("IDLE");
      stopMovement();
  }
  Serial.print("Motor1: ");
  motor1.tick(dt);
  Serial.print("Motor2: ");
  motor2.tick(dt);
  // calculateSpeed(); 
 }

// Display sensor values to the serial monitor
void MotorControlUnit::clearMovementData() {
    motor1.clearHistory();
    motor2.clearHistory();
}

bool MotorControlUnit::begin(){
  attachAllEncoderInterrupts();
  return true;
}

// Call this periodically (e.g., every 50–100 ms)
void MotorControlUnit::updateOdometry() {
  static double lastLeftDeg = 0.0;
  static double lastRightDeg = 0.0;

  double leftDeg = motor1.get_deg();
  double rightDeg = motor2.get_deg();

  double dLeft = DEG_TO_DIST(leftDeg - lastLeftDeg);   // meters
  double dRight = DEG_TO_DIST(rightDeg - lastRightDeg); // meters

  lastLeftDeg = leftDeg;
  lastRightDeg = rightDeg;

  double dCenter = (dLeft + dRight) / 2.0; // linear displacement
  double dTheta = (dRight - dLeft) / WHEEL_BASE; // angular displacement
  if (fabs(dTheta) < 1e-6) {
      // Straight line approximation for tiny turns
      robotPos.loc += Point(dCenter * cos(robotPos.theta), dCenter * sin(robotPos.theta));
  } else {
      double radius = dCenter / dTheta; // Radius of the turn
      double cx = robotPos.loc.x - radius * sin(robotPos.theta);
      double cy = robotPos.loc.y + radius * cos(robotPos.theta);

      robotPos.theta += dTheta;
      robotPos.theta = fmod(robotPos.theta + 2 * M_PI, 2 * M_PI);

      robotPos.loc.x = cx + radius * sin(robotPos.theta);
      robotPos.loc.y = cy - radius * cos(robotPos.theta);
  }
  double deltaTime = 1.0 / 50.0; // Assuming you're updating at 50 Hz (adjust as needed)
  robotPos.vLinear = dCenter / deltaTime;
  robotPos.vAngular = dTheta / deltaTime;
}

bool MotorControlUnit::isMovementComplete() const{
  return motor1.get_DriveMode() == DriveMode::IDLE && motor2.get_DriveMode() == DriveMode::IDLE;
}       // Check if current motion command is complete

void MotorControlUnit::updatePosition(const Pose& newPos){
  
}
