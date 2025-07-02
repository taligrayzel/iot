#pragma once

// === MODULE SERIAL DEBUG FLAGS ===
#define DEBUG_MCU      (0 << 0)  // MotorControlUnit.cpp
#define DEBUG_MOTOR    (0 << 1)  // Motor.cpp
#define DEBUG_SENSOR   (0 << 2)  // Sensors
#define DEBUG_DB       (1 << 3)  // Database 
#define DEBUG_ROBOT    (1 << 4)  // Robot
#define DEBUG_MAIN     (1 << 5)  // Main

// Default enabled debug flags — combine the ones you want active here
#ifndef DEBUG_PRINT_FLAG
#define DEBUG_PRINT_FLAG (DEBUG_MCU | DEBUG_MOTOR | DEBUG_SENSOR | DEBUG_DB | DEBUG_ROBOT | DEBUG_MAIN)
#endif

// === MODULE LOG FLAGS ===
#define LOG_MCU      (1 << 0)  // MotorControlUnit.cpp
#define LOG_MOTOR    (1 << 1)  // Motor.cpp
#define LOG_SENSOR   (1 << 2)  // Sensors
#define LOG_DB       (1 << 3)  // Database Logging
#define LOG_ROBOT    (1 << 4)  // Robot
#define LOG_MAIN     (1 << 5)  // Main

// Default enabled log flags — combine the ones you want active here
#ifndef LOG_FLAG
#define LOG_FLAG (LOG_MCU | LOG_MOTOR | LOG_SENSOR | LOG_DB | LOG_ROBOT | LOG_MAIN)
#endif

// --- Compile-time Debug Print Macros ---

// Helper: Check if debug enabled for a flag
#define DEBUG_ENABLED(flag) ((DEBUG_PRINT_FLAG & (flag)) != 0)

// Debug print macros, removed completely at compile time if flag is not enabled
#if DEBUG_PRINT_FLAG & DEBUG_MCU
  #define DPRINT_MCU(x) Serial.print(x)
  #define DPRINTLN_MCU(x) Serial.println(x)
#else
  #define DPRINT_MCU(x)
  #define DPRINTLN_MCU(x)
#endif

#if DEBUG_PRINT_FLAG & DEBUG_MOTOR
  #define DPRINT_MOTOR(x) Serial.print(x)
  #define DPRINTLN_MOTOR(x) Serial.println(x)
#else
  #define DPRINT_MOTOR(x)
  #define DPRINTLN_MOTOR(x)
#endif

#if DEBUG_PRINT_FLAG & DEBUG_SENSOR
  #define DPRINT_SENSOR(x) Serial.print(x)
  #define DPRINTLN_SENSOR(x) Serial.println(x)
#else
  #define DPRINT_SENSOR(x)
  #define DPRINTLN_SENSOR(x)
#endif

#if DEBUG_PRINT_FLAG & DEBUG_DB
  #define DPRINT_DB(x) Serial.print(x)
  #define DPRINTLN_DB(x) Serial.println(x)
#else
  #define DPRINT_DB(x)
  #define DPRINTLN_DB(x)
#endif

#if DEBUG_PRINT_FLAG & DEBUG_ROBOT
  #define DPRINT_ROBOT(x) Serial.print(x)
  #define DPRINTLN_ROBOT(x) Serial.println(x)
#else
  #define DPRINT_ROBOT(x)
  #define DPRINTLN_ROBOT(x)
#endif

#if DEBUG_PRINT_FLAG & DEBUG_MAIN
  #define DPRINT_MAIN(x) Serial.print(x)
  #define DPRINTLN_MAIN(x) Serial.println(x)
#else
  #define DPRINT_MAIN(x)
  #define DPRINTLN_MAIN(x)
#endif

// --- Compile-time Log Macros ---

// Helper: Check if log enabled for a flag

// Logging macros that only compile if log flag enabled
#if LOG_FLAG & LOG_MCU
  #define LOG_MCU_MSG(x) x
#else
  #define LOG_MCU_MSG(x)
#endif

#if LOG_FLAG & LOG_MOTOR
  #define LOG_MOTOR_MSG(x) x
#else
  #define LOG_MOTOR_MSG(x)
#endif

#if LOG_FLAG & LOG_SENSOR
  #define LOG_SENSOR_MSG(x) x
#else
  #define LOG_SENSOR_MSG(x)
#endif

#if LOG_FLAG & LOG_DB
  #define LOG_DB_MSG(x) x
#else
  #define LOG_DB_MSG(x)
#endif

#if LOG_FLAG & LOG_ROBOT
  #define LOG_ROBOT_MSG(x) x
#else
  #define LOG_ROBOT_MSG(x)
#endif

#if LOG_FLAG & LOG_MAIN
  #define LOG_MAIN_MSG(x) x
#else
  #define LOG_MAIN_MSG(x)
#endif

