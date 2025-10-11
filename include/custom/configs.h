#pragma once

#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/misc.hpp" // IWYU pragma: keep
#include "pros/motor_group.hpp" // IWYU pragma: keep
#include "pros/motors.hpp"   // IWYU pragma: keep
#include "pros/optical.hpp"     // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "cmath"

#include "custom/RclTracking.h"

// Alliance Color
enum class alliance_color { RED, BLUE, NONE };
extern alliance_color allianceColor;

// Controller
extern pros::Controller controller;

// Motors
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor frontMotor;
extern pros::Motor topMotor;

// Drivetrain
extern lemlib::Drivetrain drivetrain;

// IMU
extern pros::Imu imu;

// Distance
extern pros::Distance dist1;

// Optical
extern pros::Optical topOptic;

// Pneumatics
extern pros::adi::Pneumatics matchLoadGate;
extern pros::adi::Pneumatics middleMech;
extern pros::adi::Pneumatics middleDescore;
extern pros::adi::Pneumatics leftDescoreArm;
extern pros::adi::Pneumatics rightDescoreArm;

// Odometry
extern lemlib::OdomSensors sensors;

// Lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// Angular PID controller
extern lemlib::ControllerSettings angular_controller;

// Throttle curve
extern lemlib::ExpoDriveCurve throttle_curve;

// Steer curve
extern lemlib::ExpoDriveCurve steer_curve;

// Chassis
extern lemlib::Chassis chassis;

// Rcl setup
//static RclSensor rcl1(&dist1, 0.0, 0.0, 0.0, 15.0);
extern RclTracking RclMain;

// loaders
extern Circle_Obstacle redUpLoader;
extern Circle_Obstacle redDownLoader;
extern Circle_Obstacle blueUpLoader;
extern Circle_Obstacle blueDownLoader;

// legs
extern Circle_Obstacle upLongGoalLeft;
extern Circle_Obstacle upLongGoalRight;
extern Circle_Obstacle downLongGoalLeft;
extern Circle_Obstacle downLongGoalRight;

extern Circle_Obstacle centerGoals;


