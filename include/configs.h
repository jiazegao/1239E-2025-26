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

#include "RclTracking.h"

// Controller
static pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
static pros::MotorGroup LeftMotors({-1, -13, -12}, pros::MotorGearset::blue);
static pros::MotorGroup RightMotors({11, 10, 9}, pros::MotorGearset::blue);
static pros::Motor FrontIndexer(-2, pros::MotorGearset::blue);
static pros::Motor BackIndexer(-8, pros::MotorGearset::blue);

// Drivetrain
static lemlib::Drivetrain drivetrain(&LeftMotors,
                              &RightMotors,
                              15,
                              3.25,
                              450,
                              2
);

// IMU
static pros::Imu imu(19);

// Distance
//static pros::Distance dist1(18);

// Odometry
static lemlib::OdomSensors sensors( nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    &imu
);

// Lateral PID controller
static lemlib::ControllerSettings lateral_controller(
                                              5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0 , // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Angular PID controller
static lemlib::ControllerSettings angular_controller(5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Throttle curve
static lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                            10, // minimum output where drivetrain will move out of 127
                                                1.019 // expo curve gain
);

// Steer curve
static lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                         10, // minimum output where drivetrain will move out of 127
                                             1.019 // expo curve gain
);

// Chassis
static lemlib::Chassis chassis( drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// Rcl setup
//static RclSensor rcl1(&dist1, 0.0, 0.0, 0.0, 15.0);
static RclTracking RclMain(&chassis);

// loaders
static Circle_Obstacle redUpLoader(-67.5, 46.5, 4);
static Circle_Obstacle redDownLoader(-67.5, -46.5, 4);
static Circle_Obstacle blueUpLoader(67.5, 46.5, 4);
static Circle_Obstacle blueDownLoader(67.5, -46.5, 4);

// legs
static Circle_Obstacle upConeLeft(-21, 47.5, 4);
static Circle_Obstacle upConeRight(21, 47.5, 4);
static Circle_Obstacle downConeLeft(-21, -47.5, 4);
static Circle_Obstacle downConeRight(21, -47.5, 4);

static Circle_Obstacle center(0, 0, 5);


