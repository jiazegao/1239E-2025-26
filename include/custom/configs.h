#pragma once

#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
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
inline alliance_color allianceColor = alliance_color::RED;

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
inline pros::MotorGroup leftMotors({-11, -12, -10}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({19, 17, 18}, pros::MotorGearset::blue);
inline pros::Motor frontMotor(20, pros::MotorGearset::blue);
inline pros::Motor topMotor(9, pros::MotorGearset::blue);

inline lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.375,
                              3.25,
                              450,
                              2
);

// Odometry
inline pros::Rotation vertSensor(-7);
inline pros::Rotation horiSensor(16);

// IMU
inline pros::Imu imu(19);

// Distance
inline pros::Distance dist1(18);

// Optical
inline pros::Optical topOptic(15);

// Pneumatics
inline pros::adi::Pneumatics matchLoadGate('F', false, false);
inline pros::adi::Pneumatics middleMech('B', false, false);
inline pros::adi::Pneumatics middleDescore('C', false, false);
inline pros::adi::Pneumatics leftDescoreArm('A', false, false);
inline pros::adi::Pneumatics rightDescoreArm('H', false, false);

// Odometry

lemlib::TrackingWheel horizontal_tracking_wheel(&horiSensor, lemlib::Omniwheel::NEW_275, -2.4399925, 1.0);
lemlib::TrackingWheel vertical_tracking_wheel(&vertSensor, lemlib::Omniwheel::NEW_275, -0.409145, 1.0);

inline lemlib::OdomSensors sensors( &vertical_tracking_wheel,
                                    nullptr,
                                    &horizontal_tracking_wheel,
                                    nullptr,
                                    &imu
);

// Lateral PID controller
inline lemlib::ControllerSettings lateral_controller(
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
inline lemlib::ControllerSettings angular_controller(5, // proportional gain (kP)
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
inline lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                            10, // minimum output where drivetrain will move out of 127
                                                1.019 // expo curve gain
);

// Steer curve
inline lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                         10, // minimum output where drivetrain will move out of 127
                                             1.019 // expo curve gain
);

// Chassis
inline lemlib::Chassis chassis( drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// Rcl setup
//static RclSensor rcl1(&dist1, 0.0, 0.0, 0.0, 15.0);
inline RclTracking RclMain(&chassis);

// loaders
inline Circle_Obstacle redUpLoader(-67.5, 46.5, 4);
inline Circle_Obstacle redDownLoader(-67.5, -46.5, 4);
inline Circle_Obstacle blueUpLoader(67.5, 46.5, 4);
inline Circle_Obstacle blueDownLoader(67.5, -46.5, 4);

// legs
inline Circle_Obstacle upLongGoalLeft(-21, 47.5, 4);
inline Circle_Obstacle upLongGoalRight(21, 47.5, 4);
inline Circle_Obstacle downLongGoalLeft(-21, -47.5, 4);
inline Circle_Obstacle downLongGoalRight(21, -47.5, 4);

inline Circle_Obstacle centerGoals(0, 0, 5);


