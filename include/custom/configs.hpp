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
#include <cmath>
#include <numbers>

#include "custom/RclTracking.hpp"
#include "custom/MclTracking.hpp"

const int tempPort = 21;

// Alliance Color
enum class alliance_color { RED, BLUE, NONE };
inline alliance_color allianceColor = alliance_color::BLUE;

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
inline pros::MotorGroup leftMotors({tempPort, tempPort, tempPort}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({tempPort, tempPort, tempPort}, pros::MotorGearset::blue);
inline pros::Motor frontMotor(tempPort, pros::MotorGearset::blue);
inline pros::Motor leverMotor(-19, pros::MotorGearset::red);

inline lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.375,
                              3.25,
                              450,
                              2
);

// Odometry
inline pros::Rotation vertSensor(tempPort);

// IMU
inline pros::Imu imu(tempPort);

// Optical
inline pros::Optical frontOptic(tempPort);

// Pneumatics
inline pros::adi::Potentiometer leverPotent('A');
inline pros::adi::Pneumatics matchLoadGate('B', false, false);
inline pros::adi::Pneumatics lift('B', true, true);
inline pros::adi::Pneumatics middleDescore('B', false, false);
inline pros::adi::Pneumatics leftDescoreArm('B', false, false);
inline pros::adi::Pneumatics odomLift('B', false, false);
inline pros::adi::Pneumatics trapDoor('B', false, false);

inline const int potentLimit = 4090;
inline int getLeverPotentReading() {
    return (4090 - leverPotent.get_value());
}

// Odometry
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertSensor, lemlib::Omniwheel::NEW_275, -0.5, 1.0);

inline lemlib::OdomSensors sensors( nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    &imu
);

// Lateral PID controller
inline lemlib::ControllerSettings lateral_controller(
                                              9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              50, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              300, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Angular PID controller
inline lemlib::ControllerSettings angular_controller(3.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                              25, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              300, // large error range timeout, in milliseconds
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

// Distance
inline pros::Distance midDist(tempPort);
inline pros::Distance topDist(tempPort);
inline pros::Distance descoreDist(tempPort);

inline pros::Distance back_dist(tempPort);
inline pros::Distance right_dist(tempPort);
inline pros::Distance lf_dist(tempPort);
inline pros::Distance lb_dist(tempPort);
inline pros::Distance fl_dist(tempPort);
inline pros::Distance fr_dist(tempPort);
inline const std::array<pros::Distance*, 6> DISTANCE_COLLECTION = {&back_dist, &right_dist, &lf_dist, &lb_dist, &fl_dist, &fr_dist};

// Rcl setup
inline RclSensor back_rcl(&back_dist, 5.375, -4.25, 180, 15.0);
inline RclSensor right_rcl(&right_dist, 4.5, 0.0, 90.0, 15.0);
inline RclSensor lf_rcl(&lf_dist, -4.5, 0.0, 270.0, 15.0);
inline RclSensor lb_rcl(&lb_dist, -4.5, 0.0, 270.0, 15.0);
inline RclSensor fl_rcl(&fl_dist, 0.0, 0.0, 0.0, 15.0);
inline RclSensor fr_rcl(&fr_dist, 0.0, 0.0, 0.0, 15.0);
inline RclTracking RclMain(&chassis, 20, true, 0.5, 4.0, 10.0, 6.0, 20);

// Mcl setup
inline MclTracking MclMain(&chassis, &leftMotors, &rightMotors, DISTANCE_COLLECTION, {&vertSensor, 2.75, -0.5}, {nullptr, 0.0, 0.0}, 0, 0, 0, true);

// Logging
inline bool logging = false;
inline std::ofstream* mclLog = new std::ofstream("/usd/MCLDefault.1239e");
inline std::ofstream* rclLog = new std::ofstream("/usd/RCLDefault.1239e");
inline Timer mclLogTimer(10000000000000);
inline Timer rclLogTimer(10000000000000);

// loaders
inline Circle_Obstacle redUpLoader(-67.5, 46.5, 3);
inline Circle_Obstacle redDownLoader(-67.5, -46.5, 3);
inline Circle_Obstacle blueUpLoader(67.5, 46.5, 3);
inline Circle_Obstacle blueDownLoader(67.5, -46.5, 3);

// legs
inline Circle_Obstacle upLongGoalLeft(-21, 47.5, 4);
inline Circle_Obstacle upLongGoalRight(21, 47.5, 4);
inline Circle_Obstacle downLongGoalLeft(-21, -47.5, 4);
inline Circle_Obstacle downLongGoalRight(21, -47.5, 4);

// Disable Line
inline Line_Obstacle disableLine(0, FIELD_NEG_HALF_LENGTH, 0, FIELD_HALF_LENGTH);

inline Circle_Obstacle centerGoals(0, 0, 5);