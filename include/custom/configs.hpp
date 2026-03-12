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
inline pros::MotorGroup leftMotors({-11, -12, 13}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({20, 19, -18}, pros::MotorGearset::blue);
inline pros::Motor frontMotor(17, pros::MotorGearset::blue);
inline pros::Motor leverMotor(-16, pros::MotorGearset::red);

inline lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              10.400,
                              3.25,
                              450,
                              2
);

// IMU
inline pros::Imu imu(15);

// Optical
inline pros::Optical frontOptic(3);

// Pneumatics
inline pros::adi::Potentiometer leverPotent('H');
inline pros::adi::Pneumatics matchLoadGate('D', false, false);
inline pros::adi::Pneumatics lift('C', true, true);
inline pros::adi::Pneumatics leftDescoreArm('A', false, false);
inline pros::adi::Pneumatics trapDoor('B', false, false);
inline pros::adi::Pneumatics intakeLift('G', true, true);


inline const int potentLimit = 4090;
inline int getLeverPotentReading() {
    return (4090 - leverPotent.get_value());
}

// Odometry
inline lemlib::OdomSensors sensors( nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    &imu
);

// Lateral PID controller
inline lemlib::ControllerSettings lateral_controller(
                                              7.0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30.0, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1.5, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Angular PID controller
inline lemlib::ControllerSettings angular_controller(4.0, // proportional gain (kP)
                                            0, // integral gain (kI)
                                              37.7, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in degrees
                                              200, // large error range timeout, in milliseconds
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
inline pros::Distance midDist(9);
inline pros::Distance topDist(10);

enum DISTSENSORS {FRONT, LEFT, BACK, RIGHT, FRONT_LEFT, BACK_LEFT, BACK_RIGHT, FRONT_RIGHT};
inline pros::Distance front_dist(1);
inline pros::Distance left_dist(2);
inline pros::Distance back_dist(14);
inline pros::Distance right_dist(8);
inline pros::Distance fl_dist(4);
inline pros::Distance bl_dist(6);
inline pros::Distance br_dist(5);
inline pros::Distance fr_dist(7);

inline std::array<pros::Distance*, 8> DISTANCE_COLLECTION = {&front_dist, &left_dist, &back_dist, &right_dist, &fl_dist, &bl_dist, &br_dist, &fr_dist};

// Rcl setup
inline RclSensor front_rcl(&front_dist, -2.277110, 6.184952, 0, 15.0);
inline RclSensor left_rcl(&left_dist, -2.674094, -0.733924, 270.0, 15.0);
inline RclSensor back_rcl(&back_dist, 1.75, -5.374061, 180.0, 15.0);
inline RclSensor right_rcl(&right_dist, 2.674094, -0.733924, 90.0, 15.0);
inline RclTracking RclMain(&chassis, 1, false, 0.5, 4.0, 200.0, 6.0, 50);
inline MclTracking MclMain(&chassis, &drivetrain, DISTANCE_COLLECTION, {nullptr, 0.0, 0.0}, {nullptr, 0.0, 0.0}, 0, 0, 0, true);

enum MCL_Log_Format {DISABLED, SDCARD, SCREEN};
inline MCL_Log_Format mclLogType = DISABLED;
inline std::ofstream* mclLog = nullptr;
inline Timer mclLogTimer(100000000.0f);

// Mcl obstacles
inline std::vector<Line_> soloAWP_obstacles = {
    // Alliance Robot Disable Lines
    {{-72.0f, 8.0f}, {-36.0f, 8.0f}},
    {{-36.0f, 8.0f}, {-36.0f, 32.0f}},
    {{-72.0f, 32.0f}, {-36.0f, 32.0f}},
    {{-72.0f, 8.0f}, {-72.0f, 32.0f}},
    // Middle Line
    {{-2.0f, -71.0f}, {-2.0f, 71.0f}}
};
inline std::vector<Line_> quadrant_dividers = {
    // x-axis
    {{-71.0f, 0.0f}, {71.0f, 0.0f}},
    // y-axis
    {{0.0f, -71.0f}, {0.0f, 71.0f}}
};

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