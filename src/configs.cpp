
#include "custom/configs.h"

alliance_color allianceColor = alliance_color::RED;

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup leftMotors({-1, -13, -12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({11, 10, 9}, pros::MotorGearset::blue);
pros::Motor frontMotor(-2, pros::MotorGearset::blue);
pros::Motor topMotor(-8, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              15,
                              3.25,
                              450,
                              2
);

// IMU
pros::Imu imu(19);

// Distance
pros::Distance dist1(18);

// Optical
pros::Optical topOptic(15);

// Pneumatics
pros::adi::Pneumatics matchLoadGate('A', false, false);
pros::adi::Pneumatics middleMech('B', false, false);
pros::adi::Pneumatics middleDescore('C', false, false);
pros::adi::Pneumatics leftDescoreArm('D', false, false);
pros::adi::Pneumatics rightDescoreArm('E', false, false);

// Odometry
lemlib::OdomSensors sensors( nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    &imu
);

// Lateral PID controller
lemlib::ControllerSettings lateral_controller(
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
lemlib::ControllerSettings angular_controller(5, // proportional gain (kP)
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
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                            10, // minimum output where drivetrain will move out of 127
                                                1.019 // expo curve gain
);

// Steer curve
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                         10, // minimum output where drivetrain will move out of 127
                                             1.019 // expo curve gain
);

// Chassis
lemlib::Chassis chassis( drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// Rcl setup
//static RclSensor rcl1(&dist1, 0.0, 0.0, 0.0, 15.0);
RclTracking RclMain(&chassis);

// loaders
Circle_Obstacle redUpLoader(-67.5, 46.5, 4);
Circle_Obstacle redDownLoader(-67.5, -46.5, 4);
Circle_Obstacle blueUpLoader(67.5, 46.5, 4);
Circle_Obstacle blueDownLoader(67.5, -46.5, 4);

// legs
Circle_Obstacle upLongGoalLeft(-21, 47.5, 4);
Circle_Obstacle upLongGoalRight(21, 47.5, 4);
Circle_Obstacle downLongGoalLeft(-21, -47.5, 4);
Circle_Obstacle downLongGoalRight(21, -47.5, 4);

Circle_Obstacle centerGoals(0, 0, 5);