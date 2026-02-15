#include "main.h"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/auton.hpp"

#include "custom/util_funcs.hpp"
#include "custom/auton_selector.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.h"

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

	pros::lcd::initialize();
	initControllerDisplay();
	initBrainDisplay();

	// Retrive file count
	int fileCount = 0;
	std::ifstream dataFileR("/usd/DATA.1239e");
	// If file exist, read from it
	if (dataFileR.is_open()) {
		dataFileR >> fileCount;
		dataFileR.close();
	}
	// (Default is 1)
	fileCount++;
	
	std::string mclFileName = "/usd/mcl_log" + std::to_string(fileCount) + ".1239e";
	std::string rclFileName = "/usd/rcl_log" + std::to_string(fileCount) + ".1239e";
	mclLog = new std::ofstream(mclFileName);
	rclLog = new std::ofstream(rclFileName);

	std::ofstream dataFileW("/usd/DATA.1239e");
	dataFileW << fileCount;
	dataFileW.close();

	rclLogTimer.hardReset(10000000000);
	mclLogTimer.hardReset(10000000000);

	// Brain display (disabled)

	// Ensure odom pod is down
	odomLift.retract();

	// Auton Selection
	// startControllerAutonSelectorDisplay();
	// init_auton_selector();

	// Set Optical LED
	topOptic.set_led_pwm(100);

	MclMain.startTracking();
	RclMain.startTracking();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	
	odomLift.retract();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	startControllerRclCoordDisplay();

	// Ensure descore arms are retracted
	extendLeftArm();
	extendLeftArm();
	
	soloAWP();
}

void opcontrol() {
	
	//Actual OPcontrol
	//startControllerMatchDisplay(); 
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	// odomLift.extend();
	odomLift.retract();

	// Retract both descore arms
	extendLeftArm();
	stopTopScore();
	stopIntake();

	// RclMain.setMaxSyncPerSec(0.001);
	startControllerCoordDisplay();

	// Display FB Logo
	// startBrainFBDisplay();

	//RclMain.startTracking();
	//startMclBenchmark(0, 0, 270.0, true);

	chassis.setPose(0, 0, 270);
	RclMain.setRclPose({0, 0, 270});
	RclMain.updateBotPose(&right_rcl);
	RclMain.updateBotPose(&back_rcl);
	MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

	while (true) {
		// Update Controls
		updateTankDrive();
		updateIntake();
		updatePneumatics();

		pros::delay(20);
	}
}