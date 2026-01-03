#include "main.h"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/auton.hpp"

#include "custom/util_funcs.hpp"
#include "custom/auton_selector.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/motors.h"
#include "pros/motors.h"

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

	//Ensure odom pod is down
	odomLift.retract();

	// Auton Selection
	//startControllerAutonSelectorDisplay();
	//init_auton_selector();
	pros::lcd::initialize();

    RclMain.startTracking();
	topOptic.set_led_pwm(100);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	startControllerRCLUpdate();

	// Ensure descore arms are retracted
	extendLeftArm();
	extendLeftArm();
	
	//Change this back later
	//skills();
	
	runAuton();
}

void opcontrol() {
	
	//startControllerMatchDisplay(); 
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	odomLift.extend();

	// Retract both descore arms
	extendLeftArm();
	extendLeftArm();
	stopTopScore();

	// Display FB Logo
	//pros::Task ([](){pros::delay(100); startBrainFBDisplay();});
	// Mcl testing
	startMclBenchmark();

	while (true) {
		// Update Controls
		updateTankDrive();
		updateIntake();
		updatePneumatics();

		pros::delay(20);
	}

	
}