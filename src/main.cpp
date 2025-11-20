#include "main.h"
#include "custom/RclTracking.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/motors.h"

void initialize() {
    chassis.calibrate();
    chassis.setPose(64, -62, 0);

    init_auton_selector();
//	startControllerAutonSelectorDisplay();
//startControllerRclDisplay();
startControllerDisplay();
    RclMain.startTracking();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	//Ensure odom pod is down
	odomLift.retract();

	// Ensure descore arms are retracted
	extendLeftArm();
	extendRightArm();

	soloSAWP();	
	
	leftControlRush();

	runAuton();

}

void opcontrol() {
	
	//startControllerDisplay();
	startControllerRclDisplay();
    chassis.setPose(-46, 0, 270);
	RclMain.setRclPose(chassis.getPose());
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	odomLift.extend();
	odomLift.extend();

	// Retract both descore arms
	extendLeftArm();
	extendRightArm();

	// Display FB Logo
	pros::Task ([](){pros::delay(100); startBrainFBDisplay();});

	while (true) {
		// Update Controls
		updateTankDrive();
		updateIntake();
		updatePneumatics();

		pros::delay(20);
	}

	
}