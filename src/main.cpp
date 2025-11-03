#include "main.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    init_auton_selector();
    startControllerAutonSelectorDisplay();

    // --- Monte Carlo Localization Initialization ---
extern lemlib::Chassis chassis;

pros::Task mclTask([]() {
    lemlib::Pose prev = chassis.getPose();
    while (true) {
        lemlib::Pose cur = chassis.getPose();
        const double dx = cur.x - prev.x;          // inches, field-frame
        const double dy = cur.y - prev.y;          // inches, field-frame
        const double dth = cur.theta - prev.theta; // radians

        const double b = back_dist.get();  // mm
        const double l = left_dist.get();  // mm
        const double r = right_dist.get(); // mm

        MclMain.update(dx, dy, dth, b, l, r);

        // (optional) show estimate
        auto est = MclMain.getPoseEstimate();
        pros::lcd::print(5, "MCL  X=%.2f  Y=%.2f  Th=%.2f", est.x, est.y, est.theta);

        prev = cur;
        pros::delay(50);
    }
});
}


void disabled() {}

void competition_initialize() {}

void autonomous() {

	//Ensure odom pod is down
	odomLift.retract();

	// Ensure descore arms are retracted
	extendLeftArm();
	extendRightArm();
		skills();

	// Auton Selection
	if (runningSkills) {
		skills();
		return;
	}

	switch (allianceColor) {
		case alliance_color::RED:
			switch (autonType) {
				case autonTypes::LEFT:
					red_left();
					return;
				case autonTypes::RIGHT_NOMID:
					red_right_noScoreMid();
					return;
				case autonTypes::RIGHT_WMID:
					red_right_scoreMid();
					return;
				case autonTypes::SOLO_AWP:
					red_soloAWP();
					return;
				default:
					red_left();
					return;
			}
			return;
		case alliance_color::BLUE:
			switch (autonType) {
				case autonTypes::LEFT:
					blue_left();
					return;
				case autonTypes::RIGHT_NOMID:
					blue_right_noScoreMid();
					return;
				case autonTypes::RIGHT_WMID:
					blue_right_scoreMid();
					return;
				case autonTypes::SOLO_AWP:
					blue_soloAWP();
					return;
				default:
					blue_left();
					return;
			}
			return;
		default:
			blue_soloAWP();
			return;
	}	


}

void opcontrol() {
	
	startControllerDisplay();

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