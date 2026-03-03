
#include "custom/util_funcs.hpp"

#include "RclTracking.hpp"
#include "configs.hpp"
#include "custom/auton_selector.hpp"
#include <cmath>
#include <numbers>

#include "MclTracking.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "lever_control.hpp"

// Pneumatics functions
void openGate() {
    matchLoadGate.extend();
};
void closeGate() {
    matchLoadGate.retract();
};
void extendLeftArm() {
    leftDescoreArm.extend();
};
void retractLeftArm() {
    leftDescoreArm.retract();
};

// Auton functions
void moveForward(float inches, int timeout, float maxSpeed, float minSpeed, bool async) {
    chassis.moveToPoint(chassis.getPose().x+inches*std::cos(vexToStd(chassis.getPose().theta)), chassis.getPose().y+inches*std::sin(vexToStd(chassis.getPose().theta)), timeout, {.forwards=inches > 0 ? true : false, .maxSpeed=maxSpeed, .minSpeed=minSpeed}, async);
}
void moveBackward(float inches, int timeout, float maxSpeed, float minSpeed,  bool async) {
    chassis.moveToPoint(chassis.getPose().x+inches*std::cos(vexToStd(chassis.getPose().theta+180)), chassis.getPose().y+inches*std::sin(vexToStd(chassis.getPose().theta+180)), timeout, {.forwards=false, .maxSpeed=maxSpeed, .minSpeed=minSpeed}, async);
}
void jiggle(int repeats, int time, float forward, float backward) {
    for (int i = 0; i < repeats; i++) {
        moveForward(forward, time/repeats*3/4, 80, 50, false);
        moveForward(-backward, time/repeats/4, 31, 30, false);
    }
}
void shake(int repeats, int time) {
    float orig_theta = chassis.getPose().theta;
    for (int i = 0; i < repeats; i++) {
        chassis.turnToHeading(orig_theta + 10, time/repeats/4, {.maxSpeed=60}, false);
        chassis.turnToHeading(orig_theta - 10, time/repeats/4, {.maxSpeed=60}, false);
        chassis.turnToHeading(orig_theta, time/repeats/4, {.maxSpeed=60}, false);
        moveForward(8, time/repeats/4, 81, 40, false);
    }
}

// Fucntion for managing pneumatics controls
void updatePneumatics() {
    // Button X - Match load mech (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        matchLoadGate.toggle();
    }
    // Button Down - Left descore arm (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        leftDescoreArm.toggle();
    }
    // Button Y - Lower lift + Hood down (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        retractLift();
        closeHood();
    }
    // Button Up - Open hood
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (!hoodLock) trapDoor.extend();
        else trapDoor.retract();
        hoodLock = !hoodLock;
    }
}

// Intake management
void updateIntake() {

    // Motor Controls ----------------------------------------------------
    enum scoringStates {INACTIVE, SCORE3, SCOREALL};
    static scoringStates midState = INACTIVE;
    static scoringStates topState = INACTIVE;

    setAutoReset(false);

    // Button B - Outtake (Hold)
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        extendLift();
        startOuttake();
    }
    // Button A - Slow outtake (Hold)
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        extendLift();
        frontMotor.move(-30);
    }
    // Button R2 - Top state (Toggle)
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        extendLift();
        midState = INACTIVE;
        if (topState == INACTIVE) {
            topState = SCORE3;
            score(0, 3, FAST_TOP_SCORE);
        }
        else if (topState == SCORE3) {
            topState = SCOREALL;
            score(0, 7, FAST_TOP_SCORE);
        }
        else if (topState == SCOREALL) {
            topState = INACTIVE;
            resetLever();
        }
    }
    // Button R1 - Mid state (Toggle)
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        retractLift();
        topState = INACTIVE;
        if (midState == INACTIVE) {
            midState = SCORE3;
            score(0, 3, FAST_MID_SCORE);
        }
        else if (midState == SCORE3) {
            midState = SCOREALL;
            score(0, 7, FAST_MID_SCORE);
        }
        else if (midState == SCOREALL) {
            midState = INACTIVE;
            resetLever();
        }
    }
    // Button L2 - Raise mid + hold intake (Hold)
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        extendLift();
        startIntake();
    }
    // Button L1 - Macro intake
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        intakeFromMatchLoader();
    }
    // If no button is pressed, stop everything
    else {
        frontMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        stopIntake();
    }
}

// Tank drive
void updateTankDrive() { chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); }

// Display
void initBrainDisplay() {
    if (brainDisplayTask == nullptr) {
        brainDisplayTask = new pros::Task ([]() {
            while (true) {
                if (brainDisplayFunc != nullptr) brainDisplayFunc();
                pros::delay(brainDisplayDelay);
            }
        });
    }
};
void initControllerDisplay() {
    if (controllerDisplayTask == nullptr) {
        controllerDisplayTask = new pros::Task ([]() {
            while (true) {
                if (controllerDisplayFunc != nullptr) controllerDisplayFunc();
                pros::delay(controllerDisplayDelay);
            }
        });
    }
};

void stopBrainDisplay() {
    brainDisplayFunc = nullptr;
}
void stopControllerDisplay() {
    controllerDisplayFunc = nullptr;
}

void startControllerCoordDisplay() {
    controllerDisplayFunc = [](){
        controller.clear();
        pros::delay(50);
        controller.print(0, 0, "X: %f", chassis.getPose().x);
        pros::delay(50);
        controller.print(1, 0, "Y: %f", chassis.getPose().y);
        pros::delay(50);
        controller.print(2, 0, "Heading: %f", chassis.getPose().theta);
    };
};
void startControllerAutonSelectorDisplay() {
    controllerDisplayFunc = [](){
        controller.clear();
        pros::delay(50);
        controller.print(0, 0, "Color: %s", allianceColor == alliance_color::RED ? "RED" : "BLUE");
        pros::delay(50);
        controller.print(1, 0, "Type: %s", autonType == autonTypes::NAAUTO ? "NA_AUTO" : autonType == autonTypes::LEFT ? "LEFT" : autonType == autonTypes::LEFT_RUSH ? "LEFT_RUSH" : autonType == autonTypes::LEFT_FAST ? "LEFT_FAST" : autonType == autonTypes::LEFT_V2 ? "LEFT_V2" : autonType == autonTypes::RIGHT ? "RIGHT" : autonType == autonTypes::RIGHT_RUSH ? "RIGHT_RUSH" : autonType == autonTypes::RIGHT_FAST ? "RIGHT_FAST" : autonType == autonTypes::RIGHT_V2 ? "RIGHT_V2" : autonType == autonTypes::SOLO_AWP ? "SOLO_AWP" : "NULL");
        pros::delay(50);
        controller.print(2, 0, "Skills: %s", runningSkills ? "YES" : "NO");
    };
};

void startBrainCoordDisplay() {
    brainDisplayFunc = [](){
        pros::lcd::print(0, 0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, 0, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, 0, "Heading: %f", chassis.getPose().theta);
    };
};

void startBrainMotorInfoDisplay() {
    brainDisplayFunc = [](){
        for (int i = 0; i < 3; i++) {
            pros::lcd::print(i, 0, "LMotor%d: %f", i+1, leftMotors.get_position_all()[i]);
			pros::lcd::print(i+3, 0, "RMotor%d: %f", i+4, rightMotors.get_position_all()[i]);
        }
    };
};

void startBrainFBDisplay() {
    static lv_obj_t* image = lv_image_create(lv_screen_active());
    lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
    lv_image_set_src(image, &FB_Logo);
};

// Test Functions
void startControllerOpticDisplay() {
    controllerDisplayFunc = [](){
        controller.clear();
        pros::delay(50);
        controller.print(0, 0, "Hue: %f", frontOptic.get_hue());
        pros::delay(50);
        controller.print(1, 0, "Prox: %d", frontOptic.get_proximity());
    };
};
void startControllerRCLInfoDisplay() {
    controllerDisplayFunc = [](){
        controller.clear();
        pros::delay(50);
        controller.print(0, 0, "X: %.1f, %.1f", chassis.getPose().x, RclMain.getRclPose().x);
        pros::delay(50);
        controller.print(1, 0, "Y: %.1f, %.1f", chassis.getPose().y, RclMain.getRclPose().y);
        pros::delay(50);
        controller.print(2, 0, "Heading: %.1f, %.1f", chassis.getPose().theta, RclMain.getRclPose().theta);
    };
};


// Mcl Benchmark with Heading Conversion for LCD
inline Pose rawMcl = {0,0,0};
inline Timer MclT(15);
inline float MclRate = 0.0;
inline float MclComputeTime = 0.0;

static pros::Task* mclTask = nullptr;
static pros::Task* displayTask = nullptr;

void startMclBenchmark(float x, float y, float theta, float autoReset) {

    if (mclTask != nullptr) return;

    // Initial Sync
    chassis.setPose(x, y, theta);
    if (autoReset) {
        RclMain.updateBotPose(&right_rcl);
        RclMain.updateBotPose(&back_rcl);
    }
    lemlib::Pose odomLast = chassis.getPose();
    MclMain.set_pose(odomLast.x, odomLast.y, odomLast.theta);

    stopBrainDisplay();
    stopControllerDisplay();

    mclTask = new pros::Task([](){
        while (true) {
            MclT.reset();
            rawMcl = MclMain.updateMcl();
            // Sync On Demand
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                MclMain.updateBotPose();
            }
            MclComputeTime = MclT.elapsed();
            pros::delay(10);
            MclRate = 1000.0 / (MclT.elapsed() + 1e-10);
        }
    });

    displayTask = new pros::Task([](){
        while (true) {
            lemlib::Pose odomLast = chassis.getPose();
            lemlib::Pose RclPose = RclMain.getRclPose();
            // Display Stats
            float mclTheta = 90.0 - (rawMcl.theta * 180.0 / M_PI);
            pros::lcd::print(0, "Mcl Rate: %.1f Hz, %.4f ms", MclRate, MclComputeTime);
            pros::lcd::print(1, "MclPos: X:%.1f Y:%.1f T:%.1f", rawMcl.x, rawMcl.y, mclTheta);
            pros::lcd::print(2, "OdomPos: X:%.1f Y:%.1f T:%.1f", odomLast.x, odomLast.y, odomLast.theta);
            pros::lcd::print(3, "RclPos: X:%.1f Y:%.1f T:%.1f", RclPose.x, RclPose.y, RclPose.theta);
            pros::delay(60);
        }
    });
}

void startMcl(float x, float y, float vexTheta, bool resetLeft, bool resetBack, bool resetRight){
    // Reset Chassis and RCL
    lemlib::Pose p(x,y,vexTheta);
    chassis.setPose(x, y, vexTheta);
	RclMain.setRclPose(p);
    
    // Perform RCL Resets
    if (resetBack) RclMain.updateBotPose(&back_rcl);
    if (resetRight) RclMain.updateBotPose(&right_rcl);

    // Reset MCL
	MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // Start MCL Tracking
    MclMain.startTracking();
}

void initLog() {
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

    // If SD card is absent, rumble controller
    if (!mclLog->is_open()) controller.rumble("-.-.-.-");
}