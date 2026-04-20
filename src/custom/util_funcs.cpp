
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
        chassis.turnToHeading(orig_theta + 15, time/repeats/5, {}, false);
        chassis.turnToHeading(orig_theta - 15, time/repeats/5, {}, false);
        chassis.turnToHeading(orig_theta, time/repeats/5, {}, false);
        moveForward(-4, time/repeats/5, 80, 40, false);
        moveForward(4, time/repeats/5, 80, 40, false);
    }
}

// Fucntion for managing pneumatics controls
void updatePneumatics() {
    // Button X - Match load mech (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        toggleGate();
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
    // Rumble controller if hood lock is activated
    if (hoodLock) controller.rumble(".");
}

// Intake management
void updateIntake() {

    // Motor Controls ----------------------------------------------------
    enum scoringStates {INACTIVE, SCORE3, SCOREALL};
    static scoringStates midState = INACTIVE;
    static scoringStates topState = INACTIVE;
    static int currTopLeverSpeed = 100;
    static int currMidLeverSpeed = 80;

    setAutoReset(false);

    // Button R2 - Top state (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        extendLift();
        midState = INACTIVE;
        if (topState == INACTIVE) {
            topState = SCORE3;
            score(0, 3, currTopLeverSpeed);
        }
        else if (topState == SCORE3) {
            topState = SCOREALL;
            score(0, 7, currTopLeverSpeed);
        }
        else if (topState == SCOREALL) {
            topState = INACTIVE;
            midState = INACTIVE;
            resetLever();
        }
    }
    // Button R1 - Mid state (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        retractLift();
        topState = INACTIVE;
        if (midState == INACTIVE) {
            midState = SCORE3;
            score(0, 3, currMidLeverSpeed);
        }
        else if (midState == SCORE3) {
            midState = SCOREALL;
            score(0, 7, currMidLeverSpeed);
        }
        else if (midState == SCOREALL) {
            midState = INACTIVE;
            topState = INACTIVE;
            resetLever();
        }
    }
    // Button Right - Priming
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        startPriming();
    }

    if (!intake_macro_lock) {
        // Button L1 - Macro intake
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeFromMatchLoader(allianceColor);
        }
        // Button L2 - Raise mid + hold intake (Hold)
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            midState = INACTIVE;
            topState = INACTIVE;
            extendLift();
            if (getLeverStage() != PRIMING) {
                resetLever();
                startIntake();
            }
            else {
                frontMotor.move(127);
            }
        }
        // Button B - Outtake (Hold)
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            extendLift();
            startOuttake(600);
        }
        // Button A - Slow outtake (Hold)
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            extendLift();
            startOuttake(100);
        }
        // If no button is pressed, stop everything
        else {
            frontMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            stopIntake();
        }
    }
    else {
        if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            endIntakeMacro();
            intake_macro_lock = false;
            resetLever();
        }
    }

    // Update scoring speed with secondary controller
    if (partner_controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        currTopLeverSpeed = 100;
        currMidLeverSpeed = 80;
    }
    else if (partner_controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        currTopLeverSpeed = 30;
        currMidLeverSpeed = 20;
    }
    else {
        currTopLeverSpeed = 50;
        currMidLeverSpeed = 30;
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
        controller.print(1, 0, "Type: %s", AutonCollection[autonCount].Name);
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
        RclMain.updateBotPose(&left_rcl);
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
            pros::lcd::print(4, "B: %d mm L: %d mm R:%d mm", back_dist.get(), left_dist.get(), right_dist.get());
            pros::lcd::print(5, "Confs: B:%d L:%d R:%d", back_dist.get_confidence(), left_dist.get_confidence(), right_dist.get_confidence());
            pros::delay(60);
        }
    });
}

void startMcl(float x, float y, float vexTheta, bool resetFront, bool resetLeft, bool resetBack, bool resetRight) {
    // Reset Chassis and RCL
    lemlib::Pose p(x,y,vexTheta);
    chassis.setPose(x, y, vexTheta);
	RclMain.setRclPose(p);
    
    // Perform RCL Resets
    if (resetFront) RclMain.updateBotPose(&front_rcl);
    if (resetLeft) RclMain.updateBotPose(&left_rcl);
    if (resetBack) RclMain.updateBotPose(&back_rcl);
    if (resetRight) RclMain.updateBotPose(&right_rcl);

    // Reset MCL
	MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // Start MCL Tracking
    MclMain.startTracking();
}

void initLog() {
    if (mclLogType == SDCARD) {
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
        mclLog = new std::ofstream(mclFileName);
        
        std::ofstream dataFileW("/usd/DATA.1239e");
        dataFileW << fileCount;
        dataFileW.close();

        mclLogTimer.hardReset(10000000000);

        // If SD card is absent, rumble controller
        if (!mclLog->is_open()) {
            controller.rumble("-.-.");
            mclLogType = DISABLED;
        }
        else {
            mclLog->precision(8);
            MclMain.startAsyncLogger();
        }
    }
}

void savePIDValues() {
    std::ofstream PID_Write("/usd/PIDValues.1239e");
    if (PID_Write.is_open()) PID_Write << chassis.lateralPID.kP << " " << chassis.lateralPID.kI << " " << chassis.lateralPID.kD << " " << chassis.angularPID.kP << " " << chassis.angularPID.kI << " " << chassis.angularPID.kD;
    PID_Write.close();
}

// PID Tuner
void runPIDTuner() {

    float forwardAmount = 20.0;
    float turnAmount = 90.0;
    bool managingLateral = true;

    // Retrive file count
    std::ifstream dataFileR("/usd/PIDValues.1239e");
    // If file exist, read from it
    if (dataFileR.is_open()) {
        dataFileR >> chassis.lateralPID.kP >> chassis.lateralPID.kI >> chassis.lateralPID.kD >> chassis.angularPID.kP >> chassis.angularPID.kI >> chassis.angularPID.kD;
        dataFileR.close();
    }

    while (true) {
        // General Display
        pros::lcd::print(0, "Currently Managing: %s", managingLateral ? "LATERAL" : "ANGULAR");
        pros::lcd::print(2, "Lateral P: %.2f, I: %.2f, D: %.2f", chassis.lateralPID.kP, chassis.lateralPID.kI, chassis.lateralPID.kD);
        pros::lcd::print(3, "Angular P: %.2f, I: %.2f, D: %.2f", chassis.angularPID.kP, chassis.angularPID.kI, chassis.angularPID.kD);
        pros::lcd::print(4, "Forward Amount: %.2f in.", forwardAmount);
        pros::lcd::print(5, "Turn Amount: %.2f deg", turnAmount);
        pros::lcd::print(7, "Ready.");

        // Lateral Movement & PID Adjustment
        if (managingLateral) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                forwardAmount += 2.0;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
                forwardAmount -= 2.0;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                chassis.lateralPID.kP += 0.1;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                chassis.lateralPID.kP -= 0.1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                chassis.lateralPID.kI += 0.05;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                chassis.lateralPID.kI -= 0.05;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                chassis.lateralPID.kD += 0.1;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                chassis.lateralPID.kD -= 0.1;
            }
        }
        // Angular PID Adjustment
        else {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                turnAmount += 5.0;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
                turnAmount -= 5.0;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                chassis.angularPID.kP += 0.1;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                chassis.angularPID.kP -= 0.1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                chassis.angularPID.kI += 0.05;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                chassis.angularPID.kI -= 0.05;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                chassis.angularPID.kD += 0.1;
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                chassis.angularPID.kD -= 0.1;
            }
        }

        // Resets & Toggles
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            pros::lcd::print(7, "Resetting...");
            chassis.turnToPoint(0, 0, 1000, {}, false);
            chassis.moveToPoint(0, 0, 2500, {}, false);
            chassis.turnToHeading(0, 1000, {}, false);
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            managingLateral = !managingLateral;
        }

        // Movements
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            pros::lcd::print(7, "Moving to point...");
            moveForward(forwardAmount, 2000, 127, 1, false);
            savePIDValues();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            pros::lcd::print(7, "Turning to heading...");
            auto p = chassis.getPose();
            chassis.turnToHeading(p.theta+turnAmount, 1500, {}, false);
            savePIDValues();
        }

        pros::delay(50);
    }
}
void partnerControllerVibrate() {

    static bool vibrated60 = false;
    static bool vibrated30 = false;
    static bool vibrated20 = false;

    double matchTime = 105 - (pros::millis() / 1000);

    if (matchTime <= 60 && !vibrated60) {
        partner_controller.rumble(".");
        vibrated60 = true;
    }

    if (matchTime <= 30 && !vibrated30) {
        partner_controller.rumble(". .");
        vibrated30 = true;
    }

    if (matchTime <= 20 && !vibrated20) {
        partner_controller.rumble(". . .");
        vibrated20 = true;
    }
}