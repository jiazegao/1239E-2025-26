#include "lever_control.hpp"
#include "configs.hpp"
#include <atomic>
#include <queue>
#include "custom/util_funcs.hpp"
#include "pros/abstract_motor.hpp"
#include "string"

pros::Task* ballTrackingTask = nullptr;
pros::Task* leverControlTask = nullptr;
pros::Task* frontIntakeControlTask = nullptr;

LEVER_STAGE currentStage = INACTIVE;
Timer leverScoringTimeout(4000);

// Circular Array
const int INTAKE_CAPACITY = 6;
std::array<alliance_color, INTAKE_CAPACITY> intake_array;
int currSize = 0;
int head = 0;
int tail = -1;

LEVER_STAGE getLeverStage() {
    return currentStage;
}

// Distance Readings
std::array<int, 3> midDistPresets = {313, 116, 34}; // bottom, top1, top2
std::array<int, 5> topDistPresets = {370, 270, 202, 107, 20}; // top2/bottom, top3, top4, top5, top6
const int SEVEN_THRESHOLD_MID = 20;
const int SEVEN_THRESHOLD_TOP = 18;
const int INCREMENT_THRESHOLD = 60;
const int DECREMENT_THRESHOLD = 60;

// Scoring presets
std::array<int, 8> midDistCumulative = {313,313,313,313,313,313,313,313};
int midDistCumulativeIndex = 0;
float midDistReading = 313.0;
std::array<int, 8> topDistCumulative = {370,370,370,370,370,370,370,370};
int topDistCumulativeIndex = 0;
float topDistReading = 370.0;
inline std::array<float, INTAKE_CAPACITY> scoringPresetsTop = {78.0f, 100.0f, 125.0f, 145.0f, 175.0f, 205.0f};
inline std::array<float, INTAKE_CAPACITY> scoringPresetsMid = {90.0f, 120.0f, 134.0f, 154.0f, 188.0f, 200.0f};
bool positionedForTop = true;
bool removedFromTop = false;

alliance_color topColor() { return currSize > 0 ? intake_array[head] : alliance_color::NONE; }
alliance_color frontColor() { return currSize > 0 ? intake_array[tail] : alliance_color::NONE; }
bool intake(alliance_color ballColor) {
    if (currSize < INTAKE_CAPACITY) {
        tail++;
        tail %= INTAKE_CAPACITY;
        intake_array[tail] = ballColor;
        currSize++;
        return true;
    }
    return false;
}
void removeTop(int count) {
    count = std::min(count, currSize);
    head += count;
    head %= INTAKE_CAPACITY;
    currSize -= count;
}
void removeFront(int count) {
    count = std::min(count, currSize);
    tail -= count;
    tail += INTAKE_CAPACITY;
    tail %= INTAKE_CAPACITY;
    currSize -= count;
}
std::pair<alliance_color, int> topContColor() {
    if (currSize > 0) {
        alliance_color color = topColor();
        int ptr = head;
        int elapsed = 1;
        while (elapsed < currSize && intake_array[(ptr+1)%INTAKE_CAPACITY] == color) {
            ptr++;
            ptr %= INTAKE_CAPACITY;
            elapsed++;
        }
        return std::make_pair(color, elapsed);
    }
    return std::make_pair(alliance_color::NONE, 0);
}
std::pair<alliance_color, int> frontContColor() {
    if (currSize > 0) {
        alliance_color color = frontColor();
        int ptr = tail;
        int elapsed = 1;
        while (elapsed < currSize && intake_array[(ptr-1+INTAKE_CAPACITY)%INTAKE_CAPACITY] == color) {
            ptr--;
            ptr += INTAKE_CAPACITY;
            ptr %= INTAKE_CAPACITY;
            elapsed++;
        }
        return std::make_pair(color, elapsed);
    }
    return std::make_pair(alliance_color::NONE, 0);
}

// Color Detection
alliance_color getOpticColor() {
    if (340 < frontOptic.get_hue() || frontOptic.get_hue() < 30) return alliance_color::RED;
    else if (20 < frontOptic.get_hue() && frontOptic.get_hue() < 340) return alliance_color::BLUE;
    return alliance_color::NONE;
}

// Global variables for lever control
int currTarget = 0;
int currMaxSpeed = 0;
bool autoReset = true;
int currOuttakeSpeed = 600;
bool intakeStaged = false;

bool intakeLiftKeptUp = false;

Timer intakeSwapTimer(500);
Timer afterScoreHoodCloseTimeout(500);
Timer sevenOuttakeTimeout(200);
Timer leverAntiStuckTimeout(500);

bool postPriming = false;
void startPriming() {
    if (currSize < INTAKE_CAPACITY) {
        positionedForTop ? currTarget = scoringPresetsTop[INTAKE_CAPACITY-1-currSize] : currTarget = scoringPresetsMid[INTAKE_CAPACITY-1-currSize];
    }
    else {
        currTarget = 0.0f;
    }
    currentStage = PRIMING;
    postPriming = true;
}
void endPriming() {
    trapDoor.extend();
    positionedForTop ? currTarget = scoringPresetsTop[INTAKE_CAPACITY-1] : currTarget = scoringPresetsMid[INTAKE_CAPACITY-1];
    currentStage = RAISING;
}

// --------------------- USER FUNCTIONS --------------------------
void initLeverControl() {

    intakeSwapTimer.reset();
    leverScoringTimeout.reset();
    afterScoreHoodCloseTimeout.reset();
    sevenOuttakeTimeout.reset();
    leverAntiStuckTimeout.reset();
    
    ballTrackingTask = new pros::Task([](){

        Timer incrementCoolDown(300);
        Timer decrementCoolDown(300);

        alliance_color opticColor = getOpticColor();

        auto handleIntake = [&]() {
            intake(opticColor != alliance_color::NONE ? opticColor : allianceColor);
            decrementCoolDown.reset();
            incrementCoolDown.reset();
        };
        auto handleOuttake = [&]() {
            removedFromTop ? removeTop(1) : removeFront(1);
            incrementCoolDown.reset();
            decrementCoolDown.reset();
        };

        while (true) {
            if (currentStage != RAISING && currentStage != PRIMING && currentStage != LOWERING && positionedForTop) {

                // Update readings
                opticColor = getOpticColor();

                // Update cumulative values
                int currMid = midDist.get();
                midDistReading += (float) (currMid-midDistCumulative[midDistCumulativeIndex])/midDistCumulative.size();
                midDistCumulative[midDistCumulativeIndex] = currMid;
                midDistCumulativeIndex = (midDistCumulativeIndex+1)%midDistCumulative.size();
                    
                int currTop = topDist.get();
                topDistReading += (float) (currTop-topDistCumulative[topDistCumulativeIndex])/topDistCumulative.size();
                topDistCumulative[topDistCumulativeIndex] = currTop;
                topDistCumulativeIndex = (topDistCumulativeIndex+1)%topDistCumulative.size();

                // Full middle sensor control
                if (currSize < midDistPresets.size()-1) {
                    if (incrementCoolDown.timeIsUp() && midDistReading < midDistPresets[currSize]-INCREMENT_THRESHOLD) {
                        handleIntake();
                    }
                    else if (decrementCoolDown.timeIsUp() && currSize > 0 && midDistReading > midDistPresets[currSize]+DECREMENT_THRESHOLD) {
                        handleOuttake();
                    }
                }
                // Half / Half
                else if (currSize == midDistPresets.size()-1) {
                    if (incrementCoolDown.timeIsUp() && topDistReading < topDistPresets[0]-INCREMENT_THRESHOLD) {
                        handleIntake();
                    }
                    else if (decrementCoolDown.timeIsUp() && midDistReading > midDistPresets[currSize]+DECREMENT_THRESHOLD) {
                        handleOuttake();
                    }
                }
                // Full top sensor control
                else {
                    int index = currSize-midDistPresets.size()+1;
                    if (incrementCoolDown.timeIsUp() && currSize < INTAKE_CAPACITY && topDistReading < topDistPresets[index]-INCREMENT_THRESHOLD) {
                        handleIntake();
                    }
                    else if (decrementCoolDown.timeIsUp() && topDistReading > topDistPresets[index]+DECREMENT_THRESHOLD) {
                        handleOuttake();
                    }
                }
            }
            pros::delay(25);
        }
    });

    // Bang-Bang controller
    leverControlTask = new pros::Task([](){
        while (true) {
            if (currentStage == PRIMING) {
                // Haven't reached target, keep going
                if (leverMotor.get_position() < currTarget-5.0f) {
                    leverMotor.move_velocity(5 + std::abs(leverMotor.get_position()-currTarget));
                }
                else if (leverMotor.get_position() > currTarget+5.0f){
                    leverMotor.move_velocity(-5 - std::abs(leverMotor.get_position()-currTarget));
                }
                else {
                    leverMotor.move(0);
                }
            }
            // Raising - Move upward
            if (currentStage == RAISING) {
                // Lever Anti Stuck
                if (!postPriming && lever_antistuck_on && !leverAntiStuckTimeout.timeIsUp()) {
                    frontMotor.move(127);
                }
                // Timeout
                else if (autoReset && leverScoringTimeout.timeIsUp()) {
                    afterScoreHoodCloseTimeout.reset();
                    currentStage = LOWERING;
                }
                // Haven't reached target, keep going
                else if (leverMotor.get_position() < currTarget) {
                    leverMotor.move_velocity(currMaxSpeed);
                }
                // Reached target, immediately reverse then move position
                else if (leverMotor.get_position() >= currTarget) {
                    leverMotor.move(0);
                    // If auto reset, move to the next stage
                    if (autoReset) {
                        afterScoreHoodCloseTimeout.reset();
                        currentStage = LOWERING;
                    }
                    else {
                        afterScoreHoodCloseTimeout.reset();
                        currentStage = INACTIVE;
                    }
                }
            }
            if (currentStage == LOWERING) {
                // Keep reversing until back to resting position
                if (leverMotor.get_position() > 0.5) leverMotor.move(-127);
                else {
                    leverMotor.move(0);
                    currentStage = INACTIVE;
                    if (intakeStaged) startIntake();
                }
                postPriming = false;
            }
            // Rudimentary motor control
            if (currentStage == INTAKING) {
                frontMotor.move(127);
                if (!intakeLiftKeptUp) intakeLift.extend();
                else intakeLift.retract();
                // Anti-stuck; only act if intake has been occuring for a certain amount of time
                if (antiStuckOn && std::abs(frontMotor.get_actual_velocity()) < 20 && intakeSwapTimer.timeIsUp()) {
                    frontMotor.move(-127);
                    pros::delay(150);
                    frontMotor.move(127);
                    intakeSwapTimer.reset();
                }
            }
            if (currentStage == OUTTAKING) {
                if (!fastOuttake || intakeLiftKeptUp) intakeLift.retract();
                else intakeLift.extend();

                if (!fastOuttake) frontMotor.move_velocity(-currOuttakeSpeed);
                else frontMotor.move(-currOuttakeSpeed);
            }
            if (currentStage == INACTIVE) {
                frontMotor.move(0);
            }

            if (!hoodLock && currentStage != RAISING && afterScoreHoodCloseTimeout.timeIsUp()) closeHood();

            pros::delay(30);
        }
    });
}

void stopIntake() {
    if (currentStage != RAISING && currentStage != LOWERING && currentStage != PRIMING && currentStage != INACTIVE) {
        currentStage = INACTIVE;
        intakeStaged = false;
        removedFromTop = false;
        intakeSwapTimer.reset();
    }
    if (leverAntiStuckTimeout.timeIsUp()) frontMotor.move(0);
}

void startIntake() {
    if (currentStage != INTAKING) {
        if (currentStage != RAISING && currentStage != PRIMING && currentStage != LOWERING) {
            currentStage = INTAKING;
            intakeStaged = false;
            removedFromTop = false;
            intakeSwapTimer.reset();
        }
        else {
            intakeStaged = true;
        }
    }
    frontMotor.move(127);
}

void startOuttake(int speed) {
    currOuttakeSpeed = std::abs(speed);
    if (currentStage != RAISING && currentStage != LOWERING && currentStage != PRIMING && currentStage != OUTTAKING) {
        currentStage = OUTTAKING;
        intakeStaged = false;
        removedFromTop = false;
        intakeSwapTimer.reset();
    }
    if (currentStage != OUTTAKING) {
        if (fastOuttake) frontMotor.move(-currOuttakeSpeed);
        else frontMotor.move_velocity(-currOuttakeSpeed);
    }
}

void extendLift() {
    lift.extend();
    positionedForTop = true;
}

void retractLift() {
    lift.retract();
    positionedForTop = false;
}

void openHood() {
    if (!hoodLock) trapDoor.extend();
}
void closeHood() {
    if (!hoodLock) trapDoor.retract();
}

void hardResetLever() {
    leverMotor.move(-127);
    pros::delay(1200);
    leverMotor.move(0);
    pros::delay(500);
    leverMotor.set_zero_position(0.0);
}


void intakeLiftLock(bool up) {
    intakeLiftKeptUp = up;
}

void openGate() {
    matchLoadGate.extend();
    MclMain.disableSensFor(FRONT, 500.0f);
}
void closeGate() {
    matchLoadGate.retract();
    MclMain.disableSensFor(FRONT, 500.0f);
}
void toggleGate() {
    matchLoadGate.toggle();
    MclMain.disableSensFor(FRONT, 500.0f);
}


void score(int timeOut, int count, int maxScoringSpeed) {
    int level = std::min(count, currSize) + (INTAKE_CAPACITY-currSize) - 1;
    stopIntake();
    trapDoor.extend();  // open trapdoor

    // Anti stuck
    if (lever_antistuck_on && midDist.get() <= SEVEN_THRESHOLD_MID && topDist.get() <= SEVEN_THRESHOLD_TOP) {
        leverAntiStuckTimeout.reset();
    }

    if (currentStage != PRIMING) {
        positionedForTop ? currTarget = scoringPresetsTop[level] : currTarget = scoringPresetsMid[count < INTAKE_CAPACITY ? std::max(0, level-2) : INTAKE_CAPACITY-1];
    }
    else {
        positionedForTop ? currTarget = scoringPresetsTop[INTAKE_CAPACITY-1] : currTarget = scoringPresetsMid[INTAKE_CAPACITY-1];
    }
    currMaxSpeed = maxScoringSpeed;

    leverScoringTimeout.reset();
    currentStage = RAISING;
    removedFromTop = true;

    Timer t(timeOut);
    while (currentStage == RAISING && !t.timeIsUp()) {pros::delay(20);}
}

void scoreReserve(int timeOut, int reserving, int maxScoringSpeed) {
    int level = 5 - reserving;
    if (level < 1) return;
    stopIntake();
    trapDoor.extend();  // open trapdoor

    // Anti stuck
    if (lever_antistuck_on && midDist.get() <= SEVEN_THRESHOLD_MID && topDist.get() <= SEVEN_THRESHOLD_TOP) {
        leverAntiStuckTimeout.reset();
    }

    if (currentStage != PRIMING) {
        positionedForTop ? currTarget = scoringPresetsTop[level] : currTarget = scoringPresetsMid[std::max(0, level-2)];
    }
    else {
        positionedForTop ? currTarget = scoringPresetsTop[INTAKE_CAPACITY-1] : currTarget = scoringPresetsMid[INTAKE_CAPACITY-1];
    }
    currMaxSpeed = maxScoringSpeed;

    leverScoringTimeout.reset();
    currentStage = RAISING;
    removedFromTop = true;

    Timer t(timeOut);
    while (currentStage == RAISING && !t.timeIsUp()) {pros::delay(20);}
}

void scoreColor(int timeOut, int maxScoringSpeed, alliance_color color) {
    auto info = topContColor();
    if (info.first == color) {
        score(timeOut, info.second, maxScoringSpeed);
    }
}

void scoreAll(int timeOut, int maxScoringSpeed) {
    score(timeOut, INTAKE_CAPACITY, maxScoringSpeed);
}

pros::Task* intakeMacroTask;
void intakeFromMatchLoader(alliance_color color) {
    intakeMacroTask = new pros::Task ([color](){
        intake_macro_lock= true;
        // Get balls with wrong color
        startIntake();

        Timer t(2000);
        while (getOpticColor() != color && !t.timeIsUp()) {pros::delay(20);}
        int discarding_count = currSize;

        t.reset();
        while (currSize < INTAKE_CAPACITY && !t.timeIsUp()) {pros::delay(20);}
        scoreReserve(1600, std::min(5, currSize-discarding_count+1), 20);
        
        startIntake();
        trapDoor.retract();
        resetLever();
        intake_macro_lock = false;
    });
}

void endIntakeMacro() {
    intakeMacroTask->remove();
}

void resetLever() {
    //afterScoreHoodCloseTimeout.reset();
    currentStage = LOWERING;
}

void setAutoReset(bool newConfig) {
    autoReset = newConfig;
}

void updateLeverTuningDisplay() {
    std::string intake_info = "";
    int currHead = head;
    
    // Use a simple for-loop based on the actual number of items
    for (int i = 0; i < currSize; i++) {
        if (intake_array[currHead] == alliance_color::RED) {
            intake_info += "RED ";
        }
        else if (intake_array[currHead] == alliance_color::BLUE) {
            intake_info += "BLUE ";
        }
        else {
            intake_info += "NAN ";
        }
        // Move to the next index, wrapping around if necessary
        currHead = (currHead + 1) % INTAKE_CAPACITY;
    }

    pros::lcd::print(0, "Intake: %d, %s", currSize, intake_info.c_str());


    if (currentStage == INACTIVE) {
        pros::lcd::print(1, "Current Stage: %s", "INACTIVE");
    }
    else if (currentStage == INTAKING) {
        pros::lcd::print(1, "Current Stage: %s", "INTAKING");
    }
    else if (currentStage == OUTTAKING) {
        pros::lcd::print(1, "Current Stage: %s", "OUTTAKING");
    }
    else if (currentStage == RAISING) {
        pros::lcd::print(1, "Current Stage: %s", "RAISING");
    }
    else if (currentStage == LOWERING) {
        pros::lcd::print(1, "Current Stage: %s", "LOWERING");
    }

    pros::lcd::print(2, "Hood Sens: %d mm", topDist.get());
    pros::lcd::print(3, "Intake Sens: %d mm", midDist.get());
    pros::lcd::print(4, "Optic Hue: %.2f", frontOptic.get_hue());
    pros::lcd::print(5, "Optic Proximity: %d", frontOptic.get_proximity());
    pros::lcd::print(6, "Lever IME: %f", leverMotor.get_position());
}

void startLeverTuningDisplay() {
    brainDisplayFunc = updateLeverTuningDisplay;
}