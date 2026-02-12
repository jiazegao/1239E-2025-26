#include "lever_control.hpp"
#include "configs.hpp"
#include "custom/PID.hpp"
#include <atomic>
#include <queue>

pros::Task* ballTrackingTask = nullptr;
pros::Task* leverControlTask = nullptr;

LEVER_STAGE currentStage = INACTIVE;

// Circular Array
const int INTAKE_CAPACITY = 6;
std::array<alliance_color, INTAKE_CAPACITY> intake_array;
std::atomic<int> currSize(0);
int head = 0;
int tail = -1;

// Distance Readings
std::array<int, 3> midDistPresets = {200, 120, 50}; // bottom, top1, top2
std::array<int, 5> topDistPresets = {250, 170, 90, 20, 0}; // top2/bottom, top3, top4, top5, top6
const int INCREMENT_THRESHOLD = 20;
const int DECREMENT_THRESHOLD = 30;

// Scoring presets
std::array<double, 10> midDistCumulative = {200,200,200,200,200,200,200,200,200,200};
int midDistCumulativeIndex = 0;
double midDistReading = 200.0;
std::array<double, 10> topDistCumulative = {250,250,250,250,250,250,250,250,250,250};
int topDistCumulativeIndex = 0;
double topDistReading = 250.0;
inline std::array<int, INTAKE_CAPACITY> scoringPresets = {10, 20, 30, 40, 50, 60};

// Color / Ball management
bool ballInGate = false;
const int ENGAGE_DIST = 40;  // Ball is present (mm)
const int RELEASE_DIST = 60; // Gap/No ball present (mm)
bool removedFromTop = true;

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
    count = std::min(count, currSize.load());
    head += count;
    head %= INTAKE_CAPACITY;
    currSize -= count;
}
void removeFront(int count) {
    count = std::min(count, currSize.load());
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

// Lever PID
Lever_PID leverPID(
    &leverMotor,
    &leverSensor, // rotation sensor
    2.0, // kP
    0.0, // kI
    1.5, // kD
    2.0, // error range
    150, // error range timeout
    -127, // min speed
    127, // max speed
    true,
    &currentStage
);

// Color Detection
std::queue<alliance_color> colorSequence;
alliance_color getOpticColor() {
    if (330 < frontOptic.get_hue() || frontOptic.get_hue() < 30) return alliance_color::RED;
    else if (170 < frontOptic.get_hue() && frontOptic.get_hue() < 250) return alliance_color::BLUE;
    return alliance_color::NONE;
}

// --------------------- USER FUNCTIONS --------------------------
void initLeverControl() {
    ballTrackingTask = new pros::Task([](){

        Timer incrementCoolDown(100);
        Timer decrementCoolDown(100);

        Timer firstIntakeTimeout(1000);

        auto handleIntake = [&]() {
            // Intake the previous / latest color
            if (!colorSequence.empty()) {
                intake(colorSequence.front());
                colorSequence.pop();
            }
            else intake(allianceColor);
            // Cooldown
            decrementCoolDown.reset();
        };
        auto handleOuttake = [&]() {
            // Determine output direction based on latest actions
            removedFromTop ? removeTop(1) : removeFront(1);
            // Cooldown
            incrementCoolDown.reset();
        };

        while (true) {
            if (currentStage != SCORING) {

                int currentDist = lowDist.get_distance();
                alliance_color opticColor = getOpticColor();

                // Add balls to the queue
                if (currentDist < ENGAGE_DIST && !ballInGate && colorSequence.size() < 2) {
                    // A ball just entered the gate
                    ballInGate = true;
                    // Record the color
                    colorSequence.push(opticColor);
                } 
                else if (currentDist > RELEASE_DIST && ballInGate) {
                    // The ball has cleared the gate, ready for the next one
                    ballInGate = false;
                }

                // Update cumulative values
                double currMid = midDist.get();
                midDistReading += (currMid-midDistCumulative[midDistCumulativeIndex])/midDistCumulative.size();
                midDistCumulative[midDistCumulativeIndex] = currMid;
                midDistCumulativeIndex = (midDistCumulativeIndex+1)%midDistCumulative.size();
                    
                double currTop = topDist.get();
                topDistReading += (currTop-topDistCumulative[topDistCumulativeIndex])/topDistCumulative.size();
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
            pros::delay(20);
        }
    });

    leverControlTask = new pros::Task([](){
        leverPID.mainloop();
    });
}

void stopIntake() {
    frontMotor.move(0);
    if (currentStage != SCORING) currentStage = INACTIVE;
    removedFromTop = true;
}

void startIntake() {
    if (currentStage != SCORING) {
        frontMotor.move(127);
        currentStage = INTAKING;
        removedFromTop = true;
    }
}

void startOuttake() {
    if (currentStage != SCORING) {
        frontMotor.move(-127);
        currentStage = OUTTAKING;
        removedFromTop = false;
    }
}

void extendLift() {
    if (currentStage != SCORING) lift.extend();
}

void retractLift() {
    if (currentStage != SCORING) lift.retract();
}

void score(int timeOut, int count, int maxScoringSpeed) {
    int level = std::min(count, currSize.load()) + (INTAKE_CAPACITY-currSize) - 1;
    stopIntake();
    trapDoor.extend();  // open trapdoor

    leverPID.setTarget(scoringPresets[level], -400, maxScoringSpeed);

    removedFromTop = true;
    currentStage = SCORING;
    pros::delay(timeOut);
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

void intakeFromMatchLoader(alliance_color color) {
    pros::Task ([color](){
        // Get balls with wrong color
        startIntake();
        Timer t(2000);
        while (frontColor() != color && !t.timeIsUp()) {pros::delay(20);}
        // Discard balls with the wrong color and get balls with the right color
        stopIntake();
        score(5, std::max(0, currSize-1), FAST_TOP_SCORE);
        while (currentStage == SCORING) {pros::delay(20);}
        startIntake();
        pros::delay(500);
        stopIntake();
    });
}