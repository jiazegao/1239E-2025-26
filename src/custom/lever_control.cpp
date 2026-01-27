#include "configs.hpp"
#include "custom/PID.hpp"
#include <tuple>

pros::Task* ballTrackingTask = nullptr;
pros::Task* leverControlTask = nullptr;

LEVER_STAGE currentStage = INACTIVE;

// Circular Array
const int INTAKE_CAPACITY = 7;
std::array<alliance_color, INTAKE_CAPACITY> intake_array;
int currSize = 0;
int head = 0;
int tail = -1;

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

// Distance Readings
std::array<int, 3> midDistReadings = {100, 50, 20};
std::array<int, 4> topDistReadings = {150, 100, 50, 20};
const int INCREMENT_THRESHOLD = 20;
const int DECREMENT_THRESHOLD = 30;

bool removedFromTop = true;

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
alliance_color getOpticColor() {
    if (335 < frontOptic.get_hue() || frontOptic.get_hue() < 25) return alliance_color::RED;
    else if (185 < frontOptic.get_hue() && frontOptic.get_hue() < 235) return alliance_color::BLUE;
    return alliance_color::NONE;
}

// Scoring Presets
inline std::array<std::tuple<int, int, int>, INTAKE_CAPACITY> scoringPresetsFast = {
    std::make_tuple(10, -127, 127),
    std::make_tuple(20, -127, 127),
    std::make_tuple(30, -127, 127),
    std::make_tuple(40, -127, 127),
    std::make_tuple(50, -127, 127),
    std::make_tuple(60, -127, 127),
    std::make_tuple(70, -127, 127)
};

inline std::array<std::tuple<int, int, int>, INTAKE_CAPACITY> scoringPresetsSlow = {
    std::make_tuple(10, -127, 60),
    std::make_tuple(20, -127, 60),
    std::make_tuple(30, -127, 60),
    std::make_tuple(40, -127, 60),
    std::make_tuple(50, -127, 60),
    std::make_tuple(60, -127, 60),
    std::make_tuple(70, -127, 60)
};

// --------------------- USER FUNCTIONS --------------------------
void initLeverControl() {
    ballTrackingTask = new pros::Task([](){
        while (true) {
            if (currentStage != SCORING) {
                bool decremented = false;
                do {
                    decremented = false;
                    // Domain of the middle distance sensor
                    if (currSize < midDistReadings.size()) {
                        if (midDist.get() < midDistReadings[currSize]-INCREMENT_THRESHOLD && getOpticColor() != alliance_color::NONE) {
                            intake(getOpticColor());
                        }
                        else if (midDist.get() > midDistReadings[currSize]+DECREMENT_THRESHOLD) {
                            if (removedFromTop) removeTop(1);
                            else removeFront(1);
                            decremented = true;
                        }
                    }
                    // Domain of the top distance sensor
                    else {
                        if (topDist.get() < topDistReadings[currSize-midDistReadings.size()]-INCREMENT_THRESHOLD && getOpticColor() != alliance_color::NONE) {
                            intake(getOpticColor());
                        }
                        else if (topDist.get() > topDistReadings[currSize-midDistReadings.size()]+DECREMENT_THRESHOLD) {
                            if (removedFromTop) removeTop(1);
                            else removeFront(1);
                            decremented = true;
                        }
                    }
                } while (decremented);
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

void score(int count, bool slowScore) {
    int level = std::min(count, currSize) + (INTAKE_CAPACITY-currSize) - 1;
    stopIntake();
    trapDoor.extend();  // open trapdoor

    if (slowScore) leverPID.setTarget(std::get<0>(scoringPresetsSlow[level]), std::get<1>(scoringPresetsSlow[level]), std::get<2>(scoringPresetsSlow[level]));
    else leverPID.setTarget(std::get<0>(scoringPresetsFast[level]), std::get<1>(scoringPresetsFast[level]), std::get<2>(scoringPresetsFast[level]));

    removedFromTop = true;
    currentStage = SCORING;
}

void scoreColor(alliance_color color, bool slowScore) {
    auto info = topContColor();
    if (info.first == color) {
        score(info.second, slowScore);
    }
}

void scoreAll(bool slowScore) {
    score(INTAKE_CAPACITY, slowScore);
}

void intakeFromMatchLoader(alliance_color color) {
    pros::Task ([&](){
        // Get balls with wrong color
        startIntake();
        Timer t(2000);
        while (frontColor() != color && !t.timeIsUp()) {pros::delay(20);}
        // Discard balls with the wrong color and get balls with the right color
        stopIntake();
        score(std::max(0, currSize-1), false);
        while (currentStage == SCORING) {pros::delay(20);}
        startIntake();
        pros::delay(500);
        stopIntake();
    });
}