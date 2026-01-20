#include "configs.hpp"
#include "custom/PID.hpp"
#include <queue>
#include <tuple>

enum LEVER_STAGE {INACTIVE, INTAKING, SCORING};

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
std::array<int, 3> midDistReading = {100, 50, 20};
std::array<int, 4> topDistReading = {150, 100, 50, 20};
const int INCREMENT_THRESHOLD = 20;
const int DECREMENT_THRESHOLD = 20;

bool ballTrackingActivated = true;
bool removedFromTop = true;

// Lever PID
Lever_PID leverPID(
    &leverMotor,
    &leverSensor, // rotation sensor
    2.0, // kP
    0.0, // kI
    1.5, // kD
    1.0, // error range
    200, // error range timeout
    -127, // min speed
    127, // max speed
    true,
    &ballTrackingActivated
);
std::queue<std::function<void()>> leverTaskQueue;

// Color Detection
alliance_color getOpticColor() {
    if (335 < frontOptic.get_hue() || frontOptic.get_hue() < 25) return alliance_color::RED;
    else if (185 < frontOptic.get_hue() && frontOptic.get_hue() < 235) return alliance_color::BLUE;
    return alliance_color::NONE;
}

// Scoring Presets
inline std::array<std::tuple<int, int, int>, INTAKE_CAPACITY> scoringPresets = {
    std::make_tuple(10, -127, 127),
    std::make_tuple(20, -127, 127),
    std::make_tuple(30, -127, 127),
    std::make_tuple(40, -127, 127),
    std::make_tuple(50, -127, 127),
    std::make_tuple(60, -127, 127),
    std::make_tuple(70, -127, 127),
};

// --------------------- USER FUNCTIONS --------------------------
void initLeverControl() {
    ballTrackingTask = new pros::Task([](){
        while (true) {
            if (ballTrackingActivated) {
                bool decremented = false;
                do {
                    decremented = false;
                    // Domain of the middle distance sensor
                    if (currSize < midDistReading.size()) {
                        if (midDist.get() < midDistReading[currSize]-INCREMENT_THRESHOLD) {
                            intake(getOpticColor());
                        }
                        else if (midDist.get() > midDistReading[currSize]+DECREMENT_THRESHOLD) {
                            if (removedFromTop) removeTop(1);
                            else removeFront(1);
                            decremented = true;
                        }
                    }
                    // Domain of the top distance sensor
                    else {
                        if (topDist.get() < topDistReading[currSize-midDistReading.size()]-INCREMENT_THRESHOLD) {
                            intake(getOpticColor());
                        }
                        else if (topDist.get() > topDistReading[currSize-midDistReading.size()]+DECREMENT_THRESHOLD) {
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

void score(int count) {
    int level = std::min(count, currSize) + (INTAKE_CAPACITY-currSize) - 1;
    trapDoor.retract();
    ballTrackingActivated = false;
    removedFromTop = true;
    leverPID.setTarget(std::get<0>(scoringPresets[level]), std::get<1>(scoringPresets[level]), std::get<2>(scoringPresets[level]));
}

void scoreColor(alliance_color color) {
    auto info = topContColor();
    if (info.first == color) {
        score(info.second);
    }
}

void scoreAll() {
    score(INTAKE_CAPACITY);
}

void stopIntake() {
    frontMotor.move(0);
    removedFromTop = true;
}

void startIntake() {
    frontMotor.move(127);
    removedFromTop = true;
}

void startOuttake() {
    frontMotor.move(-127);
    removedFromTop = false;
}

void extendLift() {
    lift.extend();
}

void retractLift() {
    lift.retract();
}