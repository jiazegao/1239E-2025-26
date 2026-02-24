#ifndef TRACKING_UTIL_HPP
#define TRACKING_UTIL_HPP

#include <chrono>
#include <math.h>

// Timer
enum class TimeUnit { SECOND, MILLISECOND };
class Timer {
    public:
        Timer(float timeoutMs_ = 0) : timeoutMs(timeoutMs_), startTime(std::chrono::high_resolution_clock::now()) {};

        void reset() {
            startTime = std::chrono::high_resolution_clock::now();
        };

        void hardReset(float newTimeoutMs) {
            timeoutMs = newTimeoutMs;
            reset();
        };

        bool timeIsUp() const {
            return elapsedMs() > timeoutMs;
        };

        float timeLeft() const {
            float e = elapsedMs();
            return e < timeoutMs ? (timeoutMs - e) : 0;
        };

        float elapsed(TimeUnit unit = TimeUnit::MILLISECOND) const {
            float ms = elapsedMs();
            return (unit == TimeUnit::SECOND) ? (ms / 1000.0) : ms;
        };
    
    private:
        float timeoutMs;
        std::chrono::high_resolution_clock::time_point startTime;

        float elapsedMs() const {
            auto dt = std::chrono::high_resolution_clock::now() - startTime;
            return std::chrono::duration<float, std::milli>(dt).count();
        };
};

// Consts
constexpr float mmToInch = 0.039370078740157;
constexpr float MAX_OBSTACLE_DURATION = 1e12;         // ms
constexpr float FIELD_HALF_LENGTH = 70.5;         // inches
constexpr float FIELD_NEG_HALF_LENGTH = -70.5;

// Util funcs
inline float botToTrig(float ang) {
    float result = 90.0 - ang;
    while (result > 360.0) result -= 360.0;
    while (result < 0) result += 360.0;
    return result;
}
inline float vexToStd(float vexDegrees) {
    float rads = (90.0 - vexDegrees) * (M_PI / 180.0);
    while (rads > M_PI) rads -= 2 * M_PI;
    while (rads < -M_PI) rads += 2 * M_PI;
    return rads;
}
inline float stdToVex(float stdRads) {
    float deg = 90.0 - (stdRads * 180.0 / M_PI);
    while (deg < 0) deg += 360;
    while (deg >= 360) deg -= 360;
    return deg;
}

// Utility conversions
inline float degToRad(float deg) { return deg * M_PI / 180.0; }

#endif