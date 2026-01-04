#ifndef TRACKING_UTIL_HPP
#define TRACKING_UTIL_HPP

#include <chrono>
#include <math.h>

// Timer
enum class TimeUnit { SECOND, MILLISECOND };
class Timer {
    public:
        Timer(double timeoutMs_ = 0) : timeoutMs(timeoutMs_), startTime(std::chrono::high_resolution_clock::now()) {};

        void reset() {
            startTime = std::chrono::high_resolution_clock::now();
        };

        void hardReset(double newTimeoutMs) {
            timeoutMs = newTimeoutMs;
            reset();
        };

        bool timeIsUp() const {
            return elapsedMs() > timeoutMs;
        };

        int timeLeft() const {
            double e = elapsedMs();
            return e < timeoutMs ? std::round(timeoutMs - e) : 0;
        };

        double elapsed(TimeUnit unit = TimeUnit::MILLISECOND) const {
            double ms = elapsedMs();
            return (unit == TimeUnit::SECOND) ? (ms / 1000.0) : ms;
        };
    
    private:
        double timeoutMs;
        std::chrono::high_resolution_clock::time_point startTime;

        double elapsedMs() const {
            auto dt = std::chrono::high_resolution_clock::now() - startTime;
            return std::chrono::duration<double, std::milli>(dt).count();
        };
};

// Consts
constexpr double mmToInch = 0.039370078740157;
constexpr double MAX_OBSTACLE_DURATION = 1e12;         // ms
constexpr double FIELD_HALF_LENGTH = 70.5;         // inches
constexpr double FIELD_NEG_HALF_LENGTH = -70.5;

// Util funcs
inline double botToTrig (double ang) { return (std::fmod(360.0-std::abs(90.0-ang), 360.0)); }
inline double vexToStd(double vexDegrees) {
    double rads = (90.0 - vexDegrees) * (M_PI / 180.0);
    while (rads > M_PI) rads -= 2 * M_PI;
    while (rads < -M_PI) rads += 2 * M_PI;
    return rads;
}

// Utility conversions
inline double degToRad(double deg) { return deg * M_PI / 180.0; }

#endif