#ifndef MCLTRACKING_HPP
#define MCLTRACKING_HPP

#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "main.h"
#include <vector>
#include <cmath>
#include <random>
#include "Tracking_Util.hpp"
#include "pros/rotation.hpp"

struct Pose { float x, y, theta; };
struct Coord {float x, y; };
struct Circle { float x, y, radius; };
struct Line_ { Pose p1, p2; };
struct Trig { float cos_m, sin_m; };

class MclTracking {
private:

    // --- Configuration Constants ---
    static constexpr int PARTICLE_COUNT = 1280;
    static constexpr float INV_PARTICLE_COUNT = 1.0f / PARTICLE_COUNT;
    static constexpr int RESAMPLE_THRESHOLD = PARTICLE_COUNT / 3;
    static constexpr float MIN_DIST_FROM_RESAMPLE = 5.0f;
    static constexpr float MAX_VELO_RESAMPLE = 100.0f;
    static constexpr int LOG_AMOUNT = 1;
    static constexpr int LOG_RATIO = PARTICLE_COUNT / LOG_AMOUNT;

    static constexpr float MAX_RANGE = 100.0f;
    static constexpr float DIST_RESAMPLE_VARIANCE = 2.0f;
    static constexpr float THETA_RESAMPLE_VARIANCE = 0.02f;
    static constexpr float MAX_THETA_DEVIATION = 0.10f;
    static constexpr int CONFIDENCE_THRESHOLD = 35;
    static constexpr float CONFIDENCE_SCALING_BASE = 50.0f;
    static constexpr float RIGHT_ANG_MULTIPLIER = 2.0f;
    static constexpr float RIGHT_ANG_CONST = 2.0 / M_PI * RIGHT_ANG_MULTIPLIER;
    static constexpr float SENSOR_COUNT_SCALING = 0.25f;
    
    static constexpr float TRACKING_WHEEL_VARIANCE = 0.20f;
    static constexpr float IMU_VARIANCE = 0.01f;
    static constexpr float FAULT_TOLERANCE = 0.01f;
    float DIST_SYNC_PROP = 0.15f;
    static constexpr float THETA_SYNC_PROP = 0.001f;
    static constexpr float HORIZ_DEPENDENT_VARIANCE_PROP = 0.20f;

    static constexpr float MSPT = 15.0f;
    static constexpr float INV_MSPT = 1.0f / MSPT;
    static constexpr float MINPAUSE = 8.0f;
    
    struct Particle {
        Pose pose;
        float weight;
    };

    // Walls
    static constexpr Line_ walls[4] = {
        {{-70.2, -70.2}, { 70.2, -70.2}}, 
        {{ 70.2, -70.2}, { 70.2,  70.2}}, 
        {{ 70.2,  70.2}, {-70.2,  70.2}}, 
        {{-70.2,  70.2}, {-70.2, -70.2}}
    };

    // Line obstacles
    static constexpr Line_ goal_legs[10] = {
        // Middle goal
        {{0.400000f, -2.902659f}, {2.902659f, -0.400000f}},
        {{-2.902659f, 0.400000f}, {-0.400000f, 2.902659f}},
        // Long goal legs
        // Top left
        {{-20.7f, 47.12f}, {-22.288033f, 48.659643f}},
        {{-20.7f, 47.12f}, {-22.288033f, 45.590357f}},
        // Top right
        {{20.7f, 47.12f}, {22.288033f, 48.659643f}},
        {{20.7f, 47.12f}, {22.288033f, 45.590357f}},
        // Bottom left
        {{-20.7f, -47.12f}, {-22.288033f, -48.659643f}},
        {{-20.7f, -47.12f}, {-22.288033f, -45.590357f}},
        // Bottom right
        {{20.7f, -47.12f}, {22.288033f, -48.659643f}},
        {{20.7f, -47.12f}, {22.288033f, -45.590357f}}
    };

    // Top middle
    static constexpr Line_ top_middle = {{-10.0f, 10.0f}, {10.0f, -10.0f}};
    static constexpr Line_ low_middle = {{-10.0f, -10.0f}, {10.0f, 10.0f}};

    // Matchloaders
    static constexpr Circle match_loaders[4] = {
        {-67.635f, 46.765f, 2.00f},  {-67.635f, -46.765f, 2.00f}, // Match loaders
        {67.635f, 46.765f, 2.00f},   {67.635f, -46.765f, 2.00f}
    };

    // Sensor mounts
    static constexpr int SENSOR_COUNT = 8;
    static constexpr Pose sensor_mounts[SENSOR_COUNT] = {
        // x (fwd/back), y (left/right), theta (angle sensor is pointing)
        {6.184952f, 2.277110f, 0.0f},    // FRONT
        {-0.733924f, 2.674094f, std::numbers::pi/2}, // LEFT
        {-5.374061f, -1.75f, std::numbers::pi},   // BACK
        {-0.733924f, -2.674094f, std::numbers::pi*3/2},   // RIGHT
        {-4.092471f, 4.526609f, 0.77411928726f},    // FRONT LEFT
        {-4.992970f, -2.989211f, 2.36747336632f},    // BACK LEFT
        {-4.992970f, 2.989211f, 3.91571194086f},     // BACK RIGHT
        {-4.092471f, -4.526609f, 5.50906601991f}     // FRONT RIGHT
    };
    std::array<pros::Distance*, SENSOR_COUNT> distance_collection = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    std::array<Trig, SENSOR_COUNT> mountTrigs;
    std::array<Timer, SENSOR_COUNT> disableTimers;

    // Disabling line obstacles
    std::vector<Line_>* disabling_line_obstacles = nullptr;

    // Disabling circle obstacles
    std::vector<Circle>* disabling_circle_obstacles = nullptr;

    // Gaussian cheatsheeet for dynamic sigma
    static constexpr int GAUSSIAN_LUT_RES = 1024;
    alignas(64) float gaussian_lut[GAUSSIAN_LUT_RES];

    // Particles
    std::array<Particle, PARTICLE_COUNT> particles_array;
    std::array<Particle, PARTICLE_COUNT> new_gen_array;
    std::array<Particle, PARTICLE_COUNT>* particles_ptr = &particles_array;
    std::array<Particle, PARTICLE_COUNT>* new_gen_ptr = &new_gen_array;
    Trig currTrig;
    std::array<Trig, PARTICLE_COUNT> pTrigs = {};
    std::mt19937 gen;
    lemlib::Chassis* chassis;
    lemlib::Drivetrain* dt;
    bool vertical_tracking_mode;
    pros::Task* MclTrackingTask;
    bool autoSync = false;
    float lastImuTheta = 0.0;
    pros::Rotation* vertical_tracking_wheel = nullptr;
    float vert_c = 0.0;
    float vert_offset = 0.0;
    float last_vertical_reading = 0.0;
    pros::Rotation* horizontal_tracking_wheel = nullptr;
    float horiz_c = 0.0;
    float horiz_offset = 0.0;
    float last_horizontal_reading = 0.0;
    Pose lastResamplePose = {0, 0, 0};
    float latest_speed = 0.0;

    lemlib::Pose odomLast = {0, 0, 0};
    Timer t = Timer(MSPT);
    int minPause = MINPAUSE;
    Pose rawMcl = {0, 0, 0};

    // Sensor data access
    std::array<bool, SENSOR_COUNT> valid_sensors;
    std::array<int, SENSOR_COUNT> sensor_readings_mm;
    std::array<float, SENSOR_COUNT> sensor_readings_inch;
    std::array<int, SENSOR_COUNT> sensor_confs;
    std::array<bool, SENSOR_COUNT> disabled_sensors = {0, 0, 0, 0, 0, 0, 0, 0};

    static constexpr int NOISE_POOL_SIZE = 2048;
    static constexpr int NOISE_MASK = NOISE_POOL_SIZE - 1;

    alignas(64) std::array<float, NOISE_POOL_SIZE> noise_pool;
    std::normal_distribution<float> dist;
    int noise_idx = 0;
    int regen_idx = 0;
    static constexpr int REGEN_PT = 3;

    // Helper to get next noise value
    inline float next_noise() {
        noise_idx = (noise_idx + 123) & NOISE_MASK;
        return noise_pool[noise_idx];
    }

    inline void regen_noise() {
        noise_pool[regen_idx] = dist(gen);
        regen_idx = (regen_idx + 1) & NOISE_MASK;
    }

    float vertical_drift = 0.0;
    float horizontal_drift = 0.0;

    float intersect_line(Coord ray, Line_ wall, float max_range, float rayCos, float raySin);

    float intersect_circle(Coord ray, Circle c, float max_range, float dx, float dy);

public:
    MclTracking(lemlib::Chassis* chassis, lemlib::Drivetrain* dt, std::array<pros::Distance*, SENSOR_COUNT> dist_collection, std::tuple<pros::Rotation*, float, float> vertical_tracking_wheel, std::tuple<pros::Rotation*, float, float> horizontal_tracking_wheel, float start_x, float start_y, float start_vex_theta, bool autoSync_ = true);

    // Update particles and pTrigs
    void predict();

    void update_weights();

    void resample();

    std::pair<Pose, float> get_estimate();

    void set_pose(float x, float y, float vex_theta);

    Pose updateMcl();

    void updateBotPose();

    void startTracking();

    void setDistSyncProp(float newDistSyncProp);

    void stopTracking();

    void logMcl();

    void uniform_reset();

    void enableSens(int sens);

    void disableSens(int sens);

    void disableSensFor(int sens, float ms);

    void startAsyncLogger();

    void stopAsyncLogger();

    void setObstacles(std::vector<Line_>* newLineObstaclesPtr = nullptr, std::vector<Circle>* newCirleObstaclesPtr = nullptr);

    void setDrift(float verticalDrift, float horizontalDrift);

    float getDTWheelDegrees();

    Pose getRawMcl();

    ~MclTracking();
};

#endif