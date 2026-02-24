#ifndef MCLTRACKING_HPP
#define MCLTRACKING_HPP

#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include <vector>
#include <cmath>
#include <random>
#include "Tracking_Util.hpp"
#include "pros/rotation.hpp"

// --- Configuration Constants ---
const int PARTICLE_COUNT = 700;
const int RESAMPLE_THRESHOLD = 150;
const float MIN_DIST_FROM_RESAMPLE = 6.0;
const float MAX_VELO_RESAMPLE = 100.0;
const int LOG_AMOUNT = 10;
const int LOG_RATIO = PARTICLE_COUNT / LOG_AMOUNT;

const float MAX_RANGE = 300.0;
const float BASE_DIST_SIGMA_L787 = 0.8;    // 0 ~ 200 mm
const float BASE_DIST_SIGMA_G787 = 1.5;    // > 200 mm
const float HEADING_SIGMA = 0.04;
const float DIST_RESAMPLE_VARIANCE = 2.0;
const float THETA_RESAMPLE_VARIANCE = 0.02;
const int CONFIDENCE_THRESHOLD = 40;
const float TRACKING_WHEEL_VARIANCE = 0.05;
const float FAULT_TOLERANCE = 1e-3;
const float UNCERTAINTY_TOLERANCE = 1.0;
const float DIST_SYNC_PROP = 0.1;
const float THETA_SYNC_PROP = 0.001;

const float HORIZONTAL_DRIFT_PROP = 0.5;

const float MSPT = 20;
const float MINPAUSE = 10;

struct Pose { float x, y, theta; };
struct Circle { float x, y, radius; };
struct Line_ { Pose p1, p2; };

inline float roundTwoPlaces(int x);

class MclTracking {
private:
    struct Particle {
        Pose pose;
        float weight;
    };

    // walls
    static constexpr Line_ walls[] = {
        {{-70.5, -70.5}, { 70.5, -70.5}}, 
        {{ 70.5, -70.5}, { 70.5,  70.5}}, 
        {{ 70.5,  70.5}, {-70.5,  70.5}}, 
        {{-70.5,  70.5}, {-70.5, -70.5}}
    };

    // Solid line obstacles (Middle goal posts)
    static constexpr Line_ solid_line_obstacles[] = {
        {{0.3068843, -2.711047}, {2.711047, -0.3068843}},
        {{-2.711047, 0.3068843}, {-0.3068843, 2.711047}}
    };

    // See-through line obstacles
    static constexpr Line_ see_through_line_obstacles[] = {
        {{-21, 47}, {-21.7955, 47.7955}},
        {{-21, 47}, {-21.7955, 46.2045}},
        {{21, 47}, {21.7955, 47.7955}},
        {{21, 47}, {21.7955, 46.2045}},
        {{-21, -47}, {-21.7955, -46.2045}},
        {{-21, -47}, {-21.7955, -47.7955}},
        {{21, -47}, {21.7955, -46.2045}},
        {{21, -47}, {21.7955, -47.7955}}
    };

    // Circle obstacles (Match loaders)
    static constexpr Circle circle_obstacles[] = {
        {-67.5, 46.5, 2.5},  {-67.5, -46.5, 2.5}, // Match loaders
        {67.5, 46.5, 2.5},   {67.5, -46.5, 2.5}
    };

    // Sensor mounts
    static constexpr int SENSOR_COUNT = 6;
    static constexpr Pose sensor_mounts[] = {
        // x (fwd/back), y (left/right), theta (angle sensor is pointing)
        {0, 0, std::numbers::pi},       // back
        {0, 0, std::numbers::pi*3/2},   // right
        {0, 0, std::numbers::pi/2},     // left front
        {0, 0, std::numbers::pi/2},     // left back
        {0, 0, 0.0},                    // front left
        {0, 0, 0.0}                     // front right
    };
    
    struct Trig { float cos_m, sin_m; };

    // Particles
    std::array<Particle, PARTICLE_COUNT> particles_array;
    std::array<Particle, PARTICLE_COUNT> new_gen_array;
    std::array<Particle, PARTICLE_COUNT>* particles_ptr = &particles_array;
    std::array<Particle, PARTICLE_COUNT>* new_gen_ptr = &new_gen_array;
    std::array<Trig, PARTICLE_COUNT> pTrigs = {};
    std::mt19937 gen;
    std::vector<Trig> mountTrigs;
    lemlib::Chassis* chassis;
    pros::MotorGroup* leftMotorGroup;
    pros::MotorGroup* rightMotorGroup;
    bool vertical_tracking_mode;
    pros::Task* MclTrackingTask;
    std::array<pros::Distance*, SENSOR_COUNT> distance_collection;
    bool autoSync = false;
    float lastTheta = 0.0;
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

    float vertical_drift = 0.0;
    float horizontal_drift = 0.0;

    float intersect_line(Pose ray, Line_ wall, float max_range, float rayCos, float raySin);

    float intersect_circle(Pose ray, Circle c, float max_range, float dx, float dy);

public:
    MclTracking(lemlib::Chassis* chassis, pros::MotorGroup* leftMotorGroup, pros::MotorGroup* rightMotorGroup, std::array<pros::Distance*, SENSOR_COUNT> dist_collection, std::tuple<pros::Rotation*, float, float> vertical_tracking_wheel, std::tuple<pros::Rotation*, float, float> horizontal_tracking_wheel, float start_x, float start_y, float start_vex_theta, bool autoSync_ = true);

    // Update particles and pTrigs
    void predict(float current_std_theta);

    void update_weights(const std::vector<float>& sensor_readings, const std::vector<int>& confidences, float current_std_theta);

    void resample();

    std::pair<Pose, float> get_estimate();

    Pose step(float vex_theta, const std::vector<float>& dists, const std::vector<int>& confs);

    void set_pose(float x, float y, float vex_theta);

    Pose updateMcl();

    void updateBotPose();

    void startTracking();

    void stopTracking();

    void logMcl();

    void uniform_reset();

    void setDrift(float verticalDrift, float horizontalDrift);

    ~MclTracking();
};

#endif