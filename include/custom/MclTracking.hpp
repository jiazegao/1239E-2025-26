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
const int PARTICLE_COUNT = 1000;
const float INV_PARTICLE_COUNT = 1.0f / PARTICLE_COUNT;
const int RESAMPLE_THRESHOLD = 300;
const float MIN_DIST_FROM_RESAMPLE = 6.0f;
const float MAX_VELO_RESAMPLE = 100.0f;
const int LOG_AMOUNT = 10;
const int LOG_RATIO = PARTICLE_COUNT / LOG_AMOUNT;

const float MAX_RANGE = 300.0f;
const float BASE_DIST_SIGMA_L787 = 0.6f;    // 0 ~ 200 mm
const float BASE_DIST_SIGMA_G787 = 1.2f;    // > 200 mm
const float HEADING_SIGMA = 0.04f;
const float DIST_RESAMPLE_VARIANCE = 2.0f;
const float THETA_RESAMPLE_VARIANCE = 0.02f;
const int CONFIDENCE_THRESHOLD = 40;
const float TRACKING_WHEEL_VARIANCE = 0.05f;
const float FAULT_TOLERANCE = 1e-4f;
inline float DIST_SYNC_PROP = 0.1f;
const float THETA_SYNC_PROP = 0.001f;

const float MSPT = 20.0f;
const float INV_MSPT = 1.0f / MSPT;
const float MINPAUSE = 10.0f;

// Likelihood map
static constexpr int MAP_RES = 288; // 144 inches * 2 samples per inch
static constexpr float MAP_SCALE = 2.0f; // samples per inch
static constexpr float MAP_OFFSET = 72.0f; // field center offset
static constexpr float GAUSSIAN_SIGMA = 1.5f; // "Blur" width in inches
static constexpr float LIKELIHOOD_RANGE = 10.0f; // maximum differentiation of 10.0 inches from an object
static const float DIST_MULTIPLIER = 255.0f / std::sqrt(LIKELIHOOD_RANGE);
static const float INV_DIST_MULTIPLIER = 1 / DIST_MULTIPLIER;

struct Pose { float x, y, theta; };
struct Circle { float x, y, radius; };
struct Line_ { Pose p1, p2; };

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

    // Line obstacles
    static constexpr Line_ line_obstacles[] = {
        // Middle goal
        {{-6.7171, 9.1919}, {9.1919, -6.7171}},
        {{-9.1919, 6.7171}, {6.7171, -9.1919}},
        // Long goal legs
        {{-21, 47}, {-21.7955, 47.7955}},
        {{-21, 47}, {-21.7955, 46.2045}},
        {{21, 47}, {21.7955, 47.7955}},
        {{21, 47}, {21.7955, 46.2045}},
        {{-21, -47}, {-21.7955, -46.2045}},
        {{-21, -47}, {-21.7955, -47.7955}},
        {{21, -47}, {21.7955, -46.2045}},
        {{21, -47}, {21.7955, -47.7955}}
    };

    // Circle obstacles
    static constexpr Circle circle_obstacles[] = {
        {-67.5, 46.5, 2.75},  {-67.5, -46.5, 2.75}, // Match loaders
        {67.5, 46.5, 2.75},   {67.5, -46.5, 2.75}
    };

    // Sensor mounts
    static constexpr int SENSOR_COUNT = 3;
    static constexpr Pose sensor_mounts[] = {
        // x (fwd/back), y (left/right), theta (angle sensor is pointing)
        {-4.25, -5.375, M_PI},      // Back sensor: faces West (180 degrees)
        {0.0, -4.5, -M_PI/2.0},     // Right sensor: faces South (-90 degrees)
        {0.0, 4.5, M_PI/2.0}        // Left sensor: faces North (90 degrees)
    };
    
    struct Trig { float cos_m, sin_m; };

    // Likelihood Map Constants
    static constexpr int MAP_RES = 288;
    static constexpr float MAP_SCALE = 2.0f; 
    static constexpr float MAP_OFFSET = 72.0f;
    static constexpr float GAUSSIAN_SIGMA = 1.5f;

    // 81KB Map
    uint8_t likelihood_map[MAP_RES * MAP_RES];

    // Gaussian cheatsheeet for dynamic sigma
    float gaussian_lut[1024]; 

    void generate_likelihood_map();
    float get_dist_to_segment(float px, float py, Line_ seg);

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
    std::vector<pros::Distance*> distance_collection;
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

    static constexpr int NOISE_POOL_SIZE = 4001;
    std::array<float, NOISE_POOL_SIZE> noise_pool;
    int noise_idx = 0;

    // Helper to get next noise value
    inline float next_noise() {
        noise_idx = (noise_idx + 1) % NOISE_POOL_SIZE;
        return noise_pool[noise_idx];
    }

    float vertical_drift = 0.0;
    float horizontal_drift = 0.0;

    float intersect_line(Pose ray, Line_ wall, float max_range, float rayCos, float raySin);

    float intersect_circle(Pose ray, Circle c, float max_range, float dx, float dy);

public:
    MclTracking(lemlib::Chassis* chassis, pros::MotorGroup* leftMotorGroup, pros::MotorGroup* rightMotorGroup, std::vector<pros::Distance*> dist_collection, std::tuple<pros::Rotation*, float, float> vertical_tracking_wheel, std::tuple<pros::Rotation*, float, float> horizontal_tracking_wheel, float start_x, float start_y, float start_vex_theta, bool autoSync_ = true);

    // Update particles and pTrigs
    void predict(float current_std_theta);

    void update_weights(const std::vector<float>& readings, const std::vector<int>& confs);

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