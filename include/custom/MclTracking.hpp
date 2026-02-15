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
const int RESAMPLE_THRESHOLD = 200;
const double MIN_DIS_FROM_RESAMPLE = 8.0;
const double MAX_VELO_RESAMPLE = 10.0;
const int LOG_AMOUNT = 10;
const int LOG_RATIO = PARTICLE_COUNT / LOG_AMOUNT;

const double MAX_RANGE = 300.0;
const double BASE_DIST_SIGMA_L787 = 1.5;    // 0 ~ 200 mm
const double BASE_DIST_SIGMA_G787 = 1.8;    // > 200 mm
const double HEADING_SIGMA = 0.04;
const double DIST_RESAMPLE_VARIANCE = 2.0;
const double DYNAMIC_DIST_VARIANCE_THRESHOLD = 40.0;
const double THETA_RESAMPLE_VARIANCE = 0.02;
const int CONFIDENCE_THRESHOLD = 40;
const double TRACKING_WHEEL_VARIANCE = 0.05;
const double FAULT_TOLERANCE = 1e-5;
const double UNCERTAINTY_TOLERANCE = 1.0;
const double DIST_SYNC_PROP = 0.1;
const double THETA_SYNC_PROP = 0.001;

const double MSPT = 20;
const double MINPAUSE = 10;

struct Pose { double x, y, theta; };
struct Circle { double x, y, radius; };
struct Line_ { Pose p1, p2; };

inline double roundTwoPlaces(int x);

class MclTracking {
private:
    struct Particle {
        Pose pose;
        double weight;
    };

    // walls
    static constexpr Line_ walls[] = {
        {{-70.5, -70.5}, { 70.5, -70.5}}, 
        {{ 70.5, -70.5}, { 70.5,  70.5}}, 
        {{ 70.5,  70.5}, {-70.5,  70.5}}, 
        {{-70.5,  70.5}, {-70.5, -70.5}}
    };

    // Solid line obstacles
    static constexpr Line_ solid_line_obstacles[] = {
        {{-6.7171, 9.1919}, {9.1919, -6.7171}},
        {{-9.1919, 6.7171}, {6.7171, -9.1919}}
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
    
    struct Trig { double cos_m, sin_m; };

    // Particles
    std::array<Particle, PARTICLE_COUNT> particles_array;
    std::array<Particle, PARTICLE_COUNT> new_gen_array;
    std::array<Particle, PARTICLE_COUNT>* particles_ptr = &particles_array;
    std::array<Particle, PARTICLE_COUNT>* new_gen_ptr = &new_gen_array;
    std::array<Trig, PARTICLE_COUNT> pTrigs = {};
    std::mt19937 gen;
    std::vector<Trig> mountTrigs;
    lemlib::Chassis* chassis;
    pros::Task* MclTrackingTask;
    std::vector<pros::Distance*> distance_collection;
    bool autoSync = false;
    double lastTheta = 0.0;
    pros::Rotation* vertical_tracking_wheel = nullptr;
    double vert_c = 0.0;
    double vert_offset = 0.0;
    double last_vertical_reading = 0.0;
    pros::Rotation* horizontal_tracking_wheel = nullptr;
    double horiz_c = 0.0;
    double horiz_offset = 0.0;
    double last_horizontal_reading = 0.0;
    Pose lastResamplePose = {0, 0, 0};
    double latest_speed = 0.0;

    lemlib::Pose odomLast = {0, 0, 0};
    Timer t = Timer(MSPT);
    int minPause = MINPAUSE;
    Pose rawMcl = {0, 0, 0};

    double intersect_line(Pose ray, Line_ wall, double max_range, double rayCos, double raySin);

    double intersect_circle(Pose ray, Circle c, double max_range, double dx, double dy);

public:
    MclTracking(lemlib::Chassis* chassis, std::vector<pros::Distance*> dist_collection, std::tuple<pros::Rotation*, double, double> vertical_tracking_wheel, std::tuple<pros::Rotation*, double, double> horizontal_tracking_wheel, double start_x, double start_y, double start_vex_theta, bool autoSync_ = false);

    // Update particles and pTrigs
    void predict(double current_std_theta);

    void update_weights(const std::vector<double>& sensor_readings, const std::vector<int>& confidences, double current_std_theta);

    void resample();

    std::pair<Pose, double> get_estimate();

    Pose step(double vex_theta, const std::vector<double>& dists, const std::vector<int>& confs);

    void set_pose(double x, double y, double vex_theta);

    Pose updateMcl();

    void updateBotPose();

    void startTracking();

    void stopTracking();

    void logMcl();

    void uniform_reset();

    ~MclTracking();
};

#endif