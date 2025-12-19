#pragma once

#include "custom/configs.hpp"
#include <vector>
#include <random>
#include <cmath>

// Monte Carlo Localization using 3 distance sensors (back, left, right)
// Sensor geometry matches your RCL config: offsets + facing angles.
// Units: distances in inches internally. Raw sensor inputs expected in mm.

struct MclParticle {
    double x;       // inches
    double y;       // inches
    double theta;   // radians (0..2π)
    double weight;  // normalized (sum=1)
};

class MclTracking {
public:
    // chassis pointer used to seed particles near current pose
    explicit MclTracking(lemlib::Chassis* chassisPtr, int numParticles = 100);

    // (Re)initialize particle cloud near chassis pose (spread in inches & radians)
    void initParticles(double posSpreadIn = 1.5, double angSpreadRad = 0.05);
    void initParticles(const lemlib::Pose& center, double posSpreadIn = 1.5, double angSpreadRad = 0.05);

    // Main step: apply odom delta (field-frame) and 3 distance readings (mm)
    void update(double dx_in, double dy_in, double dtheta_rad,
                double back_mm, double left_mm, double right_mm);

    // Current pose estimate (weighted mean)
    lemlib::Pose getPoseEstimate() const;

    // Tuning knobs
    void setMotionNoise(double linearIn, double rotRad);
    void setSensorNoise(double sensorIn);

private:
    // --- internals ---
    lemlib::Chassis* chassis;
    std::vector<MclParticle> particles;
    int N;

    std::default_random_engine rng;

    // Noise (tunable)
    double motionNoiseIn  = 0.40;   // inches (per update)
    double rotNoiseRad    = 0.015;  // radians (per update)
    double sensorNoiseIn  = 1.20;   // inches (per sensor reading)

    // Field bounds (±72 in is a 144" field)
    static constexpr double FIELD_HALF = 72.0;
    static constexpr double MM_TO_IN   = 0.03937007874;

    // Sensor geometry from your RCL config:
    // back_rcl(&back_dist,  5.375, -4.25, 180, 15.0);
    // right_rcl(&right_dist, 4.5,   0.0,   90,  15.0);
    // left_rcl(&left_dist,  -4.5,   0.0,   270, 15.0);
    struct SensorGeo {
        double horizOffsetIn; // +x forward (inches)
        double vertOffsetIn;  // +y left    (inches)
        double mainAngleDeg;  // sensor facing (degrees, adds to robot heading)
        // Precomputed polar form of offset
        double offsetDistIn;
        double offsetAngleDeg;
    };

    SensorGeo backS  { 5.375, -4.25, 180.0, 0, 0 };
    SensorGeo rightS { 4.5,    0.0,   90.0,  0, 0 };
    SensorGeo leftS  { -4.5,   0.0,   270.0, 0, 0 };

    // helpers
    static inline double rad2deg(double r) { return r * (180.0 / M_PI); }
    static inline double deg2rad(double d) { return d * (M_PI / 180.0); }
    double randNormal(double mean, double stddev);
    double randUniform(double lo, double hi);
    double intersection(const SensorGeo& sens, const Circle_Obstacle& circle);

    // MCL steps
    void motionUpdate(double dx_in, double dy_in, double dtheta_rad);
    void measurementUpdate(double back_in, double left_in, double right_in);
    void resample();

    // Ray cast from a sensor attached to particle pose; returns distance to walls in inches
    double simulateDistance(const MclParticle& p, const SensorGeo& sg) const;

    // Precompute sensor polar offsets
    void computeSensorPolars();
};