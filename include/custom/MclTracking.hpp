#ifndef MCLTRACKING_HPP
#define MCLTRACKING_HPP

#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include "Tracking_Util.hpp"

// --- Configuration Constants ---
const double MAX_RANGE = 78.0;  
const double BASE_DIST_SIGMA = 2.5; 
const double HEADING_SIGMA = 0.08; 
const double PASS_THROUGH_CHANCE = 0.40; 
const int CONFIDENCE_THRESHOLD = 32; 
const int PARTICLE_COUNT = 400;

struct Pose { double x, y, theta; };
struct Circle { double x, y, radius; };
struct Line_ { Pose p1, p2; };

class MclTracking {
private:
    struct Particle {
        Pose pose;
        double weight;
    };

    std::vector<Particle> particles;
    std::mt19937 gen;
    std::vector<Circle> obstacles;
    std::vector<Line_> walls;
    std::vector<Pose> sensor_mounts;
    struct Trig { double cos_m, sin_m; };
    std::vector<Trig> mountTrigs;
    lemlib::Chassis* chassis;
    pros::Task* MclTrackingTask;
    std::vector<pros::Distance*> distance_collection;

    double intersect_line(Pose ray, Line_ wall, double max_range, double rayCos, double raySin) {
        double x1 = wall.p1.x; double y1 = wall.p1.y;
        double x2 = wall.p2.x; double y2 = wall.p2.y;
        double x3 = ray.x;     double y3 = ray.y;
        double x4 = ray.x + rayCos * max_range;
        double y4 = ray.y + raySin * max_range;

        double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (den == 0) return max_range;

        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return u * max_range;
        return max_range;
    }

    double intersect_circle(Pose ray, Circle c, double max_range, double dx, double dy) {
        // Bound check
        double x_diff = ray.x - c.x;
        double y_diff = ray.y - c.y;
        double c_temp = max_range + c.radius;
        if (x_diff * x_diff + y_diff * y_diff > c_temp * c_temp) {
            return max_range; 
        }
        // Calculations
        double fx = ray.x - c.x;
        double fy = ray.y - c.y;
        double b = 2 * (fx * dx + fy * dy);
        double val_c = (fx * fx + fy * fy) - (c.radius * c.radius);
        double discriminant = b * b - 4 * val_c;
        if (discriminant < 0) return max_range;
        discriminant = std::sqrt(discriminant);
        double t1 = (-b - discriminant) / 2;
        double t2 = (-b + discriminant) / 2;
        if (t1 >= 0 && t1 <= max_range) return t1;
        if (t2 >= 0 && t2 <= max_range) return t2;
        return max_range;
    }

public:
    MclTracking(lemlib::Chassis* chassis, std::vector<pros::Distance*> dist_collection, double start_x, double start_y, double start_vex_theta) {
        this->chassis = chassis;
        this->distance_collection = dist_collection;
        std::random_device rd;
        gen = std::mt19937(rd());
        
        double start_std_theta = vexToStd(start_vex_theta);
        std::normal_distribution<double> x_init(start_x, 2.0);
        std::normal_distribution<double> y_init(start_y, 2.0);
        std::normal_distribution<double> t_init(start_std_theta, 0.05);

        for (int i = 0; i < PARTICLE_COUNT; ++i) {
            particles.push_back({{x_init(gen), y_init(gen), t_init(gen)}, 1.0});
        }

        // Walls set to 70.5 (inner playable area)
        walls = {
            {{-70.5, -70.5}, { 70.5, -70.5}}, 
            {{ 70.5, -70.5}, { 70.5,  70.5}}, 
            {{ 70.5,  70.5}, {-70.5,  70.5}}, 
            {{-70.5,  70.5}, {-70.5, -70.5}}  
        };

        // Added centerGoal(0, 0, 5) to the map
        obstacles = {
            {0.0, 0.0, 3.0},     // Center Goal
            {-67.5, 46.5, 2.75},  {-67.5, -46.5, 2.75}, // Match loaders
            {67.5, 46.5, 2.75},   {67.5, -46.5, 2.75},
            {-21.5, 47, 4.0},  {21.5, 47, 4.0},   // Long goals
            {-21.5, -47, 4.0}, {21.5, -47, 4.0}
        };

        // Sensor mounts
        sensor_mounts = {
            // x (fwd/back), y (left/right), theta (angle sensor is pointing)
            {-4.25, -5.375, M_PI},      // Back sensor: faces West (180 degrees)
            {0.0, -4.5, -M_PI/2.0},     // Right sensor: faces South (-90 degrees)
            {0.0, 4.5, M_PI/2.0}        // Left sensor: faces North (90 degrees)
        };

        // Mount trig calculation
        for(int i = 0; i < sensor_mounts.size(); ++i) {
            mountTrigs.push_back({std::cos(sensor_mounts[i].theta), std::sin(sensor_mounts[i].theta)});
        }

    }

    void predict(double dist_traveled, double current_std_theta) {
        std::normal_distribution<double> dist_noise(0, 0.3);
        std::normal_distribution<double> theta_noise(0, 0.002);
        for (auto& p : particles) {
            double d_theta = current_std_theta - p.pose.theta;
            while (d_theta > M_PI) d_theta -= 2 * M_PI;
            while (d_theta < -M_PI) d_theta += 2 * M_PI;

            p.pose.theta += d_theta + theta_noise(gen);
            p.pose.x += std::cos(p.pose.theta) * dist_traveled + dist_noise(gen);
            p.pose.y += std::sin(p.pose.theta) * dist_traveled + dist_noise(gen);
        }
    }

    void update_weights(const std::vector<double>& sensor_readings, const std::vector<int>& confidences, double current_std_theta) {

        double robotCos = std::cos(current_std_theta);
        double robotSin = std::sin(current_std_theta);

        std::vector<double> sigmas_sq_2;
        for(int i = 0; i < sensor_readings.size(); ++i) {
            double s = BASE_DIST_SIGMA * (63.0 / (double)confidences[i]);
            sigmas_sq_2.push_back(2.0 * s * s); // Pre-square and multiply by 2
        }
        
        for (auto& p : particles) {
            double combined_prob = 1.0;

            // Pre-calcuolate particle trig
            double pCos = std::cos(p.pose.theta);
            double pSin = std::sin(p.pose.theta);

            double diff = p.pose.theta - current_std_theta;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            combined_prob *= std::exp(-(diff * diff) / (2 * HEADING_SIGMA * HEADING_SIGMA));

            for (size_t i = 0; i < sensor_readings.size(); ++i) {
                if (i >= sensor_mounts.size()) break; 
                if (confidences[i] < CONFIDENCE_THRESHOLD || sensor_readings[i] > MAX_RANGE) continue;

                double s_theta = p.pose.theta + sensor_mounts[i].theta;
                double s_x = p.pose.x + (robotCos * sensor_mounts[i].x) - (robotSin * sensor_mounts[i].y);
                double s_y = p.pose.y + (robotSin * sensor_mounts[i].x) + (robotCos * sensor_mounts[i].y);

                double p_dist = MAX_RANGE;
                bool hit_hollow = false;

                double rayCos = pCos * mountTrigs[i].cos_m - pSin * mountTrigs[i].sin_m;
                double raySin = pSin * mountTrigs[i].cos_m + pCos * mountTrigs[i].sin_m;

                for (const auto& w : walls) {
                    p_dist = std::min(p_dist, intersect_line({s_x, s_y, s_theta}, w, MAX_RANGE, rayCos, raySin));
                }
                for (const auto& c : obstacles) {
                    double d = intersect_circle({s_x, s_y, s_theta}, c, MAX_RANGE, rayCos, raySin);
                    if (d < p_dist) { p_dist = d; hit_hollow = true; }
                }

                double error = std::abs(sensor_readings[i] - p_dist);
                double prob_match = std::exp(-(error * error) / sigmas_sq_2[i]);

                if (hit_hollow && sensor_readings[i] > p_dist + 4.0) {
                    combined_prob *= (prob_match + PASS_THROUGH_CHANCE);
                } else {
                    combined_prob *= prob_match;
                }
            }
            p.weight = combined_prob + 1e-300;
        }
    }

    void resample() {
        std::vector<double> weights;
        for (const auto& p : particles) weights.push_back(p.weight);
        std::discrete_distribution<int> sampler(weights.begin(), weights.end());
        std::vector<Particle> new_gen(PARTICLE_COUNT);
        for (int i = 0; i < PARTICLE_COUNT; ++i) {
            Particle selected = particles[sampler(gen)];
            std::uniform_real_distribution<double> jitter(-0.2, 0.2);
            selected.pose.x += jitter(gen);
            selected.pose.y += jitter(gen);
            new_gen[i] = selected;
        }
        particles = new_gen;
    }

    Pose get_estimate() {
        double x = 0, y = 0, sin_sum = 0, cos_sum = 0;
        for (const auto& p : particles) { 
            x += p.pose.x; 
            y += p.pose.y; 
            sin_sum += std::sin(p.pose.theta);
            cos_sum += std::cos(p.pose.theta);
        }
        return {
            x / PARTICLE_COUNT, 
            y / PARTICLE_COUNT, 
            std::atan2(sin_sum, cos_sum) // This handles the 359/1 degree wrap-around
        };
    }

    Pose step(double dist, double vex_theta, const std::vector<double>& dists, const std::vector<int>& confs) {
        double std_theta = vexToStd(vex_theta);
        predict(dist, std_theta);
        update_weights(dists, confs, std_theta);
        resample();
        return get_estimate();
    }

    void set_pose(double x, double y, double vex_theta) {
        double std_theta = vexToStd(vex_theta);
        std::normal_distribution<double> x_dist(x, 1.0);
        std::normal_distribution<double> y_dist(y, 1.0);
        std::normal_distribution<double> t_dist(std_theta, 0.02);

        for (auto& p : particles) {
            p.pose = {x_dist(gen), y_dist(gen), t_dist(gen)};
            p.weight = 1.0;
        }
    }

    void startTracking() {
        if (MclTrackingTask == nullptr) {
            MclTrackingTask = new pros::Task([this](){

                lemlib::Pose odomLast = chassis->getPose();
                Timer t(50);   // 20 Hz update
                int minPause = 20;
                
                while (true) {
                    t.reset();

                    // Get Sensors
                    std::vector<double> dists = {distance_collection[0]->get()*mmToInch, distance_collection[1]->get()*mmToInch, distance_collection[2]->get()*mmToInch};
                    std::vector<int> confs = {distance_collection[0]->get_confidence(), distance_collection[1]->get_confidence(), distance_collection[2]->get_confidence()};

                    // Calculate displacement
                    double dx = chassis->getPose().x - odomLast.x;
                    double dy = chassis->getPose().y - odomLast.y;
                    double move_dist = std::sqrt(dx * dx + dy * dy);

                    // --- Corrected Direction Logic ---
                    // Convert current VEX heading to Standard Math Radians
                    double std_theta = vexToStd(this->chassis->getPose().theta); 
                    
                    // If moving against the heading, flip move_dist sign
                    double headX = std::cos(std_theta);
                    double headY = std::sin(std_theta);
                    if ((dx * headX + dy * headY) < 0) {
                        move_dist *= -1.0;
                    }
                    
                    // Update Filter
                    Pose rawMcl = this->step(move_dist, this->chassis->getPose().theta, dists, confs);
                    
                    // Convert MCL Result back to VEX Degrees for the LCD
                    // Standard Radians to VEX Degrees: degrees = 90 - (rads * 180 / PI)
                    double mclVexTheta = 90.0 - (rawMcl.theta * 180.0 / M_PI);
                    while (mclVexTheta < 0) mclVexTheta += 360;
                    while (mclVexTheta >= 360) mclVexTheta -= 360;

                    // Sync
                    chassis->setPose(rawMcl.x, rawMcl.y, mclVexTheta);
                    odomLast = this->chassis->getPose();
                
                    if (t.timeLeft() < minPause) pros::delay(minPause);
                    else pros::delay(t.timeLeft());
                }
            });
        }
    }

    void stopTracking() {
        if (MclTrackingTask != nullptr) { MclTrackingTask->remove(); delete MclTrackingTask; MclTrackingTask = nullptr; }
    }
};

#endif