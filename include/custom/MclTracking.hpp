#ifndef MCLTRACKING_HPP
#define MCLTRACKING_HPP

#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include <numbers>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include "Tracking_Util.hpp"
#include "pros/rotation.hpp"

// --- Configuration Constants ---
const double MAX_RANGE = 78.0;
const double BASE_DIST_SIGMA = 2.5;
const double DIST_RESAMPLE_VARIANCE = 1.0;
const double THETA_RESAMPLE_VARIANCE = 0.02;
const double HEADING_SIGMA = 0.03;
const int CONFIDENCE_THRESHOLD = 45; 
const int PARTICLE_COUNT = 500;
const int RESAMPLE_THRESHOLD = 50;

struct Pose { double x, y, theta; };
struct Circle { double x, y, radius; };
struct Line_ { Pose p1, p2; };

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
    double last_vertical_reading = 0.0;
    pros::Rotation* horizontal_tracking_wheel = nullptr;
    double horiz_c = 0.0;
    double last_horizontal_reading = 0.0;

    lemlib::Pose odomLast = {0, 0, 0};
    Timer t = Timer(30);
    int minPause = 15;
    Pose rawMcl = {0, 0, 0};

    double intersect_line(Pose ray, Line_ wall, double max_range, double rayCos, double raySin) {

        // Vert wall
        double xMin = std::min(wall.p1.x, wall.p2.x);
        double xMax = std::max(wall.p1.x, wall.p2.x);
        double yMin = std::min(wall.p1.y, wall.p2.y);
        double yMax = std::max(wall.p1.y, wall.p2.y);

        if ((rayCos > 0 && ray.x > xMax) ||
            (rayCos < 0 && ray.x < xMin) ||
            (raySin > 0 && ray.y > yMax) ||
            (raySin < 0 && ray.y < yMin)) return max_range;

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
    MclTracking(lemlib::Chassis* chassis, std::vector<pros::Distance*> dist_collection, std::pair<pros::Rotation*, double> vertical_tracking_wheel, std::pair<pros::Rotation*, double> horizontal_tracking_wheel, double start_x, double start_y, double start_vex_theta, bool autoSync_ = false) {
        this->chassis = chassis;
        this->distance_collection = dist_collection;
        this->autoSync = autoSync_;

        this->vertical_tracking_wheel = vertical_tracking_wheel.first;
        this->vert_c = vertical_tracking_wheel.second*std::numbers::pi;
        this->last_vertical_reading = this->vertical_tracking_wheel->get_angle();

        this->horizontal_tracking_wheel = horizontal_tracking_wheel.first;
        this->horiz_c = horizontal_tracking_wheel.second*std::numbers::pi;
        this->last_horizontal_reading = this->horizontal_tracking_wheel->get_angle();

        std::random_device rd;
        gen = std::mt19937(rd());
        
        double start_std_theta = vexToStd(start_vex_theta);
        this->lastTheta = start_std_theta;
        std::normal_distribution<double> x_init(start_x, 2.0);
        std::normal_distribution<double> y_init(start_y, 2.0);
        std::normal_distribution<double> t_init(start_std_theta, 0.05);

        particles_ptr = &particles_array;
        new_gen_ptr = &new_gen_array;

        auto& particles = *particles_ptr;
        for (int i = 0; i < PARTICLE_COUNT; ++i) {
            particles[i] = {{x_init(gen), y_init(gen), t_init(gen)}, 1.0};
        }

        // Mount trig calculation
        for(int i = 0; i < SENSOR_COUNT; ++i) {
            mountTrigs.push_back({std::cos(sensor_mounts[i].theta), std::sin(sensor_mounts[i].theta)});
        }
    }

    // Update particles and pTrigs
    void predict(double current_std_theta) {
        // Update theta deviation
        double d_theta = current_std_theta - lastTheta;
        while (d_theta > M_PI) d_theta -= 2 * M_PI;
        while (d_theta < -M_PI) d_theta += 2 * M_PI;
        lastTheta = current_std_theta;

        // Noise (Avg 0.002 rad / 6%)
        std::normal_distribution<double> theta_noise(0, 0.002);
        std::normal_distribution<double> tracking_wheel_noise(1, 0.06);

        // Calculate vertical tracking wheel vector
        double vert_reading = vertical_tracking_wheel->get_angle();
        double d_vert = (vert_reading-last_vertical_reading)/360.0 * vert_c;
        last_vertical_reading = vert_reading;

        // Calculate horizontal tracking wheel vector
        double horiz_reading = horizontal_tracking_wheel->get_angle();
        double d_horiz = (horiz_reading-last_horizontal_reading)/360.0 * horiz_c;
        last_horizontal_reading = horiz_reading;

        auto& particles = *particles_ptr;
        for (int i = 0; i < PARTICLE_COUNT; i++) {
            auto& p = particles[i];
            double pCos = std::cos(p.pose.theta + (d_theta/2));
            double pSin = std::sin(p.pose.theta + (d_theta/2));

            // Get noise
            double vert_noise = tracking_wheel_noise(gen);
            double horiz_noise = tracking_wheel_noise(gen);

            // Calculate movement vectors with noise
            std::pair<double, double> vert_vector = std::make_pair(d_vert*pCos*vert_noise, d_vert*pSin*vert_noise);
            std::pair<double, double> horiz_vector = std::make_pair(d_horiz*pSin*horiz_noise, (-1)*d_horiz*pCos*horiz_noise);   // cos(x-90) = sin(x); sin(x-90) = -cos(x)

            p.pose.x += vert_vector.first + horiz_vector.first;
            p.pose.y += vert_vector.second + horiz_vector.second;
            p.pose.theta += d_theta + theta_noise(gen);

            while (p.pose.theta > M_PI) p.pose.theta -= 2 * M_PI;
            while (p.pose.theta < -M_PI) p.pose.theta += 2 * M_PI;

            pTrigs[i] = {std::cos(p.pose.theta), std::sin(p.pose.theta)};
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

        auto& particles = *particles_ptr;
        for (int count = 0; count < PARTICLE_COUNT; count++) {
            auto& p = particles[count];
            double combined_prob = 1.0;

            // Intant penalize if out of bounds
            if (p.pose.x < -70.5 || p.pose.x > 70.5 || p.pose.y < -70.5 || p.pose.y > 70.5) {
                p.weight = 1e-300;
                continue;
            }

            double diff = p.pose.theta - current_std_theta;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            combined_prob *= std::exp(-(diff * diff) / (2 * HEADING_SIGMA * HEADING_SIGMA));

            for (size_t i = 0; i < sensor_readings.size(); ++i) {
                if (i >= SENSOR_COUNT) break; 
                if (confidences[i] < CONFIDENCE_THRESHOLD || sensor_readings[i] > MAX_RANGE) continue;

                double current_sigma_sq = sigmas_sq_2[i];

                double s_theta = p.pose.theta + sensor_mounts[i].theta;
                double s_x = p.pose.x + (pTrigs[count].cos_m * sensor_mounts[i].x) - (pTrigs[count].sin_m * sensor_mounts[i].y);
                double s_y = p.pose.y + (pTrigs[count].sin_m * sensor_mounts[i].x) + (pTrigs[count].cos_m * sensor_mounts[i].y);

                double p_dist = MAX_RANGE;
                bool hit_hollow = false;

                double rayCos = pTrigs[count].cos_m * mountTrigs[i].cos_m - pTrigs[count].sin_m  * mountTrigs[i].sin_m;
                double raySin = pTrigs[count].sin_m * mountTrigs[i].cos_m + pTrigs[count].cos_m  * mountTrigs[i].sin_m;

                for (const auto& wall : walls) {
                    p_dist = std::min(p_dist, intersect_line({s_x, s_y, s_theta}, wall, MAX_RANGE, rayCos, raySin));
                }
                for (const auto& slo : solid_line_obstacles) {
                    p_dist = std::min(p_dist, intersect_line({s_x, s_y, s_theta}, slo, MAX_RANGE, rayCos, raySin));
                }
                for (const auto& c : circle_obstacles) {
                    p_dist = std::min(p_dist, intersect_circle({s_x, s_y, s_theta}, c, MAX_RANGE, rayCos, raySin));
                }
                for (const auto& stlo : see_through_line_obstacles) {
                    double d = intersect_line({s_x, s_y, s_theta}, stlo, MAX_RANGE, rayCos, raySin);
                    if (d < p_dist) { p_dist = d; hit_hollow = true; }
                }

                if (sensor_readings[i] < (p_dist - 10.0)) {
                    continue; // Skip this sensor for this particle; don't update combined_prob
                }

                if (hit_hollow) {
                    current_sigma_sq *= 3.0;
                }

                double error = std::abs(sensor_readings[i] - p_dist);
                double prob_match = std::exp(-(error * error) / current_sigma_sq);
                
                combined_prob *= (prob_match + 0.02);
            }
            p.weight = combined_prob + 1e-300;
        }
    }

    void resample() {
        std::vector<double> weights;
        auto& particles = *particles_ptr;
        for (const auto& p : particles) weights.push_back(p.weight);

        std::discrete_distribution<int> sampler(weights.begin(), weights.end());
        std::uniform_real_distribution<double> dist_jitter(-DIST_RESAMPLE_VARIANCE, DIST_RESAMPLE_VARIANCE);
        std::uniform_real_distribution<double> theta_jitter(-THETA_RESAMPLE_VARIANCE, THETA_RESAMPLE_VARIANCE);

        for (int i = 0; i < PARTICLE_COUNT; ++i) {
            Particle selected = particles[sampler(gen)];
            selected.pose.x += dist_jitter(gen);
            selected.pose.y += dist_jitter(gen);
            selected.pose.theta += theta_jitter(gen);
            selected.weight = 1.0;
            (*new_gen_ptr)[i] = selected;
        }
        std::swap(particles_ptr, new_gen_ptr);
    }

    std::pair<Pose, double> get_estimate() {
        double x = 0, y = 0, sin_sum = 0, cos_sum = 0;
        double total_weight = 0;
        double weight_sqr_sum = 0;

        auto& particles = *particles_ptr;
        for (int count = 0; count < PARTICLE_COUNT; count++) {
            const auto& p = particles[count];
            
            // Multiply each coordinate by the particle's weight
            x += p.pose.x * p.weight;
            y += p.pose.y * p.weight;
            sin_sum += pTrigs[count].sin_m * p.weight;
            cos_sum += pTrigs[count].cos_m * p.weight;
            
            total_weight += p.weight;
            weight_sqr_sum += p.weight * p.weight;
        }

        // Handle the case where all weights are zero (safety)
        if (total_weight < 1e-9) return {rawMcl, 0.0}; 

        return {{
            x / total_weight, 
            y / total_weight, 
            std::atan2(sin_sum, cos_sum)
        }, total_weight*total_weight/weight_sqr_sum};
    }

    Pose step(double vex_theta, const std::vector<double>& dists, const std::vector<int>& confs) {
        double std_theta = vexToStd(vex_theta);
        predict(std_theta);
        update_weights(dists, confs, std_theta);

        auto estimate = get_estimate();

        if (estimate.second < RESAMPLE_THRESHOLD) resample();

        return estimate.first;
    }

    void set_pose(double x, double y, double vex_theta) {
        double std_theta = vexToStd(vex_theta);
        std::normal_distribution<double> x_dist(x, 1.0);
        std::normal_distribution<double> y_dist(y, 1.0);
        std::normal_distribution<double> t_dist(std_theta, 0.02);
        lastTheta = std_theta;

        auto& particles = *particles_ptr;
        for (auto& p : particles) {
            p.pose = {x_dist(gen), y_dist(gen), t_dist(gen)};
            p.weight = 1.0;
        }

        odomLast = chassis->getPose();
    }

    Pose updateMcl() {
        // Get Sensors
        std::vector<double> dists = {distance_collection[0]->get()*mmToInch, distance_collection[1]->get()*mmToInch, distance_collection[2]->get()*mmToInch};
        std::vector<int> confs = {distance_collection[0]->get_confidence(), distance_collection[1]->get_confidence(), distance_collection[2]->get_confidence()};
        
        // Update Filter
        rawMcl = step(chassis->getPose().theta, dists, confs);

        // Sync
        if (autoSync) updateBotPose();
        odomLast = chassis->getPose();

        return rawMcl;
    }

    void updateBotPose(double weight = 0.1) {
        // Clamp weight
        weight = std::clamp(weight, 0.0, 1.0);

        lemlib::Pose odomPose = chassis->getPose();
        
        // Interpolate target x
        double newX = odomPose.x + weight * (rawMcl.x - odomPose.x);
        double newY = odomPose.y + weight * (rawMcl.y - odomPose.y);

        // Circular interpolation for theta
        // Convert Odom to Std Radians for calculation
        double odomRad = vexToStd(odomPose.theta);
        
        // Calculate shortest difference
        double diff = rawMcl.theta - odomRad;
        
        // Normalize difference to [-PI, PI]
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;

        // Apply weighted difference
        double newThetaRad = odomRad + (diff * weight);

        // Convert back to VEX Degrees for LemLib
        chassis->setPose(newX, newY, stdToVex(newThetaRad));
        this->lastTheta = newThetaRad;
    }

    void startTracking() {
        if (MclTrackingTask == nullptr) {
            MclTrackingTask = new pros::Task([this](){
                
                while (true) {
                    this->t.reset();

                    this->updateMcl();
                
                    if (t.timeLeft() < minPause) pros::delay(minPause);
                    else pros::delay(round(t.timeLeft()));
                }
            });
        }
    }

    void stopTracking() {
        if (MclTrackingTask != nullptr) { MclTrackingTask->remove(); delete MclTrackingTask; MclTrackingTask = nullptr; }
    }

    ~MclTracking() {
        stopTracking();
    }
};

#endif