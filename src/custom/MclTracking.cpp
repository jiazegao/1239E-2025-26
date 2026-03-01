#include "MclTracking.hpp"
#include "Tracking_Util.hpp"
#include "configs.hpp"
#include "pros/motor_group.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <queue>

#include "fast_trig.hpp"
#include "pros/rtos.h"

// --- Async Logging Variables ---
enum class LogType { PARTICLE, POSE, SENSOR };

struct LogData {
    LogType type;
    float v1, v2, v3, v4, v5, v6, v7; // Generic payload to hold either pose or particle data
};

std::queue<LogData> log_queue;
pros::Mutex log_mutex;
pros::Task* asyncLogTask = nullptr;

float MclTracking::intersect_line(Pose ray, Line_ wall, float max_range, float rayCos, float raySin) {

    // Vert wall
    float xMin = std::min(wall.p1.x, wall.p2.x);
    float xMax = std::max(wall.p1.x, wall.p2.x);
    float yMin = std::min(wall.p1.y, wall.p2.y);
    float yMax = std::max(wall.p1.y, wall.p2.y);

    if ((rayCos > 0 && ray.x > xMax) ||
        (rayCos < 0 && ray.x < xMin) ||
        (raySin > 0 && ray.y > yMax) ||
        (raySin < 0 && ray.y < yMin)) return max_range;

    float x1 = wall.p1.x; float y1 = wall.p1.y;
    float x2 = wall.p2.x; float y2 = wall.p2.y;
    float x3 = ray.x;     float y3 = ray.y;
    float x4 = ray.x + rayCos * max_range;
    float y4 = ray.y + raySin * max_range;

    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(den) < 1e-6f) return max_range;

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return u * max_range;
    return max_range;
}

float MclTracking::intersect_circle(Pose ray, Circle c, float max_range, float dx, float dy) {
    // Bound check
    float x_diff = ray.x - c.x;
    float y_diff = ray.y - c.y;
    float c_temp = max_range + c.radius;
    if (x_diff * x_diff + y_diff * y_diff > c_temp * c_temp) {
        return max_range; 
    }
    // Calculations
    float fx = ray.x - c.x;
    float fy = ray.y - c.y;
    float b = 2 * (fx * dx + fy * dy);
    float val_c = (fx * fx + fy * fy) - (c.radius * c.radius);
    float discriminant = b * b - 4 * val_c;
    if (discriminant < 0) return max_range;
    discriminant = std::sqrtf(discriminant);
    float t1 = (-b - discriminant) / 2;
    float t2 = (-b + discriminant) / 2;
    if (t1 >= 0 && t1 <= max_range) return t1;
    if (t2 >= 0 && t2 <= max_range) return t2;
    return max_range;
}

MclTracking::MclTracking(lemlib::Chassis* chassis, pros::MotorGroup* leftMotorGroup, pros::MotorGroup* rightMotorGroup, std::array<pros::Distance*, SENSOR_COUNT> dist_collection, std::tuple<pros::Rotation*, float, float> vertical_tracking_wheel, std::tuple<pros::Rotation*, float, float> horizontal_tracking_wheel, float start_x, float start_y, float start_vex_theta, bool autoSync_) {
    this->chassis = chassis;
    this->leftMotorGroup = leftMotorGroup;
    this->rightMotorGroup = rightMotorGroup;
    for (int i = 0; i < SENSOR_COUNT; i++) {this->distance_collection[i] = dist_collection[i];}
    this->autoSync = autoSync_;

    this->vertical_tracking_wheel = get<0>(vertical_tracking_wheel);
    this->vert_c = get<1>(vertical_tracking_wheel)*std::numbers::pi;
    this->vert_offset = get<2>(vertical_tracking_wheel);
    this->last_vertical_reading = this->vertical_tracking_wheel->get_position()/100.0f;

    this->horizontal_tracking_wheel = get<0>(horizontal_tracking_wheel);
    this->horiz_c = get<1>(horizontal_tracking_wheel)*std::numbers::pi;
    this->horiz_offset = get<2>(horizontal_tracking_wheel);
    this->last_horizontal_reading = (-1)*this->horizontal_tracking_wheel->get_position()/100.0f;

    std::random_device rd;
    gen = std::mt19937(rd());
    
    float start_std_theta = vexToStd(start_vex_theta);
    this->lastTheta = vexToStd(this->chassis->getPose().theta);
    std::normal_distribution<float> x_init(start_x, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<float> y_init(start_y, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<float> t_init(start_std_theta, THETA_RESAMPLE_VARIANCE);

    particles_ptr = &particles_array;
    new_gen_ptr = &new_gen_array;

    auto& particles = *particles_ptr;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        particles[i] = {{x_init(gen), y_init(gen), t_init(gen)}, 1.0f};
    }

    // Mount trig calculation
    for(int i = 0; i < SENSOR_COUNT; i++) {
        mountTrigs[i] = {FastTrig::cos(sensor_mounts[i].theta), FastTrig::sin(sensor_mounts[i].theta)};
    }

    // Pre-generated noise
    std::normal_distribution<float> dist(0.0f, 1.0f);
    for (int i = 0; i < NOISE_POOL_SIZE; i++) {
        noise_pool[i] = dist(gen);
    }

    // Generate distance map
    this->generate_distance_map();
}

void MclTracking::predict(float current_std_theta) {
    // Update theta deviation
    float d_theta = current_std_theta - lastTheta;
    while (d_theta > M_PI) d_theta -= 2 * M_PI;
    while (d_theta < -M_PI) d_theta += 2 * M_PI;
    lastTheta = current_std_theta;

    // Noise variables
    float drift_variance = std::hypotf(vertical_drift, horizontal_drift)/2.0f;

    // Calculate vertical tracking wheel vector
    float vert_reading = vertical_tracking_wheel->get_position()/100.0f;
    float d_vert_raw = (vert_reading-last_vertical_reading)/360.0f * vert_c;
    last_vertical_reading = vert_reading;

    // Calculate horizontal tracking wheel vector
    float horiz_reading = (-1)*horizontal_tracking_wheel->get_position()/100.0f;
    float d_horiz_raw = (horiz_reading-last_horizontal_reading)/360.0f * horiz_c;
    last_horizontal_reading = horiz_reading;

    // Get raw reading
    float d_vert_pure = d_vert_raw - (vert_offset * d_theta) + vertical_drift;
    float d_horiz_pure = d_horiz_raw + (horiz_offset * d_theta) + horizontal_drift;

    this->latest_speed = std::hypotf(d_vert_pure, d_horiz_pure) * INV_MSPT * 1000.0f;

    // Sync right after tracking wheel calculations to minimize data loss
    if (autoSync) updateBotPose();

    auto& particles = *particles_ptr;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        auto& p = particles[i];
        float pCos = FastTrig::cos(p.pose.theta + (d_theta/2));
        float pSin = FastTrig::sin(p.pose.theta + (d_theta/2));

        // Get noise
        float vert_noise = 1.0f + next_noise()*TRACKING_WHEEL_VARIANCE;
        float horiz_noise = 1.0f + next_noise()*TRACKING_WHEEL_VARIANCE;

        // Forward / Backward motion with noise
        float forward_dist = d_vert_pure * vert_noise + next_noise()*drift_variance;
        float strafe_dist = d_horiz_pure * horiz_noise + next_noise()*drift_variance;

        // Update position
        p.pose.x += forward_dist*pCos + strafe_dist*pSin;
        p.pose.y += forward_dist*pSin - strafe_dist*pCos;
        p.pose.theta += d_theta + next_noise()*THETA_RESAMPLE_VARIANCE;

        while (p.pose.theta > M_PI) p.pose.theta -= 2 * M_PI;
        while (p.pose.theta < -M_PI) p.pose.theta += 2 * M_PI;

        pTrigs[i] = {FastTrig::cos(p.pose.theta), FastTrig::sin(p.pose.theta)};
    }
}

void MclTracking::update_weights() {

    // Pre-processing filters

    // Bot trigs
    float botCos = FastTrig::cos(rawMcl.theta);
    float botSin = FastTrig::sin(rawMcl.theta);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_readings_mm[i] = distance_collection[i]->get();
        sensor_readings_inch[i] = sensor_readings_mm[i] * mmToInch;
        sensor_confs[i] = distance_collection[i]->get_confidence();
        valid_sensors[i] = true;

        // Validify sensors

        // Case #1: Sensor disabled
        if (disabled_sensors[i]) {
            valid_sensors[i] = false;
            continue;
        }
        // Case #2: Invalid reading
        if (sensor_readings_mm[i] > 9000) {
            valid_sensors[i] = false;
            continue;
        }
        // Case #3: Invalid confidence
        else if (sensor_readings_mm[i] > 200 && sensor_confs[i] < CONFIDENCE_THRESHOLD)  {
            valid_sensors[i] = false;
            continue;
        }
        // Case #4: Disqualifying intersection with obstacles

        // Sensor positions
        float sx = rawMcl.x + (sensor_mounts[i].x * botCos - sensor_mounts[i].y * botSin);
        float sy = rawMcl.y + (sensor_mounts[i].x * botSin + sensor_mounts[i].y * botCos);
        float ray_ang = rawMcl.theta + sensor_mounts[i].theta;

        // Log
        LogData pLog;
        pLog.type = LogType::SENSOR;
        pLog.v1 = sensor_readings_mm[i];
        pLog.v2 = sx;
        pLog.v3 = sy;
        pLog.v4 = ray_ang;

        log_mutex.take();
        log_queue.push(pLog);
        log_mutex.give();

        // Test for intersections
        if (disabling_line_obstacles != nullptr) {
            for (auto line : *disabling_line_obstacles) {
                if (intersect_line({sx, sy, ray_ang}, line, MAX_RANGE, FastTrig::cos(ray_ang), FastTrig::sin(ray_ang)) < MAX_RANGE) {
                    valid_sensors[i] = false;
                    break;
                }
            }
        }
        if (!valid_sensors[i]) continue;

        if (disabling_circle_obstacles != nullptr) {
            for (auto circle : *disabling_circle_obstacles) {
                if (intersect_circle({sx, sy, ray_ang}, circle, MAX_RANGE, FastTrig::cos(ray_ang), FastTrig::sin(ray_ang)) < MAX_RANGE) {
                    valid_sensors[i] = false;
                    break;
                }
            }
        }
    }

    // Calculate sigmas
    float inv_sigmas[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++) {
        // Sigma in inches: <= 200mm -> 0.787 inch ; > 200mm -> %5 reading inch
        float sigma = (sensor_readings_mm[i] <= 200) ? 0.787f : (sensor_readings_inch[i] * 0.05f);
        inv_sigmas[i] = 1.0f / sigma;
    }

    // Process particles
    for (auto& p : *particles_ptr) {
        float total_weight = 1.0f;
        
        // Cache particle trig
        float cp = FastTrig::cos(p.pose.theta);
        float sp = FastTrig::sin(p.pose.theta);

        for (int i = 0; i < SENSOR_COUNT; i++) {
            if (!valid_sensors[i]) continue;

            // Transform Sensor Mount to World Space
            // (Standard 2D Rotation: x' = x*cos - y*sin, y' = x*sin + y*cos)
            float sx = p.pose.x + (sensor_mounts[i].x * cp - sensor_mounts[i].y * sp);
            float sy = p.pose.y + (sensor_mounts[i].x * sp + sensor_mounts[i].y * cp);

            // Project Sensor Reading (Hit Point)
            float ray_angle = p.pose.theta + sensor_mounts[i].theta;
            float hx = sx + FastTrig::cos(ray_angle) * sensor_readings_inch[i];
            float hy = sy + FastTrig::sin(ray_angle) * sensor_readings_inch[i];

            // Grid Lookup
            int gx = (int)((hx + MAP_OFFSET) * MAP_SCALE);
            int gy = (int)((hy + MAP_OFFSET) * MAP_SCALE);

            if (gx >= 0 && gx < MAP_RES && gy >= 0 && gy < MAP_RES) {
                // Retrieve d^0.5
                float d_root = (float)distance_map[gy * MAP_RES + gx] * INV_DIST_MULTIPLIER;
                
                // Calculate z = d / sigma
                // Since d_root is d^0.5, d is (d_root * d_root)
                float z = (d_root * d_root) * inv_sigmas[i];

                // 4. LUT Lookup (Index = z * 256, because 1024 samples / 4.0 max sigma)
                int lut_idx = (int)(z * 256.0f);

                if (lut_idx < 1024) {
                    total_weight *= gaussian_lut[lut_idx];
                } else {
                    total_weight *= FAULT_TOLERANCE; 
                }
            } else {
                total_weight *= FAULT_TOLERANCE;
            }
        }
        p.weight = total_weight;
    }
}

void MclTracking::resample() {
    auto& particles = *particles_ptr;

    // Calculate the total weight of all particles
    float total_weight = 0.0f;
    for (const auto& p : particles) {
        total_weight += p.weight;
    }

    // Average step distance
    float step = total_weight * INV_PARTICLE_COUNT;

    // Generate one random starting point inside the first gap
    std::uniform_real_distribution<float> starter(0.0f, step);
    float currPos = starter(gen);

    float cumWeight = particles[0].weight; // Cumulative weight
    int index = 0;  // particle index

    for (int m = 0; m < PARTICLE_COUNT; ++m) {
        currPos += step;    // Current target position

        // Walk down until reach target
        while (currPos > cumWeight) {
            index++;
            cumWeight += particles[index].weight;
        }

        // Select particle at current position
        Particle selected = particles[index];

        selected.pose.x += next_noise() * DIST_RESAMPLE_VARIANCE;
        selected.pose.y += next_noise() * DIST_RESAMPLE_VARIANCE;
        selected.pose.theta += next_noise() * THETA_RESAMPLE_VARIANCE;
        selected.weight = 1.0f;

        (*new_gen_ptr)[m] = selected;
    }
    std::swap(particles_ptr, new_gen_ptr);
}

std::pair<Pose, float> MclTracking::get_estimate() {
    float x = 0, y = 0, sin_sum = 0, cos_sum = 0;
    float total_weight = 0;
    float weight_sqr_sum = 0;

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

        // Log - Log Particle Positions
        if (mclLogType == SDCARD && count % LOG_RATIO == 0) {
            LogData pLog;
            pLog.type = LogType::PARTICLE;
            pLog.v1 = p.pose.x;
            pLog.v2 = p.pose.y;

            log_mutex.take();
            log_queue.push(pLog);
            log_mutex.give();
        }
    }

    // Handle the case where all weights are zero (safety)
    if (total_weight < 1e-20f) {
        uniform_reset();
        return {rawMcl, 0.0};
    }

    return {{
        x / total_weight, 
        y / total_weight, 
        std::atan2(sin_sum, cos_sum)
    }, total_weight*total_weight/weight_sqr_sum};
}

Pose MclTracking::updateMcl() {

    predict(vexToStd(chassis->getPose().theta)); // Also syncs position back to lemlib
    update_weights();

    // Log general position
    auto estimate = get_estimate(); // (Logs Particles)

    // Prevent resampling during rotations at a single point
    float distSinceResample = std::hypotf(estimate.first.x - lastResamplePose.x, estimate.first.y - lastResamplePose.y);

    if ((estimate.second < RESAMPLE_THRESHOLD * 0.5f) || (estimate.second < RESAMPLE_THRESHOLD && distSinceResample > MIN_DIST_FROM_RESAMPLE && latest_speed < MAX_VELO_RESAMPLE)) {
        resample();
        lastResamplePose = estimate.first;
    }

    return estimate.first;
}

void MclTracking::set_pose(float x, float y, float vex_theta) {
    float std_theta = vexToStd(vex_theta);
    std::normal_distribution<float> x_dist(x, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<float> y_dist(y, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<float> t_dist(std_theta, THETA_RESAMPLE_VARIANCE);

    this->lastTheta = vexToStd(this->chassis->getPose().theta);
    this->last_vertical_reading = this->vertical_tracking_wheel->get_position()/100.0f;
    this->last_horizontal_reading = (-1)*this->horizontal_tracking_wheel->get_position()/100.0f;
    this->latest_speed = 0.0f;

    auto& particles = *particles_ptr;
    for (auto& p : particles) {
        p.pose = {x_dist(gen), y_dist(gen), t_dist(gen)};
        p.weight = 1.0f;
    }
}

void MclTracking::uniform_reset() {
    std::uniform_real_distribution<float> x_dist(FIELD_NEG_HALF_LENGTH, FIELD_HALF_LENGTH);
    std::uniform_real_distribution<float> y_dist(FIELD_NEG_HALF_LENGTH, FIELD_HALF_LENGTH);

    this->lastTheta = vexToStd(this->chassis->getPose().theta);
    this->last_vertical_reading = this->vertical_tracking_wheel->get_position()/100.0f;
    this->last_horizontal_reading = (-1)*this->horizontal_tracking_wheel->get_position()/100.0f;
    this->latest_speed = 0.0f;

    auto& particles = *particles_ptr;
    for (auto& p : particles) {
        p.pose = {x_dist(gen), y_dist(gen), lastTheta};
        p.weight = 1.0f;
    }
}

void MclTracking::updateBotPose() {

    lemlib::Pose odomPose = chassis->getPose();
    
    // Interpolate target x
    float newX = odomPose.x + DIST_SYNC_PROP * (rawMcl.x - odomPose.x);
    float newY = odomPose.y + DIST_SYNC_PROP * (rawMcl.y - odomPose.y);

    // Circular interpolation for theta
    // Convert Odom to Std Radians for calculation
    float odomRad = vexToStd(odomPose.theta);
    
    // Calculate shortest difference
    float diff = rawMcl.theta - odomRad;
    
    // Normalize difference to [-PI, PI]
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    // Apply weighted difference
    float newThetaRad = odomRad + (diff * THETA_SYNC_PROP);

    // Convert back to VEX Degrees for LemLib
    chassis->setPose(newX, newY, stdToVex(newThetaRad));
    this->lastTheta = newThetaRad;
}

void MclTracking::startTracking() {
    if (MclTrackingTask == nullptr) {

        // Log distance map
        if (mclLogType == SDCARD) {
            for (int i = 0; i < MAP_RES*MAP_RES-1; i++) {
                *mclLog << distance_map[i] << " ";
            }
            *mclLog << distance_map[MAP_RES*MAP_RES-1] << "\n";
        }

        MclTrackingTask = new pros::Task([this](){
            while (true) {
                this->t.reset();

                this->updateMcl();

                // Log on brain
                if (mclLogType == SCREEN) {
                    pros::lcd::print(5, "MCL Calculation Time: %f", t.elapsed());
                }
            
                if (t.timeLeft() < minPause) pros::delay(minPause);
                else pros::delay(round(t.timeLeft()));
            }
        });
    }
}

void MclTracking::stopTracking() {
    if (MclTrackingTask != nullptr) { MclTrackingTask->remove(); delete MclTrackingTask; MclTrackingTask = nullptr; }
}

void MclTracking::setDistSyncProp(float newDistSyncProp) {
    DIST_SYNC_PROP = newDistSyncProp;
}

void MclTracking::logMcl() {
    // Inside step()
    float sum_sq_diff_x = 0, sum_sq_diff_y = 0;
    float sum_sin = 0, sum_cos = 0;

    auto& particles = *particles_ptr;

    for (const auto& p : particles) {
        sum_sq_diff_x += (p.pose.x - rawMcl.x)*(p.pose.x - rawMcl.x);
        sum_sq_diff_y += (p.pose.y - rawMcl.y)*(p.pose.y - rawMcl.y);
        
        // Use circular statistics for theta
        sum_sin += FastTrig::sin(p.pose.theta);
        sum_cos += FastTrig::cos(p.pose.theta);
    }

    // Calculate standard deviation
    float std_dev_x = std::sqrtf(sum_sq_diff_x * INV_PARTICLE_COUNT);
    float std_dev_y = std::sqrtf(sum_sq_diff_y * INV_PARTICLE_COUNT);

    // Circular standard deviation for heading
    float R = std::hypotf(sum_sin * INV_PARTICLE_COUNT, sum_cos * INV_PARTICLE_COUNT);
    float std_dev_theta = std::sqrtf(-2.0f * std::log(R)); 

    // Log to Queue (Zero Blocking)
    LogData pLog;
    pLog.type = LogType::POSE;
    pLog.v1 = mclLogTimer.elapsed(TimeUnit::SECOND);
    pLog.v2 = rawMcl.x;
    pLog.v3 = rawMcl.y;
    pLog.v4 = rawMcl.theta;
    pLog.v5 = std_dev_x;
    pLog.v6 = std_dev_y;
    pLog.v7 = std_dev_theta;

    log_mutex.take();
    log_queue.push(pLog);
    log_mutex.give();
}

void MclTracking::setDrift(float verticalDriftPerSec, float horizontalDriftPerSec) {
    this->vertical_drift = verticalDriftPerSec / (1000.0f * INV_MSPT);
    this->horizontal_drift = horizontalDriftPerSec / (1000.0f * INV_MSPT);
}

void MclTracking::generate_distance_map() {
    // Initialize gaussian lut
    for (int i = 0; i < 1024; i++) {
        float x = (i / 1024.0f) * 4.0f; // Map index to 0-4 sigmas
        gaussian_lut[i] = std::exp(-(x * x) / 2.0f);
    }

    // Build the Map
    for (int y = 0; y < MAP_RES; y++) {
        for (int x = 0; x < MAP_RES; x++) {
            // Index to world coordinates (Center of cell)
            float fx = (x / MAP_SCALE) - MAP_OFFSET + (0.5f / MAP_SCALE);
            float fy = (y / MAP_SCALE) - MAP_OFFSET + (0.5f / MAP_SCALE);
            
            float min_d = 1e6;

            // Distance to perimeter walls
            for (const auto& wall : walls) 
                min_d = std::min(min_d, get_dist_to_segment(fx, fy, wall));

            // Distance to diagonal obstacles (the middle bars)
            for (const auto& obs : line_obstacles) 
                min_d = std::min(min_d, get_dist_to_segment(fx, fy, obs));

            // Distance to circular match loaders
            for (const auto& circle : circle_obstacles) {
                float d = std::hypot(fx - circle.x, fy - circle.y);
                min_d = std::min(min_d, std::abs(d - circle.radius));
            }

            // Store distance to objects
            float stored_val = std::sqrt(min_d) * DIST_MULTIPLIER;
            distance_map[y * MAP_RES + x] = (uint8_t)std::min(stored_val, 255.0f);
        }
    }
}

void MclTracking::enableSens(int sens) {
    int filtered = std::max(std::min(SENSOR_COUNT-1, sens), 0);
    disabled_sensors[filtered] = false;
}

void MclTracking::disableSens(int sens) {
    int filtered = std::max(std::min(SENSOR_COUNT-1, sens), 0);
    disabled_sensors[filtered] = true;
}

void MclTracking::setObstacles(std::vector<Line_>* newLineObstaclesPtr, std::vector<Circle>* newCirleObstaclesPtr) {
    disabling_line_obstacles = newLineObstaclesPtr;
    disabling_circle_obstacles = newCirleObstaclesPtr;
}

float MclTracking::get_dist_to_segment(float px, float py, Line_ seg) {
    float dx = seg.p2.x - seg.p1.x;
    float dy = seg.p2.y - seg.p1.y;
    float l2 = dx*dx + dy*dy;
    
    // If the line is actually a point
    if (l2 == 0.0) return std::hypot(px - seg.p1.x, py - seg.p1.y);
    
    // Project point onto line, clamped to [0, 1]
    float t = ((px - seg.p1.x) * dx + (py - seg.p1.y) * dy) / l2;
    t = std::max(0.0f, std::min(1.0f, t));
    
    float closest_x = seg.p1.x + t * dx;
    float closest_y = seg.p1.y + t * dy;
    
    return std::hypot(px - closest_x, py - closest_y);
}

void MclTracking::startAsyncLogger() {
    if (asyncLogTask == nullptr) {
        asyncLogTask = new pros::Task([this]() {
            while (true) {
                std::queue<LogData> local_queue;

                // Lock once, swap the contents, and unlock immediately
                log_mutex.take();
                if (!log_queue.empty()) {
                    std::swap(log_queue, local_queue);
                }
                log_mutex.give();

                if (!local_queue.empty()) {
                    // Process the entire batch completely outside the lock
                    while (!local_queue.empty()) {
                        LogData data = local_queue.front();
                        local_queue.pop();
                        
                        if (data.type == LogType::PARTICLE) {
                            *mclLog << data.v1 << "," << data.v2 << "\n";
                        } else if (data.type == LogType::POSE) {
                            *mclLog << data.v1 << "\n"; 
                            *mclLog << data.v2 << "," << data.v3 << "," << data.v4 << "\n"; 
                            *mclLog << data.v5 << "," << data.v6 << "," << data.v7 << "\n"; 
                        } else if (data.type == LogType::SENSOR ) {
                            *mclLog << data.v1 << "," << data.v2 << "," << data.v3 << "," << data.v4 << "\n"; 
                        }
                    }
                } else {
                    // Sleep to prevent CPU hogging when queue is empty
                    pros::delay(MSPT); 
                }
            }
        }, TASK_PRIORITY_DEFAULT-1);
    }
}

MclTracking::~MclTracking() {
    stopTracking();
}