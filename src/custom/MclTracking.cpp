#include "MclTracking.hpp"
#include "Tracking_Util.hpp"
#include "configs.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <queue>

#include "pros/rtos.h"

// --- Async Logging Variables ---
enum class LogType { PARTICLE, POSE, SENSOR };

struct LogData {
    LogType type;
    float v1, v2, v3, v4, v5, v6; // Generic payload to hold either pose or particle data
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
    float t1 = (-b - discriminant) * 0.5f;
    float t2 = (-b + discriminant) * 0.5f;
    if (t1 >= 0 && t1 <= max_range) return t1;
    if (t2 >= 0 && t2 <= max_range) return t2;
    return max_range;
}

MclTracking::MclTracking(lemlib::Chassis* chassis, lemlib::Drivetrain* dt, std::array<pros::Distance*, SENSOR_COUNT> dist_collection, std::tuple<pros::Rotation*, float, float> vertical_tracking_wheel, std::tuple<pros::Rotation*, float, float> horizontal_tracking_wheel, float start_x, float start_y, float start_vex_theta, bool autoSync_) {
    this->chassis = chassis;
    this->dt = dt;
    for (int i = 0; i < SENSOR_COUNT; i++) {this->distance_collection[i] = dist_collection[i];}
    this->autoSync = autoSync_;

    this->vertical_tracking_wheel = get<0>(vertical_tracking_wheel);
    if (this->vertical_tracking_wheel != nullptr) {
        this->vert_c = get<1>(vertical_tracking_wheel)*std::numbers::pi;
        this->vert_offset = get<2>(vertical_tracking_wheel);
        this->last_vertical_reading = this->vertical_tracking_wheel->get_position()/100.0f;
    }
    else {
        this->vert_c = dt->wheelDiameter*std::numbers::pi;
        this->vert_offset = 0.0;
        this->last_vertical_reading = getDTWheelDegrees();
    }

    this->horizontal_tracking_wheel = get<0>(horizontal_tracking_wheel);
    if (this->horizontal_tracking_wheel != nullptr) {
        this->horiz_c = get<1>(horizontal_tracking_wheel)*std::numbers::pi;
        this->horiz_offset = get<2>(horizontal_tracking_wheel);
        this->last_horizontal_reading = -this->horizontal_tracking_wheel->get_position()/100.0f;
    }
    else {
        this->horiz_c = 0.0;
        this->horiz_offset = 0.0;
        this->last_horizontal_reading = 0.0;
    }

    std::random_device rd;
    gen = std::mt19937(rd());
    
    this->lastTheta = vexToStd(this->chassis->getPose().theta);
    std::normal_distribution<float> x_init(start_x, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<float> y_init(start_y, DIST_RESAMPLE_VARIANCE);

    particles_ptr = &particles_array;
    new_gen_ptr = &new_gen_array;

    auto& particles = *particles_ptr;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        particles[i] = {{x_init(gen), y_init(gen)}, 1.0f};
    }

    // Mount trig calculation
    for(int i = 0; i < SENSOR_COUNT; i++) {
        mountTrigs[i] = {std::cos(sensor_mounts[i].theta), std::sin(sensor_mounts[i].theta)};
    }

    // Pre-generated noise
    std::normal_distribution<float> dist(0.0f, 1.0f);
    for (int i = 0; i < NOISE_POOL_SIZE; i++) {
        noise_pool[i] = dist(gen);
    }

    // Reset timers
    for (int i = 0; i < SENSOR_COUNT; i++) {
        disableTimers[i] = Timer(0);
    }

    // Initialize gaussian lut
    for (int i = 0; i < 1024; i++) {
        float x = (i / 1024.0f) * 4.0f; // Map index to 0-4 sigmas
        gaussian_lut[i] = std::exp(-(x * x) / 2.0f);
    }
}

void MclTracking::predict() {
    // Update theta deviation
    float current_std_theta = vexToStd(chassis->getPose().theta);

    float d_theta = current_std_theta - lastTheta;
    while (d_theta > M_PI) d_theta -= 2 * M_PI;
    while (d_theta < -M_PI) d_theta += 2 * M_PI;

    // Arc Approximation
    float half_d_theta = d_theta * 0.5f;
    float move_scale;

    // Avoid division by zero when driving straight
    if (std::abs(d_theta) < 1e-6f) {
        move_scale = 1.0f;
    }
    else {
        move_scale = std::sinf(half_d_theta) / half_d_theta;
    }

    // Noise variables
    float drift_variance = std::hypotf(vertical_drift, horizontal_drift) * 0.5f;

    // Calculate vertical tracking wheel vector
    float vert_reading = 0.0f;
    // Retrieve reading from verticle tracking wheel
    if (vertical_tracking_wheel != nullptr) {
        vert_reading = vertical_tracking_wheel->get_position() * 0.01f;
    }
    // Retrieve readings from drive motors
    else {
        vert_reading = getDTWheelDegrees();
    }
    float d_vert_raw = (vert_reading-last_vertical_reading) * 0.0027777777777f * vert_c;
    last_vertical_reading = vert_reading;

    // Calculate horizontal tracking wheel vector
    float horiz_reading = 0.0f;
    // Retrieve reading from horizontal tracking wheel
    if (horizontal_tracking_wheel != nullptr) {
        horiz_reading = -horizontal_tracking_wheel->get_position() * 0.01f;
    }
    float d_horiz_raw = (horiz_reading-last_horizontal_reading) * 0.0027777777777f * horiz_c;
    last_horizontal_reading = horiz_reading;

    // Get raw reading
    float d_vert_pure = d_vert_raw - (vert_offset * d_theta) + vertical_drift;
    float d_horiz_pure = d_horiz_raw + (horiz_offset * d_theta) + horizontal_drift;

    this->latest_speed = std::hypotf(d_vert_pure, d_horiz_pure) * INV_MSPT * 1000.0f;

    // If horizontal wheel is absent, introduce more variance
    float horiz_dependent_variance = 0.0f;
    if (horizontal_tracking_wheel == nullptr) {
        horiz_dependent_variance = d_vert_pure * HORIZ_DEPENDENT_VARIANCE_PROP;
    }

    // Half angle trig
    currTrig.cos_m = std::cos(lastTheta+half_d_theta);
    currTrig.sin_m = std::sin(lastTheta+half_d_theta);

    auto& particles = *particles_ptr;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        auto& p = particles[i];

        // Get noise
        float vert_noise = 1.0f + next_noise()*TRACKING_WHEEL_VARIANCE;
        float horiz_noise = 1.0f + next_noise()*TRACKING_WHEEL_VARIANCE;

        // Forward / Backward motion with noise
        float forward_dist = d_vert_pure * vert_noise + next_noise()*drift_variance;
        float strafe_dist = d_horiz_pure * horiz_noise + next_noise()*drift_variance + next_noise()*horiz_dependent_variance;

        // Update position using Arc Approximation
        float local_vert = forward_dist * move_scale;
        float local_horiz = strafe_dist * move_scale;

        p.pose.x += local_vert*currTrig.cos_m + local_horiz*currTrig.sin_m;
        p.pose.y += local_vert*currTrig.sin_m - local_horiz*currTrig.cos_m;
    }

    // Full angle trig
    currTrig.cos_m = std::cos(current_std_theta);
    currTrig.sin_m = std::sin(current_std_theta);

    lastTheta = current_std_theta;
}

void MclTracking::update_weights() {

    // Pre-processing filters
    Coord sensor_offsets[SENSOR_COUNT];
    Trig sensor_trigs[SENSOR_COUNT];
    float active_sensors = 0.0f;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_readings_mm[i] = distance_collection[i]->get();
        sensor_readings_inch[i] = sensor_readings_mm[i] * mmToInch;
        sensor_confs[i] = distance_collection[i]->get_confidence();
        valid_sensors[i] = true;

        // Sensor positions
        float sx = rawMcl.x + (sensor_mounts[i].x * currTrig.cos_m - sensor_mounts[i].y * currTrig.sin_m);
        float sy = rawMcl.y + (sensor_mounts[i].x * currTrig.sin_m + sensor_mounts[i].y * currTrig.cos_m);
        float ray_ang = rawMcl.theta + sensor_mounts[i].theta;
        
        Pose sray = {sx, sy, ray_ang};

        float scos = currTrig.cos_m * mountTrigs[i].cos_m - currTrig.sin_m  * mountTrigs[i].sin_m;
        float ssin = currTrig.sin_m * mountTrigs[i].cos_m + currTrig.cos_m  * mountTrigs[i].sin_m;

        if (mclLogType == SDCARD) {
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
        }

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
        if (sensor_readings_mm[i] > 200 && sensor_confs[i] < CONFIDENCE_THRESHOLD)  {
            valid_sensors[i] = false;
            continue;
        }
        // Case #4: Top middle goal lip disabling
        if ((i == DISTSENSORS::BACK_LEFT || i == DISTSENSORS::BACK_RIGHT || i == DISTSENSORS::FRONT_LEFT || i == DISTSENSORS::FRONT_RIGHT)) {
            if (intersect_line(sray, top_middle, MAX_RANGE, scos, ssin) < MAX_RANGE) {
                valid_sensors[i] = false;
                continue;
            }
        }
        // Case #5: Lower middle goal disabling
        if (i == BACK) {
            if (intersect_line(sray, low_middle, MAX_RANGE, scos, ssin) < MAX_RANGE) {
                valid_sensors[i] = false;
                continue;
            }
        }
        // Case #6: Disqualifying intersection with obstacles
        
        // Test for intersections
        if (disabling_line_obstacles != nullptr) {
            for (auto line : *disabling_line_obstacles) {
                if (intersect_line(sray, line, MAX_RANGE, scos, ssin) < MAX_RANGE) {
                    valid_sensors[i] = false;
                    break;
                }
            }
        }
        if (!valid_sensors[i]) continue;

        if (disabling_circle_obstacles != nullptr) {
            for (auto circle : *disabling_circle_obstacles) {
                if (intersect_circle(sray, circle, MAX_RANGE, scos, ssin) < MAX_RANGE) {
                    valid_sensors[i] = false;
                    break;
                }
            }
        }
        if (!valid_sensors[i]) continue;

        // Calculate sensor transformations
        sensor_offsets[i].x = sensor_mounts[i].x * currTrig.cos_m - sensor_mounts[i].y * currTrig.sin_m;
        sensor_offsets[i].y = sensor_mounts[i].x * currTrig.sin_m + sensor_mounts[i].y * currTrig.cos_m;

        sensor_trigs[i].cos_m = scos;
        sensor_trigs[i].sin_m = ssin;

        active_sensors += 1.0f; // Increment valid sensors
    }

    // Calculate sigma
    float inv_sigmas[SENSOR_COUNT]; // Calculate inv_sigmas along the way
    float wall_inv_sigmas[SENSOR_COUNT];
    float sensor_count_multiplier = (active_sensors > 0) ? std::sqrtf(active_sensors * SENSOR_COUNT_SCALING) : 1.0f;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (valid_sensors[i]) {
            float angle_offset = std::fmod(std::abs(rawMcl.theta + sensor_mounts[i].theta), HALF_PIF);

            if (angle_offset > QTR_PIF) {
                angle_offset = HALF_PIF - angle_offset;
            }

            float angle_multiplier = 1.0f + angle_offset * RIGHT_ANG_CONST;

            // Sigma in inches: <= 200mm -> 0.787 inch ; > 200mm -> %5 reading inch
            float sigma = (sensor_readings_mm[i] <= 200) ? 0.787f : (sensor_readings_inch[i] * 0.05f);
            if (sensor_readings_mm[i] > 200) {
                sigma *= CONFIDENCE_SCALING_BASE / sensor_confs[i];
            }

            // Sensor count dynamic scaling
            sigma *= sensor_count_multiplier;

            // Angle dynamic scaling
            inv_sigmas[i] = 1.0f / sigma;
            wall_inv_sigmas[i] = inv_sigmas[i] / angle_multiplier;
        }
    }

    // Process particles
    auto& derefP = *particles_ptr;

    for (int i = 0; i < PARTICLE_COUNT; i++) {
        float total_weight = 1.0f;
        auto& p = derefP[i];

        // Instant DQ if out of bounds
        if (std::abs(p.pose.x) > FIELD_HALF_LENGTH || std::abs(p.pose.y) > FIELD_HALF_LENGTH) {
            p.weight = 1e-35f;
            continue;
        }

        for (int j = 0; j < SENSOR_COUNT; j++) {
            if (!valid_sensors[j]) continue;

            // Transform Sensor Mount to World Space
            // (Standard 2D Rotation: x' = x*cos - y*sin, y' = x*sin + y*cos)
            float sx = p.pose.x + sensor_offsets[j].x;
            float sy = p.pose.y + sensor_offsets[j].y;

            Pose sPose = {sx, sy, lastTheta};

            float rayCos = sensor_trigs[j].cos_m;
            float raySin = sensor_trigs[j].sin_m;

            float p_dist = MAX_RANGE;
            bool hit_wall = false;

            // Check for obstacle intersection first
            for (const auto& line : goal_legs) {
                p_dist = std::min(p_dist, intersect_line(sPose, line, MAX_RANGE, rayCos, raySin));
            }
            // If didn't intersect any legs, check other objects
            if (std::abs(p_dist-MAX_RANGE) < 1e-6) {
                for (const auto& c : match_loaders) {
                    p_dist = std::min(p_dist, intersect_circle(sPose, c, MAX_RANGE, rayCos, raySin));
                }
                // If didn't intersect matchloaders, check walls
                if (std::abs(p_dist-MAX_RANGE) < 1e-6) {
                    for (const auto& wall : walls) {
                        p_dist = std::min(p_dist, intersect_line(sPose, wall, MAX_RANGE, rayCos, raySin));
                        // Immediately break if intersect with one of the walls
                        if (std::abs(p_dist-MAX_RANGE) > 1e-6) {
                            hit_wall = true;
                            break;
                        }
                    }
                }
            }
        
            // Apply angle sigma scaling
            float z;
            if (hit_wall) {
                z = std::abs(sensor_readings_inch[j] - p_dist) * wall_inv_sigmas[j];
            }
            else {
                z = std::abs(sensor_readings_inch[j] - p_dist) * inv_sigmas[j];
            }

            // Gaussian LUT process
            int lut_idx = (int)(z * 256.0f);

            if (lut_idx < 1024) {
                total_weight *= gaussian_lut[lut_idx];
            } else {
                total_weight *= gaussian_lut[1023] * FAULT_TOLERANCE; 
            }    
        }
        p.weight = total_weight;
    }
}

void MclTracking::resample() {
    auto& particles = *particles_ptr;
    auto& new_gen = *new_gen_ptr;

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
        // Walk down until reach target
        while (index < PARTICLE_COUNT-1 && currPos > cumWeight) {
            index++;
            cumWeight += particles[index].weight;
        }

        // Select particle at current position
        Particle selected = particles[index];
        selected.pose.x += next_noise() * DIST_RESAMPLE_VARIANCE;
        selected.pose.y += next_noise() * DIST_RESAMPLE_VARIANCE;
        selected.weight = 1.0f;

        new_gen[m] = selected;
        currPos += step;    // Increment currPos
    }
    std::swap(particles_ptr, new_gen_ptr);
}

std::pair<Pose, float> MclTracking::get_estimate() {
    float x = 0, y = 0;
    float total_weight = 0;
    float weight_sqr_sum = 0;

    auto& particles = *particles_ptr;
    for (int count = 0; count < PARTICLE_COUNT; count++) {
        const auto& p = particles[count];
        
        // Multiply each coordinate by the particle's weight
        x += p.pose.x * p.weight;
        y += p.pose.y * p.weight;

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
        lastTheta
    }, total_weight*total_weight/weight_sqr_sum};
}

Pose MclTracking::updateMcl() {

    // Update sensor disability
    for (int i = 0; i < SENSOR_COUNT; i++) {
        auto& t = disableTimers[i];
        if (t.timeoutMs < 1.0f || t.timeIsUp()) disabled_sensors[i] = false;
        else disabled_sensors[i] = true;
    }

    predict();  // Update position
    update_weights(); // Log sensor

    auto estimate = get_estimate(); // Log particles
    // Log general position
    if (mclLogType == SDCARD) {
        logMcl();
    }

    // Prevent resampling during rotations at a single point
    float distSinceResample = std::hypotf(estimate.first.x - lastResamplePose.x, estimate.first.y - lastResamplePose.y);

    if ((estimate.second < RESAMPLE_THRESHOLD * 0.5f) || (estimate.second < RESAMPLE_THRESHOLD && distSinceResample > MIN_DIST_FROM_RESAMPLE && latest_speed < MAX_VELO_RESAMPLE)) {
        resample();
        lastResamplePose = estimate.first;
    }

    rawMcl = estimate.first;

    // Sync to chassis
    if (autoSync) updateBotPose();

    return estimate.first;
}

void MclTracking::set_pose(float x, float y, float vex_theta) {
    float std_theta = vexToStd(vex_theta);
    std::normal_distribution<float> x_dist(x, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<float> y_dist(y, DIST_RESAMPLE_VARIANCE);

    this->lastTheta = vexToStd(this->chassis->getPose().theta);
    // Verticle / DT
    this->last_vertical_reading = (vertical_tracking_wheel != nullptr) ? vertical_tracking_wheel->get_position()/100.0f : getDTWheelDegrees();
    // Horizontal / NULL
    this->last_horizontal_reading = (horizontal_tracking_wheel != nullptr) ? -horizontal_tracking_wheel->get_position()/100.0f : 0.0f;
    this->latest_speed = 0.0f;

    auto& particles = *particles_ptr;
    for (auto& p : particles) {
        p.pose = {x_dist(gen), y_dist(gen)};
        p.weight = 1.0f;
    }

    rawMcl = {x, y, std_theta};
}

void MclTracking::uniform_reset() {
    std::uniform_real_distribution<float> x_dist(FIELD_NEG_HALF_LENGTH, FIELD_HALF_LENGTH);
    std::uniform_real_distribution<float> y_dist(FIELD_NEG_HALF_LENGTH, FIELD_HALF_LENGTH);

    this->lastTheta = vexToStd(this->chassis->getPose().theta);
    // Verticle / DT
    this->last_vertical_reading = (vertical_tracking_wheel != nullptr) ? vertical_tracking_wheel->get_position()/100.0f : getDTWheelDegrees();
    // Horizontal / NULL
    this->last_horizontal_reading = (horizontal_tracking_wheel != nullptr) ? -horizontal_tracking_wheel->get_position()/100.0f : 0.0f;
    this->latest_speed = 0.0f;

    auto& particles = *particles_ptr;
    for (auto& p : particles) {
        p.pose = {x_dist(gen), y_dist(gen)};
        p.weight = 1.0f;
    }

    rawMcl = {0, 0, 0};
}

void MclTracking::updateBotPose() {

    lemlib::Pose odomPose = chassis->getPose();
    
    // Interpolate target x
    float newX = odomPose.x + DIST_SYNC_PROP * (rawMcl.x - odomPose.x);
    float newY = odomPose.y + DIST_SYNC_PROP * (rawMcl.y - odomPose.y);

    // Convert back to VEX Degrees for LemLib
    chassis->setPose(newX, newY, chassis->getPose().theta);
}

void MclTracking::startTracking() {
    if (MclTrackingTask == nullptr) {

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

    auto& particles = *particles_ptr;

    for (int i = 0; i < PARTICLE_COUNT; i++) {
        const auto& p = particles[i];

        sum_sq_diff_x += (p.pose.x - rawMcl.x)*(p.pose.x - rawMcl.x);
        sum_sq_diff_y += (p.pose.y - rawMcl.y)*(p.pose.y - rawMcl.y);
        
    }

    // Calculate standard deviation
    float std_dev_x = std::sqrtf(sum_sq_diff_x * INV_PARTICLE_COUNT);
    float std_dev_y = std::sqrtf(sum_sq_diff_y * INV_PARTICLE_COUNT);

    // Log to Queue (Zero Blocking)
    LogData pLog;
    pLog.type = LogType::POSE;
    pLog.v1 = mclLogTimer.elapsed(TimeUnit::SECOND);
    pLog.v2 = rawMcl.x;
    pLog.v3 = rawMcl.y;
    pLog.v4 = rawMcl.theta;
    pLog.v5 = std_dev_x;
    pLog.v6 = std_dev_y;

    log_mutex.take();
    log_queue.push(pLog);
    log_mutex.give();
}

void MclTracking::setDrift(float verticalDriftPerSec, float horizontalDriftPerSec) {
    this->vertical_drift = verticalDriftPerSec / (1000.0f * INV_MSPT);
    this->horizontal_drift = horizontalDriftPerSec / (1000.0f * INV_MSPT);
}

void MclTracking::enableSens(int sens) {
    if (sens < 0 || sens > SENSOR_COUNT-1) return;
    disableTimers[sens].hardReset(0);
}

void MclTracking::disableSens(int sens) {
    if (sens < 0 || sens > SENSOR_COUNT-1) return;
    disableTimers[sens].hardReset(1e20f);
}

void MclTracking::disableSensFor(int sens, float ms) {
    if (sens < 0 || sens > SENSOR_COUNT-1) return;
    disableTimers[sens].hardReset(ms);
}

void MclTracking::setObstacles(std::vector<Line_>* newLineObstaclesPtr, std::vector<Circle>* newCirleObstaclesPtr) {
    disabling_line_obstacles = newLineObstaclesPtr;
    disabling_circle_obstacles = newCirleObstaclesPtr;
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
                            *mclLog << data.v5 << "," << data.v6 << "," << "\n"; 
                        } else if (data.type == LogType::SENSOR ) {
                            *mclLog << data.v1 << "," << data.v2 << "," << data.v3 << "," << data.v4 << "\n"; 
                        }
                    }
                } else {
                    // Sleep to prevent CPU hogging when queue is empty
                    pros::delay(MSPT); 
                }
            }
        });
    }
}

void MclTracking::stopAsyncLogger() {
    if (asyncLogTask != nullptr) {
        asyncLogTask->remove(); 
        delete asyncLogTask; 
        asyncLogTask = nullptr; 
    }
}

float MclTracking::getDTWheelDegrees() {
    if (dt == nullptr || dt->leftMotors == nullptr || dt->rightMotors == nullptr) {
        return 0.0f;
    }

    // Get average raw rotations
    std::vector<double> leftPos = dt->leftMotors->get_position_all();
    std::vector<double> rightPos = dt->rightMotors->get_position_all();

    double sum = 0;
    for (double pos : leftPos) sum += pos;
    for (double pos : rightPos) sum += pos;

    // 6 wheels, 3 on each side
    float avgRotates = static_cast<float>(sum) * 0.166666666667f;

    // Calculate Manual Gear Ratio
    // Blue cartridge (11W) = 600 RPM internal
    float gearRatio = dt->rpm * 0.0016666666667f;

    // Convert Rotations to Wheel Degrees
    // Multiply by gearRatio converts motor degrees to wheel degrees
    float wheelDegrees = avgRotates * gearRatio * 360.0;

    return wheelDegrees;
}

MclTracking::~MclTracking() {
    stopTracking();
    stopAsyncLogger();
}