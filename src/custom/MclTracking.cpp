#include "MclTracking.hpp"
#include "Tracking_Util.hpp"
#include "configs.hpp"
#include "pros/motor_group.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include "fast_trig.hpp"

inline float roundTwoPlaces(float x) {
    return std::round(x*100)/100;
}

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
    if (den == 0) return max_range;

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

MclTracking::MclTracking(lemlib::Chassis* chassis, pros::MotorGroup* leftMotorGroup, pros::MotorGroup* rightMotorGroup, std::vector<pros::Distance*> dist_collection, std::tuple<pros::Rotation*, float, float> vertical_tracking_wheel, std::tuple<pros::Rotation*, float, float> horizontal_tracking_wheel, float start_x, float start_y, float start_vex_theta, bool autoSync_) {
    this->chassis = chassis;
    this->leftMotorGroup = leftMotorGroup;
    this->rightMotorGroup = rightMotorGroup;
    this->distance_collection = dist_collection;
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
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        particles[i] = {{x_init(gen), y_init(gen), t_init(gen)}, 1.0f};
    }

    // Mount trig calculation
    for(int i = 0; i < SENSOR_COUNT; ++i) {
        mountTrigs.push_back({FastTrig::cos(sensor_mounts[i].theta), FastTrig::sin(sensor_mounts[i].theta)});
    }

    // Pre-generated noise
    std::normal_distribution<float> dist(0.0f, 1.0f);
    for (int i = 0; i < NOISE_POOL_SIZE; i++) {
        noise_pool[i] = dist(gen);
    }
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

    this->latest_speed = std::hypotf(d_vert_pure, d_horiz_pure) / MSPT * 1000.0f;

    // Sync right after tracking wheel calculations to minimize data loss
    if (autoSync) updateBotPose();

    auto& particles = *particles_ptr;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        auto& p = particles[i];
        float pCos = FastTrig::cos(p.pose.theta + (d_theta/2));
        float pSin = FastTrig::sin(p.pose.theta + (d_theta/2));

        // Get noise
        float vert_noise = 1.0 + next_noise()*TRACKING_WHEEL_VARIANCE;
        float horiz_noise = 1.0 + next_noise()*TRACKING_WHEEL_VARIANCE;

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

void MclTracking::update_weights(const std::vector<float>& sensor_readings, const std::vector<int>& confidences, float current_std_theta) {

    float robotCos = FastTrig::cos(current_std_theta);
    float robotSin = FastTrig::sin(current_std_theta);

    std::vector<float> sigmas_sq_2;
    for(int i = 0; i < sensor_readings.size(); ++i) {
        float s = 0.0f;
        if (sensor_readings[i] < 7.87f) {
            s = BASE_DIST_SIGMA_L787;
        }
        else {
            s = BASE_DIST_SIGMA_G787 * (63.0f / (float)confidences[i]);
        }
        sigmas_sq_2.push_back(2.0f * s * s); // Pre-square and multiply by 2
    }

    auto& particles = *particles_ptr;
    for (int count = 0; count < PARTICLE_COUNT; count++) {
        auto& p = particles[count];
        float combined_prob = 1.0f;

        // Intant penalize if out of bounds
        if (p.pose.x < FIELD_NEG_HALF_LENGTH || p.pose.x > FIELD_HALF_LENGTH || p.pose.y < FIELD_NEG_HALF_LENGTH || p.pose.y > FIELD_HALF_LENGTH) {
            p.weight = 1e-20f;
            continue;
        }

        float diff = p.pose.theta - current_std_theta;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        combined_prob *= std::expf(-(diff * diff) / (2 * HEADING_SIGMA * HEADING_SIGMA));

        for (size_t i = 0; i < sensor_readings.size(); ++i) {
            if (i >= SENSOR_COUNT) break; 
            if (confidences[i] < CONFIDENCE_THRESHOLD || sensor_readings[i] > MAX_RANGE) continue;

            float current_sigma_sq = sigmas_sq_2[i];

            float s_theta = p.pose.theta + sensor_mounts[i].theta;
            float s_x = p.pose.x + (pTrigs[count].cos_m * sensor_mounts[i].x) - (pTrigs[count].sin_m * sensor_mounts[i].y);
            float s_y = p.pose.y + (pTrigs[count].sin_m * sensor_mounts[i].x) + (pTrigs[count].cos_m * sensor_mounts[i].y);

            float p_dist = MAX_RANGE;
            bool hit_hollow = false;

            float rayCos = pTrigs[count].cos_m * mountTrigs[i].cos_m - pTrigs[count].sin_m  * mountTrigs[i].sin_m;
            float raySin = pTrigs[count].sin_m * mountTrigs[i].cos_m + pTrigs[count].cos_m  * mountTrigs[i].sin_m;

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
                float d = intersect_line({s_x, s_y, s_theta}, stlo, MAX_RANGE, rayCos, raySin);
                if (d < p_dist) { p_dist = d; hit_hollow = true; }
            }

            if (hit_hollow) {
                current_sigma_sq *= UNCERTAINTY_TOLERANCE;
            }

            float error = std::abs(sensor_readings[i] - p_dist);
            float prob_match = std::expf(-(error * error) / current_sigma_sq);
            
            combined_prob *= (prob_match + FAULT_TOLERANCE);
        }
        p.weight = combined_prob + 1e-20f;
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
    float step = total_weight / PARTICLE_COUNT;

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
        if (count % LOG_RATIO == 0) {
            while (logging == true) {pros::delay(1);}
            logging = true;
            *mclLog << roundTwoPlaces(p.pose.x) << "," << roundTwoPlaces(p.pose.y) << "\n";
            logging = false;
        }
    }

    // Handle the case where all weights are zero (safety)
    if (total_weight < 1e-20f) return {rawMcl, 0.0}; 

    return {{
        x / total_weight, 
        y / total_weight, 
        std::atan2(sin_sum, cos_sum)
    }, total_weight*total_weight/weight_sqr_sum};
}

Pose MclTracking::step(float vex_theta, const std::vector<float>& dists, const std::vector<int>& confs) {
    
    float std_theta = vexToStd(vex_theta);
    predict(std_theta); // Also syncs position back to lemlib
    update_weights(dists, confs, std_theta);

    // Log
    logMcl();
    lemlib::Pose p (rawMcl.x, rawMcl.y, stdToVex(rawMcl.theta));
    for (auto x: RclSensor::sensorCollection) {
        x->updatePose(p);
        x->logPos(mclLog);
    }

    auto estimate = get_estimate(); // (Logs Particles)

    // Prevent resampling during rotations at a single point
    float distSinceResample = std::hypotf(estimate.first.x - lastResamplePose.x, estimate.first.y - lastResamplePose.y);

    if ((estimate.second < RESAMPLE_THRESHOLD / 2.0f) || (estimate.second < RESAMPLE_THRESHOLD && distSinceResample > MIN_DIST_FROM_RESAMPLE && latest_speed < MAX_VELO_RESAMPLE)) {
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
    std::uniform_real_distribution<float> t_dist(-std::numbers::pi, std::numbers::pi);

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

Pose MclTracking::updateMcl() {
    // Get Sensors
    std::vector<float> dists = {distance_collection[0]->get()*mmToInch, distance_collection[1]->get()*mmToInch, distance_collection[2]->get()*mmToInch};
    std::vector<int> confs = {distance_collection[0]->get_confidence(), distance_collection[1]->get_confidence(), distance_collection[2]->get_confidence()};

    // Update Filter
    rawMcl = step(chassis->getPose().theta, dists, confs);

    return rawMcl;
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

void MclTracking::stopTracking() {
    if (MclTrackingTask != nullptr) { MclTrackingTask->remove(); delete MclTrackingTask; MclTrackingTask = nullptr; }
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
    float std_dev_x = std::sqrtf(sum_sq_diff_x / PARTICLE_COUNT);
    float std_dev_y = std::sqrtf(sum_sq_diff_y / PARTICLE_COUNT);

    // Circular standard deviation for heading
    float R = std::hypotf(sum_sin / PARTICLE_COUNT, sum_cos / PARTICLE_COUNT);
    float std_dev_theta = std::sqrtf(-2.0f * std::log(R)); 

    // Log to CSV
    while (logging == true) {pros::delay(1);}
    logging = true;
    *mclLog << mclLogTimer.elapsed(TimeUnit::SECOND) << "\n";
    *mclLog << roundTwoPlaces(rawMcl.x) << "," << roundTwoPlaces(rawMcl.y) << "," << roundTwoPlaces(rawMcl.theta) << "\n";
    *mclLog << roundTwoPlaces(std_dev_x) << "," << roundTwoPlaces(std_dev_y) << "," << roundTwoPlaces(std_dev_theta) << "\n";
    logging = false;
}

void MclTracking::setDrift(float verticalDriftPerSec, float horizontalDriftPerSec) {
    this->vertical_drift = verticalDriftPerSec / (1000.0f / MSPT);
    this->horizontal_drift = horizontalDriftPerSec / (1000.0f / MSPT);
}

MclTracking::~MclTracking() {
    stopTracking();
}