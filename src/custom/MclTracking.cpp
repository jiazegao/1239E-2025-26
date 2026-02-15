#include "MclTracking.hpp"
#include "Tracking_Util.hpp"
#include "configs.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

inline double roundTwoPlaces(double x) {
    return std::round(x*100)/100;
}

double MclTracking::intersect_line(Pose ray, Line_ wall, double max_range, double rayCos, double raySin) {

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

double MclTracking::intersect_circle(Pose ray, Circle c, double max_range, double dx, double dy) {
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

MclTracking::MclTracking(lemlib::Chassis* chassis, std::vector<pros::Distance*> dist_collection, std::tuple<pros::Rotation*, double, double> vertical_tracking_wheel, std::tuple<pros::Rotation*, double, double> horizontal_tracking_wheel, double start_x, double start_y, double start_vex_theta, bool autoSync_) {
    this->chassis = chassis;
    this->distance_collection = dist_collection;
    this->autoSync = autoSync_;

    this->vertical_tracking_wheel = get<0>(vertical_tracking_wheel);
    this->vert_c = get<1>(vertical_tracking_wheel)*std::numbers::pi;
    this->vert_offset = get<2>(vertical_tracking_wheel);
    this->last_vertical_reading = this->vertical_tracking_wheel->get_position()/100.0;

    this->horizontal_tracking_wheel = get<0>(horizontal_tracking_wheel);
    this->horiz_c = get<1>(horizontal_tracking_wheel)*std::numbers::pi;
    this->horiz_offset = get<2>(horizontal_tracking_wheel);
    this->last_horizontal_reading = (-1)*this->horizontal_tracking_wheel->get_position()/100.0;

    std::random_device rd;
    gen = std::mt19937(rd());
    
    double start_std_theta = vexToStd(start_vex_theta);
    this->lastTheta = start_std_theta;
    std::normal_distribution<double> x_init(start_x, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<double> y_init(start_y, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<double> t_init(start_std_theta, THETA_RESAMPLE_VARIANCE);

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

void MclTracking::predict(double current_std_theta) {
    // Update theta deviation
    double d_theta = current_std_theta - lastTheta;
    while (d_theta > M_PI) d_theta -= 2 * M_PI;
    while (d_theta < -M_PI) d_theta += 2 * M_PI;
    lastTheta = current_std_theta;

    // Noise (Avg 0.002 rad / 6%)
    // Less certain if turning
    std::normal_distribution<double> theta_noise(0, THETA_RESAMPLE_VARIANCE);
    std::normal_distribution<double> tracking_wheel_noise(1, TRACKING_WHEEL_VARIANCE);

    // Calculate vertical tracking wheel vector
    double vert_reading = vertical_tracking_wheel->get_position()/100.0;
    double d_vert_raw = (vert_reading-last_vertical_reading)/360.0 * vert_c;
    last_vertical_reading = vert_reading;

    // Calculate horizontal tracking wheel vector
    double horiz_reading = (-1)*horizontal_tracking_wheel->get_position()/100.0;
    double d_horiz_raw = (horiz_reading-last_horizontal_reading)/360.0 * horiz_c;
    last_horizontal_reading = horiz_reading;

    // Get raw reading
    double d_vert_pure = d_vert_raw - (vert_offset * d_theta);
    double d_horiz_pure = d_horiz_raw + (horiz_offset * d_theta);

    // Sync right after tracking wheel calculations to minimize data loss
    if (autoSync) updateBotPose();

    auto& particles = *particles_ptr;
    for (int i = 0; i < PARTICLE_COUNT; i++) {
        auto& p = particles[i];
        double pCos = std::cos(p.pose.theta + (d_theta/2));
        double pSin = std::sin(p.pose.theta + (d_theta/2));

        // Get noise
        double vert_noise = tracking_wheel_noise(gen);
        double horiz_noise = tracking_wheel_noise(gen);

        // Forward / Backward motion with noise
        double forward_dist = d_vert_pure * vert_noise;
        double strafe_dist = d_horiz_pure * horiz_noise;

        // Update position
        p.pose.x += forward_dist*pCos + strafe_dist*pSin;
        p.pose.y += forward_dist*pSin - strafe_dist*pCos;
        p.pose.theta += d_theta + theta_noise(gen);

        while (p.pose.theta > M_PI) p.pose.theta -= 2 * M_PI;
        while (p.pose.theta < -M_PI) p.pose.theta += 2 * M_PI;

        pTrigs[i] = {std::cos(p.pose.theta), std::sin(p.pose.theta)};
    }
}

void MclTracking::update_weights(const std::vector<double>& sensor_readings, const std::vector<int>& confidences, double current_std_theta) {

    double robotCos = std::cos(current_std_theta);
    double robotSin = std::sin(current_std_theta);

    std::vector<double> sigmas_sq_2;
    for(int i = 0; i < sensor_readings.size(); ++i) {
        double s = 0.0;
        if (sensor_readings[i] < 7.87) {
            s = BASE_DIST_SIGMA_L787;
        }
        else {
            s = BASE_DIST_SIGMA_G787 * (63.0 / (double)confidences[i]) * (std::max(sensor_readings[i], 50.0) / 50.0);
        }
        sigmas_sq_2.push_back(2.0 * s * s); // Pre-square and multiply by 2
    }

    auto& particles = *particles_ptr;
    for (int count = 0; count < PARTICLE_COUNT; count++) {
        auto& p = particles[count];
        double combined_prob = 1.0;

        // Intant penalize if out of bounds
        if (p.pose.x < FIELD_NEG_HALF_LENGTH || p.pose.x > FIELD_HALF_LENGTH || p.pose.y < FIELD_NEG_HALF_LENGTH || p.pose.y > FIELD_HALF_LENGTH) {
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

            if (hit_hollow) {
                current_sigma_sq *= UNCERTAINTY_TOLERANCE;
            }

            double error = std::abs(sensor_readings[i] - p_dist);
            double prob_match = std::exp(-(error * error) / current_sigma_sq);
            
            combined_prob *= (prob_match + FAULT_TOLERANCE);
        }
        p.weight = combined_prob + 1e-300;
    }
}

void MclTracking::resample() {
    std::vector<double> weights;
    auto& particles = *particles_ptr;
    for (const auto& p : particles) weights.push_back(p.weight);

    std::discrete_distribution<int> sampler(weights.begin(), weights.end());
    std::normal_distribution<double> dist_jitter(0, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<double> theta_jitter(0, THETA_RESAMPLE_VARIANCE);

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

std::pair<Pose, double> MclTracking::get_estimate() {
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

        // Log - Log Particle Positions
        if (count % LOG_RATIO == 0) {
            while (logging == true) {pros::delay(1);}
            logging = true;
            *mclLog << roundTwoPlaces(p.pose.x) << "," << roundTwoPlaces(p.pose.y) << "\n";
            logging = false;
        }
    }

    // Handle the case where all weights are zero (safety)
    if (total_weight < 1e-9) return {rawMcl, 0.0}; 

    return {{
        x / total_weight, 
        y / total_weight, 
        std::atan2(sin_sum, cos_sum)
    }, total_weight*total_weight/weight_sqr_sum};
}

Pose MclTracking::step(double vex_theta, const std::vector<double>& dists, const std::vector<int>& confs) {
    
    double std_theta = vexToStd(vex_theta);
    predict(std_theta); // Also syncs position back to lemlib
    update_weights(dists, confs, std_theta);

    // Log
    logMcl();

    auto estimate = get_estimate();

    // Prevent resampling during rotations at a single point
    double distSinceResample = std::hypot(estimate.first.x - lastResamplePose.x, estimate.first.y - lastResamplePose.y);

    if (estimate.second < RESAMPLE_THRESHOLD && distSinceResample > 4.0) {
        resample();
        lastResamplePose = estimate.first;
    }

    return estimate.first;
}

void MclTracking::set_pose(double x, double y, double vex_theta) {
    double std_theta = vexToStd(vex_theta);
    std::normal_distribution<double> x_dist(x, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<double> y_dist(y, DIST_RESAMPLE_VARIANCE);
    std::normal_distribution<double> t_dist(std_theta, THETA_RESAMPLE_VARIANCE);
    lastTheta = std_theta;

    auto& particles = *particles_ptr;
    for (auto& p : particles) {
        p.pose = {x_dist(gen), y_dist(gen), t_dist(gen)};
        p.weight = 1.0;
    }
}

void MclTracking::uniform_reset() {
    std::uniform_real_distribution<double> x_dist(FIELD_NEG_HALF_LENGTH, FIELD_HALF_LENGTH);
    std::uniform_real_distribution<double> y_dist(FIELD_NEG_HALF_LENGTH, FIELD_HALF_LENGTH);
    std::uniform_real_distribution<double> t_dist(-std::numbers::pi, std::numbers::pi);
    lastTheta = 0;

    auto& particles = *particles_ptr;
    for (auto& p : particles) {
        p.pose = {x_dist(gen), y_dist(gen), t_dist(gen)};
        p.weight = 1.0;
    }
}

Pose MclTracking::updateMcl() {
    // Get Sensors
    std::vector<double> dists = {distance_collection[0]->get()*mmToInch, distance_collection[1]->get()*mmToInch, distance_collection[2]->get()*mmToInch};
    std::vector<int> confs = {distance_collection[0]->get_confidence(), distance_collection[1]->get_confidence(), distance_collection[2]->get_confidence()};

    // Update Filter
    rawMcl = step(chassis->getPose().theta, dists, confs);

    return rawMcl;
}

void MclTracking::updateBotPose() {

    lemlib::Pose odomPose = chassis->getPose();
    
    // Interpolate target x
    double newX = odomPose.x + DIST_SYNC_PROP * (rawMcl.x - odomPose.x);
    double newY = odomPose.y + DIST_SYNC_PROP * (rawMcl.y - odomPose.y);

    // Circular interpolation for theta
    // Convert Odom to Std Radians for calculation
    double odomRad = vexToStd(odomPose.theta);
    
    // Calculate shortest difference
    double diff = rawMcl.theta - odomRad;
    
    // Normalize difference to [-PI, PI]
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    // Apply weighted difference
    double newThetaRad = odomRad + (diff * THETA_SYNC_PROP);

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
    // Inside get_estimate() or at the end of step()
    double sum_sq_diff_x = 0, sum_sq_diff_y = 0;
    double sum_sin = 0, sum_cos = 0;

    auto& particles = *particles_ptr;

    for (const auto& p : particles) {
        sum_sq_diff_x += std::pow(p.pose.x - rawMcl.x, 2);
        sum_sq_diff_y += std::pow(p.pose.y - rawMcl.y, 2);
        
        // Use circular statistics for theta
        sum_sin += std::sin(p.pose.theta);
        sum_cos += std::cos(p.pose.theta);
    }

    // Calculate standard deviation
    double std_dev_x = std::sqrt(sum_sq_diff_x / PARTICLE_COUNT);
    double std_dev_y = std::sqrt(sum_sq_diff_y / PARTICLE_COUNT);

    // Circular standard deviation for heading
    double R = std::hypot(sum_sin / PARTICLE_COUNT, sum_cos / PARTICLE_COUNT);
    double std_dev_theta = std::sqrt(-2.0 * std::log(R)); 

    // Log to CSV
    while (logging == true) {pros::delay(1);}
    logging = true;
    *mclLog << mclLogTimer.elapsed(TimeUnit::SECOND) << "\n";
    *mclLog << roundTwoPlaces(rawMcl.x) << "," << roundTwoPlaces(rawMcl.y) << "," << roundTwoPlaces(rawMcl.theta) << "\n";
    *mclLog << roundTwoPlaces(std_dev_x) << "," << roundTwoPlaces(std_dev_y) << "," << roundTwoPlaces(std_dev_theta) << "\n";
    logging = false;
}

MclTracking::~MclTracking() {
    stopTracking();
}