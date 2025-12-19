#include "custom/MclTracking.hpp"
#include "custom/RclTracking.hpp"

#include <algorithm>

// -------- ctor / init --------
MclTracking::MclTracking(lemlib::Chassis* chassisPtr, int numParticles)
: chassis(chassisPtr), N(numParticles) {
    rng.seed(pros::millis());
    initParticles();
}

void MclTracking::initParticles(double posSpreadIn, double angSpreadRad) {
    // seed around current chassis pose
    lemlib::Pose c = chassis->getPose();
    initParticles(c, posSpreadIn, angSpreadRad);
}

void MclTracking::initParticles(const lemlib::Pose& center, double posSpreadIn, double angSpreadRad) {
    particles.clear();
    particles.reserve(N);
    for (int i = 0; i < N; ++i) {
        MclParticle p;
        p.x     = center.x + randNormal(0.0, posSpreadIn);
        p.y     = center.y + randNormal(0.0, posSpreadIn);
        p.theta = center.theta + randNormal(0.0, angSpreadRad);
        // wrap theta
        if (p.theta >  M_PI) p.theta -= 2*M_PI;
        if (p.theta < -M_PI) p.theta += 2*M_PI;
        p.weight = 1.0 / N;
        particles.push_back(p);
    }
}

// -------- tuning --------
void MclTracking::setMotionNoise(double linearIn, double rotRad) {
    motionNoiseIn = std::max(0.0, linearIn);
    rotNoiseRad   = std::max(0.0, rotRad);
}
void MclTracking::setSensorNoise(double sensorIn) {
    sensorNoiseIn = std::max(1e-6, sensorIn);
}

// -------- random helpers --------
double MclTracking::randNormal(double mean, double stddev) {
    std::normal_distribution<double> d(mean, stddev);
    return d(rng);
}
double MclTracking::randUniform(double lo, double hi) {
    std::uniform_real_distribution<double> d(lo, hi);
    return d(rng);
}

// -------- MCL steps --------
void MclTracking::motionUpdate(double dx_in, double dy_in, double dtheta_rad) {
    // dx,dy are field-frame deltas from LemLib pose; add directly
    for (auto& p : particles) {
        p.x     += dx_in + randNormal(0.0, motionNoiseIn);
        p.y     += dy_in + randNormal(0.0, motionNoiseIn);
        p.theta += dtheta_rad + randNormal(0.0, rotNoiseRad);
        // keep theta bounded
        if (p.theta >  M_PI) p.theta -= 2*M_PI;
        if (p.theta < -M_PI) p.theta += 2*M_PI;
        // clamp to field-ish box with soft walls
        p.x = std::clamp(p.x, -FIELD_HALF, FIELD_HALF);
        p.y = std::clamp(p.y, -FIELD_HALF, FIELD_HALF);
    }
}

double MclTracking::simulateDistance(const MclParticle& p, const SensorGeo& sg) const {
    // Reproduce your RCL geometry: compute sensor world position from (offsetDist, offsetAngle)
    // RCL used: offTheta = deg2rad(offsetAngle - botPose.thetaDeg); sp.x = x + cos(offTheta)*offsetDist; sp.y = y + sin(offTheta)*offsetDist
    // And sensor ray heading: sp.headingDeg = botHeadingDeg + mainAngleDeg
    const double botHeadingDeg = rad2deg(p.theta);
    const double offThetaRad   = deg2rad( botToTrig(botHeadingDeg) + sg.offsetAngleDeg );
    const double sx = p.x + std::cos(offThetaRad) * sg.offsetDistIn;
    const double sy = p.y + std::sin(offThetaRad) * sg.offsetDistIn;

    const double rayDeg = botToTrig(botHeadingDeg + sg.mainAngleDeg);
    const double ang    = deg2rad(rayDeg);
    const double dx     = std::cos(ang);
    const double dy     = std::sin(ang);

    // Intersect with square walls x = ±FIELD_HALF, y = ±FIELD_HALF (inches)
    double tMin = 1e9;

    // Circular obstacles
    // Return Dist - Radius as estimation
    bool intersects = false;
    for (auto circle: Circle_Obstacle::obstacleCollection) {
        SensorPose s;
        s.x = sx;
        s.y = sy;
        s.heading = std::fmod(botHeadingDeg + sg.mainAngleDeg, 360.0);
        s.slope = std::tan(ang);
        s.yIntercept = sy - s.slope * sx;

        if (circle->isIntersecting(s)) {
            return std::sqrt(std::pow(sx-circle->x, 2)+std::pow(sy-circle->y, 2))-circle->radius;
        }
    }

    // x walls
    if (std::abs(dx) > 1e-6) {
        double tx1 = ( FIELD_HALF - sx) / dx;
        double tx2 = (-FIELD_HALF - sx) / dx;
        if (tx1 > 0) tMin = std::min(tMin, tx1);
        if (tx2 > 0) tMin = std::min(tMin, tx2);
    }
    // y walls
    if (std::abs(dy) > 1e-6) {
        double ty1 = ( FIELD_HALF - sy) / dy;
        double ty2 = (-FIELD_HALF - sy) / dy;
        if (ty1 > 0) tMin = std::min(tMin, ty1);
        if (ty2 > 0) tMin = std::min(tMin, ty2);
    }

    // If no positive intersection found, return a large distance
    if (tMin == 1e9 || tMin < 0) return FIELD_HALF * 3.0;
    return tMin;
}

void MclTracking::measurementUpdate(double back_in, double left_in, double right_in) {
    // back/left/right are in inches already
    const double sigma2 = sensorNoiseIn * sensorNoiseIn;

    double total = 0.0;
    for (auto& p : particles) {
        // Predicted distances for each sensor
        const double pb = simulateDistance(p, backS);
        const double pl = simulateDistance(p, leftS);
        const double pr = simulateDistance(p, rightS);

        // Gaussian likelihoods (independent)
        const double db = pb - back_in;
        const double dl = pl - left_in;
        const double dr = pr - right_in;

        const double wb = std::exp(-0.5 * (db*db) / sigma2);
        const double wl = std::exp(-0.5 * (dl*dl) / sigma2);
        const double wr = std::exp(-0.5 * (dr*dr) / sigma2);

        p.weight = wb * wl * wr;
        total += p.weight;
    }

    // Normalize; if degenerate, reset uniform
    if (total < 1e-12) {
        const double w = 1.0 / N;
        for (auto& p : particles) p.weight = w;
        return;
    }
    for (auto& p : particles) p.weight /= total;
}

void MclTracking::resample() {
    // Low-variance (systematic) resampling
    std::vector<MclParticle> newP;
    newP.reserve(N);

    double r = randUniform(0.0, 1.0 / N);
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < N; ++m) {
        const double U = r + (double)m / N;
        while (U > c && i < N - 1) {
            ++i;
            c += particles[i].weight;
        }
        newP.push_back(particles[i]);
        newP.back().weight = 1.0 / N; // reset weight
    }
    particles.swap(newP);
}

void MclTracking::update(double dx_in, double dy_in, double dtheta_rad,
                         double back_mm, double left_mm, double right_mm) {
    // 1) motion
    motionUpdate(dx_in, dy_in, dtheta_rad);

    // 2) measurement (convert mm->in)
    const double back_in  = back_mm  * MM_TO_IN;
    const double left_in  = left_mm  * MM_TO_IN;
    const double right_in = right_mm * MM_TO_IN;
    measurementUpdate(back_in, left_in, right_in);

    // 3) resample
    resample();
}

lemlib::Pose MclTracking::getPoseEstimate() const {
    // Weighted mean of x,y; mean of angle via sin/cos
    double x=0, y=0, s=0, c=0;
    for (const auto& p : particles) {
        x += p.x * p.weight;
        y += p.y * p.weight;
        s += std::sin(p.theta) * p.weight;
        c += std::cos(p.theta) * p.weight;
    }
    double th = std::atan2(s, c);
    // Convert to LemLib Pose (θ in radians)
    return lemlib::Pose{ static_cast<float>(x), static_cast<float>(y), static_cast<float>(th) };
}