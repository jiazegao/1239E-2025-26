// RclTracking.cpp
// Description: This file implements the RCL Tracking system for a robot, including
//              sensor management, obstacle detection, position tracking and syncing
//              using VEX V5 distance sensors.
//              Most functionalities are achieved through basic sensor fusion and
//              intersection math.

#include "custom/RclTracking.hpp"
#include <cstdint>

// Timer class for timeouts
Timer::Timer(double timeoutMs_) : timeoutMs(timeoutMs_), startTime(std::chrono::high_resolution_clock::now()) {}

void Timer::reset() {
    startTime = std::chrono::high_resolution_clock::now();
}

void Timer::hardReset(double newTimeoutMs) {
    timeoutMs = newTimeoutMs;
    reset();
}

bool Timer::timeIsUp() const {
    return elapsedMs() > timeoutMs;
}
    
int Timer::timeLeft() const {
    double e = elapsedMs();
    return e < timeoutMs ? std::round(timeoutMs - e) : 0;
}
    
double Timer::elapsed(TimeUnit unit) const {
    double ms = elapsedMs();
    return (unit == TimeUnit::SECOND) ? (ms / 1000.0) : ms;
}

double Timer::elapsedMs() const {
    auto dt = std::chrono::high_resolution_clock::now() - startTime;
    return std::chrono::duration<double, std::milli>(dt).count();
}

// Line obstacle class
Singly_Linked_List<Line_Obstacle> Line_Obstacle::obstacleCollection = Singly_Linked_List<Line_Obstacle>();
Line_Obstacle::Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs)
    : lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    // Init line
    line.pt1[0] = x1;
    line.pt1[1] = y1;
    line.pt2[0] = x2;
    line.pt2[1] = y2;
    line.slope = (y2 - y1) / (x2 - x1);
    line.yIntercept = y1 - line.slope * x1;
    // Add to collection if space available
    Line_Obstacle::obstacleCollection.add_front(this);
}

// Check if the obstacle has expired
bool Line_Obstacle::expired() {
    return lifeTimer.timeIsUp();
}

// Check if sensor ray intersects this obstacle line
bool Line_Obstacle::isIntersecting(const SensorPose& sp) const {
    // Calculate intersection point
    double xi = (sp.yIntercept - line.yIntercept) / (line.slope - sp.slope);
    double yi = line.slope * xi + line.yIntercept;

    // Check if intersection is within the line segment
    if ((xi < std::min(line.pt1[0], line.pt2[0]) || xi > std::max(line.pt1[0], line.pt2[0])) ||
        (yi < std::min(line.pt1[1], line.pt2[1]) || yi > std::max(line.pt1[1], line.pt2[1])))
        return false;

    // Check if the intersection point is in the direction of the sensor ray
    double dx = xi - sp.x;
    double dy = yi - sp.y;
    double ang = degToRad(sp.heading);
    return (dx * std::cos(ang) > 0) || (dy * std::sin(ang) > 0);
}

// Polygon obstacles
void Line_Obstacle::addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs) {
    // If the points are not enough to form a polygon, do nothing
    if (points.size() < 3) return;
    // Create a line for each pair of points
    for (size_t i = 0; i < points.size(); ++i) {
        size_t next = (i + 1) % points.size();
        new Line_Obstacle(points[i].first, points[i].second, points[next].first, points[next].second, lifeTimeMs);
    }
}

// Circle obstacle class
Singly_Linked_List<Circle_Obstacle> Circle_Obstacle::obstacleCollection = Singly_Linked_List<Circle_Obstacle>();
Circle_Obstacle::Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs)
    : x(x_), y(y_), radius(r_), lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    Circle_Obstacle::obstacleCollection.add_front(this);
}

// Check if the obstacle expired
bool Circle_Obstacle::expired() {
    return lifeTimer.timeIsUp();
}

// Check if sensor ray intersects this obstacle circle
bool Circle_Obstacle::isIntersecting(const SensorPose& sp) const {
    double mPerp = -1.0 / sp.slope;
    double bPerp = y - mPerp * x;
    double xi = (bPerp - sp.yIntercept) / (sp.slope - mPerp);
    double yi = sp.slope * xi + sp.yIntercept;
    double d = std::hypot(xi - x, yi - y);
    if (d > radius) return false;
    double dx = xi - sp.x;
    double dy = yi - sp.y;
    double ang = degToRad( botToTrig(sp.heading) );
    return (dx * std::cos(ang) > 0) || (dy * std::sin(ang) > 0);
}

// RCL sensor class
std::vector<RclSensor*> RclSensor::sensorCollection = std::vector<RclSensor*>();
RclSensor::RclSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol)
    : sensor(distSensor), mainAngle(mainAng), angleTolerance(std::abs(angleTol)) {
    offsetDist = std::hypot(horizOffset, vertOffset);
    offsetAngle = std::fmod(((std::atan2(vertOffset, horizOffset)*180.0/M_PI) + 360), 360);
    RclSensor::sensorCollection.push_back(this);
}

// Compute pose of the ray origin & slope
void RclSensor::updatePose(const lemlib::Pose& botPose) {
    // sensor's angle position relative to the bot's center
    double theta = degToRad(offsetAngle - botPose.theta);
    sp.x = botPose.x + std::cos(theta) * offsetDist;
    sp.y = botPose.y + std::sin(theta) * offsetDist;
    // sensor ray's heading
    sp.heading = std::fmod(botPose.theta + mainAngle, 360.0);
    if (sp.heading <= 0) sp.heading += 360.0;  // avoid 0 or negative
    if (sp.heading == 180) sp.heading += 0.00001;  // avoid 180
    // slope and y-intercept
    sp.slope = std::tan(degToRad(botToTrig(sp.heading)));
    sp.yIntercept = sp.y - sp.slope * sp.x;
}

bool RclSensor::isValid(double distVal) const {
    if (distVal > 2000) return false;  // Invalid Distance
    if (distVal > 200 && this->sensor->get_confidence() < 60) return false; // Invalid confidence
    if (std::abs( std::fmod(this->sp.heading, 90.0) ) > angleTolerance &&
        std::abs( std::fmod(this->sp.heading, 90.0) ) < (90 - angleTolerance)) return false;   // Limit heading to intervals around 90 degrees to increase accuracy
    // Detect if the ray is intersecting with any circular obstacles
    for (auto* item : Circle_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }
    // Detect if the ray is intersectign with any line obstacles
    for (auto* item : Line_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }

    return true;
}

// Return which coordinate (X or Y) and its value
std::pair<CoordType, double> RclSensor::getBotCoord(const lemlib::Pose& botPose, double accum) {

    // Update sensor pose
    this->updatePose(botPose);

    // accumulative?
    double val = std::isnan(accum) ? sensor->get() : accum;

    // verify sensor data
    if (!isValid(val)) return {CoordType::INVALID, 0.0};

    val *= mmToInch;

    // Determine which wall
    int wall;
    double heading = sp.heading;
    if (heading == 90.0) wall = 2;
    else if (heading == 270.0) wall = 4;
    else if (heading < 90.0 || heading > 270.0) {
        double tx = (FIELD_HALF_LENGTH - sp.yIntercept) / sp.slope;
        wall = (tx <= FIELD_NEG_HALF_LENGTH ? 4 : (tx < FIELD_HALF_LENGTH ? 1 : 2));
    }
    else {
        double tx = (FIELD_NEG_HALF_LENGTH - sp.yIntercept) / sp.slope;
        wall = (tx <= FIELD_NEG_HALF_LENGTH ? 4 : (tx < FIELD_HALF_LENGTH ? 3 : 2));
    }

    // Compute coordinate
    double res;
    CoordType type;
    double angRad = degToRad(sp.heading);
    if (wall == 1) { type = CoordType::Y; res = FIELD_HALF_LENGTH - std::cos(angRad) * val; }
    else if (wall == 2) { type = CoordType::X; res = FIELD_HALF_LENGTH - std::sin(angRad) * val; }
    else if (wall == 3) { type = CoordType::Y; res = FIELD_NEG_HALF_LENGTH - std::cos(angRad) * val; }
    else if (wall == 4) { type = CoordType::X; res = FIELD_NEG_HALF_LENGTH - std::sin(angRad) * val; }
    else return {CoordType::INVALID, 0.0};  // Error case

    // Adjust by offset
    double offRad = degToRad(offsetAngle - botPose.theta);
    if (type == CoordType::X) res -= std::cos(offRad) * offsetDist;
    else if (type == CoordType::Y) res -= std::sin(offRad) * offsetDist;

    return {type, res};
}

int RclSensor::rawReading() const { return sensor->get(); }
SensorPose RclSensor::getPose() const { return sp; }

// Main RCL Tracking
RclTracking::RclTracking(lemlib::Chassis* chassis_,
            int frequencyHz_,
            bool autoSync_,
            double minDelta_,
            double maxDelta_,
            double maxDeltaFromLemlib_,
            double maxSyncPerSec_,
            int minPause_)
    : chassis(chassis_),
        goalMSPT(std::round(1000.0 / frequencyHz_)),
        minPause(minPause_),
        maxSyncPT(maxSyncPerSec_ / frequencyHz_),
        minDelta(minDelta_),
        maxDelta(maxDelta_),
        maxDeltaFromLemlib(maxDeltaFromLemlib_),
        autoSync(autoSync_),
        latestPrecise{0, 0, 0},
        poseAtLatest{0, 0, 0} {}

// Start background task
void RclTracking::startTracking() {
    // Start position update loop
    if (mainLoopTask == nullptr)  {
        mainLoopTask = new pros::Task([this](){ this->mainLoop(); });
    }
    // Start miscellaneous update loop
    if (miscLoopTask == nullptr) {
        miscLoopTask = new pros::Task([this](){ this->miscLoop(); });
    }
}
void RclTracking::stopTracking() {
    // Stop both loops if possible
    if (mainLoopTask != nullptr) { mainLoopTask->remove(); delete mainLoopTask; mainLoopTask = nullptr; }
    if (miscLoopTask != nullptr) { miscLoopTask->remove(); delete miscLoopTask; miscLoopTask = nullptr; } 
}
// Accessors
lemlib::Pose RclTracking::getRclPose() const {
    auto chassisPose = chassis->getPose();
    return { latestPrecise.x + (chassisPose.x - poseAtLatest.x),
                latestPrecise.y + (chassisPose.y - poseAtLatest.y),
                chassisPose.theta };
}
void RclTracking::setRclPose(const lemlib::Pose& p) {
    latestPrecise = p;
    poseAtLatest = chassis->getPose();
}
void RclTracking::updateBotPose() {
    auto p = getRclPose();
    chassis->setPose(p);
    setRclPose(p);
}
void RclTracking::updateBotPose(RclSensor* sens) {
    if (sens != nullptr && chassis != nullptr) {
        // Get data from the target sensor
        auto data = sens->getBotCoord(chassis->getPose());
        // Retrieve Lemlib pose as the base
        auto pose = chassis->getPose();

        // Update pose based on sensor reading
        if (data.first == CoordType::X) pose.x = data.second;
        else if (data.first == CoordType::Y) pose.y = data.second;

        // Sync to Lemlib
        chassis->setPose({pose.x, pose.y, chassis->getPose().theta});
        setRclPose({pose.x, pose.y, chassis->getPose().theta});
    }
}

// Accumulation control
void RclTracking::startAccumulating(bool autoUpdateAfterAccum) { accumulating = true; updateAfterAccum = autoUpdateAfterAccum; }
void RclTracking::stopAccumulating() { accumulating = false; }
void RclTracking::accumulateFor (int ms, bool autoUpdateAfterAccum) {
    startAccumulating( autoUpdateAfterAccum );
    Timer t(ms);
    while (!t.timeIsUp()) { pros::delay(minPause); }
    stopAccumulating();
}

// Reset Rcl
void RclTracking::discardData () {
    latestPrecise = chassis->getPose();
    poseAtLatest = chassis->getPose();
}

// Single updates
void RclTracking::mainUpdate() {
    // Verify that there is at least one sensor
    if (RclSensor::sensorCollection.size() > 0) {
        // Accumulators
        std::vector<int> accTotal(RclSensor::sensorCollection.size());
        std::vector<int> accCount(RclSensor::sensorCollection.size());
        // If accumulating, gather readings
        while (accumulating) {
            for (int i = 0; i < RclSensor::sensorCollection.size(); i++) {
                accTotal[i] += RclSensor::sensorCollection[i]->rawReading();
                accCount[i] ++;
            }
            pros::delay(goalMSPT);
        }

        // Collections
        std::vector<double> xs, ys;

        // Make sure RclPosition doesn't deviate too much from the chassis position
        auto botPose = getRclPose();
        double diff_from_lemlib = std::hypot(botPose.x-chassis->getPose().x, botPose.y-chassis->getPose().y);
        if (diff_from_lemlib > maxDeltaFromLemlib) {
            botPose.x += (chassis->getPose().x-botPose.x) / diff_from_lemlib;
            botPose.y += (chassis->getPose().y-botPose.y) / diff_from_lemlib;
        }

        // Loop sensors
        for (int i = 0; i < RclSensor::sensorCollection.size(); i++) {
            auto sens = RclSensor::sensorCollection[i];

            double avg = (accCount[i] > 0) ? (1.0 * accTotal[i] / accCount[i]) : NAN;
            auto [type, coord] = sens->getBotCoord(botPose, avg);

            // Validate and collect
            if (type == CoordType::X) {
                double diff = std::abs(coord - botPose.x);
                if (diff <= maxDelta) xs.push_back(coord);
            }
            else if (type == CoordType::Y) {
                double diff = std::abs(coord - botPose.y);
                if (diff <= maxDelta) ys.push_back(coord);
            }
        }

        // Update means
        if (!xs.empty()) {
            double meanX = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
            if (meanX > FIELD_NEG_HALF_LENGTH && meanX < FIELD_HALF_LENGTH && std::abs(meanX-botPose.x) >= minDelta) {
                latestPrecise.x = meanX;
                poseAtLatest.x = chassis->getPose().x;
            }
        }
        if (!ys.empty()) {
            double meanY = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
            if (meanY > FIELD_NEG_HALF_LENGTH && meanY < FIELD_HALF_LENGTH && std::abs(meanY-botPose.y) >= minDelta) {
                latestPrecise.y = meanY;
                poseAtLatest.y = chassis->getPose().y;
            }
        }

        // Determine if bot position should be automatically updated
        if (updateAfterAccum && std::any_of(accCount.begin(), accCount.end(), [](int c){ return c>0; }))
            updateBotPose();
    }
}
void RclTracking::syncUpdate() {
    // variables
    double x_diff = 0.0;
    double y_diff = 0.0;
    double real_diff = 0.0;

    double x_update = 0.0;
    double y_update = 0.0;

    // retrive rcl-based position
    lemlib::Pose currRclPosition = getRclPose();

    // calculate differences
    x_diff = currRclPosition.x - chassis->getPose().x;
    y_diff = currRclPosition.y - chassis->getPose().y;
    real_diff = std::hypot(std::abs(x_diff), std::abs(y_diff));

    // If within maximum sync distance, update the chassis pose directly
    if (real_diff <= maxSyncPT) {
        chassis->setPose(currRclPosition.x, currRclPosition.y, chassis->getPose().theta);
        
        poseAtLatest.x += x_diff;
        poseAtLatest.y += y_diff;
        poseAtLatest.theta = chassis->getPose().theta;
    }
    // Otherwise, only sync part of the discrepency
    else {
        // Calculate the updating amount
        x_update = x_diff / real_diff * std::sqrt(maxSyncPT);
        y_update = y_diff / real_diff * std::sqrt(maxSyncPT);

        // Update (Sync)
        chassis->setPose(chassis->getPose().x+x_update, chassis->getPose().y+y_update, chassis->getPose().theta);
        
        poseAtLatest.x += x_update;
        poseAtLatest.y += y_update;
        poseAtLatest.theta = chassis->getPose().theta;
    }
}
void RclTracking::lifeTimeUpdate() {
    // Clean circular obstacles
    auto circle_itr = Circle_Obstacle::obstacleCollection.begin();
    while (circle_itr != Circle_Obstacle::obstacleCollection.end()) {
        if ((**circle_itr).expired()) circle_itr.remove(true);
        else ++circle_itr;
    }

    // Clean line obstacles
    auto line_itr = Line_Obstacle::obstacleCollection.begin();
    while (line_itr != Line_Obstacle::obstacleCollection.end()) {
        if ((**line_itr).expired()) line_itr.remove(true);
        else ++line_itr;
    }
}

// Update loops
void RclTracking::mainLoop() {
    Timer frequencyTimer(goalMSPT);    // Create timer for frequency

    while (true) {
        // Reset frequency timer
        frequencyTimer.reset();

        // Call the update function
        mainUpdate();

        // Delay to save resources
        if ( frequencyTimer.timeLeft() < minPause ) pros::delay(minPause); // Ensure that the loop pauses at least for minPause
        else pros::delay(frequencyTimer.timeLeft()); // Otherwise, wait for the remaining time
    }
}
void RclTracking::miscLoop() {

    Timer frequencyTimer(goalMSPT);    // Create timer for frequency

    while (true) {

        frequencyTimer.reset(); // Reset frequency timer

        // main update functions
        if (autoSync) { syncUpdate(); }
        // lifeTimeUpdate();

        // Pause for the remaining time
        if (frequencyTimer.timeLeft() < minPause) pros::delay(minPause); // Ensure that the loop pauses at least for minPause
        else pros::delay(frequencyTimer.timeLeft()); // Otherwise, wait for the remaining time
    }
}