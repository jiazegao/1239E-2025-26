// rclTracking.cpp
// Author: Jiaze Gao, Aiden Kim (based on the Lemlib Library)
// Description: This file implements the RCL Tracking system for a robot, including
//              sensor management, obstacle detection, position tracking and syncing
//              using VEX V5 distance sensors.
//              Most functionalities are achieved through basic sensor fusion and
//              intersection math.

#include "main.h"
#include "lemlib/api.hpp"

#include <chrono>
#include <cmath>
#include <utility>
#include <vector>
#include <numeric>

// Enumerations
enum class TimeUnit { SECOND, MILLISECOND };
enum class CoordType { X, Y, INVALID };

// Timer class for timeouts
class Timer {
    public:
        Timer(double timeoutMs = 0)
            : timeoutMs(timeoutMs), startTime(std::chrono::high_resolution_clock::now()) {}
    
        void reset() {
            startTime = std::chrono::high_resolution_clock::now();
        }
    
        void hardReset(double newTimeoutMs) {
            timeoutMs = newTimeoutMs;
            reset();
        }
    
        bool timeIsUp() const {
            return elapsedMs() > timeoutMs;
        }
    
        int timeLeft() const {
            double e = elapsedMs();
            return e < timeoutMs ? std::round(timeoutMs - e) : 0;
        }
    
        double elapsed(TimeUnit unit = TimeUnit::MILLISECOND) const {
            double ms = elapsedMs();
            return (unit == TimeUnit::SECOND) ? (ms / 1000.0) : ms;
        }
    
    private:
        double timeoutMs;
        std::chrono::high_resolution_clock::time_point startTime;
    
        double elapsedMs() const {
            auto dt = std::chrono::high_resolution_clock::now() - startTime;
            return std::chrono::duration<double, std::milli>(dt).count();
        }
};

// Singly linked list for quick removal / insertion
template <typename AnyType>
class Singly_Linked_List {

public:
    Singly_Linked_List() {
        head->next = tail;
    }

    class Node {
        public:
            Node(AnyType* obj_ptr) {
                this->ptr = obj_ptr;
                this->next = nullptr;
            }
            AnyType* ptr;
            Node* next;
    };

    class Iterator {
    private:
        Node* current;
    public:
        // Constructor
        Iterator(Node* node) : current(node) {}

        // Dereference operator
        AnyType*& operator*() const {
            return current->ptr;
        }

        // Pre-increment operator
        Iterator& operator++() {
            current = current->next;
            return *this;
        }

        // Post-increment operator (optional, but good practice)
        Iterator operator++(int) {
            Iterator temp = *this;
            current = current->next;
            return temp;
        }

        // Equality operator
        bool operator==(const Iterator& other) const {
            return current == other.current;
        }

        // Inequality operator
        bool operator!=(const Iterator& other) const {
            return current != other.current;
        }
    };

    Iterator begin() {
        return Iterator(head->next);
    }

    Iterator end() {
        return Iterator(tail);
    }

    void push_back(AnyType* obj_ptr) {
        insert(length, obj_ptr);
    }

    void insert(int index, AnyType* obj_ptr) {
        if (obj_ptr != nullptr) {
            int temp_index = index;

            if (temp_index >= 0 && temp_index <= length) {
                Node* currNode = this->head;
                while (temp_index > 0){
                    currNode = currNode->next;
                    temp_index--;
                }
                Node* oldNode = currNode->next;
                currNode->next = new Node(obj_ptr);
                currNode->next->next = oldNode;
                length++;
            }
        }
    }

    void pop(int index){
        if (index >= 0 && index < length) {
            Node* currNode = head;
            while (index > 0){
                currNode = currNode->next;
                index--;
            }
            Node* oldNode = currNode->next;
            currNode->next = oldNode->next;
            delete oldNode;
            length--;
        }
    }

    AnyType* get(int index){
        Node* currNode = head;
        if (index >= 0 && index < length) {
            while (index >= 0){
                currNode = currNode->next;
                index--;
            }
        }
        return currNode->ptr;
    }

    void remove(AnyType* obj_ptr) {
        if (obj_ptr != nullptr) {
            Node* currNode = this->head;
            while (currNode->next != nullptr && currNode->next->ptr != obj_ptr){
                currNode = currNode->next;
            }
            if (currNode->next->ptr == obj_ptr) {
                Node* oldNode = currNode->next;
                currNode->next = oldNode->next;
                delete oldNode;
                length--;
            }
        }
    }

    int size(){
        return length;
    }

    void setSize(int size) {
        length = size;
    }
    
    Node* getHead() {
        return this->head;
    }

private:
    Node* head = new Node(nullptr);
    Node* tail = new Node(nullptr);
    int length = 0;
};

// Constants
constexpr double mmToInch = 0.039370078740157;
constexpr double MAX_OBSTACLE_DURATION = 1e9;         // ms
constexpr double FIELD_HALF_LENGTH = 70.5;         // inches
constexpr double FIELD_NEG_HALF_LENGTH = -70.5;

// Utility conversions
inline double degToRad(double deg) { return deg * M_PI / 180.0; }

// Pose of a distance sensor (ray) for intersection math
struct SensorPose {
    double x = 0;
    double y = 0;
    double heading = 0;
    double slope = 0;
    double yIntercept = 0;
};

struct Line {
    double pt1[2] = {0, 0};
    double pt2[2] = {0, 0};
    double slope = 0;
    double yIntercept = 0;
};

// Line obstacle class
class Line_Obstacle {
public:

    static Singly_Linked_List<Line_Obstacle> obstacleCollection;

    Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs = -1)
        : lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
        // Init line
        line.pt1[0] = x1;
        line.pt1[1] = y1;
        line.pt2[0] = x2;
        line.pt2[1] = y2;
        line.slope = (y2 - y1) / (x2 - x1);
        line.yIntercept = y1 - line.slope * x1;
        // Add to collection if space available
        Line_Obstacle::obstacleCollection.push_back(this);
    }

    // Check if the obstacle has expired
    bool expired() {
        return lifeTimer.timeIsUp();
    }

    // Check if sensor ray intersects this obstacle line
    bool isIntersecting(const SensorPose& sp) const {
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

private:
    Line line;
    Timer lifeTimer;
};

// Polygon obstacles
void addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs = -1) {
    // If the points are not enough to form a polygon, do nothing
    if (points.size() < 3) return;
    // Create a line for each pair of points
    for (size_t i = 0; i < points.size(); ++i) {
        size_t next = (i + 1) % points.size();
        new Line_Obstacle(points[i].first, points[i].second, points[next].first, points[next].second, lifeTimeMs);
    }
}

// Circle obstacle class
class Circle_Obstacle {
public:

    static Singly_Linked_List<Circle_Obstacle> obstacleCollection;

    Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs = -1)
        : x(x_), y(y_), radius(r_), lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
        Circle_Obstacle::obstacleCollection.push_back(this);
    }

    // Check if the obstacle expired
    bool expired() {
        return lifeTimer.timeIsUp();
    }

    // Check if sensor ray intersects this obstacle circle
    bool isIntersecting(const SensorPose& sp) const {
        double mPerp = -1.0 / sp.slope;
        double bPerp = y - mPerp * x;
        double xi = (bPerp - sp.yIntercept) / (sp.slope - mPerp);
        double yi = sp.slope * xi + sp.yIntercept;
        double d = std::hypot(xi - x, yi - y);
        if (d > radius) return false;
        double dx = xi - sp.x;
        double dy = yi - sp.y;
        double ang = degToRad(sp.heading);
        return (dx * std::cos(ang) > 0) || (dy * std::sin(ang) > 0);
    }

    
private:
    double x, y, radius;
    Timer lifeTimer;
};

// RCL sensor class
class RclSensor {
public:

    static std::vector<RclSensor*> sensorCollection;

    RclSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol = 10.0)
        : sensor(distSensor), mainAngle(mainAng), angleTolerance(std::abs(angleTol)) {
        offsetDist = std::hypot(horizOffset, vertOffset);
        offsetAngle = std::atan2(vertOffset, horizOffset) * 180.0 / M_PI;
        RclSensor::sensorCollection.push_back(this);
    }

    // Compute pose of the ray origin & slope
    void updatePose(const lemlib::Pose& botPose) {
        // sensor's angle position relative to the bot's center
        double theta = degToRad(offsetAngle - botPose.theta);
        sp.x = botPose.x + std::cos(theta) * offsetDist;
        sp.y = botPose.y + std::sin(theta) * offsetDist;
        // sensor ray's heading
        sp.heading = std::fmod(botPose.theta + mainAngle, 360.0);
        if (sp.heading <= 0) sp.heading += 360.0;  // avoid 0 or negative
        if (sp.heading == 180) sp.heading += 0.00001;  // avoid 180
        // slope and y-intercept
        sp.slope = 1.0 / std::tan(degToRad(sp.heading));
        sp.yIntercept = sp.y - sp.slope * sp.x;
    }

    bool isValid(double distVal) const {
        if (distVal > 390.0) return false;  // Invalid Distance
        if (std::abs( std::fmod(sp.heading, 90.0) ) > angleTolerance &&
            std::abs( std::fmod(sp.heading, 90.0) ) < (90 - angleTolerance)) return false;   // Limit heading to intervals around 90 degrees to increase accuracy
        // Detect if the ray is intersecting with any circular obstacles
        for (auto* item : Circle_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }
        // Detect if the ray is intersectign with any line obstacles
        for (auto* item : Line_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }

        return true;
    }

    // Return which coordinate (X or Y) and its value
    std::pair<CoordType, double> getBotCoord(const lemlib::Pose& botPose, double accum = NAN) {

        // accumulative?
        double val = std::isnan(accum) ? sensor->get() * mmToInch : accum;

        // verify sensor data
        if (!isValid(val)) return {CoordType::INVALID, 0.0};

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

    double rawReading() const { return sensor->get() * mmToInch; }
    SensorPose getPose() const { return sp; }

private:
    pros::Distance* sensor;
    double offsetDist;
    double offsetAngle;
    double mainAngle;
    SensorPose sp;
    double angleTolerance;  // degrees
};

// Main RCL Tracking
class RclTracking {
public:
    RclTracking(lemlib::Chassis* chassis_,
                int frequencyHz_ = 20,
                bool autoUpdate_ = true,
                double minDelta_ = 0.6,
                double maxDelta_ = 6.0,
                double maxSyncPerSec_ = 1.5,
                int minPause_ = 10)
        : chassis(chassis_),
          goalMSPT(std::round(1000.0 / frequencyHz_)),
          minPause(minPause_),
          maxSyncPT(maxSyncPerSec_ / frequencyHz_),
          minDelta(minDelta_),
          maxDelta(maxDelta_),
          autoUpdate(autoUpdate_),
          latestPrecise{0, 0, 0},
          poseAtLatest{0, 0, 0} {}

    // Start background task
    void startTracking() {
        // Start position update loop
        if (!mainLoopRunning) 
            mainLoopTask = pros::Task([this]{ this->mainLoopRunning = true; this->mainLoop(); });
        // Start sync loop
        if (autoUpdate && !syncLoopRunning)
            syncLoopTask = pros::Task([this]{ this->syncLoopRunning = true; this->syncLoop(); });
        // Start lifetime update loop
        if (!lifeLoopRunning)
            lifeLoopTask = pros::Task([this]{ this->lifeLoopRunning = true; this->lifeTimeLoop(); });
    }
    void stopTracking() {
        // Stop both loops if possible
        if (mainLoopRunning) { mainLoopTask.remove(); mainLoopRunning = false; }
        if (syncLoopRunning) { syncLoopTask.remove(); syncLoopRunning = false; }
        if (lifeLoopRunning) { lifeLoopTask.remove(); lifeLoopRunning = false; }
    }

    // Accessors
    lemlib::Pose getRclPosition() const {
        auto chassisPose = chassis->getPose();
        return { latestPrecise.x + (chassisPose.x - poseAtLatest.x),
                 latestPrecise.y + (chassisPose.y - poseAtLatest.y),
                 chassisPose.theta };
    }
    void setRclPosition(const lemlib::Pose& p) {
        latestPrecise = p;
        poseAtLatest = chassis->getPose();
    }
    void updateBotPosition() {
        auto p = getRclPosition();
        chassis->setPose(p);
        setRclPosition(p);
    }

    // Accumulation control
    void startAccumulating(bool autoUpdateAfterAccum = true) { accumulating = true; updateAfterAccum = autoUpdateAfterAccum; }
    void stopAccumulating() { accumulating = false; }
    void accumulateFor (int ms, bool autoUpdateAfterAccum = true) {
        startAccumulating( autoUpdateAfterAccum );
        Timer t(ms);
        while (!t.timeIsUp()) { pros::delay(minPause); }
        stopAccumulating();
    }

    // Reset Rcl
    void discardData () {
        latestPrecise = chassis->getPose();
        poseAtLatest = chassis->getPose();
    }

    // Single updates
    void mainUpdate() {
        // Accumulators
        std::vector<double> accTotal(RclSensor::sensorCollection.size());
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

        // Loop sensors
        auto botPose = getRclPosition();
        for (int i = 0; i < RclSensor::sensorCollection.size(); i++)
        {
            auto sens = RclSensor::sensorCollection[i];
            sens->updatePose(botPose);

            double avg = (accCount[i] > 0) ? (accTotal[i] / accCount[i]) : NAN;
            auto [type, coord] = sens->getBotCoord(botPose, avg);

            // Validate and collect
            if (type == CoordType::X)
            {
                double diff = std::abs(coord - botPose.x);
                if (diff <= maxDelta && (diff >= minDelta || accCount[i] > 0)) xs.push_back(coord);
            }
            else if (type == CoordType::Y)
            {
                double diff = std::abs(coord - botPose.y);
                if (diff <= maxDelta && (diff >= minDelta || accCount[i] > 0)) ys.push_back(coord);
            }
        }

        // Update means
        if (!xs.empty())
        {
            double meanX = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
            if (meanX > FIELD_NEG_HALF_LENGTH && meanX < FIELD_HALF_LENGTH) latestPrecise.x = meanX, poseAtLatest.x = chassis->getPose().x;
        }
        if (!ys.empty())
        {
            double meanY = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
            if (meanY > FIELD_NEG_HALF_LENGTH && meanY < FIELD_HALF_LENGTH) latestPrecise.y = meanY, poseAtLatest.y = chassis->getPose().y;
        }

        // Determine if bot position should be automatically updated
        if (updateAfterAccum && std::any_of(accCount.begin(), accCount.end(), [](int c){ return c>0; }))
            updateBotPosition();
    }
    void syncUpdate() {
        // variables
        double x_diff = 0.0;
        double y_diff = 0.0;
        double real_diff = 0.0;

        double x_update = 0.0;
        double y_update = 0.0;

        // retrive rcl-based position
        lemlib::Pose currRclPosition = getRclPosition();

        // calculate differences
        x_diff = currRclPosition.x - chassis->getPose().x;
        y_diff = currRclPosition.y - chassis->getPose().y;
        real_diff = std::hypot(std::abs(x_diff), std::abs(y_diff));

        // If within maximum sync distance, update the chassis pose directly
        if (real_diff <= maxSyncPT) {
            chassis->setPose(currRclPosition.x, currRclPosition.y, chassis->getPose().theta);
        }
        // Otherwise, only sync part of the discrepency
        else {
            // Calculate the updating amount
            x_update = x_diff / real_diff * std::sqrt(maxSyncPT);
            y_update = y_diff / real_diff * std::sqrt(maxSyncPT);

            // Update (Sync)
            chassis->setPose(chassis->getPose().x+x_update, chassis->getPose().y+y_update, chassis->getPose().theta);
        }
    }
    void lifeTimeUpdate() {
        // Clean circular obstacles
        auto* currCircle = Circle_Obstacle::obstacleCollection.getHead();
        while (currCircle->next != nullptr) {
            if (currCircle->next->ptr->expired()) {
                auto* oldNode = currCircle->next;
                currCircle->next = oldNode->next;
                delete oldNode->ptr;
                delete oldNode;
                Circle_Obstacle::obstacleCollection.setSize(Circle_Obstacle::obstacleCollection.size()-1);
            }
            else {
                currCircle = currCircle->next;
            }
        }
        
        // Clean line obstacles
        auto* currLine = Line_Obstacle::obstacleCollection.getHead();
        while (currLine->next != nullptr) {
            if (currLine->next->ptr->expired()) {
                auto* oldNode = currLine->next;
                currLine->next = oldNode->next;
                delete oldNode->ptr;
                delete oldNode;
                Line_Obstacle::obstacleCollection.setSize(Line_Obstacle::obstacleCollection.size()-1);
            }
            else {
                currLine = currLine->next;
            }
        }
    }

private:
    lemlib::Chassis* chassis;
    int goalMSPT;
    int minPause;
    double maxSyncPT;
    double minDelta, maxDelta;
    bool autoUpdate;
    bool accumulating;
    pros::Task mainLoopTask = pros::Task([](){});
    bool mainLoopRunning = false;
    pros::Task syncLoopTask = pros::Task([](){});
    bool syncLoopRunning = false;
    pros::Task lifeLoopTask = pros::Task([](){});
    bool lifeLoopRunning = false;
    lemlib::Pose latestPrecise, poseAtLatest;
    bool updateAfterAccum = false;

    // Update loops
    void mainLoop() {
        
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
    void syncLoop() {

        Timer frequencyTimer(goalMSPT);    // Create timer for frequency

        while (true) {

            frequencyTimer.reset(); // Reset frequency timer

            // main update function
            syncUpdate();

            // Pause for the remaining time
            if (frequencyTimer.timeLeft() < minPause) pros::delay(minPause); // Ensure that the loop pauses at least for minPause
            else pros::delay(frequencyTimer.timeLeft()); // Otherwise, wait for the remaining time
        }
    }
    void lifeTimeLoop() {
        while (true) {
            lifeTimeUpdate();
            pros::delay(goalMSPT);
        }
    }
};
