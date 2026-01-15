#pragma once

// RclTracking.cpp
// Author: Jason - Team 1239E
// Description: This file implements the RCL Tracking system for a robot, including
//              sensor management, obstacle detection, position tracking and syncing
//              using VEX V5 distance sensors.
//              Most functionalities are achieved through basic sensor fusion and
//              intersection math.

#include "main.h" // IWYU pragma: keep
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>
#include <numeric> // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "Tracking_Util.hpp"

// Enumerations
enum class CoordType { X, Y, INVALID };

// Singly linked list for quick removal / insertion
template <typename AnyType>
class Singly_Linked_List {

public:
    Singly_Linked_List() {
        head = new Node(nullptr);
        tail = new Node(nullptr);
        length = 0;
        head->next = tail;
    }

    ~Singly_Linked_List() {
        Node* curr = head;
        while (curr != nullptr) {
            Node* nextNode = curr->next;
            if (curr->ptr != nullptr) delete curr->ptr;
            delete curr; // This deletes the Node container
            curr = nextNode;
        }
    }

    Singly_Linked_List(const Singly_Linked_List&) = delete;
    Singly_Linked_List& operator=(const Singly_Linked_List&) = delete;

    class Node {
        public:
            explicit Node(AnyType* obj_ptr) {
                this->ptr = obj_ptr;
                this->next = nullptr;
            }
            AnyType* ptr;
            Node* next;
    };

    class Iterator {
    private:
        Node* next;
        Node* previous;
        Singly_Linked_List* list;
        int count;

    public:
        // Constructor
        explicit Iterator(Singly_Linked_List* list, Node* node) : next(node), previous(list->getHead()), list(list), count(0) {}

        // Dereference operator
        AnyType*& operator*() const {
            return next->ptr;
        }

        // Pre-increment operator
        Iterator& operator++() {
            if (next != list->getTail() && count < list->size()) {
                previous = next;
                next = next->next;
                count++;
            }
            return *this;
        }

        // Post-increment operator (optional, but good practice)
        Iterator operator++(int) {
            Iterator temp = *this;
            if (next != list->getTail() && count < list->size()) {
                previous = next;
                next = next->next;
                count++;
            }
            return temp;
        }

        // Equality operator
        bool operator==(const Iterator& other) const {
            return next == other.next;
        }

        // Inequality operator
        bool operator!=(const Iterator& other) const {
            return next != other.next;
        }

        // Insert at current location
        void insert(AnyType* obj_ptr) {
            if (obj_ptr != nullptr) {
                // previous * next  =>  previous * new - next
                Node* oldNode = next;
                previous->next = new Node(obj_ptr);
                previous->next->next = oldNode;
                ++list->length;

                next = previous->next; // Update iterator
            }
        }

        // Remove current object
        bool remove(bool clean = false) {
            if (next == list->getTail()) return false;

            if (0 <= count && count < list->size()) {
                // previous * next - nextNext  =>  previous * nextNext
                Node* oldNode = next;
                next = next->next;
                previous->next = next;
                if (clean && oldNode->ptr != nullptr) delete oldNode->ptr;
                delete oldNode;
                --list->length;
                return true;
            }
            else {
                return false;
            }
        }

        // Get current index
        [[nodiscard]] int getIndex() const {
            return count;
        }
    };

    Iterator begin() {
        return Iterator(this, head->next);
    }

    Iterator end() {
        return Iterator(this, tail);
    }

    void push_back(AnyType* obj_ptr) {
        insert(length, obj_ptr);
    }

    void add_front(AnyType* obj_ptr) {
        insert(0, obj_ptr);
    }

    void insert(int index, AnyType* obj_ptr) {
        if (obj_ptr != nullptr) {
            int temp_index = index;

            if (this->head != nullptr && temp_index >= 0 && temp_index <= length) {
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

    void pop(int index, bool clean = false){
        if (index >= 0 && index < length) {
            Node* currNode = head;
            while (index > 0){
                currNode = currNode->next;
                index--;
            }
            Node* oldNode = currNode->next;
            currNode->next = oldNode->next;
            if (clean && oldNode->ptr != nullptr) delete oldNode->ptr;
            delete oldNode;
            length--;
        }
    }

    AnyType* get(int index){
        if (index < 0 || index >= length) return nullptr;
        Node* currNode = head->next; // Start at the first REAL node
        for (int i = 0; i < index; i++) {
            currNode = currNode->next;
        }
        return currNode->ptr;
    }

    AnyType* operator[](int index) {
        return get(index);
    }

    void remove(AnyType* obj_ptr, bool clean = false) {
        if (obj_ptr != nullptr) {
            Node* currNode = this->head;
            while (currNode->next != nullptr && currNode->next->ptr != obj_ptr){
                currNode = currNode->next;
            }
            if (currNode->next != nullptr && currNode->next->ptr == obj_ptr) {
                Node* oldNode = currNode->next;
                currNode->next = oldNode->next;
                if (clean && oldNode->ptr != nullptr) delete oldNode->ptr;
                delete oldNode;
                length--;
            }
        }
    }

    [[nodiscard]] int size() const {
        return length;
    }

    Node* getHead() {
        return this->head;
    }

    Node* getTail() {
        return this->tail;
    }

private:
    Node* head;
    Node* tail;
    int length;
};

// Pose of a distance sensor (ray) for intersection math
struct SensorPose {
    double x = 0;
    double y = 0;
    double heading = 0;
};

struct Line {
    double pt1[2] = {0, 0};
    double pt2[2] = {0, 0};
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
        // Add to collection if space available
        Line_Obstacle::obstacleCollection.add_front(this);
    }

    bool expired() {return lifeTimer.timeIsUp();};

    bool isIntersecting(const SensorPose& sp) const {
        // Ray direction vector
        double angRad = degToRad(botToTrig(sp.heading));
        double vAx = std::cos(angRad);
        double vAy = std::sin(angRad);

        // Obstacle segment vector
        double vBx = line.pt2[0] - line.pt1[0];
        double vBy = line.pt2[1] - line.pt1[1];

        // Determinant (Cross product of direction vectors)
        double det = (vAx * vBy) - (vAy * vBx);

        // If det is 0, the ray and the obstacle are parallel
        if (std::abs(det) < 1e-6) return false;

        // Solve for t (distance along sensor ray) and u (position along obstacle segment)
        // Formula: P_sensor + t*vA = P_obstacle_start + u*vB
        double dx = line.pt1[0] - sp.x;
        double dy = line.pt1[1] - sp.y;

        double t = (dx * vBy - dy * vBx) / det;
        double u = (dx * vAy - dy * vAx) / det;

        // Valid intersection if:
        // 1. t > 0 (Intersection is in front of the sensor)
        // 2. 0 <= u <= 1 (Intersection lies on the actual segment, not just the infinite line)
        return (t > 0 && u >= 0 && u <= 1);
    };

    static void addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs = -1) {
        // If the points are not enough to form a polygon, do nothing
        if (points.size() < 3) return;
        // Create a line for each pair of points
        for (size_t i = 0; i < points.size(); ++i) {
            size_t next = (i + 1) % points.size();
            new Line_Obstacle(points[i].first, points[i].second, points[next].first, points[next].second, lifeTimeMs);
        }
    };

private:
    Line line;
    Timer lifeTimer;
};
inline Singly_Linked_List<Line_Obstacle> Line_Obstacle::obstacleCollection = Singly_Linked_List<Line_Obstacle>();

// Circle obstacle class
class Circle_Obstacle {
public:

    static Singly_Linked_List<Circle_Obstacle> obstacleCollection;

    Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs = -1)
    : x(x_), y(y_), radius(r_), lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
        Circle_Obstacle::obstacleCollection.add_front(this);
    }
    // Check if the obstacle expired
    bool expired() {return lifeTimer.timeIsUp();};

    // Check if sensor ray intersects this obstacle circle
    bool isIntersecting(const SensorPose& sp) const {
        double angRad = degToRad(botToTrig(sp.heading));
        double vx = std::cos(angRad);
        double vy = std::sin(angRad);

        // Vector from sensor to circle center
        double dx = x - sp.x;
        double dy = y - sp.y;

        // Calculate 't' (the distance along the ray to the point closest to the circle center)
        // t = dot product of (vector to center) and (ray direction)
        double t = dx * vx + dy * vy;

        // If t is negative, the circle is behind the sensor
        if (t < 0) return false;

        // Find the point on the ray at distance t
        double closestX = sp.x + t * vx;
        double closestY = sp.y + t * vy;

        // Check distance from that point to circle center
        double distSq = std::pow(closestX - x, 2) + std::pow(closestY - y, 2);
        
        return distSq <= (radius * radius);
    };

    double x, y, radius;
    Timer lifeTimer;
};
inline Singly_Linked_List<Circle_Obstacle> Circle_Obstacle::obstacleCollection = Singly_Linked_List<Circle_Obstacle>();

// RCL sensor class
class RclSensor {
public:
    static std::vector<RclSensor*> sensorCollection;

    RclSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol = 10.0)
    : sensor(distSensor), mainAngle(mainAng), angleTolerance(std::abs(angleTol)) {
        offsetDist = std::hypot(horizOffset, vertOffset);
        offsetAngle = std::fmod(((std::atan2(vertOffset, horizOffset)*180.0/M_PI) + 360), 360);
        RclSensor::sensorCollection.push_back(this);
    }

    void updatePose(const lemlib::Pose& botPose) {
        // sensor's angle position relative to the bot's center
        double theta = degToRad(offsetAngle - botPose.theta);
        sp.x = botPose.x + std::cos(theta) * offsetDist;
        sp.y = botPose.y + std::sin(theta) * offsetDist;
        // sensor ray's heading
        sp.heading = std::fmod(botPose.theta + mainAngle, 360.0);
        if (sp.heading <= 0) sp.heading += 360.0;  // avoid 0 or negative
    };

    bool isValid(double distVal) const {
        if (distVal > 2000) return false;  // Invalid Distance
        if (distVal > 200 && this->sensor->get_confidence() < 60) return false; // Invalid confidence
        if (std::abs( std::fmod(this->sp.heading, 90.0) ) > angleTolerance &&
            std::abs( std::fmod(this->sp.heading, 90.0) ) < (90 - angleTolerance)) return false;   // Limit heading to intervals around 90 degrees to increase accuracy
        // Detect if the ray is intersecting with any circular obstacles
        for (auto* item : Circle_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }
        // Detect if the ray is intersectign with any line obstacles
        for (auto* item : Line_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }

        return true;
    };

    std::pair<CoordType, double> getBotCoord(const lemlib::Pose& botPose, double accum = NAN) {
        // Update sensor pose
        this->updatePose(botPose);

        // accumulative?
        double val = std::isnan(accum) ? sensor->get() : accum;

        // verify sensor data
        if (!isValid(val)) return {CoordType::INVALID, 0.0};
        val *= mmToInch;

        //
        double angRad = degToRad(botToTrig(this->sp.heading));
        double cosA = std::cos(angRad);
        double sinA = std::sin(angRad);

        double minDist = 1e9; // Start with a huge number
        int wall = -1;

        // Check X-walls (Vertical lines)
        if (std::abs(cosA) > 1e-6) {
            double dEast = (FIELD_HALF_LENGTH - sp.x) / cosA;
            if (dEast > 0 && dEast < minDist) { minDist = dEast; wall = 2; }
            
            double dWest = (FIELD_NEG_HALF_LENGTH - sp.x) / cosA;
            if (dWest > 0 && dWest < minDist) { minDist = dWest; wall = 4; }
        }

        // Check Y-walls (Horizontal lines)
        if (std::abs(sinA) > 1e-6) {
            double dNorth = (FIELD_HALF_LENGTH - sp.y) / sinA;
            if (dNorth > 0 && dNorth < minDist) { minDist = dNorth; wall = 1; }
            
            double dSouth = (FIELD_NEG_HALF_LENGTH - sp.y) / sinA;
            if (dSouth > 0 && dSouth < minDist) { minDist = dSouth; wall = 3; }
        }

        // Compute coordinate
        double res;
        CoordType type;
        if (wall == 1) { type = CoordType::Y; res = FIELD_HALF_LENGTH - sinA * val; }
        else if (wall == 2) { type = CoordType::X; res = FIELD_HALF_LENGTH - cosA * val; }
        else if (wall == 3) { type = CoordType::Y; res = FIELD_NEG_HALF_LENGTH - sinA * val; }
        else if (wall == 4) { type = CoordType::X; res = FIELD_NEG_HALF_LENGTH - cosA * val; }
        else return {CoordType::INVALID, 0.0};  // Error case

        // Adjust by offset
        double offRad = degToRad(offsetAngle - botPose.theta);
        if (type == CoordType::X) res -= std::cos(offRad) * offsetDist;
        else if (type == CoordType::Y) res -= std::sin(offRad) * offsetDist;

        return {type, res};
    };

    int rawReading() const {return sensor->get();};
    SensorPose getPose() const {return sp;};

private:
    pros::Distance* sensor;
    double offsetDist;
    double offsetAngle;
    double mainAngle;
    SensorPose sp;
    double angleTolerance;  // degrees
};
inline std::vector<RclSensor*> RclSensor::sensorCollection = std::vector<RclSensor*>();

// Main RCL Tracking
class RclTracking {
public:
    RclTracking(lemlib::Chassis* chassis_,
                int frequencyHz_ = 25,
                bool autoSync_ = true,
                double minDelta_ = 0.5,
                double maxDelta_ = 4.0,
                double maxDeltaFromChassis_ = 15.0,
                double maxSyncPerSec_ = 4.0,
                int minPause_ = 15)
        : chassis(chassis_),
        goalMSPT(std::round(1000.0 / frequencyHz_)),
        minPause(minPause_),
        maxSyncPT(maxSyncPerSec_ / frequencyHz_),
        minDelta(minDelta_),
        maxDelta(maxDelta_),
        maxDeltaFromChassis(maxDeltaFromChassis_),
        autoSync(autoSync_),
        latestPrecise{0, 0, 0},
        poseAtLatest{0, 0, 0} {};

    
    // Initialize and create tasks
    void initialize() {
        // Start position update loop
        if (mainLoopTask == nullptr)  {
            mainLoopTask = new pros::Task([this](){ this->mainLoop(); });
        }
        // Start miscellaneous update loop
        if (miscLoopTask == nullptr) {
            miscLoopTask = new pros::Task([this](){ this->miscLoop(); });
        }
        startTracking();
    }

    // Start loop
    void startTracking() {
        mainLoopRunning = true;
        miscLoopRunning = true;
    };

    // Stop loop
    void stopTracking() {
        mainLoopRunning = false;
        miscLoopRunning = false;
    };

    // Accessors
    lemlib::Pose getRclPose() const {
        auto chassisPose = chassis->getPose();
        return { latestPrecise.x + (chassisPose.x - poseAtLatest.x),
                    latestPrecise.y + (chassisPose.y - poseAtLatest.y),
                    chassisPose.theta };
    };

    void setRclPose(const lemlib::Pose& p) {
        latestPrecise = p;
        poseAtLatest = chassis->getPose();
    };

    void updateBotPose() {
        auto p = getRclPose();
        chassis->setPose(p);
        setRclPose(p);
    };

    void updateBotPose(RclSensor* sens) {
        if (sens != nullptr && chassis != nullptr) {
            // Get data from the target sensor
            auto data = sens->getBotCoord(chassis->getPose());
            // Retrieve Lemlib pose as the base
            auto pose = chassis->getPose();

            // Update pose based on sensor reading
            if (data.first == CoordType::X) pose.x = data.second;
            else if (data.first == CoordType::Y) pose.y = data.second;
            else return;

            // Sync to Lemlib
            chassis->setPose({pose.x, pose.y, chassis->getPose().theta});
            setRclPose({pose.x, pose.y, chassis->getPose().theta});
        }
    };

    // Accumulation control
    void startAccumulating(bool autoUpdateAfterAccum = true) {accumulating = true; updateAfterAccum = autoUpdateAfterAccum;};
    void stopAccumulating() {accumulating = false;};
    void accumulateFor (int ms, bool autoUpdateAfterAccum = true) {
        startAccumulating( autoUpdateAfterAccum );
        Timer t(ms);
        while (!t.timeIsUp()) { pros::delay(minPause); }
        stopAccumulating();
    };

    // Reset Rcl
    void discardData() {latestPrecise = chassis->getPose(); poseAtLatest = chassis->getPose();};

    // Set sync rate
    void setMaxSyncPerSec(double _maxSyncPerSec) {
        maxSyncPT = _maxSyncPerSec / (1000.0 / goalMSPT);
    }

    // Single updates
    void mainUpdate() {
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
            if (diff_from_lemlib > maxDeltaFromChassis) {
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
    };
    void syncUpdate() {
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
            x_update = x_diff / real_diff * maxSyncPT;
            y_update = y_diff / real_diff * maxSyncPT;

            // Update (Sync)
            chassis->setPose(chassis->getPose().x+x_update, chassis->getPose().y+y_update, chassis->getPose().theta);
            
            poseAtLatest.x += x_update;
            poseAtLatest.y += y_update;
            poseAtLatest.theta = chassis->getPose().theta;
        }
    };
    void lifeTimeUpdate() {
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
    };

private:
    lemlib::Chassis* chassis;
    int goalMSPT;
    int minPause;
    double maxSyncPT;
    double minDelta, maxDelta, maxDeltaFromChassis;
    bool autoSync;
    bool accumulating;
    pros::Task* mainLoopTask = nullptr;
    pros::Task* miscLoopTask = nullptr;
    lemlib::Pose latestPrecise, poseAtLatest;
    bool updateAfterAccum = false;
    bool mainLoopRunning = false;
    bool miscLoopRunning = false;

    // Update loops
    void mainLoop() {
        Timer frequencyTimer(goalMSPT);    // Create timer for frequency

        while (true) {

            while (mainLoopRunning) {
                // Reset frequency timer
                frequencyTimer.reset();

                // Call the update function
                mainUpdate();

                // Delay to save resources
                if ( frequencyTimer.timeLeft() < minPause ) pros::delay(minPause); // Ensure that the loop pauses at least for minPause
                else pros::delay(frequencyTimer.timeLeft()); // Otherwise, wait for the remaining time
            }

            pros::delay(minPause);
        }
    };
    void miscLoop() {
        Timer frequencyTimer(goalMSPT);    // Create timer for frequency

        while (true) {

            while (miscLoopRunning) {

                frequencyTimer.reset(); // Reset frequency timer

                // main update functions
                if (autoSync) { syncUpdate(); }
                lifeTimeUpdate();

                // Pause for the remaining time
                if (frequencyTimer.timeLeft() < minPause) pros::delay(minPause); // Ensure that the loop pauses at least for minPause
                else pros::delay(frequencyTimer.timeLeft()); // Otherwise, wait for the remaining time
            }

            pros::delay(minPause);
        }
    };
};