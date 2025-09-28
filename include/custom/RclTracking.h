#pragma once

#include "main.h" // IWYU pragma: keep
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>
#include <numeric> // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"

// Enumerations
enum class TimeUnit { SECOND, MILLISECOND };
enum class CoordType { X, Y, INVALID };

// Timer
class Timer {
    public:
        Timer(double timeoutMs_ = 0);
        void reset();
        void hardReset(double newTimeoutMs);
        bool timeIsUp() const;
        int timeLeft() const;
        double elapsed(TimeUnit unit = TimeUnit::MILLISECOND) const;
    
    private:
        double timeoutMs;
        std::chrono::high_resolution_clock::time_point startTime;
    
        double elapsedMs() const;
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

    void add_front(AnyType* obj_ptr) {
        insert(0, obj_ptr);
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

    Node* getTail() {
        return this->tail;
    }

private:
    Node* head = new Node(nullptr);
    Node* tail = new Node(nullptr);
    int length = 0;
};

// Constants
constexpr double mmToInch = 0.039370078740157;
constexpr double MAX_OBSTACLE_DURATION = 1e12;         // ms
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

    Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs = -1);
    bool expired();
    bool isIntersecting(const SensorPose& sp) const;
    static void addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs = -1);

private:
    Line line;
    Timer lifeTimer;
};

// Circle obstacle class
class Circle_Obstacle {
public:

    static Singly_Linked_List<Circle_Obstacle> obstacleCollection;

    Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs = -1);

    // Check if the obstacle expired
    bool expired();

    // Check if sensor ray intersects this obstacle circle
    bool isIntersecting(const SensorPose& sp) const;

    
private:
    double x, y, radius;
    Timer lifeTimer;
};

// RCL sensor class
class RclSensor {
public:
    static std::vector<RclSensor*> sensorCollection;

    RclSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol = 10.0);
    void updatePose(const lemlib::Pose& botPose);
    bool isValid(double distVal) const;
    std::pair<CoordType, double> getBotCoord(const lemlib::Pose& botPose, double accum = NAN);
    double rawReading() const;
    SensorPose getPose() const;

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
                int frequencyHz_ = 25,
                bool autoSync_ = true,
                double minDelta_ = 0.5,
                double maxDelta_ = 4.0,
                double maxDeltaFromLemlib_ = 8.0,
                double maxSyncPerSec_ = 2.0,
                int minPause_ = 20);

    // Start background task
    void startTracking();
    void stopTracking();

    // Accessors
    lemlib::Pose getRclPosition() const;
    void setRclPosition(const lemlib::Pose& p);
    void updateBotPosition();

    // Accumulation control
    void startAccumulating(bool autoUpdateAfterAccum = true);
    void stopAccumulating();
    void accumulateFor (int ms, bool autoUpdateAfterAccum = true);

    // Reset Rcl
    void discardData ();

    // Single updates
    void mainUpdate();
    void syncUpdate();
    void lifeTimeUpdate();

private:
    lemlib::Chassis* chassis;
    int goalMSPT;
    int minPause;
    double maxSyncPT;
    double minDelta, maxDelta, maxDeltaFromLemlib;
    bool autoSync;
    bool accumulating;
    pros::Task mainLoopTask = pros::Task([](){});
    bool mainLoopRunning = false;
    pros::Task miscLoopTask = pros::Task([](){});
    bool miscLoopRunning = false;
    lemlib::Pose latestPrecise, poseAtLatest;
    bool updateAfterAccum = false;

    // Update loops
    void mainLoop();
    void miscLoop();
};

