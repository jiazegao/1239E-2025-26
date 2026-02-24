#pragma once

#include "main.h" // IWYU pragma: keep
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>
#include <numeric> // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "Tracking_Util.hpp"
#include <fstream>

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
    float x = 0;
    float y = 0;
    float heading = 0;
    float slope = 0;
    float yIntercept = 0;
};

struct Line {
    float pt1[2] = {0, 0};
    float pt2[2] = {0, 0};
    float slope = 0;
    float yIntercept = 0;
};

// Line obstacle class
class Line_Obstacle {
public:

    static Singly_Linked_List<Line_Obstacle> obstacleCollection;

    Line_Obstacle(float x1, float y1, float x2, float y2, float lifeTimeMs = -1);
    bool expired();
    bool isIntersecting(const SensorPose& sp) const;
    static void addPolygonObstacle(const std::vector<std::pair<float, float>>& points, float lifeTimeMs = -1);

private:
    Line line;
    Timer lifeTimer;
};

// Circle obstacle class
class Circle_Obstacle {
public:

    static Singly_Linked_List<Circle_Obstacle> obstacleCollection;

    Circle_Obstacle(float x_, float y_, float r_, float lifeTimeMs = -1);

    // Check if the obstacle expired
    bool expired();

    // Check if sensor ray intersects this obstacle circle
    bool isIntersecting(const SensorPose& sp) const;

    float x, y, radius;
    Timer lifeTimer;
};

// RCL sensor class
class RclSensor {
public:
    static std::vector<RclSensor*> sensorCollection;

    RclSensor(pros::Distance* distSensor, float horizOffset, float vertOffset, float mainAng, float angleTol = 10.0);
    void updatePose(const lemlib::Pose& botPose);
    bool isValid(float distVal) const;
    std::pair<CoordType, float> getBotCoord(const lemlib::Pose& botPose, float accum = NAN);
    int rawReading() const;
    SensorPose getPose() const;
    void logPos(std::ofstream* targetFile);

private:
    pros::Distance* sensor;
    float offsetDist;
    float offsetAngle;
    float mainAngle;
    SensorPose sp;
    float angleTolerance;  // degrees
};

// Main RCL Tracking
inline std::vector<RclSensor*> RclSensor::sensorCollection = std::vector<RclSensor*>();
class RclTracking {
public:
    RclTracking(lemlib::Chassis* chassis_,
                int frequencyHz_ = 25,
                bool autoSync_ = true,
                float minDelta_ = 0.5,
                float maxDelta_ = 4.0,
                float maxDeltaFromLemlib_ = 10.0,
                float maxSyncPerSec_ = 3.0,
                int minPause_ = 20);

    // Start background task
    void startTracking();
    void stopTracking();

    // Accessors
    lemlib::Pose getRclPose() const;
    void setRclPose(const lemlib::Pose& p);
    void updateBotPose();
    void updateBotPose(RclSensor* sens);

    void setMaxSyncPerSec(float _maxSyncPerSec); 

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
    float maxSyncPT;
    float minDelta, maxDelta, maxDeltaFromLemlib;
    bool autoSync;
    bool accumulating;
    pros::Task* mainLoopTask = nullptr;
    pros::Task* miscLoopTask = nullptr;
    lemlib::Pose latestPrecise, poseAtLatest;
    bool updateAfterAccum = false;

    // Update loops
    void mainLoop();
    void miscLoop();
};