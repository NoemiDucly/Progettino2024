#include <iostream>
#include <vector>
#include "boids.hpp"

#ifndef FLOCK_HPP
#define FLOCK_HPP

namespace boids {
class Flock {
    private:
    std::vector<Boid> flock;

    public:
    Flock() {} // constructor

    int getSize();
    Boid &getBoid(int i);
    void addBoid(const Boid& b);
    void flocking();
    double calculateAverageSpeed();
    double calculateAverageDistance();
    double calculateSpeedStandardDeviation();
    double alculateDistanceStandardDeviation();
};
}

#endif 
