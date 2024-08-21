#include "boids.hpp"
#include "flock.hpp"

namespace boids {
double a = 0;
int Flock::getSize() {
    return flock.size();
}

Boid &Flock::getBoid(int i) {
    return flock[i];
}

void Flock::addBoid(const Boid& b) {
    flock.push_back(b);
};


void Flock::flocking() {
  for (int i = 0; i < flock.size(); ++i) { flock[i].run(flock, a); }
}
double Flock::calculateAverageSpeed() const {
    double totalSpeed = 0.0;

    for (const auto& boid : boids) {
        totalSpeed += boid.getVelocity().length();
    }

    return boids_.size() > 0 ? totalSpeed / boids.size() : 0.0;
}
double Flock::calculateAverageDistance() const {
    double totalDistance = 0.0;
    int count = 0;

    for (int i = 0; i < boids.size(); ++i) {
        for (int j = i + 1; j < boids.size(); ++j) {
            totalDistance += boids[i].getPosition().distance(boids[j].getPosition());
            count++;
        }
    }

    return count > 0 ? totalDistance / count : 0.0;
}
double Flock::calculateSpeedStandardDeviation() const {
    double averageSpeed = calculateAverageSpeed();
    double variance = 0.0;

    for (const auto& boid : boids) {
        double speed = boid.getVelocity().length();
        variance += (speed - averageSpeed) * (speed - averageSpeed);
    }

    return boids.size() > 0 ? std::sqrt(variance / boids.size()) : 0.0;
}
double Flock::calculateDistanceStandardDeviation() const {
    double averageDistance = calculateAverageDistance();
    double variance = 0.0;
    int count = 0;

    for (int i = 0; i < boids.size(); ++i) {
        for (int j = i + 1; j < boids.size(); ++j) {
            double distance = boids[i].getPosition().distance(boids[j].getPosition());
            variance += (distance - averageDistance) * (distance - averageDistance);
            count++;
        }
    }

    return count > 0 ? std::sqrt(variance / count) : 0.0;
}
}
