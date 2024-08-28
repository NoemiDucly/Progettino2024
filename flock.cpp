#include "flock.hpp"

#include "boids.hpp"

#include <cmath>

namespace boids {
long unsigned int Flock::getSize() const { return flock.size(); }

const Boid& Flock::getBoid(long unsigned int i) const { return flock[i]; }

void Flock::addBoid(const Boid& b) { flock.push_back(b); };

void Flock::flocking(double a, double c, double s, double d, double ds) {
  for (Boid& boid : flock) {
    boid.run(flock, a, c, s, d, ds);
  }
}

double Flock::calculateAverageSpeed() const {
  double totalSpeed {0.0};

  for (const auto& boid : flock) {
    totalSpeed += boid.getVelocity().lenght();
  }

  return flock.size() > 0 ? totalSpeed / static_cast<double>(flock.size())
                          : 0.0;
}

double Flock::calculateAverageDistance() const {
  double totalDistance {0.0};
  int count {0};

  for (long unsigned int i = 0; i < flock.size(); ++i) {
    for (long unsigned int j = i + 1; j < flock.size(); ++j) {
      totalDistance += flock[i].getPosition().distance(flock[j].getPosition());
      count++;
    }
  }
  return count > 0 ? totalDistance / count : 0.0;
}

double Flock::calculateSpeedStandardDeviation() const {
  double averageSpeed {calculateAverageSpeed()};
  double variance {0.0};

  for (const auto& boid : flock) {
    double speed {boid.getVelocity().lenght()};
    variance += (speed - averageSpeed) * (speed - averageSpeed);
  }

  return flock.size() > 0
             ? std::sqrt(variance / static_cast<double>(flock.size()))
             : 0.0;
}

double Flock::calculateDistanceStandardDeviation() const {
  double averageDistance {calculateAverageDistance()};
  double variance {0.0};
  int count {0};

  for (long unsigned int i = 0; i < flock.size(); ++i) {
    for (long unsigned int j = i + 1; j < flock.size(); ++j) {
      double distance = flock[i].getPosition().distance(flock[j].getPosition());
      variance += (distance - averageDistance) * (distance - averageDistance);
      count++;
    }
  }
  return count > 0 ? std::sqrt(variance / count) : 0.0;
}
}  // namespace boids
