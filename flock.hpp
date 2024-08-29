#ifndef FLOCK_HPP
#define FLOCK_HPP

#include "boids.hpp"


namespace boids {
class Flock {
 private:
  std::vector<Boid> flock;

 public:
  Flock() {}

  long unsigned int getSize() const;
  const Boid& getBoid(long unsigned int i) const;
  void addBoid(const Boid& b);
  void flocking(double a, double c, double s, double d, double ds);
  double calculateAverageSpeed() const;
  double calculateAverageDistance() const;
  double calculateSpeedStandardDeviation() const;
  double calculateDistanceStandardDeviation() const;
};
}  // namespace boids

#endif
