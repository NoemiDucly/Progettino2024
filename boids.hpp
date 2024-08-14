#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace boids {
extern double a;

struct Vec2 {  // vettore in due dimensioni
  double x_;
  double y_;
  Vec2() : x_(0), y_(0) {}
  Vec2(double x, double y) : x_(x), y_(y) {}

  Vec2 operator+(const Vec2& other) const;

  Vec2 operator-(const Vec2& other) const;

  Vec2& operator+=(const Vec2& other);

  Vec2& operator-=(const Vec2& other);

  Vec2 operator*(double scalar) const;

  Vec2& operator*=(double scalar); 

  Vec2 operator/(double scalar) const;

  double lenght() const;

  double distance(const Vec2& other);
};

class Boid {
 private:
  Vec2 position_;
  Vec2 velocity_;
  Vec2 acceleration_;


 public:
  Boid() {}
  Boid(double x, double y);

  Vec2 getPosition() { return position_; }
  Vec2 getVelocity() { return velocity_; }
  Vec2 getAcceleration() { return acceleration_; }


  Vec2 setPosition(double x, double y);

  std::vector<Boid> getNeighbors(const std::vector<Boid>& boids);

  // funzioni delle tre regole
  Vec2 alignment(const std::vector<Boid>& boids, double a);
  Vec2 cohesion(const std::vector<Boid>& boids, float cohesionFactor);

  void update(const std::vector<Boid>& v, double a);

  void borders();

  void run(const std::vector<Boid>& v, double a);
};
};  // namespace boids

#endif
