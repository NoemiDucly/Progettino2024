#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace boids {
struct Vec2 {  // vettore in due dimensioni
  double x;
  double y;
  Vec2() : x(0), y(0) {}
  Vec2(double x, double y) : x(x), y(y) {}

  Vec2 operator+(const Vec2& other) const;

  Vec2 operator-(const Vec2& other) const;

  Vec2& operator+=(const Vec2& other);

  Vec2& operator-=(const Vec2& other);

  Vec2 operator*(double scalar) const;

  double lenght() const;
};

class Boid {
 private:
  Vec2 position_;
  Vec2 velocity_;

 public:
  Boid(double x, double y) {  // costruttore
    position_ = Vec2(x, y);
    // velocity_ = Vec2(random ????)
  }

  Vec2 getPosition() { return position_; }
  Vec2 getVelocity() { return velocity_; }

  // aggiornare posizione e velocità
  // funzioni delle tre regole
};
};  // namespace boids

#endif BOIDS_HPP
