#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <vector>

namespace boids {

struct Vec2 {
  double x_;
  double y_;
  Vec2() : x_(0), y_(0) {}
  Vec2(double x, double y) : x_(x), y_(y) {}

  void setValues(double x, double y);

  Vec2 operator+(const Vec2& other) const;

  Vec2 operator-(const Vec2& other) const;

  Vec2& operator+=(const Vec2& other);

//  Vec2& operator-=(const Vec2& other);

  Vec2 operator*(double scalar) const;

  Vec2& operator*=(double scalar);

  Vec2 operator/(double scalar) const;

//  bool operator==(const Vec2& other) const;

  double lenght() const;

  void limit(double max);

  double distance(const Vec2& other) const;

  Vec2 normalize() const;
};

class Boid {
 private:
  Vec2 position_;
  Vec2 velocity_;
  Vec2 acceleration_;
  double maxSpeed;

 public:
  Boid() {}
  Boid(double x, double y, double vx, double vy);

  Vec2 getPosition() const { return position_; }
  Vec2 getVelocity() const { return velocity_; }
  Vec2 getAcceleration() const { return acceleration_; }

  Vec2 setPosition(double x, double y);

  std::vector<Boid> getNeighbors(const std::vector<Boid>& boids, double perception) const;

  // funzioni delle tre regole
  Vec2 alignment(const std::vector<Boid>& boids, double alignmentFactor, double perception) const;
  Vec2 cohesion(const std::vector<Boid>& boids, double cohesionFactor, double perception) const;
  Vec2 separation(const std::vector<Boid>& boids, double separationDistance,
                  double separationFactor, double perception) const;

  void update(const std::vector<Boid>& v, double a, double c, double s, double d, double ds);

  void borders();

  void run(const std::vector<Boid>& v, double a, double c, double s, double d, double ds);
};
};  // namespace boids

#endif
