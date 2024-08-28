#include "boids.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
// #include "SFML/Graphics.hpp"
// #include <random>

namespace boids {

void Vec2::setValues(double x, double y) {
  x_ = x;
  y_ = y;
}

Vec2 Vec2::operator+(const Vec2& other) const {
  return Vec2(x_ + other.x_, y_ + other.y_);
}

Vec2 Vec2::operator-(const Vec2& other) const {
  return Vec2(x_ - other.x_, y_ - other.y_);
}

Vec2& Vec2::operator+=(const Vec2& other) {
  x_ += other.x_;
  y_ += other.y_;
  return *this;
}

// Vec2& Vec2::operator-=(const Vec2& other) {
//   x_ -= other.x_;
//   y_ -= other.y_;
//   return *this;
// }

Vec2 Vec2::operator*(double scalar) const {
  return Vec2(x_ * scalar, y_ * scalar);
}

Vec2& Vec2::operator*=(double scalar) {
  x_ *= x_ * scalar;
  y_ *= y_ * scalar;
  return *this;
}

Vec2 Vec2::operator/(double scalar) const {
  assert(scalar != 0);
  if (scalar == 0) {
    throw std::runtime_error{"math error: cannot divide by 0"};
  }

  return Vec2(x_ / scalar, y_ / scalar);
}

double Vec2::lenght() const {
  double len{std::sqrt(x_ * x_ + y_ * y_)};
  assert(len >= 0);
  return len;
}

// bool Vec2::operator==(const Vec2& other) const {
//   return x_ == other.x_ && y_ == other.y_;
// }

void Vec2::limit(double max) {
  double module = lenght();
  if (module >= max) {
    setValues(x_ * max / module, y_ * max / module);
  }
}

double Vec2::distance(const Vec2& other) const {
  double dx_ = x_ - other.x_;
  double dy_ = y_ - other.y_;
  double dist = std::sqrt(dx_ * dx_ + dy_ * dy_);
  assert(dist >= 0);
  return dist;
}

Vec2 Vec2::normalize() const {
  double len = Vec2::lenght();
  if (len > 0) {
    Vec2 norm(x_ / len, y_ / len);
    assert(std::abs(norm.lenght() - 1.0) < 1e-9);
    return norm;
  } else {
    return Vec2(0, 0);
  }
}

Boid::Boid(double x, double y, double vx, double vy)
    : position_{Vec2(x, y)},
      velocity_{Vec2(vx, vy)},
      acceleration_{Vec2(0, 0)},
      maxSpeed{5} {
  assert(velocity_.lenght() <= maxSpeed);
}

Vec2 Boid::setPosition(double x, double y) {
  position_.x_ = x;
  position_.y_ = y;
  return position_;
}

std::vector<Boid> Boid::getNeighbors(const std::vector<Boid>& boids,
                                     double perception) const {
  std::vector<Boid> neighbors;

  for (const Boid& boid : boids) {
    double d = position_.distance(boid.position_);
    if ((d > 0) && (d < perception)) {
      neighbors.push_back(boid);
    }
  }
  return neighbors;
}

Vec2 Boid::alignment(const std::vector<Boid>& boids, double alignmentFactor,
                     double perception) const {
  assert(alignmentFactor >= 0);

  auto neighbors = getNeighbors(boids, perception);
  Vec2 steer(0, 0);
  Vec2 sum(0, 0);

  for (const Boid& neighbor : neighbors) {
    sum += neighbor.velocity_;
  }

  if (neighbors.size() > 0) {
    steer = ((sum / static_cast<double>(neighbors.size())) - velocity_) *
            alignmentFactor;
  }

  return steer;
};

Vec2 Boid::cohesion(const std::vector<Boid>& boids, double cohesionFactor,
                    double perception) const {
  assert(cohesionFactor >= 0);

  auto neighbors = getNeighbors(boids, perception);
  Vec2 steer(0, 0);
  Vec2 centerOfMass(0, 0);

  for (const Boid& neighbor : neighbors) {
    centerOfMass += neighbor.position_;
  }

  if (neighbors.size() > 0) {
    centerOfMass = centerOfMass / static_cast<double>(neighbors.size());
    steer = (centerOfMass - position_) * cohesionFactor;
  }
  
  return steer;
};

Vec2 Boid::separation(const std::vector<Boid>& boids, double separationDistance,
                      double separationFactor, double perception) const {
  assert(separationFactor >= 0);
  assert(separationDistance <= perception);

  Vec2 steer(0, 0);
  int count{0};

  auto neighbors = getNeighbors(boids, perception);

  for (const Boid& neighbor : neighbors) {
    double distanceS = (position_ - neighbor.position_).lenght();
    if (distanceS > 0 && distanceS < separationDistance) {
      Vec2 diff = position_ - neighbor.position_;
      diff = diff.normalize() / distanceS;
      steer = (steer + diff) * separationFactor;
      ++count;
    }
  }

  if (count > 0) {
    steer = steer / count;
  }
  return steer;
};

void Boid::update(const std::vector<Boid>& v, double a, double c, double s,
                  double d, double ds) {
  Vec2 ali = alignment(v, a, d);
  Vec2 cohe = cohesion(v, c, d);
  Vec2 sep = separation(v, ds, s, d);
  acceleration_ += ali;
  acceleration_ += cohe;
  acceleration_ += sep;
  velocity_ += acceleration_;
  velocity_.limit(maxSpeed);
  position_ += velocity_;
  acceleration_ *= 0;
  assert(acceleration_.x_ == 0);
  assert(acceleration_.y_ == 0);
};

void Boid::borders() {
  if (position_.x_ < 0) {
    position_.x_ += 810;
  }
  if (position_.x_ > 800) {
    position_.x_ -= 810;
  }
  if (position_.y_ < 0) {
    position_.y_ += 610;
  }
  if (position_.y_ > 600) {
    position_.y_ -= 610;
  }
}

void Boid::run(const std::vector<Boid>& v, double a, double c, double s,
               double d, double ds) {
  update(v, a, c, s, d, ds);
  borders();
  assert(velocity_.lenght() <= (maxSpeed + 0.001));
}

}  // namespace boids
