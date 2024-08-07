#include "boids.hpp"

namespace boids {
      Vec2 Vec2::operator+(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
  }

  Vec2 Vec2::operator-(const Vec2& other) const {
    return Vec2(x - other.x, y - other.y);
  }

Vec2& Vec2::operator+=(const Vec2& other) {
    x += other.x;
    y += other.y;
    return *this;
  }

    Vec2& Vec2::operator-=(const Vec2& other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  Vec2 Vec2::operator*(double scalar) const { return Vec2(x * scalar, y * scalar); }

  double Vec2::lenght() const { return std::sqrt(x * x + y * y); }

}
