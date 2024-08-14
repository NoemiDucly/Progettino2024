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
class Boid {
public:
    Vec2 position;
    Vec2 velocity;

    /*Boid(double x, double y) : position{x, y}, velocity{0, 0} {}*/
private:
    Vec2 calculateCohesion(int boidIndex, const std::vector<Boid>& boids) {
    Vec2 centerOfMass(0, 0);
    int count = 0;

    for (int j = 0; j < boids.size(); j++) {
        if (j != boidIndex) {  // Evita di considerare il boid stesso
            centerOfMass += boids[j].position;
            count++;
        }
    }

    if (count > 0) {
        centerOfMass = centerOfMass / count;  // Calcola il centro di massa
    }

    Vec2 cohesionVelocity = (centerOfMass - boids[boidIndex].position) * cohesionFactor;
    return cohesionVelocity;
}
}
}
