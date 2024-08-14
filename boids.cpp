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
    Vec2 calculateCohesion(const std::vector<Boid>& boids, float cohesionFactor) {
        Vec2 centerOfMass(0, 0);
        int count = 0;

        for (const auto& boid : boids) {
            if (&boid != this) {  // Confronta l'indirizzo dell'oggetto
                centerOfMass += boid.position;
                count++;
            }
        }

   if (count > 0) {
            centerOfMass = centerOfMass / count;  // Calcola il centro di massa
        }

        Vec2 cohesionVelocity = (centerOfMass - this->position) * cohesionFactor;
        return cohesionVelocity;
    }
}
}
