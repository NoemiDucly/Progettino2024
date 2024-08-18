

#include "boids.hpp"

#include "SFML/Graphics.hpp"

namespace boids {

double perception = 50;  // field of vision
double dangerRadius= 25; // predator's perception

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

Vec2& Vec2::operator-=(const Vec2& other) {
  x_ -= other.x_;
  y_ -= other.y_;
  return *this;
}

Vec2 Vec2::operator*(double scalar) const {
  return Vec2(x_ * scalar, y_ * scalar);
}

Vec2& Vec2::operator*=(double scalar) {
  x_ *= x_ * scalar;
  y_ *= y_ * scalar;
  return *this;
}

Vec2 Vec2::operator/(double scalar) const {
  return Vec2(x_ / scalar, y_ / scalar);
}

double Vec2::lenght() const { return std::sqrt(x_ * x_ + y_ * y_); }

double Vec2::distance(const Vec2& other) {
  double dx_ = x_ - other.x_;
  double dy_ = y_ - other.y_;
  double dist = std::sqrt(dx_ * dx_ + dy_ * dy_);
  return dist;
}
class Predator {
public:
    Vec2 position_;
    Vec2 velocity_;

    Predator(double x, double y) {
        position_ = Vec2(x, y);
        velocity_ = Vec2(rand() % 3 - 1, rand() % 3 - 1);  // Velocità iniziale casuale
    }

    void move() {
        position_ += velocity_;
    if (position_.x < 0) position_.x = 800;
    if (position_.x > 800) position_.x = 0;
    if (position_.y < 0) position_.y = 600;
    if (position_.y > 600) position_.y = 0; //limitare i movimenti nella finestra 
    }
};


Boid::Boid(double x_, double y_) {
  position_ = Vec2(x_, y_);
  velocity_ = Vec2(rand() % 3, rand() % 3);
  acceleration_ = Vec2(0, 0);
}

Vec2 Boid::setPosition(double x_, double y_) {
  position_.x_ = x_;
  position_.y_ = y_;
  return position_;
}

std::vector<Boid> Boid::getNeighbors(const std::vector<Boid>& boids) {
    std::vector<Boid> neighbors;

  for (int i = 0; i < boids.size();
       ++i) {  // calculating respective distance of boids
    double d = position_.distance(boids[i].position_);
    if ((d > 0) && (d < perception)) {
      neighbors.push_back(boids[i]);
    }
  }
  return neighbors;
}

Vec2 Boid::alignment(const std::vector<Boid>& boids, double a) {
  auto neighbors = getNeighbors(boids);
  Vec2 steer;
  Vec2 sum(0,0);

  for (int i = 0; i < neighbors.size();
       ++i) {  
     sum += neighbors[i].velocity_;
    }

  if (neighbors.size() > 0) {
    // dividing by number of neighbor boids to
    // obtain average velocity_
    steer = ((sum / neighbors.size()) - velocity_) * a;  // a is the alignment factor
  }

  return steer;
};

Vec2 Boid::calculateCohesion(const std::vector<Boid>& boids, double cohesionFactor) {
    Vec2 centerOfMass(0, 0);
    int count = 0;

    // Ottieni i vicini entro il raggio di percezione
    auto neighbors = getNeighbors(boids);

    // Calcola il centro di massa dei vicini
    for (const Boid& neighbor : neighbors) {
        centerOfMass += neighbor.position_;
        count++;
    }
  //Una volta ottenuti i vicini, viene calcolato il centro di massa sommando le posizioni di tutti i vicini 
  //e dividendo per il loro numero

    if (count > 0) {
        centerOfMass = centerOfMass / count;  // Calcola il centro di massa
        // Calcola la velocità di coesione
        Vec2 cohesionVelocity = (centerOfMass - position_) * cohesionFactor;
        return cohesionVelocity;
    } else {
        return Vec2(0, 0);  // Nessun vicino, nessuna coesione
    };

Vec2 Boid::avoidance(const Boid& predator, double dangerRadius) {
    Vec2 avoidanceForce(0, 0);
    
    Vec2 difference = position_ - predator.position_; //vettore che unisce pos predatore e pos boid
    double distance = difference.lenght();

    // Se il predatore è entro il raggio di pericolo
    if (distance < dangerRadius) {
        // Aumenta la forza di repulsione se il predatore è vicino
        avoidanceForce = difference / (distance * distance);
    }

    return avoidanceForce;
}


void Boid::update(const std::vector<Boid>& v, double a,const Predator& p) {
  Vec2 ali = alignment(v, 0.6);
  Vec2 cohe = cohesion(v, 0.001);
 Vec2 avoid = avoidance (p, dangerRadius);
  acceleration_ += ali;
  acceleration_ += cohe;
acceleration_ += avoid;
  velocity_ += acceleration_;
  position_ += velocity_;
  acceleration_ *= 0;
}

// sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

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

void Boid::run(const std::vector<Boid>& v, double a) {
  update(v, a);
  borders();
}

}  // namespace boids
