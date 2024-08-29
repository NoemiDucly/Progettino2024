

#include "boids.hpp"

#include "SFML/Graphics.hpp"

namespace boids {

double perception{50};  // field of vision
double dangerRadius{25}; // predator's perception

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
  double dx_{x_ - other.x_};
  double dy_{y_ - other.y_};
  double dist{std::sqrt(dx_ * dx_ + dy_ * dy_)};
  return dist;
}



Boid::Boid(double x_, double y_) {
    position_{Vec2(x_, y_)};
    velocity_{Vec2(rand() % 3, rand() % 3)};
    acceleration_{Vec2(0, 0)};
    isPredator_{false};  // Imposta isPredator_ a false per i boid normali
}

// Costruttore che accetta posizione, velocità e un flag per il predatore
Boid::Boid(const Vec2& pos, const Vec2& vel, bool isPredator)
    : position_(pos), velocity_(vel), isPredator_(isPredator) {
    acceleration_{Vec2(0, 0)};
}

Vec2 Boid::setPosition(double x_, double y_) {
  position_.x_{x_};
  position_.y_{y_};
  return position_;
}

std::vector<Boid> Boid::getNeighbors(const std::vector<Boid>& boids) {
    std::vector<Boid> neighbors;

  for (int i{0}; i < boids.size();
       ++i) {  // calculating respective distance of boids
    double d{position_.distance(boids[i].position_)};
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

  for (int i{0}; i < neighbors.size();
       ++i) {  
     sum += neighbors[i].velocity_;
    }

  if (neighbors.size() > 0) {
    // dividing by number of neighbor boids to
    // obtain average velocity_
    steer{((sum / neighbors.size()) - velocity_) * a};  // a is the alignment factor
  }

  return steer;
};

Vec2 Boid::calculateCohesion(const std::vector<Boid>& boids, double cohesionFactor) {
    Vec2 centerOfMass(0, 0);
    int count{0};

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
        centerOfMass{centerOfMass / count};  // Calcola il centro di massa
        // Calcola la velocità di coesione
        Vec2 cohesionVelocity{(centerOfMass - position_) * cohesionFactor};
        return cohesionVelocity;
    } else {
        return Vec2(0, 0);  // Nessun vicino, nessuna coesione
    };

Vec2 Boid::avoidance(const Predator& predator, double dangerRadius) {
    Vec2 avoidanceForce(0, 0);
    
    double distance{position_.distance(predator.position_)};  // Usa la funzione distance

    // Se il predatore è entro il raggio di pericolo
    if (distance < dangerRadius) {
        Vec2 difference{position_ - predator.position_};  // Vettore che unisce pos predatore e pos boid
        // Aumenta la forza di repulsione se il predatore è vicino
        avoidanceForce{difference / (distance * distance)};
    }

    return avoidanceForce;
}



void Boid::update(const std::vector<Boid>& v, double a,const Predator& p) {
  Vec2 ali{alignment(v, 0.6)};
  Vec2 cohe{cohesion(v, 0.001)};
 Vec2 avoid{avoidance (p, dangerRadius)};
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
int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "simulation", sf::Style::Titlebar);
    window.setPosition(sf::Vector2i(10, 50));
    window.setFramerateLimit(60);

    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

    sf::RectangleShape sliderBar(sf::Vector2f(600, 10));
    sliderBar.setPosition(100, 100);
    sliderBar.setFillColor(sf::Color::White);

    sf::CircleShape sliderKnob(10);
    sliderKnob.setFillColor(sf::Color::Red);
    sliderKnob.setPosition(100, 95);

    bool isDragging{false};
    double a{0};

    boids::Flock flock;
    std::vector<sf::CircleShape> shapes;

    for (int i{0}; i < 100; ++i) {
        boids::Boid b(rand() % 800, rand() % 600);
        sf::CircleShape shape(8, 3);
        shape.setFillColor(sf::Color::White);  // Colore per i boid normali
        flock.addBoid(b);
        shapes.push_back(shape);
    }

    // Creazione del predatore
    boids::Boid predatorBoid(400, 300);
    predatorBoid.isPredator_{true};
    sf::CircleShape predatorShape(10, 3);
    predatorShape.setFillColor(sf::Color::Red);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            if (event.type == sf::Event::MouseButtonPressed) {
                if (sliderKnob.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
                    isDragging{true};
                }
            }

            if (event.type == sf::Event::MouseButtonReleased) {
                isDragging{false};
            }

            if (event.type == sf::Event::MouseMoved && isDragging) {
                float newPosX{event.mouseMove.x - sliderKnob.getRadius()};
                newPosX{std::max(100.0f, std::min(newPosX, 690.0f))};
                sliderKnob.setPosition(newPosX, sliderKnob.getPosition().y);

                // Mappare la posizione del knob a un valore tra 0 e 2
                a{((newPosX - 100.0f) / 590.0f) * 2.0f};
            }
        }

        window.clear(sf::Color::Black);

        for (int i{0}; i < shapes.size(); ++i) {
            flock.getBoid(i).update(flock.getBoids(), a, predatorBoid);
            shapes[i].setPosition(flock.getBoid(i).getPosition().x_, flock.getBoid(i).getPosition().y_);
            window.draw(shapes[i]);
        }

        // Aggiornamento e disegno del predatore
        predatorShape.setPosition(predatorBoid.getPosition().x_, predatorBoid.getPosition().y_);
        window.draw(predatorShape);

        flock.flocking();
        window.draw(sliderBar);
        window.draw(sliderKnob);
        window.display();
    }

    return 0;
}
