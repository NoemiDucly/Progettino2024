#include "boids.hpp"
#include "flock.hpp"
#include "slider.hpp"

#include <ctime>
#include <iostream>
#include <limits>

int numberOfBoids() {
  std::cout << "insert desired (integer) number of boids between 2 and "
               "700:\n -numbers "
               "between 300 and 450 are recommended for a better performance "
               "\n -decimal numbers or numbers followed by letters will be rounded to first integer figure"
            << '\n';
  int n;
  while (true) {
    std::cin >> n;
    if (n >= 2 && n <= 700) {
      return n;
    } else {
      std::cout << "invalid input, try again." << '\n';
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
  }
}

int main() {
  sf::RenderWindow window(sf::VideoMode(800, 600), "simulation",
                          sf::Style::Titlebar);
  window.setPosition(
      sf::Vector2i(10, 50));
  window.setFramerateLimit(60);

  boids::Flock flock;
  std::vector<sf::CircleShape> shapes;

  Slider alignmentSlider(0, 1, 0, "alignment");
  alignmentSlider.setPosition(100, 100);

  Slider cohesionSlider(0, 0.005, 0, "cohesion");
  cohesionSlider.setPosition(100, 150);

  Slider separationSlider(0, 5, 0, "separation");
  separationSlider.setPosition(100, 200);

  Slider perceptionSlider(0, 60, 0, "perception");
  perceptionSlider.setPosition(100, 250);

  Slider sepDistanceSlider(0, 30, 0, "separation distance");
  sepDistanceSlider.setPosition(100, 300);

  srand(static_cast<unsigned int>(time(0)));

  int n { numberOfBoids() };

  for (int i = 0; i < n; ++i) {
    boids::Boid b(rand() % 800, rand() % 600, rand() % 3, rand() % 3);
    sf::CircleShape shape(5);
    shape.setPosition(static_cast<float>(b.getPosition().x_),
                      static_cast<float>(b.getPosition().y_));
    flock.addBoid(b);
    shapes.push_back(shape);
  }

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }

      alignmentSlider.handleEvent(event);
      cohesionSlider.handleEvent(event);
      separationSlider.handleEvent(event);
      perceptionSlider.handleEvent(event);
      sepDistanceSlider.handleEvent(event);
    }

    double a { alignmentSlider.getValue() };
    double c { cohesionSlider.getValue() };
    double s { separationSlider.getValue() };
    double d { perceptionSlider.getValue() };
    double ds { sepDistanceSlider.getValue() };

    window.clear(sf::Color::Black);

    for (long unsigned int i = 0; i < shapes.size(); ++i) {
      window.draw(shapes[i]);
      shapes[i].setPosition(
          static_cast<float>(flock.getBoid(i).getPosition().x_),
          static_cast<float>(flock.getBoid(i).getPosition().y_));
    }

    flock.flocking(a, c, s, d, ds);
    alignmentSlider.render(window);
    cohesionSlider.render(window);
    separationSlider.render(window);
    perceptionSlider.render(window);
    sepDistanceSlider.render(window);
    window.display();

    // Calcolo delle statistiche
    double averageDistance { flock.calculateAverageDistance() };
    double averageSpeed { flock.calculateAverageSpeed() };
    double distanceStdDev { flock.calculateDistanceStandardDeviation() };
    double speedStdDev { flock.calculateSpeedStandardDeviation() };

    // Visualizzazione delle statistiche sulla console
    std::cout << "\rAverage Distance: " << averageDistance << '\n';
    std::cout << "\rAverage Speed: " << averageSpeed << '\n';
    std::cout << "\rDistance Standard Deviation: " << distanceStdDev << '\n';
    std::cout << "\rSpeed Standard Deviation: " << speedStdDev << '\n';
  }
  return 0;
}
