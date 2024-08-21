#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <vector>

#include "boids.hpp"
#include "flock.hpp"

int main() {
  // biulding the window through constructor
  sf::RenderWindow window(
      sf::VideoMode(800, 600), "simulation",
      sf::Style::Titlebar);  // video mode defines the size of the window
  window.setPosition(
      sf::Vector2i(10, 50));  // window appears in the corner of the screen
                              //  window.setVerticalSyncEnabled(true);
  window.setFramerateLimit(60);

  sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

  sf::RectangleShape sliderBar(sf::Vector2f(600, 10));
  sliderBar.setPosition(100, 100);
  sliderBar.setFillColor(sf::Color::White);

  sf::CircleShape sliderKnob(10);
  sliderKnob.setFillColor(sf::Color::Red);
  sliderKnob.setPosition(100, 95);

  bool isDragging = false;
  double a = 0;

  boids::Flock flock;
  std::vector<sf::CircleShape> shapes;

  for (int i = 0; i < 100; ++i) {
    boids::Boid b(rand() % 800, rand() % 600);
    sf::CircleShape shape(8, 3);
    shape.setPosition(desktop.width, desktop.height);
    flock.addBoid(b);
    shapes.push_back(shape);
  }

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }

      if (event.type == sf::Event::MouseButtonPressed) {
        if (sliderKnob.getGlobalBounds().contains(event.mouseButton.x,
                                                  event.mouseButton.y)) {
          isDragging = true;
        }
      }

      if (event.type == sf::Event::MouseButtonReleased) {
        isDragging = false;
      }

      if (event.type == sf::Event::MouseMoved && isDragging) {
        float newPosX = event.mouseMove.x - sliderKnob.getRadius();
        newPosX = std::max(100.0f, std::min(newPosX, 690.0f));
        sliderKnob.setPosition(newPosX, sliderKnob.getPosition().y);

        // Mappare la posizione del knob a un valore tra 0 e 2
        a = ((newPosX - 100.0f) / 590.0f) * 2.0f;
      }
    }

    window.clear(sf::Color::Black);  // always call clear before draw

    for (int i = 0; i < shapes.size(); ++i) {
      window.draw(shapes[i]);
      shapes[i].setPosition(flock.getBoid(i).getPosition().x_,
                            flock.getBoid(i).getPosition().y_);
    }

    flock.flocking();
    window.draw(sliderBar);
    window.draw(sliderKnob);
    window.display();
    while (window.isOpen()) {
    // Gestione degli eventi

    // Calcolo delle statistiche
    double averageDistance = flock.calculateAverageDistance();
    double averageSpeed = flock.calculateAverageSpeed();
    double distanceStdDev = flock.calculateDistanceStandardDeviation();
    double speedStdDev = flock.calculateSpeedStandardDeviation();

    // Visualizzazione delle statistiche sulla console
    std::cout << "Average Distance: " << averageDistance << "\n";
    std::cout << "Average Speed: " << averageSpeed << "\n";
    std::cout << "Distance Standard Deviation: " << distanceStdDev << "\n";
    std::cout << "Speed Standard Deviation: " << speedStdDev << "\n";


}

  }
  return 0;
}
