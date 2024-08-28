#ifndef SLIDER_HPP
#define SLIDER_HPP

#include <SFML/Graphics.hpp>
#include <string>

class Slider {
 private:
  sf::RectangleShape sliderBar_;
  sf::CircleShape sliderKnob_;
  double minValue_, maxValue_, value_;
  bool isDragging_;
  sf::Font font;
  sf::Text title_;

 public:
  Slider(double minValue, double maxValue, double startValue,
         const std::string& title);

  void setPosition(float x, float y);
  void handleEvent(const sf::Event& event);
  void render(sf::RenderWindow& window);
  double getValue() const;
};

#endif
