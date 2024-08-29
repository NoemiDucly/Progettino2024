#include "slider.hpp"

#include <iostream>

Slider::Slider(double minValue, double maxValue, double value,
               const std::string& title)
    : minValue_(minValue),
      maxValue_(maxValue),
      value_(value),
      isDragging_(false) {
  sliderBar_.setSize(sf::Vector2f(100, 6));
  sliderBar_.setFillColor(sf::Color::White);

  sliderKnob_.setRadius(6);
  sliderKnob_.setFillColor(sf::Color::Red);

  if (!font.loadFromFile("VCR_OSD_MONO_1.001.ttf")) {
    std::cout << "font not loaded correctly" << '\n';
  }

  title_.setFont(font);
  title_.setString(title);
  title_.setCharacterSize(10);
  title_.setFillColor(sf::Color::White);
}

void Slider::setPosition(float x, float y) {
  sliderBar_.setPosition(x, y);
  //   sliderKnob_.setPosition(
  //       x + (value_ - minValue_) * 590 / (maxValue_ - minValue_), y - 5);
  sliderKnob_.setPosition(x, y - 5);
  title_.setPosition(x, y - 15);
}

void Slider::handleEvent(const sf::Event& event) {
  if (event.type == sf::Event::MouseButtonPressed) {
    if (sliderKnob_.getGlobalBounds().contains(
            static_cast<float>(event.mouseButton.x),
            static_cast<float>(event.mouseButton.y))) {
      isDragging_{true};
    }
  }

  if (event.type == sf::Event::MouseButtonReleased) {
    isDragging_{false};
  }

  if (event.type == sf::Event::MouseMoved && isDragging_) {
    float newPosX{
        static_cast<float>(event.mouseMove.x) - sliderKnob_.getRadius()};
    newPosX{std::max(
        sliderBar_.getPosition().x,
        std::min(newPosX, sliderBar_.getPosition().x + sliderBar_.getSize().x -
                              2 * sliderKnob_.getRadius()))};
    sliderKnob_.setPosition(newPosX, sliderKnob_.getPosition().y);
    value_{
        minValue_ + ((newPosX - sliderBar_.getPosition().x) /
                     (sliderBar_.getSize().x - 2 * sliderKnob_.getRadius())) *
                        (maxValue_ - minValue_)};
  }
}

void Slider::render(sf::RenderWindow& window) {
  window.draw(sliderBar_);
  window.draw(sliderKnob_);
  window.draw(title_);
};

double Slider::getValue() const { return value_; }
