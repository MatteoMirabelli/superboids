#include "bird.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>

Bird::Bird(float const& size, bool const& state) : size(size), state(state) {
  bird_shape_left.setPointCount(3);
  bird_shape_left.setPoint(0, sf::Vector2f(size / 2, size));
  bird_shape_left.setPoint(1, sf::Vector2f(size / 2, 0.375 * size));
  bird_shape_left.setFillColor(sf::Color::Black);
  bird_shape_left.setOutlineThickness(0);
  bird_shape_left.setOrigin(sf::Vector2f(size / 2, size / 2));
  bird_shape_right.setPointCount(3);
  bird_shape_right.setPoint(0, sf::Vector2f(size / 2, size));
  bird_shape_right.setPoint(1, sf::Vector2f(size / 2, 0.375 * size));
  bird_shape_right.setFillColor(sf::Color::Black);
  bird_shape_right.setOutlineThickness(0);
  bird_shape_right.setOrigin(sf::Vector2f(size / 2, size / 2));
  // da rifare anche qui!
  (state == true)
      ? bird_shape_left.setPoint(2, bird_shape_left.getOrigin() +
                                        sf::Vector2f(-std::cos(30) * size / 2,
                                                     -std::sin(30) * size / 2)),
      bird_shape_right.setPoint(2, bird_shape_right.getOrigin() +
                                       sf::Vector2f(std::cos(30) * size / 2,
                                                    -std::sin(30) * size / 2))
      : bird_shape_left.setPoint(2, bird_shape_left.getOrigin() +
                                        sf::Vector2f(-std::cos(60) * size / 2,
                                                     -std::sin(60) * size / 2)),
      bird_shape_right.setPoint(2, bird_shape_right.getOrigin() +
                                       sf::Vector2f(std::cos(60) * size / 2,
                                                    -std::sin(60) * size / 2));
}

void Bird::setPosition(float const& x, float const& y) {
  sf::Vector2f position(x, y);
  bird_shape_left.setPosition(position);
  bird_shape_right.setPosition(position);
}

void Bird::setRotation(float const& angle) {
  bird_shape_left.setRotation(angle);
  bird_shape_right.setRotation(angle);
}

void Bird::setState(bool const& state) { this->state = state; }

void Bird::animate() {
  // da rifare : qualcosa non torna
  (state == false)
      ? bird_shape_left.setPoint(2, sf::Vector2f(-std::cos(30) * size / 2,
                                                 -std::sin(30) * size / 2)),
      bird_shape_right.setPoint(2, bird_shape_right.getOrigin() +
                                       sf::Vector2f(std::cos(30) * size / 2,
                                                    -std::sin(30) * size / 2))
      : bird_shape_left.setPoint(2, bird_shape_left.getOrigin() +
                                        sf::Vector2f(-std::cos(60) * size / 2,
                                                     -std::sin(60) * size / 2)),
      bird_shape_right.setPoint(2, bird_shape_right.getOrigin() +
                                       sf::Vector2f(std::cos(60) * size / 2,
                                                    -std::sin(60) * size / 2));
  state = !state;
}
