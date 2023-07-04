#include "bird.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>

Bird::Bird(float const& b_size) : size(b_size), bird_shape() {
Bird::Bird(float const& b_size)
    : size(b_size), bird_shape() {
  // crea un triangolo
  bird_shape.setPointCount(3);
  bird_shape.setPoint(0, sf::Vector2f(0.f, 0.f));
  bird_shape.setPoint(1, sf::Vector2f(b_size, 0.f));
  bird_shape.setPoint(2, sf::Vector2f(b_size / 2, b_size));
  // colore di default (possibile eventualmente anche far passare al
  // costruttore)
  bird_shape.setFillColor(sf::Color::Black);
  // centra l'origine (x il movimento e la rotazione)
  bird_shape.setOrigin(b_size / 2, b_size / 2);
}

Bird::Bird(float const& b_size, sf::Color const& color)
    : size(b_size), bird_shape() {
  // crea un triangolo
  bird_shape.setPointCount(3);
  bird_shape.setPoint(0, sf::Vector2f(0.f, 0.f));
  bird_shape.setPoint(1, sf::Vector2f(b_size, 0.f));
  bird_shape.setPoint(2, sf::Vector2f(b_size / 2, b_size));
  // colore
  bird_shape.setFillColor(color);
  // centra l'origine (x il movimento e la rotazione)
  bird_shape.setOrigin(b_size / 2, b_size / 2);
}

void Bird::setPosition(float const& x, float const& y) {
  // per movimento
  sf::Vector2f position(x, y);
  bird_shape.setPosition(position);
}

sf::Vector2f Bird::getPosition() const { return bird_shape.getPosition(); }

void Bird::setRotation(float const& angle) {
  // per rotazione
  bird_shape.setRotation(angle);
}

void Bird::setFillColor(sf::Color const& color) {
  // strega comanda colore
  bird_shape.setFillColor(color);
}
