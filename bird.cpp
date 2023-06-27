#include "bird.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>

Bird::Bird(float const& b_size, bool const& b_state)
    : size(b_size), state(b_state), bird_shape() {
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

  /*bird_shape.setTextureRect(sf::IntRect(0, 0, size, size));
  bird_shape.setOrigin(sf::Vector2f(size / 2, size / 2));*/
}

// qui per il futuro:
/*void Bird::addTexture(sf::Texture const& texture) {
  //textures.push_back(texture);
  bird_shape.setTexture(texture);
}*/

void Bird::setPosition(float const& x, float const& y) {
  // per movimento
  sf::Vector2f position(x, y);
  bird_shape.setPosition(position);
}

void Bird::setRotation(float const& angle) {
  // per rotazione
  bird_shape.setRotation(angle);
}

void Bird::setState(bool const& statet) { this->state = statet; }

void Bird::setFillColor(sf::Color const& color) {
  // strega comanda colore
  bird_shape.setFillColor(color);
}

void Bird::animate() {
  // quando avremo più forme / texture per sprite aggiorna
  // per ora flippa solo stato
  state = !state;
}
