#include "bird.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>

Bird::Bird(float const& b_size) : size(b_size), bird_shape() {
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

sf::Color const& Bird::getFillColor() const {
  return bird_shape.getFillColor();
}

std::vector<Bird> create_birds(Flock& flock, sf::Color const& color,
                               float margin) {
  std::vector<Bird> birds;
  std::transform(flock.begin(), flock.end(), std::back_inserter(birds),
                 [&margin, &color](Boid b) -> Bird {
                   Bird tr_boid(b.get_par_ds()*0.4, color);
                   tr_boid.setPosition(b.get_pos()[0] + margin,
                                       b.get_pos()[1] + margin);
                   tr_boid.setRotation(b.get_angle());
                   return tr_boid;
                 });
  return birds;
}

void update_birds(std::vector<Bird>& birds, Flock& flock, float margin) {
  int diff = flock.size() - birds.size();
  if (diff > 0) {
    auto color = birds[0].getFillColor();
    for (int i = 0; i < diff; ++i) {
      Bird tr_bird(flock.get_params().d_s*0.4, color);
      birds.push_back(tr_bird);
    }
  } else if (diff < 0) {
    auto sort_birds = [](Bird const& b1, Bird const& b2) -> bool {
      if (b1.getPosition().x == b2.getPosition().x)
        return b1.getPosition().y < b2.getPosition().y;
      else
        return b1.getPosition().x < b2.getPosition().x;
    };
    std::sort(birds.begin(), birds.end(), sort_birds);
    birds.erase(birds.begin(), birds.begin() - diff);
  }
  std::transform(flock.begin(), flock.end(), birds.begin(), birds.begin(),
                 [&margin](Boid& b, Bird& tr_boid) -> Bird {
                   tr_boid.setPosition(b.get_pos()[0] + margin,
                                       b.get_pos()[1] + margin);
                   tr_boid.setRotation(-b.get_angle());
                   return tr_boid;
                 });
}