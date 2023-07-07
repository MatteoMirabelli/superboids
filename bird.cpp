#include "bird.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>

Bird::Bird(float b_size) : size(), bird_shape() {
  assert(b_size > 0.);
  size = b_size;
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

Bird::Bird(float b_size, sf::Color const& color) : size(), bird_shape() {
  assert(b_size > 0.);
  size = b_size;
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

void Bird::setSize(float new_size) {
  assert(new_size > 0.);
  // crea un triangolo
  bird_shape.scale(new_size / size, new_size / size);
  size = new_size;
}

void Bird::setPosition(float x, float y) {
  // per movimento
  sf::Vector2f position(x, y);
  bird_shape.setPosition(position);
}

sf::Vector2f Bird::getPosition() const { return bird_shape.getPosition(); }

void Bird::setRotation(float angle) {
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

sf::ConvexShape& Bird::getShape() { return bird_shape; }

void Bird::move(sf::Vector2f const& offset) { bird_shape.move(offset); }

std::vector<Bird> create_birds(fk::Flock& flock, sf::Color const& color,
                               float margin) {
  std::vector<Bird> birds;
  std::transform(flock.begin(), flock.end(), std::back_inserter(birds),
                 [&margin, &color](bd::Boid& b) -> Bird {
                   Bird tr_boid(margin / 2, color);
                   tr_boid.setPosition(b.get_pos()[0] + margin,
                                       b.get_pos()[1] + margin);
                   tr_boid.setRotation(b.get_angle());
                   return tr_boid;
                 });
  return birds;
}

std::vector<Bird> create_birds(std::vector<pr::Predator> const& preds, sf::Color const& color,
                               float margin) {
  std::vector<Bird> birds;
  std::transform(preds.begin(), preds.end(), std::back_inserter(birds),
                 [&margin, &color](pr::Predator const& b) -> Bird {
                   Bird tr_boid(margin, color);
                   tr_boid.setPosition(b.get_pos()[0] + margin,
                                       b.get_pos()[1] + margin);
                   tr_boid.setRotation(b.get_angle());
                   return tr_boid;
                 });
  return birds;
}

void update_birds(std::vector<Bird>& birds, fk::Flock& flock, float margin) {
  int diff = flock.size() - birds.size();
  if (diff > 0) {
    auto color = birds[0].getFillColor();
    for (int i = 0; i < diff; ++i) {
      Bird tr_bird(margin / 2, color);
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
                 [&margin](bd::Boid& b, Bird& tr_boid) -> Bird {
                   tr_boid.setPosition(b.get_pos()[0] + margin,
                                       b.get_pos()[1] + margin);
                   tr_boid.setRotation(-b.get_angle());
                   return tr_boid;
                 });
}

void update_birds(std::vector<Bird>& birds, std::vector<pr::Predator> const& flock, float margin) {
  int diff = flock.size() - birds.size();
  if (diff > 0) {
    auto color = birds[0].getFillColor();
    for (int i = 0; i < diff; ++i) {
      Bird tr_bird(margin, color);
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
                 [&margin](pr::Predator const& b, Bird& tr_boid) -> Bird {
                   tr_boid.setPosition(b.get_pos()[0] + margin,
                                       b.get_pos()[1] + margin);
                   tr_boid.setRotation(-b.get_angle());
                   return tr_boid;
                 });
}
