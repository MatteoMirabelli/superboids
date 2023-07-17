#include "bird.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>

gf::Bird::Bird(float b_size) : size(), bird_shape() {
  assert(b_size > 0.f);
  size = b_size;
  // creates triangle
  bird_shape.setPointCount(3);
  bird_shape.setPoint(0, sf::Vector2f(0.f, 0.f));
  bird_shape.setPoint(1, sf::Vector2f(b_size, 0.f));
  bird_shape.setPoint(2, sf::Vector2f(b_size / 2, b_size));
  // default color
  bird_shape.setFillColor(sf::Color::Black);
  // centers origin (for movement and rotation)
  bird_shape.setOrigin(b_size / 2, b_size / 2);
}

gf::Bird::Bird(float b_size, sf::Color const& color) : size(), bird_shape() {
  assert(b_size > 0.f);
  size = b_size;
  // creates triangle
  bird_shape.setPointCount(3);
  bird_shape.setPoint(0, sf::Vector2f(0.f, 0.f));
  bird_shape.setPoint(1, sf::Vector2f(b_size, 0.f));
  bird_shape.setPoint(2, sf::Vector2f(b_size / 2, b_size));
  // custom color
  bird_shape.setFillColor(color);
  // centers origin (for movement and rotation)
  bird_shape.setOrigin(b_size / 2, b_size / 2);
}

void gf::Bird::draw(sf::RenderTarget& target, sf::RenderStates states) const {
  target.draw(bird_shape, states);
}

void gf::Bird::setSize(float new_size) {
  assert(new_size > 0.f);
  // sets triangle size by scaling
  bird_shape.scale(new_size / size, new_size / size);
  size = new_size;
}

void gf::Bird::setPosition(float x, float y) {
  // moves triangle
  sf::Vector2f position(x, y);
  bird_shape.setPosition(position);
}

sf::Vector2f gf::Bird::getPosition() const { return bird_shape.getPosition(); }

void gf::Bird::setRotation(float angle) {
  // rotation
  bird_shape.setRotation(angle);
}

void gf::Bird::setFillColor(sf::Color const& color) {
  // customizes color
  bird_shape.setFillColor(color);
}

sf::Color const& gf::Bird::getFillColor() const {
  // returns fill color
  return bird_shape.getFillColor();
}

void gf::Bird::setOutlineColor(sf::Color const& color) {
  // customizes outline color
  bird_shape.setOutlineColor(color);
}

sf::Color const& gf::Bird::getOutlineColor() const {
  // returns outline color
  return bird_shape.getOutlineColor();
}

void gf::Bird::setOutlineThickness(float thickness) {
  // customizes outline thickness
  bird_shape.setOutlineThickness(thickness);
}

float gf::Bird::getOutlineThickness() const {
  // returns outline thickness
  return bird_shape.getOutlineThickness();
}

void gf::Bird::move(sf::Vector2f const& offset) { bird_shape.move(offset); }

std::vector<gf::Bird> gf::create_birds(fk::Flock& flock, sf::Color const& color,
                                       float margin) {
  std::vector<gf::Bird> birds;
  std::transform(flock.begin(), flock.end(), std::back_inserter(birds),
                 [&margin, &color](bd::Boid& b) -> gf::Bird {
                   gf::Bird tr_boid(margin / 2.f, color);
                   tr_boid.setPosition(
                       static_cast<float>(b.get_pos()[0]) + margin,
                       static_cast<float>(b.get_pos()[1]) + margin);
                   tr_boid.setRotation(-static_cast<float>(b.get_angle()));
                   return tr_boid;
                 });
  return birds;
}

std::vector<gf::Bird> gf::create_birds(std::vector<pr::Predator> const& preds,
                                       sf::Color const& color, float margin) {
  std::vector<gf::Bird> birds;
  std::transform(preds.begin(), preds.end(), std::back_inserter(birds),
                 [&margin, &color](pr::Predator const& b) -> gf::Bird {
                   gf::Bird tr_boid(margin, color);
                   tr_boid.setPosition(
                       static_cast<float>(b.get_pos()[0]) + margin,
                       static_cast<float>(b.get_pos()[1]) + margin);
                   tr_boid.setRotation(-static_cast<float>(b.get_angle()));
                   return tr_boid;
                 });
  return birds;
}

void gf::update_birds(std::vector<gf::Bird>& birds, fk::Flock& flock,
                      float margin) {
  int diff = flock.size() - static_cast<int>(birds.size());
  if (diff > 0) {
    auto color = birds[0].getFillColor();
    for (int i = 0; i < diff; ++i) {
      gf::Bird tr_bird(margin / 2.f, color);
      birds.push_back(tr_bird);
    }
  } else if (diff < 0) {
    auto sort_birds = [](gf::Bird const& b1, gf::Bird const& b2) -> bool {
      if (b1.getPosition().x == b2.getPosition().x)
        return b1.getPosition().y < b2.getPosition().y;
      else
        return b1.getPosition().x < b2.getPosition().x;
    };
    std::sort(birds.begin(), birds.end(), sort_birds);
    birds.erase(birds.begin(), birds.begin() - diff);
  }
  std::transform(flock.begin(), flock.end(), birds.begin(), birds.begin(),
                 [&margin](bd::Boid& b, gf::Bird& tr_boid) -> gf::Bird {
                   tr_boid.setPosition(
                       static_cast<float>(b.get_pos()[0]) + margin,
                       static_cast<float>(b.get_pos()[1]) + margin);
                   tr_boid.setRotation(-static_cast<float>(b.get_angle()));
                   return tr_boid;
                 });
}

void gf::update_birds(std::vector<gf::Bird>& birds,
                      std::vector<pr::Predator> const& flock, float margin) {
  int diff = static_cast<int>(flock.size()) - static_cast<int>(birds.size());
  if (diff > 0) {
    auto color = birds[0].getFillColor();
    for (int i = 0; i < diff; ++i) {
      gf::Bird tr_bird(margin, color);
      birds.push_back(tr_bird);
    }
  } else if (diff < 0) {
    auto sort_birds = [](gf::Bird const& b1, gf::Bird const& b2) -> bool {
      if (b1.getPosition().x == b2.getPosition().x)
        return b1.getPosition().y < b2.getPosition().y;
      else
        return b1.getPosition().x < b2.getPosition().x;
    };
    std::sort(birds.begin(), birds.end(), sort_birds);
    birds.erase(birds.begin(), birds.begin() - diff);
  }
  std::transform(
      flock.begin(), flock.end(), birds.begin(), birds.begin(),
      [&margin](pr::Predator const& b, gf::Bird& tr_boid) -> gf::Bird {
        tr_boid.setPosition(static_cast<float>(b.get_pos()[0]) + margin,
                            static_cast<float>(b.get_pos()[1]) + margin);
        tr_boid.setRotation(-static_cast<float>(b.get_angle()));
        return tr_boid;
      });
}