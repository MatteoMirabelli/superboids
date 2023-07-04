#ifndef BIRD_HPP
#define BIRD_HPP

#include <SFML/Graphics.hpp>
#include <vector>

#include "flock.hpp"
// #include <cassert>

class Bird : public sf::Drawable {
  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(bird_shape, states);
  }
  float size;
  sf::ConvexShape bird_shape;

 public:
  Bird(float const&);
  Bird(float const&, sf::Color const&);
  Bird() = default;
  void setPosition(float const&, float const&);
  sf::Vector2f getPosition() const;
  void setRotation(float const&);
  void setFillColor(sf::Color const&);
  sf::Color const& getFillColor() const;
};

std::vector<Bird> create_birds(Flock&, sf::Color const&, float margin);
void update_birds(std::vector<Bird>&, Flock&, float margin);

#endif