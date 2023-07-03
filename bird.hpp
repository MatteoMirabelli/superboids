#ifndef BIRD_HPP
#define BIRD_HPP

#include <SFML/Graphics.hpp>
#include <vector>
// #include <cassert>

class Bird : public sf::Drawable {
  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(bird_shape, states);
  }
  float size;
  sf::ConvexShape bird_shape;

 public:
  Bird(float const&);
  void setPosition(float const&, float const&);
  void setRotation(float const&);
  void setFillColor(sf::Color const&);
};

#endif