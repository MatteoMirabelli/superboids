#ifndef BIRD_HPP
#define BIRD_HPP

#include <SFML/Graphics.hpp>
#include <SFML/System/String.hpp>
// #include <cassert>

class Bird : public sf::Drawable {
  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(bird_shape_left, states);
    target.draw(bird_shape_right, states);
  }
  float size;
  bool state;
  sf::ConvexShape bird_shape_left;
  sf::ConvexShape bird_shape_right;

 public:
  Bird(float const&, bool const&);
  void setPosition(float const&, float const&);
  void setRotation(float const&);
  void setState(bool const&);
  void animate();
};

#endif