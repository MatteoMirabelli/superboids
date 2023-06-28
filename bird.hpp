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
  bool state;
  // std::vector<sf::Texture> textures;
  // sf::Sprite bird_shape;
  sf::ConvexShape bird_shape;

 public:
  Bird(float const&, bool const&);
  // void addTexture(sf::Texture const&);
  void setPosition(float const&, float const&);
  void setRotation(float const&);
  void setState(bool const&);
  void setFillColor(sf::Color const&);
  void animate();
};

#endif