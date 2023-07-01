#ifndef ANIMATION_HPP
#define ANIMATION_HPP

#include <SFML/Graphics.hpp>
#include <vector>

class Animate : public sf::Drawable, public sf::Transformable {
  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(a_sprite, states);
  }
  float a_scale;
  int a_state;
  std::vector<sf::Texture> a_textures;
  sf::Sprite a_sprite;

 public:
  Animate(sf::Texture const&);
  Animate(float const&, sf::Texture const&);
  void addTexture(sf::Texture const&);
  void addTextures(std::vector<sf::Texture> const&);
  void addTextures(std::string const&);
  void setPosition(float const&, float const&);
  void setRotation(float const&);
  void setState(int const&);
  void animate();
};

bool rec_contains(sf::FloatRect const&, sf::FloatRect const&);

#endif