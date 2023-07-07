#ifndef ANIMATION_HPP
#define ANIMATION_HPP

#include <SFML/Graphics.hpp>
#include <valarray>
#include <vector>

#include "bird.hpp"

namespace gf {

class Animate : public sf::Drawable, public sf::Transformable {
  float a_scale;
  int a_state;
  std::vector<sf::Texture> a_textures;
  sf::Sprite a_sprite;

 protected:
  virtual void draw(sf::RenderTarget&, sf::RenderStates) const;

 public:
  Animate(sf::Texture const&);
  Animate(float, sf::Texture const&);
  Animate(float, std::vector<sf::Texture> const&);
  Animate() = default;
  void addTexture(sf::Texture const&);
  void addTextures(std::vector<sf::Texture> const&);
  void addTextures(std::string const&);
  std::vector<sf::Texture> const& getTextures() const;
  void setPosition(float, float);
  void setPosition(sf::Vector2f const&);
  void setRotation(float);
  void setScale(float);
  void setState(int);
  void animate();
};

std::vector<Animate> create_animates(fk::Flock&,
                                     std::vector<sf::Texture> const&, float);
std::vector<Animate> create_animates(std::vector<pr::Predator> const&,
                                     std::vector<sf::Texture> const&, float);

class Tracker : public sf::Drawable, public sf::Transformable {
  sf::RectangleShape t_outer;
  sf::RectangleShape t_inner;
  Bird t_bird;
  sf::VertexArray t_path;
  std::valarray<float> t_range;
  std::valarray<float> t_pos;

 protected:
  virtual void draw(sf::RenderTarget&, sf::RenderStates) const;

 public:
  Tracker(std::valarray<float> const&, std::valarray<float> const&, float,
          float);
  void setPosition(sf::Vector2f const&);
  void setFillColors(sf::Color const&, sf::Color const&, sf::Color const&);
  void setOutlineColors(sf::Color const&, sf::Color const&, sf::Color const&);
  void setOutlineThickness(float, float, float);
  void update_pos(std::valarray<float> const&);
  void update_angle(float);
  sf::RectangleShape const& getOuter() const;
};

class StatusBar : public sf::Drawable, public sf::Transformable {
  sf::Text s_text;
  sf::RectangleShape s_outer;
  sf::RectangleShape s_bar;
  std::valarray<float> s_range;
  float s_value;
  sf::Text s_min;
  sf::Text s_max;

 protected:
  virtual void draw(sf::RenderTarget&, sf::RenderStates) const;

 public:
  StatusBar(std::string const&, sf::Font const&, float, float,
            std::valarray<float> const&);
  void setPosition(sf::Vector2f const&);
  void setColors(sf::Color const&, sf::Color const&);
  void setOutlineThickness(float);
  void setRange(std::valarray<float> const&);
  void update_value(float);
  void set_text(std::string const&);
};

}  // namespace gf

#endif