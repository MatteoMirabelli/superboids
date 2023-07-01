#ifndef ANIMATION_HPP
#define ANIMATION_HPP

#include <SFML/Graphics.hpp>
#include <valarray>
#include <vector>

class Animate : public sf::Drawable, public sf::Transformable {
  float a_scale;
  int a_state;
  std::vector<sf::Texture> a_textures;
  sf::Sprite a_sprite;

 protected:
  virtual void draw(sf::RenderTarget&, sf::RenderStates) const;

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

class Tracker : public sf::Drawable, public sf::Transformable {
  sf::RectangleShape t_outer;
  sf::RectangleShape t_inner;
  sf::CircleShape t_circle;
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
  void setOutlineThickness(float const&, float const&, float const&);
  void update_pos(std::valarray<float> const&);
  sf::RectangleShape const& getOuter() const;
};

class StatusBar : public sf::Drawable, public sf::Transformable {
  sf::Text s_text;
  sf::RectangleShape s_outer;
  sf::RectangleShape s_bar;
  std::valarray<float> s_range;
  float s_value;

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

#endif