#ifndef BIRD_HPP
#define BIRD_HPP

#include <SFML/Graphics.hpp>
#include <vector>

#include "flock.hpp"

class Bird : public sf::Drawable {
  virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(bird_shape, states);
  }
  float size;
  sf::ConvexShape bird_shape;

 public:
  Bird(float);
  Bird(float, sf::Color const&);
  Bird() = default;
  void setSize(float);
  void setPosition(float, float);
  sf::Vector2f getPosition() const;
  void setRotation(float);
  void setFillColor(sf::Color const&);
  sf::Color const& getFillColor() const;
  void move(sf::Vector2f const&);
  sf::ConvexShape& getShape();
};

std::vector<Bird> create_birds(Flock&, sf::Color const&, float);
std::vector<Bird> create_birds(std::vector<Predator> const&, sf::Color const&,
                               float);
void update_birds(std::vector<Bird>&, Flock&, float);
void update_birds(std::vector<Bird>&, std::vector<Predator> const&, float);

#endif