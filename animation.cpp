#include "animation.hpp"

#include <SFML/Graphics.hpp>
#include <cassert>
#include <cmath>
#include <filesystem>

// costruttore, draw e metodi di Animate

Animate::Animate(sf::Texture const& texture)
    : a_scale(0.), a_state(0), a_sprite(texture), a_textures() {
  a_textures.push_back(texture);
}

Animate::Animate(float const& scale, sf::Texture const& texture)
    : a_scale(scale), a_state(0), a_sprite(texture), a_textures() {
  a_textures.push_back(texture);
  a_sprite.setScale(.3, .5);
  a_sprite.setOrigin(sf::Vector2f(a_sprite.getGlobalBounds().width / 2,
                                  a_sprite.getGlobalBounds().height / 2));
}

void Animate::draw(sf::RenderTarget& target, sf::RenderStates states) const {
  target.draw(a_sprite, states);
}

void Animate::addTexture(sf::Texture const& texture) {
  a_textures.push_back(texture);
}

void Animate::addTextures(std::vector<sf::Texture> const& textures) {
  for (auto& texture : textures) {
    a_textures.push_back(texture);
  }
}

void Animate::addTextures(std::string const& path) {
  for (auto const& entry :
       std::filesystem::recursive_directory_iterator(path)) {
    sf::Texture texture;
    if (entry.path().extension() == ".png" ||
        entry.path().extension() == ".jpg") {
      if (!texture.loadFromFile(entry.path().string())) {
        break;
      }
      a_textures.push_back(texture);
    }
  }
}

void Animate::setPosition(float const& x, float const& y) {
  // per movimento
  sf::Vector2f position(x, y);
  a_sprite.setPosition(position);
}

void Animate::setRotation(float const& angle) {
  // per rotazione
  a_sprite.setRotation(angle);
}

void Animate::setState(int const& state) {
  assert(state >= 0);
  int state_ = state % (a_textures.size() - 1);
  a_state = state_;
  a_sprite.setTexture(a_textures[state_]);
}

void Animate::animate() { setState(a_state + 1); }

// costruttore, draw e metodi di Tracker

Tracker::Tracker(std::valarray<float> const& range,
                 std::valarray<float> const& pos, float scale, float margin)
    : t_range(range),
      t_pos(pos),
      t_inner(),
      t_outer(),
      t_circle(),
      t_path(sf::LineStrip, 0) {
  assert(t_range.size() == 2);
  assert(t_pos.size() == 2);
  assert(scale > 0);
  t_inner.setSize(sf::Vector2f(t_range[0] * scale, t_range[1] * scale));
  t_outer.setSize(sf::Vector2f(t_range[0] * scale + 2 * margin,
                               t_range[1] * scale + 2 * margin));
  t_inner.move(margin, margin);
  t_circle.setRadius(margin / 2);
  t_circle.setOrigin(margin / 2, margin / 2);
  t_circle.setPosition(t_pos[0] * scale + margin, t_pos[1] * scale + margin);
}

void Tracker::draw(sf::RenderTarget& target, sf::RenderStates states) const {
  target.draw(t_outer, states);
  target.draw(t_inner, states);
  target.draw(t_path, states);
  if (t_inner.getGlobalBounds().contains(t_circle.getPosition())) {
    target.draw(t_circle, states);
  }
}

void Tracker::setPosition(sf::Vector2f const& position) {
  sf::Vector2f displacement = position - t_outer.getPosition();
  t_outer.setPosition(position);
  t_inner.move(displacement);
  t_circle.move(displacement);
}

void Tracker::setFillColors(sf::Color const& outer, sf::Color const& inner,
                            sf::Color const& circle) {
  t_outer.setFillColor(outer);
  t_inner.setFillColor(inner);
  t_circle.setFillColor(circle);
  for (int idx = 0; idx < t_path.getVertexCount(); ++idx) {
    t_path[idx].color = circle;
  }
}

void Tracker::setOutlineColors(sf::Color const& outer, sf::Color const& inner,
                               sf::Color const& circle) {
  t_outer.setOutlineColor(outer);
  t_inner.setOutlineColor(inner);
  t_circle.setOutlineColor(circle);
}

void Tracker::setOutlineThickness(float const& outer, float const& inner,
                                  float const& circle) {
  t_outer.setOutlineThickness(outer);
  t_inner.setOutlineThickness(inner);
  t_circle.setOutlineThickness(circle);
}

void Tracker::update_pos(std::valarray<float> const& position) {
  assert(position.size() == 2);
  t_pos = position;
  t_circle.setPosition(
      t_pos[0] * t_inner.getSize().x / t_range[0] + t_inner.getPosition().x,
      t_pos[1] * t_inner.getSize().y / t_range[1] + t_inner.getPosition().y);
  if (t_pos[0] > 0. && t_pos[1] > 0. && t_pos[0] < t_range[0] &&
      t_pos[1] < t_range[1]) {
    sf::Vertex vertex(t_circle.getPosition(), t_circle.getFillColor());
    t_path.append(vertex);
  }
}