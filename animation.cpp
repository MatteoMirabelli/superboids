#include "animation.hpp"

#include <SFML/Graphics.hpp>
#include <cassert>
#include <cmath>
#include <filesystem>

// costruttore, draw e metodi di Animate

Animate::Animate(sf::Texture const& texture)
    : a_scale(0.), a_state(0), a_textures(), a_sprite(texture) {
  a_textures.push_back(texture);
  a_sprite.setOrigin(sf::Vector2f(a_sprite.getGlobalBounds().width / 2,
                                  a_sprite.getGlobalBounds().height / 2));
}

Animate::Animate(float const& scale, sf::Texture const& texture)
    : a_scale(scale), a_state(0), a_textures(), a_sprite(texture) {
  a_textures.push_back(texture);
  a_sprite.setScale(scale, scale);
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
    : t_outer(),
      t_inner(),
      t_circle(),
      t_path(sf::LineStrip, 0),
      t_range(range),
      t_pos(pos) {
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
  for (int idx = 0; static_cast<unsigned int>(idx) < t_path.getVertexCount();
       ++idx) {
    t_path[idx].color = sf::Color(circle.r, circle.g, circle.b, 100);
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
    if (t_path.getVertexCount() < 30) {
      sf::Vertex vertex(
          t_circle.getPosition(),
          sf::Color(t_circle.getFillColor().r, t_circle.getFillColor().g,
                    t_circle.getFillColor().b, 100));
      t_path.append(vertex);
    } else {
      for (int idx = 0;
           static_cast<unsigned int>(idx) < t_path.getVertexCount() - 1;
           ++idx) {
        t_path[static_cast<unsigned int>(idx)].position =
            t_path[static_cast<unsigned int>(idx) + 1].position;
      }
      t_path[t_path.getVertexCount() - 1].position = t_circle.getPosition();
    }
  }
}

sf::RectangleShape const& Tracker::getOuter() const { return t_outer; }

// costruttore, draw e metodi di StatusBar

StatusBar::StatusBar(std::string const& title, sf::Font const& font,
                     float width, float height,
                     std::valarray<float> const& range)
    : s_text(title, font) {
  assert(range.size() == 2 && range[0] < range[1] && width > 0 && height > 0);
  s_outer = sf::RectangleShape(sf::Vector2f{width, height});
  s_bar = sf::RectangleShape(sf::Vector2f{width, height});
  s_range = range;
  s_value = range[0];
  s_text.setCharacterSize(height);
  s_outer.setFillColor(sf::Color::Transparent);
  s_outer.setOutlineColor(sf::Color::Black);
  s_text.setFillColor(sf::Color::Black);
  s_outer.setOutlineThickness(2);
  s_bar.setFillColor(sf::Color::Black);
  s_outer.setPosition(0, s_text.getGlobalBounds().top +
                             s_text.getGlobalBounds().height +
                             s_text.getCharacterSize() / 2);
  s_bar.setPosition(s_outer.getPosition());
  std::ostringstream s_min_max;
  s_min_max << std::setprecision(1) << std::fixed << s_range[0];
  s_min = sf::Text(s_min_max.str(), font, height);
  s_min.setFillColor(sf::Color::Black);
  s_min_max.str("");
  s_min_max << std::setprecision(1) << std::fixed << s_range[1];
  s_max = sf::Text(s_min_max.str(), font, height);
  s_max.setFillColor(sf::Color::Black);
  s_min_max.str("");
  s_text.setOrigin(0, 0);
  s_min.setPosition(0.05 * s_min.getLocalBounds().width, s_outer.getPosition().y +
                           s_outer.getGlobalBounds().height +
                           s_min.getCharacterSize() / 7);
  s_max.setOrigin(1.1 * s_max.getLocalBounds().width, 0);
  s_max.setPosition(s_outer.getGlobalBounds().width,
                    s_outer.getPosition().y + s_outer.getGlobalBounds().height +
                        s_max.getCharacterSize() / 7);
}

void StatusBar::draw(sf::RenderTarget& target, sf::RenderStates states) const {
  target.draw(s_text, states);
  target.draw(s_bar, states);
  target.draw(s_outer, states);
  target.draw(s_min, states);
  target.draw(s_max, states);
}

void StatusBar::setPosition(sf::Vector2f const& position) {
  sf::Vector2f displacement = position - s_text.getPosition();
  s_text.setPosition(position);
  s_outer.move(displacement);
  s_bar.move(displacement);
  s_min.move(displacement);
  s_max.move(displacement);
}

void StatusBar::setColors(sf::Color const& text_color,
                          sf::Color const& bar_color) {
  s_text.setFillColor(text_color);
  s_min.setFillColor(text_color);
  s_max.setFillColor(text_color);
  s_bar.setFillColor(bar_color);
  s_outer.setOutlineColor(text_color);
}
void StatusBar::setOutlineThickness(float thickness) {
  s_outer.setOutlineThickness(thickness);
}
void StatusBar::setRange(std::valarray<float> const& new_range) {
  assert(new_range.size() == 2 && new_range[0] < new_range[1]);
  s_range = new_range;
  std::ostringstream s_min_max;
  s_min_max << std::setprecision(1) << std::fixed << s_range[0];
  s_min.setString(s_min_max.str());
  s_min_max.str("");
  s_min_max << std::setprecision(1) << std::fixed << s_range[1];
  s_max.setString(s_min_max.str());
  s_min_max.str("");
}

void StatusBar::update_value(float new_value) {
  assert(new_value >= s_range[0] && new_value <= s_range[1]);
  s_value = new_value;
  s_bar.setScale(s_value / (s_range[1] - s_range[0]), 1);
}
void StatusBar::set_text(std::string const& new_text) {
  s_text.setString(new_text);
  s_outer.setPosition(s_text.getGlobalBounds().left,
                      s_text.getGlobalBounds().top +
                          s_text.getGlobalBounds().height +
                          s_text.getCharacterSize() / 2);
  auto displacement = s_outer.getPosition() - s_bar.getPosition();
  s_bar.move(displacement);
  s_min.move(displacement);
  s_max.move(displacement);
}