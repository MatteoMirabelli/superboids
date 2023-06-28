#include "animation.hpp"

#include <SFML/Graphics.hpp>
#include <cassert>
#include <cmath>
#include <filesystem>

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

bool rec_contains(sf::FloatRect const& container, sf::FloatRect const& contained) {
  return container.contains(contained.left, contained.top) &&
         container.contains(contained.left + contained.width,
                            contained.top + contained.height);
}