#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>

#include "boid.hpp"
#include "flock.hpp"

int main() {
  try {
    Flock bd_flock{60., std::valarray<double>{25., 1., 0.07, 0.}, 70,
                   Boid{{800., 400.}, {3., 4.}}};
    std::vector<sf::ConvexShape> tr_boids;
    std::transform(bd_flock.begin(), bd_flock.end(),
                   std::back_inserter(tr_boids), [](Boid b) -> sf::ConvexShape {
                     sf::ConvexShape tr_boid;
                     tr_boid.setPointCount(3);
                     tr_boid.setPoint(0, sf::Vector2f(0.f, 0.f));
                     tr_boid.setPoint(1, sf::Vector2f(15.f, 0.f));
                     tr_boid.setPoint(2, sf::Vector2f(7.5f, 15.f));
                     tr_boid.setFillColor(sf::Color::Black);
                     tr_boid.setOrigin(7.5f, 7.5f);
                     tr_boid.setPosition(b.get_pos()[0], b.get_pos()[1]);
                     tr_boid.setRotation(b.get_angle());
                     return tr_boid;
                   });
    sf::RenderWindow window(sf::VideoMode(1920, 1080), "First boid test");
    window.setFramerateLimit(60);
    sf::Font font;
    if (!font.loadFromFile("Inter-Medium.otf"))
    {
        return 0;
    }
    sf::Text mag_display;
    mag_display.setFillColor(sf::Color::Blue);
    mag_display.setFont(font);
    mag_display.setPosition(65, 15);
    mag_display.setCharacterSize(15);
    auto init = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> step;
    while (window.isOpen()) {
      std::transform(bd_flock.begin(), bd_flock.end(), tr_boids.begin(),
                     tr_boids.begin(),
                     [](Boid& b, sf::ConvexShape& tr_boid) -> sf::ConvexShape {
                       tr_boid.setPosition(b.get_pos()[0], b.get_pos()[1]);
                       tr_boid.setRotation(-b.get_angle());
                       return tr_boid;
                     });
      // Process events
      sf::Event event;
      while (window.pollEvent(event)) {
        // Close window: exit
        if (event.type == sf::Event::Closed) window.close();
      }
      mag_display.setString("Draw time: " + std::to_string(step.count()));
      window.clear(sf::Color::White);
      window.draw(mag_display);
      init = std::chrono::steady_clock::now();
      for (sf::ConvexShape& tr_boid : tr_boids) {
        window.draw(tr_boid);
      }
      step = std::chrono::steady_clock::now() - init;
      window.display();
      bd_flock.update_flock_state(0.016);
    }
    return EXIT_SUCCESS;
  } catch (std::exception& e) {
    std::cerr << e.what();
  } catch (...) {
    std::cerr << "Unknown exception";
  }
}