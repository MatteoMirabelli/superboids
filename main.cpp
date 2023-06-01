#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <exception>
#include <iostream>

#include "boid.hpp"
#include "flock.hpp"

int main() {
  try {
    Flock bd_flock{50., std::valarray<double>{12., 0.01, 0., 0.}, 50,
                   Boid{{500., 300.}, {0., 0.}}};
    std::vector<sf::ConvexShape> tr_boids;
    std::transform(bd_flock.begin(), bd_flock.end(),
                   std::back_inserter(tr_boids), [](Boid b) -> sf::ConvexShape {
                     sf::ConvexShape tr_boid;
                     tr_boid.setPointCount(3);
                     tr_boid.setPoint(0, sf::Vector2f(0.f, 0.f));
                     tr_boid.setPoint(1, sf::Vector2f(8.f, 0.f));
                     tr_boid.setPoint(2, sf::Vector2f(4.f, 8.f));
                     tr_boid.setFillColor(sf::Color::Black);
                     tr_boid.setOrigin(5.f, 5.f);
                     tr_boid.setPosition(b.get_pos()[0], b.get_pos()[1]);
                     tr_boid.setRotation(b.get_angle());
                     return tr_boid;
                   });
    sf::RenderWindow window(sf::VideoMode(1000, 780), "First boid test");
    window.setFramerateLimit(60);
    while (window.isOpen()) {
      // Process events
      sf::Event event;
      while (window.pollEvent(event)) {
        // Close window: exit
        if (event.type == sf::Event::Closed) window.close();
      }
      std::transform(bd_flock.begin(), bd_flock.end(), tr_boids.begin(),
                     tr_boids.begin(), [](Boid b, sf::ConvexShape& tr_boid) {
                       tr_boid.setPosition(b.get_pos()[0], b.get_pos()[1]);
                       tr_boid.setRotation(-b.get_angle());
                       return tr_boid;
                     });
      window.clear(sf::Color::White);
      for (auto& tr_boid : tr_boids) {
        window.draw(tr_boid);
      }
      window.display();
      bd_flock.update_flock_state(0.01);
    }
    return EXIT_SUCCESS;
  } catch (std::exception& e) {
    std::cerr << e.what();
  } catch (...) {
    std::cerr << "Unknown exception";
  }
}