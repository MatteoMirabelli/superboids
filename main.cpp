#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>

#include "bird.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "predator.hpp"

int main() {
  try {
    Parameters params(80., 50., 1.5, 0.2, 0.3);
    Flock bd_flock{params, 100, 120.};
    std::vector<Bird> tr_boids;
    int i = 0;
    std::transform(bd_flock.begin(), bd_flock.end(),
                   std::back_inserter(tr_boids), [&i](Boid b) -> Bird {
                     Bird tr_boid(15., true);
                     if (i % 2 == 0) {
                     } else {
                       tr_boid.animate();
                     }
                     i++;
                     return tr_boid;
                   });
    i = 0;
    sf::RenderWindow window(sf::VideoMode(1920, 1080), "First boid test");
    window.setFramerateLimit(60);
    sf::Font font;
    if (!font.loadFromFile("Inter-Medium.otf")) {
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
                     tr_boids.begin(), [](Boid& b, Bird& tr_boid) -> Bird {
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
      mag_display.setString("Computation time: " +
                            std::to_string(step.count()));
      window.clear(sf::Color::White);
      window.draw(mag_display);

      for (Bird& tr_boid : tr_boids) {
        window.draw(tr_boid);
        tr_boid.animate();
        //++i;
      }
      window.display();
      init = std::chrono::steady_clock::now();
      bd_flock.update_flock_state(0.016);
      /*for(auto it = bd_flock.begin(); it < bd_flock.end(); ++it){
        auto dd = bd_flock.get_neighbours(it);
      }*/
      step = std::chrono::steady_clock::now() - init;
    }
    return EXIT_SUCCESS;
  } catch (std::exception& e) {
    std::cerr << e.what();
  } catch (...) {
    std::cerr << "Unknown exception";
  }
}