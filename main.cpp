#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>

#include "animation.hpp"
#include "bird.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "predator.hpp"

int main() {
  try {
    // inizializzo parametri stormo
    Parameters params(80., 25., 0.2, 0.05, 0.009);
    // prende parametri finestra (in fullscreen)
    double video_x = sf::VideoMode::getFullscreenModes()[1].width * 0.85;
    double video_y = sf::VideoMode::getFullscreenModes()[1].height;
    // inizializzo stormo
    Flock bd_flock{params, 100, 120., {video_x, video_y}};
    // vettore di oggetti grafici stormo:
    std::vector<Bird> tr_boids;
    // inizializzo predatore
    Predator predator(600., 300., 100., 0., 140., 70., 0.6, video_x, video_y,
                      90., 0.8);
    // qui servirà per caricare le texture:
    sf::Texture bd_texture;
    if (!bd_texture.loadFromFile("textures/eagle.png")) {
      return 0;
    }
    // oggetto grafico predatore:
    Animate tr_predator(bd_texture);

    // tr_predator.addTexture(bd_texture);

    // imposto colore, posizione e rotazione predatore
    // tr_predator.setFillColor(sf::Color::Red);
    tr_predator.setPosition(predator.get_pos()[0], predator.get_pos()[1]);
    tr_predator.setRotation(-predator.get_angle());
    int i = 0;
    // trasformo gli oggetti boid in oggetti grafici bird
    std::transform(bd_flock.begin(), bd_flock.end(),
                   std::back_inserter(tr_boids), [&](Boid b) -> Bird {
                     Bird tr_boid(15., true);
                     /*if (i % 2 == 0) {
                     } else {
                       tr_boid.animate();
                     }*/
                     // tr_boid.addTexture(bd_texture);
                     i++;
                     tr_boid.setPosition(b.get_pos()[0], b.get_pos()[1]);
                     tr_boid.setRotation(b.get_angle());
                     return tr_boid;
                   });
    i = 0;  // potrà tornarci di nuovo utile nel ciclo...
    // inizializzo finestra (in fullscreen)
    sf::RenderWindow window(sf::VideoMode::getFullscreenModes()[1],
                            "n-th boid test", sf::Style::Fullscreen);
    window.setFramerateLimit(60);
    // mettete anche da voi questa impostazione sotto,
    // aggiusta il framerate con quello del computer:
    window.setVerticalSyncEnabled(true);

    // carica font per il testo a schermo
    sf::Font font;
    if (!font.loadFromFile("Inter-Medium.otf")) {
      return 0;
    }
    // inizializza e sistema testo a schermo
    sf::Text mag_display;
    mag_display.setFillColor(sf::Color::Blue);
    mag_display.setFont(font);
    mag_display.setPosition(65, 15);
    mag_display.setCharacterSize(15);

    // finestra posizione com
    sf::RectangleShape rec(sf::Vector2f(video_x * 0.12, video_y * 0.12));
    rec.setFillColor(sf::Color::White);
    rec.setOutlineColor(sf::Color::Black);
    rec.setOutlineThickness(2);
    rec.setPosition(video_x + 40., 20.);

    // indicatore posizione com
    sf::CircleShape com_circle(5.);
    com_circle.setFillColor(sf::Color::Red);
    float com_x = bd_flock.get_com().get_pos()[0];
    float com_y = bd_flock.get_com().get_pos()[1];
    com_circle.setPosition(video_x + 40. + 0.12 * com_x, 20. + 0.12 * com_y);

    // per lo sfondo:
    /*sf::Texture bg_texture;
    if (!bg_texture.loadFromFile("textures/backg.png")) {
      return 0;
    }
    sf::Sprite backg;
    backg.setTexture(bg_texture);
    backg.setPosition(0., 0.);*/

    // calcolo tempi di computazione / disegno
    // inizializza init = istante di tempo
    auto init = std::chrono::steady_clock::now();
    // iniziali step = durata di tempo
    std::chrono::duration<double, std::milli> step;
    // game cicle
    while (window.isOpen()) {
      // aggiorna vettore di oggetti grafici bird
      tr_boids.erase(tr_boids.begin() + bd_flock.size(), tr_boids.end());
      std::transform(bd_flock.begin(), bd_flock.end(), tr_boids.begin(),
                     tr_boids.begin(), [](Boid& b, Bird& tr_boid) -> Bird {
                       tr_boid.setPosition(b.get_pos()[0], b.get_pos()[1]);
                       tr_boid.setRotation(-b.get_angle());
                       return tr_boid;
                     });
      // aggiorna oggetto grafico predatore
      tr_predator.setPosition(predator.get_pos()[0], predator.get_pos()[1]);
      tr_predator.setRotation(180. - predator.get_angle());
      //  Process events
      sf::Event event;
      while (window.pollEvent(event)) {
        // Close window: exit
        if (event.type == sf::Event::Closed) window.close();
        // non mi andava con la tastiera quindi ho messo il mouse
        if (event.type == sf::Event::MouseButtonPressed) {
          if (event.mouseButton.button == sf::Mouse::Left) {
            window.close();
          }
        }
      }
      // stampa a schermo tempo di calcolo
      mag_display.setString("Computation time: " +
                            std::to_string(step.count()));
      // clear window
      window.clear(sf::Color(200, 200, 255));
      // disegna sfondo
      // window.draw(backg);
      // disegna testo
      window.draw(mag_display);
      // disegna predatore
      window.draw(tr_predator);
      // disegna stormo
      for (Bird& tr_boid : tr_boids) {
        window.draw(tr_boid);
        // tr_boid.animate();
        //++i;
      }
      // finestra per posizione com flock
      window.draw(rec);
      if (rec_contains(rec.getGlobalBounds(), com_circle.getGlobalBounds())) {
        window.draw(com_circle);
      }
      // taaac TAAAAAAC
      window.display();
      // avvia cronometro
      init = std::chrono::steady_clock::now();
      // aggiorna stato flock e predatore
      bd_flock.update_flock_pred_state(0.016, true, predator);
      com_x = bd_flock.get_com().get_pos()[0];
      com_y = bd_flock.get_com().get_pos()[1];
      com_circle.setPosition(video_x + 40. + 0.12 * com_x, 20. + 0.12 * com_y);
      // ferma cronometro: calcola tempo di computazione
      step = std::chrono::steady_clock::now() - init;
    }
    return EXIT_SUCCESS;
  } catch (std::exception& e) {
    // eccezione standard (x metodi SL)
    std::cerr << e.what();
  } catch (...) {
    // eccezione generica
    std::cerr << "Unknown exception";
  }
}