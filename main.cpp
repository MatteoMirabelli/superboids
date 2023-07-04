#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "animation.hpp"
#include "bird.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "predator.hpp"

int main() {
  try {
    // inizializzo parametri stormo
    Parameters params(80., 25., 0.2, 0.05, 0.009);
    // parametri finestra e video tarati sul device
    float window_x = sf::VideoMode::getFullscreenModes()[0].width;
    float window_y = sf::VideoMode::getFullscreenModes()[0].height * 0.92;
    float video_x = window_x * 0.75;
    float video_y = window_y * 0.96;
    // margini
    float margin = (window_y - video_y) / 2;
    std::vector<Obstacle> obstacles =
        generate_obstacles(10, 20., {video_x, video_y});

    // inizializzo stormo
    Flock bd_flock{params, 100, 120., {video_x, video_y}, obstacles};
    // vettore di oggetti grafici stormo:
    std::vector<Bird> tr_boids;
    // inizializzo predatore
    Predator predator(600., 300., 100., 0., 140., 30., 0.6, video_x, video_y,
                      90., 0.8);

    sf::Texture backg;
    /*if (!backg.loadFromFile("textures/sky.jpg")) {
      return 0;
    }
    backg.setSmooth(true);*/

    // qui servirà per caricare le texture:
    sf::Texture bd_texture;
    if (!bd_texture.loadFromFile("textures/eagle.png")) {
      return 0;
    }
    // oggetto grafico predatore:
    Animate tr_predator(bd_texture);

    // tr_predator.addTexture(bd_texture);

    // imposto colore, posizione e rotazione predatore
    tr_predator.setPosition(predator.get_pos()[0] + margin,
                            predator.get_pos()[1] + margin);
    tr_predator.setRotation(180. - predator.get_angle());

    // trasformo gli oggetti boid in oggetti grafici bird
    std::transform(bd_flock.begin(), bd_flock.end(),
                   std::back_inserter(tr_boids), [&margin](Boid b) -> Bird {
                     Bird tr_boid(15.);
                     tr_boid.setPosition(b.get_pos()[0] + margin,
                                         b.get_pos()[1] + margin);
                     tr_boid.setRotation(b.get_angle());
                     return tr_boid;
                   });

    // disegna ostacoli
    std::vector<sf::CircleShape> obs_circles;
    std::transform(
        obstacles.begin(), obstacles.end(), std::back_inserter(obs_circles),
        [&margin](Obstacle b) -> sf::CircleShape {
          sf::CircleShape ob_circ(b.get_size());
          ob_circ.setFillColor(sf::Color::Red);
          ob_circ.setPosition(b.get_pos()[0] + margin, b.get_pos()[1] + margin);
          return ob_circ;
        });

    // impostazioni di anti-aliasing
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    // inizializzo finestra
    sf::RenderWindow window(sf::VideoMode(window_x, window_y), "n-th boid test",
                            sf::Style::Default, settings);
    window.setFramerateLimit(60);
    // posiziona finestra in alto a sinistra
    window.setPosition(sf::Vector2i(0, window_x * 0.023));
    // ottimizza il framerate per quello del computer
    window.setVerticalSyncEnabled(true);

    // carica font per il testo a schermo
    sf::Font font;
    if (!font.loadFromFile("Inter-Medium.otf")) {
      return 0;
    }

    // riquadro simulazione
    sf::RectangleShape rec_sim(sf::Vector2f(video_x, video_y));
    rec_sim.setFillColor(sf::Color::White);
    // rec_sim.setTexture(&backg);
    rec_sim.setOutlineColor(sf::Color::Black);
    rec_sim.setOutlineThickness(2);
    rec_sim.setPosition(margin, margin);

    // finestra posizione com
    float com_ratio = (window_x - 4 * margin - video_x) / video_x;

    float pos_com_x = bd_flock.get_com().get_pos()[0];
    float pos_com_y = bd_flock.get_com().get_pos()[1];
    float tracker_x = window_x - video_x * com_ratio - 2 * margin;
    float tracker_y = margin;

    Tracker com_tracker(std::valarray<float>{video_x, video_y},
                        {pos_com_x, pos_com_y}, com_ratio, margin / 2);
    com_tracker.setPosition(sf::Vector2f{tracker_x, tracker_y});
    com_tracker.setFillColors(sf::Color::White, sf::Color(220, 220, 220),
                              sf::Color::Blue);
    com_tracker.setOutlineColors(sf::Color::Black, sf::Color::Black,
                                 sf::Color::Black);
    com_tracker.setOutlineThickness(2, 0, 0);

    // inizializza e sistema testo a schermo
    sf::Text comp_text;
    comp_text.setFillColor(sf::Color::Black);
    comp_text.setFont(font);
    comp_text.setCharacterSize(22);
    comp_text.setPosition(video_x + 2 * margin,
                          video_y * com_ratio + 3 * margin);

    // indicatore velocità media
    StatusBar speed_bar("Mean speed", font,
                        com_tracker.getOuter().getGlobalBounds().width, 22.,
                        {0., 350.});
    speed_bar.setPosition(sf::Vector2f{
        video_x + 2 * margin,
        video_y * com_ratio + 3 * margin + 4.2 * comp_text.getCharacterSize()});
    Statistics flock_stats = bd_flock.get_stats();

    // calcolo tempi di computazione / disegno
    // inizializza init = istante di tempo
    auto init = std::chrono::steady_clock::now();
    // iniziali step = durata di tempo
    std::chrono::duration<double, std::milli> step_cmpt{
        std::chrono::duration<double, std::milli>::zero()};
    std::chrono::duration<double, std::milli> step_draw{
        std::chrono::duration<double, std::milli>::zero()};
    std::chrono::duration<double, std::milli> step_update{
        std::chrono::duration<double, std::milli>::zero()};
    // stringstream per convertire durate in stringhe
    std::ostringstream step_ss;

    // counter per operazioni a intervalli discreti
    int counter = 0;

    // game cicle
    while (window.isOpen()) {
      init = std::chrono::steady_clock::now();
      // aggiorna vettore di oggetti grafici bird
      // da implementare in file separato per gestire aggiunta/rimozione
      tr_boids.erase(tr_boids.begin() + bd_flock.size(), tr_boids.end());
      std::transform(bd_flock.begin(), bd_flock.end(), tr_boids.begin(),
                     tr_boids.begin(),
                     [&margin](Boid& b, Bird& tr_boid) -> Bird {
                       tr_boid.setPosition(b.get_pos()[0] + margin,
                                           b.get_pos()[1] + margin);
                       tr_boid.setRotation(-b.get_angle());
                       return tr_boid;
                     });
      // aggiorna oggetto grafico predatore
      tr_predator.setPosition(predator.get_pos()[0] + margin,
                              predator.get_pos()[1] + margin);
      tr_predator.setRotation(180. - predator.get_angle());
      // aggiorna posizione com
      if (counter % 4 == 0) {
        pos_com_x = bd_flock.get_com().get_pos()[0];
        pos_com_y = bd_flock.get_com().get_pos()[1];
        /*com_circle.setPosition(com_rec.getPosition().x + com_x * com_ratio,
                               com_rec.getPosition().y + com_y * com_ratio);*/
        com_tracker.update_pos({pos_com_x, pos_com_y});

        // aggiorna velocità media
        bd_flock.update_stats();
        flock_stats = bd_flock.get_stats();
        speed_bar.update_value(flock_stats.av_vel);
      }
      step_update += std::chrono::steady_clock::now() - init;
      //  Process events
      sf::Event event;
      while (window.pollEvent(event)) {
        // Close window: exit
        if (event.type == sf::Event::Closed) window.close();
        // eventualmente altri comandi qui:
      }
      // stampa a schermo tempo di calcolo e disegno (mediato su 4 frame)
      if (counter % 4 == 0) {
        step_ss.str("");
        step_ss << "Computation time: " << std::setprecision(2) << std::fixed
                << step_cmpt.count() / 4 << " ms \n";
        step_ss << "Drawing time: " << std::setprecision(2) << std::fixed
                << step_draw.count() / 4 << " ms \n";
        step_ss << "Update time: " << std::setprecision(2) << std::fixed
                << step_update.count() / 4 << " ms";
        comp_text.setString(step_ss.str());
        step_cmpt = std::chrono::duration<double, std::milli>::zero();
        step_draw = std::chrono::duration<double, std::milli>::zero();
        step_update = std::chrono::duration<double, std::milli>::zero();
      }

      // calcola tempo di computazione
      init = std::chrono::steady_clock::now();
      // clear window
      window.clear(sf::Color(210, 210, 210));
      // disegna riquadro simulazione
      window.draw(rec_sim);
      // disegna ostacoli
      for (sf::CircleShape& obs : obs_circles) {
        window.draw(obs);
      }
      // disegna stormo
      for (Bird& tr_boid : tr_boids) {
        window.draw(tr_boid);
      }
      // disegna predatore
      window.draw(tr_predator);
      // disegna tracker centro di massa
      window.draw(com_tracker);
      // disegna testo
      window.draw(comp_text);
      // disegna barra velocità media
      window.draw(speed_bar);
      // taaac TAAAAAAC
      window.display();
      // calcola tempo di disegno
      step_draw += std::chrono::steady_clock::now() - init;
      // avvia cronometro per computazione
      init = std::chrono::steady_clock::now();
      // aggiorna stato flock e predatore
      bd_flock.update_flock_pred_state(0.0166, true, obstacles, predator);
      // ferma cronometro: calcola tempo di computazione
      step_cmpt += std::chrono::steady_clock::now() - init;

      // incrementa counter
      (counter == 1000) ? counter = 0 : ++counter;
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