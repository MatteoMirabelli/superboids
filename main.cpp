#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <execution>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "animation.hpp"
#include "bird.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "multiflock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

int main() {
  try {
    // booleano per scelta periodiche / bordi
    bool behaviour = true;

    // inizializzo parametri stormo
    Parameters params(50., 25., 1.2, 0.1, 0.01);
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
    // inizializzo vettore di predatori
    std::vector<Predator> predators = random_predators(
        obstacles, 2, {video_x, video_y}, 150., 30., 1., 70., 0.7);
    // predators.push_back(predator);

    sf::Texture backg;
    if (!backg.loadFromFile("textures/wowo.jpg")) {
      return 0;
    }
    backg.setSmooth(false);

    // qui servirà per caricare le texture:
    sf::Texture bd_texture;
    if (!bd_texture.loadFromFile("textures/bomber.png")) {
      return 0;
    }
    bd_texture.setSmooth(true);

    // oggetti grafici stormo:
    std::vector<Bird> tr_boids =
        create_birds(bd_flock, sf::Color::Blue, margin);

    // oggetto grafico predatore:
    std::vector<Animate> tr_predators;
    for (int i = 0; i < predators.size(); ++i) {
      Animate tr_predator(
          predators[static_cast<unsigned int>(i)].get_par_ds() / 24.,
          bd_texture);
      tr_predators.push_back(tr_predator);
    }

    // imposto posizione e rotazione predatore
    for (int indx = 0; static_cast<unsigned int>(indx) < predators.size();
         ++indx) {
      tr_predators[static_cast<unsigned int>(indx)].setPosition(
          predators[static_cast<unsigned int>(indx)].get_pos()[0] + margin,
          predators[static_cast<unsigned int>(indx)].get_pos()[1] + margin);
      tr_predators[static_cast<unsigned int>(indx)].setRotation(
          180. - predators[static_cast<unsigned int>(indx)].get_angle());
    }

    // disegna ostacoli
    std::vector<sf::CircleShape> obs_circles;
    std::transform(
        obstacles.begin(), obstacles.end(), std::back_inserter(obs_circles),
        [&margin](Obstacle b) -> sf::CircleShape {
          sf::CircleShape ob_circ(b.get_size() * 0.7);
          ob_circ.setFillColor(sf::Color::Red);
          ob_circ.setPosition(b.get_pos()[0] + margin, b.get_pos()[1] + margin);
          return ob_circ;
        });

    // impostazioni di anti-aliasing
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    // inizializzo finestra
    sf::RenderWindow window(sf::VideoMode(static_cast<float>(window_x),
                                          static_cast<float>(window_y)),
                            "Star Boids", sf::Style::Default, settings);
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
    // rec_sim.setFillColor(sf::Color(203, 245, 255));
    rec_sim.setTexture(&backg);
    rec_sim.setOutlineColor(sf::Color::White);
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
    com_tracker.setFillColors(sf::Color::White, sf::Color(240, 240, 240),
                              sf::Color::Black);
    com_tracker.setOutlineColors(sf::Color::Black, sf::Color::Black,
                                 sf::Color::Black);
    com_tracker.setOutlineThickness(2, 0, 0);

    // inizializza e sistema testo a schermo
    sf::Text comp_text;
    comp_text.setFillColor(sf::Color::White);
    comp_text.setFont(font);
    comp_text.setCharacterSize(20);
    comp_text.setPosition(video_x + 2 * margin,
                          video_y * com_ratio + 3 * margin);

    // indicatore velocità media
    StatusBar speed_bar("Mean speed", font,
                        com_tracker.getOuter().getGlobalBounds().width, 20.,
                        {0., 350.});
    speed_bar.setColors(sf::Color::White, sf::Color::White);
    speed_bar.setPosition(sf::Vector2f{
        video_x + 2 * margin,
        video_y * com_ratio + 3 * margin + 4.2 * comp_text.getCharacterSize()});
    Statistics flock_stats = bd_flock.get_stats();

    // testo per guida ai comandi
    sf::Text commands_text("", font, 20);
    commands_text.setFillColor(sf::Color::White);
    commands_text.setString(
        "COMMANDS:\n"
        " > CTRL + B : generate boid\n"
        "   (hold for loop generation)\n"
        " > CTRL + P : generate predator\n"
        " > CTRL + O : generate obstacle\n"
        " > esc : close program");
    commands_text.setOrigin(0, commands_text.getLocalBounds().height);
    commands_text.setPosition(video_x + 2 * margin, window_y - margin);

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

    // booleano per la generazione in loop di boid
    bool boid_gen = false;

    // game cicle
    while (window.isOpen()) {
      init = std::chrono::steady_clock::now();
      // aggiorna vettore di oggetti grafici bird
      update_birds(tr_boids, bd_flock, margin);
      for (int i = 0;
           i < static_cast<int>(predators.size() - tr_predators.size()); ++i) {
        Animate tr_predator(
            predators[static_cast<unsigned int>(i)].get_par_ds() / 24.,
            bd_texture);
        tr_predators.push_back(tr_predator);
      }
      // aggiorna oggetti grafici predatori
      for (int indx = 0; static_cast<unsigned int>(indx) < predators.size();
           ++indx) {
        tr_predators[static_cast<unsigned int>(indx)].setPosition(
            predators[indx].get_pos()[0] + margin,
            predators[static_cast<unsigned int>(indx)].get_pos()[1] + margin);
        tr_predators[static_cast<unsigned int>(indx)].setRotation(
            180. - predators[static_cast<unsigned int>(indx)].get_angle());
      }
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
        step_ss.str("");
        step_ss << "Mean distance (px): " << std::setw(8)
                << std::setprecision(1) << std::fixed << flock_stats.av_dist
                << " +/- " << std::setprecision(1) << std::fixed
                << flock_stats.dist_RMS << '\n';
        step_ss << "Mean speed (px/s): " << std::setw(8) << std::setprecision(1)
                << std::fixed << flock_stats.av_vel << " +/- "
                << std::setprecision(1) << std::fixed << flock_stats.vel_RMS;
        speed_bar.set_text(step_ss.str());
      }
      step_update += std::chrono::steady_clock::now() - init;

      //  Process events
      sf::Event event;
      while (window.pollEvent(event)) {
        switch (event.type) {
          case sf::Event::Closed:
            window.close();
            break;
          case sf::Event::KeyPressed:
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
              window.close();
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
              if (sf::Keyboard::isKeyPressed(sf::Keyboard::B)) {
                // attiva la generazione di Boid
                boid_gen = true;
              } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::P)) {
                // aggiunge predatore (no generazione continua!)
                add_predator(predators, obstacles, {video_x, video_y}, 150.,
                             30., 1., 70., 0.7);
              } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::O)) {
                // aggiunge ostacolo (no gen continua!)
              }
            }
            break;
          case sf::Event::KeyReleased:
            if (event.key.code == sf::Keyboard::B ||
                event.key.code == sf::Keyboard::LControl) {
              // disattiva la generazione di Boid
              boid_gen = false;
            }
            break;
          default:
            break;
        }
      }
      if (boid_gen) bd_flock.add_boid(obstacles);

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
      window.clear(sf::Color::Black);
      // disegna riquadro simulazione
      window.draw(rec_sim);
      // disegna stormo
      for (Bird& tr_boid : tr_boids) {
        window.draw(tr_boid);
      }
      // disegna predatore
      for (Animate& tr_predator : tr_predators) window.draw(tr_predator);
      // disegna ostacoli
      for (sf::CircleShape& obs : obs_circles) {
        window.draw(obs);
      }
      // disegna tracker centro di massa
      window.draw(com_tracker);
      // disegna testo
      window.draw(comp_text);
      // disegna barra velocità media
      window.draw(speed_bar);
      // disegna istruzioni
      window.draw(commands_text);
      // taaac TAAAAAAC
      window.display();
      // calcola tempo di disegno
      step_draw += std::chrono::steady_clock::now() - init;
      // avvia cronometro per computazione
      init = std::chrono::steady_clock::now();
      // aggiorna stato flock e predatore
      bd_flock.update_global_state(0.0166, behaviour, predators, obstacles);
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