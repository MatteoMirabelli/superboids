#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <execution>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "animation.hpp"
#include "bird.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "multiflock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

int main() {
  // try-catch structure is used to handle exceptions
  try {
    // -- GENERAL --

    // boolean to choose mode: classic (false) vs Star Boids (true)
    bool mode = false;

    // -- USER INPUT --

    std::cout << " Choose mode: classic (0) or Star Boids (1): ";
    std::cin >> mode;

    if (mode == true) {
      // funny intro
      std::cout << "\n A long time ago,\n in a stack far, far away...\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(800));
      std::cout << '\n';
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << '\n';
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "============================= STAR BOIDS "
                   "=============================\n\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "======================= Episode 11: A New Heap "
                   "=======================\n\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << " It is a period of hunt. Rebel boids, striking from an "
                   "hidden nest,\n ";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "have won their first victory against the evil Predator "
                   "Empire.\n ";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "During the battle, Boid spies managed to steal secret "
                   "headers of the\n ";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "Empire's ultimate weapon, the DEATH SEGFAULT, an armored "
                   "derived\n ";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout
          << "object with enough power to destroy an entire flock. Pursued "
             "by the\n ";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "Empire's sinister agents, young boids race home across the "
                   "screen,\n ";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout
          << "custodians of the stolen files that can save their people and\n ";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "restore freedom to the RAM...\n\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      std::cout << "======================== press Enter to start "
                   "========================\n";

    } else {
      std::cout << "======================== press Enter to start "
                   "========================\n";
    }
    std::cin.get();

    // -- WINDOW PARAMETERS --

    // window and simulation space dimensions,
    // fitted onto screen properties for better portability

    // window dimensions
    float window_x = sf::VideoMode::getFullscreenModes()[0].width;
    float window_y = sf::VideoMode::getFullscreenModes()[0].height * 0.92;

    // simulation space dimensions
    float video_x = window_x * 0.75;
    float video_y = window_y * 0.96;

    // margin value, also fitted onto screen
    float margin = (window_y - video_y) / 2;

    // -- SIMULATION OBJECTS --

    // boolean for behaviour:
    // false = borders, true = toroidal
    bool behaviour = true;

    // initialization of flock parameters
    Parameters params(50., 25., 1.2, 0.1, 0.01);

    // initialization of obstacles
    std::vector<Obstacle> obstacles =
        generate_obstacles(10, 20., {video_x, video_y});

    // flock initialization
    Flock bd_flock{params, 100, 120., {video_x, video_y}, obstacles};

    // predators initialization
    std::vector<Predator> predators = random_predators(
        obstacles, 2, {video_x, video_y}, 150., 30., 1., 70., 1.2);

    // -- GRAPHIC OBJECTS --

    // define palette
    std::vector<sf::Color> palette(2);
    switch (mode) {
      case false:
        palette[0] = sf::Color(240, 240, 240);
        palette[1] = sf::Color::Black;
        break;
      case true:
        palette[0] = sf::Color::Black;
        palette[1] = sf::Color::White;
        break;
    }

    // load textures
    sf::Texture backg;
    if (!backg.loadFromFile("textures/wowo.jpg")) {
      return 0;
    }
    backg.setSmooth(true);

    sf::Texture boid_texture_normal;
    if (!boid_texture_normal.loadFromFile("textures/xwing.png")) {
      return 0;
    }
    boid_texture_normal.setSmooth(true);

    sf::Texture boid_texture_sped;
    if (!boid_texture_sped.loadFromFile("textures/xwing_speed.png")) {
      return 0;
    }

    sf::Texture pred_texture_normal;
    if (mode == true) {
      if (!pred_texture_normal.loadFromFile("textures/falcon.png")) {
        return 0;
      }
      pred_texture_normal.setSmooth(true);
    }

    sf::Texture pred_texture_sped;
    if (mode == true) {
      if (!pred_texture_sped.loadFromFile("textures/falcon_speed.png")) {
        return 0;
      }
      pred_texture_sped.setSmooth(true);
    }

    sf::Texture obs_texture;
    if (mode == true) {
      if (!obs_texture.loadFromFile("textures/obstar-cle.png")) {
        return 0;
      }
      obs_texture.setSmooth(true);
    }

    // loads font
    sf::Font font;
    if (!font.loadFromFile("cmunsx.ttf")) {
      return 0;
    }

    // boids graphical objects:
    std::vector<Bird> graph_boids_tr;
    std::vector<Animate> graph_boids_sp;
    if (mode == false) {
      graph_boids_tr = create_birds(bd_flock, sf::Color::White, margin);
    } else {
      graph_boids_sp = create_animates(
          bd_flock, {boid_texture_normal, boid_texture_sped}, margin);
    }

    // predators graphical objects
    std::vector<Bird> graph_preds_tr;
    std::vector<Animate> graph_preds_sp;
    if (mode == false) {
      graph_preds_tr = create_birds(predators, sf::Color::Red, margin);
    } else {
      graph_preds_sp = create_animates(
          predators, {pred_texture_normal, pred_texture_sped}, margin);
    }

    // obstacles graphical objects
    std::vector<sf::CircleShape> graph_obs;
    std::transform(
        obstacles.begin(), obstacles.end(), std::back_inserter(graph_obs),
        [&margin, &obs_texture, &mode](Obstacle b) -> sf::CircleShape {
          sf::CircleShape ob_circ(b.get_size());
          switch (mode) {
            case false:
              ob_circ.setFillColor(sf::Color(210, 210, 210));
              break;
            case true:
              ob_circ.setTexture(&obs_texture);
              break;
            default:
              break;
          }
          ob_circ.setOrigin(ob_circ.getRadius(), ob_circ.getRadius());
          ob_circ.setPosition(b.get_pos()[0] + margin, b.get_pos()[1] + margin);
          return ob_circ;
        });

    // anti-alising settings
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    // window initialization
    sf::RenderWindow window(sf::VideoMode(static_cast<float>(window_x),
                                          static_cast<float>(window_y)),
                            "Boids simulation", sf::Style::Default, settings);
    if (mode == true) window.setTitle("STAR BOIDS");
    window.setFramerateLimit(60);
    // places window in top left corner, taking into account titlebar height
    window.setPosition(sf::Vector2i(0, window_x * 0.022));
    // framerate optimization (if supported)
    window.setVerticalSyncEnabled(true);

    // riquadro simulazione
    sf::RectangleShape rec_sim(sf::Vector2f(video_x, video_y));
    switch (mode) {
      case false:
        rec_sim.setFillColor(sf::Color::Blue);
        break;
      case true:
        rec_sim.setTexture(&backg);
        break;
      default:
        break;
    }
    rec_sim.setOutlineColor(palette[1]);
    rec_sim.setOutlineThickness(2);
    rec_sim.setPosition(margin, margin);

    // defines parameters for COM tracker
    float com_ratio = (window_x - 4 * margin - video_x) / video_x;

    float pos_com_x = bd_flock.get_com().get_pos()[0];
    float pos_com_y = bd_flock.get_com().get_pos()[1];
    float com_angle =
        static_cast<float>(compute_angle<double>(bd_flock.get_com().get_vel()));
    float tracker_x = window_x - video_x * com_ratio - 2 * margin;
    float tracker_y = margin;

    // initializes COM tracker
    Tracker com_tracker(std::valarray<float>{video_x, video_y},
                        {pos_com_x, pos_com_y}, com_ratio, margin / 2);
    com_tracker.setPosition(sf::Vector2f{tracker_x, tracker_y});
    com_tracker.setFillColors(sf::Color::White, sf::Color(210, 210, 210),
                              sf::Color::Black);
    com_tracker.setOutlineColors(sf::Color::Black, sf::Color::Black,
                                 sf::Color::Black);
    com_tracker.setOutlineThickness(2, 0, 0);
    com_tracker.update_angle(com_angle);

    // initializes and places text for time trackers
    sf::Text comp_text("Computation time: \nUpdate time: \nDraw time: ", font,
                       20);
    comp_text.setFillColor(palette[1]);
    comp_text.setPosition(video_x + 2 * margin,
                          video_y * com_ratio + 3 * margin);

    // mean speed status bar initialization
    StatusBar speed_bar("Mean distance (px): \nMean speed (px/s): ", font,
                        com_tracker.getOuter().getGlobalBounds().width, 20.,
                        {0., 350.});
    speed_bar.setColors(palette[1], palette[1]);
    speed_bar.setPosition(sf::Vector2f{
        video_x + 2 * margin,
        video_y * com_ratio + 3 * margin + 5.5 * comp_text.getCharacterSize()});
    // declares and initializes object for stats tracking
    Statistics flock_stats = bd_flock.get_stats();

    // text for user messages
    sf::Text message_text("Sim started", font, 20);
    if (mode == true) message_text.setString("Use the flock, Luke!");
    message_text.setFillColor(palette[0]);
    message_text.setOrigin(0, message_text.getLocalBounds().height);
    // background rectangle for messages
    sf::RectangleShape message_rect(
        sf::Vector2f(video_x * com_ratio + margin,
                     message_text.getGlobalBounds().height + margin));
    message_rect.setFillColor(palette[1]);
    message_rect.setOrigin(0, message_rect.getLocalBounds().height);
    message_rect.setPosition(video_x + 2 * margin, window_y - margin);
    message_text.setPosition(
        video_x + 2.5 * margin,
        window_y - margin - 0.4 * message_rect.getGlobalBounds().height);

    // text for user commands guide
    sf::Text commands_text("", font, 20);
    commands_text.setFillColor(palette[1]);
    commands_text.setString(
        "COMMANDS:\n"
        " > CTRL + B : generate boid\n"
        "    (hold for loop generation)\n"
        " > CTRL + P : generate predator\n"
        " > CTRL + O : toggle place obst.\n"
        "    (place with left click)\n"
        " > CTRL + A : pause / resume sim\n"
        " > esc : close program");
    commands_text.setOrigin(0, commands_text.getLocalBounds().height);
    commands_text.setPosition(message_rect.getPosition().x,
                              message_rect.getPosition().y -
                                  message_rect.getGlobalBounds().height -
                                  0.7 * commands_text.getCharacterSize());

    // -- TIME MEASUREMENTS --

    // initialization of init (time instant)
    auto init = std::chrono::steady_clock::now();
    // time steps initialization
    std::chrono::duration<double, std::milli> step_cmpt{
        std::chrono::duration<double, std::milli>::zero()};
    std::chrono::duration<double, std::milli> step_draw{
        std::chrono::duration<double, std::milli>::zero()};
    std::chrono::duration<double, std::milli> step_update{
        std::chrono::duration<double, std::milli>::zero()};

    // stringstream for duration to string conversion
    std::ostringstream values_ss;

    // counter for discrete interval operations
    int counter = 0;

    // boid loop generation boolean
    bool boid_gen = false;
    // obstacle placement boolean
    bool obstacle_gen = false;
    // simulation suspension boolean
    bool pause = false;

    // -- DATA OUTPUT --

    /*auto now = std::chrono::system_clock::now();
    auto present_time = std::chrono::system_clock::to_time_t(now);
    values_ss << "output/boids " << std::setw(18) <<
    std::ctime(&present_time)
            << ".txt";*/
    values_ss << "output/boids.txt";
    std::ofstream output_file{values_ss.str()};
    if (!output_file) {
      throw std::runtime_error("Cannot open output file");
    }
    values_ss.str("");
    output_file << "Mean distance + / - Mean d RMS  ||  Mean speed + / - "
                   "Mean s RMS\n"
                << "--------------------------------||---------------------"
                   "--------\n";

    // -- GAME LOOP --

    while (window.isOpen()) {
      // -- GRAPHIC OBJECTS UPDATE --

      // start 'cronometer'
      init = std::chrono::steady_clock::now();
      if (mode == false) {
        // update birds in classic mode
        update_birds(graph_boids_tr, bd_flock, margin);
        update_birds(graph_preds_tr, predators, margin);
      } else {
        // update animates in SW mode

        // update graphic boids number
        int diff = bd_flock.size() - static_cast<int>(graph_boids_sp.size());
        if (diff > 0) {
          for (int i = 0; i < diff; ++i) {
            Animate an_boid(static_cast<float>(bd_flock.get_params().d_s * 0.6 /
                                               boid_texture_normal.getSize().x),
                            {boid_texture_normal, boid_texture_sped});
            graph_boids_sp.push_back(an_boid);
          }
        } else if (diff < 0) {
          graph_boids_sp.erase(graph_boids_sp.begin() + bd_flock.size(),
                               graph_boids_sp.end());
        }
        // update graphic boids properties
        assert(graph_boids_sp.size() == bd_flock.size());
        auto bd_indx = bd_flock.begin();
        for (auto indx = graph_boids_sp.begin(); indx != graph_boids_sp.end();
             ++indx) {
          bd_indx = bd_flock.begin() + (indx - graph_boids_sp.begin());
          indx->setPosition(bd_indx->get_pos()[0] + margin,
                            bd_indx->get_pos()[1] + margin);
          indx->setRotation(180. - bd_indx->get_angle());
          (vec_norm<double>(bd_indx->get_vel()) > 150.) ? indx->setState(1)
                                                        : indx->setState(0);
        }

        // update graphic predators number
        for (int i = 0;
             i < static_cast<int>(predators.size() - graph_preds_sp.size());
             ++i) {
          Animate tr_predator(
              0.9 * predators[static_cast<unsigned int>(i)].get_par_ds() /
                  pred_texture_normal.getSize().x,
              {pred_texture_normal, pred_texture_sped});
          graph_preds_sp.push_back(tr_predator);
        }
        assert(graph_preds_sp.size() == predators.size());
        // update graphic predators properties
        for (int indx = 0; static_cast<unsigned int>(indx) < predators.size();
             ++indx) {
          graph_preds_sp[static_cast<unsigned int>(indx)].setPosition(
              predators[indx].get_pos()[0] + margin,
              predators[static_cast<unsigned int>(indx)].get_pos()[1] + margin);
          graph_preds_sp[static_cast<unsigned int>(indx)].setRotation(
              180. - predators[static_cast<unsigned int>(indx)].get_angle());
          (vec_norm<double>(
               predators[static_cast<unsigned int>(indx)].get_vel()) > 150.)
              ? graph_preds_sp[static_cast<unsigned int>(indx)].setState(1)
              : graph_preds_sp[static_cast<unsigned int>(indx)].setState(0);
        }
      }

      // update graphic obstacles
      if (graph_obs.size() != obstacles.size()) {
        graph_obs.resize(obstacles.size());
        assert(graph_obs.size() == obstacles.size());
        std::transform(obstacles.begin() + graph_obs.size(), obstacles.end(),
                       std::back_inserter(graph_obs),
                       [&margin, &obs_texture,
                        &mode](Obstacle const& b) -> sf::CircleShape {
                         sf::CircleShape ob_circ(b.get_size());
                         switch (mode) {
                           case false:
                             ob_circ.setFillColor(sf::Color(210, 210, 210));
                             break;
                           case true:
                             ob_circ.setTexture(&obs_texture);
                             break;
                           default:
                             break;
                         }
                         ob_circ.setOrigin(ob_circ.getRadius(),
                                           ob_circ.getRadius());
                         ob_circ.setPosition(b.get_pos()[0] + margin,
                                             b.get_pos()[1] + margin);
                         return ob_circ;
                       });
      }
      // update COM tracker at discrete rate
      if (counter % 4 == 0) {  // every four frames
        pos_com_x = bd_flock.get_com().get_pos()[0];
        pos_com_y = bd_flock.get_com().get_pos()[1];
        com_tracker.update_pos({pos_com_x, pos_com_y});
        com_angle = static_cast<float>(
            compute_angle<double>(bd_flock.get_com().get_vel()));
        com_tracker.update_angle(com_angle);

        // update stats
        bd_flock.update_stats();
        flock_stats = bd_flock.get_stats();
        // update status bar
        speed_bar.update_value(flock_stats.av_vel);
        values_ss.str("");
        // update mean distance & RMS
        values_ss << "Mean distance (px): " << std::setw(5)
                  << std::setprecision(1) << std::fixed << flock_stats.av_dist
                  << " +/- " << std::setprecision(1) << std::fixed
                  << flock_stats.dist_RMS << '\n';
        // update mean speed & RMS
        values_ss << "Mean speed (px/s): " << std::setw(5)
                  << std::setprecision(1) << std::fixed << flock_stats.av_vel
                  << " +/- " << std::setprecision(1) << std::fixed
                  << flock_stats.vel_RMS;
        speed_bar.set_text(values_ss.str());
      }
      if (counter % 60 == 0) {  // every second
        // print mean distance and speed with own RMSs on text file
        output_file << std::setw(13) << std::setprecision(3) << std::fixed
                    << flock_stats.av_dist << "       " << std::setw(10)
                    << std::setprecision(3) << std::fixed
                    << flock_stats.dist_RMS << "  ||  " << std::setw(10)
                    << std::setprecision(3) << std::fixed << flock_stats.av_vel
                    << "       " << std::setw(10) << std::setprecision(3)
                    << std::fixed << flock_stats.vel_RMS << '\n';
      }
      // stop 'cronometer'
      step_update += std::chrono::steady_clock::now() - init;

      // -- EVENT HANDLING --
      sf::Event event;
      while (window.pollEvent(event)) {
        switch (event.type) {
          // handle close button
          case sf::Event::Closed:
            window.close();
            break;
          // handle keyboard commands
          case sf::Event::KeyPressed:
            if (event.key.code == sf::Keyboard::Escape)
              window.close();
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
              // handle CTRL + key combinations
              if (sf::Keyboard::isKeyPressed(sf::Keyboard::B)) {
                // enactive boid generation, setting user message text
                if (boid_gen == false) {
                  boid_gen = true;
                  message_text.setString("Add boid");
                } else {
                  message_text.setString("Add boid (loop)");
                }
              } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::P)) {
                // add predator (no loop generation!)
                message_text.setString("Add predator");
                // generation time is taken into account for
                // global computation time
                init = std::chrono::steady_clock::now();
                add_predator(predators, obstacles, {video_x, video_y}, 150.,
                             30., 1., 70., 0.7);
                step_cmpt += std::chrono::steady_clock::now() - init;
              } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
                // pause / resume if obstacle insertion mode is off
                if (obstacle_gen == false) {
                  pause = !pause;
                  if (pause == true)
                    message_text.setString("Pause");
                  else
                    message_text.setString("");
                }
              } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::O)) {
                // enable obstacle insertion mode
                if (obstacle_gen == false) {
                  message_text.setString("Add obstacle");
                  obstacle_gen = true;
                  pause = true;
                } else {
                  obstacle_gen = false;
                  pause = false;
                  if (message_text.getString() == "Add obstacle" ||
                      message_text.getString() ==
                          "Impossible to add obstacle" ||
                      message_text.getString() == "Obstacle added")
                    message_text.setString("");
                }
              }
            }
            break;
          // handle key release
          case sf::Event::KeyReleased:
            if (event.key.code == sf::Keyboard::B ||
                event.key.code == sf::Keyboard::LControl) {
              // disable boid generation
              boid_gen = false;
              // clear message
              if (message_text.getString() == "Add boid" ||
                  message_text.getString() == "Add boid (loop)")
                message_text.setString("");
              // clears also predator message, comprehending
              // also the case of CTRL key release
              if (message_text.getString() == "Add predator")
                message_text.setString("");
            } else if (event.key.code == sf::Keyboard::P) {
              // clear predator message
              if (message_text.getString() == "Add predator")
                message_text.setString("");
            }
            break;
          // handle mouse left button pressed: obstacle insertion
          case sf::Event::MouseButtonPressed:
            if (event.mouseButton.button == sf::Mouse::Left && obstacle_gen) {
              // as usual, insertion time is taken into account
              init = std::chrono::steady_clock::now();
              if (add_obstacle(
                      obstacles,
                      {static_cast<double>(sf::Mouse::getPosition(window).x) -
                           static_cast<double>(margin),
                       static_cast<double>(sf::Mouse::getPosition(window).y) -
                           static_cast<double>(margin)},
                      20., {video_x, video_y})) {
                // set appropriate message
                message_text.setString("Obstacle added");
                step_cmpt += std::chrono::steady_clock::now() - init;
              } else {
                // set appropriate message
                message_text.setString("Impossible to add obstacle");
                step_cmpt += std::chrono::steady_clock::now() - init;
              }
            }
            break;
          // hande mouse left button release: clear message
          case sf::Event::MouseButtonReleased:
            if (event.mouseButton.button == sf::Mouse::Left && obstacle_gen) {
              if (message_text.getString() == "Obstacle added" ||
                  message_text.getString() == "Impossible to add obstacle")
                message_text.setString("Add obstacle");
            }
            break;
          // handle everything else
          default:
            break;
        }
      }
      // add boid if boid generation is active
      if (boid_gen) {
        // as usual, generation time is taken into account
        init = std::chrono::steady_clock::now();
        bd_flock.add_boid(obstacles);
        step_cmpt += std::chrono::steady_clock::now() - init;
      }

      // print computation, draw and update time
      if (counter % 4 == 0) {
        values_ss.str("");
        values_ss << "Computation time: " << std::setprecision(2) << std::fixed
                  << step_cmpt.count() / 4 << " ms \n";
        values_ss << "Update time: " << std::setprecision(2) << std::fixed
                  << step_update.count() / 4 << " ms \n";
        values_ss << "Draw time: " << std::setprecision(2) << std::fixed
                  << step_draw.count() / 4 << " ms";
        comp_text.setString(values_ss.str());
        step_cmpt = std::chrono::duration<double, std::milli>::zero();
        step_draw = std::chrono::duration<double, std::milli>::zero();
        step_update = std::chrono::duration<double, std::milli>::zero();
      }

      // -- DRAWING --

      // start draw time 'cronometer'
      init = std::chrono::steady_clock::now();
      // clear window
      window.clear(palette[0]);
      // draw simulation rectangle
      window.draw(rec_sim);
      if (mode == false) {
        // draw flock
        for (Bird& tr_boid : graph_boids_tr) {
          window.draw(tr_boid);
        }
        // draw predators
        for (Bird& tr_predator : graph_preds_tr) window.draw(tr_predator);
      } else {
        // draw flock
        for (Animate& sp_boid : graph_boids_sp) {
          window.draw(sp_boid);
        }
        // draw predators
        for (Animate& sp_predator : graph_preds_sp) window.draw(sp_predator);
      }
      // draw obstacles
      for (sf::CircleShape& obs : graph_obs) {
        window.draw(obs);
      }

      // draw COM tracker
      window.draw(com_tracker);
      // draw time trackers
      window.draw(comp_text);
      // draw mean speed bar + mean values (with RMSs)
      window.draw(speed_bar);
      // draw messages rectangle
      window.draw(message_rect);
      // draw messages text
      window.draw(message_text);
      // draw commands text
      window.draw(commands_text);
      // display
      window.display();
      // stop draw time 'cronometer'
      step_draw += std::chrono::steady_clock::now() - init;

      // start computation time 'cronometer'
      init = std::chrono::steady_clock::now();
      // update flock & predators state if not paused
      if (!pause)
        bd_flock.update_global_state(0.0166, behaviour, predators, obstacles);
      // stop computation time 'cronometer'
      step_cmpt += std::chrono::steady_clock::now() - init;

      // increment counter
      (counter == 1200) ? counter = 0 : ++counter;
    }

    return EXIT_SUCCESS;
  } catch (std::exception& e) {
    // handle standard exceptions (for SL functions)
    // print content to standard error output
    std::cerr << e.what();
  } catch (...) {
    // handle generic exception
    // print content to standard error output
    std::cerr << "Unknown exception";
  }
}