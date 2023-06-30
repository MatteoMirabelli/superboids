#include "obstacles.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <random>
#include <valarray>

Obstacle::Obstacle(std::valarray<double> const& pos, double const& size) {
  assert(size > 0 && pos.size() == 2);
  o_size = size;
  o_pos = pos;
}

Obstacle::Obstacle(double const& pos_x, double const& pos_y,
                   double const& size) {
  assert(pos_x >= 0 && pos_y >= 0 && size > 0);
  o_size = size;
  o_pos = {pos_x, pos_y};
}

std::vector<Obstacle> generate_obstacles(int n, double size) {}

void add_obstacle(std::vector<Obstacle>, std::valarray<double> pos,
                  double size) {}

std::valarray<double> const& Obstacle::get_pos() { return o_pos; };
double const& Obstacle::get_size() { return o_size; };

// Metodi del FLock di ostacoli

// Genera n_ostacoli di dimensione size casualmente, controllando di non
// sovrapporli...
// Ho usato cose strane, non so se funziona, guardateci benes

/* Obstacle_set::Obstacle_set(int n_obstacles, double size,
                           std::valarray<double> space)
    : g_obstacles{} {
  std::random_device rd;
  std::uniform_real_distribution<> dist_pos_x(20., space[0] - 20.);
  std::uniform_real_distribution<> dist_pos_y(20., space[1] - 20.);

  auto overlap = [&](std::vector<Obstacle>::iterator it, double const& x,
                     double const& y) {
    return (
        std::abs((x - size) - (it->get_pos()[0] - it->get_size())) < 2 * size &&
        std::abs((y - size) - (it->get_pos()[1] - it->get_size())) < 2 * size);
  };

  for (int i = 0; i < n_obstacles; ++i) {

    for (int j = 0;; ++j) {
      std::valarray<double> pos;
      double pos_x = dist_pos_x(rd);
      double pos_y = dist_pos_y(rd);

      auto et = std::find_if(
          g_obstacles.begin(), g_obstacles.end(),
          overlap(std::next(g_obstacles.begin(), j), pos_x, pos_y));
      if (et == g_obstacles.end()) {
        pos = {pos_x, pos_y};
        Obstacle obs(pos, size);
        g_obstacles.push_back(obs);
        break;
      }
    }
  }
} */

// Restituiscono un ostacolo specifico, una versione solo in lettura,
// l'altra anche in modifica

Obstacle& Obstacle_set::get_obstacle(std::vector<Obstacle>::iterator it) {
  return *it;
}
Obstacle const& Obstacle_set::get_obstacle(
    std::vector<Obstacle>::iterator it) const {
  return *it;
}

// Inserire un ostacolo

void Obstacle_set::push_back(Obstacle const& obs) {
  g_obstacles.push_back(obs);
}

// Eliminare un ostacolo

void Obstacle_set::erase(std::vector<Obstacle>::iterator it) {
  assert(it != g_obstacles.end());
  g_obstacles.erase(it);
}

double Obstacle_set::size() { return g_obstacles.size(); };

// Ordinare gli ostacoli sulle x, magari può aiutare a velocizzare il
// programma

void Obstacle_set::sort() {
  auto is_less = [](Obstacle& obs1, Obstacle& obs2) {
    if (obs1.get_pos()[0] != obs2.get_pos()[0]) {
      return obs1.get_pos()[0] < obs2.get_pos()[0];
    } else {
      return obs1.get_pos()[1] < obs2.get_pos()[1];
    }
  };

  std::sort(g_obstacles.begin(), g_obstacles.end(), is_less);
}