#include "obstacles.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <valarray>

// OBSTACLE

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

std::valarray<double> const& Obstacle::get_pos() { return o_pos; };

double const& Obstacle::get_size() { return o_size; };

// VECTOR OF OBSTACLES

std::vector<Obstacle> generate_obstacles(int n_obstacles, double size,
                                         std::valarray<double> const& space) {
  // Genera casualmente, secondo distribuzioni uniformi, gli ostacoli
  assert(n_obstacles >= 0);
  std::vector<Obstacle> g_obstacles;

  std::random_device rd;
  double x_max = (space[0] - 40.);
  double y_max = (space[1] - 40.);

  std::uniform_real_distribution<> dist_pos_x(0, x_max);
  std::uniform_real_distribution<> dist_pos_y(0, y_max);

  for (auto n = 0; n < n_obstacles; ++n) {
    std::valarray<double> pos = {dist_pos_x(rd), dist_pos_y(rd)};
    g_obstacles.push_back(Obstacle{pos, size});
  }

  sort_obstacles(g_obstacles);

  auto overlap = [&](Obstacle& obs1, Obstacle& obs2) {
    return (std::abs(obs1.get_pos()[0] - obs2.get_pos()[0]) < 2 * size &&
            std::abs((obs1.get_pos()[1] - obs2.get_pos()[1]) < 2 * size));
  };

  // verifica la prima volta che non ci siano ostacoli coincidenti
  auto last = std::unique(g_obstacles.begin(), g_obstacles.end(), overlap);
  g_obstacles.erase(last, g_obstacles.end());

  // se ce n'erano, rigenera e ripulisce fino a quando non
  while (g_obstacles.size() < n_obstacles) {
    for (int i = 0; i < g_obstacles.size() - n_obstacles; ++i) {
      std::valarray<double> pos = {dist_pos_x(rd), dist_pos_y(rd)};
      g_obstacles.push_back(Obstacle{pos, size});
    }
    sort_obstacles(g_obstacles);
    auto lt = std::unique(g_obstacles.begin(), g_obstacles.end(), overlap);
    g_obstacles.erase(lt, g_obstacles.end());
  }
}

void add_obstacle(std::vector<Obstacle>& g_obstacles, std::valarray<double> pos,
                  double size) {
  auto overlap = [&](Obstacle& obs) {
    return (std::abs(pos[0] - obs.get_pos()[0]) < 2 * size &&
            std::abs((pos[1] - obs.get_pos()[1]) < 2 * size));
  };

  auto it = std::find_if(g_obstacles.begin(), g_obstacles.end(), overlap);
  if (it == g_obstacles.end()) {
    g_obstacles.push_back(Obstacle(pos, size));
    sort_obstacles(g_obstacles);
  } else {
    std::cout << "Impossibile aggiungere l'ostacolo" << '\n';
  }
}

void sort_obstacles(std::vector<Obstacle>& g_obstacles) {
  auto is_less = [](Obstacle& obs1, Obstacle& obs2) {
    if (obs1.get_pos()[0] != obs2.get_pos()[0]) {
      return obs1.get_pos()[0] < obs2.get_pos()[0];
    } else {
      return obs1.get_pos()[1] < obs2.get_pos()[1];
    }
  };

  std::sort(g_obstacles.begin(), g_obstacles.end(), is_less);
};
