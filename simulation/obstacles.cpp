#include "obstacles.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <random>

ob::Obstacle::Obstacle(std::valarray<double> const& pos, double size) {
  assert(size > 0 && pos.size() == 2);
  o_size = size;
  o_pos = pos;
}

ob::Obstacle::Obstacle(double pos_x, double pos_y, double size) {
  assert(pos_x >= 0 && pos_y >= 0 && size > 0);
  o_size = size;
  o_pos = {pos_x, pos_y};
}

std::valarray<double> const& ob::Obstacle::get_pos() const { return o_pos; }

double ob::Obstacle::get_size() const { return o_size; }

// VECTOR OF OBSTACLES

std::vector<ob::Obstacle> ob::generate_obstacles(int n_obstacles, double max_size,
                                         std::valarray<double> const& space) {
  // Generates randomly the positions of the obstacles
  assert(n_obstacles >= 0);
  std::vector<ob::Obstacle> g_obstacles;

  std::random_device rd;
  double x_max = (space[0] - 4.5 * max_size);
  double y_max = (space[1] - 4.5 * max_size);

  std::uniform_real_distribution<> dist_pos_x(4.5 * max_size, x_max);
  std::uniform_real_distribution<> dist_pos_y(4.5 * max_size, y_max);
  std::uniform_real_distribution<> size(15., max_size);

  for (auto n = 0; n < n_obstacles; ++n) {
    std::valarray<double> pos = {dist_pos_x(rd), dist_pos_y(rd)};
    g_obstacles.push_back(ob::Obstacle{pos, size(rd)});
  }

  ob::sort_obstacles(g_obstacles);

  // Checks if there are overlapping obstacles and removes overlapping ones

  auto overlap = [&](ob::Obstacle& obs1, ob::Obstacle& obs2) {
    return (mt::vec_norm<double>(obs1.get_pos() - obs2.get_pos()) <
            obs1.get_size() + obs2.get_size());
  };
  auto last = std::unique(g_obstacles.begin(), g_obstacles.end(), overlap);
  g_obstacles.erase(last, g_obstacles.end());

  // It regenerates osbtacles until there're no more overlapping oness
  while (g_obstacles.size() < static_cast<unsigned int>(n_obstacles)) {
    for (int i = 0; i < n_obstacles - static_cast<int>(g_obstacles.size());
         ++i) {
      std::valarray<double> pos = {dist_pos_x(rd), dist_pos_y(rd)};
      g_obstacles.push_back(ob::Obstacle{pos, size(rd)});
    }
    ob::sort_obstacles(g_obstacles);
    auto lt = std::unique(g_obstacles.begin(), g_obstacles.end(), overlap);
    g_obstacles.erase(lt, g_obstacles.end());
  }

  return g_obstacles;
}

bool ob::add_obstacle(std::vector<ob::Obstacle>& g_obstacles,
                  std::valarray<double> const& pos, double max_size,
                  std::valarray<double> const& space) {
  std::random_device rd;
  std::uniform_real_distribution<> ran_size(15., max_size);
  double size = ran_size(rd);
  if (pos[0] < size || pos[1] < size || (space[0] - pos[0]) < size ||
      (space[1] - pos[1]) < size || pos[0] > space[0] || pos[1] > space[1])
    return false;

  // Checks if it overlaps with another obstacle or with borders
  auto overlap = [&](ob::Obstacle const& obs) {
    return (mt::vec_norm<double>(obs.get_pos() - pos) < obs.get_size() + size);
  };

  // If it doesn't, it pushbacks obstacle in the vector and returns true, if it
  // does, it returns false
  if (std::none_of(g_obstacles.begin(), g_obstacles.end(), overlap)) {
    g_obstacles.push_back(ob::Obstacle(pos, size));
    return true;
  } else {
    return false;
  }
}

void ob::add_fixed_obstacle(std::vector<ob::Obstacle>& g_obstacles,
                        std::valarray<double> const& pos, double size,
                        std::valarray<double> const& space) {
  ob::sort_obstacles(g_obstacles);

  // Checks if it overlaps with another obstacle or with borders
  auto overlap = [&](ob::Obstacle& obs) {
    return (std::abs(pos[0] - obs.get_pos()[0]) < obs.get_size() + size ||
            std::abs((pos[1] - obs.get_pos()[1])) < obs.get_size() + size ||
            pos[0] < size || pos[1] < size ||
            std::abs(pos[0] - space[0]) < size ||
            std::abs(pos[1] - space[1]) < size);
  };

  // If it doesn't, it pushbacks obstacle in the vectorif it
  // does, it prints at screen "Impossible to add obstacles"
  auto it = std::find_if(g_obstacles.begin(), g_obstacles.end(), overlap);
  if (it == g_obstacles.end()) {
    g_obstacles.push_back(ob::Obstacle(pos, size));
    ob::sort_obstacles(g_obstacles);
  } else {
    std::cout << "Impossible to add this obstacle" << '\n';
  }
}

void ob::sort_obstacles(std::vector<ob::Obstacle>& g_obstacles) {
  auto is_less = [](ob::Obstacle const& obs1, ob::Obstacle const& obs2) {
    if (obs1.get_pos()[0] != obs2.get_pos()[0]) {
      return obs1.get_pos()[0] < obs2.get_pos()[0];
    } else {
      return obs1.get_pos()[1] < obs2.get_pos()[1];
    }
  };
  std::sort(std::execution::par, g_obstacles.begin(), g_obstacles.end(),
            is_less);
}
