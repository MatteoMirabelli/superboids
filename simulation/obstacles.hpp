#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <cassert>
#include <execution>

#include "math.hpp"
namespace ob {
class Obstacle {
  std::valarray<double> o_pos;
  double o_size;

 public:
  Obstacle(std::valarray<double> const&, double);
  Obstacle(double, double, double);
  Obstacle() = default;
  std::valarray<double> const& get_pos() const;
  double get_size() const;
};

// Generate a random vector of obstacles
std::vector<Obstacle> generate_obstacles(int, double,
                                         std::valarray<double> const&);

// It sorts the vector in ascending order according to x_position. If x_positons
// are the same, it sorts according to y_positions
void sort_obstacles(std::vector<Obstacle>&);

// Adds an obstacle with random size, and returns true in case it can add it,
// returns false in case it can't
bool add_obstacle(std::vector<Obstacle>&, std::valarray<double> const&, double,
                  std::valarray<double> const&);

// Add_obstacle WITHOUT random size, used in tests
void add_fixed_obstacle(std::vector<Obstacle>& g_obstacles,
                        std::valarray<double> const& pos, double size,
                        std::valarray<double> const& space);
}  // namespace ob
#endif