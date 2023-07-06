#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <cassert>
#include <execution>
#include <valarray>

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

// Genera il vettore di ostacoli
std::vector<Obstacle> generate_obstacles(int n_obstacles, double max_size,
                                         std::valarray<double> const& space);

// Ordina il vettore di ostacoli
void sort_obstacles(std::vector<Obstacle>&);

// Aggiungi un ostacolo
void add_obstacle(std::vector<Obstacle>& g_obstacles,
                  std::valarray<double> const& pos, double size,
                  std::valarray<double> const& space);

#endif