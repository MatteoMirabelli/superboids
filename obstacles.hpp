#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <cassert>
#include <valarray>

// classe ostacoli, dotata di posizione, dimensioni, dimensioni schermo (?)
class Obstacle {
  std::valarray<double> o_pos;
  double o_size;

 public:
  Obstacle(std::valarray<double> const&, double const&);
  Obstacle(double const&, double const&, double const&);
  std::valarray<double> const& get_pos();
  double const& get_size();
};

// Flock di ostacoli

std::vector<Obstacle> generate_obstacles(int n, double size);
void add_obstacle(std::vector<Obstacle>&, std::valarray<double> pos,
                  double size);

class Obstacle_set {
  std::vector<Obstacle> g_obstacles;

 public:
  // costruttori
  // Obstacle_set(int, double, std::valarray<double> o_pos);

  Obstacle& get_obstacle(std::vector<Obstacle>::iterator);
  Obstacle const& get_obstacle(std::vector<Obstacle>::iterator) const;

  void push_back(Obstacle const&);

  void erase(std::vector<Obstacle>::iterator);
  double size();

  void sort();
};

#endif