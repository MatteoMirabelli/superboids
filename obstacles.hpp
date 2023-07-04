#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <cassert>
#include <execution>
#include <valarray>

// classe ostacoli, dotata di posizione, dimensioni, dimensioni schermo (?)
class Obstacle {
  std::valarray<double> o_pos;
  double o_size;

 public:
  Obstacle(std::valarray<double> const&, double const&);
  Obstacle(double const&, double const&, double const&);
  std::valarray<double> const& get_pos() const;
  double const& get_size() const;
};

// Genera il vettore di ostacoli
std::vector<Obstacle> generate_obstacles(int n, double size, std::valarray<double> const& space);

// Ordina il vettore di ostacoli
void sort_obstacles(std::vector<Obstacle>&);

// Aggiungi un ostacolo
void add_obstacle(std::vector<Obstacle>& g_obstacles, std::valarray<double> const& pos,
                  double size, std::valarray<double> const& space);

#endif