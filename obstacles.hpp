#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <cassert>
#include <execution>

#include "math.hpp"

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
std::vector<Obstacle> generate_obstacles(int n, double size,
                                         std::valarray<double> const& space);

// Ordina il vettore di ostacoli
void sort_obstacles(std::vector<Obstacle>&);

// Aggiungi un ostacolo, ritornando un booleano per indicare
// se è stato o meno possibile compiere l'operazione
bool add_obstacle(std::vector<Obstacle>&, std::valarray<double> const&, double,
                  std::valarray<double> const&);

#endif