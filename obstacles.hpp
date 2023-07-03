#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <cassert>
#include <execution>
#include <valarray>

class Entity {
 protected:
  std::valarray<double> b_pos;

 public:
  virtual ~Entity() = 0;
  virtual char type() const = 0;
};

// classe ostacoli, dotata di posizione, dimensioni, dimensioni schermo (?)
class Obstacle : public Entity {
  double o_size;
 public:
  char type() const override;
  Obstacle(std::valarray<double> const&, double const&);
  Obstacle(double const&, double const&, double const&);
  Obstacle() = default;
  std::valarray<double> const& get_pos() const;
  double const& get_size() const;
};

// Genera il vettore di ostacoli
std::vector<Obstacle> generate_obstacles(int, double,
                                         std::valarray<double> const&);

// Ordina il vettore di ostacoli
void sort_obstacles(std::vector<Obstacle>&);

// Aggiungi un ostacolo
void add_obstacle(std::vector<Obstacle>&, std::valarray<double> pos,
                  double size);

#endif