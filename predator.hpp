#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <vector>

#include "boid.hpp"

class Predator : public Boid {
  double p_range;   // range delle prede
  double p_hunger;  // fattore di coesione verso com locale prede

 public:
  Predator(std::valarray<double> const&, std::valarray<double> const&,
           double const&, double const&, double const&,
           std::valarray<double> const&);
  Predator(double const&, double const&, double const&, double const&,
           double const&, double const&, double const&, double const&,
           double const&);
  Predator() = default;
  double get_angle() const;
  double get_range() const;
  double get_hunger() const;
  std::valarray<double> predate(std::vector<Boid>&);
};

#endif