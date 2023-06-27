#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include "boid.hpp"
#include <vector>

class Predator : public Boid {
  double p_range;
  double p_hunger;

 public:
  Predator(std::valarray<double> const&, std::valarray<double> const&,
           double const&, double const&);
  Predator(double const&, double const&, double const&, double const&,
           double const&, double const&);
  Predator() = default;
  double get_range() const;
  double get_hunger() const;
  std::valarray<double> predate(std::vector<Boid>const&);
};

#endif