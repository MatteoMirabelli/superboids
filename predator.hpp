#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <cassert>
#include <cmath>
#include <valarray>

#include "flock.hpp"

class Predator {
  std::valarray<double> p_pos;
  std::valarray<double> p_vel;
  double p_angle;
  double p_range;

 public:
  explicit Predator(std::valarray<double> pos, std::valarray<double> vel,
                    double range);
  Predator(double, double, double, double, double);
  Predator() = default;

  std::valarray<double>& get_pos();
  std::valarray<double> const& get_pos() const;

  std::valarray<double>& get_vel();
  std::valarray<double> const& get_vel() const;

  double& get_angle();
  double const& get_angle() const;

  std::vector<Boid> get_neighbours(double const&, Flock&), ;

  std::valarray<double> vel_correction(double const&);

  void update_state(double, std::valarray<double>);
  void update_state(double, std::valarray<double>, bool const&, double, double);

  bool is_visible(Boid const&, double view_angle);
};

#endif