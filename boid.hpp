#ifndef BOID_HPP
#define BOID_HPP

#include <cassert>
#include <cmath>
#include <valarray>

class Boid {
  std::valarray<double> b_pos;
  std::valarray<double> b_vel;
  double b_angle;

 public:
  explicit Boid(std::valarray<double>, std::valarray<double>);
  Boid(double, double, double, double);
  Boid() = default;

  std::valarray<double>& get_pos();
  std::valarray<double> const& get_pos() const;

  std::valarray<double>& get_vel();
  std::valarray<double> const& get_vel() const;

  double& get_angle();
  double const& get_angle() const;

  void update_state(double, std::valarray<double>);
  void update_state(double, std::valarray<double>, bool const&, double, double);
};

template <typename T>
T vec_norm(std::valarray<T>);

double boid_dist(Boid const&, Boid const&);

template <typename T>
T compute_angle(std::valarray<T> const&);

bool is_visible(Boid const&, Boid const&, double const&);

#endif