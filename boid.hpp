#ifndef BOID_HPP
#define BOID_HPP

#include <cassert>
#include <cmath>
#include <valarray>

class Boid {
  std::valarray<double> b_pos;
  std::valarray<double> b_vel;
  double b_angle;
  double b_view_angle;
  std::valarray<double> b_space;

 public:
  Boid(std::valarray<double>, std::valarray<double>, double,
       std::valarray<double>);
  Boid(double, double, double, double, double, double, double);
  Boid() = default;

  std::valarray<double>& get_pos();
  std::valarray<double> const& get_pos() const;

  std::valarray<double>& get_vel();
  std::valarray<double> const& get_vel() const;

  double& get_angle();
  double const& get_angle() const;

  double const& get_view_angle() const;

  void set_space(double const&, double const&);
  void set_space(std::valarray<double> const&);

  void update_state(double, std::valarray<double>);
  void update_state(double, std::valarray<double>, bool const&, double, double);
};

template <typename T>
T vec_norm(std::valarray<T> vec);

double boid_dist(Boid const& bd_1, Boid const& bd_2);

template <typename T>
T compute_angle(std::valarray<T> const&);

bool is_visible(Boid const&, Boid const&);

#endif