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
  explicit Boid(std::valarray<double> pos, std::valarray<double> vel);
  Boid() = default;

  std::valarray<double>& get_pos();
  std::valarray<double> const& get_pos() const;

  std::valarray<double>& get_vel();
  std::valarray<double> const& get_vel() const;

  double& get_angle();
  double const& get_angle() const;

  void update_state(double delta_t, std::valarray<double> vel);
};

template <typename T>
T vec_norm(std::valarray<T> vec);

double boid_dist(Boid const& bd_1, Boid const& bd_2);

template <typename T>
T compute_angle(std::valarray<T> const&);

bool is_visible(Boid const&, Boid const&, double const&);

#endif