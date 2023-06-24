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
  explicit Boid(std::valarray<double> const&, std::valarray<double> const&);
  Boid(double const&, double const&, double const&, double const&);
  Boid() = default;

  std::valarray<double>& get_pos();
  std::valarray<double> const& get_pos() const;

  std::valarray<double>& get_vel();
  std::valarray<double> const& get_vel() const;

  double& get_angle();
  double const& get_angle() const;

  void update_state(double const&, std::valarray<double> const&);
  void update_state(double const&, std::valarray<double> const&, bool const&,
                    double const&, double const&);
};

template <typename T>
T vec_norm(std::valarray<T> const&);

double boid_dist(Boid const&, Boid const&);

template <typename T>
T compute_angle(std::valarray<T> const&);

bool is_visible(Boid const&, Boid const&, double const&);

#endif