#ifndef BOID_HPP
#define BOID_HPP

#include <cassert>

#include "math.hpp"
#include "obstacles.hpp"

namespace bd {
class Boid {
  std::valarray<double> b_pos;
  std::valarray<double> b_vel;
  double b_angle;
  double b_view_angle;
  std::valarray<double> b_space;
  double b_param_ds;
  double b_param_s;

 public:
  Boid(std::valarray<double>, std::valarray<double>, double,
       std::valarray<double>, double, double);
  Boid(double, double, double, double, double, double, double, double, double);
  Boid() = default;

  std::valarray<double>& get_pos();
  std::valarray<double> const& get_pos() const;

  std::valarray<double>& get_vel();
  std::valarray<double> const& get_vel() const;

  double get_angle() const;
  double get_view_angle() const;

  std::valarray<double> const& get_space() const;
  void set_space(double, double);
  void set_space(std::valarray<double> const&);

  double get_par_ds() const;
  double get_par_s() const;
  void set_par_ds(double);
  void set_par_s(double);

  // Avoid_obs for tests
  std::valarray<double> avoid_obs(std::vector<ob::Obstacle> const&, double,
                                  double) const;
  std::valarray<double> avoid_obs(std::vector<ob::Obstacle> const&) const;

  void update_state(double, std::valarray<double>);
  void update_state(double, std::valarray<double>, bool);
  // update_state for tests
  void update_state(double, std::valarray<double>, bool, double, double);
};

double boid_dist(Boid const& bd_1, Boid const& bd_2);

bool is_visible(Boid const&, Boid const&);
bool is_obs_visible(ob::Obstacle const& obs, Boid const& bd);

std::vector<Boid> get_vector_neighbours(std::vector<Boid> const&,
                                        std::vector<Boid>::iterator, double);
}  // namespace bd
#endif