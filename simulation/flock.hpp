#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"
#include "predator.hpp"

namespace fk {

struct Statistics {
  double av_dist;
  double dist_RMS;
  double av_vel;
  double vel_RMS;

  Statistics() = default;

  Statistics(double, double, double, double);
};

struct Parameters {
  double d;
  double d_s;
  double s;
  double a;
  double c;

  Parameters(double, double, double, double, double);
};

class Flock {
  std::vector<bd::Boid> f_flock;
  bd::Boid f_com;
  Parameters f_params;
  Statistics f_stats;

 public:
  Flock(Parameters const&, int, bd::Boid const&, double,
        std::valarray<double> const&);
  Flock(Parameters const&, int, double, std::valarray<double> const&);
  Flock(Parameters const&, int, double, std::valarray<double> const&,
        std::vector<ob::Obstacle> const&);
  Flock() = default;
  void add_boid();
  void add_boid(std::vector<ob::Obstacle> const&);
  int size() const;
  void push_back(bd::Boid const& boid);
  std::vector<bd::Boid>::iterator begin();
  std::vector<bd::Boid>::iterator end();

  std::vector<bd::Boid> const& get_flock() const;
  bd::Boid const& get_boid(int) const;
  bd::Boid const& get_com() const;
  Parameters const& get_params() const;
  void set_parameter(int, double);
  void set_space(double, double);
  void erase(std::vector<bd::Boid>::iterator);
  void update_com();

  std::vector<bd::Boid> get_neighbours(std::vector<bd::Boid>::iterator) const;

  // Avoid_pred for tests
  std::valarray<double> avoid_pred(bd::Boid const&, pr::Predator const&, double,
                                   double) const;
  std::valarray<double> avoid_pred(bd::Boid const&, pr::Predator const&) const;

  // Vel_correction for tests
  std::valarray<double> vel_correction(std::vector<bd::Boid>::iterator);
  std::valarray<double> vel_correction(std::vector<bd::Boid>::iterator it,
                                       std::vector<pr::Predator> const& preds,
                                       double boid_pred_detection,
                                       double boid_pred_repulsion);
  // per aggiornare lo stato:
  std::valarray<double> vel_correction(std::vector<bd::Boid> const&,
                                       std::vector<bd::Boid>::iterator);

  // update_global_state for tests
  // Parameters in order: border_detection, border_repulsion,
  // boid_pred_detection, boid_pred_repulsion, boid_obs_detection,
  // boid_obs_repulsion, pred_pred_repulsion
  void update_global_state(double, bool, std::vector<pr::Predator>&,
                           std::vector<ob::Obstacle> const&,
                           double border_detection, double border_repulsion,
                           double boid_pred_detection,
                           double boid_pred_repulsion,
                           double boid_obs_detection, double boid_obs_repulsion,
                           double pred_pred_repulsion);

  void update_global_state(double, bool, std::vector<pr::Predator>&,
                           std::vector<ob::Obstacle> const&);

  void sort();

  void update_stats();
  Statistics const& get_stats() const;
};
}  // namespace fk

#endif