#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"
#include "predator.hpp"

struct Statistics {
  double av_dist;
  double dist_RMS;
  double av_vel;
  double vel_RMS;

  Statistics() = default;

  Statistics(double mean_dist, double rms_dist, double mean_vel,
             double rms_vel) {
    assert(mean_dist >= 0 && rms_dist >= 0 && mean_vel > 0 && rms_vel > 0);
    av_dist = mean_dist;
    dist_RMS = rms_dist;
    av_vel = mean_vel;
    vel_RMS = rms_vel;
  }
};

struct Parameters {
  double d;
  double d_s;
  double s;
  double a;
  double c;

  Parameters(double p_d, double p_ds, double p_s, double p_a, double p_c) {
    assert(p_d >= 0 && p_ds >= 0 && p_s >= 0 && p_a >= 0 && p_c >= 0);
    d = p_d;
    d_s = p_ds;
    s = p_s;
    a = p_a;
    c = p_c;
  }
};

class Flock {
  std::vector<Boid> f_flock;
  Boid f_com;
  Parameters f_params;
  Statistics f_stats;

 public:
  Flock(Parameters const&, int, Boid const&, double,
        std::valarray<double> const&);
  Flock(Parameters const&, int, double, std::valarray<double> const&);
  Flock(Parameters const&, int, double, std::valarray<double> const&,
        std::vector<Obstacle> const&);
  Flock() = default;
  void add_boid();
  double size() const;
  void push_back(Boid const& boid);
  std::vector<Boid>::iterator begin();
  std::vector<Boid>::iterator end();

  std::vector<Boid> const& get_flock() const;
  Boid const& get_boid(int) const;
  Boid const& get_com() const;
  Parameters const& get_params() const;
  void set_parameter(int, double);
  void set_space(double, double);
  void erase(std::vector<Boid>::iterator);
  void update_com();

  std::vector<Boid> get_neighbours(std::vector<Boid>::iterator);

  // Avoid_pred for tests
  std::valarray<double> avoid_pred(Boid const&, Predator const&, double,
                                   double);
  std::valarray<double> avoid_pred(Boid const&, Predator const&);

  // per i test:
  std::valarray<double> vel_correction(std::vector<Boid>::iterator);
  std::valarray<double> vel_correction(std::vector<Boid>::iterator it,
                                       std::vector<Predator> const& preds,
                                       double boid_pred_detection,
                                       double boid_pred_repulsion);

  // per aggiornare lo stato:
  std::valarray<double> vel_correction(std::vector<Boid> const&,
                                       std::vector<Boid>::iterator);

  // update_global_state for tests
  // Parameters in order: border_detection, border_repulsion,
  // boid_pred_detection, boid_pred_repulsion, boid_obs_detection,
  // boid_obs_repulsion, pred_pred_repulsion
  void update_global_state(double, bool, std::vector<Predator>&,
                           std::vector<Obstacle> const&,
                           double border_detection, double border_repulsion,
                           double boid_pred_detection,
                           double boid_pred_repulsion,
                           double boid_obs_detection, double boid_obs_repulsion,
                           double pred_pred_repulsion);
  void update_global_state(double, bool, std::vector<Predator>&,
                           std::vector<Obstacle> const&);

  void sort();

  void update_stats();
  Statistics const& get_stats() const;
};

#endif