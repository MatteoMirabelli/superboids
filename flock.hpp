#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"
#include "obstacles.hpp"
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
  // costruttore con COM impostato
  Flock(Parameters const&, int const&, Boid const&, double const&,
        std::valarray<double> const&);

  // Costruttore senza COM
  Flock(Parameters const&, int const&, double const&,
        std::valarray<double> const&);
  // Costruttore senza COM con ostacoli
  Flock(Parameters const&, int, double, std::valarray<double> const&,
        std::vector<Obstacle> const&);
  Flock() = default;

  void add_boid();
  double size() const;
  std::vector<Boid>::iterator begin();
  std::vector<Boid>::iterator end();
  void push_back(Boid const&);
  Boid& get_boid(int);
  Boid const& get_boid(int) const;
  Boid const& get_com() const;
  Parameters const& get_params() const;
  void set_parameter(int const&, double const&);
  void set_space(double const&, double const&);
  void erase(std::vector<Boid>::iterator);
  void update_com();

  std::vector<Boid> get_neighbours(std::vector<Boid>::iterator);

  // vel_correction solo stormo (1)
  std::valarray<double> vel_correction(std::vector<Boid>::iterator);

  // vel_correction con un predatore (2)
  std::valarray<double> vel_correction(std::vector<Boid>::iterator,
                                       Predator const&);


  // vel_correction con più predatori, parametri per il comportamento ai bordi e
  // ostacoli (4)
  std::valarray<double> vel_correction(std::vector<Boid>::iterator it,
                                       std::vector<Predator> const& preds,
                                       std::vector<Obstacle> const& obs, double,
                                       double);

  // vel_correction (5)
  std::valarray<double> vel_correction(std::vector<Boid> const&,
                                       std::vector<Boid>::iterator);

  // vel_correction (6)
  std::valarray<double> vel_correction(std::vector<Boid> const&,
                                       std::vector<Boid>::iterator,
                                       Predator const&);

  // Vel correction dovuto al predatore
  std::valarray<double> avoid_pred(Boid const&, Predator const&,
                                   double boid_pred_detection,
                                   double boid_pred_repulsion);

  void update_flock_state(double const&, bool const&);
  void update_flock_pred_state(double delta_t, bool brd_bhv,
                               std::vector<Obstacle> obs, Predator& pred);

  // update_flock_state con un predatore e ostacoli
  void update_flock_pred_obs_state(double const& delta_t, bool const& brd_bhv,
                                   Predator& pred,
                                   std::vector<Obstacle> const& obs,
                                   double boid_pred_detection,
                                   double boid_pred_repulsion, double param_d,
                                   double repulsion_factor);

  void update_global_state(double const&, bool const&, Predator&);
  void update_global_state(double const&, bool const&, Predator&,
                           std::vector<Obstacle> const&);
  void update_global_state(double, bool, std::vector<Predator>&,
                           std::vector<Obstacle> const&);

  void sort();

  void update_stats();
  Statistics const& get_stats() const;
};

#endif