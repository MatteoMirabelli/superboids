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
  explicit Flock(Parameters const&, int const&, Boid const&, double const&,
                 std::valarray<double> const&);
  Flock(Parameters const&, int const&, double const&,
        std::valarray<double> const&);
  Flock() = default;
  double size() const;
  std::vector<Boid>::iterator begin();
  std::vector<Boid>::iterator end();
  void push_back(Boid const&);
  Boid& get_boid(int);
  Boid const& get_boid(int) const;
  Boid const& get_com() const;
  Parameters const& get_params() const;
  void set_space(double const&, double const&);
  void erase(std::vector<Boid>::iterator);
  void update_com();

  std::vector<Boid> get_neighbours(std::vector<Boid>::iterator);
  std::vector<Boid> get_neighbours(double const&, Boid const&);

  std::valarray<double> vel_correction(std::vector<Boid>::iterator);
  std::valarray<double> vel_correction(std::vector<Boid>::iterator,
                                       Predator const&);

  void update_flock_state(double const&, bool const&);

  void update_flock_pred_state(double const&, bool const&, Predator&);

  void sort();

  void update_stats();
  Statistics const& get_stats() const;
};

#endif