#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"

struct Statistics {
  double av_dist;
  double dist_RMS;
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

 public:
  explicit Flock(Parameters const&, int const&, Boid const&);
  Flock(Parameters const& params, int const& bd_n);
  Flock() = default;
  double size() const;
  std::vector<Boid>::iterator begin();
  std::vector<Boid>::iterator end();
  void push_back(Boid const&);
  Boid& get_boid(int);
  Boid const& get_boid(int) const;
  Boid const& get_com() const;
  Parameters const& get_params() const;
  void erase(int n);
  void update_com();

  std::vector<Boid> get_neighbours(std::vector<Boid>::iterator it);

  std::valarray<double> vel_correction(std::vector<Boid>::iterator);

  void update_flock_state(double const&);

  void sort();
};

#endif