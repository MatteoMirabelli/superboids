#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"
class Flock {
  double f_d;
  std::vector<Boid> f_flock;
  Boid f_com;
  std::valarray<double> f_params;

 public:
  explicit Flock(double const&, std::valarray<double> const&, int const&,
                 Boid const&);
  Flock() = default;
  double size() const;
  std::vector<Boid>::iterator begin();
  std::vector<Boid>::iterator end();
  void push_back(Boid const&);
  Boid& get_boid(int);
  Boid const& get_boid(int) const;
  std::valarray<double> get_params() const;
  void erase(int n);
  void update_com();

  std::vector<Boid> get_neighbours(std::vector<Boid>::iterator);

  std::valarray<double> vel_correction(std::vector<Boid>::iterator);

  void update_flock_state(double const&);
};

#endif
