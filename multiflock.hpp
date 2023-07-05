#ifndef MULTIFLOCK_HPP
#define MULTIFLOCK_HPP

#include <vector>

#include "boid.hpp"
#include "flock.hpp"
#include "predator.hpp"

class Multiflock {
  std::vector<std::pair<Boid, int>> mf_flock;
  std::vector<Boid> mf_com;
  std::vector<Parameters> mf_params;
  std::vector<Statistics> mf_stats;

 public:
  Multiflock(std::vector<Parameters> const&, std::vector<int> const&,
             double const&, std::valarray<double> const&);
  /*Multiflock(int, Parameters const&, int, double, std::valarray<double> const&,
             std::vector<Obstacle> const&);*/
  Multiflock() = default;
  void add_boid(int);
  double size() const;
  std::vector<std::pair<Boid, int>>::iterator begin();
  std::vector<std::pair<Boid, int>>::iterator end();
  std::vector<Boid> const& get_coms() const;
  std::vector<Parameters> const& get_params() const;
  Parameters const& get_params(int) const;
  void set_parameter(int, int, double);
  void set_space(double const&, double const&);
  void update_coms();

  std::valarray<double> vel_correction(
      std::vector<std::pair<Boid, int>> const&,
      std::vector<std::pair<Boid, int>>::iterator);
  std::valarray<double> avoid_pred(std::pair<Boid, int> const&, Predator const&);

  void update_flock_state(double const&, bool const&);

  void update_global_state(double, bool, std::vector<Predator>&,
                           std::vector<Obstacle> const&);

  void sort();

  //void update_stats();
  std::vector<Statistics> const& get_stats() const;
};

#endif