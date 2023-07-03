#ifndef MULTIFLOCK_HPP
#define MULTIFLOCK_HPP

#include <vector>

#include "boid.hpp"
#include "flock.hpp"
#include "predator.hpp"

class Multiflock {
  std::vector<std::pair<Boid, unsigned int> > mf_flock;
  std::vector<Boid> mf_com;
  std::vector<Parameters> mf_params;
  std::vector<Statistics> mf_stats;
};

#endif