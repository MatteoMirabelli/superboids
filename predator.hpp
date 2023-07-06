#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <vector>

#include "boid.hpp"

class Predator : public Boid {
  double p_range;   // range delle prede
  double p_hunger;  // fattore di coesione verso com locale prede

 public:
  Predator(std::valarray<double> const&, std::valarray<double> const&, double,
           double, double, std::valarray<double> const&, double, double);
  Predator(double, double, double, double, double, double, double, double,
           double, double, double);
  Predator() = default;
  double get_angle() const;
  double get_range() const;
  double get_hunger() const;
  std::valarray<double> predate(std::vector<Boid>&);
};

std::vector<Predator> random_predators(std::vector<Obstacle> const&, int,
                                       std::valarray<double> const&, double,
                                       double, double, double, double);
void update_predators_state(std::vector<Predator>&, double, bool,
                            std::vector<std::pair<Boid, int>> const&,
                            std::vector<Obstacle> const&);

// update_predators_state for tests
void update_predators_state(std::vector<Predator>&, double, bool,
                            std::vector<std::pair<Boid, int>> const&,
                            std::vector<Obstacle> const&,
                            double pred_pred_repulsion,
                            double pred_obs_detection,
                            double pred_obstacle_separation,
                            double pred_brd_detection,
                            double pred_brd_repulsion);

#endif