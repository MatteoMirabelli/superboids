#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include <vector>

#include "boid.hpp"

namespace pr {
class Predator : public bd::Boid {
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

  // It returns the vel_correction that must be applied to a predator due to its
  // preys
  std::valarray<double> predate(std::vector<bd::Boid>&);
};

// Generates random predators with determined view_angle, param_ds, param_s,
// range and hunger checking they don't overlap with obstacles
std::vector<Predator> random_predators(std::vector<ob::Obstacle> const&, int,
                                       std::valarray<double> const&, double,
                                       double, double, double, double);

// It add a new predator to the simulation
void add_predator(std::vector<Predator>&, std::vector<ob::Obstacle> const&,
                  std::valarray<double> const&, double, double, double, double,
                  double);

// It returns the neighbours of a predator
std::vector<Predator> get_vector_neighbours(std::vector<Predator> const&,
                                            std::vector<Predator>::iterator,
                                            double);

// Updates the state of all predators
void update_predators_state(std::vector<Predator>&, double, bool,
                            std::vector<std::pair<bd::Boid, int>> const&,
                            std::vector<ob::Obstacle> const&);

// update_predators_state for tests
void update_predators_state(std::vector<Predator>&, double, bool,
                            std::vector<std::pair<bd::Boid, int>> const&,
                            std::vector<ob::Obstacle> const&,
                            double pred_pred_repulsion,
                            double pred_obs_detection,
                            double pred_obstacle_separation,
                            double pred_brd_detection,
                            double pred_brd_repulsion);
}  // namespace pr
#endif