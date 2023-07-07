#include "predator.hpp"

#include <algorithm>
#include <random>

pr::Predator::Predator(std::valarray<double> const& pos,
                   std::valarray<double> const& vel, double view_ang,
                   double param_d_s, double param_s,
                   std::valarray<double> const& space, double range,
                   double hunger)
    : bd::Boid(pos, vel, view_ang, space, param_d_s, param_s),
      p_range(range),
      p_hunger(hunger) {}

pr::Predator::Predator(double x, double y, double vx, double vy, double view_ang,
                   double param_d_s, double param_s, double sx, double sy,
                   double range, double hunger)
    : bd::Boid(x, y, vx, vy, view_ang, sx, sy, param_d_s, param_s),
      p_range(range),
      p_hunger(hunger) {}

double pr::Predator::get_angle() const { return bd::Boid::get_angle(); }

double pr::Predator::get_range() const { return p_range; }

double pr::Predator::get_hunger() const { return p_hunger; }

// it sorts preys according to x position and calculates the centre of mass of
// its preys
std::valarray<double> pr::Predator::predate(std::vector<bd::Boid>& preys) {
  std::valarray<double> prey_com_pos(2);
  if (preys.size() > 0) {
    auto nearest = [&](bd::Boid const& b1, bd::Boid const& b2) {
      return boid_dist(b1, *this) < boid_dist(b2, *this);
    };

    std::sort(preys.begin(), preys.end(), nearest);
    for (auto const& prey : preys) {
      prey_com_pos += prey.get_pos();
    }

    prey_com_pos /= static_cast<double>(preys.size());

    // It returns a vel correction proportional to the distance aqay from local
    // preys' centre of mass and proportional to its nearest prey
    return p_hunger * (prey_com_pos - get_pos()) +
           p_hunger * (static_cast<double>(preys.size())) *
               (preys[0].get_pos() - get_pos());
  } else {
    return std::valarray<double>{0., 0.};
  }
}

std::vector<pr::Predator> pr::random_predators(std::vector<ob::Obstacle> const& obs,
                                       int pred_num,
                                       std::valarray<double> const& pred_space,
                                       double pred_view_ang, double pred_ds,
                                       double pred_s, double pred_range,
                                       double pred_hunger) {
  std::vector<pr::Predator> predators;
  assert(pred_num >= 0 && pred_view_ang > 0. && pred_ds > 0. && pred_s > 0. &&
         pred_range > 0. && pred_hunger > 0.);
  if (pred_num == 0) return predators;
  std::random_device rd;
  int x_max = 2.5 * (pred_space[0] - 40.) / pred_ds;
  int y_max = 2.5 * (pred_space[1] - 40.) / pred_ds;

  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  auto generator = [&dist_pos_x, &dist_pos_y, &dist_vel_x, &dist_vel_y, &rd,
                    &pred_space, &pred_view_ang, &pred_ds, &pred_s, &pred_range,
                    &pred_hunger, &obs]() -> pr::Predator {
    // It generates the positons
    std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (pred_ds) + 20.,
                                 dist_pos_y(rd) * 0.4 * (pred_ds) + 20.};

    // Checks wheter there're are no oveerlapping obstacles
    auto overlap = [&pos, &pred_ds](ob::Obstacle const& obstacle) -> bool {
      std::valarray<double> dist = pos - obstacle.get_pos();
      return mt::vec_norm<double>(dist) < obstacle.get_size() + 0.6 * pred_ds;
    };

    while (std::any_of(obs.begin(), obs.end(), overlap)) {
      // As long as predators overlap with obstacle, it regenerates them
      pos = {dist_pos_x(rd) * 0.4 * (pred_ds) + 20.,
             dist_pos_y(rd) * 0.4 * (pred_ds) + 20.};
    }
    // If there are no predators overlapping with obstacles, it returns the
    // predator
    std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
    return pr::Predator{pos,    vel,        pred_view_ang, pred_ds,
                    pred_s, pred_space, pred_range,    pred_hunger};
  };

  // Generates predators
  std::generate_n(std::back_insert_iterator(predators), pred_num, generator);

  auto sort_pred = [](pr::Predator const& pred1, pr::Predator const& pred2) {
    if (pred1.get_pos()[0] == pred2.get_pos()[0]) {
      return pred1.get_pos()[1] < pred2.get_pos()[1];
    } else {
      return pred1.get_pos()[0] < pred2.get_pos()[0];
    }
  };

  std::sort(predators.begin(), predators.end(), sort_pred);

  // Checks if there are overlapping predator
  auto compare_pred = [&](pr::Predator const& b1, pr::Predator const& b2) {
    return boid_dist(b1, b2) < 0.3 * pred_ds;
  };

  auto last = std::unique(predators.begin(), predators.end(), compare_pred);

  // It regenerates predators until all of them don't overlap both with obstacle
  // and with one another
  while (last != predators.end()) {
    std::generate(last, predators.end(), generator);
    std::sort(predators.begin(), predators.end(), sort_pred);
    last = std::unique(predators.begin(), predators.end(), compare_pred);
  }
  return predators;
}

void pr::add_predator(std::vector<pr::Predator>& predators,
                  std::vector<ob::Obstacle> const& obstacles,
                  std::valarray<double> const& pred_space, double pred_ang,
                  double pred_ds, double pred_s, double pred_range,
                  double pred_hunger) {
  assert(pred_space[0] > 0 && pred_space[1] > 0 && pred_ang > 0. &&
         pred_ds > 0. && pred_s >= 0. && pred_range > 0. && pred_hunger > 0.);
  std::random_device rd;
  int x_max = static_cast<int>(2.5 * (pred_space[0] - 40.) / pred_ds);
  int y_max = static_cast<int>(2.5 * (pred_space[1] - 40.) / pred_ds);

  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (pred_ds) + 20.,
                               dist_pos_y(rd) * 0.4 * (pred_ds) + 20.};

  auto overlap_pred = [&pos, &pred_ds](pr::Predator& p1) -> bool {
    return mt::vec_norm<double>(static_cast<std::valarray<double>>(p1.get_pos() - pos)) <
           0.6 * pred_ds;
  };
  auto overlap_obs = [&pos, &pred_ds](ob::Obstacle const& obstacle) -> bool {
    std::valarray<double> dist = pos - obstacle.get_pos();
    return mt::vec_norm<double>(dist) < obstacle.get_size() + 0.6 * pred_ds;
  };

  // Until predator overlaps with obstacles or other predators, it regenerates
  while (std::any_of(predators.begin(), predators.end(), overlap_pred) ||
         std::any_of(obstacles.begin(), obstacles.end(), overlap_obs)) {
    pos = {dist_pos_x(rd) * 0.4 * (pred_ds) + 20.,
           dist_pos_y(rd) * 0.4 * (pred_ds) + 20.};
  }

  // Adds predator
  std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
  predators.push_back(pr::Predator{pos, vel, pred_ang, pred_ds, pred_s, pred_space,
                               pred_range, pred_hunger});
}

std::vector<pr::Predator> pr::get_vector_neighbours(
    std::vector<pr::Predator> const& full_vec, std::vector<pr::Predator>::iterator it,
    double dist) {
  std::vector<pr::Predator> neighbours;
  assert(it >= full_vec.begin() && it <= full_vec.end());

  // With the condition that the vector is sorted, it checks predators to the
  // right and to the left, untile the distance on x_axis becomes larger that
  // dist This way allows not to check the whole vector to find neighbours, but
  // just the closest ones

  if (it >= full_vec.begin() && it < full_vec.end()) {
    auto et = it;
    for (; et != full_vec.end(); ++et) {
      // If the distance on x axis is larger than dist, it breaks the loop
      if (std::abs(it->get_pos()[0] - et->get_pos()[0]) > dist) break;
      // If not, it checks wheter the predator can be considered a neighbour and
      // pushes it back into neighbours
      if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
          is_visible(*et, *it) == true) {
        neighbours.push_back(*et);
      }
    }
    et = it;
    for (; et != full_vec.begin(); --et) {
      // If the distance on x axis is larger than dist, it breaks the loop
      if (std::abs(it->get_pos()[0] - et->get_pos()[0]) > dist) break;

      // If not, it checks wheter the predator can be considered a neighbour and
      // pushes it back into neighbours
      if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
          is_visible(*et, *it) == true) {
        neighbours.push_back(*et);
      }
    }
    // Checks wheter .begin is a neighbour or not
    if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
        is_visible(*et, *it) == true) {
      neighbours.push_back(*et);
    }
  }
  return neighbours;
}

// Updates state of all predators
void pr::update_predators_state(std::vector<pr::Predator>& predators, double delta_t,
                            bool bhv,
                            std::vector<std::pair<bd::Boid, int>> const& preys,
                            std::vector<ob::Obstacle> const& obstacles) {
  std::vector<pr::Predator> copy_predators = predators;
  bool predation = (preys.size() > 0);
  for (auto idx = predators.begin(); idx != predators.end(); ++idx) {
    std::valarray<double> pred_separation = {0., 0.};

    // For each predator, it does:
    for (auto neighbour_pred : pr::get_vector_neighbours(
             copy_predators, copy_predators.begin() + (idx - predators.begin()),
             idx->get_par_ds())) {
      // apply separation from others
      pred_separation -=
          3 * idx->get_par_s() * (neighbour_pred.get_pos() - idx->get_pos());
    }
    // If at least one predator has at least one prey
    if (predation) {
      // Checks wheter a prey is its, and, in case, add to its "own_preys"
      std::vector<bd::Boid> own_preys;
      for (auto prey : preys)
        if (prey.second == idx - predators.begin())
          own_preys.push_back(prey.first);

      // Updates states of predator with vel correction due to obstacles, preys
      // and borders (param bhr tells wheter to apply periodic conditions or
      // border repulsion)
      idx->update_state(
          delta_t,
          pred_separation + idx->predate(own_preys) + idx->avoid_obs(obstacles),
          bhv);
    } else {
      // If no predator as a preys, it simply applies update state with obstacle
      // and neighbourss
      idx->update_state(delta_t, pred_separation + idx->avoid_obs(obstacles),
                        bhv);
    }
  }

  // Sorts predators
  auto sort_pred = [](pr::Predator const& pred1, pr::Predator const& pred2) {
    if (pred1.get_pos()[0] == pred2.get_pos()[0]) {
      return pred1.get_pos()[1] < pred2.get_pos()[1];
    } else {
      return pred1.get_pos()[0] < pred2.get_pos()[0];
    }
  };

  std::sort(predators.begin(), predators.end(), sort_pred);
}

// The same as previous function, but with parameters to be passed to
// avoid_obstacles and update_state; Used in tests and to find nice values
void pr::update_predators_state(
    std::vector<pr::Predator>& predators, double delta_t, bool bhv,
    std::vector<std::pair<bd::Boid, int>> const& preys,
    std::vector<ob::Obstacle> const& obstacles, double pred_pred_repulsion,
    double pred_obs_detection, double pred_obstacle_separation,
    double pred_brd_detection, double pred_brd_repulsion) {
  std::vector<pr::Predator> copy_predators = predators;
  bool predation = (preys.size() > 0);
  for (auto idx = predators.begin(); idx != predators.end(); ++idx) {
    std::valarray<double> pred_separation = {0., 0.};

    for (auto neighbour_pred : pr::get_vector_neighbours(
             copy_predators, copy_predators.begin() + (idx - predators.begin()),
             idx->get_par_ds())) {
      pred_separation -= pred_pred_repulsion * idx->get_par_s() *
                         (neighbour_pred.get_pos() - idx->get_pos());
    }

    if (predation) {
  
      std::vector<bd::Boid> own_preys;
      for (auto prey : preys) {
        if (prey.second == idx - predators.begin())
          own_preys.push_back(prey.first);
      }

      idx->update_state(delta_t,
                        pred_separation + idx->predate(own_preys) +
                            idx->avoid_obs(obstacles, pred_obs_detection,
                                           pred_obstacle_separation),
                        bhv, pred_brd_detection, pred_brd_repulsion);
    } else {
      idx->update_state(
          delta_t,
          pred_separation + idx->avoid_obs(obstacles, pred_obs_detection,
                                           pred_obstacle_separation),
          bhv, pred_brd_detection, pred_brd_repulsion);
    }
  }

  auto sort_pred = [](pr::Predator const& pred1, pr::Predator const& pred2) {
    if (pred1.get_pos()[0] == pred2.get_pos()[0]) {
      return pred1.get_pos()[1] < pred2.get_pos()[1];
    } else {
      return pred1.get_pos()[0] < pred2.get_pos()[0];
    }
  };

  std::sort(predators.begin(), predators.end(), sort_pred);
}
