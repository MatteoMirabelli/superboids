#include "flock.hpp"

#include <algorithm>
#include <cassert>
#include <execution>
#include <mutex>
#include <numeric>
#include <random>
#include <utility>

// Statistics constructor
fk::Statistics::Statistics(double mean_dist, double rms_dist, double mean_vel,
                           double rms_vel) {
  assert(mean_dist >= 0 && rms_dist >= 0 && mean_vel > 0 && rms_vel > 0);
  av_dist = mean_dist;
  dist_RMS = rms_dist;
  av_vel = mean_vel;
  vel_RMS = rms_vel;
}

// Parameters constructor
fk::Parameters::Parameters(double p_d, double p_ds, double p_s, double p_a,
                           double p_c) {
  assert(p_d >= 0 && p_ds >= 0 && p_s >= 0 && p_a >= 0 && p_c >= 0);
  d = p_d;
  d_s = p_ds;
  s = p_s;
  a = p_a;
  c = p_c;
}

// Flock constructor with centre_of_mass... no more used in the simulation, but
// used in many tests!
fk::Flock::Flock(fk::Parameters const& params, int bd_n, bd::Boid const& com,
                 double view_ang, std::valarray<double> const& space)
    : f_com{com}, f_flock{}, f_params{params}, f_stats{} {
  // Generates randomly boids around centre of masss
  assert(bd_n >= 0);

  if (bd_n == 0) {
    // f_flock = std::vector<bd::Boid>(0);
  } else {
    double rg_x;
    double rg_y;
    (com.get_pos()[0] < space[0] / 2.)
        ? rg_x = com.get_pos()[0] - 20.
        : rg_x = space[0] - com.get_pos()[0] - 20.;
    (com.get_pos()[1] < space[1] / 2.)
        ? rg_y = com.get_pos()[1] - 20.
        : rg_y = space[1] - com.get_pos()[1] - 20.;
    std::random_device rd;
    std::uniform_real_distribution<> dist_pos_x(com.get_pos()[0] - rg_x,
                                                com.get_pos()[0] + rg_x + 0.1);
    std::uniform_real_distribution<> dist_vel_x(com.get_vel()[0] - 150.,
                                                com.get_vel()[0] + 150.1);
    std::uniform_real_distribution<> dist_pos_y(com.get_pos()[1] - rg_y,
                                                com.get_pos()[1] + rg_y + 0.1);
    std::uniform_real_distribution<> dist_vel_y(com.get_vel()[1] - 150.,
                                                com.get_vel()[1] + 150.1);
    std::valarray<double> final_pos{0., 0.};
    std::valarray<double> final_vel{0., 0.};
    for (auto n = 0; n < bd_n - 1; ++n) {
      f_flock.push_back(bd::Boid{{dist_pos_x(rd), dist_pos_y(rd)},
                                 {dist_vel_x(rd), dist_vel_y(rd)},
                                 view_ang,
                                 space,
                                 params.d_s,
                                 params.s});
      final_pos += f_flock[n].get_pos();
      final_vel += f_flock[n].get_vel();
    }
    f_flock.push_back(bd::Boid{bd_n * com.get_pos() - final_pos,
                               bd_n * com.get_vel() - final_vel, view_ang,
                               space, params.d_s, params.s});
  }

  sort();
}

// Flock constructor without obstacles
fk::Flock::Flock(fk::Parameters const& params, int bd_n, double view_ang,
                 std::valarray<double> const& space)
    : f_flock{}, f_params{params}, f_stats{} {
  // Generates randomly boids in the simulation area (space)
  assert(bd_n >= 0);
  std::random_device rd;
  int x_max = static_cast<int>(2.5 * (space[0] - 40.) / params.d_s);
  int y_max = static_cast<int>(2.5 * (space[1] - 40.) / params.d_s);

  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  f_com = bd::Boid{{0., 0.}, {0., 0.}, 0., space, params.d_s, params.s};

  auto generator = [&]() -> bd::Boid {
    std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (params.d_s) + 20.,
                                 dist_pos_y(rd) * 0.4 * (params.d_s) + 20.};
    std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
    return bd::Boid{pos, vel, view_ang, space, params.d_s, params.s};
  };

  std::generate_n(std::execution::par, std::back_insert_iterator(f_flock), bd_n,
                  generator);

  sort();

  auto compare_bd = [&](bd::Boid& b1, bd::Boid& b2) {
    return bd::boid_dist(b1, b2) < 0.3 * params.d_s;
  };

  // Checks wheter or not there are overlapping boids
  auto last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                          compare_bd);
  // f_flock.erase(last, f_flock.end());

  // If there are, it regeneates them
  while (last != f_flock.end()) {
    std::generate(std::execution::par, last, f_flock.end(), generator);
    sort();
    last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                       compare_bd);
  }

  update_com();
}

// fk::Flock constructor with obstacles
fk::Flock::Flock(fk::Parameters const& params, int bd_n, double view_ang,
                 std::valarray<double> const& space,
                 std::vector<ob::Obstacle> const& obs)
    : f_flock{}, f_params{params}, f_stats{} {
  // Generates randomly boids in the suitable simulation area
  assert(bd_n > 0);
  f_com = bd::Boid{{0., 0.}, {0., 0.}, view_ang, space, params.d_s, params.s};
  if (bd_n > 0) {
    std::random_device rd;
    int x_max = 2.5 * (space[0] - 40.) / params.d_s;
    int y_max = 2.5 * (space[1] - 40.) / params.d_s;

    std::uniform_int_distribution<> dist_pos_x(0, x_max);
    std::uniform_int_distribution<> dist_pos_y(0, y_max);

    std::uniform_real_distribution<> dist_vel_x(-150., 150.);
    std::uniform_real_distribution<> dist_vel_y(-150., 150.);

    auto generator = [&dist_pos_x, &dist_pos_y, &dist_vel_x, &dist_vel_y, &rd,
                      &params, &space, &view_ang, &obs]() -> bd::Boid {
      // Generates position
      std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (params.d_s) + 20.,
                                   dist_pos_y(rd) * 0.4 * (params.d_s) + 20.};
      // Checks wheter it overlaps or not with obstacles
      auto overlap = [&pos, &params](ob::Obstacle const& obstacle) -> bool {
        std::valarray<double> dist = pos - obstacle.get_pos();
        return mt::vec_norm<double>(dist) <
               obstacle.get_size() + 0.6 * params.d_s;
      };

      while (std::any_of(obs.begin(), obs.end(), overlap)) {
        // As long as it overlaps with obstacles, it regenerates
        pos = {dist_pos_x(rd) * 0.4 * (params.d_s) + 20.,
               dist_pos_y(rd) * 0.4 * (params.d_s) + 20.};
      }

      // If it doesn't overlap, it returns a boidd with random position and
      // speed
      std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
      return bd::Boid{pos, vel, view_ang, space, params.d_s, params.s};
    };

    // Generates flock
    std::generate_n(std::execution::par, std::back_insert_iterator(f_flock),
                    bd_n, generator);

    // It sorts it
    sort();

    // Checks wheter two boids overlap
    auto compare_bd = [&](bd::Boid& b1, bd::Boid& b2) {
      return b1.get_pos()[0] == b2.get_pos()[0] &&
             b1.get_pos()[1] == b2.get_pos()[1];
    };

    auto last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                            compare_bd);

    // Until there are overlapping boids, it regenerates checking they don't
    // overlap with obstacless
    while (last != f_flock.end()) {
      std::generate(std::execution::par, last, f_flock.end(), generator);
      sort();
      last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                         compare_bd);
    }
    sort();
    update_com();
  } else {
  }
}

// Add_boid in a random position without obstacles
void fk::Flock::add_boid() {
  std::random_device rd;
  int x_max =
      static_cast<int>(2.5 * (f_com.get_space()[0] - 40.) / f_params.d_s);
  int y_max =
      static_cast<int>(2.5 * (f_com.get_space()[1] - 40.) / f_params.d_s);

  // Distributions of ints!! Positons are discretized. Two boids either
  // coincide, or do not coincide
  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (f_params.d_s) + 20.,
                               dist_pos_y(rd) * 0.4 * (f_params.d_s) + 20.};

  // Checks wheter a boid coincide with another

  auto find_clone = [&pos](bd::Boid& b1) {
    return b1.get_pos()[0] == pos[0] && b1.get_pos()[1] == pos[1];
  };

  // Until there are no coinciding boids, it regenerates positions
  while (std::any_of(std::execution::par, f_flock.begin(), f_flock.end(),
                     find_clone)) {
    pos = {dist_pos_x(rd) * 0.4 * (f_params.d_s) + 20.,
           dist_pos_y(rd) * 0.4 * (f_params.d_s) + 20.};
  }

  // It generates its speed and adds it to the flock
  std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
  f_flock.push_back(bd::Boid{pos, vel, f_flock[0].get_view_angle(),
                             f_flock[0].get_space(), f_params.d_s, f_params.s});
  sort();
  update_com();
}

// Add_boid in a random position considering obstacles
void fk::Flock::add_boid(std::vector<ob::Obstacle> const& obstacles) {
  std::random_device rd;
  int x_max =
      static_cast<int>(2.5 * (f_com.get_space()[0] - 40.) / f_params.d_s);
  int y_max =
      static_cast<int>(2.5 * (f_com.get_space()[1] - 40.) / f_params.d_s);

  // Distributions of ints!! Positons are discretized. Two boids either
  // coincide, or do not coincide
  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (f_params.d_s) + 20.,
                               dist_pos_y(rd) * 0.4 * (f_params.d_s) + 20.};

  // Checks wheter a boid coincide with another or it overlaps with an obstacle

  auto find_clone = [&pos](bd::Boid& b1) {
    return b1.get_pos()[0] == pos[0] && b1.get_pos()[1] == pos[1];
  };
  auto overlap = [&pos, this](ob::Obstacle const& obstacle) -> bool {
    std::valarray<double> dist = pos - obstacle.get_pos();
    return mt::vec_norm<double>(dist) <
           obstacle.get_size() + 0.6 * f_params.d_s;
  };

  // Until boid coincide with another one or overlaps with obstacle, it
  // regenerates
  while (std::any_of(std::execution::par, f_flock.begin(), f_flock.end(),
                     find_clone) ||
         std::any_of(obstacles.begin(), obstacles.end(), overlap)) {
    pos = {dist_pos_x(rd) * 0.4 * (f_params.d_s) + 20.,
           dist_pos_y(rd) * 0.4 * (f_params.d_s) + 20.};
  }

  // It generates its speed and adds it to the flock
  std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
  f_flock.push_back(bd::Boid{pos, vel, f_flock[0].get_view_angle(),
                             f_flock[0].get_space(), f_params.d_s, f_params.s});
  sort();
  update_com();
}

std::vector<bd::Boid>::iterator fk::Flock::begin() { return f_flock.begin(); }
std::vector<bd::Boid>::iterator fk::Flock::end() { return f_flock.end(); }

int fk::Flock::size() const { return static_cast<int>(f_flock.size()); }

// Add a boid in a flock, used in tests
void fk::Flock::push_back(bd::Boid const& boid) {
  assert(boid.get_par_ds() == f_params.d_s);
  assert(boid.get_par_s() == f_params.s);
  f_flock.push_back(boid);
}

// Used in tests
std::vector<bd::Boid> const& fk::Flock::get_flock() const { return f_flock; }

bd::Boid const& fk::Flock::get_boid(int n) const {
  assert(n >= 1 && n <= f_flock.size());
  return f_flock[static_cast<unsigned int>(n - 1)];
}

bd::Boid const& fk::Flock::get_com() const { return f_com; }

fk::Parameters const& fk::Flock::get_params() const { return f_params; }

void fk::Flock::set_parameter(int index, double value) {
  assert(index >= 0 && index < 5);
  switch (index) {
    case 0:
      f_params.d = value;
      break;
    case 1:
      f_params.d_s = value;
      for (auto it_s = f_flock.begin(); it_s != f_flock.end(); ++it_s) {
        it_s->set_par_ds(value);
      }
      break;
    case 2:
      f_params.s = value;
      for (auto it_s = f_flock.begin(); it_s != f_flock.end(); ++it_s) {
        it_s->set_par_s(value);
      }
      break;
    case 3:
      f_params.a = value;
      break;
    case 4:
      f_params.c = value;
      break;
    default:
      break;
  }
}

void fk::Flock::set_space(double sx, double sy) {
  assert(sx > 0 && sy > 0);
  f_com.set_space(sx, sy);
  for (auto& bd : f_flock) {
    bd.set_space(sx, sy);
  }
}

void fk::Flock::erase(std::vector<bd::Boid>::iterator it) { f_flock.erase(it); }

void fk::Flock::update_com() {
  f_com.get_vel() = {0., 0.};
  f_com.get_pos() = {0., 0.};
  for (auto bd : f_flock) {
    f_com.get_vel() += bd.get_vel();
    f_com.get_pos() += bd.get_pos();
  }
  f_com.get_vel() /= static_cast<double>(f_flock.size());
  f_com.get_pos() /= static_cast<double>(f_flock.size());
}

std::vector<bd::Boid> fk::Flock::get_neighbours(
    std::vector<bd::Boid>::iterator it) const {
  return get_vector_neighbours(f_flock, it, f_params.d);
}

// Avoid_pred for tests
std::valarray<double> fk::Flock::avoid_pred(bd::Boid const& bd,
                                            pr::Predator const& pred,
                                            double boid_pred_detection,
                                            double boid_pred_repulsion) const {
  std::valarray<double> delta_vel = {0., 0.};
  // Determines wheter to apply or not separation from predator
  (bd::boid_dist(pred, bd) < boid_pred_detection * f_params.d)
      ? delta_vel -=
        boid_pred_repulsion * f_params.s * (pred.get_pos() - bd.get_pos())
      : delta_vel;
  return delta_vel;
}

std::valarray<double> fk::Flock::avoid_pred(bd::Boid const& bd,
                                            pr::Predator const& pred) const {
  std::valarray<double> delta_vel = {0., 0.};
  // Determines wheter to apply or not separation from predator
  (bd::boid_dist(pred, bd) < 1.2 * f_params.d)
      ? delta_vel -= 0.3 * f_params.s * (pred.get_pos() - bd.get_pos())
      : delta_vel;
  return delta_vel;
}

// vel correction without obstacles (used in tests)
std::valarray<double> fk::Flock::vel_correction(
    std::vector<bd::Boid>::iterator it) {
  assert(it >= f_flock.begin() && it < f_flock.end());
  // Find its neighbours
  auto neighbours = get_neighbours(it);
  std::valarray<double> delta_vel = {0., 0.};
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size();
    std::valarray<double> local_com = {0., 0.};
    // For each boid in neighbours it applies separation is it's in range d_s,
    // and alignmnet
    for (bd::Boid bd : neighbours) {
      // Separation
      (bd::boid_dist(bd, *it) < f_params.d_s)
          ? delta_vel -= f_params.s * (bd.get_pos() - it->get_pos())
          : delta_vel;
      // Alignment
      delta_vel += f_params.a * (bd.get_vel() - it->get_vel()) /
                   static_cast<double>(n_minus);

      local_com += bd.get_pos();
    }

    // Calculates local centre of mass and apllies cohesion
    delta_vel +=
        f_params.c * (local_com / static_cast<double>(n_minus) - it->get_pos());
  }
  return delta_vel;
}

// vel correction without predators and copy_flock: used in update state
std::valarray<double> fk::Flock::vel_correction(
    std::vector<bd::Boid> const& copy_flock,
    std::vector<bd::Boid>::iterator it) {
  assert(it >= copy_flock.begin() && it < copy_flock.end() &&
         copy_flock.size() == f_flock.size());
  // Find its neighbours
  auto neighbours = get_vector_neighbours(copy_flock, it, f_params.d);
  std::valarray<double> delta_vel = {0., 0.};

  // For each boid in neighbours it applies separation is it's in range d_s,
  // and alignmnet
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size();
    std::valarray<double> local_com = {0., 0.};
    for (bd::Boid bd : neighbours) {
      // Separation
      (bd::boid_dist(bd, *it) < f_params.d_s)
          ? delta_vel -= f_params.s * (bd.get_pos() - it->get_pos())
          : delta_vel;
      // Alignment
      delta_vel += f_params.a * (bd.get_vel() - it->get_vel()) /
                   static_cast<double>(n_minus);

      local_com += bd.get_pos();
    }
    // Calculates local centre of mass and apllies cohesion
    delta_vel +=
        f_params.c * (local_com / static_cast<double>(n_minus) - it->get_pos());
  }
  return delta_vel;
}

// Overload di vel_correction with more predators used in tests
std::valarray<double> fk::Flock::vel_correction(
    std::vector<bd::Boid>::iterator it, std::vector<pr::Predator> const& preds,
    double boid_pred_detection, double boid_pred_repulsion) {
  assert(it >= f_flock.begin() && it < f_flock.end());

  auto neighbours = get_neighbours(it);
  std::valarray<double> delta_vel = {0., 0.};

  // Checks separation from predators if needed
  for (pr::Predator pt : preds) {
    (bd::boid_dist(pt, *it) < boid_pred_detection * f_params.d)
        ? delta_vel -=
          boid_pred_repulsion * f_params.s * (pt.get_pos() - it->get_pos())
        : delta_vel;
  }

  // Checks separation from neighbours if needed
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size();
    std::valarray<double> local_com = {0., 0.};
    for (bd::Boid bd : neighbours) {
      // Separation
      (bd::boid_dist(bd, *it) < f_params.d_s)
          ? delta_vel -= f_params.s * (bd.get_pos() - it->get_pos())
          : delta_vel;
      // Alignment
      delta_vel += f_params.a * (bd.get_vel() - it->get_vel()) /
                   static_cast<double>(n_minus);

      local_com += bd.get_pos();
    }
    // Cohesion
    delta_vel +=
        f_params.c * (local_com / static_cast<double>(n_minus) - it->get_pos());
  }
  return delta_vel;
}

void fk::Flock::update_global_state(double delta_t, bool brd_bhv,
                                    std::vector<pr::Predator>& preds,
                                    std::vector<ob::Obstacle> const& obs) {
  // It creates a vector of pairs of boids and ints that stores preys. The int
  // states for the predator whose preys it is.
  std::vector<std::pair<bd::Boid, int>> preys;

  std::mutex mtx;

  // Finds victims of predators
  auto bd_eaten = [this, &preds](bd::Boid const& bd) {
    // valuta se è mangiato da (almeno) un predatore
    auto above = [this, &bd](pr::Predator const& pred) -> bool {
      return bd::boid_dist(pred, bd) < 0.3 * f_params.d_s;
    };
    return std::any_of(preds.begin(), preds.end(), above);
  };

  // Removes victims
  auto last = std::remove_if(std::execution::par, f_flock.begin(),
                             f_flock.end(), bd_eaten);
  f_flock.erase(last, f_flock.end());

  //  Duplicates f_flock in copy_flock to keep track of states before updating
  //  it
  std::vector<bd::Boid> copy_flock = f_flock;

  // It initialises a vector of ints, that enables us to parallelize the update
  std::vector<int> indexes;

  for (int i = 0; static_cast<unsigned int>(i) < f_flock.size(); ++i) {
    indexes.push_back(i);
  }

  // lambda used to update global state
  auto boid_update = [&mtx, &preds, &preys, this, delta_t, brd_bhv, &copy_flock,
                      &obs](int const& index, bd::Boid const& bd) -> bd::Boid {
    // aggiorna lo stato del boid con o senza percezione predatore
    std::valarray<double> corr = {0., 0.};
    // For each boid, it calculates it vel_correction to avoid predators and
    // check wheter it's a prey or not. In case, its pushed back in the preys
    // vector, together with an int indicated whose predator it's a prey
    for (int idx = 0; static_cast<long unsigned int>(idx) < preds.size();
         ++idx) {
      corr += avoid_pred(bd, preds[static_cast<unsigned int>(idx)]);
      if (is_visible(bd, preds[static_cast<unsigned int>(idx)]) &&
          bd::boid_dist(preds[static_cast<unsigned int>(idx)], bd) <
              preds[static_cast<unsigned int>(idx)].get_range()) {
        // blocca il mutex prima della modifica di preys
        std::lock_guard<std::mutex> lck(mtx);
        preys.push_back({bd, idx});
      }
    }

    // Updates the boid state
    f_flock[static_cast<unsigned int>(index)].update_state(
        delta_t,
        vel_correction(copy_flock, copy_flock.begin() + index) +
            bd.avoid_obs(obs) + corr,
        brd_bhv);
    return f_flock[static_cast<long unsigned int>(index)];
  };

  // For each boid (from index.begin() == 1 to index.end() == end) updates its
  // state using lambda boid_update
  std::transform(std::execution::par, indexes.begin(), indexes.end(),
                 copy_flock.begin(), f_flock.begin(), boid_update);

  update_com();
  sort();

  // Using the vector of preys, it updates the state of all predators
  update_predators_state(preds, delta_t, brd_bhv, preys, obs);
}

// Update_global_state used during development in order to find correct values
// Its the same as the one previously defines, except for parameters regarding
// pred-pred repulsion, border and obstacle avoidance behavour
void fk::Flock::update_global_state(
    double delta_t, bool brd_bhv, std::vector<pr::Predator>& preds,
    std::vector<ob::Obstacle> const& obs, double border_detection,
    double border_repulsion, double boid_pred_detection,
    double boid_pred_repulsion, double boid_obs_detection,
    double boid_obs_repulsion, double pred_pred_repulsion) {
  // boid su cui applica caccia = prede
  std::vector<std::pair<bd::Boid, int>> preys;

  std::mutex mtx;

  auto bd_eaten = [this, &preds](bd::Boid const& bd) {
    auto above = [this, &bd](pr::Predator const& pred) -> bool {
      return bd::boid_dist(pred, bd) < 0.3 * f_params.d_s;
    };
    return std::any_of(preds.begin(), preds.end(), above);
  };

  auto last = std::remove_if(std::execution::par, f_flock.begin(),
                             f_flock.end(), bd_eaten);
  f_flock.erase(last, f_flock.end());

  std::vector<bd::Boid> copy_flock = f_flock;

  std::vector<int> indexes;

  for (int i = 0; i < f_flock.size(); ++i) {
    indexes.push_back(i);
  }

  auto boid_update = [&mtx, &preds, &preys, this, delta_t, brd_bhv, &copy_flock,
                      &obs, border_detection, border_repulsion,
                      boid_pred_detection, boid_pred_repulsion,
                      boid_obs_detection, boid_obs_repulsion](
                         int const& index, bd::Boid const& bd) -> bd::Boid {
    std::valarray<double> corr = {0., 0.};
    for (int idx = 0; idx < preds.size(); ++idx) {
      corr +=
          avoid_pred(bd, preds[idx], boid_pred_detection, boid_pred_repulsion);
      if (is_visible(bd, preds[idx]) &&
          bd::boid_dist(preds[idx], bd) < preds[idx].get_range()) {
        std::lock_guard<std::mutex> lck(mtx);
        preys.push_back({bd, idx});
      }
    }
    f_flock[index].update_state(
        delta_t,
        vel_correction(copy_flock, copy_flock.begin() + index) +
            bd.avoid_obs(obs, boid_obs_detection, boid_obs_repulsion) + corr,
        brd_bhv, border_detection, border_repulsion);
    return f_flock[static_cast<long unsigned int>(index)];
  };

  std::transform(std::execution::par, indexes.begin(), indexes.end(),
                 copy_flock.begin(), f_flock.begin(), boid_update);

  update_com();
  sort();

  update_predators_state(preds, delta_t, brd_bhv, preys, obs,
                         pred_pred_repulsion, boid_obs_detection,
                         boid_obs_repulsion, border_detection,
                         border_repulsion);
}

void fk::Flock::sort() {
  // Sorts boids in the flock in ascending order relative to x_position.
  // If two boids have the same x_position, it considers y_position

  auto is_less = [](bd::Boid const& bd1, bd::Boid const& bd2) {
    if (bd1.get_pos()[0] != bd2.get_pos()[0]) {
      return bd1.get_pos()[0] < bd2.get_pos()[0];
    } else {
      return bd1.get_pos()[1] < bd2.get_pos()[1];
    }
  };

  std::sort(std::execution::par, f_flock.begin(), f_flock.end(), is_less);
}

void fk::Flock::update_stats() {
  if (this->size() <= 1) {
    f_stats.av_dist = 0.;
    f_stats.dist_RMS = 0.;
    f_stats.av_vel = 0.;
    f_stats.vel_RMS = 0.;
  } else {
    double mean_dist{0};
    double square_mean_dist{0};
    double mean_vel{0};
    double square_mean_vel{0};
    int number_of_couples{0};

    // For each boid, it checks all other boids. If the distance between them is
    // less than d, it considers it as a neighbour and adds the distance to the
    // average_distance. Variable number_couples takes count of, as the name
    // says, the number of couples counted

    for (auto it = f_flock.begin(); it < f_flock.end(); ++it) {
      mean_vel += mt::vec_norm<double>(it->get_vel());
      square_mean_vel += (mt::vec_norm<double>(it->get_vel()) *
                          mt::vec_norm<double>(it->get_vel()));

      for (auto ut = it;
           ut < f_flock.end() &&
           std::abs(it->get_pos()[0] - ut->get_pos()[0]) < f_params.d;
           ++ut) {
        if (bd::boid_dist(*it, *ut) <= f_params.d &&
            bd::boid_dist(*it, *ut) > 0) {
          mean_dist += bd::boid_dist(*it, *ut);
          square_mean_dist +=
              (bd::boid_dist(*it, *ut) * bd::boid_dist(*it, *ut));
          ++number_of_couples;
        }
      }
    }

    mean_vel /= static_cast<double>(this->size());
    square_mean_vel /= static_cast<double>(this->size());
    double vel_RMS = sqrt(square_mean_vel - mean_vel * mean_vel);

    f_stats.av_vel = mean_vel;
    f_stats.vel_RMS = vel_RMS;

    // If no couples are founded, average distance and its RMS are 0
    if (number_of_couples == 0) {
      f_stats.av_dist = 0;
      f_stats.dist_RMS = 0;
    } else {
      mean_dist /= static_cast<double>(number_of_couples);
      square_mean_dist /= static_cast<double>(number_of_couples);
      double dist_RMS = sqrt(square_mean_dist - mean_dist * mean_dist);

      f_stats.av_dist = mean_dist;
      f_stats.dist_RMS = dist_RMS;
    }
  }
}

fk::Statistics const& fk::Flock::get_stats() const { return f_stats; }