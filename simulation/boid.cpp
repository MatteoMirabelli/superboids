#include "boid.hpp"

#include <algorithm>
#include <mutex>
#include <type_traits>

#include "math.hpp"

bd::Boid::Boid(std::valarray<double> pos, std::valarray<double> vel,
           double view_ang, std::valarray<double> space, double param_ds,
           double param_s) {
  assert(pos.size() == 2 && vel.size() == 2 && space.size() == 2 &&
         pos[0] >= 0. && pos[1] >= 0. && space[0] > 0. && space[1] > 0. &&
         mt::vec_norm<double>(vel) < 350. && view_ang >= 0. && view_ang <= 180. &&
         param_ds >= 0. && param_s >= 0.);
  b_pos = pos;
  b_vel = vel;
  b_angle = mt::compute_angle<double>(vel);
  b_view_angle = view_ang;
  b_space = space;
  b_param_ds = param_ds;
  b_param_s = param_s;
}

bd::Boid::Boid(double x, double y, double vx, double vy, double view_ang, double sx,
           double sy, double param_ds, double param_s) {
  assert(x >= 0. && y >= 0. && sx > 0. && sy > 0. &&
         mt::vec_norm<double>(std::valarray<double>{vx, vy}) < 350. && view_ang >= 0. &&
         view_ang <= 180. && param_ds >= 0. && param_s >= 0.);
  b_pos = std::valarray<double>(2);
  b_vel = std::valarray<double>(2);
  b_pos[0] = x;
  b_pos[1] = y;
  b_vel[0] = vx;
  b_vel[1] = vy;
  b_angle = mt::compute_angle<double>(b_vel);
  b_view_angle = view_ang;
  b_space = std::valarray<double>{sx, sy};
  b_param_ds = param_ds;
  b_param_s = param_s;
}

std::valarray<double>& bd::Boid::get_pos() { return b_pos; }

std::valarray<double> const& bd::Boid::get_pos() const { return b_pos; }

std::valarray<double>& bd::Boid::get_vel() { return b_vel; }

std::valarray<double> const& bd::Boid::get_vel() const { return b_vel; }

double bd::Boid::get_angle() const { return b_angle; }

double bd::Boid::get_view_angle() const { return b_view_angle; }

std::valarray<double> const& bd::Boid::get_space() const { return b_space; }

void bd::Boid::set_space(double sx, double sy) {
  assert(sx > 0 && sy > 0);
  b_space[0] = sx;
  b_space[1] = sy;
}

void bd::Boid::set_space(std::valarray<double> const& space) { b_space = space; }

// Used in a few tests implemented early
void bd::Boid::update_state(double delta_t, std::valarray<double> delta_vel) {
  // Update speed and position
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  // Computes boid's angle
  b_angle = mt::compute_angle<double>(b_vel);
  // velocità massima:
  (mt::vec_norm<double>(b_vel) > 350.) ? b_vel *= (350. / mt::vec_norm<double>(b_vel)) : b_vel;
}

// Update_state for tests
void bd::Boid::update_state(double delta_t, std::valarray<double> delta_vel,
                        bool brd_bhv, double border_detection,
                        double border_repulsion) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  if (brd_bhv == true) {
    // Periodic conditions
    (b_pos[0] > b_space[0] - 20.) ? b_pos[0] = 21. : b_pos[0];
    (b_pos[0] < 20.) ? b_pos[0] = b_space[0] - 21. : b_pos[0];
    (b_pos[1] > b_space[1] - 20) ? b_pos[1] = 21. : b_pos[1];
    (b_pos[1] < 20.) ? b_pos[1] = b_space[1] - 21. : b_pos[1];
  } else {
    // Border repulsion
    double rep = border_repulsion * mt::vec_norm<double>(b_vel);
    (b_pos[0] > b_space[0] - 30. - border_detection * b_param_ds)
        ? b_vel[0] -= rep * b_param_s / (b_space[0] - b_pos[0])
        : b_vel[0];
    (b_pos[0] < border_detection * b_param_ds + 30.)
        ? b_vel[0] += rep * b_param_s / (b_pos[0])
        : b_vel[0];
    (b_pos[1] > b_space[1] - 30. - border_detection * b_param_ds)
        ? b_vel[1] -= rep * b_param_s / (b_space[1] - b_pos[1])
        : b_vel[1];
    (b_pos[1] < border_detection * b_param_ds + 30.)
        ? b_vel[1] += rep * b_param_s / (b_pos[1])
        : b_vel[1];
  }

  // Computes boid's angle
  b_angle = mt::compute_angle<double>(b_vel);

  // Corrects, if needed, the speed according to minimum and maximum velocity
  (mt::vec_norm<double>(b_vel) > 350.) ? b_vel *= (350. / mt::vec_norm<double>(b_vel)) : b_vel;
  (mt::vec_norm<double>(b_vel) < 70.) ? b_vel *= (70. / mt::vec_norm<double>(b_vel)) : b_vel;
}

void bd::Boid::update_state(double delta_t, std::valarray<double> delta_vel,
                        bool brd_bhv) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  if (brd_bhv == true) {
    // Periodic conditions
    (b_pos[0] > b_space[0] - 20.) ? b_pos[0] = 21. : b_pos[0];
    (b_pos[0] < 20.) ? b_pos[0] = b_space[0] - 21. : b_pos[0];
    (b_pos[1] > b_space[1] - 20) ? b_pos[1] = 21. : b_pos[1];
    (b_pos[1] < 20.) ? b_pos[1] = b_space[1] - 21. : b_pos[1];
  } else {
    // Border repulsion
    (b_pos[0] > b_space[0] - 30. - 13. * b_param_ds)
        ? b_vel[0] -=
          3.5 * b_param_s / (b_pos[0] - b_space[0] + 30. + 2.5 * b_param_ds)
        : b_vel[0];
    (b_pos[0] < 13. * b_param_ds + 30.)
        ? b_vel[0] += 3.5 * b_param_s / (2.5 * b_param_ds + 30. - b_pos[0])
        : b_vel[0];
    (b_pos[1] > b_space[1] - 30. - 13. * b_param_ds)
        ? b_vel[1] -=
          3.5 * b_param_s / (b_pos[1] - b_space[1] + 30. + 2.5 * b_param_ds)
        : b_vel[1];
    (b_pos[1] < 13. * b_param_ds + 30.)
        ? b_vel[1] += 3.5 * b_param_s / (2.5 * b_param_ds + 30. - b_pos[1])
        : b_vel[1];
  }

  // Computes boid's angle
  b_angle = mt::compute_angle<double>(b_vel);

  // Corrects, if needed, the speed according to minimum and maximum velocity
  (mt::vec_norm<double>(b_vel) > 350.) ? b_vel *= (350. / mt::vec_norm<double>(b_vel)) : b_vel;
  (mt::vec_norm<double>(b_vel) < 70.) ? b_vel *= (70. / mt::vec_norm<double>(b_vel)) : b_vel;
}

// Avoid_obs for tests
std::valarray<double> bd::Boid::avoid_obs(std::vector<ob::Obstacle> const& obstacles,
                                      double obstacle_detection,
                                      double obstacle_repulsion) const {
  if (obstacles.size() == 0) {
    return std::valarray<double>{0., 0.};
  } else {
    std::valarray<double> delta_vel{0., 0.};
    // for each obstacles, it checks wheter the bois is or not near it and
    // wheter or not it sees it. In case it applies a repulsion inverse to the
    // distace for each componenent
    double rep = obstacle_repulsion * mt::vec_norm<double>(b_vel);
    for (auto const& ob : obstacles) {
      double range = ob.get_size() + obstacle_detection * b_param_ds;
      ((b_pos[0] - ob.get_pos()[0]) > 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] += rep * b_param_s / (b_pos[0] - ob.get_pos()[0])
          : delta_vel[0];

      ((b_pos[0] - ob.get_pos()[0]) < 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] -= rep * b_param_s / (ob.get_pos()[0] - b_pos[0])
          : delta_vel[0];

      ((b_pos[1] - ob.get_pos()[1]) > 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] += rep * b_param_s / (b_pos[1] - ob.get_pos()[1])
          : delta_vel[1];

      ((b_pos[1] - ob.get_pos()[1]) < 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] -= rep * b_param_s / (ob.get_pos()[1] - b_pos[1])
          : delta_vel[1];
    }
    return delta_vel;
  }
}

std::valarray<double> bd::Boid::avoid_obs(
    std::vector<ob::Obstacle> const& obstacles) const {
  if (obstacles.size() == 0) {
    return std::valarray<double>{0., 0.};
  } else {
    std::valarray<double> delta_vel{0., 0.};

    // for each obstacles, it checks wheter the bois is or not near it and
    // wheter or not it sees it. In case it applies a repulsion inverse to the
    // distace for each componenent

    double rep = 1.8 * mt::vec_norm<double>(b_vel);
    for (auto const& ob : obstacles) {
      double range = ob.get_size() + 2.7 * b_param_ds;

      ((b_pos[0] - ob.get_pos()[0]) > 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] += rep * b_param_s / (b_pos[0] - ob.get_pos()[0])
          : delta_vel[0];

      ((b_pos[0] - ob.get_pos()[0]) < 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] -= rep * b_param_s / (ob.get_pos()[0] - b_pos[0])
          : delta_vel[0];

      ((b_pos[1] - ob.get_pos()[1]) > 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] += rep * b_param_s / (b_pos[1] - ob.get_pos()[1])
          : delta_vel[1];

      ((b_pos[1] - ob.get_pos()[1]) < 0 &&
       mt::vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] -= rep * b_param_s / (ob.get_pos()[1] - b_pos[1])
          : delta_vel[1];
    }
    return delta_vel;
  }
}

double bd::Boid::get_par_ds() const { return b_param_ds; }

double bd::Boid::get_par_s() const { return b_param_s; }

void bd::Boid::set_par_ds(double new_ds) { b_param_ds = new_ds; }

void bd::Boid::set_par_s(double new_s) { b_param_s = new_s; }

// If bd_1 is visible by bd_2, it returns true
bool bd::is_visible(bd::Boid const& bd_1, bd::Boid const& bd_2) {
  double view_angle = bd_2.get_view_angle();
  double boid_angle = bd_2.get_angle();
  assert(view_angle >= 0. && view_angle <= 180.);

  double relative_angle =
      mt::compute_angle<double>(bd_1.get_pos() - bd_2.get_pos());

  if (std::abs(relative_angle - boid_angle) <= 180.) {
    return std::abs(relative_angle - boid_angle) <= view_angle;
  } else {
    return (360. - std::abs(relative_angle - boid_angle)) <= view_angle;
  }
}

// If obs is visible by bd, it returns true
bool bd::is_obs_visible(ob::Obstacle const& obs, bd::Boid const& bd) {
  double view_angle = bd.get_view_angle();

  assert(view_angle >= 0. && view_angle <= 180.);
  double relative_angle = mt::compute_angle<double>(obs.get_pos() - bd.get_pos());

  if (std::abs(relative_angle - bd.get_angle()) <= 180.) {
    return std::abs(relative_angle - bd.get_angle()) <= view_angle;
  } else {
    return (360. - std::abs(relative_angle - bd.get_angle())) <= view_angle;
  }
}

double bd::boid_dist(bd::Boid const& bd_1, bd::Boid const& bd_2) {
  return mt::vec_norm<double>(bd_1.get_pos() - bd_2.get_pos());
}

// Given a vector and an iterator, it finds all of its neighbours, with the condition that the vector is SORTED
std::vector<bd::Boid> bd::get_vector_neighbours(std::vector<bd::Boid> const& full_vec,
                                        std::vector<bd::Boid>::iterator it,
                                        double dist) {
  std::vector<bd::Boid> neighbours;
  assert(it >= full_vec.begin() && it <= full_vec.end());
  if (it >= full_vec.begin() && it < full_vec.end()) {
    auto et = it;
    for (; et != full_vec.end(); ++et) {
      if (std::abs(it->get_pos()[0] - et->get_pos()[0]) > dist) break;
      if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
          is_visible(*et, *it) == true) {
        neighbours.push_back(*et);
      }
    }
    et = it;
    for (; et != full_vec.begin(); --et) {
      if (std::abs(it->get_pos()[0] - et->get_pos()[0]) > dist) break;
      if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
          is_visible(*et, *it) == true) {
        neighbours.push_back(*et);
      }
    }
    if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
        is_visible(*et, *it) == true) {
      neighbours.push_back(*et);
    }
  }
  return neighbours;
}