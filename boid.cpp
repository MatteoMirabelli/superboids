#include "boid.hpp"

#include <algorithm>
#include <mutex>
#include <type_traits>

// aggiunto il passaggio delle dimensioni dello schermo

Boid::Boid(std::valarray<double> pos, std::valarray<double> vel,
           double view_ang, std::valarray<double> space, double param_ds,
           double param_s) {
  assert(pos.size() == 2 && vel.size() == 2 && space.size() == 2 &&
         pos[0] >= 0. && pos[1] >= 0. && space[0] > 0. && space[1] > 0. &&
         vec_norm(vel) < 350. && view_ang >= 0. && view_ang <= 180. &&
         param_ds >= 0. && param_s >= 0.);
  b_pos = pos;
  b_vel = vel;
  b_angle = compute_angle<double>(vel);
  b_view_angle = view_ang;
  b_space = space;
  b_param_ds = param_ds;
  b_param_s = param_s;
}

Boid::Boid(double x, double y, double vx, double vy, double view_ang, double sx,
           double sy, double param_ds, double param_s) {
  assert(x >= 0. && y >= 0. && sx > 0. && sy > 0. &&
         vec_norm(std::valarray<double>{vx, vy}) < 350. && view_ang >= 0. &&
         view_ang <= 180. && param_ds >= 0. && param_s >= 0.);
  b_pos = std::valarray<double>(2);
  b_vel = std::valarray<double>(2);
  b_pos[0] = x;
  b_pos[1] = y;
  b_vel[0] = vx;
  b_vel[1] = vy;
  b_angle = compute_angle<double>(b_vel);
  b_view_angle = view_ang;
  b_space = std::valarray<double>{sx, sy};
  b_param_ds = param_ds;
  b_param_s = param_s;
}

std::valarray<double>& Boid::get_pos() { return b_pos; }

std::valarray<double> const& Boid::get_pos() const { return b_pos; }

std::valarray<double>& Boid::get_vel() { return b_vel; }

std::valarray<double> const& Boid::get_vel() const { return b_vel; }

double Boid::get_angle() const { return b_angle; }

double Boid::get_view_angle() const { return b_view_angle; }

std::valarray<double> const& Boid::get_space() const { return b_space; }

// per cambiare range (overloadato)
void Boid::set_space(double sx, double sy) {
  assert(sx > 0 && sy > 0);
  b_space[0] = sx;
  b_space[1] = sy;
}

void Boid::set_space(std::valarray<double> const& space) { b_space = space; }

void Boid::update_state(double delta_t, std::valarray<double> delta_vel) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  // calcola l'angolo di orientamento dalla velocità:
  b_angle = compute_angle<double>(b_vel);
  // velocità massima:
  (vec_norm(b_vel) > 350.) ? b_vel *= (350. / vec_norm(b_vel)) : b_vel;
}

// Update_state for tests
void Boid::update_state(double delta_t, std::valarray<double> delta_vel,
                        bool brd_bhv, double border_detection,
                        double border_repulsion) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  if (brd_bhv == true) {
    // implementazione con periodiche
    (b_pos[0] > b_space[0] - 20.) ? b_pos[0] = 21. : b_pos[0];
    (b_pos[0] < 20.) ? b_pos[0] = b_space[0] - 21. : b_pos[0];
    (b_pos[1] > b_space[1] - 20) ? b_pos[1] = 21. : b_pos[1];
    (b_pos[1] < 20.) ? b_pos[1] = b_space[1] - 21. : b_pos[1];
  } else {
    // implementazione con bordi
    double rep = border_repulsion * vec_norm<double>(b_vel);
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

  // calcola l'angolo di orientamento dalla velocità:
  b_angle = compute_angle<double>(b_vel);
  // viene fatto *dopo* le correzioni per i bordi / periodiche!
  // velocità massima e minima:
  (vec_norm(b_vel) > 350.) ? b_vel *= (350. / vec_norm(b_vel)) : b_vel;
  (vec_norm(b_vel) < 70.) ? b_vel *= (70. / vec_norm(b_vel)) : b_vel;
}

void Boid::update_state(double delta_t, std::valarray<double> delta_vel,
                        bool brd_bhv) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  if (brd_bhv == true) {
    // implementazione con periodiche
    (b_pos[0] > b_space[0] - 20.) ? b_pos[0] = 21. : b_pos[0];
    (b_pos[0] < 20.) ? b_pos[0] = b_space[0] - 21. : b_pos[0];
    (b_pos[1] > b_space[1] - 20) ? b_pos[1] = 21. : b_pos[1];
    (b_pos[1] < 20.) ? b_pos[1] = b_space[1] - 21. : b_pos[1];
  } else {
    // implementazione con bordi
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

  // calcola l'angolo di orientamento dalla velocità:
  b_angle = compute_angle<double>(b_vel);
  // viene fatto *dopo* le correzioni per i bordi / periodiche!
  // velocità massima e minima:
  (vec_norm(b_vel) > 350.) ? b_vel *= (350. / vec_norm(b_vel)) : b_vel;
  (vec_norm(b_vel) < 70.) ? b_vel *= (70. / vec_norm(b_vel)) : b_vel;
}

// Avoid_obs for tests
std::valarray<double> Boid::avoid_obs(std::vector<Obstacle> const& obstacles,
                                      double obstacle_detection,
                                      double obstacle_repulsion) const {
  if (obstacles.size() == 0) {
    return std::valarray<double>{0., 0.};
  } else {
    std::valarray<double> delta_vel{0., 0.};

    double rep = obstacle_repulsion * vec_norm<double>(b_vel);
    for (auto const& ob : obstacles) {
      double range = ob.get_size() + obstacle_detection * b_param_ds;
      ((b_pos[0] - ob.get_pos()[0]) > 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] += rep * b_param_s / (b_pos[0] - ob.get_pos()[0])
          : delta_vel[0];

      ((b_pos[0] - ob.get_pos()[0]) < 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] -= rep * b_param_s / (ob.get_pos()[0] - b_pos[0])
          : delta_vel[0];

      ((b_pos[1] - ob.get_pos()[1]) > 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] += rep * b_param_s / (b_pos[1] - ob.get_pos()[1])
          : delta_vel[1];

      ((b_pos[1] - ob.get_pos()[1]) < 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] -= rep * b_param_s / (ob.get_pos()[1] - b_pos[1])
          : delta_vel[1];
    }
    return delta_vel;
  }
}

std::valarray<double> Boid::avoid_obs(
    std::vector<Obstacle> const& obstacles) const {
  if (obstacles.size() == 0) {
    return std::valarray<double>{0., 0.};
  } else {
    std::valarray<double> delta_vel{0., 0.};

    double rep = 1.8 * vec_norm<double>(b_vel);
    for (auto const& ob : obstacles) {
      double range = ob.get_size() + 2.7 * b_param_ds;

      ((b_pos[0] - ob.get_pos()[0]) > 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] += rep * b_param_s / (b_pos[0] - ob.get_pos()[0])
          : delta_vel[0];

      ((b_pos[0] - ob.get_pos()[0]) < 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[0] -= rep * b_param_s / (ob.get_pos()[0] - b_pos[0])
          : delta_vel[0];

      ((b_pos[1] - ob.get_pos()[1]) > 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] += rep * b_param_s / (b_pos[1] - ob.get_pos()[1])
          : delta_vel[1];

      ((b_pos[1] - ob.get_pos()[1]) < 0 &&
       vec_norm<double>(b_pos - ob.get_pos()) < range &&
       is_obs_visible(ob, *this))
          ? delta_vel[1] -= rep * b_param_s / (ob.get_pos()[1] - b_pos[1])
          : delta_vel[1];
    }
    return delta_vel;
  }
}

double Boid::get_par_ds() const { return b_param_ds; }

double Boid::get_par_s() const { return b_param_s; }

void Boid::set_par_ds(double new_ds) { b_param_ds = new_ds; }

void Boid::set_par_s(double new_s) { b_param_s = new_s; }

template <typename T>
T vec_norm(std::valarray<T> vec) {
  return std::sqrt(std::pow(vec, {2., 2.}).sum());
  return std::sqrt(std::pow(vec, {2, 2}).sum());
}

double boid_dist(Boid const& bd_1, Boid const& bd_2) {
  // distanza = norma della differenza
  return vec_norm<double>(bd_1.get_pos() - bd_2.get_pos());
}

template <typename T>
T compute_angle(std::valarray<T> const& vec) {
  // assert(vec.size() == 2);
  double angle{0.};
  if (vec[1] == 0. && vec[0] < 0.) {
    angle = -90.;
  } else if (vec[1] == 0. && vec[0] > 0.) {
    angle = 90.;
  } else if (vec[0] == 0. && vec[1] == 0.) {  // rimediato allo spiacevole baco
    angle = 0.;
  } else if (vec[0] == 0. && vec[1] > 0.) {
    angle = 0.;
  } else if (vec[0] == 0. && vec[1] < 0.) {
    angle = 180.;
  } else {
    angle = std::atan(vec[0] / vec[1]) / M_PI * 180;
    (vec[1] < 0. && vec[0] < 0.) ? angle -= 180. : angle;
    (vec[1] < 0. && vec[0] > 0.) ? angle += 180. : angle;
  }
  return angle;
}

bool is_visible(Boid const& bd_1, Boid const& bd_2) {
  double view_angle = bd_2.get_view_angle();
  double boid_angle = bd_2.get_angle();
  assert(view_angle >= 0. && view_angle <= 180.);

  double relative_angle =
      compute_angle<double>(bd_1.get_pos() - bd_2.get_pos());

  if (std::abs(relative_angle - boid_angle) <= 180.) {
    return std::abs(relative_angle - boid_angle) <= view_angle;
  } else {
    return (360. - std::abs(relative_angle - boid_angle)) <= view_angle;
  }
}

bool is_obs_visible(Obstacle const& obs, Boid const& bd) {
  double view_angle = bd.get_view_angle();

  assert(view_angle >= 0. && view_angle <= 180.);
  double relative_angle = compute_angle<double>(obs.get_pos() - bd.get_pos());

  if (std::abs(relative_angle - bd.get_angle()) <= 180.) {
    return std::abs(relative_angle - bd.get_angle()) <= view_angle;
  } else {
    return (360. - std::abs(relative_angle - bd.get_angle())) <= view_angle;
  }
}

std::vector<Boid> get_vector_neighbours(std::vector<Boid> const& full_vec,
                                        std::vector<Boid>::iterator it,
                                        double dist) {
  std::vector<Boid> neighbours;
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