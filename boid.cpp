#include "boid.hpp"

#include <type_traits>

// aggiunto il passaggio delle dimensioni dello schermo

Boid::Boid(std::valarray<double> pos, std::valarray<double> vel,
           double view_ang, std::valarray<double> space, double param_ds,
           double param_s) {
  assert(pos.size() == 2 && vel.size() == 2 && space.size() == 2 &&
         pos[0] >= 0 && pos[1] >= 0 && space[0] > 0 && space[1] > 0 &&
         vec_norm(vel) < 350. && view_ang >= 0. && view_ang <= 180. &&
         param_ds >= 0 && param_s >= 0);
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
  assert(x >= 0 && y >= 0 && sx > 0 && sy > 0 &&
         vec_norm(std::valarray<double>{vx, vy}) < 350. && view_ang >= 0. &&
         view_ang <= 180. && param_ds >= 0 && param_s >= 0);
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

double& Boid::get_angle() { return b_angle; }

double const& Boid::get_angle() const { return b_angle; }

double const& Boid::get_view_angle() const { return b_view_angle; }

// per cambiare range (overloadato)
void Boid::set_space(double const& sx, double const& sy) {
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

void Boid::update_state(double delta_t, std::valarray<double> delta_vel,
                        bool const& b, double d, double k) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  if (b == true) {
    // implementazione con periodiche
    (b_pos[0] > b_space[0] - 20.) ? b_pos[0] = 21. : b_pos[0];
    (b_pos[0] < 20.) ? b_pos[0] = b_space[0] - 21. : b_pos[0];
    (b_pos[1] > b_space[1]) ? b_pos[1] = 21. : b_pos[1];
    (b_pos[1] < 20.) ? b_pos[1] = b_space[1] - 21. : b_pos[1];
  } else {
    // implementazione con bordi
    (b_pos[0] > b_space[0] - 1.7 * d)
        ? b_vel[0] -= 2 * k * (b_space[0] - b_pos[0])
        : b_vel[0];
    (b_pos[0] < 1.7 * d) ? b_vel[0] += 2 * k * b_pos[0] : b_vel[0];
    (b_pos[1] > b_space[1] - 1.7 * d)
        ? b_vel[1] -= 2 * k * (b_space[1] - b_pos[1])
        : b_vel[1];
    (b_pos[1] < 1.7 * d) ? b_vel[1] += 2 * k * b_pos[1] : b_vel[1];
  }
  // calcola l'angolo di orientamento dalla velocità:
  b_angle = compute_angle<double>(b_vel);
  // viene fatto *dopo* le correzioni per i bordi / periodiche!
  // velocità massima:
  (vec_norm(b_vel) > 350.) ? b_vel *= (350. / vec_norm(b_vel)) : b_vel;
}

template <typename T>
T vec_norm(std::valarray<T> vec) {
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
    angle = 270.;
  } else if (vec[1] == 0. && vec[0] > 0.) {
    angle = 90.;
  } else if (vec[1] == 0. && vec[0] == 0.) {  // rimediato allo spiacevole baco
    angle = 0.;
  } else {
    angle = std::atan(vec[0] / vec[1]) / M_PI * 180;
    (vec[1] < 0.) ? angle += 180. : angle;
  }
  return angle;
}

bool is_visible(Boid const& bd_1, Boid const& bd_2) {
  double angle = bd_2.get_view_angle();
  assert(angle >= 0. && angle <= 180.);
  return std::abs(compute_angle<double>(bd_1.get_pos() - bd_2.get_pos()) -
                  bd_2.get_angle()) <= angle;
}