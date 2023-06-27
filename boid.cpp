#include "boid.hpp"

#include <type_traits>

Boid::Boid(std::valarray<double> pos, std::valarray<double> vel, double angle) {
  assert(pos.size() == 2 || vel.size() == 2);
  b_pos = pos;
  b_vel = vel;
  b_angle = compute_angle<double>(vel);
  b_view_angle = angle;
}

Boid::Boid(double x, double y, double vx, double vy, double ang) {
  assert(x >= 0 && y >= 0);
  b_pos = std::valarray<double>(2);
  b_vel = std::valarray<double>(2);
  b_pos[0] = x;
  b_pos[1] = y;
  b_vel[0] = vx;
  b_vel[1] = vy;
  b_angle = compute_angle<double>(b_vel);
  b_view_angle = ang;
}

std::valarray<double>& Boid::get_pos() { return b_pos; }

std::valarray<double> const& Boid::get_pos() const { return b_pos; }

std::valarray<double>& Boid::get_vel() { return b_vel; }

std::valarray<double> const& Boid::get_vel() const { return b_vel; }

double& Boid::get_angle() { return b_angle; }

double const& Boid::get_angle() const { return b_angle; }

double const& Boid::get_view_angle() const { return b_view_angle; }

void Boid::update_state(double delta_t, std::valarray<double> delta_vel) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  b_angle = compute_angle<double>(b_vel);
}

void Boid::update_state(double delta_t, std::valarray<double> delta_vel,
                        bool const& b, double d, double k) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  b_angle = compute_angle<double>(b_vel);

  if (b == true) {
    // implementazione con periodiche
    (b_pos[0] > 1800.) ? b_pos[0] = 21. : b_pos[0];
    (b_pos[0] < 20.) ? b_pos[0] = 1799. : b_pos[0];
    (b_pos[1] > 1000.) ? b_pos[1] = 21. : b_pos[1];
    (b_pos[1] < 20.) ? b_pos[1] = 999. : b_pos[1];
  } else {
    // implementazione con bordi

    (b_pos[0] > 1800 - 1.7 * d) ? b_vel[0] -= 2 * k * (1800 - b_pos[0])
                                : b_vel[0];
    (b_pos[0] < 1.7 * d) ? b_vel[0] += 2 * k * b_pos[0] : b_vel[0];
    (b_pos[1] > 1000 - 1.7 * d) ? b_vel[1] -= 2 * k * (1000 - b_pos[1])
                                : b_vel[1];
    (b_pos[1] < 1.7 * d) ? b_vel[1] += 2 * k * b_pos[1] : b_vel[1];
  }
}

// Nota per il futuro: passare ai boids i parametri dello schermo per non avere
// problemi di portabilità

template <typename T>
T vec_norm(std::valarray<T> vec) {
  assert(std::is_arithmetic_v<T>);
  return std::sqrt(std::pow(vec, {2, 2}).sum());
}

double boid_dist(Boid const& bd_1, Boid const& bd_2) {
  return vec_norm<double>(bd_1.get_pos() - bd_2.get_pos());
}

template <typename T>
T compute_angle(std::valarray<T> const& vec) {
  // assert(vec.size() == 2);
  double angle{0.};
  if (vec[1] == 0 && vec[0] < 0) {
    angle = 270.;
  } else if (vec[1] == 0 && vec[0] > 0) {
    angle = 90.;
  } else {
    angle = std::atan(vec[0] / vec[1]) / M_PI * 180;
    (vec[1] < 0) ? angle += 180 : angle;
  }
  return angle;
}

bool is_visible(Boid const& bd_1, Boid const& bd_2) {
  double angle = bd_2.get_view_angle();
  assert(angle >= 0. &&  angle <= 180.);
  return std::abs(compute_angle<double>(bd_1.get_pos() - bd_2.get_pos()) -
                  bd_2.get_angle()) < angle;
}