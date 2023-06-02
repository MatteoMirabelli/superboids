#include "boid.hpp"

#include <type_traits>

Boid::Boid(std::valarray<double> pos, std::valarray<double> vel) {
  assert(pos.size() == 2 || vel.size() == 2);
  b_pos = pos;
  b_vel = vel;
  b_angle = std::atan(b_vel[0] / b_vel[1]) / M_PI * 180;
  (b_vel[1] < 0) ? b_angle += 180 : b_angle;
}

std::valarray<double>& Boid::get_pos() { return b_pos; }

std::valarray<double> const& Boid::get_pos() const { return b_pos; }

std::valarray<double>& Boid::get_vel() { return b_vel; }

std::valarray<double> const& Boid::get_vel() const { return b_vel; }

double& Boid::get_angle() { return b_angle; }

double const& Boid::get_angle() const { return b_angle; }

void Boid::update_state(double delta_t, std::valarray<double> delta_vel) {
  b_vel += delta_vel;
  b_pos += (b_vel * delta_t);
  b_angle = std::atan(b_vel[0] / b_vel[1]) / M_PI * 180;
  (b_vel[1] < 0) ? b_angle += 180 : b_angle;
  (b_pos[0] > 1900.) ? b_pos[0] = 0. : b_pos[0];
  (b_pos[0] < 0.) ? b_pos[0] = 1900. : b_pos[0];
  (b_pos[1] > 1000.) ? b_pos[1] = 0. : b_pos[1];
  (b_pos[1] < 0.) ? b_pos[1] = 1000. : b_pos[1];
}

template <typename T>
T vec_norm(std::valarray<T> vec) {
  assert(std::is_arithmetic_v<T>);
  return std::sqrt(std::pow(vec, {2, 2}).sum());
}

double boid_dist(Boid const& bd_1, Boid const& bd_2) {
  return vec_norm<double>(bd_1.get_pos() - bd_2.get_pos());
}