#include "boid.hpp"

Boid::Boid(std::valarray<double> pos, std::valarray<double> vel) {
  assert(pos.size() != 2 || vel.size() != 2);
  b_pos = pos;
  b_vel = vel;
  b_angle = std::atan(b_vel[1] / b_vel[0]);
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
}