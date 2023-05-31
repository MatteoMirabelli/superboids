#include "boid.hpp"

std::vector<double>& Boid::get_pos() { return b_pos; }

std::vector<double> const& Boid::get_pos() const { return b_pos; }

std::vector<double>& Boid::get_vel() { return b_vel; }

std::vector<double> const& Boid::get_vel() const { return b_vel; }

double& Boid::get_angle() { return b_angle; }

double const& Boid::get_angle() const { return b_angle; }