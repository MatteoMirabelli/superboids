#include "predator.hpp"

Predator::Predator(std::valarray<double> const& p_pos,
                   std::valarray<double> const& p_vel, double const& p_ang,
                   double const& p_range)
    : Boid(p_pos, p_vel, p_ang), p_range(p_range) {}

Predator::Predator(double const& x, double const& y, double const& vx,
                   double const& vy, double const& ang, double const& p_range)
    : Boid(x, y, vx, vy, ang), p_range(p_range) {}

double Predator::get_range() const { return p_range; }

double Predator::get_hunger() const { return p_hunger; }

std::valarray<double> Predator::predate(std::vector<Boid> const& preys){
    std::valarray<double> prey_pos(2);
    for(auto const& prey : preys){
            prey_pos += prey.get_pos();
    }
    prey_pos /= preys.size();
    return p_hunger * (prey_pos - get_pos());
}