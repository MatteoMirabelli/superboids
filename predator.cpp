#include "predator.hpp"

//#include <algorithm>
#include <execution>

Predator::Predator(std::valarray<double> const& pos,
                   std::valarray<double> const& vel, double const& view_ang,
                   double const& param_d_s, double const& param_s,
                   std::valarray<double> const& space, double const& range,
                   double const& hunger)
    : Boid(pos, vel, view_ang, space, param_d_s, param_s),
      p_range(range),
      p_hunger(hunger) {}

Predator::Predator(double const& x, double const& y, double const& vx,
                   double const& vy, double const& view_ang,
                   double const& param_d_s, double const& param_s,
                   double const& sx, double const& sy, double const& range,
                   double const& hunger)
    : Boid(x, y, vx, vy, view_ang, sx, sy, param_d_s, param_s),
      p_range(range),
      p_hunger(hunger) {}

double Predator::get_angle() const { return Boid::get_angle(); }

double Predator::get_range() const { return p_range; }

double Predator::get_hunger() const { return p_hunger; }

// vel correction per predatori: calcola contributo coesione verso com prede
std::valarray<double> Predator::predate(std::vector<Boid>& preys) {
  std::valarray<double> prey_com_pos(2);

  if (preys.size() > 0) {
    auto nearest = [&](Boid const& b1, Boid const& b2) {
      return boid_dist(b1, *this) < boid_dist(b2, *this);
    };

    std::sort(std::execution::par, preys.begin(), preys.end(), nearest);
    for (auto const& prey : preys) {
      prey_com_pos += prey.get_pos();
    }

    prey_com_pos /= preys.size();

    return p_hunger * (prey_com_pos - this->get_pos()) +
           p_hunger * (preys.size() / 2) *
               (preys[0].get_pos() - this->get_pos());
  } else {
    return std::valarray<double>{0., 0.};
    // chiaramente no prede = no correzione
  }
}

// qui lasciata commentata nel caso si voglia cambiare qualcosa rispetto a boid
// (dubito) al momento si usa update_state del sottooggetto boid, tanto Boid non
// è astratta!
/*void Predator::update_state(double delta_t, std::valarray<double> v_corr,
                            bool const& cf, double d, double k) {
  Boid::update_state(delta_t, v_corr, cf, d, k);
}*/