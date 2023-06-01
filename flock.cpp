#include "flock.hpp"

#include <algorithm>
#include <numeric>
#include <random>

Flock::Flock(double const& d, std::valarray<double> const& params,
             int const& bd_n, Boid const& com)
    : f_d{d}, f_com{com} {
  assert(params.size() == 4);
  f_params = params;
  std::random_device rd;
  std::uniform_real_distribution<double> dist_pos_x(com.get_pos()[0] - 250.,
                                                    com.get_pos()[0] + 250.);
  std::uniform_real_distribution<double> dist_vel_x(com.get_vel()[0] - 20.,
                                                    com.get_vel()[0] + 20.);
  std::uniform_real_distribution<double> dist_pos_y(com.get_pos()[1] - 250.,
                                                    com.get_pos()[1] + 250.);
  std::uniform_real_distribution<double> dist_vel_y(com.get_vel()[1] - 20.,
                                                    com.get_vel()[1] + 20.);
  for (auto n = 0; n < bd_n; ++n) {
    Boid boid{{dist_pos_x(rd), dist_pos_y(rd)},
              {dist_vel_x(rd), dist_vel_y(rd)}};
    f_flock.push_back(boid);
  }
  (--f_flock.end())->get_pos() += bd_n * (com.get_pos());
  (--f_flock.end())->get_vel() += bd_n * (com.get_vel());
}

std::vector<Boid>::iterator Flock::begin() { return f_flock.begin(); }
std::vector<Boid>::iterator Flock::end() { return f_flock.end(); }

double Flock::size() const { return f_flock.size(); }

void Flock::push_back(Boid const& boid) { f_flock.push_back(boid); }

Boid& Flock::get_boid(int n) { return f_flock[n - 1]; }

Boid const& Flock::get_boid(int n) const { return f_flock[n - 1]; }

std::valarray<double> Flock::get_params() const { return f_params; }

void Flock::erase(int n) {
  auto it = f_flock.begin() + n - 1;
  f_flock.erase(it);
}

void Flock::update_com() {
  f_com.get_vel() = {0., 0.};
  f_com.get_pos() = {0., 0.};
  for (auto bd : f_flock) {
    f_com.get_vel() += bd.get_vel();
    f_com.get_pos() += bd.get_pos();
  }
  f_com.get_vel() /= f_flock.size();
  f_com.get_pos() /= f_flock.size();
};

// implementato il get neighbour e la correzione con iteratore; alternativa con
// i numeri per non avere problemi ma tanto la chiamata è fatta dentro update
// state, quindi non c'è il rischio di passare iteratori di altri flock!
std::vector<Boid> Flock::get_neighbours(std::vector<Boid>::iterator it) {
  std::vector<Boid> neighbours;
  // auto bd_2 = f_flock[n - 1];
  auto ev_dist = [&](Boid bd_1) {
    return boid_dist(bd_1, *it) < f_d && boid_dist(bd_1, *it) > 0.;
  };
  std::copy_if(f_flock.begin(), f_flock.end(), std::back_inserter(neighbours),
               ev_dist);
  return neighbours;
}

std::valarray<double> Flock::vel_correction(std::vector<Boid>::iterator it) {
  auto neighbours = this->get_neighbours(it);
  std::valarray<double> delta_vel = {0., 0.};
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size() - 1;
    std::valarray<double> local_com = {0., 0.};
    for (Boid bd : neighbours) {
      // separation
      (boid_dist(bd, *it) < f_params[0])
          ? delta_vel -= f_params[1] * (bd.get_pos() - it->get_pos())
          : delta_vel;
      // alignment
      delta_vel += f_params[2] * (bd.get_vel() - it->get_vel()) / n_minus;
      local_com += bd.get_pos();
    }
    // cohesion
    delta_vel += f_params[3] * (local_com / n_minus - it->get_pos());
  }
  return delta_vel;
}

void Flock::update_flock_state(double const& delta_t) {
  for (auto it = f_flock.begin(); it != f_flock.end(); ++it) {
    it->update_state(delta_t, this->vel_correction(it));
  }
  this->update_com();
}