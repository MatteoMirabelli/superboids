#include "flock.hpp"

#include <algorithm>
#include <numeric>
#include <random>

Flock::Flock(double const& d, Parameters const& params, int const& bd_n,
             Boid const& com)
    : f_d{d}, f_com{com}, f_flock{} {
  f_params = params;  // In ordine d_s, s, a, c

  // Genera casualmente, secondo distribuzioni uniformi attorno al centro di
  // massa, lo stormo

  std::random_device rd;
  std::uniform_real_distribution<> dist_pos_x(com.get_pos()[0] - 360.,
                                              com.get_pos()[0] + 360.1);
  std::uniform_real_distribution<> dist_vel_x(com.get_vel()[0] - 150.,
                                              com.get_vel()[0] + 150.1);
  std::uniform_real_distribution<> dist_pos_y(com.get_pos()[1] - 360.,
                                              com.get_pos()[1] + 360.1);
  std::uniform_real_distribution<> dist_vel_y(com.get_vel()[1] - 150.,
                                              com.get_vel()[1] + 150.1);
  std::valarray<double> final_pos{0., 0.};
  std::valarray<double> final_vel{0., 0.};
  for (auto n = 0; n < bd_n - 1; ++n) {
    f_flock.push_back(Boid{{dist_pos_x(rd), dist_pos_y(rd)},
                           {dist_vel_x(rd), dist_vel_y(rd)}});
    final_pos += f_flock[n].get_pos();
    final_vel += f_flock[n].get_vel();
  }
  f_flock.push_back(
      Boid{bd_n * com.get_pos() - final_pos, bd_n * com.get_vel() - final_vel});
}

std::vector<Boid>::iterator Flock::begin() { return f_flock.begin(); }
std::vector<Boid>::iterator Flock::end() { return f_flock.end(); }

double Flock::size() const { return f_flock.size(); }

void Flock::push_back(Boid const& boid) { f_flock.push_back(boid); }

Boid& Flock::get_boid(int n) { return f_flock[n - 1]; }

Boid const& Flock::get_boid(int n) const { return f_flock[n - 1]; }

Parameters const& Flock::get_params() const { return f_params; }

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
}

// implementato il get neighbour e la correzione con iteratore; alternativa con
// i numeri per non avere problemi ma tanto la chiamata è fatta dentro update
// state, quindi non c'è il rischio di passare iteratori di altri flock!

// errata corrige: può essere anzi utile che non vi sia vincolo per utilizzare
// flock di ostacoli!
std::vector<Boid> Flock::get_neighbours(std::vector<Boid>::iterator it) {
  std::vector<Boid> neighbours;
  // auto bd_2 = f_flock[n - 1];
  auto ev_dist = [&](Boid bd_1) {
    return boid_dist(bd_1, *it) < f_d && boid_dist(bd_1, *it) > 0. &&
           is_visible(bd_1, *it, 120.);
  };
  std::copy_if(f_flock.begin(), f_flock.end(), std::back_inserter(neighbours),
               ev_dist);
  return neighbours;
}

std::valarray<double> Flock::vel_correction(std::vector<Boid>::iterator it) {
  auto neighbours = this->get_neighbours(it);
  std::valarray<double> delta_vel = {0., 0.};
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size();
    std::valarray<double> local_com = {0., 0.};
    for (Boid bd : neighbours) {
      // separation
      (boid_dist(bd, *it) < f_params.d_s)
          ? delta_vel -= f_params.s * (bd.get_pos() - it->get_pos())
          : delta_vel;
      // alignment
      delta_vel += f_params.a * (bd.get_vel() - it->get_vel()) / n_minus;

      local_com += bd.get_pos();
    }
    // cohesion
    delta_vel += f_params.c * (local_com / n_minus - it->get_pos());
  }
  return delta_vel;
}

void Flock::update_flock_state(double const& delta_t) {
  std::vector<Boid> copy_flock = f_flock;
  auto it = f_flock.begin();
  std::for_each(copy_flock.begin(), copy_flock.end(), [&](Boid& bd) {
    bd.update_state(delta_t, this->vel_correction(it));
    ++it;
  });
  f_flock = copy_flock;
  this->update_com();
}

Statistics Flock::get_statistics() {}