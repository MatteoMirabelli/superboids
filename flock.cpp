#include "flock.hpp"

#include <algorithm>
#include <numeric>
#include <random>

Flock::Flock(double const& d, Parameters const& params, int const& bd_n,
             Boid const& com)
    : f_d{d}, f_com{com}, f_params{params}, f_flock{} {
  // Genera casualmente, secondo distribuzioni uniformi attorno al centro di
  // massa, lo stormo
  assert(bd_n >= 0);

  if (bd_n == 0) {
    f_flock = std::vector<Boid>(0);
  } else {
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
    f_flock.push_back(Boid{bd_n * com.get_pos() - final_pos,
                           bd_n * com.get_vel() - final_vel});
  }
}

Flock::Flock(double const& d, Parameters const& params, int const& bd_n)
    : f_d{d}, f_params{params}, f_flock{} {
  // Genera casualmente, secondo distribuzioni uniformi, i boids
  assert(bd_n >= 0);
  std::random_device rd;
  std::uniform_real_distribution<> dist_pos_x(20., 1880.);
  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_pos_y(20., 980.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  f_com = Boid{{0., 0.}, {0., 0.}};
  for (auto n = 0; n < bd_n; ++n) {
    std::valarray<double> pos = {dist_pos_x(rd), dist_pos_y(rd)};
    std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
    f_flock.push_back(Boid{pos, vel});
    f_com.get_vel() += vel;
    f_com.get_pos() += pos;
  }
  f_com.get_vel() /= f_flock.size();
  f_com.get_pos() /= f_flock.size();
}

std::vector<Boid>::iterator Flock::begin() { return f_flock.begin(); }
std::vector<Boid>::iterator Flock::end() { return f_flock.end(); }

double Flock::size() const { return f_flock.size(); }

void Flock::push_back(Boid const& boid) { f_flock.push_back(boid); }

Boid& Flock::get_boid(int n) { return f_flock[n - 1]; }

Boid const& Flock::get_boid(int n) const { return f_flock[n - 1]; }

Parameters const& Flock::get_params() const { return f_params; }

Boid const& Flock::get_com() const { return f_com; }

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
std::vector<std::vector<Boid>> Flock::neighbours() {
  std::vector<std::vector<Boid>> neighbours;
  for (int i = 0; i < f_flock.size(); ++i) {
    for (auto j = i + 1; j < f_flock.size(); ++j) {
      if (boid_dist(f_flock[i], f_flock[j]) < f_d &&
          boid_dist(f_flock[i], f_flock[j]) > 0.) {
        if (is_visible(f_flock[i], f_flock[j], 120.)) {
          neighbours[i].push_back(f_flock[j]);
        } else if (is_visible(f_flock[j], f_flock[i], 120.)) {
          neighbours[j].push_back(f_flock[i]);
        }
      }
    }
  }
  return neighbours;
}

std::valarray<double> Flock::vel_correction(
    std::vector<std::vector<Boid>> neighbours, int i) {
  std::valarray<double> delta_vel = {0., 0.};
  if (neighbours[i].size() > 0) {
    auto n_minus = neighbours[i].size();
    std::valarray<double> local_com = {0., 0.};
    auto ev_b = this->get_boid(i);
    for (Boid bd : neighbours[i]) {
      // separation
      (boid_dist(bd, ev_b) < f_params.d_s)
          ? delta_vel -= f_params.s * (bd.get_pos() - ev_b.get_pos())
          : delta_vel;
      // alignment
      delta_vel += f_params.a * (bd.get_vel() - ev_b.get_vel()) / n_minus;

      local_com += bd.get_pos();
    }
    // cohesion
    delta_vel += f_params.c * (local_com / n_minus - ev_b.get_pos());
  }
  return delta_vel;
}

void Flock::update_flock_state(double const& delta_t) {
  auto neighbours = this->neighbours();
  for (int j = 0; j < f_flock.size(); ++j) {
    this->get_boid(j).update_state(delta_t, this->vel_correction(neighbours, j),
                                   0, f_params.d_s, f_params.s);
  }
  this->update_com();
}

Statistics Flock::get_statistics() {}