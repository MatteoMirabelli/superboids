#include "flock.hpp"

#include <algorithm>
#include <numeric>
#include <random>

Flock::Flock(double const& d, std::valarray<double> const& params,
             int const& bd_n, Boid const& com)
    : f_d{d}, f_com{com}, f_flock(bd_n) {
  assert(params.size() == 4);
  f_params = params;
  std::random_device rd;
  std::uniform_real_distribution<std::valarray<double>> dist_pos(
      com.get_pos() - std::valarray<double>{10., 10.},
      com.get_pos() + std::valarray<double>{10., 10.});
  std::uniform_real_distribution<std::valarray<double>> dist_vel(
      com.get_vel() - std::valarray<double>{10., 10.},
      com.get_vel() + std::valarray<double>{10., 10.});
  for (auto it = f_flock.begin(); it != --f_flock.end(); ++it) {
    (--f_flock.end())->get_pos() -= it->get_pos();
    (--f_flock.end() - 1)->get_vel() -= it->get_vel();
    it->get_pos() = dist_pos(rd);
    it->get_vel() = dist_vel(rd);
  }
  (--f_flock.end())->get_pos() += bd_n * (com.get_pos());
  (--f_flock.end())->get_vel() += bd_n * (com.get_vel());
}

auto Flock::begin() { return f_flock.begin(); }
auto Flock::end() { return f_flock.end(); }
auto Flock::begin() const { return f_flock.begin(); }
auto Flock::end() const { return f_flock.end(); }

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
  auto update_lambda = [&](Boid b, std::valarray<double> p_0) {
    f_com.get_vel() += b.get_vel();
    return p_0 + b.get_pos();
  };
  f_com.get_pos() =
      std::accumulate(f_flock.begin(), f_flock.end(),
                      std::valarray<double>{0., 0.}, update_lambda) /
      (this->size());
  f_com.get_vel() /= Flock::size();
};

// implementato il get neighbour e la correzione con iteratore; alternativa con
// i numeri per non avere problemi ma tanto la chiamata è fatta dentro update
// state, quindi non c'è il rischio di passare iteratori di altri flock!
std::vector<Boid> Flock::get_neighbours(std::vector<Boid>::iterator it) {
  std::vector<Boid> neighbours;
  // auto bd_2 = f_flock[n - 1];
  auto ev_dist = [&](Boid const& bd_1) {
    return boid_dist(bd_1, *it) < f_d && boid_dist(bd_1, *it) > 0;
  };
  std::copy_if(f_flock.begin(), f_flock.end(), std::back_inserter(neighbours),
               ev_dist);
  return neighbours;
}

std::valarray<double> Flock::vel_correction(std::vector<Boid>::iterator it) {
  auto neighbours = this->get_neighbours(it);
  std::valarray<double> delta_vel = {0., 0.};
  if (neighbours.size() > 0) {
    auto compute = [&](Boid const& bd) {
      std::valarray<double> corr = {0., 0.};
      (boid_dist(bd, *it) < f_params[0])
          ? corr -= f_params[1] * (bd.get_pos() - it->get_pos())
          : corr += {0., 0.};
      corr += f_params[2] * (1 / (neighbours.size() - 1)) *
              (bd.get_vel() - it->get_vel());
      corr +=
          f_params[3] *
          (std::accumulate(neighbours.begin(), neighbours.end(),
                           std::valarray<double>{0., 0.},
                           [&](std::valarray<double> p_0, Boid const& bd_1) {
                             return p_0 + bd_1.get_pos();
                           }) /
               neighbours.size() -
           it->get_pos());
    };
  }
}

void Flock::update_flock_state(double const& delta_t) {
  for (auto it = f_flock.begin(); it != f_flock.end(); ++it) {
    it->update_state(delta_t, this->vel_correction(it));
  }
}

/*
std::valarray<double> separation(double s) {}
void alignment();
void cohesion();
*/