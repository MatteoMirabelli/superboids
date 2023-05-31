#include "flock.hpp"

#include <numeric>

double Flock::size() const { return f_flock.size(); }

void Flock::push_back(Boid const& boid) { f_flock.push_back(boid); }

Boid& Flock::get_boid(int n) { return f_flock[n - 1]; }

Boid const& Flock::get_boid(int n) const { return f_flock[n - 1]; }

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

void friend Flock::separation(double a);
void friend Flock::alignement();
void friend Flock::cohesion();
