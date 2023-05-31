#include "flock.hpp"

double Flock::size() const { return f_flock.size(); }

void Flock::push_back(Boid boid) { f_flock.push_back(boid); }

Boid& Flock::get_boid(int n) { return f_flock[n - 1]; }

Boid const& Flock::get_boid(int n) const { return f_flock[n - 1]; }

void Flock::erase(int n) {
  auto it = f_flock.begin() + n - 1;
  f_flock.erase(it);
}

void friend separation(double a);
void friend alignement();
void friend cohesion();