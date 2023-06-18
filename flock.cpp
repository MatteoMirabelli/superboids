#include "flock.hpp"

#include <algorithm>
#include <execution>
#include <numeric>
#include <random>

Flock::Flock(Parameters const& params, int const& bd_n, Boid const& com)
    : f_com{com}, f_params{params}, f_stats{}, f_flock{} {
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

  this->sort();
}

Flock::Flock(Parameters const& params, int const& bd_n)
    : f_params{params}, f_stats{}, f_flock{} {
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

  this->sort();
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
std::vector<Boid> Flock::get_neighbours(std::vector<Boid>::iterator it) {
  std::vector<Boid> neighbours;

  /* auto ev_dist = [&](std::vector<Boid>::iterator ut) {
     return boid_dist(*ut, *it) < f_params.d && boid_dist(*ut, *it) > 0. &&
            is_visible(*ut, *it, 120.);
   }; */

  // Versione Matteo (Con la condizione che il vettore sia ordinato per distanza
  // dall'origine, controllo solo i boid vicini a *it)

  /* if (it == f_flock.end()) {
    return neighbours;
  } else {
    auto ut = it;
    auto et = it;

    for (; ut >= f_flock.begin(); --ut) {
      if (boid_dist(*ut, *it) < f_params.d && boid_dist(*ut, *it) > 0. &&
          is_visible(*ut, *it, 120.) == true) {
        neighbours.push_back(*ut);
      } else {
        break;
      }
    }

    for (; et < f_flock.end(); ++et) {
      if (boid_dist(*et, *it) < f_params.d && boid_dist(*et, *it) > 0. &&
          is_visible(*et, *it, 120.) == true) {
        neighbours.push_back(*et);
      } else {
        break;
      }
    }

    return neighbours;
  } */

  // Versione Alberto (copy_if)

  if (it != this->end()) {
    auto ev_dist = [&](Boid bd_1) {
      return boid_dist(bd_1, *it) < f_params.d && boid_dist(bd_1, *it) > 0. &&
             is_visible(bd_1, *it, 120.);
    };
    std::copy_if(f_flock.begin(), f_flock.end(), std::back_inserter(neighbours),
                 ev_dist);
    return neighbours;
  } else {
    neighbours = std::vector<Boid>(0);
    return neighbours;
  }

  // Versione Andrea (For Loop su tutti il flock)

  /* double visibility_angle = 120.0;

  for (const Boid& bd_1 : f_flock) {
    double dist = boid_dist(bd_1, *it);
    if (dist > 0. && dist < this->get_params().d &&
        is_visible(bd_1, *it, visibility_angle)) {
      neighbours.push_back(bd_1);
    }
  }

  return neighbours; */
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
  std::for_each(std::execution::par_unseq, copy_flock.begin(), copy_flock.end(),
                [&](Boid& bd) {
                  bd.update_state(delta_t, this->vel_correction(it), 1,
                                  f_params.d_s, f_params.s);
                  ++it;
                });
  f_flock = copy_flock;
  this->update_com();
  this->sort();
}

void Flock::sort() {
  // Ordina i boids del vettore f_flock in ordine crescente secondo la
  // posizione in x. Se le posizioni in x sono uguali, allora ordina secondo
  // le posizioni in y

  auto is_less = [&](Boid& bd1, Boid& bd2) {
    if (bd1.get_pos()[0] != bd2.get_pos()[0]) {
      return bd1.get_pos()[0] < bd2.get_pos()[0];
    } else {
      return bd1.get_pos()[1] < bd2.get_pos()[1];
    }
  };

  std::sort(f_flock.begin(), f_flock.end(), is_less);
}

void Flock::update_stats() {
  double mean_dist{0};
  double square_mean_dist{0};
  double mean_vel{0};
  double square_mean_vel{0};
  int number_of_couples{0};

  for (auto it = f_flock.begin(); it < f_flock.end(); ++it) {
    mean_vel += vec_norm(it->get_vel());
    square_mean_vel += (vec_norm(it->get_vel()) * vec_norm(it->get_vel()));

    for (auto ut = it; ut < f_flock.end(); ++ut) {
      if (boid_dist(*it, *ut) <= f_params.d && boid_dist(*it, *ut) > 0) {
        mean_dist += boid_dist(*it, *ut);
        square_mean_dist += (boid_dist(*it, *ut) * boid_dist(*it, *ut));
        ++number_of_couples;
      }
    }
  }

  if (this->size() == 0) {
    f_stats.av_dist = 0.;
    f_stats.dist_RMS = 0.;
    f_stats.av_dist = 0.;
    f_stats.vel_RMS = 0.;
  } else {
    mean_vel /= this->size();
    square_mean_vel /= this->size();
    double vel_RMS = sqrt(square_mean_vel - mean_vel * mean_vel);

    f_stats.av_vel = mean_vel;
    f_stats.vel_RMS = vel_RMS;

    if (number_of_couples == 0) {
      f_stats.av_dist = 0;
      f_stats.dist_RMS = 0;
    } else {
      mean_dist /= number_of_couples;
      square_mean_dist /= number_of_couples;
      double dist_RMS = sqrt(square_mean_dist - mean_dist * mean_dist);

      f_stats.av_dist = mean_dist;
      f_stats.dist_RMS = dist_RMS;
    }
  }
}

Statistics const& Flock::get_stats() const { return f_stats; }
