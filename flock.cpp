#include "flock.hpp"

#include <algorithm>
#include <cassert>
#include <execution>
#include <numeric>
#include <random>

Flock::Flock(Parameters const& params, int const& bd_n, Boid const& com,
             double const& view_ang, std::valarray<double> const& space)
    : f_com{com}, f_params{params}, f_stats{}, f_flock{} {
  // Genera casualmente, secondo distribuzioni uniformi attorno al centro di
  // massa, lo stormo
  assert(bd_n >= 0);

  if (bd_n == 0) {
    // f_flock = std::vector<Boid>(0);
  } else {
    double rg_x;
    double rg_y;
    (com.get_pos()[0] < space[0] / 2.)
        ? rg_x = com.get_pos()[0] - 20.
        : rg_x = space[0] - com.get_pos()[0] - 20.;
    (com.get_pos()[1] < space[1] / 2.)
        ? rg_x = com.get_pos()[1] - 20.
        : rg_x = space[1] - com.get_pos()[1] - 20.;
    std::random_device rd;
    std::uniform_real_distribution<> dist_pos_x(com.get_pos()[0] - rg_x,
                                                com.get_pos()[0] + rg_x + 0.1);
    std::uniform_real_distribution<> dist_vel_x(com.get_vel()[0] - 150.,
                                                com.get_vel()[0] + 150.1);
    std::uniform_real_distribution<> dist_pos_y(com.get_pos()[1] - rg_y,
                                                com.get_pos()[1] + rg_y + 0.1);
    std::uniform_real_distribution<> dist_vel_y(com.get_vel()[1] - 150.,
                                                com.get_vel()[1] + 150.1);
    std::valarray<double> final_pos{0., 0.};
    std::valarray<double> final_vel{0., 0.};
    for (auto n = 0; n < bd_n - 1; ++n) {
      f_flock.push_back(Boid{{dist_pos_x(rd), dist_pos_y(rd)},
                             {dist_vel_x(rd), dist_vel_y(rd)},
                             view_ang,
                             space,
                             params.d_s,
                             params.s});
      final_pos += f_flock[n].get_pos();
      final_vel += f_flock[n].get_vel();
    }
    f_flock.push_back(Boid{bd_n * com.get_pos() - final_pos,
                           bd_n * com.get_vel() - final_vel, view_ang, space,
                           params.d_s, params.s});
  }

  sort();
}

Flock::Flock(Parameters const& params, int const& bd_n, double const& view_ang,
             std::valarray<double> const& space)
    : f_params{params}, f_stats{}, f_flock{} {
  // Genera casualmente, secondo distribuzioni uniformi, i boids
  assert(bd_n >= 0);
  std::random_device rd;
  int x_max = 2.5 * (space[0] - 40.) / params.d_s;
  int y_max = 2.5 * (space[1] - 40.) / params.d_s;

  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  f_com = Boid{{0., 0.}, {0., 0.}, 0., space, params.d_s, params.s};
  for (auto n = 0; n < bd_n; ++n) {
    std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (params.d_s) + 20.,
                                 dist_pos_y(rd) * 0.4 * (params.d_s) + 20.};
    std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
    f_flock.push_back(Boid{pos, vel, view_ang, space, params.d_s, params.s});
  }

  sort();

  auto compare_bd = [&](Boid& b1, Boid& b2) {
    return boid_dist(b1, b2) < 0.3 * params.d_s;
  };

  // verifica la prima volta che non ci siano boid coincidenti
  auto last = std::unique(f_flock.begin(), f_flock.end(), compare_bd);
  f_flock.erase(last, f_flock.end());

  // se ce n'erano, rigenera e ripulisce fino a quando non
  while (f_flock.size() < bd_n) {
    for (int i = 0; i < f_flock.size() - bd_n; ++i) {
      std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (params.d_s) + 20.,
                                   dist_pos_y(rd) * 0.4 * (params.d_s) + 20.};
      std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
      f_flock.push_back(Boid{pos, vel, view_ang, space, params.d_s, params.s});
    }
    sort();
    auto lt = std::unique(f_flock.begin(), f_flock.end(), compare_bd);
    f_flock.erase(lt, f_flock.end());
  }
  update_com();
}

std::vector<Boid>::iterator Flock::begin() { return f_flock.begin(); }
std::vector<Boid>::iterator Flock::end() { return f_flock.end(); }

double Flock::size() const { return f_flock.size(); }

void Flock::push_back(Boid const& boid) { f_flock.push_back(boid); }

Boid& Flock::get_boid(int n) { return f_flock[n - 1]; }

Boid const& Flock::get_boid(int n) const { return f_flock[n - 1]; }

Boid const& Flock::get_com() const { return f_com; }

Parameters const& Flock::get_params() const { return f_params; }

void Flock::set_parameter(int const& index, double const& value) {
  assert(index >= 0 && index < 5);
  switch (index) {
    case 0:
      f_params.d = value;
      break;
    case 1:
      f_params.d_s = value;
      for (auto it_s = f_flock.begin(); it_s != f_flock.end(); ++it_s) {
        it_s->set_par_ds(value);
      }
      break;
    case 2:
      f_params.s = value;
      for (auto it_s = f_flock.begin(); it_s != f_flock.end(); ++it_s) {
        it_s->set_par_s(value);
      }
      break;
    case 3:
      f_params.a = value;
      break;
    case 4:
      f_params.c = value;
      break;
    default:
      break;
  }
}

void Flock::set_space(double const& sx, double const& sy) {
  assert(sx > 0 && sy > 0);
  f_com.set_space(sx, sy);
  for (auto& bd : f_flock) {
    bd.set_space(sx, sy);
  }
}

void Flock::erase(std::vector<Boid>::iterator it) { f_flock.erase(it); }

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

std::vector<Boid> Flock::get_neighbours(std::vector<Boid>::iterator it) {
  std::vector<Boid> neighbours;
  assert(it >= f_flock.begin() && it < f_flock.end());
  auto et = it;
  for (; et != f_flock.end() &&
         std::abs(it->get_pos()[0] - et->get_pos()[0]) < f_params.d;
       ++et) {
    if (boid_dist(*et, *it) < f_params.d && boid_dist(*et, *it) > 0. &&
        is_visible(*et, *it) == true) {
      neighbours.push_back(*et);
    }
  }
  if (boid_dist(*et, *it) < f_params.d && boid_dist(*et, *it) > 0. &&
      is_visible(*et, *it) == true) {
    neighbours.push_back(*et);
  }
  et = it;
  for (; et != f_flock.begin() &&
         std::abs(it->get_pos()[0] - et->get_pos()[0]) < f_params.d;
       --et) {
    if (boid_dist(*et, *it) < f_params.d && boid_dist(*et, *it) > 0. &&
        is_visible(*et, *it) == true) {
      neighbours.push_back(*et);
    }
  }
  if (boid_dist(*et, *it) < f_params.d && boid_dist(*et, *it) > 0. &&
      is_visible(*et, *it) == true) {
    neighbours.push_back(*et);
  }

  return neighbours;
}

// get neighbours per boid esterno: magari potrà servire
std::vector<Boid> Flock::get_neighbours(double const& dist, Boid const& be) {
  std::vector<Boid> neighbours;
  auto ev_dist = [&](Boid bd_1) {
    return boid_dist(bd_1, be) < dist && boid_dist(bd_1, be) > 0. &&
           is_visible(bd_1, be);
  };
  std::copy_if(f_flock.begin(), f_flock.end(), std::back_inserter(neighbours),
               ev_dist);
  return neighbours;
}

// vel correction senza predatori
std::valarray<double> Flock::vel_correction(std::vector<Boid>::iterator it) {
  assert(it >= f_flock.begin() && it < f_flock.end());
  auto neighbours = get_neighbours(it);
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

// vel correction con un predatore
std::valarray<double> Flock::vel_correction(std::vector<Boid>::iterator it,
                                            Predator const& pred) {
  assert(it >= f_flock.begin() && it < f_flock.end());
  auto neighbours = get_neighbours(it);
  std::valarray<double> delta_vel = {0., 0.};
  // valuta subito se applicare separazione al predatore
  (boid_dist(pred, *it) < 5 * f_params.d_s)
      ? delta_vel -= 4 * f_params.s * (pred.get_pos() - it->get_pos())
      : delta_vel;
  // da qui in poi come caso no predatore:
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

// update senza predatori
void Flock::update_flock_state(double const& delta_t, bool const& brd_bhv) {
  std::vector<Boid> copy_flock = f_flock;
  auto it = f_flock.begin();
  std::for_each(copy_flock.begin(), copy_flock.end(), [&](Boid& bd) {
    bd.update_state(delta_t, this->vel_correction(it), brd_bhv);
    ++it;
  });
  f_flock = copy_flock;
  this->update_com();
  this->sort();
}

// update con un predatore
void Flock::update_flock_pred_state(double const& delta_t, bool const& brd_bhv,
                                    Predator& pred) {
  std::vector<Boid> copy_flock = f_flock;
  auto it = f_flock.begin();
  // boid su cui applica caccia = prede
  std::vector<Boid> preys;
  // boid mangiati (rimossi) = vittime
  std::vector<int> victims;
  auto boid_update = [&](Boid& bd) {
    if (is_visible(bd, pred)) {
      if (boid_dist(bd, pred) < 12.) {
        victims.push_back(it - f_flock.begin());
        ++it;
        return;
      }
      if (boid_dist(bd, pred) < pred.get_range() &&
          boid_dist(bd, pred) >= 12.) {
        preys.push_back(bd);
      }
    }

    (boid_dist(bd, pred) < 5 * f_params.d_s)
        ? bd.update_state(delta_t, this->vel_correction(it, pred), brd_bhv)
        : bd.update_state(delta_t, this->vel_correction(it), brd_bhv);
    ++it;
  };

  std::for_each(copy_flock.begin(), copy_flock.end(), boid_update);

  // rimuove le vittime in modo un po' intricato perché sono iteratori di
  // f_flock
  for (auto it_n : victims) {
    copy_flock.erase(copy_flock.begin() + it_n);
  }

  f_flock.resize(copy_flock.size());
  f_flock = copy_flock;
  update_com();
  sort();
  // aggiorna qui lo stato del predatore, passandogli le prede
  pred.update_state(delta_t, pred.predate(preys), brd_bhv);
}

// NB: perché vettore di vittime? Così non si invalidano iteratori nel loop!

void Flock::sort() {
  // Ordina i boids del vettore f_flock in ordine crescente secondo la
  // posizione in x. Se le posizioni in x sono uguali, allora ordina secondo
  // le posizioni in y

  auto is_less = [](Boid& bd1, Boid& bd2) {
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

    for (auto ut = it;
         ut < f_flock.end() &&
         std::abs(it->get_pos()[0] - ut->get_pos()[0]) < f_params.d;
         ++ut) {
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