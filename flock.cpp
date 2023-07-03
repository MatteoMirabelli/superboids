#include "flock.hpp"

#include <algorithm>
#include <cassert>
#include <execution>
#include <mutex>
#include <numeric>
#include <random>
#include <utility>

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
        ? rg_y = com.get_pos()[1] - 20.
        : rg_y = space[1] - com.get_pos()[1] - 20.;
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

  auto generator = [&]() -> Boid {
    std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (params.d_s) + 20.,
                                 dist_pos_y(rd) * 0.4 * (params.d_s) + 20.};
    std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
    return Boid{pos, vel, view_ang, space, params.d_s, params.s};
  };

  std::generate_n(std::execution::par, std::back_insert_iterator(f_flock), bd_n,
                  generator);

  sort();

  auto compare_bd = [&](Boid& b1, Boid& b2) {
    return boid_dist(b1, b2) < 0.3 * params.d_s;
  };

  // verifica la prima volta che non ci siano boid coincidenti
  auto last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                          compare_bd);
  // f_flock.erase(last, f_flock.end());

  // se ce n'erano, rigenera e così fino a quando unique restituisce
  // end(), ovvero non ci sono duplicati
  while (last != f_flock.end()) {
    std::generate(std::execution::par, last, f_flock.end(), generator);
    sort();
    last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                       compare_bd);
  }

  update_com();
}

Flock::Flock(Parameters const& params, int bd_n, double view_ang,
             std::valarray<double> const& space,
             std::vector<Obstacle> const& obs)
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

  auto generator = [&]() -> Boid {
    std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (params.d_s) + 20.,
                                 dist_pos_y(rd) * 0.4 * (params.d_s) + 20.};
    std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
    return Boid{pos, vel, view_ang, space, params.d_s, params.s};
  };

  std::generate_n(std::execution::par, std::back_insert_iterator(f_flock), bd_n,
                  generator);

  sort();

  auto compare_bd = [&](Boid& b1, Boid& b2) {
    return boid_dist(b1, b2) < 0.3 * params.d_s;
  };

  // verifica la prima volta che non ci siano boid coincidenti
  auto last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                          compare_bd);

  // se ce n'erano, rigenera e così fino a quando unique restituisce
  // end(), ovvero non ci sono duplicati
  while (last != f_flock.end()) {
    std::generate(std::execution::par, last, f_flock.end(), generator);
    sort();
    last = std::unique(std::execution::par, f_flock.begin(), f_flock.end(),
                       compare_bd);
  }

  auto avoid_obs = [&](Boid const& bx) {
    auto overlap = [&](Obstacle const& obs) {
      std::valarray<double> dist = bx.get_pos() - obs.get_pos();
      return vec_norm(dist) < obs.get_size() + 0.6 * f_params.d_s;
    };
    return std::any_of(std::execution::par, obs.begin(), obs.end(), overlap);
  };
  last = std::remove_if(std::execution::par, f_flock.begin(), f_flock.end(),
                        avoid_obs);
  while (last != f_flock.end()) {
    std::generate(std::execution::par, last, f_flock.end(), generator);
    last = std::remove_if(std::execution::par, f_flock.begin(), f_flock.end(),
                          avoid_obs);
  }
  sort();
  update_com();
}

void Flock::add_boid() {
  std::random_device rd;
  int x_max = 2.5 * (f_flock[0].get_space()[0] - 40.) / f_params.d_s;
  int y_max = 2.5 * (f_flock[0].get_space()[1] - 40.) / f_params.d_s;

  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (f_params.d_s) + 20.,
                               dist_pos_y(rd) * 0.4 * (f_params.d_s) + 20.};

  auto find_clone = [pos](Boid& b1) {
    return b1.get_pos()[0] == pos[0] && b1.get_pos()[1] == pos[1];
  };
  auto clone = std::find_if(std::execution::par, f_flock.begin(), f_flock.end(),
                            find_clone);
  while (clone != f_flock.end()) {
    pos = {dist_pos_x(rd) * 0.4 * (f_params.d_s) + 20.,
           dist_pos_y(rd) * 0.4 * (f_params.d_s) + 20.};
    clone = std::find_if(std::execution::unseq, f_flock.begin(), f_flock.end(),
                         find_clone);
  }
  std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
  f_flock.push_back(Boid{pos, vel, f_flock[0].get_view_angle(),
                         f_flock[0].get_space(), f_params.d_s, f_params.s});
  sort();
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
  /*std::vector<Boid> neighbours;
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
  et = it;
  for (; et != f_flock.begin() &&
         std::abs(it->get_pos()[0] - et->get_pos()[0]) < f_params.d;
       --et) {
    if (boid_dist(*et, *it) < f_params.d && boid_dist(*et, *it) > 0. &&
        is_visible(*et, *it) == true) {
      neighbours.push_back(*et);
    }
  }
  return neighbours;*/
  return get_vector_neighbours(f_flock, it, f_params.d);
}

// get neighbours per boid esterno: magari potrà servire
std::vector<Boid> Flock::get_neighbours(double const& dist, Boid const& be) {
  std::vector<Boid> neighbours;
  auto ev_dist = [&](Boid bd_1) {
    return boid_dist(bd_1, be) < dist && boid_dist(bd_1, be) > 0. &&
           is_visible(bd_1, be);
  };
  std::copy_if(std::execution::par, f_flock.begin(), f_flock.end(),
               std::back_inserter(neighbours), ev_dist);
  return neighbours;
}

// vel correction senza predatori
std::valarray<double> Flock::vel_correction(std::vector<Boid>::iterator it) {
  assert(it >= f_flock.begin() && it < f_flock.end());
  auto neighbours = get_neighbours(it);
  std::valarray<double> delta_vel = {0., 0.};
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size();
    // std::mutex mtx;
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

// vel correction senza predatore e con copy flock: per l'update state
std::valarray<double> Flock::vel_correction(std::vector<Boid> const& copy_flock,
                                            std::vector<Boid>::iterator it) {
  assert(it >= copy_flock.begin() && it < copy_flock.end() &&
         copy_flock.size() == f_flock.size());
  auto neighbours = get_vector_neighbours(copy_flock, it, f_params.d);
  std::valarray<double> delta_vel = {0., 0.};
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size();
    // std::mutex mtx;
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
  (boid_dist(pred, *it) < f_params.d)
      ? delta_vel -= f_params.s * (pred.get_pos() - it->get_pos())
      : delta_vel;
  // da qui in poi come caso no predatore:
  if (neighbours.size() > 0) {
    auto n_minus = neighbours.size();
    std::valarray<double> local_com = {0., 0.};
    // std::mutex mtx;
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

// per evitare ogni predatore. Più elegante anche per il vel correction.
// anzi, lo rende quasi inutile
std::valarray<double> Flock::avoid_pred(Boid const& bd, Predator const& pred) {
  std::valarray<double> delta_vel = {0., 0.};
  // valuta subito se applicare separazione al predatore
  (boid_dist(pred, bd) < f_params.d)
      ? delta_vel -= f_params.s * (pred.get_pos() - bd.get_pos())
      : delta_vel;
  return delta_vel;
}

// vel correction con un predatore e copy flock: per l'update state
std::valarray<double> Flock::vel_correction(std::vector<Boid> const& copy_flock,
                                            std::vector<Boid>::iterator it,
                                            Predator const& pred) {
  assert(it >= copy_flock.begin() && it < copy_flock.end() &&
         copy_flock.size() == f_flock.size());
  return vel_correction(copy_flock, it) + avoid_pred(*it, pred);

  // da qui in poi si era reimplementato come senza pred
}

// NOTA SULL'IMPLEMENTAZIONE CON COPY FLOCK
// La ratio del tutto è che quando aggiorniamo lo stato conviene usare copy
// flock per tenere lo stato all'istante precedente e da lì stampare su f_flock.
// Viceversa se usassimo f_flock per lo stato precedente alla fine dovremmo fare
// la copia di copy_flock su f_flock, che significherebbe di nuovo scorrere due
// vettori. L'uso degli indici nell'update state è per questo e per non perdere
// la coerenza quando si parallelizza. Ragionevolmente per i nuovi vel
// correction passo i parametri dello stormo (in quanto solo per esso hanno
// senso) mentre il get vector neighbours è più ampio. Anzi, sarei quasi tentato
// di metterlo nel file Boid.

// update senza predatori e senza ostacoli (da sistemare!)
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
// tentativo di implementazione alternativa per parallelizzare in modo umano:

void Flock::update_global_state(double const& delta_t, bool const& brd_bhv,
                                Predator& pred) {
  // boid su cui applica caccia = prede
  std::vector<Boid> preys;

  // il mutex è necessario per evitare problemi in race condition
  // ovvero prevenire l'accesso simultaneo da parte dei thread paralleli
  // alle medesime risorse (in questo caso, il vettore delle prede)
  std::mutex mtx;

  // rimuove le vittime
  auto bd_eaten = [&](Boid const& bd) {
    if (boid_dist(bd, pred) <= 0.4 * f_params.d_s)
      return true;
    else
      return false;
  };

  // rimuove le prede mangiate
  // mi sono accorto dopo che nella precedente implementazione
  // i morti erano ancora consideranti nel calcolo di allineamento etc:
  // aveva senso? non è difficile da reimplementare, ma mi sorge il dubbio
  f_flock.erase(std::remove_if(std::execution::par, f_flock.begin(),
                               f_flock.end(), bd_eaten),
                f_flock.end());

  // duplica il vettore per poter aggiornare tutti i boid in parallelo
  // in base allo stato al frame precedente
  std::vector<Boid> copy_flock = f_flock;

  // utilizza il vettore di indici per accedere ai corrispondenti boid di
  // f_flock ed in tal modo poter applicare la ricerca dei vicini tramite
  // iteratore
  std::vector<unsigned int> indexes;

  for (unsigned int i = 0; i < f_flock.size(); ++i) {
    indexes.push_back(i);
  }

  // lambda per aggiornare lo stato
  auto boid_update = [&](unsigned int const& index, Boid const& bd) -> Boid {
    if (is_visible(bd, pred)) {
      if (boid_dist(bd, pred) < pred.get_range() &&
          boid_dist(bd, pred) > 0.4 * f_params.d_s) {
        // blocca il mutex prima della modifica di preys
        std::lock_guard<std::mutex> lck(mtx);
        preys.push_back(bd);
      }
    }

    // aggiorna lo stato del boid
    // (la percezione del predatore è considerata in vel_correction!!)
    f_flock[index].update_state(
        delta_t, vel_correction(copy_flock, copy_flock.begin() + index, pred),
        brd_bhv);
    return f_flock[index];
  };

  // modo più efficiente che sono riuscito a implementare: in questo modo non
  // bisogna fare copie ma viene direttamente scritto su f_flock
  std::transform(std::execution::par, indexes.begin(), indexes.end(),
                 copy_flock.begin(), f_flock.begin(), boid_update);

  update_com();
  sort();
  // aggiorna qui lo stato del predatore, passandogli le prede
  pred.update_state(delta_t, pred.predate(preys), brd_bhv);
}

// update globale con un predatore e ostacoli
void Flock::update_global_state(double const& delta_t, bool const& brd_bhv,
                                Predator& pred,
                                std::vector<Obstacle> const& obs) {
  if (obs.size() == 0) {
    update_global_state(delta_t, brd_bhv, pred);
    return;
  }
  // boid su cui applica caccia = prede
  std::vector<Boid> preys;

  // il mutex è necessario per evitare problemi in race condition
  // ovvero prevenire l'accesso simultaneo da parte dei thread paralleli
  // alle medesime risorse (in questo caso, il vettore delle prede)
  std::mutex mtx;

  // rimuove le vittime
  auto bd_eaten = [&](Boid const& bd) {
    if (boid_dist(bd, pred) <= 0.3 * f_params.d_s)
      return true;
    else
      return false;
  };

  // rimuove le prede mangiate
  // mi sono accorto dopo che nella precedente implementazione
  // i morti erano ancora consideranti nel calcolo di allineamento etc:
  // aveva senso? non è difficile da reimplementare, ma mi sorge il dubbio
  auto last = std::remove_if(std::execution::par, f_flock.begin(),
                             f_flock.end(), bd_eaten);
  f_flock.erase(last, f_flock.end());

  // duplica il vettore per poter aggiornare tutti i boid in parallelo
  // in base allo stato al frame precedente
  std::vector<Boid> copy_flock = f_flock;

  // utilizza il vettore di indici per accedere ai corrispondenti boid di
  // f_flock ed in tal modo poter applicare la ricerca dei vicini tramite
  // iteratore
  std::vector<unsigned int> indexes;

  for (unsigned int i = 0; i < f_flock.size(); ++i) {
    indexes.push_back(i);
  }

  // lambda per aggiornare lo stato
  auto boid_update = [&](unsigned int const& index, Boid const& bd) -> Boid {
    if (is_visible(bd, pred) && boid_dist(bd, pred) < pred.get_range() &&
        boid_dist(bd, pred) > 0.3 * f_params.d_s) {
      // blocca il mutex prima della modifica di preys
      std::lock_guard<std::mutex> lck(mtx);
      preys.push_back(bd);
    }
    // aggiorna lo stato del boid
    f_flock[index].update_state(
        delta_t,
        vel_correction(copy_flock, copy_flock.begin() + index, pred) +
            bd.avoid_obs(obs),
        brd_bhv);
    return f_flock[index];
  };

  // modo più efficiente che sono riuscito a implementare: in questo modo non
  // bisogna fare copie ma viene direttamente scritto su f_flock
  std::transform(std::execution::par, indexes.begin(), indexes.end(),
                 copy_flock.begin(), f_flock.begin(), boid_update);

  update_com();
  sort();
  // aggiorna qui lo stato del predatore, passandogli le prede
  pred.update_state(delta_t, pred.predate(preys) + pred.avoid_obs(obs),
                    brd_bhv);
  return;
}

// update globale con più predatori e ostacoli:
void Flock::update_global_state(double delta_t, bool brd_bhv,
                                std::vector<Predator>& preds,
                                std::vector<Obstacle> const& obs) {
  // boid su cui applica caccia = prede
  std::vector<std::pair<Boid, unsigned int>> preys;

  // il mutex è necessario per evitare problemi in race condition
  // ovvero prevenire l'accesso simultaneo da parte dei thread paralleli
  // alle medesime risorse (in questo caso, il vettore delle prede)
  std::mutex mtx;

  // rimuove le vittime
  auto bd_eaten = [&](Boid const& bd) {
    auto above = [&](Predator const& pred) {
      return boid_dist(bd, pred) <= 0.3 * f_params.d_s;
    };
    return std::any_of(std::execution::par, preds.begin(), preds.end(), above);
  };

  // rimuove le prede mangiate
  auto last = std::remove_if(std::execution::par, f_flock.begin(),
                             f_flock.end(), bd_eaten);
  f_flock.erase(last, f_flock.end());

  // duplica il vettore per poter aggiornare tutti i boid in parallelo
  // in base allo stato al frame precedente
  std::vector<Boid> copy_flock = f_flock;

  // utilizza il vettore di indici per accedere ai corrispondenti boid di
  // f_flock ed in tal modo poter applicare la ricerca dei vicini tramite
  // iteratore
  std::vector<unsigned int> indexes;

  for (unsigned int i = 0; i < f_flock.size(); ++i) {
    indexes.push_back(i);
  }

  // lambda per aggiornare lo stato
  auto boid_update = [&](unsigned int const& index, Boid const& bd) -> Boid {
    // aggiorna lo stato del boid con o senza percezione predatore
    std::valarray<double> corr = {0., 0.};
    for (unsigned int idx = 0; idx < preds.size(); ++idx) {
      corr += avoid_pred(bd, preds[idx]);
      if (is_visible(bd, preds[idx]) &&
          boid_dist(bd, preds[idx]) < preds[idx].get_range() &&
          boid_dist(bd, preds[idx]) > 0.3 * f_params.d_s) {
        // blocca il mutex prima della modifica di preys
        std::lock_guard<std::mutex> lck(mtx);
        preys.push_back({bd, idx});
      }
    }
    f_flock[index].update_state(
        delta_t,
        vel_correction(copy_flock, copy_flock.begin() + index) +
            bd.avoid_obs(obs) + corr,
        brd_bhv);
    return f_flock[index];
  };

  // modo più efficiente che sono riuscito a implementare: in questo modo non
  // bisogna fare copie ma viene direttamente scritto su f_flock
  std::transform(std::execution::par, indexes.begin(), indexes.end(),
                 copy_flock.begin(), f_flock.begin(), boid_update);

  update_com();
  sort();
  // aggiorna stato del vettore di predatori
  update_predators_state(preds, delta_t, brd_bhv, preys, obs);
}

void Flock::sort() {
  // Ordina i boids del vettore f_flock in ordine crescente secondo la
  // posizione in x. Se le posizioni in x sono uguali, allora ordina secondo
  // le posizioni in y

  auto is_less = [](Boid const& bd1, Boid const& bd2) {
    if (bd1.get_pos()[0] != bd2.get_pos()[0]) {
      return bd1.get_pos()[0] < bd2.get_pos()[0];
    } else {
      return bd1.get_pos()[1] < bd2.get_pos()[1];
    }
  };

  std::sort(std::execution::par, f_flock.begin(), f_flock.end(), is_less);
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
