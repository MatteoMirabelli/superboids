#include "multiflock.hpp"

#include <mutex>
#include <random>

Multiflock::Multiflock(std::vector<Parameters> const& params,
                       std::vector<int> const& sizes, double const& view_ang,
                       std::valarray<double> const& space) {
  assert(params.size() == sizes.size() && params.size() > 0);
  assert(view_ang >= 0. && view_ang <= 180.);
  assert(space[0] > 0. && space[1] > 0.);
  mf_params = params;
  mf_com = std::vector<Boid>(params.size());
  std::random_device rd;
  int x_max = 1;
  int y_max = 1;

  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  for (int index = 0; index < sizes.size(); index++) {
    mf_com[index] =
        Boid{{0., 0.},          {0., 0.}, view_ang, space, mf_params[index].d_s,
             mf_params[index].s};
    x_max = 2.5 * (space[0] - 40.) / mf_params[index].d_s;
    y_max = 2.5 * (space[1] - 40.) / mf_params[index].d_s;
    std::valarray<double> pos = {0., 0.};
    for (int i = 0; i < sizes[index];) {
      pos = {dist_pos_x(rd) * 0.4 * (mf_params[index].d_s) + 20.,
             dist_pos_y(rd) * 0.4 * (mf_params[index].d_s) + 20.};
      auto compare_bd = [this, &pos, &index](std::pair<Boid, int> const& b1) {
        return vec_norm(std::valarray<double>(b1.first.get_pos() - pos)) <
               0.3 * mf_params[index].d_s;
      };
      if (std::none_of(std::execution::par, mf_flock.begin(), mf_flock.end(),
                       compare_bd)) {
        std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
        mf_flock.push_back({Boid{pos, vel, view_ang, space,
                                 mf_params[index].d_s, mf_params[index].s},
                            index});
        i++;
      }
    }
  }

  sort();
  update_coms();
}

/*Multiflock::Multiflock(int, Parameters const&, int, double,
                       std::valarray<double> const&,
                       std::vector<Obstacle> const&);*/

void Multiflock::add_boid(int index) {
  assert(index >= 0 && index < mf_params.size());
  std::random_device rd;
  int x_max = 2.5 * (mf_com[index].get_space()[0] - 40.) / mf_params[index].d_s;
  int y_max = 2.5 * (mf_com[index].get_space()[1] - 40.) / mf_params[index].d_s;
  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  std::valarray<double> pos = {
      dist_pos_x(rd) * 0.4 * (mf_params[index].d_s) + 20.,
      dist_pos_y(rd) * 0.4 * (mf_params[index].d_s) + 20.};
  auto compare_bd = [&](std::pair<Boid, int> const& b1) {
    return vec_norm(std::valarray<double>(b1.first.get_pos() - pos)) <
           0.3 * mf_params[index].d_s;
  };
  while (std::any_of(std::execution::par, mf_flock.begin(), mf_flock.end(),
                     compare_bd)) {
    pos = {dist_pos_x(rd) * 0.4 * (mf_params[index].d_s) + 20.,
           dist_pos_y(rd) * 0.4 * (mf_params[index].d_s) + 20.};
  }
  std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
  mf_flock.push_back(
      {Boid{pos, vel, mf_com[index].get_view_angle(), mf_com[index].get_space(),
            mf_params[index].d_s, mf_params[index].s},
       index});
  sort();
}

double Multiflock::size() const { return mf_flock.size(); }
std::vector<std::pair<Boid, int>>::iterator Multiflock::begin() {
  return mf_flock.begin();
}
std::vector<std::pair<Boid, int>>::iterator Multiflock::end() {
  return mf_flock.end();
}
std::vector<Boid> const& Multiflock::get_coms() const { return mf_com; }
std::vector<Parameters> const& Multiflock::get_params() const {
  return mf_params;
}
Parameters const& Multiflock::get_params(int flock_index) const {
  assert(flock_index >= 0 && flock_index < mf_params.size());
  return mf_params[flock_index];
}
void Multiflock::set_parameter(int flock, int index, double value) {
  assert(flock >= 0 && flock < mf_params.size());
  assert(index >= 0 && index < 5 && value >= 0.);
  switch (index) {
    case 0:
      mf_params[flock].d = value;
      break;
    case 1:
      mf_params[flock].d_s = value;
      for (auto bd : mf_flock) {
        if (bd.second == flock) {
          bd.first.set_par_ds(value);
        }
      }
      break;
    case 2:
      mf_params[flock].s = value;
      for (auto bd : mf_flock) {
        if (bd.second == flock) {
          bd.first.set_par_s(value);
        }
      }
      break;
    case 3:
      mf_params[flock].a = value;
      break;
    case 4:
      mf_params[flock].c = value;
      break;
    default:
      break;
  }
}

void Multiflock::set_space(double const& sx, double const& sy) {
  assert(sx > 0. && sy > 0.);
  for (auto& b : mf_flock) {
    b.first.set_space(sx, sy);
  }
  for (auto& com : mf_com) {
    com.set_space(sx, sy);
  }
}

void Multiflock::update_coms() {
  std::vector<int> sizes(mf_com.size());
  for (auto& com : mf_com) {
    com.get_pos() = {0., 0.};
    com.get_vel() = {0., 0.};
  }
  for (auto& b : mf_flock) {
    mf_com[b.second].get_pos() += b.first.get_pos();
    mf_com[b.second].get_vel() += b.first.get_vel();
    ++sizes[b.second];
  }
  for (int i = 0; i < mf_com.size(); ++i) {
    mf_com[i].get_pos() /= sizes[i];
    mf_com[i].get_vel() /= sizes[i];
  }
}

std::valarray<double> Multiflock::vel_correction(
    std::vector<std::pair<Boid, int>> const& copy_mf_flock,
    std::vector<std::pair<Boid, int>>::iterator it) {
  assert(it >= copy_mf_flock.begin() && it < copy_mf_flock.end());
  std::valarray<double> separation = {0., 0.};
  std::valarray<double> alignment = {0., 0.};
  std::valarray<double> local_com = {0., 0.};
  int neighbours = 0;
  auto dist_d = mf_params[(*it).second].d;
  auto dist_s = mf_params[(*it).second].d_s;
  auto et = it;
  for (; et != copy_mf_flock.end() &&
         std::abs((*it).first.get_pos()[0] - (*et).first.get_pos()[0]) < dist_d;
       ++et) {
    if (boid_dist((*et).first, (*it).first) < dist_d &&
        boid_dist((*et).first, (*it).first) > 0. &&
        is_visible((*et).first, (*it).first)) {
      (boid_dist((*et).first, (*it).first) < dist_s)
          ? separation -= mf_params[(*it).second].s *
                          ((*et).first.get_pos() - (*it).first.get_pos())
          : separation;
      if ((*et).second == (*it).second) {
        ++neighbours;
        local_com += (*et).first.get_pos();
        alignment += mf_params[(*it).second].a *
                     ((*et).first.get_vel() - (*it).first.get_vel());
      }
    }
  }
  et = it;
  for (; et != copy_mf_flock.begin() &&
         std::abs((*it).first.get_pos()[0] - (*et).first.get_pos()[0]) < dist_d;
       --et) {
    if (boid_dist((*et).first, (*it).first) < dist_d &&
        boid_dist((*et).first, (*it).first) > 0. &&
        is_visible((*et).first, (*it).first)) {
      (boid_dist((*et).first, (*it).first) < dist_s)
          ? separation -= mf_params[(*it).second].s *
                          ((*et).first.get_pos() - (*it).first.get_pos())
          : separation;
      if ((*et).second == (*it).second) {
        ++neighbours;
        local_com += (*et).first.get_pos();
        alignment += mf_params[(*it).second].a *
                     ((*et).first.get_vel() - (*it).first.get_vel());
      }
    }
  }
  return separation + alignment / neighbours +
         mf_params[(*it).second].c *
             (local_com / neighbours - (*it).first.get_pos());
}

std::valarray<double> Multiflock::avoid_pred(std::pair<Boid, int> const& bd_idx,
                                             Predator const& pred) {
  std::valarray<double> delta_vel = {0., 0.};
  // valuta subito se applicare separazione al predatore
  (boid_dist(pred, bd_idx.first) < mf_params[bd_idx.second].d)
      ? delta_vel -= 1.5 * mf_params[bd_idx.second].s *
                     (pred.get_pos() - bd_idx.first.get_pos())
      : delta_vel;
  return delta_vel;
}

void Multiflock::update_global_state(double delta_t, bool brd_bhv,
                                     std::vector<Predator>& preds,
                                     std::vector<Obstacle> const& obs) {
  // boid su cui applica caccia = prede
  std::vector<std::pair<Boid, int>> preys;

  // il mutex è necessario per evitare problemi in race condition
  // ovvero prevenire l'accesso simultaneo da parte dei thread paralleli
  // alle medesime risorse (in questo caso, il vettore delle prede)
  std::mutex mtx;

  // rimuove le vittime
  auto bd_eaten = [this, &preds](std::pair<Boid, int> const& bd) {
    auto above = [this, &bd](Predator const& pred) -> bool {
      return boid_dist(pred, bd.first) < 0.3 * mf_params[bd.second].d_s;
    };
    return std::any_of(std::execution::par_unseq, preds.begin(), preds.end(),
                       above);
  };

  // rimuove le prede mangiate
  auto last = std::remove_if(std::execution::par, mf_flock.begin(),
                             mf_flock.end(), bd_eaten);
  mf_flock.erase(last, mf_flock.end());
  // sort();
  //  duplica il vettore per poter aggiornare tutti i boid in parallelo
  //  in base allo stato al frame precedente
  std::vector<std::pair<Boid, int>> copy_flock = mf_flock;

  // utilizza il vettore di indici per accedere ai corrispondenti boid di
  // f_flock ed in tal modo poter applicare la ricerca dei vicini tramite
  // iteratore
  std::vector<int> indexes;

  for (int i = 0; i < mf_flock.size(); ++i) {
    indexes.push_back(i);
  }

  // lambda per aggiornare lo stato
  auto boid_update =
      [&mtx, &preds, &preys, this, delta_t, brd_bhv, &copy_flock, &obs](
          int const& index,
          std::pair<Boid, int> const& bd) -> std::pair<Boid, int> {
    // aggiorna lo stato del boid con o senza percezione predatore
    std::valarray<double> corr = {0., 0.};
    for (int idx = 0; idx < preds.size(); ++idx) {
      corr += avoid_pred(bd, preds[idx]);
      if (is_visible(bd.first, preds[idx]) &&
          boid_dist(preds[idx], bd.first) < preds[idx].get_range()) {
        // blocca il mutex prima della modifica di preys
        std::lock_guard<std::mutex> lck(mtx);
        preys.push_back({bd.first, idx});
      }
    }
    mf_flock[index].first.update_state(
        delta_t,
        vel_correction(copy_flock, copy_flock.begin() + index) +
            bd.first.avoid_obs(obs) + corr,
        brd_bhv);
    return mf_flock[index];
  };

  // modo più efficiente che sono riuscito a implementare: in questo modo
  // non bisogna fare copie ma viene direttamente scritto su f_flock
  std::transform(std::execution::par, indexes.begin(), indexes.end(),
                 copy_flock.begin(), mf_flock.begin(), boid_update);

  update_coms();
  sort();
  // aggiorna stato del vettore di predatori
  update_predators_state(preds, delta_t, brd_bhv, preys, obs);
}

void Multiflock::sort() {
  auto is_less = [](std::pair<Boid, int> const& bd1,
                    std::pair<Boid, int> const& bd2) {
    if (bd1.first.get_pos()[0] != bd2.first.get_pos()[0]) {
      return bd1.first.get_pos()[0] < bd2.first.get_pos()[0];
    } else {
      return bd1.first.get_pos()[1] < bd2.first.get_pos()[1];
    }
  };

  std::sort(std::execution::par, mf_flock.begin(), mf_flock.end(), is_less);
}
// void Multiflock::update_stats(){};  // TODO
std::vector<Statistics> const& Multiflock::get_stats() const {
  return mf_stats;
}