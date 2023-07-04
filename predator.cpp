#include "predator.hpp"

#include <algorithm>
#include <random>

char Predator::type() const { return 'p'; }

Predator::Predator(std::valarray<double> const& pos,
                   std::valarray<double> const& vel, double const& view_ang,
                   double const& param_d_s, double const& param_s,
                   std::valarray<double> const& space, double const& range,
                   double const& hunger)
    : Boid(pos, vel, view_ang, space, param_d_s, param_s),
      p_range(range),
      p_hunger(hunger) {}

Predator::Predator(double const& x, double const& y, double const& vx,
                   double const& vy, double const& view_ang,
                   double const& param_d_s, double const& param_s,
                   double const& sx, double const& sy, double const& range,
                   double const& hunger)
    : Boid(x, y, vx, vy, view_ang, sx, sy, param_d_s, param_s),
      p_range(range),
      p_hunger(hunger) {}

double Predator::get_angle() const { return Boid::get_angle(); }

double Predator::get_range() const { return p_range; }

double Predator::get_hunger() const { return p_hunger; }

// vel correction per predatori: calcola contributo coesione verso com prede
std::valarray<double> Predator::predate(std::vector<Boid>& preys) {
  std::valarray<double> prey_com_pos(2);

  if (preys.size() > 0) {
    auto nearest = [&](Boid const& b1, Boid const& b2) {
      return boid_dist(b1, *this) < boid_dist(b2, *this);
    };

    std::sort(preys.begin(), preys.end(), nearest);
    for (auto const& prey : preys) {
      prey_com_pos += prey.get_pos();
    }

    prey_com_pos /= preys.size();

    return p_hunger * (prey_com_pos - get_pos()) +
           2 * p_hunger * (preys.size()) * (preys[0].get_pos() - get_pos());
  } else {
    return std::valarray<double>{0., 0.};
    // chiaramente no prede = no correzione
  }
}

// genera predatori casualmente
std::vector<Predator> random_predators(int pred_num,
                                       std::valarray<double> const& pred_space,
                                       double pred_ang, double pred_ds,
                                       double pred_s, double pred_range,
                                       double pred_hunger) {
  std::vector<Predator> predators;
  assert(pred_num >= 0);
  std::random_device rd;
  int x_max = 2.5 * (pred_space[0] - 40.) / pred_ds;
  int y_max = 2.5 * (pred_space[1] - 40.) / pred_ds;

  std::uniform_int_distribution<> dist_pos_x(0, x_max);
  std::uniform_int_distribution<> dist_pos_y(0, y_max);

  std::uniform_real_distribution<> dist_vel_x(-150., 150.);
  std::uniform_real_distribution<> dist_vel_y(-150., 150.);

  auto generator = [&]() -> Predator {
    std::valarray<double> pos = {dist_pos_x(rd) * 0.4 * (pred_ds) + 20.,
                                 dist_pos_y(rd) * 0.4 * (pred_ds) + 20.};
    std::valarray<double> vel = {dist_vel_x(rd), dist_vel_y(rd)};
    return Predator{pos,    vel,        pred_ang,   pred_ds,
                    pred_s, pred_space, pred_range, pred_hunger};
  };

  std::generate_n(std::back_insert_iterator(predators), pred_num, generator);

  auto sort_pred = [](Predator const& pred1, Predator const& pred2) {
    if (pred1.get_pos()[0] == pred2.get_pos()[0]) {
      return pred1.get_pos()[1] < pred2.get_pos()[1];
    } else {
      return pred1.get_pos()[0] < pred2.get_pos()[0];
    }
  };

  std::sort(predators.begin(), predators.end(), sort_pred);

  auto compare_pred = [&](Predator const& b1, Predator const& b2) {
    return boid_dist(b1, b2) < 0.3 * pred_ds;
  };

  // verifica la prima volta che non ci siano boid coincidenti
  auto last = std::unique(predators.begin(), predators.end(), compare_pred);

  // se ce n'erano, rigenera e così fino a quando unique restituisce
  // end(), ovvero non ci sono duplicati
  while (last != predators.end()) {
    std::generate(last, predators.end(), generator);
    std::sort(predators.begin(), predators.end(), sort_pred);
    last = std::unique(predators.begin(), predators.end(), compare_pred);
  }
  return predators;
}

// copiato dal trova vicini in vettore per boid
std::vector<Predator> get_vector_neighbours(std::vector<Predator> const& flock,
                                            std::vector<Predator>::iterator it,
                                            double dist) {
  std::vector<Predator> neighbours;
  assert(it >= flock.begin() && it < flock.end());
  auto et = it;
  for (; et != flock.end() &&
         std::abs(it->get_pos()[0] - et->get_pos()[0]) < dist;
       ++et) {
    if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
        is_visible(*et, *it) == true) {
      neighbours.push_back(*et);
    }
  }
  et = it;
  for (; et != flock.begin() &&
         std::abs(it->get_pos()[0] - et->get_pos()[0]) < dist;
       --et) {
    if (boid_dist(*et, *it) < dist && boid_dist(*et, *it) > 0. &&
        is_visible(*et, *it) == true) {
      neighbours.push_back(*et);
    }
  }
  return neighbours;
}

void update_predators_state(std::vector<Predator>& predators, double delta_t,
                            bool bhv,
                            std::vector<std::pair<Boid, int>> const& preys,
                            std::vector<Obstacle> const& obstacles) {
  std::vector<Predator> copy_predators = predators;
  bool predation = (preys.size() > 0);
  for (auto idx = predators.begin(); idx != predators.end(); ++idx) {
    std::valarray<double> pred_separation = {0., 0.};
    for (auto neighbour_pred : get_vector_neighbours(
             copy_predators, copy_predators.begin() + (idx - predators.begin()),
             idx->get_par_ds())) {
      pred_separation -=
          7 * idx->get_par_s() * (neighbour_pred.get_pos() - idx->get_pos());
    }
    if (predation) {
      // trova le prede di questo predatore nel vettore di (prede, indici)
      // valutando che l'indice corrisponda all'indice del predatore
      std::vector<Boid> own_preys;
      for (auto prey : preys)
      // valuta se l'indice della preda è uguale all'indice del predatore
        if (prey.second == idx - predators.begin())
          own_preys.push_back(prey.first);
      idx->update_state(
          delta_t,
          pred_separation + idx->predate(own_preys) + idx->avoid_obs(obstacles),
          bhv);
    } else {
      idx->update_state(delta_t, pred_separation + idx->avoid_obs(obstacles),
                        bhv);
    }
  }
  auto sort_pred = [](Predator const& pred1, Predator const& pred2) {
    if (pred1.get_pos()[0] == pred2.get_pos()[0]) {
      return pred1.get_pos()[1] < pred2.get_pos()[1];
    } else {
      return pred1.get_pos()[0] < pred2.get_pos()[0];
    }
  };

  std::sort(predators.begin(), predators.end(), sort_pred);
}