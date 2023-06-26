#include "predator.hpp"

Predator::Predator(std::valarray<double> pos, std::valarray<double> vel,
                   double range) {
  assert(pos.size() == 2 || vel.size() == 2);
  p_pos = pos;
  p_vel = vel;
  p_angle = compute_angle<double>(vel);
  p_range = range;
}

Predator::Predator(double x, double y, double vx, double vy, double range) {
  assert(x >= 0 && y >= 0);
  p_pos = std::valarray<double>(2);
  p_vel = std::valarray<double>(2);
  p_pos[0] = x;
  p_pos[1] = y;
  p_vel[0] = vx;
  p_vel[1] = vy;
  p_angle = compute_angle<double>(p_vel);
  p_range = range;
}

std::valarray<double>& Predator::get_pos() { return p_pos; }

std::valarray<double> const& Predator::get_pos() const { return p_pos; }

std::valarray<double>& Predator::get_vel() { return p_vel; }

std::valarray<double> const& Predator::get_vel() const { return p_vel; }

double& Predator::get_angle() { return p_angle; }

double const& Predator::get_angle() const { return p_angle; }

void Predator::update_state(double delta_t, std::valarray<double> delta_vel) {
  p_vel += delta_vel;
  p_pos += (p_vel * delta_t);
  p_angle = compute_angle<double>(p_vel);
}

std::vector<Boid> Predator::get_neighbours(double const& view_angle,
                                           Flock& flock) {
  std::vector<Boid> neighbours;

  auto ev_dist = [&](std::vector<Boid>::iterator it) {
    return std::abs(it->get_pos()[0] - p_pos[0]) <= p_range;
  };

  auto it = std::find_if(flock.begin(), flock.end(), ev_dist);

  if (it == flock.begin()) {
    auto et = it;
    for (;
         et < --flock.end() && std::abs(p_pos[0] - et->get_pos()[0]) < p_range;
         ++et) {
      if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
          vec_norm<double>(et->get_pos() - p_pos) > 0. &&
          is_visible(*et, view_angle) == true) {
        neighbours.push_back(*et);
      }
    }
    if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
        vec_norm<double>(et->get_pos() - p_pos) > 0. &&
        is_visible(*et, view_angle) == true) {
      neighbours.push_back(*et);
    } else if (it == --flock.end()) {
      auto et = it;
      for (; et > flock.begin() &&
             std::abs(p_pos[0] - et->get_pos()[0]) < p_range;
           --et) {
        if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
            vec_norm<double>(et->get_pos() - p_pos) > 0. &&
            is_visible(*et, view_angle) == true) {
          neighbours.push_back(*et);
        }
      }
      if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
          vec_norm<double>(et->get_pos() - p_pos) > 0. &&
          is_visible(*et, view_angle) == true) {
        neighbours.push_back(*et);
      } else {
        auto et = it;
        for (; et < --flock.end() &&
               std::abs(p_pos[0] - et->get_pos()[0]) < p_range;
             ++et) {
          if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
              vec_norm<double>(et->get_pos() - p_pos) > 0. &&
              is_visible(*et, view_angle) == true) {
            neighbours.push_back(*et);
          }
        }
        if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
            vec_norm<double>(et->get_pos() - p_pos) > 0. &&
            is_visible(*et, view_angle) == true) {
          neighbours.push_back(*et);
        }
        et = it;
        for (; et > flock.begin() &&
               std::abs(p_pos[0] - et->get_pos()[0]) < p_range;
             --et) {
          if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
              vec_norm<double>(et->get_pos() - p_pos) > 0. &&
              is_visible(*et, view_angle) == true) {
            neighbours.push_back(*et);
          }
        }
        if (vec_norm<double>(et->get_pos() - p_pos) < f_params.d &&
            vec_norm<double>(et->get_pos() - p_pos) > 0. &&
            is_visible(*et, view_angle) == true) {
          neighbours.push_back(*et);
        }
      }

      return neighbours;
    }
  }
}

void Predator::update_state(double delta_t, std::valarray<double> delta_vel,
                            bool const& b, double d, double k) {
  p_vel += delta_vel;
  p_pos += (p_vel * delta_t);
  p_angle = compute_angle<double>(p_vel);

  if (b == true) {
    // implementazione con periodiche
    (p_pos[0] > 1800.) ? p_pos[0] = 21. : p_pos[0];
    (p_pos[0] < 20.) ? p_pos[0] = 1799. : p_pos[0];
    (p_pos[1] > 1000.) ? p_pos[1] = 21. : p_pos[1];
    (p_pos[1] < 20.) ? p_pos[1] = 999. : p_pos[1];
  } else {
    // implementazione con bordi

    (p_pos[0] > 1800 - 1.7 * d) ? p_vel[0] -= 2 * k * (1800 - p_pos[0])
                                : p_vel[0];
    (p_pos[0] < 1.7 * d) ? p_vel[0] += 2 * k * p_pos[0] : p_vel[0];
    (p_pos[1] > 1000 - 1.7 * d) ? p_vel[1] -= 2 * k * (1000 - p_pos[1])
                                : p_vel[1];
    (p_pos[1] < 1.7 * d) ? p_vel[1] += 2 * k * p_pos[1] : p_vel[1];
  }
}

bool Predator::is_visible(Boid const& bd, double view_angle) {
  assert(view_angle >= 0 && view_angle <= 180.);

  return std::abs(compute_angle<double>(bd.get_pos() - p_pos) - p_angle) <=
         view_angle;
}
