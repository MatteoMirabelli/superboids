#include <cassert>
#include <cmath>
#include <valarray>

class Boid {
  std::valarray<double> b_pos;
  std::valarray<double> b_vel;
  double b_angle;

 public:
  Boid(std::valarray<double> pos, std::valarray<double> vel);

  std::valarray<double>& get_pos();
  std::valarray<double> const& get_pos() const;

  std::valarray<double>& get_vel();
  std::valarray<double> const& get_vel() const;

  double& get_angle();
  double const& get_angle() const;

  void update_state(double delta_t, std::valarray<double> vel);
};

template <typename T>
double vec_norm(std::valarray<T> vec) {
  return std::sqrt(std::pow(vec, {2, 2}).sum());
}
