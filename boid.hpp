#include <vector>

class Boid {
  std::vector<double> b_pos;
  std::vector<double> b_vel;
  double b_angle;

 public:
  std::vector<double>& get_pos();
  std::vector<double> const& get_pos() const;

  std::vector<double>& get_vel();
  std::vector<double> const& get_vel() const;

  double& get_angle();
  double const& get_angle() const;

  void update_position(double delta_t);
};
