#include <vector>

#include "boid.hpp"

class Flock {
  double f_d;
  std::vector<Boid> f_flock;
  Boid f_com;

 public:
  double size() const;
  void push_back(Boid const& boid);
  Boid& get_boid(int n);
  Boid const& get_boid(int n) const;
  void erase(int n);
  void update_com();

  std::valarray<double> friend separation(double a);
  void friend alignment();
  void friend cohesion();
};
