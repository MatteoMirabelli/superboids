
// inizialmente speravo di poterlo sfruttare per i predatori, invece ho dovuto
// reimplementare. Si può anche riportare in flock
std::vector<Boid> get_vector_neighbours(std::vector<Boid> const& flock,
                                        std::vector<Boid>::iterator it,
                                        double dist) {
  std::vector<Boid> neighbours;
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