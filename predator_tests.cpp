#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"
#include "obstacles.hpp"
#include "predator.hpp"

TEST_CASE("Testing the Predator::Predate method") {
  // PREDATOR CONSTRUCTOR takes:
  // Pos {x,y}, Vel{x,y}, view_angle, window_space{1920, 1080}, param_ds_,
  // param_s, p_range, p_hunger

  SUBCASE("Testing the Predator::Predate method with two preys") {
    Predator pr(2., 2., 5., 5., 50., 1920., 1080., 10., 2., 5., 2.);

    Boid bd1(5., 3., 1., 1., 120., 1920., 1080., 10., 2.);
    Boid bd2(3., 6., 1., 1., 120., 1920., 1080., 10., 2.);

    std::vector<Boid> preys{bd1, bd2};

    auto vel_correction = pr.predate(preys);

    CHECK(vel_correction[0] == 10);
    CHECK(vel_correction[1] == 7);
  }

  SUBCASE("Testing the Predator::Predate method with no preys") {
    Predator pr(2., 2., 5., 5., 50., 1920., 1080., 10., 2., 5., 2.);

    std::vector<Boid> preys{};

    auto vel_correction = pr.predate(preys);

    CHECK(vel_correction[0] == 0);
    CHECK(vel_correction[1] == 0);
  }
}
