#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"

TEST_CASE("Testing the Boid and Flock classes") {
  SUBCASE("Testing the Boid::update_state method") {
    std::valarray<double> pos{2, 2};
    std::valarray<double> vel{2, 2};
    std::valarray<double> delta_vel{1, 1};

    Boid boid(pos, vel);
    boid.update_state(1, delta_vel);

    CHECK(boid.get_vel()[0] == 3);
    CHECK(boid.get_vel()[1] == 3);
    CHECK(boid.get_pos()[0] == 5);
    CHECK(boid.get_pos()[1] == 5);
  };

  SUBCASE("Testing the Boid::update_state method") {
    std::valarray<double> pos{2, 2};
    std::valarray<double> vel{1, 1};
    std::valarray<double> delta_vel{0, 0};

    Boid boid(pos, vel);
    boid.update_state(1, delta_vel);

    CHECK(boid.get_vel()[0] == 1);
    CHECK(boid.get_vel()[1] == 1);
    CHECK(boid.get_pos()[0] == 3);
    CHECK(boid.get_pos()[1] == 3);
  };

  SUBCASE("Testing the Boid::update_state method") {
    std::valarray<double> pos{-3, 2};
    std::valarray<double> vel{5, -4};
    std::valarray<double> delta_vel{0, -1};

    Boid boid(pos, vel);
    boid.update_state(1, delta_vel);

    CHECK(boid.get_vel()[0] == 5);
    CHECK(boid.get_vel()[1] == -5);
    CHECK(boid.get_pos()[0] == 2);
    CHECK(boid.get_pos()[1] == -3);
  };
}