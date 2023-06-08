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

  SUBCASE("Testing the Flock::update_com method") {
    Boid bd_1(2, 2, 5, 4);
    Boid bd_2(5, 7, 1, -3);
    Boid bd_3(6, 8, 4, 3);
    Boid bd_4(10, 12, -4, 8);

    Parameters params(4, 1, 2, 3);
    Boid com(0, 0, 0, 0);

    Flock flock(4, params, 0, com);

    flock.push_back(bd_1);
    flock.push_back(bd_2);
    flock.push_back(bd_3);
    flock.push_back(bd_4);

    flock.update_com();

    CHECK(flock.get_com().get_pos()[0] == 5.75);
    CHECK(flock.get_com().get_pos()[1] == 7.25);
    CHECK(flock.get_com().get_vel()[0] == 1.5);
    CHECK(flock.get_com().get_vel()[1] == 3);
  };
}
