#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boid.hpp"
#include "doctest.h"
#include "flock.hpp"

TEST_CASE("Testing the Boid class and functions") {
  SUBCASE("Testing the Boid::update_state method") {
    std::valarray<double> pos{2., 2.};
    std::valarray<double> vel{2., 2.};
    std::valarray<double> delta_vel{1., 1.};

    Boid boid(pos, vel);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == doctest::Approx(3.));
    CHECK(boid.get_vel()[1] == doctest::Approx(3.));
    CHECK(boid.get_pos()[0] == doctest::Approx(5.));
    CHECK(boid.get_pos()[1] == doctest::Approx(5.));
    CHECK(boid.get_angle() == doctest::Approx(45.));
  }

  SUBCASE("Testing the Boid::update_state method") {
    std::valarray<double> pos{2., 2.};
    std::valarray<double> vel{1., 1.};
    std::valarray<double> delta_vel{0., 0.};

    Boid boid(pos, vel);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == doctest::Approx(1.));
    CHECK(boid.get_vel()[1] == doctest::Approx(1.));
    CHECK(boid.get_pos()[0] == doctest::Approx(3.));
    CHECK(boid.get_pos()[1] == doctest::Approx(3.));
    CHECK(boid.get_angle() == doctest::Approx(45.));
  }

  SUBCASE("Testing the Boid::update_state method") {
    std::valarray<double> pos{-3., 2.};
    std::valarray<double> vel{5., -4.};
    std::valarray<double> delta_vel{0., -1.};

    Boid boid(pos, vel);
    boid.update_state(1., delta_vel);

    CHECK(boid.get_vel()[0] == doctest::Approx(5.));
    CHECK(boid.get_vel()[1] == doctest::Approx(-5.));
    CHECK(boid.get_pos()[0] == doctest::Approx(2.));
    CHECK(boid.get_pos()[1] == doctest::Approx(-3.));
    CHECK(boid.get_angle() == doctest::Approx(135.));
  }

  SUBCASE("Testing the vec_norm function") {
    std::valarray<double> vec_1{1, 4};
    std::valarray<double> vec_2{2, 5};
    std::valarray<double> vec_3{0, 0};
    std::valarray<double> vec_4{-1, -4};

    double norm1 = vec_norm(vec_1);
    double norm2 = vec_norm(vec_2);
    double norm3 = vec_norm(vec_3);
    double norm4 = vec_norm(vec_4);

    CHECK(norm1 == doctest::Approx(4.1231056));
    CHECK(norm2 == doctest::Approx(5.385164807));
    CHECK(norm3 == 0);
    CHECK(norm4 == doctest::Approx(4.1231056));
  }

  SUBCASE("Testing the boid_dist function") {
    Boid bd_1(2, 2, 5, 4);
    Boid bd_2(4, 4, 1, -3);
    Boid bd_3(0, 0, 3, 5);
    Boid bd_4(0, 0, 6, -1);
    Boid bd_5(1, 8, 0, 0);

    double d12 = boid_dist(bd_1, bd_2);
    double d13 = boid_dist(bd_1, bd_3);
    double d34 = boid_dist(bd_3, bd_4);
    double d15 = boid_dist(bd_1, bd_5);

    CHECK(d12 == doctest::Approx(2.828427));
    CHECK(d13 == doctest::Approx(2.828427));
    CHECK(d34 == 0);
    CHECK(d15 == doctest::Approx(6.082762));
  }

  SUBCASE("Testing the compute_angle function") {
    std::valarray<double> vec_1{1, 4};
    std::valarray<double> vec_2{1, -4};
    std::valarray<double> vec_3{-1, -4};

    double angle_1 = compute_angle<double>(vec_1);
    double angle_2 = compute_angle<double>(vec_2);
    double angle_3 = compute_angle<double>(vec_3);

    CHECK(angle_1 == doctest::Approx(14.036243));
    CHECK(angle_2 == doctest::Approx(165.963756));
    CHECK(angle_3 == doctest::Approx(194.036243));
  }

  SUBCASE("Testing the is_visible function") {
    Boid b1(1., 2., 3., 4.);
    Boid b2(4., 3., 2., 1.);
    Boid b3(1., 1., 1., 1.);

    double angle{150.};
    bool iv12 = is_visible(b1, b2, angle);
    bool iv13 = is_visible(b1, b3, angle);
    CHECK(iv12 == false);
    CHECK(iv13 == true);
  }
}

TEST_CASE("Testing the Flock class and functions") {
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
  }

  SUBCASE("Testing the Flock::update_com method with just one boid") {
    Boid bd_1(2, 2, 5, 4);

    Parameters params(4, 1, 2, 3);
    Boid com(0, 0, 0, 0);

    Flock flock(4, params, 0, com);

    flock.push_back(bd_1);
    flock.update_com();

    CHECK(flock.get_com().get_pos()[0] == 2);
    CHECK(flock.get_com().get_pos()[1] == 2);
    CHECK(flock.get_com().get_vel()[0] == 5);
    CHECK(flock.get_com().get_vel()[1] == 4);
  }

  SUBCASE("Testing the Flock::update_com method with two boids") {
    Boid bd_1(0, 0, 0, 0);
    Boid bd_2(0, 0, 0, 0);

    Parameters params(4, 1, 2, 3);
    Boid com(0, 0, 0, 0);

    Flock flock(4, params, 0, com);

    flock.push_back(bd_1);
    flock.push_back(bd_2);

    flock.update_com();

    CHECK(flock.get_com().get_pos()[0] == 0);
    CHECK(flock.get_com().get_pos()[1] == 0);
    CHECK(flock.get_com().get_vel()[0] == 0);
    CHECK(flock.get_com().get_vel()[1] == 0);
  }
}