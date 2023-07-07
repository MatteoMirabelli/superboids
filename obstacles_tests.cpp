#include "doctest.h"
#include "obstacles.hpp"

TEST_CASE("Testing the generate_obstacles function") {
  // generate_obstacles takes: number_of_obstacles, max_size, space{x,y};
  
  SUBCASE("Testing the generate_obstacles function") {
    auto obstacles = ob::generate_obstacles(4, 45., {1920., 1080.});

    CHECK(obstacles.size() == 4);

    CHECK(obstacles[0].get_size() <= 45.);
    CHECK(obstacles[0].get_size() >= 15.);
    CHECK(obstacles[0].get_pos()[0] >= 202.5);
    CHECK(obstacles[0].get_pos()[0] <= 1717.5);
    CHECK(obstacles[0].get_pos()[1] >= 202.5);
    CHECK(obstacles[0].get_pos()[1] <= 877.5);

    CHECK(obstacles[1].get_size() <= 45.);
    CHECK(obstacles[1].get_size() >= 15.);
    CHECK(obstacles[1].get_pos()[0] >= 202.5);
    CHECK(obstacles[1].get_pos()[0] <= 1717.5);
    CHECK(obstacles[1].get_pos()[1] >= 202.5);
    CHECK(obstacles[1].get_pos()[1] <= 877.5);

    CHECK(obstacles[2].get_size() <= 45.);
    CHECK(obstacles[2].get_size() >= 15.);
    CHECK(obstacles[2].get_pos()[0] >= 202.5);
    CHECK(obstacles[2].get_pos()[0] <= 1717.5);
    CHECK(obstacles[2].get_pos()[1] >= 202.5);
    CHECK(obstacles[2].get_pos()[1] <= 877.5);

    CHECK(obstacles[3].get_size() <= 45.);
    CHECK(obstacles[3].get_size() >= 15.);
    CHECK(obstacles[3].get_pos()[0] >= 202.5);
    CHECK(obstacles[3].get_pos()[0] <= 1717.5);
    CHECK(obstacles[3].get_pos()[1] >= 202.5);
    CHECK(obstacles[3].get_pos()[1] <= 877.5);
  }

  SUBCASE("Testing the generate_obstacles with null number of obstacles") {
    auto obstacles = ob::generate_obstacles(0, 45., {1920., 1080.});
    CHECK(obstacles.size() == 0);
  }
}

TEST_CASE("Testing the add_fixed_obstacle function") {
  // OBSTACLE CONSTRUCTOR takes: pos{x,y}, size;
  // add_fixed_obstacle takes: vector_of_obstacles, pos {x,y}, size, space{x,y};

  SUBCASE("Testing the add_fixed_obstacle with two not overlapping obstacles") {
    std::valarray<double> space{1920., 1080};
    std::valarray<double> pos1{50., 60.};
    std::valarray<double> pos2{100., 100.};

    std::vector<ob::Obstacle> g_obstacles;
    ob::add_fixed_obstacle(g_obstacles, pos1, 20., space);
    ob::add_fixed_obstacle(g_obstacles, pos2, 20., space);

    CHECK(g_obstacles[0].get_pos()[0] == 50.);
    CHECK(g_obstacles[0].get_pos()[1] == 60.);
    CHECK(g_obstacles[0].get_size() == 20.);
    CHECK(g_obstacles[1].get_pos()[0] == 100.);
    CHECK(g_obstacles[1].get_pos()[1] == 100.);
    CHECK(g_obstacles[1].get_size() == 20.);
  }

  SUBCASE("Testing the add_fixed_obstacle with two overlapping obstacles") {
    std::valarray<double> space{1920., 1080};
    std::valarray<double> pos1{50., 60.};
    std::valarray<double> pos2{60., 100.};

    std::vector<ob::Obstacle> g_obstacles;
    ob::add_fixed_obstacle(g_obstacles, pos1, 20., space);
    ob::add_fixed_obstacle(g_obstacles, pos2, 20., space);

    CHECK(g_obstacles.size() == 1);
    CHECK(g_obstacles[0].get_pos()[0] == 50.);
    CHECK(g_obstacles[0].get_pos()[1] == 60.);
    CHECK(g_obstacles[0].get_size() == 20.);
  }

  SUBCASE(
      "Testing the add_fixed_obstacle function with obstacle on the border of "
      "simulation area") {
    std::valarray<double> space{1920., 1080};
    std::valarray<double> pos1{150., 160.};
    std::valarray<double> pos2{10., 100.};

    std::vector<ob::Obstacle> g_obstacles;
    ob::add_fixed_obstacle(g_obstacles, pos1, 20., space);
    ob::add_fixed_obstacle(g_obstacles, pos2, 20., space);

    CHECK(g_obstacles.size() == 1);
    CHECK(g_obstacles[0].get_pos()[0] == 150.);
    CHECK(g_obstacles[0].get_pos()[1] == 160.);
    CHECK(g_obstacles[0].get_size() == 20.);
  }

  SUBCASE(
      "Testing the add_fixed_obstacle function with obstacle on the border of "
      "simulation area") {
    std::valarray<double> space{1920., 1080};
    std::valarray<double> pos1{150., 160.};
    std::valarray<double> pos2{1890., 1060.};

    std::vector<ob::Obstacle> g_obstacles;
    ob::add_fixed_obstacle(g_obstacles, pos1, 20., space);
    ob::add_fixed_obstacle(g_obstacles, pos2, 40., space);

    CHECK(g_obstacles.size() == 1);
    CHECK(g_obstacles[0].get_pos()[0] == 150.);
    CHECK(g_obstacles[0].get_pos()[1] == 160.);
    CHECK(g_obstacles[0].get_size() == 20.);
  }

  SUBCASE(
      "Testing the add_fixed_obstacle function with two tangent obstacles") {
    std::valarray<double> space{1920., 1080};
    std::valarray<double> pos1{50., 60.};
    std::valarray<double> pos2{90., 100.};

    std::vector<ob::Obstacle> g_obstacles;
    ob::add_fixed_obstacle(g_obstacles, pos1, 20., space);
    ob::add_fixed_obstacle(g_obstacles, pos2, 20., space);

    CHECK(g_obstacles[0].get_pos()[0] == 50.);
    CHECK(g_obstacles[0].get_pos()[1] == 60.);
    CHECK(g_obstacles[0].get_size() == 20.);
    CHECK(g_obstacles[1].get_pos()[0] == 90.);
    CHECK(g_obstacles[1].get_pos()[1] == 100.);
    CHECK(g_obstacles[1].get_size() == 20.);
  }
}

TEST_CASE("Testing the sort_obstacles function") {
  // OBSTACLE CONSTRUCTOR takes: pos{x,y}, size;

  // Be careful not to initialize overlapping obstacles. If Obstacles added
  // with add_obstacle function (previously tested), this cannot happen...

  SUBCASE(
      "Testing the sort_obstacle function with obstacles with different "
      "x_positions and y_positions") {
    ob::Obstacle ob1({100., 150.}, 20.);
    ob::Obstacle ob2({1760., 1430}, 40.);
    ob::Obstacle ob3({900., 1467.}, 35);
    ob::Obstacle ob4({400., 500.}, 20.);

    std::vector<ob::Obstacle> g_obstacles;
    g_obstacles.push_back(ob1);
    g_obstacles.push_back(ob2);
    g_obstacles.push_back(ob3);
    g_obstacles.push_back(ob4);

    ob::sort_obstacles(g_obstacles);

    CHECK(g_obstacles[0].get_pos()[0] == 100.);
    CHECK(g_obstacles[0].get_pos()[1] == 150.);
    CHECK(g_obstacles[1].get_pos()[0] == 400.);
    CHECK(g_obstacles[1].get_pos()[1] == 500.);
    CHECK(g_obstacles[2].get_pos()[0] == 900.);
    CHECK(g_obstacles[2].get_pos()[1] == 1467.);
    CHECK(g_obstacles[3].get_pos()[0] == 1760.);
    CHECK(g_obstacles[3].get_pos()[1] == 1430.);
  }

  SUBCASE(
      "Testing the sort_obstacle function with obstacles with same "
      "x_positions (but with different y_positions)") {
    ob::Obstacle ob1({100., 150.}, 20.);
    ob::Obstacle ob2({900., 1467.}, 35);
    ob::Obstacle ob3({100., 1430}, 40.);
    ob::Obstacle ob4({900., 500.}, 20.);

    std::vector<ob::Obstacle> g_obstacles;
    g_obstacles.push_back(ob1);
    g_obstacles.push_back(ob2);
    g_obstacles.push_back(ob3);
    g_obstacles.push_back(ob4);

    ob::sort_obstacles(g_obstacles);

    CHECK(g_obstacles[0].get_pos()[0] == 100.);
    CHECK(g_obstacles[0].get_pos()[1] == 150.);
    CHECK(g_obstacles[1].get_pos()[0] == 100.);
    CHECK(g_obstacles[1].get_pos()[1] == 1430.);
    CHECK(g_obstacles[2].get_pos()[0] == 900.);
    CHECK(g_obstacles[2].get_pos()[1] == 500.);
    CHECK(g_obstacles[3].get_pos()[0] == 900.);
    CHECK(g_obstacles[3].get_pos()[1] == 1467.);
  }

  SUBCASE(
      "Testing the sort_obstacle function with both obstacles with same "
      "x_positions (but with different y_positions) and different "
      "x_positions") {
    ob::Obstacle ob1({100., 150.}, 20.);
    ob::Obstacle ob2({900., 1467.}, 35);
    ob::Obstacle ob3({100., 1430}, 40.);
    ob::Obstacle ob4({1200., 900.}, 25.);
    ob::Obstacle ob5({800., 500.}, 20.);
    ob::Obstacle ob6({400., 1220}, 15.);
    ob::Obstacle ob7({1200., 670.}, 10.);

    std::vector<ob::Obstacle> g_obstacles;
    g_obstacles.push_back(ob1);
    g_obstacles.push_back(ob2);
    g_obstacles.push_back(ob3);
    g_obstacles.push_back(ob4);
    g_obstacles.push_back(ob5);
    g_obstacles.push_back(ob6);
    g_obstacles.push_back(ob7);

    ob::sort_obstacles(g_obstacles);

    CHECK(g_obstacles[0].get_pos()[0] == 100.);
    CHECK(g_obstacles[0].get_pos()[1] == 150.);
    CHECK(g_obstacles[1].get_pos()[0] == 100.);
    CHECK(g_obstacles[1].get_pos()[1] == 1430.);
    CHECK(g_obstacles[2].get_pos()[0] == 400.);
    CHECK(g_obstacles[2].get_pos()[1] == 1220.);
    CHECK(g_obstacles[3].get_pos()[0] == 800.);
    CHECK(g_obstacles[3].get_pos()[1] == 500.);
    CHECK(g_obstacles[4].get_pos()[0] == 900.);
    CHECK(g_obstacles[4].get_pos()[1] == 1467.);
    CHECK(g_obstacles[5].get_pos()[0] == 1200.);
    CHECK(g_obstacles[5].get_pos()[1] == 670.);
    CHECK(g_obstacles[6].get_pos()[0] == 1200.);
    CHECK(g_obstacles[6].get_pos()[1] == 900.);
  }
}