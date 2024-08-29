#include "flock.hpp"

#include <cmath>

#include "boids.hpp"
#include "doctest.h"

TEST_CASE("Testing function Flock::calculateAverageSpeed") {
  boids::Flock flock;

  SUBCASE(
      "Testing many boids and different speed + testing getSize "
      "function") {
  
    CHECK(flock.getSize() == 0);
    CHECK(flock.calculateAverageSpeed() == 0.0);

   
    boids::Boid b1(0, 0, 3, 4);
    flock.addBoid(b1);
    CHECK(flock.getSize() == 1);
    CHECK(flock.calculateAverageSpeed() == 5.0);


    boids::Boid b2(0, 0, 6, 8);
    flock.addBoid(b2);

    CHECK(flock.getSize() == 2);
    CHECK(flock.calculateAverageSpeed() == 7.5);

   
    boids::Boid b3(0, 0, 0, 0);
    flock.addBoid(b3);

    CHECK(flock.getSize() == 3);
    CHECK(flock.calculateAverageSpeed() == 5.0);
  }

  SUBCASE("Testing two boids with the same speed") {
    boids::Boid b1(10, 20, 5, 5);
    boids::Boid b2(30, 40, 5, 5);

    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK(flock.getSize() == 2);
    CHECK(flock.calculateAverageSpeed() == std::sqrt(50));
  }
}

TEST_CASE("Testing function Flock::calculateAverageDistance") {
  boids::Flock flock;

  SUBCASE("Testing empty flock") {
    CHECK(flock.calculateAverageDistance() == 0.0);
  }

  SUBCASE("Testing for three boids") {
    boids::Boid b1(0, 0, 1, 1);
    boids::Boid b2(3, 4, 1, 1);
    boids::Boid b3(6, 8, 1, 1);

    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);

    double d12{
        b1.getPosition().distance(b2.getPosition())};  // Distanza tra b1 e b2
    double d13{
        b1.getPosition().distance(b3.getPosition())};  // Distanza tra b1 e b3
    double d23{
        b2.getPosition().distance(b3.getPosition())};  // Distanza tra b2 e b3

    double expectedAverageDistance = (d12 + d13 + d23) / 3.0;

    CHECK(flock.calculateAverageDistance() == expectedAverageDistance);
  }

  SUBCASE("Testing for only two boids") {
    boids::Boid b1(10, 20, 1, 1);
    boids::Boid b2(30, 40, 2, 2);

    flock.addBoid(b1);
    flock.addBoid(b2);

    double expectedAverageDistance{b1.getPosition().distance(b2.getPosition())};

    CHECK(flock.calculateAverageDistance() == expectedAverageDistance);
  }
  SUBCASE("Testing two boids far away") {
    boids::Boid b1(0, 0, 1, 1);
    boids::Boid b2(700, 500, 1, 1);
    flock.addBoid(b1);
    flock.addBoid(b2);

    double expectedAverageDistance{b1.getPosition().distance(b2.getPosition())};

    CHECK(flock.calculateAverageDistance() == expectedAverageDistance);
  }
}
TEST_CASE("Testing function Flock::calculateSpeedStandardDeviation") {
  boids::Flock flock;

  SUBCASE("Testing a flock") {

    CHECK(flock.calculateSpeedStandardDeviation() == 0.0);


    boids::Boid b1(3, 4, 1, 1);
    flock.addBoid(b1);
    CHECK(flock.calculateSpeedStandardDeviation() == 0.0);


    boids::Boid b2(10, 10, 6, 8);

    flock.addBoid(b2);

    double avgSpeed{(std::sqrt(2.0) + 10.0) / 2.0};
    double variance{((std::sqrt(2.0) - avgSpeed) * (std::sqrt(2.0) - avgSpeed) +
                     (10.0 - avgSpeed) * (10.0 - avgSpeed)) /
                    2.0};
    double expectedStdDev{std::sqrt(variance)};
    CHECK(flock.calculateSpeedStandardDeviation() ==
          doctest::Approx(expectedStdDev));

   
    boids::Boid b3(20, 20, 1, 1);
    flock.addBoid(b3);

    avgSpeed = (std::sqrt(2.0) + 10.0 + std::sqrt(2.0)) / 3.0;  // Media ≈ 5.814
    variance = ((std::sqrt(2.0) - avgSpeed) * (std::sqrt(2.0) - avgSpeed) +
                (10.0 - avgSpeed) * (10.0 - avgSpeed) +
                (std::sqrt(2.0) - avgSpeed) * (std::sqrt(2.0) - avgSpeed)) /
               3.0;
    expectedStdDev = std::sqrt(variance);
    CHECK(flock.calculateSpeedStandardDeviation() ==
          doctest::Approx(expectedStdDev));
  }

  SUBCASE("Testing boids with the same speed") {
    boids::Boid b1(0, 0, 3, 4);
    flock.addBoid(b1);

    boids::Boid b2(10, 10, 3, 4);
    flock.addBoid(b2);

    boids::Boid b3(20, 20, 3, 4);
    flock.addBoid(b3);

    boids::Boid b4(30, 30, 3, 4);
    flock.addBoid(b4);

    CHECK(flock.calculateSpeedStandardDeviation() == 0.0);
  }
}

TEST_CASE("Testing function Flock::calculateDistanceStandardDeviation") {
  boids::Flock flock;

  SUBCASE("Testing a flock with boids added progressively") {
   
    CHECK(flock.calculateDistanceStandardDeviation() == 0.0);

   //We need to initialize each instance of Vec2 with specific values
    boids::Vec2 pos1(10, 10);

    flock.addBoid(boids::Boid(10, 10, 1, 1));
    flock.addBoid(boids::Boid(10, 10, 1, 1));

    CHECK(flock.calculateDistanceStandardDeviation() == 0.0);

   
    boids::Vec2 pos2(20, 20);
    flock.addBoid(boids::Boid(20, 20, 1, 1));

    //formula for the standard deviation
    double d13{pos1.distance(pos2)};
    double d23{pos1.distance(pos2)};
    double d12{pos1.distance(pos1)};

    double averageDistance{(d12 + d23 + d13) / 3};
    double variance{((d13 - averageDistance) * (d13 - averageDistance) +
                     (d23 - averageDistance) * (d23 - averageDistance) +
                     (d12 - averageDistance) * (d12 - averageDistance)) /
                    3};

    double expectedStandardDeviation{std::sqrt(variance)};

    CHECK(flock.calculateDistanceStandardDeviation() ==
          doctest::Approx(expectedStandardDeviation));

   
    boids::Vec2 pos3(30, 30);
    flock.addBoid(boids::Boid(30, 30, 1, 1));

   
    double d14{pos1.distance(pos3)};
    double d24{pos1.distance(pos3)};
    double d34{pos2.distance(pos3)};

    averageDistance = (d12 + d23 + d13 + d14 + d24 + d34) / 6;
    variance = ((d12 - averageDistance) * (d12 - averageDistance) +
                (d23 - averageDistance) * (d23 - averageDistance) +
                (d13 - averageDistance) * (d13 - averageDistance) +
                (d14 - averageDistance) * (d14 - averageDistance) +
                (d24 - averageDistance) * (d24 - averageDistance) +
                (d34 - averageDistance) * (d34 - averageDistance)) /
               6;

    expectedStandardDeviation = std::sqrt(variance);

    CHECK(flock.calculateDistanceStandardDeviation() ==
          doctest::Approx(expectedStandardDeviation));
  }

  SUBCASE("Testing boids all with the same position") {
    boids::Boid b1(0, 0, 3, 4);
    boids::Boid b2(0, 0, 3, 3);
    boids::Boid b3(0, 0, 5, 4);
    boids::Boid b4(0, 0, 2, 4);

    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);

    CHECK(flock.calculateDistanceStandardDeviation() == 0.0);
  }
}
