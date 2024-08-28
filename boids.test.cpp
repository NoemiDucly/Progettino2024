#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"

#include "doctest.h"

TEST_CASE("Testing overloaded operators for Vec2 struct") {
  boids::Vec2 v1(3.0, 2.0);
  boids::Vec2 v2(2.0, 2.0);
  double d0{0};
  double d1{2.0};
  boids::Vec2 sum{v1 + v2};
  boids::Vec2 diff{v1 - v2};
  boids::Vec2 prod{v1 * d1};
  boids::Vec2 div{v2 / d1};

  CHECK(sum.x_ == 5.0);
  CHECK(sum.y_ == 4.0);
  CHECK(diff.x_ == 1.0);
  CHECK(diff.y_ == 0.0);
  CHECK(prod.x_ == 6.0);
  CHECK(prod.y_ == 4.0);
  CHECK(div.x_ == 1.0);
  CHECK(div.y_ == 1.0);
  CHECK_THROWS(v1.operator/(d0));
}

TEST_CASE("Testing function Vec2::lenght") {
  boids::Vec2 v1(3.0, 4.0);
  CHECK(v1.lenght() == doctest::Approx(5.0));

  boids::Vec2 v2(0.0, 0.0);
  CHECK(v2.lenght() == doctest::Approx(0.0));

  boids::Vec2 v3(1.0, 1.0);
  CHECK(v3.lenght() == doctest::Approx(std::sqrt(2.0)));

  boids::Vec2 v4(-3.0, -4.0);
  CHECK(v4.lenght() == doctest::Approx(5.0));

  boids::Vec2 v5(-2.0, 1.0);
  CHECK(v5.lenght() == doctest::Approx(std::sqrt(5.0)));
}

TEST_CASE("Testing function Vec2::distance") {
  boids::Vec2 v1(0.0, 0.0);
  boids::Vec2 v2(3.0, 4.0);
  CHECK(v1.distance(v2) == doctest::Approx(5.0));

  boids::Vec2 v3(1.0, -2.0);
  boids::Vec2 v4(4.0, 2.0);
  CHECK(v3.distance(v4) == doctest::Approx(5.0));

  boids::Vec2 v5(0.0, 0.0);
  boids::Vec2 v6(0.0, 0.0);
  CHECK(v5.distance(v6) == doctest::Approx(0.0));

  boids::Vec2 v7(-3.0, 4.0);
  boids::Vec2 v8(0.0, 0.0);
  CHECK(v7.distance(v8) == doctest::Approx(5.0));

  boids::Vec2 v9(12.0, 4.0);
  boids::Vec2 v10(18.0, 3.0);
  CHECK(v9.distance(v10) == doctest::Approx(std::sqrt(37.0)));
}

TEST_CASE("Testing function Boid::alignment") {
  boids::Boid centralBoid(0, 0, 1, 1);
  double d{50.0};

  SUBCASE("Testing with alignmentFactor = 0.0") {
    double alignmentFactor{0.0};
    boids::Boid neighbor1(1, 1, 2, 2);
    boids::Boid neighbor2(2, 2, 3, 3);
    std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

    boids::Vec2 alignmentResult{
        centralBoid.alignment(boids, alignmentFactor, d)};

    CHECK(alignmentResult.x_ == doctest::Approx(0));
    CHECK(alignmentResult.y_ == doctest::Approx(0));
  }

  SUBCASE("Testing with alignmentFactor = 0.5") {
    double alignmentFactor{0.5};
    boids::Boid neighbor1(1, 1, 2, 2);
    boids::Boid neighbor2(2, 2, 3, 3);
    std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

    boids::Vec2 alignmentResult{
        centralBoid.alignment(boids, alignmentFactor, d)};

    CHECK(alignmentResult.x_ == doctest::Approx(0.75));
    CHECK(alignmentResult.y_ == doctest::Approx(0.75));
  }

  SUBCASE("Testing with many boids and alignmentFactor = 1.0") {
    double alignmentFactor{1.0};

    std::vector<boids::Boid> boids;
    boids.push_back(centralBoid);
    for (int i = 1; i <= 30; ++i) {
      boids.emplace_back(i, i, 2 + i * 0.1, 2 + i * 0.1);
    }

    boids::Vec2 alignmentResult{
        centralBoid.alignment(boids, alignmentFactor, d)};

    // Calcolo della velocità media attesa
    boids::Vec2 expectedVelocity(0, 0);
    for (int i = 1; i <= 30; ++i) {
      expectedVelocity += boids::Vec2(2 + i * 0.1, 2 + i * 0.1);
    }
    expectedVelocity = expectedVelocity / 30;

    boids::Vec2 expectedAlignment{
        (expectedVelocity - centralBoid.getVelocity())};

    CHECK(alignmentResult.x_ == doctest::Approx(expectedAlignment.x_));
    CHECK(alignmentResult.y_ == doctest::Approx(expectedAlignment.y_));
  }

  SUBCASE("Test with very high alignmentFactor") {
    double alignmentFactor{100.0};

    std::vector<boids::Boid> boids = {centralBoid, boids::Boid(1, 1, 2, 2),
                                      boids::Boid(-1, -1, 3, 3),
                                      boids::Boid(2, -2, 4, 4)};

    boids::Vec2 alignmentResult{
        centralBoid.alignment(boids, alignmentFactor, d)};

    boids::Vec2 expectedVelocity(0, 0);

    for (long unsigned int i = 1; i < boids.size(); ++i) {
      expectedVelocity += boids[i].getVelocity();
    }
    expectedVelocity =
        expectedVelocity / (static_cast<double>(boids.size()) - 1);

    boids::Vec2 expectedAlignment{
        (expectedVelocity - centralBoid.getVelocity()) * alignmentFactor};

    CHECK(alignmentResult.x_ == doctest::Approx(expectedAlignment.x_));
    CHECK(alignmentResult.y_ == doctest::Approx(expectedAlignment.y_));
  }

  SUBCASE("Testing boids with opposite velocity") {
    boids::Boid boid(0, 0, 1, 1);
    std::vector<boids::Boid> neighbors = {boids::Boid(1, 1, -1, -1),
                                          boids::Boid(-1, -1, 3, 3)};
    boids::Vec2 result{boid.alignment(neighbors, 0.5, d)};
    CHECK(result.x_ == 0.0);
    CHECK(result.y_ == 0.0);
  }

  SUBCASE("Testing with no neighbors") {
    boids::Boid boid(0, 0, 1, 1);
    std::vector<boids::Boid> neighbors;
    boids::Vec2 result{boid.alignment(neighbors, 0.5, d)};
    CHECK(result.x_ == 0.0);
    CHECK(result.y_ == 0.0);
  }

  SUBCASE("Test with one fast neighbor") {
    boids::Boid boid(0, 0, 1, 1);
    std::vector<boids::Boid> neighbors = {boids::Boid(1, 1, 10, 10)};
    boids::Vec2 result{boid.alignment(neighbors, 0.5, d)};
    CHECK(result.x_ == 4.5);
    CHECK(result.y_ == 4.5);
  }
}

TEST_CASE("Testing function Boid::cohesion") {
  boids::Boid centralBoid(0, 0, 1, 1);
  double d{50.0};

  SUBCASE("Testing with cohesionFactor = 0.0") {
    double cohesionFactor{0.0};
    boids::Boid neighbor1(1, 1, 2, 2);
    boids::Boid neighbor2(2, 2, 3, 3);
    std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

    boids::Vec2 cohesionResult{centralBoid.cohesion(boids, cohesionFactor, d)};

    CHECK(cohesionResult.x_ == doctest::Approx(0));
    CHECK(cohesionResult.y_ == doctest::Approx(0));
  }

  SUBCASE("Testing with one neighbor and cohesionFactor = 1.0") {
    double cohesionFactor{1.0};
    boids::Boid neighbor1(10, 10, 2, 2);
    std::vector<boids::Boid> boids = {centralBoid, neighbor1};

    boids::Vec2 cohesionResult{centralBoid.cohesion(boids, cohesionFactor, d)};

    CHECK(cohesionResult.x_ == doctest::Approx(10.0));
    CHECK(cohesionResult.y_ == doctest::Approx(10.0));
  }

  SUBCASE("Testing with many boids and cohesionFactor = 0.5") {
    double cohesionFactor{0.5};

    std::vector<boids::Boid> boids;
    boids.push_back(centralBoid);
    for (int i = 1; i <= 10; ++i) {
      boids.emplace_back(i, i, 2 + i * 0.1, 2 + i * 0.1);
    }

    boids::Vec2 cohesionResult{centralBoid.cohesion(boids, cohesionFactor, d)};

    // Calcolo della posizione media attesa
    boids::Vec2 expectedCenterOfMass(0, 0);
    for (int i = 1; i <= 10; ++i) {
      expectedCenterOfMass += boids::Vec2(i, i);
    }
    expectedCenterOfMass = expectedCenterOfMass / 10;

    boids::Vec2 expectedCohesion{
        (expectedCenterOfMass - centralBoid.getPosition()) * cohesionFactor};

    CHECK(cohesionResult.x_ == doctest::Approx(expectedCohesion.x_));
    CHECK(cohesionResult.y_ == doctest::Approx(expectedCohesion.y_));
  }

  SUBCASE("Test with very high cohesionFactor") {
    double cohesionFactor{100.0};

    std::vector<boids::Boid> boids = {centralBoid, boids::Boid(1, 1, 2, 2),
                                      boids::Boid(-1, -1, 3, 3),
                                      boids::Boid(2, -2, 4, 4)};

    boids::Vec2 cohesionResult{centralBoid.cohesion(boids, cohesionFactor, d)};

    // Calcolo della posizione media attesa
    boids::Vec2 expectedCenterOfMass(0, 0);
    for (long unsigned int i = 1; i < boids.size(); ++i) {
      expectedCenterOfMass += boids[i].getPosition();
    }
    expectedCenterOfMass =
        expectedCenterOfMass / (static_cast<double>(boids.size()) - 1);

    boids::Vec2 expectedCohesion =
        (expectedCenterOfMass - centralBoid.getPosition()) * cohesionFactor;

    CHECK(cohesionResult.x_ == doctest::Approx(expectedCohesion.x_));
    CHECK(cohesionResult.y_ == doctest::Approx(expectedCohesion.y_));
  }

  SUBCASE("Test with no neighbors") {
    double cohesionFactor{1.0};
    std::vector<boids::Boid> boids = {centralBoid};

    boids::Vec2 cohesionResult{centralBoid.cohesion(boids, cohesionFactor, d)};

    CHECK(cohesionResult.x_ == doctest::Approx(0));
    CHECK(cohesionResult.y_ == doctest::Approx(0));
  }
}

TEST_CASE("Testing function Boid::separation") {
  double d{50.0};
  boids::Boid centralBoid(0, 0, 1, 1);

  SUBCASE("Testing with no neighbors") {
    double separationDistance{5.0};
    std::vector<boids::Boid> boids = {centralBoid};

    boids::Vec2 separationResult{
        centralBoid.separation(boids, separationDistance, 1, d)};

    CHECK(separationResult.x_ == doctest::Approx(0));
    CHECK(separationResult.y_ == doctest::Approx(0));
  }

  SUBCASE("Testing with one neighbor within separation distance") {
    double separationDistance{5.0};
    boids::Boid neighbor(2, 3, 2, 2);
    std::vector<boids::Boid> boids = {centralBoid, neighbor};
    double distanceS{
        (centralBoid.getPosition() - neighbor.getPosition()).lenght()};
    boids::Vec2 separationResult{
        centralBoid.separation(boids, separationDistance, 1, d)};

    boids::Vec2 expectedSeparation{
        (centralBoid.getPosition() - neighbor.getPosition()).normalize() /
        distanceS};

    CHECK(separationResult.x_ == doctest::Approx(expectedSeparation.x_));
    CHECK(separationResult.y_ == doctest::Approx(expectedSeparation.y_));
  }

  SUBCASE("Testing with multiple neighbors within separation distance") {
    double separationDistance{10.0};
    boids::Boid neighbor1(5, 0, 2, 2);
    boids::Boid neighbor2(0, 5, 3, 3);
    std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

    boids::Vec2 separationResult{
        centralBoid.separation(boids, separationDistance, 1, d)};

    boids::Vec2 expectedSeparation1{
        (centralBoid.getPosition() - neighbor1.getPosition()).normalize() /
        5.0};
    boids::Vec2 expectedSeparation2{
        (centralBoid.getPosition() - neighbor2.getPosition()).normalize() /
        5.0};

    boids::Vec2 expectedSeparation{(expectedSeparation1 + expectedSeparation2) /
                                   2};  // count = 2

    CHECK(separationResult.x_ == doctest::Approx(expectedSeparation.x_));
    CHECK(separationResult.y_ == doctest::Approx(expectedSeparation.y_));
  }

  SUBCASE("Testing with neighbors outside of separation distance") {
    double separationDistance{3.0};
    boids::Boid neighbor1(10, 0, 2, 2);
    boids::Boid neighbor2(0, 10, 3, 3);
    std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

    boids::Vec2 separationResult{
        centralBoid.separation(boids, separationDistance, 1, d)};

    CHECK(separationResult.x_ == doctest::Approx(0));
    CHECK(separationResult.y_ == doctest::Approx(0));
  }

  SUBCASE("Testing with zero separation distance") {
    double separationDistance{0.0};
    boids::Boid neighbor(3, 4, 2, 2);
    std::vector<boids::Boid> boids = {centralBoid, neighbor};

    boids::Vec2 separationResult{
        centralBoid.separation(boids, separationDistance, 1, d)};

    CHECK(separationResult.x_ == doctest::Approx(0));
    CHECK(separationResult.y_ == doctest::Approx(0));
  }

  SUBCASE("Testing with very large separation distance") {
    double separationDistance{1000.0};
    boids::Boid neighbor1(3, 4, 2, 2);
    boids::Boid neighbor2(1, 1, 2, 2);
    std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

    boids::Vec2 separationResult{
        centralBoid.separation(boids, separationDistance, 1, d)};

    // Calcoliamo il vettore di separazione atteso
    boids::Vec2 expectedSeparation1{
        (centralBoid.getPosition() - neighbor1.getPosition()).normalize() /
        5.0};
    boids::Vec2 expectedSeparation2{
        (centralBoid.getPosition() - neighbor2.getPosition()).normalize() /
        1.414};

    boids::Vec2 expectedSeparation{(expectedSeparation1 + expectedSeparation2) /
                                   2};  // count = 2

    CHECK(separationResult.x_ ==
          doctest::Approx(expectedSeparation.x_).epsilon(0.01));
    CHECK(separationResult.y_ ==
          doctest::Approx(expectedSeparation.y_).epsilon(0.01));
  }
}
