#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"
#include "doctest.h"
TEST_CASE("Testing the function Vec2::lenght")
{
   boids::Vec2 v1(3.0, 4.0);
    CHECK(v1.length() == doctest::Approx(5.0)); 

    boids::Vec2 v2(0.0, 0.0);
    CHECK(v2.length() == doctest::Approx(0.0)); 

    boids::Vec2 v3(1.0, 1.0);
    CHECK(v3.length() == doctest::Approx(std::sqrt(2.0))); 
    
    boids::Vec2 v4(-3.0, -4.0);
    CHECK(v4.length() == doctest::Approx(5.0)); 

    boids::Vec2 v5(-2.0, 1.0);
    CHECK(v5.lenght()==doctest::Approx(std::sqrt(5.0)));
}
TEST_CASE("Testing Vec2::distance") {
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
    boids::Vec2 v10( 18.0,3.0);
    CHECK (v9.distance(v10)== doctest::Approx(std::sqrt(37.0)));
}
TEST_CASE("Testing the function alignment") {
  SUBCASE("Testing alignmentFactor = 0.0") {
        double alignmentFactor = 0.0;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));
        boids::Boid neighbor1(boids::Vec2(1, 1), boids::Vec2(2, 2));
        boids::Boid neighbor2(boids::Vec2(2, 2), boids::Vec2(3, 3));
        std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

        boids::Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

        // Con alignmentFactor = 0.0, il risultato dovrebbe essere (0,0)
        CHECK(alignmentResult.x_ == doctest::Approx(0));
        CHECK(alignmentResult.y_ == doctest::Approx(0));
    }

    SUBCASE("Testing alignmentFactor = 0.5") {
        double alignmentFactor = 0.5;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));
        boids::Boid neighbor1(boids::Vec2(1, 1), boids::Vec2(2, 2));
        boids::Boid neighbor2(boids::Vec2(2, 2), boids::Vec2(3, 3));
        std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

        boids::Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

        // Con alignmentFactor = 0.5, il risultato dovrebbe essere la metà della differenza tra la media delle velocità e la velocità del Boid centrale
        CHECK(alignmentResult.x_ == doctest::Approx(0.75));
        CHECK(alignmentResult.y_ == doctest::Approx(0.75));
    }

    SUBCASE("Testing con molti Boids e alignmentFactor = 1.0") {
        double alignmentFactor = 1.0;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));

        std::vector<boids::Boid> boids;
        boids.push_back(centralBoid);
        for (int i = 1; i <= 30; ++i) {
            boids.emplace_back(boids::Vec2(i, i), boids::Vec2(2 + i * 0.1, 2 + i * 0.1));
        }

        boids::Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

        // Calcolo della velocità media attesa
        boids::Vec2 expectedVelocity(0, 0);
        for (int i = 1; i <= 30; ++i) {
            expectedVelocity += boids::Vec2(2 + i * 0.1, 2 + i * 0.1);
        }
        expectedVelocity = expectedVelocity / 30;

        boids::Vec2 expectedAlignment = (expectedVelocity - centralBoid.velocity_);

        CHECK(alignmentResult.x_ == doctest::Approx(expectedAlignment.x_));
        CHECK(alignmentResult.y_ == doctest::Approx(expectedAlignment.y_));
    }

    SUBCASE("Test con alignmentFactor molto elevato") {
        double alignmentFactor = 100.0;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));

        std::vector<boids::Boid> boids = {
            centralBoid,
            boids::Boid(boids::Vec2(1, 1), boids::Vec2(2, 2)),
            boids::Boid(boids::Vec2(-1, -1), boids::Vec2(3, 3)),
            boids::Boid(boids::Vec2(2, -2), boids::Vec2(4, 4))};

        boids::Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

        // Calcolo della velocità media attesa
        boids::Vec2 expectedVelocity(0, 0);
        for (int i = 1; i < boids.size(); ++i) {
            expectedVelocity += boids[i].velocity_;
        }
        expectedVelocity = expectedVelocity / (boids.size());

        boids::Vec2 expectedAlignment = (expectedVelocity - centralBoid.velocity_) * alignmentFactor;

        CHECK(alignmentResult.x_ == doctest::Approx(expectedAlignment.x_));
        CHECK(alignmentResult.y_ == doctest::Approx(expectedAlignment.y_));
    }
SUBCASE("Testing boids with opposite velocity") {
    boids::Boid boid(boids::Vec2(0, 0), boids::Vec2(1.0, 1.0));
    std::vector<boids::Boid> neighbors = {boids::Boid(boids::Vec2(1, 1), boids::Vec2(-1.0, -1.0)),
                                          boids::Boid(boids::Vec2(-1, -1), boids::Vec2(3.0, 3.0))};
    boids::Vec2 result = boid.alignment(neighbors, 0.5);
    CHECK(result == boids::Vec2(0.0, 0.0)); // steer dovrebbe essere zero
}
   SUBCASE("Testing with no neighbors") {
    boids::Boid boid(boids::Vec2(0, 0), boids::Vec2(1.0, 1.0));
    std::vector<boids::Boid> neighbors;  // Lista vuota
    boids::Vec2 result = boid.alignment(neighbors, 0.5);
    CHECK(result == boids::Vec2(0.0, 0.0)); // steer dovrebbe essere zero
}
SUBCASE("Test with one fast neighbors") {
    boids::Boid boid(boids::Vec2(0, 0), boids::Vec2(1.0, 1.0));
    std::vector<boids::Boid> neighbors = {boids::Boid(boids::Vec2(1, 1), boids::Vec2(10.0, 10.0))};
    boids::Vec2 result = boid.alignment(neighbors, 0.5);
    CHECK(result == boids::Vec2(4.5, 4.5)); // steer dovrebbe essere una spinta verso il vicino
} 
}


TEST_CASE("Testing the function cohesion") {

    SUBCASE("Testing cohesionFactor = 0.0") {
        double cohesionFactor = 0.0;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));
        boids::Boid neighbor1(boids::Vec2(1, 1), boids::Vec2(2, 2));
        boids::Boid neighbor2(boids::Vec2(2, 2), boids::Vec2(3, 3));
        std::vector<boids::Boid> boids = {centralBoid, neighbor1, neighbor2};

        boids::Vec2 cohesionResult = centralBoid.cohesion(boids, cohesionFactor);

        // Con cohesionFactor = 0.0, il risultato dovrebbe essere (0,0)
        CHECK(cohesionResult.x_ == doctest::Approx(0));
        CHECK(cohesionResult.y_ == doctest::Approx(0));
    }

    SUBCASE("Testing with on neighbors cohesionFactor = 1.0") {
        double cohesionFactor = 1.0;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));
        boids::Boid neighbor1(boids::Vec2(10, 10), boids::Vec2(2, 2));
        std::vector<boids::Boid> boids = {centralBoid, neighbor1};

        boids::Vec2 cohesionResult = centralBoid.cohesion(boids, cohesionFactor);

        // Il risultato dovrebbe essere la differenza tra la posizione del vicino e quella del boid centrale
        CHECK(cohesionResult.x_ == doctest::Approx(10.0));
        CHECK(cohesionResult.y_ == doctest::Approx(10.0));
    }

    SUBCASE("Testing with many and cohesionFactor = 0.5") {
        double cohesionFactor = 0.5;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));

        std::vector<boids::Boid> boids;
        boids.push_back(centralBoid);
        for (int i = 1; i <= 10; ++i) {
            boids.emplace_back(boids::Vec2(i, i), boids::Vec2(2 + i * 0.1, 2 + i * 0.1));
        }

        boids::Vec2 cohesionResult = centralBoid.cohesion(boids, cohesionFactor);

        // Calcolo della posizione media attesa
        boids::Vec2 expectedCenterOfMass(0, 0);
        for (int i = 1; i <= 10; ++i) {
            expectedCenterOfMass += boids::Vec2(i, i);
        }
        expectedCenterOfMass = expectedCenterOfMass / 10;

        boids::Vec2 expectedCohesion = (expectedCenterOfMass - centralBoid.position_) * cohesionFactor;

        CHECK(cohesionResult.x_ == doctest::Approx(expectedCohesion.x_));
        CHECK(cohesionResult.y_ == doctest::Approx(expectedCohesion.y_));
    }

    SUBCASE("Test con cohesionFactor molto elevato") {
        double cohesionFactor = 100.0;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));

        std::vector<boids::Boid> boids = {
            centralBoid,
            boids::Boid(boids::Vec2(1, 1), boids::Vec2(2, 2)),
            boids::Boid(boids::Vec2(-1, -1), boids::Vec2(3, 3)),
            boids::Boid(boids::Vec2(2, -2), boids::Vec2(4, 4))
        };

        boids::Vec2 cohesionResult = centralBoid.calculateCohesion(boids, cohesionFactor);

        // Calcolo della posizione media attesa
        boids::Vec2 expectedCenterOfMass(0, 0);
        for (int i = 1; i < boids.size(); ++i) {
            expectedCenterOfMass += boids[i].position_;
        }
        expectedCenterOfMass = expectedCenterOfMass / (boids.size() - 1);

        boids::Vec2 expectedCohesion = (expectedCenterOfMass - centralBoid.position_) * cohesionFactor;

        CHECK(cohesionResult.x_ == doctest::Approx(expectedCohesion.x_));
        CHECK(cohesionResult.y_ == doctest::Approx(expectedCohesion.y_));
    }

    SUBCASE("Testing with no neighbors") {
        double cohesionFactor = 1.0;
        boids::Boid centralBoid(boids::Vec2(0, 0), boids::Vec2(1, 1));
        std::vector<boids::Boid> boids = {centralBoid};

        boids::Vec2 cohesionResult = centralBoid.calculateCohesion(boids, cohesionFactor);

        // Con nessun vicino, il risultato dovrebbe essere (0,0)
        CHECK(cohesionResult.x_ == doctest::Approx(0));
        CHECK(cohesionResult.y_ == doctest::Approx(0));
    }
}











