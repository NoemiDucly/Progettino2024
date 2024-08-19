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
        boids::double alignmentFactor = 0.0;
        boids::Boid centralBoid(0, 0, Vec2(1, 1));
        boids::Boid neighbor1(1, 1, Vec2(2, 2));
        boids::Boid neighbor2(2, 2, Vec2(3, 3));
        boids::std::vector<Boid> boids = { centralBoid, neighbor1, neighbor2 };

        boids::Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

        // Con alignmentFactor = 0.0, il risultato dovrebbe essere (0,0)
        CHECK(alignmentResult.x_ == doctest::Approx(0));
        CHECK(alignmentResult.y_ == doctest::Approx(0));
    }
 SUBCASE("Testing alignmentFactor = 0.5") {
        // Boid centrale con vicini e alignmentFactor = 0.5
        double alignmentFactor = 0.5;
        Boid centralBoid(0, 0, Vec2(1, 1));
        Boid neighbor1(1, 1, Vec2(2, 2));
        Boid neighbor2(2, 2, Vec2(3, 3));
        std::vector<Boid> boids = { centralBoid, neighbor1, neighbor2 };

        Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

        // Con alignmentFactor = 0.5, il risultato dovrebbe essere la metà della differenza tra la media delle velocità e la velocità del Boid centrale
        CHECK(alignmentResult.x_ == doctest::Approx(0.75));
        CHECK(alignmentResult.y_ == doctest::Approx(0.75));
    }
   SUBCASE("Testing con molti Boids e alignmentFactor = 1.0") {
    // Boid centrale con molti vicini e alignmentFactor = 1.0
    double alignmentFactor = 1.0;
    Boid centralBoid(0, 0, Vec2(1, 1));

    // Creiamo un grande numero di vicini
    std::vector<Boid> boids;
    boids.push_back(centralBoid);
    for (int i = 1; i <= 30; ++i) {
        boids.emplace_back(i, i, Vec2(2 + i * 0.1, 2 + i * 0.1)); // Vicini con velocità incrementale
    }

    Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

    // Calcolo della velocità media attesa
    Vec2 expectedVelocity(0, 0);
    for (int i = 1; i <= 30; ++i) {
        expectedVelocity += Vec2(2 + i * 0.1, 2 + i * 0.1);
    }
    expectedVelocity = expectedVelocity / 30;

    // Risultato atteso: differenza tra la media delle velocità e la velocità del Boid centrale
    Vec2 expectedAlignment = (expectedVelocity - centralBoid.velocity_);

    CHECK(alignmentResult.x_ == doctest::Approx(expectedAlignment.x_));
    CHECK(alignmentResult.y_ == doctest::Approx(expectedAlignment.y_));
}
SUBCASE("Test con alignmentFactor molto elevato") {
    // Boid centrale con alcuni vicini e alignmentFactor molto elevato
    double alignmentFactor = 100.0;  // Valore molto elevato
    Boid centralBoid(0, 0, Vec2(1, 1));

    // Creiamo alcuni vicini con velocità diverse
    std::vector<Boid> boids = {
        centralBoid,
        Boid(1, 1, Vec2(2, 2)),
        Boid(-1, -1, Vec2(3, 3)),
        Boid(2, -2, Vec2(4, 4))
    };

    Vec2 alignmentResult = centralBoid.alignment(boids, alignmentFactor);

    // Calcolo della velocità media attesa
    Vec2 expectedVelocity(0, 0);
    for (int i = 1; i < boids.size(); ++i) {
        expectedVelocity += boids[i].velocity_;
    }
    expectedVelocity = expectedVelocity / (boids.size());

    // Risultato atteso: differenza tra la media delle velocità e la velocità del Boid centrale, moltiplicata per alignmentFactor
    Vec2 expectedAlignment = (expectedVelocity - centralBoid.velocity_) * alignmentFactor;

    CHECK(alignmentResult.x_ == doctest::Approx(expectedAlignment.x_));
    CHECK(alignmentResult.y_ == doctest::Approx(expectedAlignment.y_));
}

   
}
