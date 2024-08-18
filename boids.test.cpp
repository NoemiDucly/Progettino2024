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
