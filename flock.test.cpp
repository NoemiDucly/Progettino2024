#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boids.hpp"
#include "flock.hpp"
#include <cmath>

TEST_CASE("Test della funzione calculateAverageSpeed") {
  SUBCASE("Testing a flock with many boids and different speed")
    boids::Flock flock;

    // Case 1: no boids in the flock
    CHECK(flock.calculateAverageSpeed() == 0.0);

    // Case 2: one single flock
    boids::Boid b1(Vec2(0, 0), Vec2(3, 4));  // velocità = sqrt(3^2 + 4^2) = 5.0
    flock.addBoid(b1);
    CHECK(flock.calculateAverageSpeed() == 5.0);

    // Case 3: we had another boid
    boids::Boid b2(boids::Vec2(0, 0), boids::Vec2(6, 8));  // velocità = sqrt(6^2 + 8^2) = 10.0
    flock.addBoid(b2);
    // La velocità media attesa è (5.0 + 10.0) / 2 = 7.5
    CHECK(flock.calculateAverageSpeed() == 7.5);

    // Case 4: we had the third flock with velocity=0
    boids::Boid b3(boids::Vec2(0, 0), boids::Vec2(0, 0));  // velocità = sqrt(0^2 + 0^2) = 0.0
    flock.addBoid(b3);

    CHECK(flock.calculateAverageSpeed() == 5.0);
}
 SUBCASE("Testing the function for two boids with the same speed"){
  boids::Flock flock;

    // Aggiungiamo due boids con la stessa velocità
    boids::Boid b1(boids::Vec2(10, 20), boids::Vec2(5, 5));  
    boids::Boid b2(boids::Vec2(30, 40), boids::Vec2(5, 5));  

    flock.addBoid(b1);
    flock.addBoid(b2);
    CHECK (flock.calculateAverageSpeed()==5.0);
}

TEST_CASE("Test della funzione calculateAverageDistance con boids a posizioni diverse") {
  SUBCASE( "Testing empty flock"){
    boids::Flock flock;

    // Verifica che la distanza media in un flock vuoto sia 0.0
    CHECK(flock.calculateAverageDistance() == 0.0);
}

  SUBCASE ("Testing for 3 boids") {
    boids::Flock flock;

    // Aggiungiamo boids a posizioni diverse
    boids::Boid b1(boids::Vec2(0, 0), boids::Vec2(1, 1));
    boids::Boid b2(boids::Vec2(3, 4), boids::Vec2(1, 1));
    boids::Boid b3(boids::Vec2(6, 8), boids::Vec2(1, 1));

    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);

    // Calcoliamo la distanza tra ogni coppia di boids
    double d12 = b1.getPosition().distance(b2.getPosition()); // Distanza tra b1 e b2
    double d13 = b1.getPosition().distance(b3.getPosition()); // Distanza tra b1 e b3
    double d23 = b2.getPosition().distance(b3.getPosition()); // Distanza tra b2 e b3

    // La distanza media attesa è (d12 + d13 + d23) / 3
    double expectedAverageDistance = (d12 + d13 + d23) / 3.0;

    // Confrontiamo con il valore calcolato dalla funzione calculateAverageDistance
    CHECK(flock.calculateAverageDistance() == expectedAverageDistance);
}
  
SUBCASE ("Testing for only two boids"){
  boids::Flock flock;

    // Creiamo due boids con posizioni diverse
    boids::Boid b1(boids::Vec2(10, 20), boids::Vec2(1, 1));
    boids::Boid b2(boids::Vec2(30, 40), boids::Vec2(2, 2));

    // Aggiungiamo i boids al flock
    flock.addBoid(b1);
    flock.addBoid(b2);

    // Calcoliamo la distanza tra i due boids
    double distance = b1.getPosition().distance(b2.getPosition());

    // La distanza media attesa è la distanza tra i due boids stessi
    double expectedAverageDistance = distance;

    // Confrontiamo con il valore calcolato dalla funzione calculateAverageDistance
    CHECK(flock.calculateAverageDistance() == expectedAverageDistance);
  
}
SUBCASE ("Testing two boids far away") {
    boids::Flock flock;

    
    boids::Boid b1(boids::Vec2(0, 0), boids::Vec2(1, 1));
    boids::Boid b2(boids::Vec2(700, 500), boids::Vec2(1, 1)); // angolo opposto dello schermo


    flock.addBoid(b1);
    flock.addBoid(b2);

    // Calcoliamo la distanza tra i due boids
    double distance = b1.getPosition().distance(b2.getPosition());

    // La distanza media attesa è la distanza stessa, poiché c'è solo una coppia di boids
    double expectedAverageDistance = distance;

    // Confrontiamo con il valore calcolato dalla funzione calculateAverageDistance
    CHECK(flock.calculateAverageDistance() == expectedAverageDistance);
}
}
TEST_CASE("Testing the function calculateSpeedStandardDeviation") {
  SUBCASE ("Testing a flock")
  {
    boids::Flock flock;

    // Case 1: empty flock 
    CHECK(flock.calculateSpeedStandardDeviation() == 0.0);

    // Case 2: one single boid
    boids::Boid b1(boids::Vec2(3, 4), boids::Vec2(1, 1));
    flock.addBoid(b1);
    CHECK(flock.calculateSpeedStandardDeviation() == 0.0);  // Solo un boid, deviazione standard è 0

    // Case 3: two boids with different speeds
    boids::Boid b2(boids::Vec2(10, 10), boids::Vec2(6, 8));
    
    flock.addBoid(b2);
    // Calcolo della deviazione standard: velocità = 5.0 e 10.0
    double avgSpeed = (5.0 + 10.0) / 2.0; // Media = 7.5
    double variance = ((5.0 - avgSpeed) * (5.0 - avgSpeed) + (10.0 - avgSpeed) * (10.0 - avgSpeed)) / 2.0;
    double expectedStdDev = std::sqrt(variance);
    CHECK(flock.calculateSpeedStandardDeviation() == doctest::Approx(expectedStdDev));

    // Caso 4: three boids with different speeds
    boids::Boid b3(boids::Vec2(20, 20), boids::Vec2(1, 1));
   
    flock.addBoid(b3);
    // Calcolo della deviazione standard: velocità = 5.0, 10.0, e ≈ 1.414
    avgSpeed = (5.0 + 10.0 + 1.414) / 3.0; // Media ≈ 5.814
    variance = ((5.0 - avgSpeed) * (5.0 - avgSpeed) + (10.0 - avgSpeed) * (10.0 - avgSpeed) + (1.414 - avgSpeed) * (1.414 - avgSpeed)) / 3.0;
    expectedStdDev = std::sqrt(variance);
    CHECK(flock.calculateSpeedStandardDeviation() == doctest::Approx(expectedStdDev));

    
}
SUBCASE ("Testing boids with the same speed")
{
     boids::Flock flock;

  boids::Boid b1(boids::Vec2(0, 0), boids::Vec2(3, 4));
 
    flock.addBoid(b1);
 boids::Boid b2(boids::Vec2(10, 10), boids::Vec2(3, 4));
   
    flock.addBoid(b2);
 boids::Boid b3(boids::Vec2(20, 20), boids::Vec2(3, 4));

    flock.addBoid(b3);
 boids::Boid b4(boids::Vec2(30, 30), boids::Vec2(3, 4));
   
    flock.addBoid(b4);

    // Tutti i boids hanno la stessa velocità, quindi la deviazione standard deve essere 0.0
    CHECK(flock.calculateSpeedStandardDeviation() == 0.0);
}
}

TEST_CASE("Test della deviazione standard della distanza con boids aggiunti progressivamente") {
  SUBCASE("Testing a flock"){
    boids::Flock flock;

    // Case 1: empty flock
    CHECK(flock.calculateDistanceStandardDeviation() == 0.0);
    
    // Due boids con la stessa posizione: deviazione standard deve essere 0.0
    CHECK(flock.calculateDistanceStandardDeviation() == 0.0);
 boids::Vec2 pos1(10, 10);
    boids::Vec2 vel(1, 1); // Velocità irrilevante per questo test

    flock.addBoid(boids::Boid(pos1, vel));
    flock.addBoid(boids::Boid(pos1, vel));
    
    // Due boids con la stessa posizione: deviazione standard deve essere 0.0
    CHECK(flock.calculateDistanceStandardDeviation() == 0.0);

    // Caso 3: Aggiungi un terzo boid con posizione diversa
    boids::Vec2 pos2(20, 20); // Distanza tra (10,10) e (20,20) è sqrt(200) ≈ 14.14
    flock.addBoid(boids::Boid(pos2, vel));

    // Calcola manualmente la deviazione standard
    double d1 = pos1.distance(pos2); // Distanza tra (10,10) e (20,20) ≈ 14.14
    double d2 = pos1.distance(pos2); // La stessa distanza
    double d3 = pos2.distance(pos2); // Distanza tra (20,20) e (20,20) è 0.0
    
    double averageDistance = (d1 + d2 + d3) / 3; // Media delle distanze
    double variance = ((d1 - averageDistance) * (d1 - averageDistance) +
                       (d2 - averageDistance) * (d2 - averageDistance) +
                       (d3 - averageDistance) * (d3 - averageDistance)) / 3;
    
    double expectedStandardDeviation = std::sqrt(variance);

    // Devi fare attenzione a quanto segue
    // Devi calcolare la distanza media delle posizioni e la deviazione standard manualmente
    CHECK(flock.calculateDistanceStandardDeviation() == doctest::Approx(expectedStandardDeviation));

    // Caso 4: Aggiungi un quarto boid con posizione diversa
    boids::Vec2 pos3(30, 30); // Distanza tra (10,10) e (30,30) è sqrt(800) ≈ 28.28
    flock.addBoid(boids::Boid(pos3, vel));
    
    // Calcola manualmente la deviazione standard
    d1 = pos1.distance(pos2);  // sqrt(200) ≈ 14.14
    d2 = pos1.distance(pos3);  // sqrt(800) ≈ 28.28
    d3 = pos2.distance(pos3);  // sqrt(800) ≈ 28.28
    
    averageDistance = (d1 + d2 + d3) / 3;  // Media delle distanze
    variance = ((d1 - averageDistance) * (d1 - averageDistance) +
                (d2 - averageDistance) * (d2 - averageDistance) +
                (d3 - averageDistance) * (d3 - averageDistance)) / 3;
    
    expectedStandardDeviation = std::sqrt(variance);

    // Devi fare attenzione a quanto segue
    // Devi calcolare la distanza media delle posizioni e la deviazione standard manualmente
    CHECK(flock.calculateDistanceStandardDeviation() == doctest::Approx(expectedStandardDeviation));
}
SUBCASE ("Testing boids all with the same position")
{
  boids::Flock flock;

    // Aggiungi boids con la stessa posizione
     boids::Boid b1(boids::Vec2(10, 10), boids::Vec2(3, 4));
    boids::Boid b2(boids::Vec2(0, 0), boids::Vec2(3, 4));
    boids::Boid b3(boids::Vec2(0, 0), boids::Vec2(3, 4));
   boids::Boid b4(boids::Vec2(0, 0), boids::Vec2(3, 4));
 
    flock.addBoid(b1);
    flock.addBoid(b2);
    flock.addBoid(b3);
    flock.addBoid(b4);

    // Tutti i boids hanno la stessa posizione, quindi la deviazione standard deve essere 0.0
    CHECK(flock.calculateDistanceStandardDeviation() == 0.0);
}
  
}


