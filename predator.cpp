class Predator {
public:
    Vec2 position_;
    Vec2 velocity_;

    Predator(double x, double y) {
        position_ = Vec2(x, y);
        velocity_ = Vec2(rand() % 3 - 1, rand() % 3 - 1);  // Velocità iniziale casuale
    }

    void move() {
        position_ += velocity_;
        // Aggiungi un codice qui per limitare la posizione all'interno della finestra
    }
};
