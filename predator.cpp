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
    if (position_.x < 0) position_.x = 800;
    if (position_.x > 800) position_.x = 0;
    if (position_.y < 0) position_.y = 600;
    if (position_.y > 600) position_.y = 0; //limitare i movimenti nella finestra 
    }
};

