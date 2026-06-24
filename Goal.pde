class Goal {
  PVector pos;
  float gravity;

  Goal(float x, float y, float g) {
    pos = new PVector(x, y);
    gravity = g;
  }

  void display() {
    noFill();
    stroke(0, 255, 0);
    ellipse(pos.x, pos.y, 20, 20);
  }
}
