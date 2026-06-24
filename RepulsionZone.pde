class RepulsionZone {
  PVector pos;
  float radius;

  RepulsionZone(float x, float y, float r) {
    pos = new PVector(x, y);
    radius = r;
  }

  void display() {
    noFill();
    stroke(255, 0, 0);
    ellipse(pos.x, pos.y, radius * 2, radius * 2);
  }
}
