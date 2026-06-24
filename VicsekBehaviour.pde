// ============================================================
// Vicsek Flocking Behaviour Module
//
// Part of SwarmManager.
// Implements the Vicsek model: each boid aligns its direction
// to the average direction of neighbours within a fixed radius
// (VICSEK_RADIUS), with added noise for realistic disorder.
// Speed is held constant (VICSEK_SPEED). Boids also seek green
// goalsCS and avoid dangersCS.
// ============================================================

void applyVicsek(SwarmManager sm, Boid b) {
  PVector avgDir = new PVector();
  int count = 0;

  // Average direction of neighbours within Vicsek radius
  for (Boid other : sm.boids) {
    if (other != b && PVector.dist(b.pos, other.pos) < sm.VICSEK_RADIUS) {
      avgDir.add(other.vel.copy().normalize());
      count++;
    }
  }

  if (count > 0) {
    avgDir.div(count);
  } else {
    avgDir = b.vel.copy().normalize();
  }

  // Rotate by random noise for realistic flocking
  float noiseAngle = random(-PI * sm.VICSEK_NOISE, PI * sm.VICSEK_NOISE);
  avgDir.rotate(noiseAngle);
  avgDir.mult(sm.VICSEK_SPEED);

  // Treat as a steering force toward desired velocity
  PVector vicsekForce = PVector.sub(avgDir, b.vel);
  vicsekForce.limit(b.maxForce);
  b.acc.add(vicsekForce);

  // Attraction toward green goals (skip if carrying)
  if (!b.carrying) {
    for (PVector g : sm.goalsCS)
      b.linear_attraction(g, int(sm.ATT_MULT * sm.CUCKER_ATTR_SCALE));
  }

  // Danger avoidance
  for (PVector d : sm.dangersCS)
    b.simpleExponential_repulsion(d, sm.PERLIMITER, sm.REP_MULT);

  // Border repulsion
  for (PVector bp : sm.border_points)
    b.simpleExponential_repulsion(bp, sm.BORDER_PERLIMITER, sm.REP_MULT);

  // Inter-boid repulsion with equilibrium distance
  for (Boid other : sm.boids) {
    if (other != b) {
      b.comfy_attraction(other.pos, sm.COMFY_DIST * 1.5, sm.DRONE_ATT_MULT);
      b.complexExponential_repulsion(other.pos, sm.DRONE_PERLIMITER, sm.DRONE_ATT_MULT, sm.DRONE_REP_MULT * 2);
    }
  }

  b.vel.add(b.acc);
  b.vel.limit(b.maxSpeed);
  b.pos.add(b.vel);
  b.acc.mult(0);
}
