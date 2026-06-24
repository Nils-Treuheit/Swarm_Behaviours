// ============================================================
// Cucker-Smale Flocking Behaviour Module
//
// Part of SwarmManager.
// Implements the Cucker-Smale flocking model: each boid's
// velocity aligns toward the weighted average of all other
// boids' velocities. The weight decays quadratically with
// distance (CUCKER_H / (CUCKER_H + d²)), so nearby boids
// have a stronger influence than distant ones. Boids also
// seek green goalsCS and avoid dangersCS.
// ============================================================

void applyCuckerSmale(Boid b) {
  PVector alignment = new PVector();
  float totalWeight = 0;

  // Weighted velocity alignment: closer neighbours have more influence
  for (Boid other : boids) {
    if (other != b) {
      float dist   = PVector.dist(b.pos, other.pos);
      float weight = CUCKER_H / (CUCKER_H + dist * dist);
      PVector diff = PVector.sub(other.vel, b.vel);
      alignment.add(PVector.mult(diff, weight));
      totalWeight += weight;
    }
  }

  if (totalWeight > 0) alignment.div(totalWeight);

  // Attraction toward green goals (skip if carrying)
  if (!b.carrying) {
    for (PVector g : goalsCS)
      b.linear_attraction(g, int(ATT_MULT * CUCKER_ATTR_SCALE));
  }

  // Danger avoidance
  for (PVector d : dangersCS)
    b.simpleExponential_repulsion(d, PERLIMITER, REP_MULT);

  // Border repulsion
  for (PVector bp : border_points)
    b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

  // Inter-boid repulsion (prevent collapse at close range)
  for (Boid other : boids) {
    if (other != b) {
      b.comfy_attraction(other.pos, COMFY_DIST * 1.5, DRONE_ATT_MULT);
      b.complexExponential_repulsion(other.pos, DRONE_PERLIMITER, DRONE_ATT_MULT, DRONE_REP_MULT * 2);
    }
  }

  b.acc.add(alignment);
  b.vel.add(b.acc);
  b.vel.limit(b.maxSpeed);
  b.pos.add(b.vel);
  b.acc.mult(0);
}
