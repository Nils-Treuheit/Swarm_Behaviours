// ============================================================
// Cucker-Smale Flocking Behaviour Module
//
// Part of SwarmManager.
// Implements the Cucker-Smale flocking model: each boid's
// velocity aligns toward the weighted average of all other
// boids' velocities. The weight decays quadratically with
// distance (CUCKER_H / (CUCKER_H + d^2)), so nearby boids
// have a stronger influence than distant ones. Boids also
// seek green goalsCS and avoid dangersCS.
// ============================================================

void applyCuckerSmale(SwarmManager sm, Boid b) {
  PVector alignment = new PVector();
  float totalWeight = 0;

  // Weighted velocity alignment: closer neighbours have more influence
  for (Boid other : sm.boids) {
    if (other != b) {
      float dist   = PVector.dist(b.pos, other.pos);
      float weight = sm.CUCKER_H / (sm.CUCKER_H + dist * dist);
      PVector diff = PVector.sub(other.vel, b.vel);
      alignment.add(PVector.mult(diff, weight));
      totalWeight += weight;
    }
  }

  if (totalWeight > 0) alignment.div(totalWeight);

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

  // Inter-boid repulsion (prevent collapse at close range)
  for (Boid other : sm.boids) {
    if (other != b) {
      b.comfy_attraction(other.pos, sm.COMFY_DIST * 1.5, sm.DRONE_ATT_MULT);
      b.complexExponential_repulsion(other.pos, sm.DRONE_PERLIMITER, sm.DRONE_ATT_MULT, sm.DRONE_REP_MULT * 2);
    }
  }

  b.acc.add(alignment);
  b.integrate();
}
