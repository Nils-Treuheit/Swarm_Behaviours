// ============================================================
// AttRep (Attraction / Repulsion) Behaviour Module
//
// Part of SwarmManager.
// Simple force-based swarm where boids are attracted toward
// green targets and repelled from red danger points, borders,
// and other boids.
// ============================================================

void applyAttRep(Boid b) {
  // Goal attraction (skip if carrying — universal centre pull handles it)
  if (!b.carrying) {
    for (PVector a : attractors)
      b.linear_attraction(a, ATT_MULT);
  }

  // Repulsion from user-placed danger dots (D items)
  for (PVector r : repellents)
    b.simpleExponential_repulsion(r, PERLIMITER, REP_MULT);

  // Border repulsion (canvas walls)
  for (PVector bp : border_points)
    b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

  // Inter-boid forces: comfy attraction at distance + exponential repulsion at close range
  for (Boid other : boids) {
    if (other != b) {
      b.comfy_attraction(other.pos, COMFY_DIST * 1.5, DRONE_ATT_MULT);
      b.complexExponential_repulsion(other.pos, DRONE_PERLIMITER, DRONE_ATT_MULT, DRONE_REP_MULT * 2);
    }
  }

  b.vel.add(b.acc);
  b.vel.limit(b.maxSpeed);
  b.pos.add(b.vel);
  b.acc.mult(0);
}
