// ============================================================
// AttRep (Attraction / Repulsion) Behaviour Module
//
// Part of SwarmManager.
// Simple force-based swarm where boids are attracted toward
// green targets and repelled from red danger points, borders,
// and other boids.
// ============================================================

void applyAttRep(SwarmManager sm, Boid b) {
  // Goal attraction (skip if carrying — universal centre pull handles it)
  if (!b.carrying) {
    for (PVector a : sm.attractors)
      b.linear_attraction(a, sm.ATT_MULT);
  }

  // Repulsion from user-placed danger dots (D items)
  for (PVector r : sm.repellents)
    b.simpleExponential_repulsion(r, sm.PERLIMITER, sm.REP_MULT);

  // Border repulsion (canvas walls)
  for (PVector bp : sm.border_points)
    b.simpleExponential_repulsion(bp, sm.BORDER_PERLIMITER, sm.REP_MULT);

  // Inter-boid forces: comfy attraction at distance + exponential repulsion at close range
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
