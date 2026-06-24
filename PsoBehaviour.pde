// ============================================================
// PSO (Particle Swarm Optimisation) Behaviour Module
//
// Part of SwarmManager.
// Each boid tracks its personal best position (closest it has
// been to a green target) and the swarm's global best. The
// velocity update blends inertia, cognitive (personal best),
// and social (global best) components, plus target attraction
// and obstacle avoidance.
// ============================================================

void applyPSO(SwarmManager sm, Boid b) {
  // Update personal best: min distance to nearest attractor
  if (!sm.attractors.isEmpty()) {
    float minDist = Float.MAX_VALUE;
    for (PVector a : sm.attractors) {
      float d = PVector.dist(b.pos, a);
      if (d < minDist) minDist = d;
    }
    if (minDist < b.pbestFitness) {
      b.pbestFitness = minDist;
      b.pbest = b.pos.copy();
    }
  }

  // PSO velocity update
  float r1 = random(1);
  float r2 = random(1);
  PVector cognitive = PVector.sub(b.pbest, b.pos).mult(sm.PSO_COGNITIVE * r1);
  PVector social    = PVector.sub(sm.gbest, b.pos).mult(sm.PSO_SOCIAL * r2);
  PVector inertia   = b.vel.copy().mult(sm.PSO_INERTIA);

  PVector psoForce = PVector.add(inertia, cognitive);
  psoForce.add(social);

  // Attraction to attractors (skip if carrying)
  if (!b.carrying) {
    for (PVector a : sm.attractors)
      b.linear_attraction(a, sm.ATT_MULT);
  }

  // Repulsion from danger dots
  for (PVector r : sm.repellents)
    b.simpleExponential_repulsion(r, sm.PERLIMITER, sm.REP_MULT);

  // Border repulsion
  for (PVector bp : sm.border_points)
    b.simpleExponential_repulsion(bp, sm.BORDER_PERLIMITER, sm.REP_MULT);

  // Inter-boid repulsion
  for (Boid other : sm.boids) {
    if (other != b) {
      b.comfy_attraction(other.pos, sm.COMFY_DIST * 1.5, sm.DRONE_ATT_MULT);
      b.complexExponential_repulsion(other.pos, sm.DRONE_PERLIMITER, sm.DRONE_ATT_MULT, sm.DRONE_REP_MULT * 2);
    }
  }

  // Blend PSO force with existing acc
  b.acc.add(psoForce.limit(10));
  b.integrate();
}
