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

void applyPSO(Boid b) {
  // Update personal best: min distance to nearest attractor
  if (!attractors.isEmpty()) {
    float minDist = Float.MAX_VALUE;
    for (PVector a : attractors) {
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
  PVector cognitive = PVector.sub(b.pbest, b.pos).mult(PSO_COGNITIVE * r1);
  PVector social    = PVector.sub(gbest, b.pos).mult(PSO_SOCIAL * r2);
  PVector inertia   = b.vel.copy().mult(PSO_INERTIA);

  PVector psoForce = PVector.add(inertia, cognitive);
  psoForce.add(social);

  // Attraction to attractors (skip if carrying)
  if (!b.carrying) {
    for (PVector a : attractors)
      b.linear_attraction(a, ATT_MULT);
  }

  // Repulsion from danger dots
  for (PVector r : repellents)
    b.simpleExponential_repulsion(r, PERLIMITER, REP_MULT);

  // Border repulsion
  for (PVector bp : border_points)
    b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

  // Inter-boid repulsion
  for (Boid other : boids) {
    if (other != b) {
      b.comfy_attraction(other.pos, COMFY_DIST * 1.5, DRONE_ATT_MULT);
      b.complexExponential_repulsion(other.pos, DRONE_PERLIMITER, DRONE_ATT_MULT, DRONE_REP_MULT * 2);
    }
  }

  // Blend PSO force with existing acc
  b.acc.add(psoForce.limit(10));
  b.vel.add(b.acc);
  b.vel.limit(b.maxSpeed);
  b.pos.add(b.vel);
  b.acc.mult(0);
}
