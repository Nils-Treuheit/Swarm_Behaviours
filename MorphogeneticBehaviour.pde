// ============================================================
// Morphogenetic Swarm Behaviour Module
//
// Part of SwarmManager.
// Inspired by gene-regulatory networks (GRN). Each boid carries a
// morphogen concentration that evolves via:
//   - Diffusion from neighbours (within MORPH_COMM_RADIUS)
//   - Production toward a random target level
//   - Decay
//
// High morphogen → strong goal-seeking (attraction to attractors).
// Low morphogen  → exploratory wandering with magnified danger
//                  sensitivity (stronger repulsion from repellents).
// ============================================================

void applyMorphogenetic(Boid b) {
  // GRN-inspired morphogen dynamics: neighbour diffusion
  float avgNeighborMorphogen = 0;
  int neighborCount = 0;
  for (Boid other : boids) {
    if (other != b && PVector.dist(b.pos, other.pos) < MORPH_COMM_RADIUS) {
      avgNeighborMorphogen += other.morphogen;
      neighborCount++;
    }
  }
  if (neighborCount > 0) {
    avgNeighborMorphogen /= neighborCount;
    b.morphogen += MORPH_DIFFUSION * (avgNeighborMorphogen - b.morphogen);
  }

  // Production toward target level + decay
  b.morphogen += MORPH_PRODUCTION * (b.morphogenTarget - b.morphogen);
  b.morphogen *= MORPH_DECAY;
  b.morphogen = constrain(b.morphogen, 0, 2);

  // Goal attraction scaled by morphogen (skip if carrying)
  if (!b.carrying) {
    for (PVector a : attractors) {
      if (b.morphogen > 0.5) {
        b.linear_attraction(a, int(ATT_MULT * b.morphogen));
      }
    }
  }

  // Repulsion from repellents: stronger when morphogen is low (exploratory state)
  for (PVector r : repellents)
    b.simpleExponential_repulsion(r, PERLIMITER, int(REP_MULT * (2 - b.morphogen)));

  // Border repulsion
  for (PVector bp : border_points)
    b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

  // Inter-boid forces
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
