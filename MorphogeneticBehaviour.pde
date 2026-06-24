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
// High morphogen -> strong goal-seeking (attraction to attractors).
// Low morphogen  -> exploratory wandering with magnified danger
//                  sensitivity (stronger repulsion from repellents).
// ============================================================

void applyMorphogenetic(SwarmManager sm, Boid b) {
  // GRN-inspired morphogen dynamics: neighbour diffusion
  float avgNeighborMorphogen = 0;
  int neighborCount = 0;
  for (Boid other : sm.boids) {
    if (other != b && PVector.dist(b.pos, other.pos) < sm.MORPH_COMM_RADIUS) {
      avgNeighborMorphogen += other.morphogen;
      neighborCount++;
    }
  }
  if (neighborCount > 0) {
    avgNeighborMorphogen /= neighborCount;
    b.morphogen += sm.MORPH_DIFFUSION * (avgNeighborMorphogen - b.morphogen);
  }

  // Production toward target level + decay
  b.morphogen += sm.MORPH_PRODUCTION * (b.morphogenTarget - b.morphogen);
  b.morphogen *= sm.MORPH_DECAY;
  b.morphogen = constrain(b.morphogen, 0, 2);

  // Goal attraction scaled by morphogen (skip if carrying)
  if (!b.carrying) {
    for (PVector a : sm.attractors) {
      if (b.morphogen > 0.5) {
        b.linear_attraction(a, int(sm.ATT_MULT * b.morphogen));
      }
    }
  }

  // Repulsion from repellents: stronger when morphogen is low (exploratory state)
  for (PVector r : sm.repellents)
    b.simpleExponential_repulsion(r, sm.PERLIMITER, int(sm.REP_MULT * (2 - b.morphogen)));

  // Border repulsion
  for (PVector bp : sm.border_points)
    b.simpleExponential_repulsion(bp, sm.BORDER_PERLIMITER, sm.REP_MULT);

  // Inter-boid forces
  for (Boid other : sm.boids) {
    if (other != b) {
      b.comfy_attraction(other.pos, sm.COMFY_DIST * 1.5, sm.DRONE_ATT_MULT);
      b.complexExponential_repulsion(other.pos, sm.DRONE_PERLIMITER, sm.DRONE_ATT_MULT, sm.DRONE_REP_MULT * 2);
    }
  }

  b.integrate();
}
