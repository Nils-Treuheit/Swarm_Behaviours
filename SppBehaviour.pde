// ============================================================
// SPP (Social Positioning Protocol) Behaviour Module
//
// Part of SwarmManager.
// Each boid is assigned a random social weight in [0, 1].
// Boids with similar weights attract (form same-weight clusters);
// boids with different weights repel. This creates emergent
// role-based social stratification without direct communication.
// Boids also seek green goalsCS and avoid dangersCS.
// ============================================================

void applySpp(Boid b) {
  // Social positioning: similar weight -> attract, different -> repel
  for (Boid other : boids) {
    if (other != b) {
      float diff = abs(b.socialWeight - other.socialWeight);
      PVector dir = PVector.sub(other.pos, b.pos);
      float dist = dir.mag();
      if (dist < 1) continue;
      float strength;
      if (diff < SPP_WEIGHT_THRESHOLD) {
        // Similar weights: social attraction (strongest when weights are close)
        strength = SPP_ATTRACT_SCALE * (SPP_WEIGHT_THRESHOLD - diff) / SPP_WEIGHT_THRESHOLD * (300 / max(dist, 10));
      } else {
        // Different weights: social repulsion
        strength = -SPP_REPEL_SCALE * (diff - SPP_WEIGHT_THRESHOLD) / (1 - SPP_WEIGHT_THRESHOLD) * (300 / max(dist, 10));
      }
      strength = constrain(strength, -SPP_MAX_FORCE, SPP_MAX_FORCE);
      dir.normalize();
      dir.mult(strength);
      b.acc.add(dir);
    }
  }

  // Attraction toward goalsCS (skip if carrying)
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

  // Basic inter-boid physical repulsion (prevents overlap)
  for (Boid other : boids) {
    if (other != b) {
      b.complexExponential_repulsion(other.pos, DRONE_PERLIMITER, DRONE_ATT_MULT, DRONE_REP_MULT);
    }
  }

  b.vel.add(b.acc);
  b.vel.limit(b.maxSpeed);
  b.pos.add(b.vel);
  b.acc.mult(0);
}
