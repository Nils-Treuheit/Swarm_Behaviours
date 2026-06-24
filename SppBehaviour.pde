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

void applySpp(SwarmManager sm, Boid b) {
  // Social positioning: similar weight -> attract, different -> repel
  for (Boid other : sm.boids) {
    if (other != b) {
      float diff = abs(b.socialWeight - other.socialWeight);
      PVector dir = PVector.sub(other.pos, b.pos);
      float dist = dir.mag();
      if (dist < 1) continue;
      float strength;
      if (diff < sm.SPP_WEIGHT_THRESHOLD) {
        // Similar weights: social attraction (strongest when weights are close)
        strength = sm.SPP_ATTRACT_SCALE * (sm.SPP_WEIGHT_THRESHOLD - diff) / sm.SPP_WEIGHT_THRESHOLD * (300 / max(dist, 10));
      } else {
        // Different weights: social repulsion
        strength = -sm.SPP_REPEL_SCALE * (diff - sm.SPP_WEIGHT_THRESHOLD) / (1 - sm.SPP_WEIGHT_THRESHOLD) * (300 / max(dist, 10));
      }
      strength = constrain(strength, -sm.SPP_MAX_FORCE, sm.SPP_MAX_FORCE);
      dir.normalize();
      dir.mult(strength);
      b.acc.add(dir);
    }
  }

  // Attraction toward goalsCS (skip if carrying)
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

  // Basic inter-boid physical repulsion (prevents overlap)
  for (Boid other : sm.boids) {
    if (other != b) {
      b.complexExponential_repulsion(other.pos, sm.DRONE_PERLIMITER, sm.DRONE_ATT_MULT, sm.DRONE_REP_MULT);
    }
  }

  b.vel.add(b.acc);
  b.vel.limit(b.maxSpeed);
  b.pos.add(b.vel);
  b.acc.mult(0);
}
