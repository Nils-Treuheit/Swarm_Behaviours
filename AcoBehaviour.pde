// ============================================================
// ACO (Ant Colony Optimisation) Behaviour Module
//
// Part of SwarmManager.
// A pheromone grid (50 px cells) overlays the world. The centre
// emits a constant semi-local pheromone beacon (home).
//
// Non-carrying boids (searching):
//   - Light pheromone deposit
//   - Repelled by pheromones (anti-gradient) — avoids explored areas,
//     spreading the swarm across the canvas
//   - Guided random walk (velocity persistence + noise) for natural
//     exploration patterns
//   - Weak attraction to green targets to eventually find food
//   - Basic inter-boid collision avoidance, no active cohesion
//
// Carrying boids (returning to centre):
//   - Attracted to pheromones (gradient following) — follows trails
//     back toward the centre emitter
//   - Heavy pheromone deposit (4x) to reinforce the return trail
//   - Inter-boid cohesion + repulsion for safe spacing
//   - Slight wander for natural braided trails
//
// Behaviour switches instantly when a boid picks up or drops
// an item — no timer or phase needed.
//
// Pheromone evaporates and diffuses each frame. Yellow dots
// (size/alpha mapped to intensity) show trail density.
// ============================================================

// ---------------------------------------------------------------
// initPheromone: allocate the 2D pheromone grid
// ---------------------------------------------------------------
void initPheromone(SwarmManager sm) {
  sm.PHEROMONE_COLS = width / sm.PHEROMONE_CELL + 1;
  sm.PHEROMONE_ROWS = height / sm.PHEROMONE_CELL + 1;
  sm.pheromone = new float[sm.PHEROMONE_COLS][sm.PHEROMONE_ROWS];
}

// ---------------------------------------------------------------
// evaporatePheromone: decay all cells uniformly
// ---------------------------------------------------------------
void evaporatePheromone(SwarmManager sm) {
  for (int x = 0; x < sm.PHEROMONE_COLS; x++)
    for (int y = 0; y < sm.PHEROMONE_ROWS; y++)
      sm.pheromone[x][y] *= (1 - sm.PHEROMONE_EVAPORATION);
}

// ---------------------------------------------------------------
// diffusePheromone: standard 4-neighbour diffusion
// ---------------------------------------------------------------
void diffusePheromone(SwarmManager sm) {
  float[][] temp = new float[sm.PHEROMONE_COLS][sm.PHEROMONE_ROWS];
  for (int x = 1; x < sm.PHEROMONE_COLS - 1; x++)
    for (int y = 1; y < sm.PHEROMONE_ROWS - 1; y++) {
      temp[x][y] = sm.pheromone[x][y] * (1 - 4 * sm.PHEROMONE_DIFFUSION)
          + (sm.pheromone[x - 1][y] + sm.pheromone[x + 1][y] + sm.pheromone[x][y - 1] + sm.pheromone[x][y + 1]) * sm.PHEROMONE_DIFFUSION;
    }
  sm.pheromone = temp;
}

// ---------------------------------------------------------------
// applyAco: per-boid ACO logic
//
// Non-carrying (searching):
//   - Light pheromone deposit
//   - Guided random walk (velocity persistence + noise)
//   - Repelled by pheromones (anti-gradient) — avoids explored areas
//   - Weak target attraction to find food
//   - Basic inter-boid repulsion (no active cohesion)
//
// Carrying (returning to centre):
//   - Heavy pheromone deposit (4x) to mark return trail
//   - Attracted to pheromones (gradient following) toward centre
//   - Inter-boid cohesion + repulsion for safe spacing
//   - Slight wander for natural braided trails
//
// Behaviour switches instantly when carrying status changes,
// no timer/phase needed.
// ---------------------------------------------------------------
void applyAco(SwarmManager sm, Boid b) {
  // Map boid position to pheromone grid cell
  int px = int(b.pos.x / sm.PHEROMONE_CELL);
  int py = int(b.pos.y / sm.PHEROMONE_CELL);
  px = constrain(px, 0, sm.PHEROMONE_COLS - 1);
  py = constrain(py, 0, sm.PHEROMONE_ROWS - 1);

  // Deposit pheromone at current cell
  float deposit = sm.PHEROMONE_DEPOSIT;
  if (b.carrying) {
    deposit *= 4; // heavy return trail
  }
  // Searching boids deposit only the base amount (no bonus near attractors)
  sm.pheromone[px][py] = min(sm.pheromone[px][py] + deposit, 100);

  // Read pheromone gradient (difference to each of the 8 neighbours)
  PVector gradient = new PVector();
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      int nx = constrain(px + dx, 0, sm.PHEROMONE_COLS - 1);
      int ny = constrain(py + dy, 0, sm.PHEROMONE_ROWS - 1);
      float diff = sm.pheromone[nx][ny] - sm.pheromone[px][py];
      gradient.add(new PVector(dx, dy).mult(diff));
    }
  }

  if (b.carrying) {
    // Carrying: attracted to pheromones — follow gradient toward centre emitter
    if (gradient.mag() > 0.1) {
      gradient.setMag(sm.PHEROMONE_INFLUENCE * 2);
      b.acc.add(gradient);
    }
    b.acc.add(PVector.random2D().mult(0.15));
  } else {
    // Searching: repelled by pheromones — anti-gradient avoids explored areas
    if (gradient.mag() > 0.1) {
      PVector anti = PVector.mult(gradient, -1);
      anti.setMag(sm.PHEROMONE_INFLUENCE * 1.2);
      b.acc.add(anti);
    }
    // Guided random walk: persist velocity direction + small noise
    PVector forward = b.vel.copy().normalize().mult(0.35);
    PVector noise = PVector.random2D().mult(0.2);
    b.acc.add(PVector.add(forward, noise));
    // Weak target attraction to find food
    for (PVector a : sm.attractors)
      b.linear_attraction(a, int(sm.ATT_MULT * 0.2));
  }

  // Repulsion from danger dots
  for (PVector r : sm.repellents)
    b.simpleExponential_repulsion(r, sm.PERLIMITER, sm.REP_MULT);

  // Border repulsion
  for (PVector bp : sm.border_points)
    b.simpleExponential_repulsion(bp, sm.BORDER_PERLIMITER, sm.REP_MULT);

  // Inter-boid forces
  for (Boid other : sm.boids) {
    if (other != b) {
      if (b.carrying) {
        // Carrying: maintain safe distance with both cohesion and repulsion
        b.comfy_attraction(other.pos, sm.COMFY_DIST * 1.5, sm.DRONE_ATT_MULT);
        b.complexExponential_repulsion(other.pos, sm.DRONE_PERLIMITER, sm.DRONE_ATT_MULT, sm.DRONE_REP_MULT * 2);
      } else {
        // Searching: just basic collision avoidance, no active cohesion
        b.complexExponential_repulsion(other.pos, sm.DRONE_PERLIMITER, sm.DRONE_ATT_MULT, sm.DRONE_REP_MULT);
      }
    }
  }

  b.integrate();
}
