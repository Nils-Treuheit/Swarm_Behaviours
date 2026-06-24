// ============================================================
// ACO (Ant Colony Optimisation) Behaviour Module
//
// Part of SwarmManager.
// A pheromone grid (50 px cells) overlays the world. The centre
// emits a constant semi-local pheromone beacon (home).
//
// Non-carrying boids (searching):
//   - Weak attraction to green targets (25% of normal)
//   - Follow pheromone gradient to converge on trails left by
//     returning boids
//   - Random wander for exploration
//
// Carrying boids (returning to centre):
//   - Follow pheromone gradient uphill toward the centre emitter
//   - Deposit heavy pheromone (4×) on the return path
//   - Slight wander for natural braided trails
//
// Pheromone evaporates and diffuses each frame. Yellow dots
// (size/alpha mapped to intensity) show trail density.
// ============================================================

// ---------------------------------------------------------------
// initPheromone: allocate the 2D pheromone grid
// ---------------------------------------------------------------
void initPheromone() {
  PHEROMONE_COLS = width / PHEROMONE_CELL + 1;
  PHEROMONE_ROWS = height / PHEROMONE_CELL + 1;
  pheromone = new float[PHEROMONE_COLS][PHEROMONE_ROWS];
}

// ---------------------------------------------------------------
// evaporatePheromone: decay all cells uniformly
// ---------------------------------------------------------------
void evaporatePheromone() {
  for (int x = 0; x < PHEROMONE_COLS; x++)
    for (int y = 0; y < PHEROMONE_ROWS; y++)
      pheromone[x][y] *= (1 - PHEROMONE_EVAPORATION);
}

// ---------------------------------------------------------------
// diffusePheromone: standard 4-neighbour diffusion
// ---------------------------------------------------------------
void diffusePheromone() {
  float[][] temp = new float[PHEROMONE_COLS][PHEROMONE_ROWS];
  for (int x = 1; x < PHEROMONE_COLS - 1; x++)
    for (int y = 1; y < PHEROMONE_ROWS - 1; y++) {
      temp[x][y] = pheromone[x][y] * (1 - 4 * PHEROMONE_DIFFUSION)
          + (pheromone[x - 1][y] + pheromone[x + 1][y] + pheromone[x][y - 1] + pheromone[x][y + 1]) * PHEROMONE_DIFFUSION;
    }
  pheromone = temp;
}

// ---------------------------------------------------------------
// applyAco: per-boid ACO logic
// ---------------------------------------------------------------
void applyAco(Boid b) {
  // Map boid position to pheromone grid cell
  int px = int(b.pos.x / PHEROMONE_CELL);
  int py = int(b.pos.y / PHEROMONE_CELL);
  px = constrain(px, 0, PHEROMONE_COLS - 1);
  py = constrain(py, 0, PHEROMONE_ROWS - 1);

  // Deposit pheromone at current cell
  float deposit = PHEROMONE_DEPOSIT;
  if (b.carrying) {
    deposit *= 4; // heavy trail back to centre
  } else {
    // Bonus deposit near attractors to mark "food sources"
    for (PVector a : attractors) {
      float d = PVector.dist(b.pos, a);
      if (d < 200) deposit *= (200 - d) / 200 * 3;
    }
  }
  pheromone[px][py] = min(pheromone[px][py] + deposit, 100);

  // Read pheromone gradient (difference to each of the 8 neighbours)
  PVector gradient = new PVector();
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      int nx = constrain(px + dx, 0, PHEROMONE_COLS - 1);
      int ny = constrain(py + dy, 0, PHEROMONE_ROWS - 1);
      float diff = pheromone[nx][ny] - pheromone[px][py];
      gradient.add(new PVector(dx, dy).mult(diff));
    }
  }

  if (b.carrying) {
    // Returning to centre: follow gradient uphill toward centre emitter
    if (gradient.mag() > 0.1) {
      gradient.setMag(PHEROMONE_INFLUENCE * 2);
      b.acc.add(gradient);
    }
    b.acc.add(PVector.random2D().mult(0.15)); // slight wander for natural trails
  } else {
    // Searching: weak attraction to targets + explore via wander
    for (PVector a : attractors)
      b.linear_attraction(a, int(ATT_MULT * 0.25));
    // Follow pheromone to converge on food trails left by returning boids
    if (gradient.mag() > 0.1) {
      gradient.setMag(PHEROMONE_INFLUENCE * 1.2);
      b.acc.add(gradient);
    }
    b.acc.add(PVector.random2D().mult(0.4)); // random exploration
  }

  // No direct strong target attraction — boids find targets via pheromone trails

  // Repulsion from danger dots
  for (PVector r : repellents)
    b.simpleExponential_repulsion(r, PERLIMITER, REP_MULT);

  // Border repulsion
  for (PVector bp : border_points)
    b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

  // Inter-boid cohesion + repulsion
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
