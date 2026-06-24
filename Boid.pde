import java.util.ArrayList;

// ============================================================
// Boid
//
// The core agent in the swarm simulation. Each boid has
// position, velocity, acceleration, and state for all 10
// behaviour modes.
//
// Carrying mechanic: when a boid touches a green target
// (distance < 2 px) it picks it up (carrying = true) and
// must deliver it to the centre point (CENTER_X, CENTER_Y).
// While carrying it appears cyan and gets 10× attraction
// toward centre. At centre (distance < 10 px) the item is
// released.
//
// Flight trail: the last 15 positions are stored for a
// fading trail effect drawn behind the main dot.
//
// Death: boids have random health (1-10). Inside repulsion
// zones they lose health proportional to depth. At health
// ≤ 0 they're removed from the swarm.
//
// This file also contains all force functions used by the
// behaviour modules:
//   - AttRep: linear_attraction, simpleExponential_repulsion,
//     complexExponential_repulsion, comfy_attraction
//   - Context Steering: linear_Repulsion, log_Attraction,
//     linear_Attraction_CS, limExp_Repulsion, strongRepulsion,
//     strongAttraction, create_context_segment, context_steering
//   - BASIC (Reynolds): alignment, cohesion, separation, seek
// ============================================================
class Boid {
  // ----- Core physics -----
  PVector pos, vel, acc;
  float maxSpeed = 2.5;
  float maxForce = 0.08;
  float perception = 60; // Reynolds flocking neighbourhood radius

  // ----- PSO state -----
  PVector pbest;          // Personal best position (closest to target)
  float pbestFitness;     // Distance at personal best

  // ----- Morphogenetic swarm state -----
  float morphogen;        // Current morphogen concentration [0..2]
  float morphogenTarget;  // Random target level the morphogen tends toward

  // ----- SPP state -----
  float socialWeight;     // Social role weight [0..1], similar = attract, different = repel

  // ----- Mass / physics -----
  float mass;               // 0.5 (light/small/fast) to 2.0 (heavy/big/slow)
  boolean carrying = false; // True when boid holds a target item
  boolean dead = false;     // True when health ≤ 0 (removed next frame)

  // ----- Health -----
  float health;           // 1-10, depleted inside repulsion zones

  // ----- Flight trail -----
  ArrayList<PVector> trail = new ArrayList<PVector>();
  final int MAX_TRAIL = 15; // Number of past positions to draw

  // ----- AttRep force parameters -----
  // These constants are shared across several behaviour modes.
  float G = 1.98;
  final int ATT_MULT    = 128;
  final int REP_MULT    = 44;
  final int DRONE_ATT_MULT = 2;
  final int DRONE_REP_MULT = 20;
  final float COMFY_DIST    = 30;
  final float PERLIMITER        = 25;
  final float BORDER_PERLIMITER = 5;
  final float DRONE_PERLIMITER  = 10;

  // ----- Context Steering structures -----
  // 4 context maps (goals, dangers, att members, rep members), each
  // with one entry per direction sector. Used by CON_STEER and COMBINED.
  ArrayList<ArrayList<PVector>> contextMaps = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> currentForces   = new ArrayList<PVector>();
  ArrayList<PVector> alignmentForces = new ArrayList<PVector>();
  ArrayList<Boolean> memberMask = new ArrayList<Boolean>();
  PVector prevForce = new PVector();

  final float GOAL_MULT        = 1.75;
  final float DANGER_MULT      = 1.75;
  final float MEMBER_ATT_MULT  = 4;
  final float MEMBER_REP_MULT  = 8;

  final static int VISUAL_SCALE    = 2;
  final static int GOAL_VECTORS    = 0;
  final static int DANGER_VECTORS  = 1;
  final static int ATT_MEMBER_VECTORS = 2;
  final static int REP_MEMBER_VECTORS = 3;
  int DIRECTIONS; // Number of directional sectors (0 if not using context steering)

  // ----- Constructor (standard) -----
  Boid(float x, float y) {
    pos = new PVector(x, y);
    vel = PVector.random2D();
    acc = new PVector();
    pbest = new PVector(x, y);
    pbestFitness = Float.MAX_VALUE;
    morphogen = random(0.5, 1.0);
    morphogenTarget = random(0.2, 0.8);
    socialWeight = random(0, 1);
    health = random(1, 11);
    dead = false;
    mass = random(0.5, 2.0);
    maxSpeed = 2.5 / mass;
    maxForce = 0.08 / mass;
  }

  // ----- Constructor (with context steering initialisation) -----
  Boid(float x, float y, int directions) {
    this(x, y);
    initContextSteering(directions);
  }

  // ---------------------------------------------------------------
  // initContextSteering: allocate empty force arrays for each sector
  // ---------------------------------------------------------------
  void initContextSteering(int directions) {
    DIRECTIONS = directions;
    contextMaps.clear();
    alignmentForces.clear();
    currentForces.clear();
    memberMask.clear();
    prevForce = new PVector();

    ArrayList<PVector> goals       = new ArrayList<PVector>();
    ArrayList<PVector> att_members = new ArrayList<PVector>();
    ArrayList<PVector> rep_members = new ArrayList<PVector>();
    ArrayList<PVector> dangers     = new ArrayList<PVector>();

    for (int it = 0; it < DIRECTIONS; ++it) {
      goals.add(new PVector());
      dangers.add(new PVector());
      att_members.add(new PVector());
      rep_members.add(new PVector());
      memberMask.add(true);
    }

    contextMaps.add(goals);
    contextMaps.add(dangers);
    contextMaps.add(att_members);
    contextMaps.add(rep_members);
  }

  // ---------------------------------------------------------------
  // update: BASIC mode Reynolds flocking
  //   - alignment, cohesion, separation
  //   - repulsion zone avoidance
  //   - velocity integration + edges wrap
  // ---------------------------------------------------------------
  void update(ArrayList<Boid> boids, ArrayList<RepulsionZone> zones) {
    PVector align = alignment(boids);
    PVector coh   = cohesion(boids).mult(0.8);
    PVector sep   = separation(boids).mult(1.5);

    acc.add(align);
    acc.add(coh);
    acc.add(sep);

    for (RepulsionZone z : zones) {
      float d = PVector.dist(pos, z.pos);
      if (d < z.radius * 1.5) {
        PVector dir = PVector.sub(pos, z.pos);
        dir.normalize();
        float strength = (z.radius * 1.5 - d) / (z.radius * 1.5) * maxForce * 5;
        dir.mult(strength);
        acc.add(dir);
      }
    }

    integrate();

    edges();
  }

  // ----- Reynolds flocking helpers -----

  PVector alignment(ArrayList<Boid> boids) {
    PVector steering = new PVector();
    int total = 0;
    for (Boid other : boids) {
      float d = dist(pos.x, pos.y, other.pos.x, other.pos.y);
      if (other != this && d < perception) {
        steering.add(other.vel);
        total++;
      }
    }
    if (total > 0) {
      steering.div(total);
      steering.setMag(maxSpeed);
      steering.sub(vel);
    }
    return steering;
  }

  PVector cohesion(ArrayList<Boid> boids) {
    PVector center = new PVector();
    int total = 0;
    for (Boid other : boids) {
      float d = dist(pos.x, pos.y, other.pos.x, other.pos.y);
      if (other != this && d < perception) {
        center.add(other.pos);
        total++;
      }
    }
    if (total > 0) {
      center.div(total);
      return seek(center);
    }
    return new PVector();
  }

  PVector separation(ArrayList<Boid> boids) {
    PVector steering = new PVector();
    int total = 0;
    for (Boid other : boids) {
      float d = dist(pos.x, pos.y, other.pos.x, other.pos.y);
      if (other != this && d < perception / 2) {
        PVector diff = PVector.sub(pos, other.pos);
        diff.div(d * d);
        steering.add(diff);
        total++;
      }
    }
    if (total > 0) {
      steering.div(total);
      steering.setMag(maxSpeed);
      steering.sub(vel);
    }
    return steering;
  }

  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, pos);
    desired.setMag(maxSpeed);
    PVector steer = PVector.sub(desired, vel);
    return steer.limit(maxForce);
  }

  // ---------------------------------------------------------------
  // edges: toroidal wrap — boids that leave one side reappear on
  // the opposite side.
  // ---------------------------------------------------------------
  void edges() {
    if (pos.x > width)  pos.x = 0;
    if (pos.x < 0)      pos.x = width;
    if (pos.y > height) pos.y = 0;
    if (pos.y < 0)      pos.y = height;
  }

  // ---------------------------------------------------------------
  // integrate: apply acceleration scaled by mass, cap speed, move,
  // then clear acceleration for the next frame.
  //   vel += acc / mass   (F = ma → a = F/m)
  // ---------------------------------------------------------------
  void integrate() {
    vel.add(PVector.div(acc, mass));
    vel.limit(maxSpeed);
    pos.add(vel);
    acc.mult(0);
  }

  // ---------------------------------------------------------------
  // recordTrail: store current position for the flight trail
  // ---------------------------------------------------------------
  void recordTrail() {
    trail.add(pos.copy());
    while (trail.size() > MAX_TRAIL) trail.remove(0);
  }

  // ---------------------------------------------------------------
  // display: render the boid as a dot + fading trail
  //   - Dot size = 3 + mass * 2 (mass 0.5 → 4, mass 2.0 → 7)
  //   - Cyan when carrying, white otherwise
  //   - Fading trail (last 15 positions, alpha/size gradient)
  // ---------------------------------------------------------------
  void display() {
    int sz = trail.size();
    for (int i = 1; i < sz; i++) {
      float frac = float(i) / sz;
      stroke(180, 180, 200, frac * 180);
      strokeWeight(frac * 2.5 * mass);
      line(trail.get(i - 1).x, trail.get(i - 1).y, trail.get(i).x, trail.get(i).y);
    }
    float dotSize = 3 + mass * 2;
    strokeWeight(dotSize);
    if (carrying) stroke(0, 255, 255);
    else          stroke(200);
    point(pos.x, pos.y);
  }

  // ============================================================
  // Force functions — used by the behaviour modules
  // ============================================================

  // ----- AttRep forces -----

  // Inverse-square attraction beyond perlimiter
  void primitive_attraction(PVector target, float perlimiter, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 1, 25);
    if (d > perlimiter) {
      float strength = (G * multiplier) / (d * d);
      force.setMag(strength);
      this.acc.add(force);
    }
  }

  // Inverse-square repulsion within perlimiter
  void primitive_repulsion(PVector target, float perlimiter, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = constrain(d, 1, 25);
    if (d < perlimiter) {
      float strength = (G * multiplier) / (d * d);
      force.setMag(strength);
      this.acc.sub(force);
    }
  }

  // 1/r attraction — decays linearly with distance
  void linear_attraction(PVector target, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = max(d, 1);
    float strength = G * multiplier / d;
    force.setMag(strength);
    this.acc.add(force);
  }

  // Gaussian repulsion — smooth exponential falloff, varies decay width
  void complexExponential_repulsion(PVector target, float perlimiter, int a, int b) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float c = (perlimiter * perlimiter) * (float) Math.log(b / a);
    float strength = (G * b) * (float) Math.exp(-(d * d) / c);
    force.setMag(strength);
    this.acc.sub(force);
  }

  // Linear attraction with a dead zone — zero within perlimiter, proportional to excess beyond
  void comfy_attraction(PVector target, float perlimiter, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G * multiplier) * (d - perlimiter) / max(d, 0.01f);
    force.setMag(strength);
    this.acc.add(force);
  }

  // Gaussian repulsion — simple single-parameter falloff
  void simpleExponential_repulsion(PVector target, float perlimiter, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G * multiplier) * (float) Math.exp(-(d * d) / (2 * perlimiter * perlimiter));
    force.setMag(strength);
    this.acc.sub(force);
  }

  // ----- Context Steering forces -----

  // Linear sector repulsion: rises until limit then saturates
  PVector linear_Repulsion(PVector target, float limit, float sigma, float gamma, float alpha) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = max((sigma / (d + alpha) - gamma), 0) * (-1);
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  // Strong repulsion (combined mode): inverse-square at close range
  PVector strongRepulsion(PVector target, float limit, float scale) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = -scale / (d * d + 0.5);
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  // Strong attraction (combined mode): linear, grows with distance, has dead zone
  PVector strongAttraction(PVector target, float limit, float scale, float deadzone) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = scale * max(d - deadzone, 0);
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  // Logarithmic attraction beyond a cutoff distance
  PVector log_Attraction(PVector target, float limit, float cutOff) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength;
    if (d > cutOff) strength = log(d - (cutOff - G / 2)) * G;
    else            strength = 0;
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  // Linear attraction with slope sigma and offset gamma
  PVector linear_Attraction_CS(PVector target, float limit, float sigma, float gamma) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = (G / sigma) * d + gamma;
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  // Limited exponential repulsion: piecewise linear + exponential
  PVector limExp_Repulsion(PVector target, float limit, float cutOff, float sigma, float gamma, float alpha) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength;
    if (d < cutOff) strength = G * (d / 2 - limit) / sigma;
    else            strength = -1 * exp(-1 * (G * (alpha * log(d) - gamma) / sigma));
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  // ---------------------------------------------------------------
  // create_context_segment: store the strongest force of each type
  // in this directional sector for context steering.
  // ---------------------------------------------------------------
  void create_context_segment(int dir, ArrayList<PVector> intrest_forces, ArrayList<PVector> danger_forces,
      ArrayList<PVector> member_atts, ArrayList<PVector> member_reps,
      ArrayList<PVector> alignment_forces, boolean mask_val) {
    PVector strongest_force;
    this.memberMask.set(dir, mask_val);
    for (PVector force : alignment_forces)
      this.alignmentForces.add(force);

    strongest_force = new PVector();
    for (PVector f : intrest_forces)
      if (strongest_force.mag() < f.mag()) strongest_force = f;
    this.contextMaps.get(GOAL_VECTORS).set(dir, strongest_force.copy());

    strongest_force = new PVector();
    for (PVector f : danger_forces)
      if (strongest_force.mag() < f.mag()) strongest_force = f;
    this.contextMaps.get(DANGER_VECTORS).set(dir, strongest_force.copy());

    strongest_force = new PVector();
    for (PVector f : member_atts)
      if (strongest_force.mag() < f.mag()) strongest_force = f;
    this.contextMaps.get(ATT_MEMBER_VECTORS).set(dir, strongest_force.copy());

    strongest_force = new PVector();
    for (PVector f : member_reps)
      if (strongest_force.mag() < f.mag()) strongest_force = f;
    this.contextMaps.get(REP_MEMBER_VECTORS).set(dir, strongest_force.copy());
  }

  // ---------------------------------------------------------------
  // context_steering: resolve forces from all sectors into a single
  // steering direction. Picks the sector with highest net force,
  // blends with the neighbouring sector, and constrains alignment
  // to the velocity direction.
  // ---------------------------------------------------------------
  void context_steering(ArrayList<PVector> rayDirs, float sectorCosSim) {
    PVector alignment = new PVector();
    for (PVector force : this.alignmentForces)
      alignment.add(force);
    alignment.add(this.vel);
    alignment.div(this.alignmentForces.size() + 1);

    ArrayList<PVector> forces      = new ArrayList<PVector>();
    ArrayList<PVector> visualForces = new ArrayList<PVector>();
    ArrayList<Boolean> mask        = new ArrayList<Boolean>();

    for (int idx = 0; idx < this.DIRECTIONS; ++idx) {
      PVector force = new PVector();
      force.add(this.contextMaps.get(GOAL_VECTORS).get(idx).mult(this.GOAL_MULT));
      force.add(this.contextMaps.get(ATT_MEMBER_VECTORS).get(idx).mult(this.MEMBER_ATT_MULT));
      force.add(this.contextMaps.get(REP_MEMBER_VECTORS).get(idx).mult(this.MEMBER_REP_MULT));
      force.add(this.contextMaps.get(DANGER_VECTORS).get(idx).mult(this.DANGER_MULT));

      // Mask sectors where force points backward or the sector was marked blocked
      if ((cosine_sim(rayDirs.get(idx), force) < 0.0) || !memberMask.get(idx))
        mask.add(false);
      else
        mask.add(true);

      // Reduce force magnitude when alignment direction disagrees with this sector
      float alignSim    = cosine_sim(rayDirs.get(idx), alignment);
      float constrainSim = max(sectorCosSim, cosine_sim(alignment, this.vel));
      if (alignSim < constrainSim)
        force.mult(map(alignSim, -1.0, constrainSim, 0.25, 1.0));

      forces.add(force.copy());
      if (mask.get(mask.size() - 1))
        visualForces.add(force.copy());
      else
        visualForces.add(new PVector());
    }
    this.currentForces = visualForces;

    // Find the unmasked sector with the strongest force
    int maxIdx = 0;
    while (maxIdx < forces.size() && !mask.get(maxIdx)) ++maxIdx;
    for (int idx = maxIdx + 1; idx < forces.size(); ++idx)
      if (mask.get(idx) && (forces.get(idx).mag() > forces.get(maxIdx).mag()))
        maxIdx = idx;

    PVector main_force = new PVector();
    if (maxIdx < forces.size())
      main_force = PVector.mult(rayDirs.get(maxIdx), maxSpeed);

    // Blend with the strongest neighbour sector for smoother steering
    PVector total_force = main_force.copy();
    if (maxIdx < forces.size() && forces.get(maxIdx).mag() > 1) {
      int leftIdx  = (maxIdx - 1 + this.DIRECTIONS) % this.DIRECTIONS;
      int rightIdx = (maxIdx + 1 + this.DIRECTIONS) % this.DIRECTIONS;
      float leftMag  = forces.get(leftIdx).mag();
      float rightMag = forces.get(rightIdx).mag();
      int neighborIdx = (leftMag < rightMag && mask.get(rightIdx)) ? rightIdx
          : (mask.get(leftIdx)) ? leftIdx : -1;
      if (neighborIdx >= 0) {
        float magnitude = forces.get(neighborIdx).mag() / forces.get(maxIdx).mag();
        PVector secondary_force = PVector.mult(rayDirs.get(neighborIdx), maxSpeed * magnitude);
        total_force.add(secondary_force);
      }
    } else if (maxIdx >= forces.size() || forces.get(maxIdx).mag() < 0.1) {
      this.vel.mult(0);
    }

    total_force.setMag(maxSpeed);
    this.prevForce = total_force.copy();
    this.acc.add(total_force);
  }
}
