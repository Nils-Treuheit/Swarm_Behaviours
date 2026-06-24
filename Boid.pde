import java.util.ArrayList;

class Boid {
  PVector pos, vel, acc;

  float maxSpeed = 2.5;
  float maxForce = 0.08;

  float perception = 60;

  // PSO state
  PVector pbest;
  float pbestFitness;

  // Morphogenetic swarm state
  float morphogen;
  float morphogenTarget;

  // SPP state
  float socialWeight;

  // Carry state
  boolean carrying = false;

  // Flight trail
  ArrayList<PVector> trail = new ArrayList<PVector>();
  final int MAX_TRAIL = 15;

  // AttRep behaviour force parameters
  float G = 1.98;
  final int ATT_MULT = 128;
  final int REP_MULT = 44;
  final int DRONE_ATT_MULT = 2;
  final int DRONE_REP_MULT = 20;
  final float COMFY_DIST = 30;
  final float PERLIMITER = 25;
  final float BORDER_PERLIMITER = 5;
  final float DRONE_PERLIMITER = 10;

  // Context steering structures
  ArrayList<ArrayList<PVector>> contextMaps = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> currentForces = new ArrayList<PVector>();
  ArrayList<PVector> alignmentForces = new ArrayList<PVector>();
  ArrayList<Boolean> memberMask = new ArrayList<Boolean>();
  PVector prevForce = new PVector();
  final float GOAL_MULT = 1.75;
  final float DANGER_MULT = 1.75;
  final float MEMBER_ATT_MULT = 4;
  final float MEMBER_REP_MULT = 8;
  final static int VISUAL_SCALE = 2;
  final static int GOAL_VECTORS = 0;
  final static int DANGER_VECTORS = 1;
  final static int ATT_MEMBER_VECTORS = 2;
  final static int REP_MEMBER_VECTORS = 3;
  int DIRECTIONS;

  Boid(float x, float y) {
    pos = new PVector(x, y);
    vel = PVector.random2D();
    acc = new PVector();
    pbest = new PVector(x, y);
    pbestFitness = Float.MAX_VALUE;
    morphogen = random(0.5, 1.0);
    morphogenTarget = random(0.2, 0.8);
    socialWeight = random(0, 1);
  }

  Boid(float x, float y, int directions) {
    this(x, y);
    initContextSteering(directions);
  }

  void initContextSteering(int directions) {
    DIRECTIONS = directions;
    contextMaps.clear();
    alignmentForces.clear();
    currentForces.clear();
    memberMask.clear();
    prevForce = new PVector();

    ArrayList<PVector> goals = new ArrayList<PVector>();
    ArrayList<PVector> att_members = new ArrayList<PVector>();
    ArrayList<PVector> rep_members = new ArrayList<PVector>();
    ArrayList<PVector> dangers = new ArrayList<PVector>();

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

  void update(ArrayList<Boid> boids, ArrayList<RepulsionZone> zones) {
    PVector align = alignment(boids);
    PVector coh = cohesion(boids).mult(0.8);
    PVector sep = separation(boids).mult(1.5);

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

    vel.add(acc);
    vel.limit(maxSpeed);
    pos.add(vel);
    acc.mult(0);

    edges();
  }

  void applyForce(PVector f) {
    acc.add(f.limit(maxForce));
  }

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

  PVector repulsion(ArrayList<RepulsionZone> zones) {
    PVector force = new PVector();
    for (RepulsionZone z : zones) {
      float d = PVector.dist(pos, z.pos);
      if (d < z.radius) {
        PVector dir = PVector.sub(pos, z.pos);
        dir.normalize();
        dir.mult((z.radius - d) * 0.05);
        force.add(dir);
      }
    }
    return force;
  }

  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, pos);
    desired.setMag(maxSpeed);
    PVector steer = PVector.sub(desired, vel);
    return steer.limit(maxForce);
  }

  void edges() {
    if (pos.x > width) pos.x = 0;
    if (pos.x < 0) pos.x = width;
    if (pos.y > height) pos.y = 0;
    if (pos.y < 0) pos.y = height;
  }

  void recordTrail() {
    trail.add(pos.copy());
    while (trail.size() > MAX_TRAIL) trail.remove(0);
  }

  void display() {
    // Flight trail
    int sz = trail.size();
    for (int i = 1; i < sz; i++) {
      float frac = float(i) / sz;
      stroke(180, 180, 200, frac * 180);
      strokeWeight(frac * 2.5);
      line(trail.get(i - 1).x, trail.get(i - 1).y, trail.get(i).x, trail.get(i).y);
    }
    strokeWeight(1);

    if (carrying) stroke(0, 255, 255);
    else stroke(200);
    point(pos.x, pos.y);
  }

  // ----- Advanced AttRep forces -----

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

  void linear_attraction(PVector target, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    d = max(d, 1);
    float strength = G * multiplier / d;
    force.setMag(strength);
    this.acc.add(force);
  }

  void complexExponential_repulsion(PVector target, float perlimiter, int a, int b) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float c = (perlimiter * perlimiter) * (float) Math.log(b / a);
    float strength = (G * b) * (float) Math.exp(-(d * d) / c);
    force.setMag(strength);
    this.acc.sub(force);
  }

  void comfy_attraction(PVector target, float perlimiter, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G * multiplier) * (d - perlimiter) / max(d, 0.01f);
    force.setMag(strength);
    this.acc.add(force);
  }

  void simpleExponential_repulsion(PVector target, float perlimiter, int multiplier) {
    PVector force = PVector.sub(target, this.pos);
    float d = force.mag();
    float strength = (G * multiplier) * (float) Math.exp(-(d * d) / (2 * perlimiter * perlimiter));
    force.setMag(strength);
    this.acc.sub(force);
  }

  // ----- Context Steering forces -----

  PVector linear_Repulsion(PVector target, float limit, float sigma, float gamma, float alpha) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = max((sigma / (d + alpha) - gamma), 0) * (-1);
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  // Strong repulsion for combined mode: inverse-square at close range
  PVector strongRepulsion(PVector target, float limit, float scale) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = -scale / (d * d + 0.5);
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  // Strong attraction for combined mode: linear, grows with distance
  PVector strongAttraction(PVector target, float limit, float scale, float deadzone) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = scale * max(d - deadzone, 0);
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  PVector log_Attraction(PVector target, float limit, float cutOff) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength;
    if (d > cutOff) strength = log(d - (cutOff - G / 2)) * G;
    else strength = 0;
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  PVector linear_Attraction_CS(PVector target, float limit, float sigma, float gamma) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    d = min(d, limit);
    float strength = (G / sigma) * d + gamma;
    force.normalize();
    force.mult(strength);
    return force.copy();
  }

  PVector limExp_Repulsion(PVector target, float limit, float cutOff, float sigma, float gamma, float alpha) {
    PVector force = PVector.sub(target, this.pos).mult(0.06);
    float d = force.mag();
    float strength;
    d = min(d, limit);
    if (d < cutOff) strength = G * (d / 2 - limit) / sigma;
    else strength = -1 * exp(-1 * (G * (alpha * log(d) - gamma) / sigma));
    force = force.normalize();
    force = force.mult(strength);
    return force.copy();
  }

  void create_context_segment(int dir, ArrayList<PVector> intrest_forces, ArrayList<PVector> danger_forces,
      ArrayList<PVector> member_atts, ArrayList<PVector> member_reps,
      ArrayList<PVector> alignment_forces, boolean mask_val) {
    PVector strongest_force;
    this.memberMask.set(dir, mask_val);
    for (PVector force : alignment_forces)
      this.alignmentForces.add(force);

    strongest_force = new PVector();
    for (PVector intrest_force : intrest_forces)
      if (strongest_force.mag() < intrest_force.mag())
        strongest_force = intrest_force;
    this.contextMaps.get(GOAL_VECTORS).set(dir, strongest_force.copy());
    strongest_force = new PVector();
    for (PVector danger_force : danger_forces)
      if (strongest_force.mag() < danger_force.mag())
        strongest_force = danger_force;
    this.contextMaps.get(DANGER_VECTORS).set(dir, strongest_force.copy());

    strongest_force = new PVector();
    for (PVector member_att : member_atts)
      if (strongest_force.mag() < member_att.mag())
        strongest_force = member_att;
    this.contextMaps.get(ATT_MEMBER_VECTORS).set(dir, strongest_force.copy());
    strongest_force = new PVector();
    for (PVector member_rep : member_reps)
      if (strongest_force.mag() < member_rep.mag())
        strongest_force = member_rep;
    this.contextMaps.get(REP_MEMBER_VECTORS).set(dir, strongest_force.copy());
  }

  void context_steering(ArrayList<PVector> rayDirs, float sectorCosSim) {
    PVector alignment = new PVector();
    for (PVector force : this.alignmentForces)
      alignment.add(force);
    alignment.add(this.vel);
    alignment.div(this.alignmentForces.size() + 1);

    ArrayList<PVector> forces = new ArrayList<PVector>();
    ArrayList<PVector> visualForces = new ArrayList<PVector>();
    ArrayList<Boolean> mask = new ArrayList<Boolean>();
    for (int idx = 0; idx < this.DIRECTIONS; ++idx) {
      PVector force = new PVector();
      force.add(this.contextMaps.get(GOAL_VECTORS).get(idx).mult(this.GOAL_MULT));
      force.add(this.contextMaps.get(ATT_MEMBER_VECTORS).get(idx).mult(this.MEMBER_ATT_MULT));
      force.add(this.contextMaps.get(REP_MEMBER_VECTORS).get(idx).mult(this.MEMBER_REP_MULT));
      force.add(this.contextMaps.get(DANGER_VECTORS).get(idx).mult(this.DANGER_MULT));

      if ((cosine_sim(rayDirs.get(idx), force) < 0.0) || !memberMask.get(idx))
        mask.add(false);
      else
        mask.add(true);

      float alignSim = cosine_sim(rayDirs.get(idx), alignment);
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

    PVector main_force = new PVector();
    int maxIdx = 0;
    while ((maxIdx < forces.size()) && !mask.get(maxIdx))
      ++maxIdx;
    for (int idx = maxIdx + 1; idx < forces.size(); ++idx)
      if (mask.get(idx) && (forces.get(idx).mag() > forces.get(maxIdx).mag()))
        maxIdx = idx;
    if (maxIdx < forces.size())
      main_force = PVector.mult(rayDirs.get(maxIdx), maxSpeed);
    if (false)
      System.out.println("Main Direction: " + maxIdx);

    PVector total_force = main_force.copy();
    if (maxIdx < forces.size() && forces.get(maxIdx).mag() > 1) {
      int leftIdx = (maxIdx - 1 + this.DIRECTIONS) % this.DIRECTIONS;
      int rightIdx = (maxIdx + 1 + this.DIRECTIONS) % this.DIRECTIONS;
      float leftMag = forces.get(leftIdx).mag();
      float rightMag = forces.get(rightIdx).mag();
      int neighborIdx = (leftMag < rightMag && mask.get(rightIdx)) ? rightIdx
          : (mask.get(leftIdx)) ? leftIdx : -1;
      if (false)
        System.out.println("Neighbor Direction:" + neighborIdx);
      if (neighborIdx >= 0) {
        float magnitude = forces.get(neighborIdx).mag() / forces.get(maxIdx).mag();
        PVector secondary_force = PVector.mult(rayDirs.get(neighborIdx), maxSpeed * magnitude);
        total_force.add(secondary_force);
      }
    } else if (maxIdx >= forces.size() || forces.get(maxIdx).mag() < 0.1)
      this.vel.mult(0);

    total_force.setMag(maxSpeed);
    this.prevForce = total_force.copy();
    this.acc.add(total_force);
  }
}
