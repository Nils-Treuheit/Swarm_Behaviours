import java.util.ArrayList;

enum BehaviourFlag {
  BASIC,
  ATT_REP,
  CON_STEER,
  COMBINED,
  PSO,
  CUCKER_SMALE,
  VICSEK,
  MORPHOGENETIC,
  ACO,
  SPP
}

class SwarmManager {
  ArrayList<Boid> boids = new ArrayList<Boid>();
  ArrayList<Goal> goals = new ArrayList<Goal>();
  ArrayList<RepulsionZone> zones = new ArrayList<RepulsionZone>();

  int targetCount = 10;
  BehaviourFlag behaviourFlag = BehaviourFlag.ATT_REP;

  // AttRep behaviour data
  ArrayList<PVector> attractors = new ArrayList<PVector>();
  ArrayList<PVector> repellents = new ArrayList<PVector>();
  ArrayList<PVector> border_points = new ArrayList<PVector>();

  // ConSteer behaviour data
  ArrayList<PVector> goalsCS = new ArrayList<PVector>();
  ArrayList<PVector> dangersCS = new ArrayList<PVector>();
  ArrayList<PVector> RAY_DIRS = new ArrayList<PVector>();
  float SECTOR_COS_SIM;

  // HUD safe zone – no points auto-spawn here
  final float HUD_W = 800;
  final float HUD_H = 640;

  PVector randomOutsideHud() {
    float x, y;
    do {
      x = random(4, width - 4);
      y = random(4, height - 4);
    } while (x < HUD_W && y < HUD_H);
    return new PVector(x, y);
  }

  // Auto-update tick system
  int ticks = 0;
  final int MAX_TICKS = 450;
  final int UPDATE_PORTION = 2;
  final float PIXEL_METRIC_CONV = 0.06;

  // ConSteer constants
  final float BORDER_TO_CLOSE = 2;
  final float DANGER_TO_CLOSE = 8;
  final float MEMBER_TO_CLOSE = 4;
  final float SWARM_DIST = 12;
  final float GOAL_LIMIT = 30;
  final float GOAL_SIGMA = -3;
  final float GOAL_GAMMA = 25;
  final float DANGER_LIMIT = 40;
  final float DANGER_CUT_OFF = 5;
  final float DANGER_SIGMA = 3.85;
  final float DANGER_GAMMA = 18.5;
  final float DANGER_ALPHA = 20;
  final float MEMBER_REP_LIMIT = 30;
  final float MEMBER_REP_SIGMA = 18;
  final float MEMBER_REP_GAMMA = 1.5;
  final float MEMBER_REP_ALPHA = 0.1;
  final float MEMBER_ATT_LIMIT = 30;
  final float MEMBER_ATT_CUT_OFF = 22;

  // Combined mode constants
  final float ZONE_DANGER_LIMIT = 100;
  final float ZONE_TO_CLOSE = 10;
  final float COMBINED_REP_SCALE = 200;
  final float COMBINED_REP_LIMIT = 30;
  final float COMBINED_ATT_SCALE = 3.0;
  final float COMBINED_ATT_DEADZONE = 3.0;
  final float COMBINED_ATT_LIMIT = 50;
  final float COMBINED_MEMBER_TO_CLOSE = 3.0;
  final float FOLLOW_SCALE = 5.0;
  final float FOLLOW_DEADZONE = 4.0;
  final float FOLLOW_LIMIT = 60;

  // PSO state
  PVector gbest = new PVector();
  float gbestFitness = Float.MAX_VALUE;
  final float PSO_INERTIA = 0.7;
  final float PSO_COGNITIVE = 1.5;
  final float PSO_SOCIAL = 1.5;

  // Cucker-Smale constants
  final float CUCKER_H = 80.0;
  final float CUCKER_ATTR_SCALE = 0.5;
  final float CUCKER_ATTR_LIMIT = 40;

  // Vicsek constants
  final float VICSEK_RADIUS = 120;
  final float VICSEK_NOISE = 0.15;
  final float VICSEK_SPEED = 3.0;

  // Morphogenetic constants
  final float MORPH_COMM_RADIUS = 150;
  final float MORPH_DECAY = 0.98;
  final float MORPH_DIFFUSION = 0.05;
  final float MORPH_PRODUCTION = 0.01;
  final float MORPH_ATTRACT_SCALE = 80;

  // ACO constants
  final int PHEROMONE_CELL = 50;
  int PHEROMONE_COLS;
  int PHEROMONE_ROWS;
  float[][] pheromone;
  final float PHEROMONE_EVAPORATION = 0.005;
  final float PHEROMONE_DIFFUSION = 0.02;
  final float PHEROMONE_DEPOSIT = 5;
  final float PHEROMONE_INFLUENCE = 3;
  final float PHEROMONE_SENSING_RADIUS = 50;

  // SPP constants
  final float SPP_WEIGHT_THRESHOLD = 0.15;
  final float SPP_ATTRACT_SCALE = 0.05;
  final float SPP_REPEL_SCALE = 0.08;
  final float SPP_MAX_FORCE = 0.5;

  Boid findLeader() {
    Boid leader = null;
    float minDist = Float.MAX_VALUE;
    for (Boid b : boids) {
      for (PVector a : attractors) {
        float d = PVector.dist(b.pos, a);
        if (d < minDist) { minDist = d; leader = b; }
      }
    }
    return leader;
  }

  // AttRep constants
  final int ATT_MULT = 128;
  final int REP_MULT = 44;
  final int DRONE_ATT_MULT = 2;
  final int DRONE_REP_MULT = 20;
  final float COMFY_DIST = 30;
  PVector mouseTarget = null;
  PVector mouseDanger = null;

  final float PERLIMITER = 25;
  final float BORDER_PERLIMITER = 5;
  final float DRONE_PERLIMITER = 10;

  void initPheromone() {
    PHEROMONE_COLS = width / PHEROMONE_CELL + 1;
    PHEROMONE_ROWS = height / PHEROMONE_CELL + 1;
    pheromone = new float[PHEROMONE_COLS][PHEROMONE_ROWS];
  }

  void evaporatePheromone() {
    for (int x = 0; x < PHEROMONE_COLS; x++)
      for (int y = 0; y < PHEROMONE_ROWS; y++)
        pheromone[x][y] *= (1 - PHEROMONE_EVAPORATION);
  }

  void diffusePheromone() {
    float[][] temp = new float[PHEROMONE_COLS][PHEROMONE_ROWS];
    for (int x = 1; x < PHEROMONE_COLS - 1; x++)
      for (int y = 1; y < PHEROMONE_ROWS - 1; y++) {
        temp[x][y] = pheromone[x][y] * (1 - 4 * PHEROMONE_DIFFUSION)
            + (pheromone[x - 1][y] + pheromone[x + 1][y] + pheromone[x][y - 1] + pheromone[x][y + 1]) * PHEROMONE_DIFFUSION;
      }
    pheromone = temp;
  }

  void initGbest() {
    gbestFitness = Float.MAX_VALUE;
    gbest = new PVector();
  }

  SwarmManager() {
    initGbest();
    initPheromone();
    for (int x = 0; x < width; ++x) {
      border_points.add(new PVector(x, 0));
      border_points.add(new PVector(x, height));
    }
    for (int y = 0; y < height; ++y) {
      border_points.add(new PVector(0, y));
      border_points.add(new PVector(width, y));
    }
  }

  void initBehaviour(BehaviourFlag flag) {
    this.behaviourFlag = flag;
    this.ticks = 0;
    clearAll();
    initGbest();

    // Reset PSO personal bests for all boids
    for (Boid b : boids) {
      b.pbest = b.pos.copy();
      b.pbestFitness = Float.MAX_VALUE;
    }

    if (flag == BehaviourFlag.CON_STEER || flag == BehaviourFlag.COMBINED) {
      int directions = 8;
      RAY_DIRS.clear();
      for (int it = 0; it < directions; ++it) {
        float angle = it * TWO_PI / directions;
        RAY_DIRS.add(PVector.fromAngle(angle));
      }
      SECTOR_COS_SIM = cos(PI / directions);

      for (Boid b : boids) {
        if (b.DIRECTIONS == 0) b.initContextSteering(directions);
      }
    }

    // Reset ACO pheromone grid
    if (flag == BehaviourFlag.ACO) initPheromone();

    if (flag == BehaviourFlag.BASIC || flag == BehaviourFlag.ATT_REP
        || flag == BehaviourFlag.CON_STEER || flag == BehaviourFlag.COMBINED
        || flag == BehaviourFlag.PSO || flag == BehaviourFlag.CUCKER_SMALE
        || flag == BehaviourFlag.VICSEK || flag == BehaviourFlag.MORPHOGENETIC
        || flag == BehaviourFlag.ACO || flag == BehaviourFlag.SPP) {
      setupAutoPoints();
    }
  }

  void setupAutoPoints() {
    if (behaviourFlag == BehaviourFlag.BASIC) {
      for (int it = 0; it < 8; ++it)
        goals.add(new Goal(randomOutsideHud().x, randomOutsideHud().y, random(30, 80)));
      for (int it = 0; it < 4; ++it)
        zones.add(new RepulsionZone(randomOutsideHud().x, randomOutsideHud().y, random(80, 200)));
    } else if (behaviourFlag == BehaviourFlag.ATT_REP || behaviourFlag == BehaviourFlag.PSO) {
      for (int it = 0; it < 8; ++it)
        attractors.add(randomOutsideHud());
      for (int it = 0; it < 4; ++it)
        repellents.add(randomOutsideHud());
    } else if (behaviourFlag == BehaviourFlag.CON_STEER
        || behaviourFlag == BehaviourFlag.CUCKER_SMALE
        || behaviourFlag == BehaviourFlag.VICSEK
        || behaviourFlag == BehaviourFlag.SPP) {
      for (int it = 0; it < 8; ++it)
        goalsCS.add(randomOutsideHud());
      for (int it = 0; it < 4; ++it)
        dangersCS.add(randomOutsideHud());
    } else if (behaviourFlag == BehaviourFlag.COMBINED) {
      for (int it = 0; it < 8; ++it)
        attractors.add(randomOutsideHud());
      for (int it = 0; it < 4; ++it)
        zones.add(new RepulsionZone(randomOutsideHud().x, randomOutsideHud().y, random(80, 200)));
    }
  }

  void autoUpdate(int portion) {
    if (ticks >= MAX_TICKS) {
      ticks = 0;
      if (behaviourFlag == BehaviourFlag.BASIC) {
        for (int it = 0; it < max(goals.size() / portion, 1); ++it) {
          if (!goals.isEmpty()) goals.remove(0);
          goals.add(new Goal(randomOutsideHud().x, randomOutsideHud().y, random(30, 80)));
        }
        for (int it = 0; it < max(zones.size() / portion, 1); ++it) {
          if (!zones.isEmpty()) zones.remove(0);
          zones.add(new RepulsionZone(randomOutsideHud().x, randomOutsideHud().y, random(80, 200)));
        }
    } else if (behaviourFlag == BehaviourFlag.ATT_REP || behaviourFlag == BehaviourFlag.PSO
        || behaviourFlag == BehaviourFlag.MORPHOGENETIC || behaviourFlag == BehaviourFlag.ACO) {
        for (int it = 0; it < max(attractors.size() / portion, 1); ++it) {
          if (!attractors.isEmpty()) attractors.remove(0);
          attractors.add(randomOutsideHud());
        }
        for (int it = 0; it < max(repellents.size() / portion, 1); ++it) {
          if (!repellents.isEmpty()) repellents.remove(0);
          repellents.add(randomOutsideHud());
        }
      } else if (behaviourFlag == BehaviourFlag.CON_STEER
          || behaviourFlag == BehaviourFlag.CUCKER_SMALE
          || behaviourFlag == BehaviourFlag.VICSEK
          || behaviourFlag == BehaviourFlag.SPP) {
        for (int it = 0; it < max(goalsCS.size() / portion, 1); ++it) {
          if (!goalsCS.isEmpty()) goalsCS.remove(0);
          goalsCS.add(randomOutsideHud());
        }
        for (int it = 0; it < max(dangersCS.size() / portion, 1); ++it) {
          if (!dangersCS.isEmpty()) dangersCS.remove(0);
          dangersCS.add(randomOutsideHud());
        }
      } else if (behaviourFlag == BehaviourFlag.COMBINED) {
        for (int it = 0; it < max(attractors.size() / portion, 1); ++it) {
          if (!attractors.isEmpty()) attractors.remove(0);
          attractors.add(randomOutsideHud());
        }
        for (int it = 0; it < max(zones.size() / portion, 1); ++it) {
          if (!zones.isEmpty()) zones.remove(0);
          zones.add(new RepulsionZone(randomOutsideHud().x, randomOutsideHud().y, random(80, 200)));
        }
      }
    }
    ++ticks;
  }

  void removeVisited() {
    if (behaviourFlag == BehaviourFlag.ATT_REP
        || behaviourFlag == BehaviourFlag.COMBINED
        || behaviourFlag == BehaviourFlag.PSO
        || behaviourFlag == BehaviourFlag.MORPHOGENETIC
        || behaviourFlag == BehaviourFlag.ACO) {
      ArrayList<PVector> updated = new ArrayList<PVector>();
      for (PVector a : attractors) {
        boolean add = true;
        for (Boid b : boids)
          if (abs(b.pos.x - a.x) < 2 && abs(b.pos.y - a.y) < 2) {
            add = false;
            break;
          }
        if (add) updated.add(a.copy());
      }
      attractors.clear();
      attractors.addAll(updated);
      // Reset PSO state when targets change
      if (behaviourFlag == BehaviourFlag.PSO) {
        for (Boid b : boids) b.pbestFitness = Float.MAX_VALUE;
        initGbest();
      }
    } else if (behaviourFlag == BehaviourFlag.CON_STEER
        || behaviourFlag == BehaviourFlag.CUCKER_SMALE
        || behaviourFlag == BehaviourFlag.VICSEK
        || behaviourFlag == BehaviourFlag.SPP) {
      ArrayList<PVector> updated = new ArrayList<PVector>();
      for (PVector g : goalsCS) {
        boolean add = true;
        for (Boid b : boids)
          if (abs(b.pos.x - g.x) < 2 && abs(b.pos.y - g.y) < 2) {
            add = false;
            break;
          }
        if (add) updated.add(g.copy());
      }
      goalsCS.clear();
      goalsCS.addAll(updated);
    }
  }

  void mousePressed(char mode) {
    if (mouseButton == LEFT) {
      if (mode == 'G') {
        addTarget(mouseX, mouseY);
      } else if (mode == 'D') {
        repellents.add(new PVector(mouseX, mouseY));
      } else if (mode == 'R') {
        zones.add(new RepulsionZone(mouseX, mouseY, random(80, 200)));
      } else if (mode == 'B') {
        boids.add(new Boid(mouseX, mouseY));
        targetCount = boids.size();
      }
    } else if (mouseButton == RIGHT) {
      if (mode == 'G') {
        removeLastTarget();
      } else if (mode == 'D') {
        if (!repellents.isEmpty()) repellents.remove(repellents.size() - 1);
      } else if (mode == 'R') {
        if (!zones.isEmpty()) zones.remove(zones.size() - 1);
      } else if (mode == 'B') {
        if (!boids.isEmpty()) boids.remove(boids.size() - 1);
        targetCount = boids.size();
      }
    }
  }

  void update() {
    managePopulation();

    Boid leader = null;
    if (behaviourFlag == BehaviourFlag.COMBINED && !attractors.isEmpty() && !boids.isEmpty()) {
      leader = findLeader();
    }

    // Update global best for PSO (min distance to nearest attractor)
    if (behaviourFlag == BehaviourFlag.PSO && !attractors.isEmpty()) {
      for (Boid b : boids) {
        float minDist = Float.MAX_VALUE;
        for (PVector a : attractors) {
          float d = PVector.dist(b.pos, a);
          if (d < minDist) minDist = d;
        }
        if (minDist < gbestFitness) {
          gbestFitness = minDist;
          gbest = b.pos.copy();
        }
      }
    }

    for (Boid b : boids) {
      // Mouse forces (M mode)
      if (mouseTarget != null) {
        b.linear_attraction(mouseTarget, ATT_MULT);
      }
      if (mouseDanger != null) {
        b.simpleExponential_repulsion(mouseDanger, PERLIMITER, REP_MULT);
      }

      // Universal danger point repulsion (repellents, user-placed D items)
      for (PVector r : repellents)
        b.simpleExponential_repulsion(r, 15, REP_MULT * 2);

      // Universal repulsion zone forces (user-placed R items)
      for (RepulsionZone z : zones)
        b.simpleExponential_repulsion(z.pos, z.radius * 0.3, int(z.radius * 0.1));

      // HUD area repulsion (all modes)
      if (b.pos.x < HUD_W + 80 && b.pos.y < HUD_H + 80) {
        PVector hudPush = new PVector();
        if (b.pos.x < HUD_W) hudPush.x = (HUD_W - b.pos.x) / HUD_W;
        if (b.pos.y < HUD_H) hudPush.y = (HUD_H - b.pos.y) / HUD_H;
        hudPush.mult(3);
        b.acc.add(hudPush);
      }

      if (behaviourFlag == BehaviourFlag.BASIC) {
        b.update(boids, goals, zones);
      } else {
        if (behaviourFlag == BehaviourFlag.ATT_REP) {
          applyAttRep(b);
        } else if (behaviourFlag == BehaviourFlag.CON_STEER) {
          applyConSteer(b);
        } else if (behaviourFlag == BehaviourFlag.COMBINED) {
          applyCombined(b, leader);
        } else if (behaviourFlag == BehaviourFlag.PSO) {
          applyPSO(b);
        } else if (behaviourFlag == BehaviourFlag.CUCKER_SMALE) {
          applyCuckerSmale(b);
        } else if (behaviourFlag == BehaviourFlag.VICSEK) {
          applyVicsek(b);
        } else if (behaviourFlag == BehaviourFlag.MORPHOGENETIC) {
          applyMorphogenetic(b);
        } else if (behaviourFlag == BehaviourFlag.ACO) {
          applyAco(b);
        } else if (behaviourFlag == BehaviourFlag.SPP) {
          applySpp(b);
        }
        b.edges();
      }
    }

    // ACO global pheromone update (once per frame)
    if (behaviourFlag == BehaviourFlag.ACO) {
      evaporatePheromone();
      diffusePheromone();
    }
  }

  void applyAttRep(Boid b) {
    for (PVector a : attractors)
      b.linear_attraction(a, ATT_MULT);

    for (PVector r : repellents)
      b.simpleExponential_repulsion(r, PERLIMITER, REP_MULT);

    for (PVector bp : border_points)
      b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

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

  void applyConSteer(Boid b) {
    for (int idx = 0; idx < RAY_DIRS.size(); ++idx) {
      ArrayList<PVector> intrest_forces = new ArrayList<PVector>();
      ArrayList<PVector> member_atts = new ArrayList<PVector>();
      ArrayList<PVector> member_reps = new ArrayList<PVector>();
      ArrayList<PVector> danger_forces = new ArrayList<PVector>();
      ArrayList<PVector> alignment_forces = new ArrayList<PVector>();
      boolean masked = false;

      for (PVector goal : goalsCS)
        if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(goal, b.pos)) >= SECTOR_COS_SIM)
          intrest_forces.add(b.linear_Attraction_CS(goal, GOAL_LIMIT, GOAL_SIGMA, GOAL_GAMMA));

      for (Boid other : boids) {
        if (other != b) {
          if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos)) >= SECTOR_COS_SIM) {
            float member_dist = PVector.sub(other.pos, b.pos).mult(PIXEL_METRIC_CONV).mag();
            if (member_dist <= MEMBER_TO_CLOSE) masked = true;
            else if (member_dist <= SWARM_DIST) alignment_forces.add(other.vel);
            member_atts.add(b.log_Attraction(other.pos, MEMBER_ATT_LIMIT, MEMBER_ATT_CUT_OFF));
          }
          if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos).rotate(PI)) >= SECTOR_COS_SIM) {
            member_reps.add(b.linear_Repulsion(other.pos, MEMBER_REP_LIMIT, MEMBER_REP_SIGMA, MEMBER_REP_GAMMA, MEMBER_REP_ALPHA));
          }
        }
      }

      for (PVector danger : dangersCS) {
        if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(danger, b.pos).rotate(PI)) >= SECTOR_COS_SIM)
          danger_forces.add(b.limExp_Repulsion(danger, DANGER_LIMIT, DANGER_CUT_OFF, DANGER_SIGMA, DANGER_GAMMA, DANGER_ALPHA));
        else if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(danger, b.pos)) >= SECTOR_COS_SIM) {
          float member_dist = PVector.sub(danger, b.pos).mult(PIXEL_METRIC_CONV).mag();
          if (member_dist <= DANGER_TO_CLOSE) masked = true;
        }
      }

      for (PVector bp : border_points)
        if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(bp, b.pos)) >= SECTOR_COS_SIM) {
          float member_dist = PVector.sub(bp, b.pos).mult(PIXEL_METRIC_CONV).mag();
          if (member_dist <= BORDER_TO_CLOSE) masked = true;
        }

      b.create_context_segment(idx, intrest_forces, danger_forces, member_atts, member_reps, alignment_forces, !masked);
    }

    b.context_steering(RAY_DIRS, SECTOR_COS_SIM);

    b.vel.add(b.acc);
    b.vel.limit(b.maxSpeed);
    b.pos.add(b.vel);
    b.acc.mult(0);
  }

  void applyCombined(Boid b, Boid leader) {
    boolean isLeader = (leader == b);

    for (int idx = 0; idx < RAY_DIRS.size(); ++idx) {
      ArrayList<PVector> intrest_forces = new ArrayList<PVector>();
      ArrayList<PVector> member_atts = new ArrayList<PVector>();
      ArrayList<PVector> member_reps = new ArrayList<PVector>();
      ArrayList<PVector> danger_forces = new ArrayList<PVector>();
      ArrayList<PVector> alignment_forces = new ArrayList<PVector>();
      boolean masked = false;

      if (isLeader) {
        // Leader navigates toward attractors
        for (PVector a : attractors)
          if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(a, b.pos)) >= SECTOR_COS_SIM)
            intrest_forces.add(b.linear_Attraction_CS(a, GOAL_LIMIT, GOAL_SIGMA, GOAL_GAMMA));
      } else if (leader != null) {
        // Followers steer toward the leader
        if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(leader.pos, b.pos)) >= SECTOR_COS_SIM)
          intrest_forces.add(b.strongAttraction(leader.pos, FOLLOW_LIMIT, FOLLOW_SCALE, FOLLOW_DEADZONE));
      }

      // Flocking from other boids (stronger forces for tight cohesion)
      for (Boid other : boids) {
        if (other != b) {
          if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos)) >= SECTOR_COS_SIM) {
            float member_dist = PVector.sub(other.pos, b.pos).mult(PIXEL_METRIC_CONV).mag();
            if (member_dist <= COMBINED_MEMBER_TO_CLOSE) masked = true;
            else if (member_dist <= SWARM_DIST) alignment_forces.add(other.vel);
            member_atts.add(b.strongAttraction(other.pos, COMBINED_ATT_LIMIT, COMBINED_ATT_SCALE, COMBINED_ATT_DEADZONE));
          }
          if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos).rotate(PI)) >= SECTOR_COS_SIM) {
            member_reps.add(b.strongRepulsion(other.pos, COMBINED_REP_LIMIT, COMBINED_REP_SCALE));
          }
        }
      }

      // Danger from RepulsionZones
      for (RepulsionZone z : zones) {
        PVector toZone = PVector.sub(z.pos, b.pos);
        float dist = toZone.mag();
        float zoneLimit = z.radius + ZONE_DANGER_LIMIT;

        if (dist < zoneLimit) {
          boolean forward = cosine_sim(RAY_DIRS.get(idx), toZone) >= SECTOR_COS_SIM;
          boolean behind = cosine_sim(RAY_DIRS.get(idx), toZone.rotate(PI)) >= SECTOR_COS_SIM;

          if (forward) {
            // Zone is ahead – compute repulsion
            float strength;
            if (dist < z.radius) {
              // Inside zone: strong push out
              strength = (z.radius - dist) * 0.5;
            } else {
              // Outside zone: gentle avoidance
              strength = (zoneLimit - dist) / zoneLimit * 2;
            }
            PVector repForce = PVector.sub(b.pos, z.pos);
            repForce.normalize();
            repForce.mult(strength);
            danger_forces.add(repForce);

            if (dist < z.radius + ZONE_TO_CLOSE) masked = true;
          } else if (behind && dist < z.radius) {
            // Inside zone even behind – still mask
            masked = true;
          }
        }
      }

      // Border danger
      for (PVector bp : border_points)
        if (cosine_sim(RAY_DIRS.get(idx), PVector.sub(bp, b.pos)) >= SECTOR_COS_SIM) {
          float member_dist = PVector.sub(bp, b.pos).mult(PIXEL_METRIC_CONV).mag();
          if (member_dist <= BORDER_TO_CLOSE) masked = true;
        }

      b.create_context_segment(idx, intrest_forces, danger_forces, member_atts, member_reps, alignment_forces, !masked);
    }

    b.context_steering(RAY_DIRS, SECTOR_COS_SIM);

    b.vel.add(b.acc);
    b.vel.limit(b.maxSpeed);
    b.pos.add(b.vel);
    b.acc.mult(0);
  }

  void applyPSO(Boid b) {
    // Update personal best (min distance to nearest attractor)
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
    PVector social = PVector.sub(gbest, b.pos).mult(PSO_SOCIAL * r2);
    PVector inertia = b.vel.copy().mult(PSO_INERTIA);

    PVector psoForce = PVector.add(inertia, cognitive);
    psoForce.add(social);

    // Attraction to attractors
    for (PVector a : attractors)
      b.linear_attraction(a, ATT_MULT);

    // Repulsion from repellents
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

  void applyCuckerSmale(Boid b) {
    PVector alignment = new PVector();
    float totalWeight = 0;

    for (Boid other : boids) {
      if (other != b) {
        float dist = PVector.dist(b.pos, other.pos);
        float weight = CUCKER_H / (CUCKER_H + dist * dist);
        PVector diff = PVector.sub(other.vel, b.vel);
        alignment.add(PVector.mult(diff, weight));
        totalWeight += weight;
      }
    }

    if (totalWeight > 0) {
      alignment.div(totalWeight);
    }

    // Attraction toward goalsCS
    for (PVector g : goalsCS)
      b.linear_attraction(g, int(ATT_MULT * CUCKER_ATTR_SCALE));

    // Danger avoidance
    for (PVector d : dangersCS)
      b.simpleExponential_repulsion(d, PERLIMITER, REP_MULT);

    // Border repulsion
    for (PVector bp : border_points)
      b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

    // Inter-boid repulsion (prevent collapse)
    for (Boid other : boids) {
      if (other != b) {
        b.comfy_attraction(other.pos, COMFY_DIST * 1.5, DRONE_ATT_MULT);
        b.complexExponential_repulsion(other.pos, DRONE_PERLIMITER, DRONE_ATT_MULT, DRONE_REP_MULT * 2);
      }
    }

    b.acc.add(alignment);
    b.vel.add(b.acc);
    b.vel.limit(b.maxSpeed);
    b.pos.add(b.vel);
    b.acc.mult(0);
  }

  void applyVicsek(Boid b) {
    PVector avgDir = new PVector();
    int count = 0;

    for (Boid other : boids) {
      if (other != b && PVector.dist(b.pos, other.pos) < VICSEK_RADIUS) {
        avgDir.add(other.vel.copy().normalize());
        count++;
      }
    }

    if (count > 0) {
      avgDir.div(count);
    } else {
      avgDir = b.vel.copy().normalize();
    }

    // Add noise
    float noiseAngle = random(-PI * VICSEK_NOISE, PI * VICSEK_NOISE);
    avgDir.rotate(noiseAngle);
    avgDir.mult(VICSEK_SPEED);

    // Treat Vicsek alignment as a steering force toward desired velocity
    PVector vicsekForce = PVector.sub(avgDir, b.vel);
    vicsekForce.limit(b.maxForce);
    b.acc.add(vicsekForce);

    // Attraction toward goalsCS
    for (PVector g : goalsCS)
      b.linear_attraction(g, int(ATT_MULT * CUCKER_ATTR_SCALE));

    // Danger avoidance
    for (PVector d : dangersCS)
      b.simpleExponential_repulsion(d, PERLIMITER, REP_MULT);

    // Border repulsion
    for (PVector bp : border_points)
      b.simpleExponential_repulsion(bp, BORDER_PERLIMITER, REP_MULT);

    // Inter-boid repulsion with equilibrium distance
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

  void applyMorphogenetic(Boid b) {
    // GRN-inspired morphogen dynamics
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
    // Production toward target + decay
    b.morphogen += MORPH_PRODUCTION * (b.morphogenTarget - b.morphogen);
    b.morphogen *= MORPH_DECAY;
    b.morphogen = constrain(b.morphogen, 0, 2);

    // Goal attraction scaled by morphogen
    for (PVector a : attractors) {
      if (b.morphogen > 0.5) {
        b.linear_attraction(a, int(ATT_MULT * b.morphogen));
      }
    }

    // Repulsion from repellents (stronger when morphogen is low = exploratory)
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

  void applyAco(Boid b) {
    // Sense pheromone at current position
    int px = int(b.pos.x / PHEROMONE_CELL);
    int py = int(b.pos.y / PHEROMONE_CELL);
    px = constrain(px, 0, PHEROMONE_COLS - 1);
    py = constrain(py, 0, PHEROMONE_ROWS - 1);

    // Deposit pheromone (more near attractors)
    float deposit = PHEROMONE_DEPOSIT;
    for (PVector a : attractors) {
      float d = PVector.dist(b.pos, a);
      if (d < 200) deposit *= (200 - d) / 200 * 3;
    }
    pheromone[px][py] = min(pheromone[px][py] + deposit, 100);

    // Read pheromone gradient for steering
    PVector gradient = new PVector();
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        int nx = constrain(px + dx, 0, PHEROMONE_COLS - 1);
        int ny = constrain(py + dy, 0, PHEROMONE_ROWS - 1);
        float diff = pheromone[nx][ny] - pheromone[px][py];
        if (diff > 0)
          gradient.add(new PVector(dx, dy).mult(diff));
      }
    }
    if (gradient.mag() > 0) {
      gradient.setMag(PHEROMONE_INFLUENCE);
      b.acc.add(gradient);
    }

    // Goal attraction to attractors
    for (PVector a : attractors)
      b.linear_attraction(a, ATT_MULT);

    // Repulsion from repellents
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

    b.vel.add(b.acc);
    b.vel.limit(b.maxSpeed);
    b.pos.add(b.vel);
    b.acc.mult(0);
  }

  void applySpp(Boid b) {
    // Social positioning: similar weight → attract, different → repel
    for (Boid other : boids) {
      if (other != b) {
        float diff = abs(b.socialWeight - other.socialWeight);
        PVector dir = PVector.sub(other.pos, b.pos);
        float dist = dir.mag();
        if (dist < 1) continue;
        float strength;
        if (diff < SPP_WEIGHT_THRESHOLD) {
          // Similar: social attraction (stronger when threshold is closer)
          strength = SPP_ATTRACT_SCALE * (SPP_WEIGHT_THRESHOLD - diff) / SPP_WEIGHT_THRESHOLD * (300 / max(dist, 10));
        } else {
          // Different: social repulsion
          strength = -SPP_REPEL_SCALE * (diff - SPP_WEIGHT_THRESHOLD) / (1 - SPP_WEIGHT_THRESHOLD) * (300 / max(dist, 10));
        }
        strength = constrain(strength, -SPP_MAX_FORCE, SPP_MAX_FORCE);
        dir.normalize();
        dir.mult(strength);
        b.acc.add(dir);
      }
    }

    // Attraction toward goalsCS
    for (PVector g : goalsCS)
      b.linear_attraction(g, int(ATT_MULT * CUCKER_ATTR_SCALE));

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

  void display() {
    // Draw ACO pheromone overlay
    if (behaviourFlag == BehaviourFlag.ACO) {
      noStroke();
      for (int x = 0; x < PHEROMONE_COLS; x++) {
        for (int y = 0; y < PHEROMONE_ROWS; y++) {
          if (pheromone[x][y] > 0.5) {
            float alpha = constrain(pheromone[x][y] / 20, 0, 60);
            fill(180, 140, 40, alpha);
            rect(x * PHEROMONE_CELL, y * PHEROMONE_CELL, PHEROMONE_CELL, PHEROMONE_CELL);
          }
        }
      }
      fill(255, 255, 255, 255);
    }

    // Draw mode-specific targets (green)
    if (behaviourFlag == BehaviourFlag.ATT_REP || behaviourFlag == BehaviourFlag.PSO
        || behaviourFlag == BehaviourFlag.MORPHOGENETIC || behaviourFlag == BehaviourFlag.ACO
        || behaviourFlag == BehaviourFlag.COMBINED) {
      stroke(0, 255, 0);
      strokeWeight(8);
      for (PVector a : attractors) point(a.x, a.y);
    } else if (behaviourFlag == BehaviourFlag.CON_STEER
        || behaviourFlag == BehaviourFlag.CUCKER_SMALE
        || behaviourFlag == BehaviourFlag.VICSEK
        || behaviourFlag == BehaviourFlag.SPP) {
      stroke(0, 255, 0);
      strokeWeight(8);
      for (PVector g : goalsCS) point(g.x, g.y);
    } else if (behaviourFlag == BehaviourFlag.BASIC) {
      for (Goal g : goals) g.display();
    }

    // Draw auto-spawned dangersCS (mode-specific)
    if (behaviourFlag == BehaviourFlag.CON_STEER
        || behaviourFlag == BehaviourFlag.CUCKER_SMALE
        || behaviourFlag == BehaviourFlag.VICSEK
        || behaviourFlag == BehaviourFlag.SPP) {
      stroke(255, 0, 0);
      strokeWeight(8);
      for (PVector d : dangersCS) point(d.x, d.y);
    }

    // Draw universal user-placed danger points (D items)
    stroke(255, 0, 0);
    strokeWeight(6);
    for (PVector r : repellents) point(r.x, r.y);

    // Draw universal user-placed repulsion zones (R items)
    for (RepulsionZone z : zones) z.display();

    // Draw boids
    for (Boid b : boids) b.display();

    // Draw mouse target/danger (M mode)
    if (mouseTarget != null) {
      stroke(0, 255, 200);
      strokeWeight(12);
      point(mouseTarget.x, mouseTarget.y);
    }
    if (mouseDanger != null) {
      stroke(255, 80, 0);
      strokeWeight(12);
      point(mouseDanger.x, mouseDanger.y);
    }
  }

  void managePopulation() {
    while (boids.size() < targetCount) {
      if (behaviourFlag == BehaviourFlag.CON_STEER || behaviourFlag == BehaviourFlag.COMBINED) {
        Boid nb = new Boid(random(width), random(height), 8);
        boids.add(nb);
      } else {
        boids.add(new Boid(random(width), random(height)));
      }
    }
    while (boids.size() > targetCount) {
      boids.remove(0);
    }
  }

  void addTarget(float x, float y) {
    switch (behaviourFlag) {
      case BASIC:
        goals.add(new Goal(x, y, random(30, 80)));
        break;
      case ATT_REP:
      case PSO:
      case MORPHOGENETIC:
      case ACO:
      case COMBINED:
        attractors.add(new PVector(x, y));
        break;
      case CON_STEER:
      case CUCKER_SMALE:
      case VICSEK:
      case SPP:
        goalsCS.add(new PVector(x, y));
        break;
    }
  }

  void addDanger(float x, float y) {
    switch (behaviourFlag) {
      case BASIC:
        zones.add(new RepulsionZone(x, y, random(80, 200)));
        break;
      case ATT_REP:
      case PSO:
      case MORPHOGENETIC:
      case ACO:
      case COMBINED:
        repellents.add(new PVector(x, y));
        break;
      case CON_STEER:
      case CUCKER_SMALE:
      case VICSEK:
      case SPP:
        dangersCS.add(new PVector(x, y));
        break;
    }
  }

  void removeLastTarget() {
    switch (behaviourFlag) {
      case BASIC:
        if (!goals.isEmpty()) goals.remove(goals.size() - 1);
        break;
      case ATT_REP:
      case PSO:
      case MORPHOGENETIC:
      case ACO:
      case COMBINED:
        if (!attractors.isEmpty()) attractors.remove(attractors.size() - 1);
        break;
      case CON_STEER:
      case CUCKER_SMALE:
      case VICSEK:
      case SPP:
        if (!goalsCS.isEmpty()) goalsCS.remove(goalsCS.size() - 1);
        break;
    }
  }

  void removeLastDanger() {
    switch (behaviourFlag) {
      case BASIC:
        if (!zones.isEmpty()) zones.remove(zones.size() - 1);
        break;
      case ATT_REP:
      case PSO:
      case MORPHOGENETIC:
      case ACO:
      case COMBINED:
        if (!repellents.isEmpty()) repellents.remove(repellents.size() - 1);
        break;
      case CON_STEER:
      case CUCKER_SMALE:
      case VICSEK:
      case SPP:
        if (!dangersCS.isEmpty()) dangersCS.remove(dangersCS.size() - 1);
        break;
    }
  }

  void addRandomBoids(int n) {
    for (int i = 0; i < n; i++)
      boids.add(new Boid(random(width), random(height)));
    targetCount = boids.size();
  }

  void removeRandomBoids(int n) {
    for (int i = 0; i < n && !boids.isEmpty(); i++)
      boids.remove(floor(random(boids.size())));
    targetCount = boids.size();
  }

  void clearAll() {
    goals.clear();
    zones.clear();
    attractors.clear();
    repellents.clear();
    goalsCS.clear();
    dangersCS.clear();
  }
}
