import java.util.ArrayList;

// ============================================================
// Behaviour Flag Enum
// Maps number keys 1-0 to each swarm intelligence algorithm.
// ============================================================
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

// ============================================================
// SwarmManager
//
// Core class that manages the boid population, universal forces
// (mouse, danger points, repulsion zones, HUD push, centre
// attraction for carrying boids), target pickup/release, the
// main update loop, and display.
//
// Each behaviour's apply* method lives in its own module file
// (e.g. AttRepBehaviour.pde, AcoBehaviour.pde, etc.) but all
// are member functions of this class — Processing concatenates
// all .pde files in the sketch directory into one compilation
// unit, so methods can be defined across files.
// ============================================================
class SwarmManager {
  // ----- Population -----
  ArrayList<Boid> boids = new ArrayList<Boid>();
  ArrayList<RepulsionZone> zones = new ArrayList<RepulsionZone>();
  int targetCount = 10;
  BehaviourFlag behaviourFlag = BehaviourFlag.ATT_REP;

  // ----- Target / danger / waypoint lists -----
  // attractors/repellents: used by BASIC, ATT_REP, PSO, MORPH, ACO, COMBINED
  ArrayList<PVector> attractors  = new ArrayList<PVector>();
  ArrayList<PVector> repellents  = new ArrayList<PVector>();
  ArrayList<PVector> border_points = new ArrayList<PVector>();

  // goalsCS/dangersCS: used by CON_STEER, CUCKER_SMALE, VICSEK, SPP
  ArrayList<PVector> goalsCS     = new ArrayList<PVector>();
  ArrayList<PVector> dangersCS   = new ArrayList<PVector>();

  // ----- ConSteer / Combined shared structures -----
  ArrayList<PVector> RAY_DIRS = new ArrayList<PVector>();
  float SECTOR_COS_SIM;

  // ----- HUD safe zone (800x640 top-left) -----
  // Boids and points never spawn inside this region.
  final float HUD_W = 800;
  final float HUD_H = 640;

  // ----- Auto-respawn timer -----
  // Periodically rotates target/danger positions so the environment stays dynamic.
  int ticks = 0;
  final int MAX_TICKS = 450;
  final int UPDATE_PORTION = 2;
  final float PIXEL_METRIC_CONV = 0.06;

  // ----- ConSteer constants -----
  final float BORDER_TO_CLOSE     = 2;
  final float DANGER_TO_CLOSE     = 8;
  final float MEMBER_TO_CLOSE     = 4;
  final float SWARM_DIST          = 12;
  final float GOAL_LIMIT          = 30;
  final float GOAL_SIGMA          = -3;
  final float GOAL_GAMMA          = 25;
  final float DANGER_LIMIT        = 40;
  final float DANGER_CUT_OFF      = 5;
  final float DANGER_SIGMA        = 3.85;
  final float DANGER_GAMMA        = 18.5;
  final float DANGER_ALPHA        = 20;
  final float MEMBER_REP_LIMIT    = 30;
  final float MEMBER_REP_SIGMA    = 18;
  final float MEMBER_REP_GAMMA    = 1.5;
  final float MEMBER_REP_ALPHA    = 0.1;
  final float MEMBER_ATT_LIMIT    = 30;
  final float MEMBER_ATT_CUT_OFF  = 22;

  // ----- Combined mode constants -----
  final float ZONE_DANGER_LIMIT        = 100;
  final float ZONE_TO_CLOSE            = 10;
  final float COMBINED_REP_SCALE       = 200;
  final float COMBINED_REP_LIMIT       = 30;
  final float COMBINED_ATT_SCALE       = 3.0;
  final float COMBINED_ATT_DEADZONE    = 3.0;
  final float COMBINED_ATT_LIMIT       = 50;
  final float COMBINED_MEMBER_TO_CLOSE = 3.0;
  final float FOLLOW_SCALE             = 5.0;
  final float FOLLOW_DEADZONE          = 4.0;
  final float FOLLOW_LIMIT             = 60;

  // ----- PSO state -----
  PVector gbest = new PVector();
  float gbestFitness = Float.MAX_VALUE;
  final float PSO_INERTIA   = 0.7;
  final float PSO_COGNITIVE = 1.5;
  final float PSO_SOCIAL    = 1.5;

  // ----- Cucker-Smale constants -----
  final float CUCKER_H          = 80.0;
  final float CUCKER_ATTR_SCALE = 0.5;
  final float CUCKER_ATTR_LIMIT = 40;

  // ----- Vicsek constants -----
  final float VICSEK_RADIUS = 120;
  final float VICSEK_NOISE  = 0.15;
  final float VICSEK_SPEED  = 3.0;

  // ----- Morphogenetic constants -----
  final float MORPH_COMM_RADIUS   = 150;
  final float MORPH_DECAY         = 0.98;
  final float MORPH_DIFFUSION     = 0.05;
  final float MORPH_PRODUCTION    = 0.01;
  final float MORPH_ATTRACT_SCALE = 80;

  // ----- ACO constants -----
  final int PHEROMONE_CELL = 50;
  int PHEROMONE_COLS;
  int PHEROMONE_ROWS;
  float[][] pheromone;
  final float PHEROMONE_EVAPORATION = 0.006;
  final float PHEROMONE_DIFFUSION   = 0.01;
  final float PHEROMONE_DEPOSIT     = 6;
  final float PHEROMONE_INFLUENCE   = 3;
  final float PHEROMONE_SENSING_RADIUS = 50;

  // ----- SPP constants -----
  final float SPP_WEIGHT_THRESHOLD = 0.15;
  final float SPP_ATTRACT_SCALE    = 0.05;
  final float SPP_REPEL_SCALE      = 0.08;
  final float SPP_MAX_FORCE        = 0.5;

  // ----- Centre point for item delivery -----
  final int CENTER_X = 2400;
  final int CENTER_Y = 1600;

  // ----- AttRep shared constants (also used by several other modes) -----
  final int ATT_MULT    = 128;
  final int REP_MULT    = 44;
  final int DRONE_ATT_MULT = 2;
  final int DRONE_REP_MULT = 20;
  final float COMFY_DIST  = 30;
  PVector mouseTarget = null;
  PVector mouseDanger = null;

  final float PERLIMITER         = 25;
  final float BORDER_PERLIMITER  = 5;
  final float DRONE_PERLIMITER   = 10;

  // ---------------------------------------------------------------
  // Utility: random position outside the HUD safe zone
  // ---------------------------------------------------------------
  PVector randomOutsideHud() {
    float x, y;
    do {
      x = random(4, width - 4);
      y = random(4, height - 4);
    } while (x < HUD_W && y < HUD_H);
    return new PVector(x, y);
  }

  // ---------------------------------------------------------------
  // findLeader: drone closest to the nearest attractor
  // (used by COMBINED mode — the leader navigates, others follow)
  // ---------------------------------------------------------------
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

  // ---------------------------------------------------------------
  // PSO helper: reset global best fitness
  // ---------------------------------------------------------------
  void initGbest() {
    gbestFitness = Float.MAX_VALUE;
    gbest = new PVector();
  }

  // ---------------------------------------------------------------
  // Constructor: init PSO, pheromone grid, border walls, auto points
  // ---------------------------------------------------------------
  SwarmManager() {
    initGbest();
    initPheromone(this);


    // Border walls: dense point grid around the canvas perimeter
    for (int x = 0; x < width; ++x) {
      border_points.add(new PVector(x, 0));
      border_points.add(new PVector(x, height));
    }
    for (int y = 0; y < height; ++y) {
      border_points.add(new PVector(0, y));
      border_points.add(new PVector(width, y));
    }
    setupAutoPoints();
  }

  // ---------------------------------------------------------------
  // initBehaviour: mode switch — init context steering dirs, reset
  // PSO pbests, re-init ACO pheromone grid. Does NOT clear existing
  // points or reset the auto-respawn timer.
  // ---------------------------------------------------------------
  void initBehaviour(BehaviourFlag flag) {
    this.behaviourFlag = flag;

    // Reset PSO personal bests for all boids
    for (Boid b : boids) {
      b.pbest = b.pos.copy();
      b.pbestFitness = Float.MAX_VALUE;
    }

    if (flag == BehaviourFlag.CON_STEER || flag == BehaviourFlag.COMBINED) {
      // Build direction vectors for context steering
      int directions = 8;
      RAY_DIRS.clear();
      for (int it = 0; it < directions; ++it) {
        float angle = it * TWO_PI / directions;
        RAY_DIRS.add(PVector.fromAngle(angle));
      }
      SECTOR_COS_SIM = cos(PI / directions);

      // Initialise context steering on boids that don't have it yet
      for (Boid b : boids) {
        if (b.DIRECTIONS == 0) b.initContextSteering(directions);
      }
    }

    // Reset ACO pheromone grid
    if (flag == BehaviourFlag.ACO) initPheromone(this);
  }

  // ---------------------------------------------------------------
  // setupAutoPoints: first-run spawn of attractors/repellents /
  // goals/dangers based on current behaviour flag.
  // ---------------------------------------------------------------
  void setupAutoPoints() {
    if (behaviourFlag == BehaviourFlag.BASIC
        || behaviourFlag == BehaviourFlag.ATT_REP || behaviourFlag == BehaviourFlag.PSO
        || behaviourFlag == BehaviourFlag.MORPHOGENETIC || behaviourFlag == BehaviourFlag.ACO
        || behaviourFlag == BehaviourFlag.COMBINED) {
      for (int it = 0; it < 8; ++it) attractors.add(randomOutsideHud());
      for (int it = 0; it < 4; ++it) repellents.add(randomOutsideHud());
    } else if (behaviourFlag == BehaviourFlag.CON_STEER
        || behaviourFlag == BehaviourFlag.CUCKER_SMALE
        || behaviourFlag == BehaviourFlag.VICSEK
        || behaviourFlag == BehaviourFlag.SPP) {
      for (int it = 0; it < 8; ++it) goalsCS.add(randomOutsideHud());
      for (int it = 0; it < 4; ++it) dangersCS.add(randomOutsideHud());
    }
  }

  // ---------------------------------------------------------------
  // autoUpdate: periodically rotate target/danger positions so the
  // environment stays dynamic and boids must keep exploring.
  // ---------------------------------------------------------------
  void autoUpdate(int portion) {
    if (ticks >= MAX_TICKS) {
      ticks = 0;
      if (behaviourFlag == BehaviourFlag.BASIC
          || behaviourFlag == BehaviourFlag.ATT_REP || behaviourFlag == BehaviourFlag.PSO
          || behaviourFlag == BehaviourFlag.MORPHOGENETIC || behaviourFlag == BehaviourFlag.ACO
          || behaviourFlag == BehaviourFlag.COMBINED) {
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
      }
    }
    ++ticks;
  }

  // ---------------------------------------------------------------
  // checkPickup: any boid touching a target (distance < 2 px) picks
  // it up (carrying = true). Runs before force application so the
  // current frame's movement handles the carrying state.
  // ---------------------------------------------------------------
  void checkPickup() {
    if (behaviourFlag == BehaviourFlag.BASIC
        || behaviourFlag == BehaviourFlag.ATT_REP
        || behaviourFlag == BehaviourFlag.COMBINED
        || behaviourFlag == BehaviourFlag.PSO
        || behaviourFlag == BehaviourFlag.MORPHOGENETIC
        || behaviourFlag == BehaviourFlag.ACO) {
      for (int i = attractors.size() - 1; i >= 0; i--) {
        PVector a = attractors.get(i);
        for (Boid b : boids) {
          if (!b.carrying && PVector.dist(b.pos, a) < 2) {
            b.carrying = true;
            attractors.remove(i);
            if (behaviourFlag == BehaviourFlag.PSO) {
              for (Boid pb : boids) pb.pbestFitness = Float.MAX_VALUE;
              initGbest();
            }
            break;
          }
        }
      }
    } else if (behaviourFlag == BehaviourFlag.CON_STEER
        || behaviourFlag == BehaviourFlag.CUCKER_SMALE
        || behaviourFlag == BehaviourFlag.VICSEK
        || behaviourFlag == BehaviourFlag.SPP) {
      for (int i = goalsCS.size() - 1; i >= 0; i--) {
        PVector g = goalsCS.get(i);
        for (Boid b : boids) {
          if (!b.carrying && PVector.dist(b.pos, g) < 2) {
            b.carrying = true;
            goalsCS.remove(i);
            break;
          }
        }
      }
    }
  }

  // ---------------------------------------------------------------
  // checkRelease: carrying boid at centre (distance < 10 px) drops
  // the item. Runs after position update so delivery is correct.
  // ---------------------------------------------------------------
  void checkRelease() {
    for (Boid b : boids) {
      if (b.carrying && dist(b.pos.x, b.pos.y, CENTER_X, CENTER_Y) < 10) {
        b.carrying = false;
      }
    }
  }

  // ---------------------------------------------------------------
  // mousePressed: LEFT-click spawns items based on current mode
  // (G=green target, D=red danger dot, R=red repulsion zone, B=boid,
  // M handled in draw()). RIGHT-click removes the last item of that
  // type. All spawns reject positions inside the HUD zone.
  // ---------------------------------------------------------------
  void mousePressed(char mode) {
    if (mouseButton == LEFT) {
      if (mouseX < HUD_W && mouseY < HUD_H) return;
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

  // ---------------------------------------------------------------
  // update: main per-frame logic
  //
  // Order of operations:
  //   1. Manage population (replenish/trim to targetCount)
  //   2. Pickup (boids grab targets they touch)
  //   3. Check anyCarrying flag, find COMBINED leader
  //   4. For each boid: universal forces (mouse, dangers, zones,
  //      HUD push, centre attraction) then mode-specific apply*
  //   5. Release (carrying boids at centre drop items)
  //   6. Remove dead boids
  //   7. ACO: evaporate, diffuse, centre emission
  // ---------------------------------------------------------------
  void update() {
    managePopulation();

    checkPickup();

    boolean anyCarrying = false;
    for (Boid b : boids) if (b.carrying) { anyCarrying = true; break; }

    Boid leader = null;
    if (behaviourFlag == BehaviourFlag.COMBINED && !attractors.isEmpty() && !boids.isEmpty()) {
      leader = findLeader();
    }

    // PSO global best: min distance to nearest attractor
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

    PVector center = new PVector(CENTER_X, CENTER_Y);

    for (Boid b : boids) {
      // Mouse target (M mode, LEFT-click hold)
      if (mouseTarget != null && !b.carrying) {
        b.linear_attraction(mouseTarget, ATT_MULT);
      }
      // Mouse danger (M mode, RIGHT-click hold)
      if (mouseDanger != null) {
        b.simpleExponential_repulsion(mouseDanger, PERLIMITER, REP_MULT);
      }

      // Universal danger point repulsion (repellents — user-placed D items)
      for (PVector r : repellents)
        b.simpleExponential_repulsion(r, 15, REP_MULT * 4);

      // Universal repulsion zone forces (user-placed R items)
      for (RepulsionZone z : zones) {
        float d = PVector.dist(b.pos, z.pos);
        b.simpleExponential_repulsion(z.pos, z.radius * 0.3, int(z.radius * 0.2));
        if (d < z.radius) {
          b.health -= (z.radius - d) / z.radius * 0.15;
          if (b.health <= 0) b.dead = true;
        }
      }

      // HUD area repulsion — boids are pushed away from the UI zone
      if (b.pos.x < HUD_W + 80 && b.pos.y < HUD_H + 80) {
        PVector hudPush = new PVector();
        if (b.pos.x < HUD_W) hudPush.x = (HUD_W - b.pos.x) / HUD_W;
        if (b.pos.y < HUD_H) hudPush.y = (HUD_H - b.pos.y) / HUD_H;
        hudPush.mult(3);
        b.acc.add(hudPush);
      }

      // Centre attraction for carrying boids — 10× normal target strength
      if (b.carrying && anyCarrying) {
        b.linear_attraction(center, ATT_MULT * 10);
      }

      // Track movement activity for ACO pheromone scaling
      b.updateActivity();

      // BASIC mode uses Reynolds flocking inside Boid.update()
      if (behaviourFlag == BehaviourFlag.BASIC && !b.carrying) {
        for (PVector a : attractors) b.linear_attraction(a, ATT_MULT);
      }

      // Dispatch to behaviour-specific apply* method
      if (behaviourFlag == BehaviourFlag.BASIC) {
        b.update(boids, zones);
      } else {
        if (behaviourFlag == BehaviourFlag.ATT_REP) {
          applyAttRep(this, b);
        } else if (behaviourFlag == BehaviourFlag.CON_STEER) {
          applyConSteer(this, b);
        } else if (behaviourFlag == BehaviourFlag.COMBINED) {
          applyCombined(this, b, leader);
        } else if (behaviourFlag == BehaviourFlag.PSO) {
          applyPSO(this, b);
        } else if (behaviourFlag == BehaviourFlag.CUCKER_SMALE) {
          applyCuckerSmale(this, b);
        } else if (behaviourFlag == BehaviourFlag.VICSEK) {
          applyVicsek(this, b);
        } else if (behaviourFlag == BehaviourFlag.MORPHOGENETIC) {
          applyMorphogenetic(this, b);
        } else if (behaviourFlag == BehaviourFlag.ACO) {
          applyAco(this, b);
        } else if (behaviourFlag == BehaviourFlag.SPP) {
          applySpp(this, b);
        }
        b.edges();
      }

      b.recordTrail();
    }

    checkRelease();

    // Remove dead boids (reverse order for safe removal)
    for (int i = boids.size() - 1; i >= 0; i--)
      if (boids.get(i).dead) boids.remove(i);
    targetCount = boids.size();

    // ACO global pheromone update
    if (behaviourFlag == BehaviourFlag.ACO) {
      evaporatePheromone(this);
      diffusePheromone(this);
      // Centre emits a constant semi-local pheromone beacon (home)
      int cx = int(CENTER_X / PHEROMONE_CELL);
      int cy = int(CENTER_Y / PHEROMONE_CELL);
      for (int dx = -4; dx <= 4; dx++) {
        for (int dy = -4; dy <= 4; dy++) {
          float dist = sqrt(dx * dx + dy * dy);
          if (dist > 4) continue;
          int nx = constrain(cx + dx, 0, PHEROMONE_COLS - 1);
          int ny = constrain(cy + dy, 0, PHEROMONE_ROWS - 1);
          float strength = (4 - dist) / 4 * 10;
          pheromone[nx][ny] = min(pheromone[nx][ny] + strength, 100);
        }
      }
    }
  }

  // ---------------------------------------------------------------
  // display: render all visual layers
  //
  // Order: ACO pheromone overlay → mode-specific targets (green) →
  // auto-spawned dangers (red) → universal user-placed D items →
  // repulsion zones (red circles) → boids → mouse target/danger →
  // cornflowerblue centre circle when any boid carries.
  // ---------------------------------------------------------------
  void display() {
    // ACO pheromone overlay (yellow ellipses, size/alpha mapped to intensity)
    if (behaviourFlag == BehaviourFlag.ACO) {
      noStroke();
      for (int x = 0; x < PHEROMONE_COLS; x++) {
        for (int y = 0; y < PHEROMONE_ROWS; y++) {
          float p = pheromone[x][y];
          if (p > 0.5) {
            float alpha = constrain(map(p, 0, 20, 30, 220), 30, 220);
            float sz = constrain(map(p, 0, 20, 2, 8), 2, 8);
            fill(255, 220, 60, alpha);
            ellipse(x * PHEROMONE_CELL + PHEROMONE_CELL / 2,
                    y * PHEROMONE_CELL + PHEROMONE_CELL / 2, sz, sz);
          }
        }
      }
    }

    // Green target dots (mode-specific)
    if (behaviourFlag == BehaviourFlag.BASIC
        || behaviourFlag == BehaviourFlag.ATT_REP || behaviourFlag == BehaviourFlag.PSO
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
    }

    // Auto-spawned danger dots (mode-specific, red)
    if (behaviourFlag == BehaviourFlag.CON_STEER
        || behaviourFlag == BehaviourFlag.CUCKER_SMALE
        || behaviourFlag == BehaviourFlag.VICSEK
        || behaviourFlag == BehaviourFlag.SPP) {
      stroke(255, 0, 0);
      strokeWeight(8);
      for (PVector d : dangersCS) point(d.x, d.y);
    }

    // Universal user-placed danger dots (D items)
    stroke(255, 0, 0);
    strokeWeight(6);
    for (PVector r : repellents) point(r.x, r.y);

    // Universal user-placed repulsion zones (R items)
    for (RepulsionZone z : zones) z.display();

    // Boids (each draws its own trail + body)
    for (Boid b : boids) b.display();

    // Mouse target/danger (M mode)
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

    // Cornflowerblue centre circle when any boid carries an item
    boolean anyCarrying = false;
    for (Boid b : boids) if (b.carrying) { anyCarrying = true; break; }
    if (anyCarrying) {
      noFill();
      stroke(100, 149, 237);
      strokeWeight(4);
      ellipse(CENTER_X, CENTER_Y, 48, 48);
      strokeWeight(1);
    }
  }

  // ---------------------------------------------------------------
  // managePopulation: add/remove boids so boids.size() == targetCount
  // New boids are spawned outside the HUD zone.
  // ---------------------------------------------------------------
  void managePopulation() {
    while (boids.size() < targetCount) {
      PVector p = randomOutsideHud();
      if (behaviourFlag == BehaviourFlag.CON_STEER || behaviourFlag == BehaviourFlag.COMBINED) {
        Boid nb = new Boid(p.x, p.y, 8);
        boids.add(nb);
      } else {
        boids.add(new Boid(p.x, p.y));
      }
    }
    while (boids.size() > targetCount) {
      boids.remove(0);
    }
  }

  // ---------------------------------------------------------------
  // addTarget / addDanger / removeLastTarget / removeLastDanger
  // Mode-aware helpers for point management.
  // ---------------------------------------------------------------
  void addTarget(float x, float y) {
    switch (behaviourFlag) {
      case BASIC:
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

  // ---------------------------------------------------------------
  // addRandomBoids / removeRandomBoids
  // Spawn/delete n boids and adjust targetCount.
  // ---------------------------------------------------------------
  void addRandomBoids(int n) {
    for (int i = 0; i < n; i++) {
      PVector p = randomOutsideHud();
      boids.add(new Boid(p.x, p.y));
    }
    targetCount = boids.size();
  }

  void removeRandomBoids(int n) {
    for (int i = 0; i < n && !boids.isEmpty(); i++)
      boids.remove(floor(random(boids.size())));
    targetCount = boids.size();
  }

  // ---------------------------------------------------------------
  // clearAll: remove all user-placed points and zones
  // ---------------------------------------------------------------
  void clearAll() {
    zones.clear();
    attractors.clear();
    repellents.clear();
    goalsCS.clear();
    dangersCS.clear();
  }
}
