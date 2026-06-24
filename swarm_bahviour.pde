import java.util.ArrayList;

// ============================================================
// Swarm Behaviour Simulation — Main Entry Point
//
// A large-canvas (4800×3200) drone swarm simulation featuring
// 10 swarm intelligence algorithms with a target-carrying
// mechanic where boids pick up green targets and deliver them
// to a cornflowerblue drop zone at centre (2400, 1600).
//
// Controls:
//   Number keys 1-0   Switch behaviour mode
//   G / D / R / B / M Set spawn mode (Goal, Danger, Repulsion zone, Boid, Mouse)
//   LEFT-click         Spawn item based on current mode
//   RIGHT-click        Delete last item of current mode
//   UP / DOWN          Add / remove 10 boids
//   C                  Clear all user-placed points
//   SPACE              Pause / resume
//
// Architecture:
//   swarm_bahviour.pde      — Setup, draw loop, key/mouse dispatch
//   SwarmManager.pde        — Core manager: update loop, universal forces, display
//   Boid.pde                — Boid agent: physics, force functions, rendering
//   RepulsionZone.pde       — Circular repulsion zone (R items)
//   AttRepBehaviour.pde     — Attraction/Repulsion (mode 2)
//   ContextSteeringBehaviour.pde  — Context steering + Combined (modes 3, 4)
//   PsoBehaviour.pde        — Particle Swarm Optimisation (mode 5)
//   CuckerSmaleBehaviour.pde — Cucker-Smale flocking (mode 6)
//   VicsekBehaviour.pde     — Vicsek flocking (mode 7)
//   MorphogeneticBehaviour.pde — Morphogenetic swarm (mode 8)
//   AcoBehaviour.pde        — Ant Colony Optimisation (mode 9)
//   SppBehaviour.pde        — Social Positioning Protocol (mode 0)
// ============================================================

// Shared utility: cosine similarity between two vectors
static float cosine_sim(PVector a, PVector b) {
  return a.dot(b) / (a.mag() * b.mag());
}

SwarmManager swarm;
boolean paused = false;
char mode = 'G'; // G=goal, D=danger, R=repulsion zone, B=boid, M=mouse

void setup() {
  size(4800, 3200);
  swarm = new SwarmManager();
  swarm.initBehaviour(BehaviourFlag.ATT_REP);
}

void draw() {
  background(20);

  if (!paused) {
    // Periodically rotate target/danger positions
    swarm.autoUpdate(2);
    // Update all boids and forces
    swarm.update();
  }

  // M mode: mouse becomes a moving target (LEFT) or danger (RIGHT) while held
  if (mode == 'M') {
    if (mousePressed) {
      if (mouseButton == LEFT) {
        swarm.mouseTarget = new PVector(mouseX, mouseY);
        swarm.mouseDanger = null;
      } else if (mouseButton == RIGHT) {
        swarm.mouseDanger = new PVector(mouseX, mouseY);
        swarm.mouseTarget = null;
      }
    } else {
      swarm.mouseTarget = null;
      swarm.mouseDanger = null;
    }
  } else {
    swarm.mouseTarget = null;
    swarm.mouseDanger = null;
  }

  swarm.display();

  // HUD overlay (top-left 800×640 safe zone)
  fill(255);
  textSize(48);
  text("Boids: " + swarm.boids.size(), 40, 80);
  text("Mode: " + mode, 40, 160);
  text("Behaviour: " + swarm.behaviourFlag, 40, 240);
  text("Paused: " + paused, 40, 320);
  text("[1]BASIC [2]ATT_REP [3]CON_STEER [4]COMBINED", 40, 400);
  text("[5]PSO [6]CUCKER_SMALE [7]VICSEK", 40, 460);
  text("[8]MORPH [9]ACO [0]SPP", 40, 520);
  text("UP/DOWN += -= boids  C=clear  Space=pause", 40, 580);
  text("[G]oal  [D]anger  [R]epulsion  [B]oid  [M]ouse", 40, 640);
  text("LMB=spawn  RMB=delete last", 40, 700);
  textSize(12);
}

void keyPressed() {
  // Population controls
  if (keyCode == UP)   { swarm.addRandomBoids(10); }
  if (keyCode == DOWN) { swarm.removeRandomBoids(10); }

  // Behaviour mode switches (number keys)
  if (key == '1') { swarm.initBehaviour(BehaviourFlag.BASIC); }
  if (key == '2') { swarm.initBehaviour(BehaviourFlag.ATT_REP); }
  if (key == '3') { swarm.initBehaviour(BehaviourFlag.CON_STEER); }
  if (key == '4') { swarm.initBehaviour(BehaviourFlag.COMBINED); }
  if (key == '5') { swarm.initBehaviour(BehaviourFlag.PSO); }
  if (key == '6') { swarm.initBehaviour(BehaviourFlag.CUCKER_SMALE); }
  if (key == '7') { swarm.initBehaviour(BehaviourFlag.VICSEK); }
  if (key == '8') { swarm.initBehaviour(BehaviourFlag.MORPHOGENETIC); }
  if (key == '9') { swarm.initBehaviour(BehaviourFlag.ACO); }
  if (key == '0') { swarm.initBehaviour(BehaviourFlag.SPP); }

  // Spawn mode selection
  if (key == 'g' || key == 'G') mode = 'G';
  if (key == 'd' || key == 'D') mode = 'D';
  if (key == 'r' || key == 'R') mode = 'R';
  if (key == 'b' || key == 'B') mode = 'B';
  if (key == 'm' || key == 'M') mode = 'M';

  // Utility
  if (key == 'c' || key == 'C') swarm.clearAll();
  if (key == ' ') paused = !paused;
}

void mousePressed() {
  swarm.mousePressed(mode);
}
