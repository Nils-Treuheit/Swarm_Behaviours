import java.util.ArrayList;

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
    swarm.autoUpdate(2);
    swarm.update();
  }

  // M mode: mouse becomes moving target/danger while held
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
  swarm.removeVisited();

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
  if (keyCode == UP) { swarm.addRandomBoids(10); }
  if (keyCode == DOWN) { swarm.removeRandomBoids(10); }

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

  if (key == 'g' || key == 'G') mode = 'G';
  if (key == 'd' || key == 'D') mode = 'D';
  if (key == 'r' || key == 'R') mode = 'R';
  if (key == 'b' || key == 'B') mode = 'B';
  if (key == 'm' || key == 'M') mode = 'M';
  if (key == 'c' || key == 'C') swarm.clearAll();
  if (key == ' ') paused = !paused;
}

void mousePressed() {
  swarm.mousePressed(mode);
}
