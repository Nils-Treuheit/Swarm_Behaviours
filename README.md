# Swarm Behaviours

An interactive multi-agent swarm simulation in Processing (Java) that lets you explore 10 different swarm intelligence algorithms side by side. Place targets, dangers, repulsion zones, and boids with your mouse; switch algorithms at runtime and watch the swarm adapt.

## Controls

### Spawn modes (letter keys)

| Key | Mode     | Left click                 | Right click         |
|-----|----------|----------------------------|---------------------|
| `G` | Goal     | Place green target         | Remove last goal    |
| `D` | Danger   | Place red danger point     | Remove last danger  |
| `R` | Repulsion| Place red repulsion circle | Remove last circle  |
| `B` | Boid     | Spawn a boid at cursor     | Remove last boid    |
| `M` | Mouse    | Hold = moving target       | Hold = moving danger|

### Swarm algorithms (number keys)

| Key | Algorithm         | Behaviour                                                  |
|-----|-------------------|------------------------------------------------------------|
| `1` | BASIC             | Classic Reynolds flocking (alignment, cohesion, separation)|
| `2` | ATT_REP           | Attraction/repulsion point forces                          |
| `3` | CON_STEER         | Context steering with 8-sector interest/danger maps        |
| `4` | COMBINED          | Leader-follower + context steering + zone avoidance        |
| `5` | PSO               | Particle Swarm Optimization (personal/global best)         |
| `6` | CUCKER_SMALE      | Cucker–Smale distance-weighted velocity alignment          |
| `7` | VICSEK            | Vicsek local directional averaging with noise              |
| `8` | MORPHOGENETIC     | Gene-regulatory-network-inspired morphogen dynamics        |
| `9` | ACO               | Ant Colony Optimization with pheromone grid               |
| `0` | SPP               | Social Positioning Protocol (weight-based clustering)      |

### Other

| Input | Action                          |
|-------|---------------------------------|
| `UP`  | Spawn 10 random boids           |
| `DOWN`| Remove 10 random boids          |
| `C`   | Clear all placed items          |
| `Space`| Pause/unpause                  |

## Visual legend

| Element          | Colour   | Meaning                                         |
|------------------|----------|-------------------------------------------------|
| Green point      | ●        | Target / goal — boids seek these                |
| Red point        | ●        | Danger point — high local repulsion             |
| Red unfilled circle | ○   | Repulsion zone — softer area-wide avoidance     |
| White point      | ●        | Boid (normal); dot size scales with mass        |
| Cyan point       | ●        | Boid carrying an item (dot size = mass)         |
| Blue circle      | ○        | Center delivery zone (cornflowerblue, active when any boid carries) |
| Cyan point (M)   | ●        | Moving mouse target when held                   |
| Orange point (M) | ●        | Moving mouse danger when held                   |
| Yellow dots      | ·        | ACO pheromone trail                             |

## Carrying mechanic

When a boid touches a target (distance < 2px), it picks it up — the target disappears from the world and the boid turns cyan. While carrying:
- The boid is **no longer attracted** to any targets (including mouse targets).
- A **cornflowerblue circle** appears at the center of the screen (2400, 1600).
- The boid receives a **10× strength attraction** toward the center.
- All other boids behave normally (still attracted to remaining targets).
- The boid's flight path is visible as a fading tail (last 15 frames).

When the carrying boid reaches the center (distance < 10px), it releases the item and resumes normal behaviour.

### Danger vs Repulsion zone

- **Danger (`D`)** — a point of extreme local repulsion. Very strong at close range but drops off quickly (~60px).
- **Repulsion zone (`R`)** — a circle with random radius. Larger circles push more gently but affect boids from farther away.

## Behaviour details

### BASIC (1)
Standard Craig Reynolds boids with alignment, cohesion, and separation. Attracted to green targets and repelled by repulsion zones.

### ATT_REP (2)
Every point exerts a force: attractors pull with `G * multiplier / distance`; repellents push with a Gaussian-exponential falloff. Inter-boid forces maintain a comfortable spacing.

### CON_STEER (3)
Each boid evaluates 8 directional sectors. Interest forces (goals) and danger forces are accumulated per sector; the sector with the strongest net interest is chosen as the movement direction. Member alignment, attraction, and repulsion are sector-aware. Border proximity masks sectors.

### COMBINED (4)
Leader-follower paradigm: the boid closest to any attractor becomes the leader and navigates toward targets using context steering. All other boids steer toward the leader instead. Strong inverse-square member repulsion keeps the formation from collapsing.

### PSO (5)
Each boid tracks a personal best position (closest it has been to any attractor) and the swarm shares a global best. Velocity is updated as `inertia + cognitive(pbest - pos) + social(gbest - pos)`. When a target is picked up, all personal bests reset automatically.

### CUCKER_SMALE (6)
Velocity alignment weighted by distance: `weight = h / (h + d²)`. Boids far apart barely influence each other; nearby boids align strongly. Inter-boid repulsion prevents collapse.

### VICSEK (7)
Each boid adopts the average heading of neighbours within a fixed radius, plus a noise term. The Vicsek direction is treated as a steering force blended with goal/danger forces.

### MORPHOGENETIC (8)
Inspired by gene regulatory networks. Each boid carries a morphogen concentration that evolves via neighbour diffusion, production toward a random target level, and decay. High morphogen → strong goal-seeking; low morphogen → exploratory wandering with heightened danger sensitivity.

### ACO (9)
A pheromone grid (50px cells) overlays the world. The centre emits a constant semi-local pheromone beacon (home). Non-carrying boids explore with random wander and weak attraction to targets; they also follow pheromone gradients to converge on trails left by returning boids. Carrying boids follow the pheromone gradient uphill toward the centre emitter and deposit heavy pheromone on their return path, creating visible ant-like trail networks. Pheromone evaporates and diffuses each frame. Yellow dots (size/alpha mapped to intensity) show trail density.

Pheromone deposition is **speed-gated** (threshold ~0.15) — stationary/wiggling boids leave no trail. Faster boids deposit more, scaled by their **activity** (sustained movement history). Deposits are also **rear-weighted**: the faster the boid, the more pheromone is placed at the cell behind it rather than its current position, creating direction-aware ant trails.

### SPP (0)
Social Positioning Protocol. Each boid gets a random social weight in [0,1]. Boids with similar weights attract (form same-weight clusters); boids with different weights repel.

## HUD safe zone

The top-left 800×640 area is reserved for the HUD. Auto-spawned points never appear there, and a repulsion force pushes boids away from this zone in every mode.

## Technical notes

- **Canvas**: 4800×3200 (4× scaled from a 1200×800 virtual space).
- **Language**: Processing 4+ (Java mode).
- **File structure**:

| File                          | Purpose                                                   |
|-------------------------------|-----------------------------------------------------------|
| `swarm_bahviour.pde`          | Main entry: setup, draw loop, key/mouse dispatch, HUD     |
| `SwarmManager.pde`            | Core manager: update loop, universal forces, display,     |
|                               | point management, population control                      |
| `Boid.pde`                    | Boid agent: mass/size physics, force functions, rendering,|
|                               | context steering, `integrate()` method                    |
| `RepulsionZone.pde`           | Circular repulsion zone (R items)                         |
| `AttRepBehaviour.pde`         | Attraction/Repulsion (mode 2)                             |
| `ContextSteeringBehaviour.pde`| Context steering (mode 3) + Combined leader-follower (4)  |
| `PsoBehaviour.pde`            | Particle Swarm Optimisation (mode 5)                      |
| `CuckerSmaleBehaviour.pde`    | Cucker-Smale flocking (mode 6)                            |
| `VicsekBehaviour.pde`         | Vicsek flocking (mode 7)                                  |
| `MorphogeneticBehaviour.pde`  | Morphogenetic swarm (mode 8)                              |
| `AcoBehaviour.pde`            | Ant Colony Optimisation (mode 9) — pheromone grid         |
| `SppBehaviour.pde`            | Social Positioning Protocol (mode 0)                      |

Each behaviour module defines a single `apply*` method on the `SwarmManager` class. Processing concatenates all `.pde` files, so methods can be spread across files while belonging to the same class.

- **Mass / Size**: each boid spawns with a random mass 0.5–2.0.
  - `maxSpeed = 2.5 / mass` — heavy boids are slower.
  - `maxForce = 0.08 / mass` — heavy boids steer less aggressively.
  - `acc /= mass` during integration (`F = ma`), so identical forces produce less acceleration on heavy boids.
  - Trail thickness and dot size scale linearly with mass (4–7 px for the dot, `frac * 2.5 * mass` for trail stroke).
  - Light boids zip around with weak force impact; heavy boids lumber powerfully through the swarm.

- **Border**: a dense ring of border points (every pixel on the canvas edge) exerts exponential repulsion in all modes.
