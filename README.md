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
| Green point      | ●        | Target / goal — boids are attracted             |
| Green unfilled circle | ○ | Goal object (BASIC mode only)              |
| Red point        | ●        | Danger — high local repulsion, avoid at all cost|
| Red unfilled circle   | ○  | Repulsion zone — softer area-wide avoidance     |
| White point      | ●        | Boid / drone                                    |
| Cyan point       | ●        | Moving mouse target (M mode)                    |
| Orange point     | ●        | Moving mouse danger (M mode)                    |
| Golden overlay   | ▨        | ACO pheromone trail (ACO mode only)             |

### Danger vs Repulsion zone

- **Danger (`D`)** — a point of extreme local repulsion. Very strong at close range but drops off quickly (~60px). Useful for obstacles that must be avoided at all cost.
- **Repulsion zone (`R`)** — a circle with random radius. Larger circles push more gently but affect boids from farther away. Smaller circles push harder but only nearby.

## Behaviour details

### BASIC (1)
Standard Craig Reynolds boids. Alignment, cohesion, and separation combined with attraction to Goal objects and avoidance of RepulsionZone circles.

### ATT_REP (2)
Every point exerts a force: attractors (green) pull with `G * multiplier / distance`; repellents (red) push with a Gaussian-exponential falloff. Inter-boid forces maintain a comfortable spacing.

### CON_STEER (3)
Each boid evaluates 8 directional sectors. Interest forces (goals) and danger forces are accumulated per sector; the sector with the strongest net interest is chosen as the movement direction. Member alignment, attraction, and repulsion are sector-aware. Border proximity masks sectors.

### COMBINED (4)
Leader-follower paradigm: the boid closest to any attractor becomes the leader and navigates toward targets using context steering. All other boids steer toward the leader instead. Strong inverse-square member repulsion keeps the formation from collapsing.

### PSO (5)
Each boid tracks a personal best position (closest it has been to any attractor) and the swarm shares a global best. Velocity is updated as `inertia + cognitive(pbest - pos) + social(gbest - pos)`. When an attractor is consumed the personal bests reset automatically.

### CUCKER_SMALE (6)
Velocity alignment weighted by distance: `weight = h / (h + d²)`. Boids far apart barely influence each other; nearby boids align strongly. Inter-boid repulsion prevents the collapse that pure Cucker–Smale would produce.

### VICSEK (7)
Each boid adopts the average heading of neighbours within a fixed radius, plus a noise term. The Vicsek direction is treated as a steering force (blended with goal/danger forces) rather than an instantaneous velocity overwrite.

### MORPHOGENETIC (8)
Inspired by gene regulatory networks. Each boid carries a morphogen concentration that evolves via neighbour diffusion, production toward a random target level, and decay. High morphogen → strong goal-seeking; low morphogen → exploratory wandering with heightened danger sensitivity.

### ACO (9)
A pheromone grid (50px cells) overlays the world. Boids deposit pheromone near attractors and follow pheromone gradients for steering. Pheromone evaporates and diffuses each frame. The golden grid overlay shows the current trail density.

### SPP (0)
Social Positioning Protocol. Each boid gets a random social weight in [0,1]. Boids with similar weights attract (form same-weight clusters); boids with different weights repel. Combined with goal-seeking and danger avoidance, this produces self-organized spatial groupings.

## HUD safe zone

The top-left 800×640 area is reserved for the HUD. Auto-spawned points never appear there, and a repulsion force pushes boids away from this zone in every mode.

## Technical notes

- **Canvas**: 4800×3200 (4× scaled from a 1200×800 virtual space).
- **Language**: Processing 4+ (Java mode).
- **Files**: `swarm_bahviour.pde`, `SwarmManager.pde`, `Boid.pde`, `Goal.pde`, `RepulsionZone.pde`.
- **Border**: a dense ring of border points (every pixel on the canvas edge) exerts exponential repulsion in all modes.
