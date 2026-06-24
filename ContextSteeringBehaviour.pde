// ============================================================
// Context Steering Behaviour Module
//
// Part of SwarmManager.
// Implements a ray-casting context steering approach for both
// CON_STEER (mode 3) and COMBINED (mode 4).
//
// CON_STEER: boids evaluate interest (goals) and danger
// (dangers, borders, member repulsion) in each of 8 directional
// sectors, then pick the best sector to steer toward.
//
// COMBINED: adds leader-follower dynamics. The drone closest to
// the nearest attractor becomes leader and navigates; followers
// steer toward the leader while maintaining flock cohesion and
// avoiding repulsion zones.
// ============================================================

// ---------------------------------------------------------------
// applyConSteer: standard context steering (mode 3)
// ---------------------------------------------------------------
void applyConSteer(SwarmManager sm, Boid b) {
  for (int idx = 0; idx < sm.RAY_DIRS.size(); ++idx) {
    ArrayList<PVector> intrest_forces   = new ArrayList<PVector>();
    ArrayList<PVector> member_atts      = new ArrayList<PVector>();
    ArrayList<PVector> member_reps      = new ArrayList<PVector>();
    ArrayList<PVector> danger_forces    = new ArrayList<PVector>();
    ArrayList<PVector> alignment_forces = new ArrayList<PVector>();
    boolean masked = false;

    // Interest: goals in this sector (skip if carrying)
    if (!b.carrying) {
      for (PVector goal : sm.goalsCS)
        if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(goal, b.pos)) >= sm.SECTOR_COS_SIM)
          intrest_forces.add(b.linear_Attraction_CS(goal, sm.GOAL_LIMIT, sm.GOAL_SIGMA, sm.GOAL_GAMMA));
    }

    // Flocking: member attraction, repulsion, alignment
    for (Boid other : sm.boids) {
      if (other != b) {
        if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos)) >= sm.SECTOR_COS_SIM) {
          float member_dist = PVector.sub(other.pos, b.pos).mult(sm.PIXEL_METRIC_CONV).mag();
          if (member_dist <= sm.MEMBER_TO_CLOSE) masked = true;
          else if (member_dist <= sm.SWARM_DIST) alignment_forces.add(other.vel);
          member_atts.add(b.log_Attraction(other.pos, sm.MEMBER_ATT_LIMIT, sm.MEMBER_ATT_CUT_OFF));
        }
        if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos).rotate(PI)) >= sm.SECTOR_COS_SIM) {
          member_reps.add(b.linear_Repulsion(other.pos, sm.MEMBER_REP_LIMIT, sm.MEMBER_REP_SIGMA, sm.MEMBER_REP_GAMMA, sm.MEMBER_REP_ALPHA));
        }
      }
    }

    // Danger: objects behind the boid in this sector
    for (PVector danger : sm.dangersCS) {
      if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(danger, b.pos).rotate(PI)) >= sm.SECTOR_COS_SIM)
        danger_forces.add(b.limExp_Repulsion(danger, sm.DANGER_LIMIT, sm.DANGER_CUT_OFF, sm.DANGER_SIGMA, sm.DANGER_GAMMA, sm.DANGER_ALPHA));
      else if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(danger, b.pos)) >= sm.SECTOR_COS_SIM) {
        float member_dist = PVector.sub(danger, b.pos).mult(sm.PIXEL_METRIC_CONV).mag();
        if (member_dist <= sm.DANGER_TO_CLOSE) masked = true;
      }
    }

    // Border proximity masks this sector
    for (PVector bp : sm.border_points)
      if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(bp, b.pos)) >= sm.SECTOR_COS_SIM) {
        float member_dist = PVector.sub(bp, b.pos).mult(sm.PIXEL_METRIC_CONV).mag();
        if (member_dist <= sm.BORDER_TO_CLOSE) masked = true;
      }

    b.create_context_segment(idx, intrest_forces, danger_forces, member_atts, member_reps, alignment_forces, !masked);
  }

  b.context_steering(sm.RAY_DIRS, sm.SECTOR_COS_SIM);

  b.integrate();
}

// ---------------------------------------------------------------
// applyCombined: leader-follower context steering (mode 4)
// ---------------------------------------------------------------
void applyCombined(SwarmManager sm, Boid b, Boid leader) {
  boolean isLeader = (leader == b);

  for (int idx = 0; idx < sm.RAY_DIRS.size(); ++idx) {
    ArrayList<PVector> intrest_forces   = new ArrayList<PVector>();
    ArrayList<PVector> member_atts      = new ArrayList<PVector>();
    ArrayList<PVector> member_reps      = new ArrayList<PVector>();
    ArrayList<PVector> danger_forces    = new ArrayList<PVector>();
    ArrayList<PVector> alignment_forces = new ArrayList<PVector>();
    boolean masked = false;

    // Leader navigates toward attractors; followers steer toward leader
    if (isLeader && !b.carrying) {
      for (PVector a : sm.attractors)
        if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(a, b.pos)) >= sm.SECTOR_COS_SIM)
          intrest_forces.add(b.linear_Attraction_CS(a, sm.GOAL_LIMIT, sm.GOAL_SIGMA, sm.GOAL_GAMMA));
    } else if (leader != null) {
      if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(leader.pos, b.pos)) >= sm.SECTOR_COS_SIM)
        intrest_forces.add(b.strongAttraction(leader.pos, sm.FOLLOW_LIMIT, sm.FOLLOW_SCALE, sm.FOLLOW_DEADZONE));
    }

    // Flocking with stronger forces for tight cohesion
    for (Boid other : sm.boids) {
      if (other != b) {
        if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos)) >= sm.SECTOR_COS_SIM) {
          float member_dist = PVector.sub(other.pos, b.pos).mult(sm.PIXEL_METRIC_CONV).mag();
          if (member_dist <= sm.COMBINED_MEMBER_TO_CLOSE) masked = true;
          else if (member_dist <= sm.SWARM_DIST) alignment_forces.add(other.vel);
          member_atts.add(b.strongAttraction(other.pos, sm.COMBINED_ATT_LIMIT, sm.COMBINED_ATT_SCALE, sm.COMBINED_ATT_DEADZONE));
        }
        if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(other.pos, b.pos).rotate(PI)) >= sm.SECTOR_COS_SIM) {
          member_reps.add(b.strongRepulsion(other.pos, sm.COMBINED_REP_LIMIT, sm.COMBINED_REP_SCALE));
        }
      }
    }

    // Danger from RepulsionZones (R items)
    for (RepulsionZone z : sm.zones) {
      PVector toZone = PVector.sub(z.pos, b.pos);
      float dist = toZone.mag();
      float zoneLimit = z.radius + sm.ZONE_DANGER_LIMIT;

      if (dist < zoneLimit) {
        boolean forward = cosine_sim(sm.RAY_DIRS.get(idx), toZone) >= sm.SECTOR_COS_SIM;
        boolean behind  = cosine_sim(sm.RAY_DIRS.get(idx), toZone.rotate(PI)) >= sm.SECTOR_COS_SIM;

        if (forward) {
          float strength;
          if (dist < z.radius) strength = (z.radius - dist) * 0.5;
          else                 strength = (zoneLimit - dist) / zoneLimit * 2;
          PVector repForce = PVector.sub(b.pos, z.pos);
          repForce.normalize();
          repForce.mult(strength);
          danger_forces.add(repForce);
          if (dist < z.radius + sm.ZONE_TO_CLOSE) masked = true;
        } else if (behind && dist < z.radius) {
          masked = true;
        }
      }
    }

    // Border proximity masks this sector
    for (PVector bp : sm.border_points)
      if (cosine_sim(sm.RAY_DIRS.get(idx), PVector.sub(bp, b.pos)) >= sm.SECTOR_COS_SIM) {
        float member_dist = PVector.sub(bp, b.pos).mult(sm.PIXEL_METRIC_CONV).mag();
        if (member_dist <= sm.BORDER_TO_CLOSE) masked = true;
      }

    b.create_context_segment(idx, intrest_forces, danger_forces, member_atts, member_reps, alignment_forces, !masked);
  }

  b.context_steering(sm.RAY_DIRS, sm.SECTOR_COS_SIM);

  b.integrate();
}
