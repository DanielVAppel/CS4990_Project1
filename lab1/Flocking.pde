ArrayList<Boid> flock = new ArrayList<Boid>();

/// called when "f" is pressed; should instantiate additional boids and start flocking
void flock() {
  for (int i = 0; i < 8; i++) {
    float x = random(0, width);
    float y = random(0, height);
    PVector random_position = new PVector(x, y);
    Boid new_boid = new Boid(random_position, BILLY_START_HEADING, BILLY_MAX_SPEED, BILLY_MAX_ROTATIONAL_SPEED, BILLY_MAX_ACCELERATION, BILLY_MAX_ROTATIONAL_ACCELERATION);
    flock.add(new_boid);
  }
}

/// called when "f" is pressed again; should remove the flock
void unflock() {
  flock.clear();
}

//draw boids
void drawFlock(float dt) {
  for (Boid boid : flock) {
    update_flocking(flock, dt);
    boid.update(dt, flock);
    boid.draw();
  }
}

//movement for boids
void update_flocking(ArrayList<Boid> flock, float dt) {
  for (Boid boid : flock) {
    PVector alignment = alignment(boid, flock);
    PVector cohesion = cohesion(boid, flock);
    PVector separation = separation(boid, flock);
    
    PVector steering = PVector.add(alignment, cohesion);
    steering.add(separation);
    
    if (steering.mag() > 0.5) { //limit steering force
      steering.setMag(0.5);
    }
    
    float currentSpeed = boid.kinematic.getSpeed();
    float newSpeed = currentSpeed + steering.mag();
    boid.kinematic.increaseSpeed(newSpeed - currentSpeed, 0);
  }
}

//separation
PVector separation(Boid boid, ArrayList<Boid> flock) {
  PVector steer = new PVector(0, 0);
  int count = 0;
  
  for (Boid other : flock) {
    if (other != boid) {
      float distance = PVector.dist(boid.kinematic.getPosition(), other.kinematic.getPosition());
      if (distance > 0 && distance < 20) {
        PVector diff = PVector.sub(boid.kinematic.getPosition(), other.kinematic.getPosition());
        diff.div(distance);
        steer.add(diff);
        count++;
      }
    }
  }
  if (count > 0) {
    steer.div(count);
    steer.normalize();
    steer.mult(boid.kinematic.max_speed);
    steer.limit(1.5); //avoid erratic movement
  }
  return steer;
}

//cohesion
PVector cohesion(Boid boid, ArrayList<Boid> flock) {
  PVector cohesion = new PVector(0, 0);
  int count = 0;
  
  for (Boid other : flock) {
    if (other != boid) {
      float distance = PVector.dist(boid.kinematic.getPosition(), other.kinematic.getPosition());
      if (distance > 0 && distance < 50) {
        cohesion.add(other.kinematic.getPosition());
        count++;
      }
    }
  }
  
  if (count > 0) {
    cohesion.div(count); //average position
    cohesion.sub(boid.kinematic.getPosition()); //get direction
    cohesion.limit(1.0); //scale force
  }
  return cohesion;
}

PVector alignment(Boid boid, ArrayList<Boid> flock) {
  PVector alignment = new PVector(0, 0);
  int count = 0;
  
  for (Boid other : flock) {
    if (other != boid) {
      float distance = PVector.dist(boid.kinematic.getPosition(), other.kinematic.getPosition());
      if (distance > 0 && distance < 50) { //boids within certain distance
        //sum headings
        alignment.add(new PVector(cos(other.kinematic.getHeading()), sin(other.kinematic.getHeading())));
        count++;
      }
    }
  }
  
  if (count > 0) {
    alignment.div(count); //average the direction
    alignment.limit(1.0); //scale the force
  }
  return alignment;
}
