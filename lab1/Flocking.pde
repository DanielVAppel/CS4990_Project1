// Initialize the flock array
ArrayList<Boid> flock = new ArrayList<Boid>();

// called when "f" is pressed; should instantiate additional boids and start flocking
void flock() {
  if (flock.size() == 0) { // Only add Boids if the flock is empty
    for (int i = 0; i < 8; i++) {
// Spawn near Billy's position to ensure close formation
      float x = billy.kinematic.getPosition().x + random(-60, 60); // Adjust spawn distance from Billy
      float y = billy.kinematic.getPosition().y + random(-60, 60);
      PVector random_position = new PVector(x, y);
      Boid new_boid = new Boid(random_position, billy.kinematic.getHeading(), BILLY_MAX_SPEED, BILLY_MAX_ROTATIONAL_SPEED, BILLY_MAX_ACCELERATION, BILLY_MAX_ROTATIONAL_ACCELERATION);
      //Boid new_boid = new Boid(random_position, BILLY_START_HEADING, BILLY_MAX_SPEED, BILLY_MAX_ROTATIONAL_SPEED, BILLY_MAX_ACCELERATION, BILLY_MAX_ROTATIONAL_ACCELERATION);
      flock.add(new_boid);
    }
  }
}

// called when "f" is pressed again; should remove the flock
void unflock() {
  flock.clear();
}

//draw and update each Boid in the flock
void drawFlock(float dt) {
  update_flocking(flock);
  for (Boid boid : flock) {
    boid.update(dt, flock);
    //boid.update(dt, flock);
    boid.draw();
  }
}

// Update the movement for each Boid in the flockement for boids
void update_flocking(ArrayList<Boid> flock) {
  float billyHeading = billy.kinematic.getHeading(); // Align flock heading with Billy
  float billySpeed = billy.kinematic.getSpeed();// Billy's speed

  for (Boid boid : flock) {
    // Calculate alignment, cohesion, separation, and seeking Billy
    PVector alignment = alignment(boid, flock).mult(1.0); // Adjust the weight as needed
    PVector cohesion = cohesion(boid, flock).mult(1.0);   // Adjust the weight as needed
    PVector separation = separation(boid, flock).mult(5.0); // Separation might need more weight
    
    // Boids move in the direction Billy is facing without directly chasing him
     PVector moveDirection = new PVector(cos(billyHeading), sin(billyHeading)).mult(billySpeed);
        
    // Combine all steering forces
    PVector steering = PVector.add(alignment, cohesion);
    steering.add(separation);
    steering.add(moveDirection);
    
    // Limit the combined steering force
    steering.limit(1.5);// Adjust this limit to balance the movement
    float currentSpeed = boid.kinematic.getSpeed();

    // Prevent Boids from moving independently before Billy starts
    if (billySpeed > 0) {
        float desiredSpeed = billySpeed ; // Sync speed with Billy
        
    // Gradually adjust speed with a delay factor
        float speedChange = (desiredSpeed - currentSpeed) * .05;// 0.05 factor introduces delay
        boid.kinematic.increaseSpeed(speedChange, 0);
    
    // Align Boid's heading gradually towards Billy's direction with a slight delay
        float targetHeading = billyHeading;  // Align directly with Billy's heading
        float currentHeading = boid.kinematic.getHeading();
        float headingChange = normalize_angle_left_right(targetHeading - currentHeading);
            
        if (abs(headingChange) > 0.1) { // Only rotate if necessary to prevent jitter
                headingChange = constrain(headingChange, -boid.kinematic.max_rotational_speed * 0.5, boid.kinematic.max_rotational_speed * 0.5);
                boid.kinematic.increaseSpeed(0, headingChange);
            } else {
                boid.kinematic.increaseSpeed(0, -boid.kinematic.getRotationalVelocity() * 0.5); // Reduce rotation gradually
            }
    
    // Handle stopping when Billy stops
    } else {
            float stopSpeed = boid.kinematic.getSpeed();  // Gradually slow down
            boid.kinematic.increaseSpeed(-stopSpeed, -boid.kinematic.getRotationalVelocity());
    }
  }
}
PVector seek(Boid boid, PVector target) {
  PVector desired = PVector.sub(target, boid.kinematic.getPosition());
  float distance = desired.mag();
  
  if (distance > 0) {
    desired.normalize();
    desired.mult(boid.kinematic.max_speed);
  
    PVector currentVelocity = PVector.sub(desired, new PVector(cos(boid.kinematic.getHeading()), sin(boid.kinematic.getHeading())));
    currentVelocity.mult(constrain(boid.kinematic.getSpeed(), 0, boid.kinematic.max_speed));
  
    PVector steer = PVector.sub(desired, currentVelocity);
    steer.limit(0.1);
    return steer;
  }
  
  return new PVector(0, 0);
}

//separation
PVector separation(Boid boid, ArrayList<Boid> flock) {
  PVector steer = new PVector(0, 0);
  int count = 0;
  
  for (Boid other : flock) {
    if (other != boid) {
      float distance = PVector.dist(boid.kinematic.getPosition(), other.kinematic.getPosition());
      if (distance > 0 && distance < 40) {
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
    steer.limit(3.0); //avoid erratic movement
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
