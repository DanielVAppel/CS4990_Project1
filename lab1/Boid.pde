/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

// Crumb class handles the trail of crumbs left behind by the Boid
class Crumb
{
  PVector position;
  
   // Constructor for the crumb, sets the position
  Crumb(PVector position)
  {
     this.position = position;
  }
  
  // Draws the crumb on the screen
  void draw()
  {
     fill(255);
     noStroke(); 
     circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

// Boid class handles movement and behavior of the Boid
class Boid
{
   Crumb[] crumbs = {};               // Array to store crumbs left by the Boid
   int last_crumb;                    // Timestamp of the last crumb placed
   float acceleration;                // Forward acceleration of the Boid
   float rotational_acceleration;     // Rotational acceleration of the Boid
   KinematicMovement kinematic;       // Kinematic movement handler
   PVector target;                    // Current target of the Boid
   float arrivalRadius = 300;       // Distance within which the Boid starts to slow down

   
    // Constructor for the Boid, initializing its kinematic movement and parameters
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }
   
  // Update method called every frame to adjust the Boid's position and orientation
   void update(float dt)
   {
    // Check if there is a target to seek
     if (target != null)
     {  
        // Calculate the desired heading using the target's position
        // TODO: Implement seek here
        float desired_heading = atan2(target.y - kinematic.getPosition().y, target.x - kinematic.getPosition().x);
        float current_heading = kinematic.getHeading();
        
        // Calculate the angle difference between current heading and desired heading
        float angle_difference = normalize_angle_left_right(desired_heading - current_heading); //TODO        
       
       // Adjust rotational speed to ensure smoother turns and prevent overshooting
      float rotationThreshold = 0.2;  // Set a small threshold for acceptable alignment
       // Rotate towards target if needed
        if (abs(angle_difference) > rotationThreshold) {
            float rotational_direction = (angle_difference > 0 ? 1 : -1) * rotational_acceleration *dt;
            // Scale rotational speed down as it gets closer to the target heading
            rotational_direction *= map(abs(angle_difference), 0, PI, 0.1, 1); // Adjust rotation speed dynamically
            kinematic.increaseSpeed(0, rotational_direction);
        } else {
        kinematic.increaseSpeed(0, -kinematic.getRotationalVelocity() * 0.5); // Gradually reduce rotation
      }
        
      // Adjust forward movement based on distance to prevent overshooting waypoints
      float distanceToTarget = dist(kinematic.getPosition().x, kinematic.getPosition().y, target.x, target.y);
      if (distanceToTarget > 10) {  // 10 is the arrival radius
        // Slow down forward speed as it gets closer to avoid circling around
        float forwardAcceleration = map(distanceToTarget, 0, 100, 0, acceleration * dt * 2);  // Scale dynamically
        kinematic.increaseSpeed(forwardAcceleration, 0);
      } else {
        // Stop movement when within arrival radius
        kinematic.increaseSpeed(-kinematic.getSpeed(), -kinematic.getRotationalVelocity());
        target = null; // Clear target to stop the circling behavior
      }
    }
     
     // place crumbs, do not change     
     if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
     {
        this.last_crumb = millis();
        this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position));
        if (this.crumbs.length > MAX_CRUMBS)
           this.crumbs = (Crumb[])subset(this.crumbs, 1);
     }
     
     // do not change
     this.kinematic.update(dt);
     
     draw();
   }
   
   // Draws the Boid and its crumbs on the screen
   void draw()
   {
     // Draw crumbs left by the Boid
     for (Crumb c : this.crumbs)
     {
       c.draw();
     }
     
     // Draw the Boid itself
     fill(255);
     noStroke(); 
     float x = kinematic.position.x;
     float y = kinematic.position.y;
     float r = kinematic.heading;
     circle(x, y, BOID_SIZE);
     
    // Draw the front of the Boid
     float xp = x + BOID_SIZE*cos(r);
     float yp = y + BOID_SIZE*sin(r);
     
    // Draw the left side of the Boid
     float x1p = x - (BOID_SIZE/2)*sin(r);
     float y1p = y + (BOID_SIZE/2)*cos(r);
     
    // Draw the right side of the Boid
     float x2p = x + (BOID_SIZE/2)*sin(r);
     float y2p = y - (BOID_SIZE/2)*cos(r);
     triangle(xp, yp, x1p, y1p, x2p, y2p);
   } 
   
   // Sets a new target for the Boid to seek
   void seek(PVector target)
   {
      this.target = target;
      
   }
   
   // Handle path-following with waypoints
   void follow(ArrayList<PVector> waypoints) {
    // Only proceed if there are waypoints left to follow
    if (waypoints.size() > 0) {
        // Check if Billy has reached the current target waypoint
        if (target == null || dist(kinematic.getPosition().x, kinematic.getPosition().y, target.x, target.y) < arrivalRadius) {
            // Set the next waypoint as the target
            target = waypoints.get(0); // Get the next waypoint
            waypoints.remove(0);      // Remove the waypoint after setting it as the target
        }

        // Continue moving towards the current target without stopping
        float distanceToTarget = dist(kinematic.getPosition().x, kinematic.getPosition().y, target.x, target.y);
        if (distanceToTarget > 10) {
            // Maintain speed while moving towards the waypoint
            kinematic.increaseSpeed(acceleration, 0);
        }
    } else {
        // No more waypoints, stop completely
        target = null;
        kinematic.increaseSpeed(-kinematic.getSpeed(), -kinematic.getRotationalVelocity());
    }
}
}
