/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

class Crumb
{
  PVector position;
  Crumb(PVector position)
  {
     this.position = position;
  }
  void draw()
  {
     fill(255);
     noStroke(); 
     circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
   Crumb[] crumbs = {};
   int last_crumb;
   float acceleration;
   float rotational_acceleration;
   KinematicMovement kinematic;
   PVector target;
   
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }

   void update(float dt, ArrayList<Boid> flock)
   {
     if (target != null)
     {  
        // TODO: Implement seek here
        //calculate heading
        float desired_heading = atan2(target.y - kinematic.getPosition().y, target.x - kinematic.getPosition().x);
        float current_heading = kinematic.getHeading();
        
        float angle_difference = desired_heading - current_heading;
        angle_difference = normalize_angle_left_right(angle_difference);
        
        //rotate towards the target if outside threshhold
        if (abs(angle_difference) > 0.1) {
          float rotational_direction = (angle_difference > 0 ? 1 : -1) * rotational_acceleration *dt;
          // Scale rotational speed down as it gets closer to the target heading
          rotational_direction *= map(abs(angle_difference), 0, PI, 1.0, 3.0); // Adjust rotation speed dynamically
          kinematic.increaseSpeed(0, rotational_direction);
        }
        else {
          kinematic.increaseSpeed(0, -kinematic.getRotationalVelocity() * 0.5); // Gradually reduce rotation
        }
        
        float distance_to_target = PVector.dist(kinematic.getPosition(), target);
        // If the Boid is close to the target, decelerate
        if (distance_to_target < 80)
        {
            // Calculate deceleration factor
            float deceleration_factor = distance_to_target / 80;
            float decelerated_speed = BILLY_MAX_SPEED * deceleration_factor;

            // Apply deceleration
            kinematic.increaseSpeed(decelerated_speed - kinematic.getSpeed(), 0);
            
            //Move to the next waypoint if within the threshold
            if (distance_to_target < 16) {
              follow(waypoints);
            }
        }
        else
        {
            // Maintain maximum speed if far from the target
            if (abs(angle_difference) < 0.1) {
                kinematic.increaseSpeed(acceleration, 0);
            }
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
   
   void draw()
   {
     for (Crumb c : this.crumbs){
       c.draw();
     }
     
     fill(255);
     noStroke(); 
     float x = kinematic.position.x;
     float y = kinematic.position.y;
     float r = kinematic.heading;
     circle(x, y, BOID_SIZE);
     // front
     float xp = x + BOID_SIZE*cos(r);
     float yp = y + BOID_SIZE*sin(r);
     
     // left
     float x1p = x - (BOID_SIZE/2)*sin(r);
     float y1p = y + (BOID_SIZE/2)*cos(r);
     
     // right
     float x2p = x + (BOID_SIZE/2)*sin(r);
     float y2p = y - (BOID_SIZE/2)*cos(r);
     triangle(xp, yp, x1p, y1p, x2p, y2p);
   }
   
   void seek(PVector target)
   {
      this.target = target;
      
   }
   
   void follow(ArrayList<PVector> waypoints) {
     if (waypoints.size() > 0) {
       target = waypoints.get(0);  // Set the first waypoint as the target
       waypoints.remove(0);        // Remove the waypoint once reached
     }
     else
     {
       target = null;  // No more waypoints to follow
       kinematic.increaseSpeed(-kinematic.getSpeed(), -kinematic.getRotationalVelocity());  // Stop movement
     }
   }
}
