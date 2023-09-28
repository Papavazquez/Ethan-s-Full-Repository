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
   ArrayList<PVector> waypoints;
   int counter;
   
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }

   void update(float dt)
   {
     if (target != null)
     {  
        // TODO: Implement seek here
        float slow_distance = 70;
        float arrival_distance = 10;
        float curr_velocity = 0.75;
        float curr_rot_speed = kinematic.getRotationalVelocity();
        float curr_dist;
        
        //float distToTarget = 
        float atanAngle = normalize_angle(atan2(kinematic.position.y - target.y, kinematic.position.x - target.x)); // Gets angle to target
        float angleToTarget = normalize_angle_left_right(-atanAngle + kinematic.getHeading()); // Determines whether to turn left or right
        curr_dist = PVector.sub(kinematic.position, target).mag();
        if (curr_dist < slow_distance)
        {
          if (waypoints != null) 
          {
            if (kinematic.getSpeed() > 25 && counter < waypoints.size()- 1) {curr_velocity = -0.35;}
            else if (kinematic.getSpeed() > 0 && counter >= waypoints.size() - 1) {curr_velocity = -0.35;}
            else if (kinematic.getSpeed() < 0) {curr_velocity = 0.35;}
          }
          else
          {
            if (kinematic.getSpeed() > 0) {curr_velocity = -0.35;}
            else if (kinematic.getSpeed() < 0) {curr_velocity = 0.35;}
          }
          //else { curr_velocity = 0.5; }
          if (curr_dist < arrival_distance)
          {
            counter += 1;
            if (waypoints != null) { 
              if (counter < waypoints.size()) {target = waypoints.get(counter);}
            }
          }
        }
        curr_rot_speed = 0.5;
        if (abs(angleToTarget) < 2) { curr_rot_speed = 2; curr_velocity = 0.01; } 
        if (angleToTarget <= 0) { curr_rot_speed = -0.5; }
        if (abs(angleToTarget) < 3.1 && abs(angleToTarget) > 2.9) 
        { 
          if (kinematic.getRotationalVelocity() < 0) { curr_rot_speed = 0.25;}
          else if (kinematic.getRotationalVelocity() > 0) { curr_rot_speed = -0.25;}
          //kinematic.setSpeed(kinematic.getSpeed(), 0); curr_rot_speed = 0; 
        }
        
        kinematic.increaseSpeed(curr_velocity, curr_rot_speed); //Generates actual movement
        //print(kinematic.getSpeed());
        //print(" ");
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
     for (Crumb c : this.crumbs)
     {
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
      //this.target = target;
      this.waypoints = nm.findPath(billy.kinematic.position, target);
      /*
      for (int i = 0; i < waypoints.size(); i++)
      {
        println(waypoints.get(i));
      }
      */
      this.target = this.waypoints.get(0);
   }
   
   void follow(ArrayList<PVector> waypoints)
   {
      // TODO: change to follow *all* waypoints
      this.counter = 0;
      this.waypoints = waypoints;
      this.target = waypoints.get(0);
   }
}
