var Vector = require('vector');
var utils  = require('./utils');

class SteeringBehaviors {
  constructor(vehicle, options) {
    this.vehicle = vehicle;

    // @TODO should be overridable with options
    // @TODO should be in a normalized form
    this.params = {
      minDetectionBoxLength: 40,
      wallDetectionFeelerLength: 40
    };
    // variables for wander behavior
    this.wanderRadius = 6;
    this.wanderDistance = 30;
    this.wanderJitter = 4;

    // antenna used in wall avoidance
    this.feelers = [];

    // initialize wander target
    // -> circle of wanderRadius centered on the agent
    var theta = utils.rand() * utils.TwoPI;
    // project theta on the wander circle
    this.wanderTarget = new Vector(this.wanderRadius * Math.cos(theta),
                                   this.wanderRadius * Math.sin(theta));

  }

  // this has too many dependecies with the world
  calculate() {
    if (!this.vehicle.world.isStarted) {
      return new Vector();
    }

    var tweakers = this.vehicle.world.steeringTweakers;
    var steerings = new Vector();

    if (this._seek && this.vehicle.world.target) {
      // these targets should be in the vehicle itself, not in world
      var steering = this.seek(this.vehicle.world.target)
      steering.multiply(tweakers.seek);
      steerings.add(steering);
    }

    if (this._flee && this.vehicle.world.target) {
      // these targets should be in the vehicle itself, not in world
      var steering = this.flee(this.vehicle.world.target)
      steering.multiply(tweakers.flee);
      steerings.add(steering);
    }

    if (this._arrive && this.vehicle.world.target) {
      // these targets should be in the vehicle itself, not in world
      var deceleration = 3;
      var steering = this.arrive(this.vehicle.world.target, deceleration);
      steering.multiply(tweakers.arrive);
      steerings.add(steering);
    }

    if (this._pursuit) {
      // these targets should be in the vehicle itself, not in world
      var steering = this.pursuit(this.vehicle.world.evader);
      steering.multiply(tweakers.pursuit);
      steerings.add(steering);
    }

    if (this. _evade) {
      // these targets should be in the vehicle itself, not in world
      var steering = this.evade(this.vehicle.world.pursuer);
      steering.multiply(tweakers.evade);
      steerings.add(steering);
    }

    if (this._wander) {
      var steering = this.wander();
      steering.multiply(tweakers.wander);
      steerings.add(steering);
    }

    // doesn't work...
    if (this._obstacleAvoidance) {
      var steering = this.obstacleAvoidance(this.vehicle.world.obstacles)
      steering.multiply(tweakers.obstacleAvoidance);
      steerings.add(steering);
    }

    // doesn't work well
    if (this._wallAvoidance) {
      var steering = this.wallAvoidance(this.vehicle.world.walls)
      steering.multiply(tweakers.wallAvoidance);
      steerings.add(steering);
    }

    if (this._flock) {
      // tag neighbors
      this.vehicle.world.tagBoidsWithinRange(this.vehicle);

      var flocks = new Vector();
      flocks.add(this.separation(this.vehicle.neighbors).multiply(tweakers.separation));
      flocks.add(this.alignment(this.vehicle.neighbors).multiply(tweakers.alignment));
      flocks.add(this.cohesion(this.vehicle.neighbors).multiply(tweakers.cohesion));

      steerings.add(flocks);
    }


    return steerings;
  }

  // behaviors On/Off
  // --------------------------------------------

  seekOn() { this._seek = true; }
  seekOff() { this._seek = false; }

  fleeOn() { this._flee = true; }
  fleeOff() { this._flee = false; }

  arriveOn() { this._arrive = true; }
  arriveOff() { this._arrive = false; }

  pursuitOn() { this._pursuit = true; }
  pursuitOff() { this._pursuit = false; }

  evadeOn() { this._evade = true; }
  evadeOff() { this._evade = false; }

  wanderOn() { this._wander = true; }
  wanderOff() { this._wander = false; }

  obstacleAvoidanceOn() { this._obstacleAvoidance = true; }
  obstacleAvoidanceOff() { this._obstacleAvoidance = false; }

  wallAvoidanceOn() { this._wallAvoidance = true; }
  wallAvoidanceOff() { this._wallAvoidance = false; }

  flockOn() { this._flock = true; }
  flockOff() { this._flock = false; }

  // behaviors
  // --------------------------------------------

  seek(targetPosition) {
    var desiredVelocity = Vector.substract(targetPosition, this.vehicle.position)
      .normalize()
      .multiply(this.vehicle.maxSpeed);

    var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
    return steering;
  }

  flee(targetPosition) {
    var panicDistance = 100 * 100; // use square domain to save computations
    if (Vector.distanceSqrt(this.vehicle.position, targetPosition) > panicDistance) {
      return new Vector();
    }

    var desiredVelocity = Vector.substract(this.vehicle.position, targetPosition)
      .normalize()
      .multiply(this.vehicle.maxSpeed);

    var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
    return steering;
  }

  // deceleration is an enum (slow = 1, normal = 2, fast = 3)
  // it describes the time the agent should take to arrive at destination
  arrive(targetPosition, deceleration) {
    var toTarget = Vector.substract(targetPosition, this.vehicle.position);
    var distance = toTarget.magnitude();

    if (distance > 0) {
      // allow to tweak deceleration
      var decelerationTweaker = 0.3;
      // define the speed the agent should have to arrive at destination
      var speed = distance / (deceleration * decelerationTweaker);
      // speed shouldn't exceed maxSpeed
      speed = Math.min(speed, this.vehicle.maxSpeed);
      // next steps are same as seek
      var desiredVelocity = toTarget
        .divide(distance) // <=> toTarget.normalize();
        .multiply(speed);

      var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
      return steering;
    }

    return new Vector();
  }

  pursuit(evader) {
    var toEvader = Vector.substract(evader.position, this.vehicle.position);
    // cosine of the angle between the 2 agents headings
    var relativeHeading = Vector.dot(evader.heading, this.vehicle.heading);

    if (
      // if the the evader is in front of the pursuer
      (Vector.dot(this.vehicle.heading, toEvader) > 0) &&
      // and the two agents are approx face to face
      (relativeHeading < -0.95) // acos(0.95) = 18deg
    ) {
      return this.seek(evader.position);
    }

    // not consirdered ahead so we have to
    // predict the future position of the evader

    // the lookAheadTime should be proportionnal to the distance,
    // and inversly proportionnal to the speed of the agents
    var lookAheadTime = toEvader.magnitude() / (this.vehicle.maxSpeed + evader.speed());
    // now seek to a prediction of the evader position
    var predictedPosition = Vector.add(evader.position, Vector.multiply(evader.velocity, lookAheadTime));
    return this.seek(predictedPosition);
  }

  evade(pursuer) {
    // no need to check if the agents are facing
    var toPursuer = Vector.substract(pursuer.position, this.vehicle.position);
    // then same as pursuit but replace seek with flee
    var lookAheadTime = toPursuer.magnitude() / (this.vehicle.maxSpeed + pursuer.speed());
    var predictedPosition = Vector.add(pursuer.position, Vector.multiply(pursuer.velocity, lookAheadTime));
    return this.flee(predictedPosition);
  }

  wander() {
    // add a small random vector to the wanderTarget
    var randomVector = new Vector(utils.randClamped() * this.wanderJitter,
                                  utils.randClamped() * this.wanderJitter);
    this.wanderTarget.add(randomVector);
    // reproject the wanderTarget on the wander circle
    this.wanderTarget
      .normalize()
      .multiply(this.wanderRadius);
    // project the wander circle in front of the agent
    var targetLocal = Vector.add(this.wanderTarget, new Vector(this.wanderDistance, 0));
    // @TODO should be an matrix transforms utils
    // project the target in world space - change name for understandability
    var targetWorld = targetLocal;
    // rotate
    targetWorld.rotate(this.vehicle.heading.direction());
    // translate
    targetWorld.add(this.vehicle.position);
    // steer toward this target
    return Vector.substract(targetWorld, this.vehicle.position);
  }

  // prevent the agent colliding with the closest obstacle
  // @NOTE - not tested properly
  obstacleAvoidance(obstacles) {
    // create a detection box proportionnal to the agent velocity
    var boxLength = this.params.minDetectionBoxLength +
                    this.vehicle.speed() / this.vehicle.maxSpeed *
                    this.params.minDetectionBoxLength;

    this.vehicle.world.tagObstaclesWithinRange(this.vehicle, boxLength);
    // closest intersecting obstacle (CIO)
    var closestInterceptingObstacle = null;
    var distanceToCIO = +Infinity;
    var localPositionOfCIO = null;

    obstacles.forEach(function(obstacle) {
      if (!obstacle.isTagged()) { return; }
      // find local coordinates of the obstacle
      var localPos = obstacle.position.clone();
      // rotate
      localPos.rotate(this.vehicle.heading.direction() * -1);
      // translate
      localPos.substract(this.vehicle.position);
      // if the local x value is negative, the obstacle is behind the agent
      if (localPos.x < 0) { return; }

      // if the distance between the x axis to the oject local position is less than
      // its radius + the radius of this.vehicle, there is a possible collision
      var expandedRadius = obstacle.boundingRadius + this.vehicle.boundingRadius;

      if (localPos.y > expandedRadius) { return; }

      // now to do a line/circle intersection test. The center of the
      // circle is represented by (cX, cY). The intersection points are
      // given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0.
      // We only need to look at the smallest positive value of x because
      // that will be the closest point of intersection.
      var cX = localPos.x;
      var cY = localPos.y;

      // calcule the sqrt part of the equation only once
      var sqrtPart = Math.sqrt(expandedRadius * expandedRadius - cY * cY);

      var ip = cX - sqrtPart;
      if (ip < 0) { ip = cX + sqrtPart; }

      // if closest so far - store all its value
      if (ip < distanceToCIO) {
        distanceToCIO = ip;
        closestInterceptingObstacle = obstacle;
        localPositionOfCIO = localPos;
      }
    }, this);

    // still in local space
    var steering = new Vector();
    // if we found some obtacle, calculate a steering force away from it
    if (closestInterceptingObstacle) {
      // the closer an agent is to an object, the stronger the steering (between 1 and 2 [?])
      var multiplier = 1 + (boxLength - localPositionOfCIO.x) / boxLength;

      steering.y = (closestInterceptingObstacle.boundingRadius -
                    localPositionOfCIO.y) * multiplier;

      var brakingWeight = 0.2;

      steering.x = (closestInterceptingObstacle.boundingRadius -
                    localPositionOfCIO.x) * brakingWeight;
    }

    // rotate to go back to world space
    steering.rotate(this.vehicle.heading.direction());
    return steering;
  }

  wallAvoidance(walls) {
    this.createFeelers();

    // IP: Intersection Point
    var distanceToClosestIP = +Infinity;
    var closestPoint = null;
    var closestWall = null;

    var steering = new Vector();
    var distance, point;

    // geometry.h (line 284) LineIntersection2d (??? what appends here ???)
    var intersect = function(f1, t1, f2, t2) {
      var rTop = (f1.y-f2.y)*(t2.x-f2.x)-(f1.x-f2.x)*(t2.y-f2.y);
      var rBot = (t1.x-f1.x)*(t2.y-f2.y)-(t1.y-f1.y)*(t2.x-f2.x);

      var sTop = (f1.y-f2.y)*(t1.x-f1.x)-(f1.x-f2.x)*(t1.y-f1.y);
      var sBot = (t1.x-f1.x)*(t2.y-f2.y)-(t1.y-f1.y)*(t2.x-f2.x);

      if ( (rBot === 0) || (sBot === 0) ) {
        return false;
      }

      var r = rTop/rBot;
      var s = sTop/sBot;

      if ( (r > 0) && (r < 1) && (s > 0) && (s < 1) ) {
        distance = Vector.distance(f1, t1) * r;
        // console.log(r, distance);
        point = new Vector(f1.x + r, f1.y + r);
        // change from c++ code here, too much force something is wrong (...probably me)
        // point.multiply(Vector.substract(t1, f1));
        point.multiply(1.08);

        return true;
      } else {
        distance = 0;

        return false;
      }
    }

    // examine each feeler onn each wall
    this.feelers.forEach(function(feeler) {
      walls.forEach(function(wall) {
        if (intersect(this.vehicle.position, feeler, wall.from, wall.to)) {
          if (distance < distanceToClosestIP) {
            distanceToClosestIP = distance;
            closestWall = wall;
            closestPoint = point;
          }
        }
      }, this);


      if (closestWall) {
        var overShoot = Vector.substract(feeler, closestPoint);
        // console.log(overShoot.magnitude())
        steering = Vector.multiply(closestWall.normal, overShoot.magnitude());
      }
    }, this);

    return steering;
  }

  // create antenna used in wallAvoidance - in world coordinates
  createFeelers() {
    // center
    var tmp = Vector.multiply(this.vehicle.heading, this.params.wallDetectionFeelerLength);
    this.feelers[0] = Vector.add(this.vehicle.position, tmp);
    // left
    var tmp = this.vehicle.heading.clone();
    tmp.rotate(Math.PI / 2 * 3.5);
    tmp = Vector.multiply(tmp, this.params.wallDetectionFeelerLength / 2);
    this.feelers[1] = Vector.add(this.vehicle.position, tmp);
    // right
    var tmp = this.vehicle.heading.clone();
    tmp.rotate(Math.PI / 2 * 0.5);
    tmp = Vector.multiply(tmp, this.params.wallDetectionFeelerLength / 2);
    this.feelers[2] = Vector.add(this.vehicle.position, tmp);
  }

  // FLOCKS
  // -------------------------------------------------------------

  separation(neighbors) {
    var steering = new Vector();

    neighbors.forEach(function(neighbor, index) {
      var toAgent = Vector.substract(this.vehicle.position, neighbor.position);
      // scale the force inversely proportionnal to the agent distance from its neighbor
      var distance = toAgent.magnitude();
      toAgent.normalize().divide(distance); // .multiply(100);

      steering.add(toAgent);
    }, this);

    // if (this.vehicle.isTest) { console.log(steering); }
    return steering;
  }

  alignment(neighbors) {
    var averageHeading = new Vector();
    var neighborCount = neighbors.length;

    neighbors.forEach(function(neighbor) {
      averageHeading.add(neighbor.heading);
    });

    if (neighborCount > 0) {
      averageHeading.divide(neighborCount);
      averageHeading.substract(this.vehicle.heading);
    }

    return averageHeading;
  }

  cohesion(neighbors) {
    var centerOfMass = new Vector();
    var steering = new Vector();
    var neighborCount = neighbors.length;

    neighbors.forEach(function(neighbor) {
      centerOfMass.add(neighbor.position);
    });

    if (neighborCount) {
      centerOfMass.divide(neighborCount);
      steering = this.seek(centerOfMass);
    }

    return steering;
  }

  // -------------------------------------------------------------
  // DEBUG VISUALIZATION
  // -------------------------------------------------------------

  debugFeelers(ctx) {
    this.feelers.forEach(function(feeler) {
      ctx.save();
      ctx.beginPath();

      ctx.strokeStyle = '#acacac';
      ctx.moveTo(this.vehicle.position.x, this.vehicle.position.y);
      ctx.lineTo(feeler.x, feeler.y);
      ctx.stroke();

      ctx.closePath();
      ctx.restore();
    }, this);
  }

  debugWander(ctx) {
    ctx.save();
    // move to wander circle center
    ctx.translate(this.wanderDistance, 0);

    ctx.strokeStyle = 'red';
    ctx.fillStyle = 'red';
    // wander circle
    ctx.beginPath();
    ctx.arc(0, 0, this.wanderRadius, 0, utils.TwoPI, false);
    ctx.stroke();
    ctx.closePath();
    // wander target
    ctx.beginPath();
    ctx.arc(this.wanderTarget.x, this.wanderTarget.y, 2, 0, utils.TwoPI, false);
    ctx.stroke();
    ctx.closePath();

    ctx.restore();
  }
}

module.exports = SteeringBehaviors;
