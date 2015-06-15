'use strict';

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var Vector = require('vector');
var utils = require('./utils');

var SteeringBehaviors = (function () {
  function SteeringBehaviors(vehicle, options) {
    _classCallCheck(this, SteeringBehaviors);

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
    this.wanderTarget = new Vector(this.wanderRadius * Math.cos(theta), this.wanderRadius * Math.sin(theta));
  }

  _createClass(SteeringBehaviors, [{
    key: 'calculate',

    // this has too many dependecies with the world
    value: function calculate() {
      if (!this.vehicle.world.isStarted) {
        return new Vector();
      }

      var tweakers = this.vehicle.world.steeringTweakers;
      var steerings = new Vector();

      if (this._seek && this.vehicle.world.target) {
        // these targets should be in the vehicle itself, not in world
        var steering = this.seek(this.vehicle.world.target);
        steering.multiply(tweakers.seek);
        steerings.add(steering);
      }

      if (this._flee && this.vehicle.world.target) {
        // these targets should be in the vehicle itself, not in world
        var steering = this.flee(this.vehicle.world.target);
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

      if (this._evade) {
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
        var steering = this.obstacleAvoidance(this.vehicle.world.obstacles);
        steering.multiply(tweakers.obstacleAvoidance);
        steerings.add(steering);
      }

      // doesn't work well
      if (this._wallAvoidance) {
        var steering = this.wallAvoidance(this.vehicle.world.walls);
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
  }, {
    key: 'seekOn',

    // behaviors On/Off
    // --------------------------------------------

    value: function seekOn() {
      this._seek = true;
    }
  }, {
    key: 'seekOff',
    value: function seekOff() {
      this._seek = false;
    }
  }, {
    key: 'fleeOn',
    value: function fleeOn() {
      this._flee = true;
    }
  }, {
    key: 'fleeOff',
    value: function fleeOff() {
      this._flee = false;
    }
  }, {
    key: 'arriveOn',
    value: function arriveOn() {
      this._arrive = true;
    }
  }, {
    key: 'arriveOff',
    value: function arriveOff() {
      this._arrive = false;
    }
  }, {
    key: 'pursuitOn',
    value: function pursuitOn() {
      this._pursuit = true;
    }
  }, {
    key: 'pursuitOff',
    value: function pursuitOff() {
      this._pursuit = false;
    }
  }, {
    key: 'evadeOn',
    value: function evadeOn() {
      this._evade = true;
    }
  }, {
    key: 'evadeOff',
    value: function evadeOff() {
      this._evade = false;
    }
  }, {
    key: 'wanderOn',
    value: function wanderOn() {
      this._wander = true;
    }
  }, {
    key: 'wanderOff',
    value: function wanderOff() {
      this._wander = false;
    }
  }, {
    key: 'obstacleAvoidanceOn',
    value: function obstacleAvoidanceOn() {
      this._obstacleAvoidance = true;
    }
  }, {
    key: 'obstacleAvoidanceOff',
    value: function obstacleAvoidanceOff() {
      this._obstacleAvoidance = false;
    }
  }, {
    key: 'wallAvoidanceOn',
    value: function wallAvoidanceOn() {
      this._wallAvoidance = true;
    }
  }, {
    key: 'wallAvoidanceOff',
    value: function wallAvoidanceOff() {
      this._wallAvoidance = false;
    }
  }, {
    key: 'flockOn',
    value: function flockOn() {
      this._flock = true;
    }
  }, {
    key: 'flockOff',
    value: function flockOff() {
      this._flock = false;
    }
  }, {
    key: 'seek',

    // behaviors
    // --------------------------------------------

    value: function seek(targetPosition) {
      var desiredVelocity = Vector.substract(targetPosition, this.vehicle.position).normalize().multiply(this.vehicle.maxSpeed);

      var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
      return steering;
    }
  }, {
    key: 'flee',
    value: function flee(targetPosition) {
      var panicDistance = 100 * 100; // use square domain to save computations
      if (Vector.distanceSqrt(this.vehicle.position, targetPosition) > panicDistance) {
        return new Vector();
      }

      var desiredVelocity = Vector.substract(this.vehicle.position, targetPosition).normalize().multiply(this.vehicle.maxSpeed);

      var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
      return steering;
    }
  }, {
    key: 'arrive',

    // deceleration is an enum (slow = 1, normal = 2, fast = 3)
    // it describes the time the agent should take to arrive at destination
    value: function arrive(targetPosition, deceleration) {
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
        var desiredVelocity = toTarget.divide(distance) // <=> toTarget.normalize();
        .multiply(speed);

        var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
        return steering;
      }

      return new Vector();
    }
  }, {
    key: 'pursuit',
    value: function pursuit(evader) {
      var toEvader = Vector.substract(evader.position, this.vehicle.position);
      // cosine of the angle between the 2 agents headings
      var relativeHeading = Vector.dot(evader.heading, this.vehicle.heading);

      if (
      // if the the evader is in front of the pursuer
      Vector.dot(this.vehicle.heading, toEvader) > 0 && relativeHeading < -0.95 // acos(0.95) = 18deg
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
  }, {
    key: 'evade',
    value: function evade(pursuer) {
      // no need to check if the agents are facing
      var toPursuer = Vector.substract(pursuer.position, this.vehicle.position);
      // then same as pursuit but replace seek with flee
      var lookAheadTime = toPursuer.magnitude() / (this.vehicle.maxSpeed + pursuer.speed());
      var predictedPosition = Vector.add(pursuer.position, Vector.multiply(pursuer.velocity, lookAheadTime));
      return this.flee(predictedPosition);
    }
  }, {
    key: 'wander',
    value: function wander() {
      // add a small random vector to the wanderTarget
      var randomVector = new Vector(utils.randClamped() * this.wanderJitter, utils.randClamped() * this.wanderJitter);
      this.wanderTarget.add(randomVector);
      // reproject the wanderTarget on the wander circle
      this.wanderTarget.normalize().multiply(this.wanderRadius);
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
  }, {
    key: 'obstacleAvoidance',

    // prevent the agent colliding with the closest obstacle
    // @NOTE - not tested properly
    value: function obstacleAvoidance(obstacles) {
      // create a detection box proportionnal to the agent velocity
      var boxLength = this.params.minDetectionBoxLength + this.vehicle.speed() / this.vehicle.maxSpeed * this.params.minDetectionBoxLength;

      this.vehicle.world.tagObstaclesWithinRange(this.vehicle, boxLength);
      // closest intersecting obstacle (CIO)
      var closestInterceptingObstacle = null;
      var distanceToCIO = +Infinity;
      var localPositionOfCIO = null;

      obstacles.forEach(function (obstacle) {
        if (!obstacle.isTagged()) {
          return;
        }
        // find local coordinates of the obstacle
        var localPos = obstacle.position.clone();
        // rotate
        localPos.rotate(this.vehicle.heading.direction() * -1);
        // translate
        localPos.substract(this.vehicle.position);
        // if the local x value is negative, the obstacle is behind the agent
        if (localPos.x < 0) {
          return;
        }

        // if the distance between the x axis to the oject local position is less than
        // its radius + the radius of this.vehicle, there is a possible collision
        var expandedRadius = obstacle.boundingRadius + this.vehicle.boundingRadius;

        if (localPos.y > expandedRadius) {
          return;
        }

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
        if (ip < 0) {
          ip = cX + sqrtPart;
        }

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

        steering.y = (closestInterceptingObstacle.boundingRadius - localPositionOfCIO.y) * multiplier;

        var brakingWeight = 0.2;

        steering.x = (closestInterceptingObstacle.boundingRadius - localPositionOfCIO.x) * brakingWeight;
      }

      // rotate to go back to world space
      steering.rotate(this.vehicle.heading.direction());
      return steering;
    }
  }, {
    key: 'wallAvoidance',
    value: function wallAvoidance(walls) {
      this.createFeelers();

      // IP: Intersection Point
      var distanceToClosestIP = +Infinity;
      var closestPoint = null;
      var closestWall = null;

      var steering = new Vector();
      var distance, point;

      // geometry.h (line 284) LineIntersection2d (??? what appends here ???)
      var intersect = function intersect(f1, t1, f2, t2) {
        var rTop = (f1.y - f2.y) * (t2.x - f2.x) - (f1.x - f2.x) * (t2.y - f2.y);
        var rBot = (t1.x - f1.x) * (t2.y - f2.y) - (t1.y - f1.y) * (t2.x - f2.x);

        var sTop = (f1.y - f2.y) * (t1.x - f1.x) - (f1.x - f2.x) * (t1.y - f1.y);
        var sBot = (t1.x - f1.x) * (t2.y - f2.y) - (t1.y - f1.y) * (t2.x - f2.x);

        if (rBot === 0 || sBot === 0) {
          return false;
        }

        var r = rTop / rBot;
        var s = sTop / sBot;

        if (r > 0 && r < 1 && s > 0 && s < 1) {
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
      };

      // examine each feeler onn each wall
      this.feelers.forEach(function (feeler) {
        walls.forEach(function (wall) {
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
  }, {
    key: 'createFeelers',

    // create antenna used in wallAvoidance - in world coordinates
    value: function createFeelers() {
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
  }, {
    key: 'separation',

    // FLOCKS
    // -------------------------------------------------------------

    value: function separation(neighbors) {
      var steering = new Vector();

      neighbors.forEach(function (neighbor, index) {
        var toAgent = Vector.substract(this.vehicle.position, neighbor.position);
        // scale the force inversely proportionnal to the agent distance from its neighbor
        var distance = toAgent.magnitude();
        toAgent.normalize().divide(distance); // .multiply(100);

        steering.add(toAgent);
      }, this);

      // if (this.vehicle.isTest) { console.log(steering); }
      return steering;
    }
  }, {
    key: 'alignment',
    value: function alignment(neighbors) {
      var averageHeading = new Vector();
      var neighborCount = neighbors.length;

      neighbors.forEach(function (neighbor) {
        averageHeading.add(neighbor.heading);
      });

      if (neighborCount > 0) {
        averageHeading.divide(neighborCount);
        averageHeading.substract(this.vehicle.heading);
      }

      return averageHeading;
    }
  }, {
    key: 'cohesion',
    value: function cohesion(neighbors) {
      var centerOfMass = new Vector();
      var steering = new Vector();
      var neighborCount = neighbors.length;

      neighbors.forEach(function (neighbor) {
        centerOfMass.add(neighbor.position);
      });

      if (neighborCount) {
        centerOfMass.divide(neighborCount);
        steering = this.seek(centerOfMass);
      }

      return steering;
    }
  }, {
    key: 'debugFeelers',

    // -------------------------------------------------------------
    // DEBUG VISUALIZATION
    // -------------------------------------------------------------

    value: function debugFeelers(ctx) {
      this.feelers.forEach(function (feeler) {
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
  }, {
    key: 'debugWander',
    value: function debugWander(ctx) {
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
  }]);

  return SteeringBehaviors;
})();

module.exports = SteeringBehaviors;

// and the two agents are approx face to face
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNyYy9zdGVlcmluZy1iZWhhdmlvcnMuZXM2LmpzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7Ozs7OztBQUFBLElBQUksTUFBTSxHQUFHLE9BQU8sQ0FBQyxRQUFRLENBQUMsQ0FBQztBQUMvQixJQUFJLEtBQUssR0FBSSxPQUFPLENBQUMsU0FBUyxDQUFDLENBQUM7O0lBRTFCLGlCQUFpQjtBQUNWLFdBRFAsaUJBQWlCLENBQ1QsT0FBTyxFQUFFLE9BQU8sRUFBRTswQkFEMUIsaUJBQWlCOztBQUVuQixRQUFJLENBQUMsT0FBTyxHQUFHLE9BQU8sQ0FBQzs7OztBQUl2QixRQUFJLENBQUMsTUFBTSxHQUFHO0FBQ1osMkJBQXFCLEVBQUUsRUFBRTtBQUN6QiwrQkFBeUIsRUFBRSxFQUFFO0tBQzlCLENBQUM7O0FBRUYsUUFBSSxDQUFDLFlBQVksR0FBRyxDQUFDLENBQUM7QUFDdEIsUUFBSSxDQUFDLGNBQWMsR0FBRyxFQUFFLENBQUM7QUFDekIsUUFBSSxDQUFDLFlBQVksR0FBRyxDQUFDLENBQUM7OztBQUd0QixRQUFJLENBQUMsT0FBTyxHQUFHLEVBQUUsQ0FBQzs7OztBQUlsQixRQUFJLEtBQUssR0FBRyxLQUFLLENBQUMsSUFBSSxFQUFFLEdBQUcsS0FBSyxDQUFDLEtBQUssQ0FBQzs7QUFFdkMsUUFBSSxDQUFDLFlBQVksR0FBRyxJQUFJLE1BQU0sQ0FBQyxJQUFJLENBQUMsWUFBWSxHQUFHLElBQUksQ0FBQyxHQUFHLENBQUMsS0FBSyxDQUFDLEVBQ25DLElBQUksQ0FBQyxZQUFZLEdBQUcsSUFBSSxDQUFDLEdBQUcsQ0FBQyxLQUFLLENBQUMsQ0FBQyxDQUFDO0dBRXJFOztlQXpCRyxpQkFBaUI7Ozs7V0E0QloscUJBQUc7QUFDVixVQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxLQUFLLENBQUMsU0FBUyxFQUFFO0FBQ2pDLGVBQU8sSUFBSSxNQUFNLEVBQUUsQ0FBQztPQUNyQjs7QUFFRCxVQUFJLFFBQVEsR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDLEtBQUssQ0FBQyxnQkFBZ0IsQ0FBQztBQUNuRCxVQUFJLFNBQVMsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDOztBQUU3QixVQUFJLElBQUksQ0FBQyxLQUFLLElBQUksSUFBSSxDQUFDLE9BQU8sQ0FBQyxLQUFLLENBQUMsTUFBTSxFQUFFOztBQUUzQyxZQUFJLFFBQVEsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsS0FBSyxDQUFDLE1BQU0sQ0FBQyxDQUFBO0FBQ25ELGdCQUFRLENBQUMsUUFBUSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztBQUNqQyxpQkFBUyxDQUFDLEdBQUcsQ0FBQyxRQUFRLENBQUMsQ0FBQztPQUN6Qjs7QUFFRCxVQUFJLElBQUksQ0FBQyxLQUFLLElBQUksSUFBSSxDQUFDLE9BQU8sQ0FBQyxLQUFLLENBQUMsTUFBTSxFQUFFOztBQUUzQyxZQUFJLFFBQVEsR0FBRyxJQUFJLENBQUMsSUFBSSxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsS0FBSyxDQUFDLE1BQU0sQ0FBQyxDQUFBO0FBQ25ELGdCQUFRLENBQUMsUUFBUSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQztBQUNqQyxpQkFBUyxDQUFDLEdBQUcsQ0FBQyxRQUFRLENBQUMsQ0FBQztPQUN6Qjs7QUFFRCxVQUFJLElBQUksQ0FBQyxPQUFPLElBQUksSUFBSSxDQUFDLE9BQU8sQ0FBQyxLQUFLLENBQUMsTUFBTSxFQUFFOztBQUU3QyxZQUFJLFlBQVksR0FBRyxDQUFDLENBQUM7QUFDckIsWUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLE1BQU0sQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEtBQUssQ0FBQyxNQUFNLEVBQUUsWUFBWSxDQUFDLENBQUM7QUFDcEUsZ0JBQVEsQ0FBQyxRQUFRLENBQUMsUUFBUSxDQUFDLE1BQU0sQ0FBQyxDQUFDO0FBQ25DLGlCQUFTLENBQUMsR0FBRyxDQUFDLFFBQVEsQ0FBQyxDQUFDO09BQ3pCOztBQUVELFVBQUksSUFBSSxDQUFDLFFBQVEsRUFBRTs7QUFFakIsWUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLE9BQU8sQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEtBQUssQ0FBQyxNQUFNLENBQUMsQ0FBQztBQUN2RCxnQkFBUSxDQUFDLFFBQVEsQ0FBQyxRQUFRLENBQUMsT0FBTyxDQUFDLENBQUM7QUFDcEMsaUJBQVMsQ0FBQyxHQUFHLENBQUMsUUFBUSxDQUFDLENBQUM7T0FDekI7O0FBRUQsVUFBSSxJQUFJLENBQUUsTUFBTSxFQUFFOztBQUVoQixZQUFJLFFBQVEsR0FBRyxJQUFJLENBQUMsS0FBSyxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsS0FBSyxDQUFDLE9BQU8sQ0FBQyxDQUFDO0FBQ3RELGdCQUFRLENBQUMsUUFBUSxDQUFDLFFBQVEsQ0FBQyxLQUFLLENBQUMsQ0FBQztBQUNsQyxpQkFBUyxDQUFDLEdBQUcsQ0FBQyxRQUFRLENBQUMsQ0FBQztPQUN6Qjs7QUFFRCxVQUFJLElBQUksQ0FBQyxPQUFPLEVBQUU7QUFDaEIsWUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLE1BQU0sRUFBRSxDQUFDO0FBQzdCLGdCQUFRLENBQUMsUUFBUSxDQUFDLFFBQVEsQ0FBQyxNQUFNLENBQUMsQ0FBQztBQUNuQyxpQkFBUyxDQUFDLEdBQUcsQ0FBQyxRQUFRLENBQUMsQ0FBQztPQUN6Qjs7O0FBR0QsVUFBSSxJQUFJLENBQUMsa0JBQWtCLEVBQUU7QUFDM0IsWUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLGlCQUFpQixDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsS0FBSyxDQUFDLFNBQVMsQ0FBQyxDQUFBO0FBQ25FLGdCQUFRLENBQUMsUUFBUSxDQUFDLFFBQVEsQ0FBQyxpQkFBaUIsQ0FBQyxDQUFDO0FBQzlDLGlCQUFTLENBQUMsR0FBRyxDQUFDLFFBQVEsQ0FBQyxDQUFDO09BQ3pCOzs7QUFHRCxVQUFJLElBQUksQ0FBQyxjQUFjLEVBQUU7QUFDdkIsWUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLGFBQWEsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEtBQUssQ0FBQyxLQUFLLENBQUMsQ0FBQTtBQUMzRCxnQkFBUSxDQUFDLFFBQVEsQ0FBQyxRQUFRLENBQUMsYUFBYSxDQUFDLENBQUM7QUFDMUMsaUJBQVMsQ0FBQyxHQUFHLENBQUMsUUFBUSxDQUFDLENBQUM7T0FDekI7O0FBRUQsVUFBSSxJQUFJLENBQUMsTUFBTSxFQUFFOztBQUVmLFlBQUksQ0FBQyxPQUFPLENBQUMsS0FBSyxDQUFDLG1CQUFtQixDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsQ0FBQzs7QUFFckQsWUFBSSxNQUFNLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUMxQixjQUFNLENBQUMsR0FBRyxDQUFDLElBQUksQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxTQUFTLENBQUMsQ0FBQyxRQUFRLENBQUMsUUFBUSxDQUFDLFVBQVUsQ0FBQyxDQUFDLENBQUM7QUFDbEYsY0FBTSxDQUFDLEdBQUcsQ0FBQyxJQUFJLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsU0FBUyxDQUFDLENBQUMsUUFBUSxDQUFDLFFBQVEsQ0FBQyxTQUFTLENBQUMsQ0FBQyxDQUFDO0FBQ2hGLGNBQU0sQ0FBQyxHQUFHLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLFNBQVMsQ0FBQyxDQUFDLFFBQVEsQ0FBQyxRQUFRLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQzs7QUFFOUUsaUJBQVMsQ0FBQyxHQUFHLENBQUMsTUFBTSxDQUFDLENBQUM7T0FDdkI7O0FBR0QsYUFBTyxTQUFTLENBQUM7S0FDbEI7Ozs7Ozs7V0FLSyxrQkFBRztBQUFFLFVBQUksQ0FBQyxLQUFLLEdBQUcsSUFBSSxDQUFDO0tBQUU7OztXQUN4QixtQkFBRztBQUFFLFVBQUksQ0FBQyxLQUFLLEdBQUcsS0FBSyxDQUFDO0tBQUU7OztXQUUzQixrQkFBRztBQUFFLFVBQUksQ0FBQyxLQUFLLEdBQUcsSUFBSSxDQUFDO0tBQUU7OztXQUN4QixtQkFBRztBQUFFLFVBQUksQ0FBQyxLQUFLLEdBQUcsS0FBSyxDQUFDO0tBQUU7OztXQUV6QixvQkFBRztBQUFFLFVBQUksQ0FBQyxPQUFPLEdBQUcsSUFBSSxDQUFDO0tBQUU7OztXQUMxQixxQkFBRztBQUFFLFVBQUksQ0FBQyxPQUFPLEdBQUcsS0FBSyxDQUFDO0tBQUU7OztXQUU1QixxQkFBRztBQUFFLFVBQUksQ0FBQyxRQUFRLEdBQUcsSUFBSSxDQUFDO0tBQUU7OztXQUMzQixzQkFBRztBQUFFLFVBQUksQ0FBQyxRQUFRLEdBQUcsS0FBSyxDQUFDO0tBQUU7OztXQUVoQyxtQkFBRztBQUFFLFVBQUksQ0FBQyxNQUFNLEdBQUcsSUFBSSxDQUFDO0tBQUU7OztXQUN6QixvQkFBRztBQUFFLFVBQUksQ0FBQyxNQUFNLEdBQUcsS0FBSyxDQUFDO0tBQUU7OztXQUUzQixvQkFBRztBQUFFLFVBQUksQ0FBQyxPQUFPLEdBQUcsSUFBSSxDQUFDO0tBQUU7OztXQUMxQixxQkFBRztBQUFFLFVBQUksQ0FBQyxPQUFPLEdBQUcsS0FBSyxDQUFDO0tBQUU7OztXQUVsQiwrQkFBRztBQUFFLFVBQUksQ0FBQyxrQkFBa0IsR0FBRyxJQUFJLENBQUM7S0FBRTs7O1dBQ3JDLGdDQUFHO0FBQUUsVUFBSSxDQUFDLGtCQUFrQixHQUFHLEtBQUssQ0FBQztLQUFFOzs7V0FFNUMsMkJBQUc7QUFBRSxVQUFJLENBQUMsY0FBYyxHQUFHLElBQUksQ0FBQztLQUFFOzs7V0FDakMsNEJBQUc7QUFBRSxVQUFJLENBQUMsY0FBYyxHQUFHLEtBQUssQ0FBQztLQUFFOzs7V0FFNUMsbUJBQUc7QUFBRSxVQUFJLENBQUMsTUFBTSxHQUFHLElBQUksQ0FBQztLQUFFOzs7V0FDekIsb0JBQUc7QUFBRSxVQUFJLENBQUMsTUFBTSxHQUFHLEtBQUssQ0FBQztLQUFFOzs7Ozs7O1dBSy9CLGNBQUMsY0FBYyxFQUFFO0FBQ25CLFVBQUksZUFBZSxHQUFHLE1BQU0sQ0FBQyxTQUFTLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLENBQzFFLFNBQVMsRUFBRSxDQUNYLFFBQVEsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDOztBQUVuQyxVQUFJLFFBQVEsR0FBRyxNQUFNLENBQUMsU0FBUyxDQUFDLGVBQWUsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDO0FBQ3hFLGFBQU8sUUFBUSxDQUFDO0tBQ2pCOzs7V0FFRyxjQUFDLGNBQWMsRUFBRTtBQUNuQixVQUFJLGFBQWEsR0FBRyxHQUFHLEdBQUcsR0FBRyxDQUFDO0FBQzlCLFVBQUksTUFBTSxDQUFDLFlBQVksQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsRUFBRSxjQUFjLENBQUMsR0FBRyxhQUFhLEVBQUU7QUFDOUUsZUFBTyxJQUFJLE1BQU0sRUFBRSxDQUFDO09BQ3JCOztBQUVELFVBQUksZUFBZSxHQUFHLE1BQU0sQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxRQUFRLEVBQUUsY0FBYyxDQUFDLENBQzFFLFNBQVMsRUFBRSxDQUNYLFFBQVEsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDOztBQUVuQyxVQUFJLFFBQVEsR0FBRyxNQUFNLENBQUMsU0FBUyxDQUFDLGVBQWUsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDO0FBQ3hFLGFBQU8sUUFBUSxDQUFDO0tBQ2pCOzs7Ozs7V0FJSyxnQkFBQyxjQUFjLEVBQUUsWUFBWSxFQUFFO0FBQ25DLFVBQUksUUFBUSxHQUFHLE1BQU0sQ0FBQyxTQUFTLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLENBQUM7QUFDdkUsVUFBSSxRQUFRLEdBQUcsUUFBUSxDQUFDLFNBQVMsRUFBRSxDQUFDOztBQUVwQyxVQUFJLFFBQVEsR0FBRyxDQUFDLEVBQUU7O0FBRWhCLFlBQUksbUJBQW1CLEdBQUcsR0FBRyxDQUFDOztBQUU5QixZQUFJLEtBQUssR0FBRyxRQUFRLElBQUksWUFBWSxHQUFHLG1CQUFtQixDQUFBLEFBQUMsQ0FBQzs7QUFFNUQsYUFBSyxHQUFHLElBQUksQ0FBQyxHQUFHLENBQUMsS0FBSyxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLENBQUM7O0FBRS9DLFlBQUksZUFBZSxHQUFHLFFBQVEsQ0FDM0IsTUFBTSxDQUFDLFFBQVEsQ0FBQztTQUNoQixRQUFRLENBQUMsS0FBSyxDQUFDLENBQUM7O0FBRW5CLFlBQUksUUFBUSxHQUFHLE1BQU0sQ0FBQyxTQUFTLENBQUMsZUFBZSxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxDQUFDLENBQUM7QUFDeEUsZUFBTyxRQUFRLENBQUM7T0FDakI7O0FBRUQsYUFBTyxJQUFJLE1BQU0sRUFBRSxDQUFDO0tBQ3JCOzs7V0FFTSxpQkFBQyxNQUFNLEVBQUU7QUFDZCxVQUFJLFFBQVEsR0FBRyxNQUFNLENBQUMsU0FBUyxDQUFDLE1BQU0sQ0FBQyxRQUFRLEVBQUUsSUFBSSxDQUFDLE9BQU8sQ0FBQyxRQUFRLENBQUMsQ0FBQzs7QUFFeEUsVUFBSSxlQUFlLEdBQUcsTUFBTSxDQUFDLEdBQUcsQ0FBQyxNQUFNLENBQUMsT0FBTyxFQUFFLElBQUksQ0FBQyxPQUFPLENBQUMsT0FBTyxDQUFDLENBQUM7O0FBRXZFOztBQUVFLEFBQUMsWUFBTSxDQUFDLEdBQUcsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLE9BQU8sRUFBRSxRQUFRLENBQUMsR0FBRyxDQUFDLElBRTlDLGVBQWUsR0FBRyxDQUFDLElBQUksQUFBQztRQUN6QjtBQUNBLGVBQU8sSUFBSSxDQUFDLElBQUksQ0FBQyxNQUFNLENBQUMsUUFBUSxDQUFDLENBQUM7T0FDbkM7Ozs7Ozs7QUFPRCxVQUFJLGFBQWEsR0FBRyxRQUFRLENBQUMsU0FBUyxFQUFFLElBQUksSUFBSSxDQUFDLE9BQU8sQ0FBQyxRQUFRLEdBQUcsTUFBTSxDQUFDLEtBQUssRUFBRSxDQUFBLEFBQUMsQ0FBQzs7QUFFcEYsVUFBSSxpQkFBaUIsR0FBRyxNQUFNLENBQUMsR0FBRyxDQUFDLE1BQU0sQ0FBQyxRQUFRLEVBQUUsTUFBTSxDQUFDLFFBQVEsQ0FBQyxNQUFNLENBQUMsUUFBUSxFQUFFLGFBQWEsQ0FBQyxDQUFDLENBQUM7QUFDckcsYUFBTyxJQUFJLENBQUMsSUFBSSxDQUFDLGlCQUFpQixDQUFDLENBQUM7S0FDckM7OztXQUVJLGVBQUMsT0FBTyxFQUFFOztBQUViLFVBQUksU0FBUyxHQUFHLE1BQU0sQ0FBQyxTQUFTLENBQUMsT0FBTyxDQUFDLFFBQVEsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDOztBQUUxRSxVQUFJLGFBQWEsR0FBRyxTQUFTLENBQUMsU0FBUyxFQUFFLElBQUksSUFBSSxDQUFDLE9BQU8sQ0FBQyxRQUFRLEdBQUcsT0FBTyxDQUFDLEtBQUssRUFBRSxDQUFBLEFBQUMsQ0FBQztBQUN0RixVQUFJLGlCQUFpQixHQUFHLE1BQU0sQ0FBQyxHQUFHLENBQUMsT0FBTyxDQUFDLFFBQVEsRUFBRSxNQUFNLENBQUMsUUFBUSxDQUFDLE9BQU8sQ0FBQyxRQUFRLEVBQUUsYUFBYSxDQUFDLENBQUMsQ0FBQztBQUN2RyxhQUFPLElBQUksQ0FBQyxJQUFJLENBQUMsaUJBQWlCLENBQUMsQ0FBQztLQUNyQzs7O1dBRUssa0JBQUc7O0FBRVAsVUFBSSxZQUFZLEdBQUcsSUFBSSxNQUFNLENBQUMsS0FBSyxDQUFDLFdBQVcsRUFBRSxHQUFHLElBQUksQ0FBQyxZQUFZLEVBQ3ZDLEtBQUssQ0FBQyxXQUFXLEVBQUUsR0FBRyxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUM7QUFDdkUsVUFBSSxDQUFDLFlBQVksQ0FBQyxHQUFHLENBQUMsWUFBWSxDQUFDLENBQUM7O0FBRXBDLFVBQUksQ0FBQyxZQUFZLENBQ2QsU0FBUyxFQUFFLENBQ1gsUUFBUSxDQUFDLElBQUksQ0FBQyxZQUFZLENBQUMsQ0FBQzs7QUFFL0IsVUFBSSxXQUFXLEdBQUcsTUFBTSxDQUFDLEdBQUcsQ0FBQyxJQUFJLENBQUMsWUFBWSxFQUFFLElBQUksTUFBTSxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsQ0FBQyxDQUFDLENBQUMsQ0FBQzs7O0FBR3BGLFVBQUksV0FBVyxHQUFHLFdBQVcsQ0FBQzs7QUFFOUIsaUJBQVcsQ0FBQyxNQUFNLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxPQUFPLENBQUMsU0FBUyxFQUFFLENBQUMsQ0FBQzs7QUFFckQsaUJBQVcsQ0FBQyxHQUFHLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxRQUFRLENBQUMsQ0FBQzs7QUFFdkMsYUFBTyxNQUFNLENBQUMsU0FBUyxDQUFDLFdBQVcsRUFBRSxJQUFJLENBQUMsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDO0tBQzdEOzs7Ozs7V0FJZ0IsMkJBQUMsU0FBUyxFQUFFOztBQUUzQixVQUFJLFNBQVMsR0FBRyxJQUFJLENBQUMsTUFBTSxDQUFDLHFCQUFxQixHQUNqQyxJQUFJLENBQUMsT0FBTyxDQUFDLEtBQUssRUFBRSxHQUFHLElBQUksQ0FBQyxPQUFPLENBQUMsUUFBUSxHQUM1QyxJQUFJLENBQUMsTUFBTSxDQUFDLHFCQUFxQixDQUFDOztBQUVsRCxVQUFJLENBQUMsT0FBTyxDQUFDLEtBQUssQ0FBQyx1QkFBdUIsQ0FBQyxJQUFJLENBQUMsT0FBTyxFQUFFLFNBQVMsQ0FBQyxDQUFDOztBQUVwRSxVQUFJLDJCQUEyQixHQUFHLElBQUksQ0FBQztBQUN2QyxVQUFJLGFBQWEsR0FBRyxDQUFDLFFBQVEsQ0FBQztBQUM5QixVQUFJLGtCQUFrQixHQUFHLElBQUksQ0FBQzs7QUFFOUIsZUFBUyxDQUFDLE9BQU8sQ0FBQyxVQUFTLFFBQVEsRUFBRTtBQUNuQyxZQUFJLENBQUMsUUFBUSxDQUFDLFFBQVEsRUFBRSxFQUFFO0FBQUUsaUJBQU87U0FBRTs7QUFFckMsWUFBSSxRQUFRLEdBQUcsUUFBUSxDQUFDLFFBQVEsQ0FBQyxLQUFLLEVBQUUsQ0FBQzs7QUFFekMsZ0JBQVEsQ0FBQyxNQUFNLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxPQUFPLENBQUMsU0FBUyxFQUFFLEdBQUcsQ0FBQyxDQUFDLENBQUMsQ0FBQzs7QUFFdkQsZ0JBQVEsQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxRQUFRLENBQUMsQ0FBQzs7QUFFMUMsWUFBSSxRQUFRLENBQUMsQ0FBQyxHQUFHLENBQUMsRUFBRTtBQUFFLGlCQUFPO1NBQUU7Ozs7QUFJL0IsWUFBSSxjQUFjLEdBQUcsUUFBUSxDQUFDLGNBQWMsR0FBRyxJQUFJLENBQUMsT0FBTyxDQUFDLGNBQWMsQ0FBQzs7QUFFM0UsWUFBSSxRQUFRLENBQUMsQ0FBQyxHQUFHLGNBQWMsRUFBRTtBQUFFLGlCQUFPO1NBQUU7Ozs7Ozs7QUFPNUMsWUFBSSxFQUFFLEdBQUcsUUFBUSxDQUFDLENBQUMsQ0FBQztBQUNwQixZQUFJLEVBQUUsR0FBRyxRQUFRLENBQUMsQ0FBQyxDQUFDOzs7QUFHcEIsWUFBSSxRQUFRLEdBQUcsSUFBSSxDQUFDLElBQUksQ0FBQyxjQUFjLEdBQUcsY0FBYyxHQUFHLEVBQUUsR0FBRyxFQUFFLENBQUMsQ0FBQzs7QUFFcEUsWUFBSSxFQUFFLEdBQUcsRUFBRSxHQUFHLFFBQVEsQ0FBQztBQUN2QixZQUFJLEVBQUUsR0FBRyxDQUFDLEVBQUU7QUFBRSxZQUFFLEdBQUcsRUFBRSxHQUFHLFFBQVEsQ0FBQztTQUFFOzs7QUFHbkMsWUFBSSxFQUFFLEdBQUcsYUFBYSxFQUFFO0FBQ3RCLHVCQUFhLEdBQUcsRUFBRSxDQUFDO0FBQ25CLHFDQUEyQixHQUFHLFFBQVEsQ0FBQztBQUN2Qyw0QkFBa0IsR0FBRyxRQUFRLENBQUM7U0FDL0I7T0FDRixFQUFFLElBQUksQ0FBQyxDQUFDOzs7QUFHVCxVQUFJLFFBQVEsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFDOztBQUU1QixVQUFJLDJCQUEyQixFQUFFOztBQUUvQixZQUFJLFVBQVUsR0FBRyxDQUFDLEdBQUcsQ0FBQyxTQUFTLEdBQUcsa0JBQWtCLENBQUMsQ0FBQyxDQUFBLEdBQUksU0FBUyxDQUFDOztBQUVwRSxnQkFBUSxDQUFDLENBQUMsR0FBRyxDQUFDLDJCQUEyQixDQUFDLGNBQWMsR0FDMUMsa0JBQWtCLENBQUMsQ0FBQyxDQUFBLEdBQUksVUFBVSxDQUFDOztBQUVqRCxZQUFJLGFBQWEsR0FBRyxHQUFHLENBQUM7O0FBRXhCLGdCQUFRLENBQUMsQ0FBQyxHQUFHLENBQUMsMkJBQTJCLENBQUMsY0FBYyxHQUMxQyxrQkFBa0IsQ0FBQyxDQUFDLENBQUEsR0FBSSxhQUFhLENBQUM7T0FDckQ7OztBQUdELGNBQVEsQ0FBQyxNQUFNLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxPQUFPLENBQUMsU0FBUyxFQUFFLENBQUMsQ0FBQztBQUNsRCxhQUFPLFFBQVEsQ0FBQztLQUNqQjs7O1dBRVksdUJBQUMsS0FBSyxFQUFFO0FBQ25CLFVBQUksQ0FBQyxhQUFhLEVBQUUsQ0FBQzs7O0FBR3JCLFVBQUksbUJBQW1CLEdBQUcsQ0FBQyxRQUFRLENBQUM7QUFDcEMsVUFBSSxZQUFZLEdBQUcsSUFBSSxDQUFDO0FBQ3hCLFVBQUksV0FBVyxHQUFHLElBQUksQ0FBQzs7QUFFdkIsVUFBSSxRQUFRLEdBQUcsSUFBSSxNQUFNLEVBQUUsQ0FBQztBQUM1QixVQUFJLFFBQVEsRUFBRSxLQUFLLENBQUM7OztBQUdwQixVQUFJLFNBQVMsR0FBRyxTQUFaLFNBQVMsQ0FBWSxFQUFFLEVBQUUsRUFBRSxFQUFFLEVBQUUsRUFBRSxFQUFFLEVBQUU7QUFDdkMsWUFBSSxJQUFJLEdBQUcsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsSUFBRyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsQUFBQyxHQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLElBQUcsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLEFBQUMsQ0FBQztBQUMzRCxZQUFJLElBQUksR0FBRyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxJQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxBQUFDLEdBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsSUFBRyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsQUFBQyxDQUFDOztBQUUzRCxZQUFJLElBQUksR0FBRyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxJQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxBQUFDLEdBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsSUFBRyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsQUFBQyxDQUFDO0FBQzNELFlBQUksSUFBSSxHQUFHLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLElBQUcsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLEFBQUMsR0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxJQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxBQUFDLENBQUM7O0FBRTNELFlBQUssQUFBQyxJQUFJLEtBQUssQ0FBQyxJQUFNLElBQUksS0FBSyxDQUFDLEFBQUMsRUFBRztBQUNsQyxpQkFBTyxLQUFLLENBQUM7U0FDZDs7QUFFRCxZQUFJLENBQUMsR0FBRyxJQUFJLEdBQUMsSUFBSSxDQUFDO0FBQ2xCLFlBQUksQ0FBQyxHQUFHLElBQUksR0FBQyxJQUFJLENBQUM7O0FBRWxCLFlBQUssQUFBQyxDQUFDLEdBQUcsQ0FBQyxJQUFNLENBQUMsR0FBRyxDQUFDLEFBQUMsSUFBSyxDQUFDLEdBQUcsQ0FBQyxBQUFDLElBQUssQ0FBQyxHQUFHLENBQUMsQUFBQyxFQUFHO0FBQzlDLGtCQUFRLEdBQUcsTUFBTSxDQUFDLFFBQVEsQ0FB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O0FBQ3hCLFNBQUcsQ0FBQyxTQUFTLEdBQUcsS0FBSyxDQUFDOztBQUV0QixTQUFHLENBQUMsU0FBUyxFQUFFLENBQUM7QUFDaEIsU0FBRyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxFQUFFLElBQUksQ0FBQyxZQUFZLEVBQUUsQ0FBQyxFQUFFLEtBQUssQ0FBQyxLQUFLLEVBQUUsS0FBSyxDQUFDLENBQUM7QUFDeEQsU0FBRyxDQUFDLE1BQU0sRUFBRSxDQUFDO0FBQ2IsU0FBRyxDQUFDLFNBQVMsRUFBRSxDQUFDOztBQUVoQixTQUFHLENBQUMsU0FBUyxFQUFFLENBQUM7QUFDaEIsU0FBRyxDQUFDLEdBQUcsQ0FBQyxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUMsRUFBRSxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUMsRUFBRSxDQUFDLEVBQUUsQ0FBQyxFQUFFLEtBQUssQ0FBQyxLQUFLLEVBQUUsS0FBSyxDQUFDLENBQUM7QUFDNUUsU0FBRyxDQUFDLE1BQU0sRUFBRSxDQUFDO0FBQ2IsU0FBRyxDQUFDLFNBQVMsRUFBRSxDQUFDOztBQUVoQixTQUFHLENBQUMsT0FBTyxFQUFFLENBQUM7S0FDZjs7O1NBM2VHLGlCQUFpQjs7O0FBOGV2QixNQUFNLENBQUMsT0FBTyxHQUFHLGlCQUFpQixDQUFDIiwiZmlsZSI6InNyYy9zdGVlcmluZy1iZWhhdmlvcnMuZXM2LmpzIiwic291cmNlc0NvbnRlbnQiOlsidmFyIFZlY3RvciA9IHJlcXVpcmUoJ3ZlY3RvcicpO1xudmFyIHV0aWxzICA9IHJlcXVpcmUoJy4vdXRpbHMnKTtcblxuY2xhc3MgU3RlZXJpbmdCZWhhdmlvcnMge1xuICBjb25zdHJ1Y3Rvcih2ZWhpY2xlLCBvcHRpb25zKSB7XG4gICAgdGhpcy52ZWhpY2xlID0gdmVoaWNsZTtcblxuICAgIC8vIEBUT0RPIHNob3VsZCBiZSBvdmVycmlkYWJsZSB3aXRoIG9wdGlvbnNcbiAgICAvLyBAVE9ETyBzaG91bGQgYmUgaW4gYSBub3JtYWxpemVkIGZvcm1cbiAgICB0aGlzLnBhcmFtcyA9IHtcbiAgICAgIG1pbkRldGVjdGlvbkJveExlbmd0aDogNDAsXG4gICAgICB3YWxsRGV0ZWN0aW9uRmVlbGVyTGVuZ3RoOiA0MFxuICAgIH07XG4gICAgLy8gdmFyaWFibGVzIGZvciB3YW5kZXIgYmVoYXZpb3JcbiAgICB0aGlzLndhbmRlclJhZGl1cyA9IDY7XG4gICAgdGhpcy53YW5kZXJEaXN0YW5jZSA9IDMwO1xuICAgIHRoaXMud2FuZGVySml0dGVyID0gNDtcblxuICAgIC8vIGFudGVubmEgdXNlZCBpbiB3YWxsIGF2b2lkYW5jZVxuICAgIHRoaXMuZmVlbGVycyA9IFtdO1xuXG4gICAgLy8gaW5pdGlhbGl6ZSB3YW5kZXIgdGFyZ2V0XG4gICAgLy8gLT4gY2lyY2xlIG9mIHdhbmRlclJhZGl1cyBjZW50ZXJlZCBvbiB0aGUgYWdlbnRcbiAgICB2YXIgdGhldGEgPSB1dGlscy5yYW5kKCkgKiB1dGlscy5Ud29QSTtcbiAgICAvLyBwcm9qZWN0IHRoZXRhIG9uIHRoZSB3YW5kZXIgY2lyY2xlXG4gICAgdGhpcy53YW5kZXJUYXJnZXQgPSBuZXcgVmVjdG9yKHRoaXMud2FuZGVyUmFkaXVzICogTWF0aC5jb3ModGhldGEpLFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLndhbmRlclJhZGl1cyAqIE1hdGguc2luKHRoZXRhKSk7XG5cbiAgfVxuXG4gIC8vIHRoaXMgaGFzIHRvbyBtYW55IGRlcGVuZGVjaWVzIHdpdGggdGhlIHdvcmxkXG4gIGNhbGN1bGF0ZSgpIHtcbiAgICBpZiAoIXRoaXMudmVoaWNsZS53b3JsZC5pc1N0YXJ0ZWQpIHtcbiAgICAgIHJldHVybiBuZXcgVmVjdG9yKCk7XG4gICAgfVxuXG4gICAgdmFyIHR3ZWFrZXJzID0gdGhpcy52ZWhpY2xlLndvcmxkLnN0ZWVyaW5nVHdlYWtlcnM7XG4gICAgdmFyIHN0ZWVyaW5ncyA9IG5ldyBWZWN0b3IoKTtcblxuICAgIGlmICh0aGlzLl9zZWVrICYmIHRoaXMudmVoaWNsZS53b3JsZC50YXJnZXQpIHtcbiAgICAgIC8vIHRoZXNlIHRhcmdldHMgc2hvdWxkIGJlIGluIHRoZSB2ZWhpY2xlIGl0c2VsZiwgbm90IGluIHdvcmxkXG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLnNlZWsodGhpcy52ZWhpY2xlLndvcmxkLnRhcmdldClcbiAgICAgIHN0ZWVyaW5nLm11bHRpcGx5KHR3ZWFrZXJzLnNlZWspO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgaWYgKHRoaXMuX2ZsZWUgJiYgdGhpcy52ZWhpY2xlLndvcmxkLnRhcmdldCkge1xuICAgICAgLy8gdGhlc2UgdGFyZ2V0cyBzaG91bGQgYmUgaW4gdGhlIHZlaGljbGUgaXRzZWxmLCBub3QgaW4gd29ybGRcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMuZmxlZSh0aGlzLnZlaGljbGUud29ybGQudGFyZ2V0KVxuICAgICAgc3RlZXJpbmcubXVsdGlwbHkodHdlYWtlcnMuZmxlZSk7XG4gICAgICBzdGVlcmluZ3MuYWRkKHN0ZWVyaW5nKTtcbiAgICB9XG5cbiAgICBpZiAodGhpcy5fYXJyaXZlICYmIHRoaXMudmVoaWNsZS53b3JsZC50YXJnZXQpIHtcbiAgICAgIC8vIHRoZXNlIHRhcmdldHMgc2hvdWxkIGJlIGluIHRoZSB2ZWhpY2xlIGl0c2VsZiwgbm90IGluIHdvcmxkXG4gICAgICB2YXIgZGVjZWxlcmF0aW9uID0gMztcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMuYXJyaXZlKHRoaXMudmVoaWNsZS53b3JsZC50YXJnZXQsIGRlY2VsZXJhdGlvbik7XG4gICAgICBzdGVlcmluZy5tdWx0aXBseSh0d2Vha2Vycy5hcnJpdmUpO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgaWYgKHRoaXMuX3B1cnN1aXQpIHtcbiAgICAgIC8vIHRoZXNlIHRhcmdldHMgc2hvdWxkIGJlIGluIHRoZSB2ZWhpY2xlIGl0c2VsZiwgbm90IGluIHdvcmxkXG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLnB1cnN1aXQodGhpcy52ZWhpY2xlLndvcmxkLmV2YWRlcik7XG4gICAgICBzdGVlcmluZy5tdWx0aXBseSh0d2Vha2Vycy5wdXJzdWl0KTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIGlmICh0aGlzLiBfZXZhZGUpIHtcbiAgICAgIC8vIHRoZXNlIHRhcmdldHMgc2hvdWxkIGJlIGluIHRoZSB2ZWhpY2xlIGl0c2VsZiwgbm90IGluIHdvcmxkXG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLmV2YWRlKHRoaXMudmVoaWNsZS53b3JsZC5wdXJzdWVyKTtcbiAgICAgIHN0ZWVyaW5nLm11bHRpcGx5KHR3ZWFrZXJzLmV2YWRlKTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIGlmICh0aGlzLl93YW5kZXIpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMud2FuZGVyKCk7XG4gICAgICBzdGVlcmluZy5tdWx0aXBseSh0d2Vha2Vycy53YW5kZXIpO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgLy8gZG9lc24ndCB3b3JrLi4uXG4gICAgaWYgKHRoaXMuX29ic3RhY2xlQXZvaWRhbmNlKSB7XG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLm9ic3RhY2xlQXZvaWRhbmNlKHRoaXMudmVoaWNsZS53b3JsZC5vYnN0YWNsZXMpXG4gICAgICBzdGVlcmluZy5tdWx0aXBseSh0d2Vha2Vycy5vYnN0YWNsZUF2b2lkYW5jZSk7XG4gICAgICBzdGVlcmluZ3MuYWRkKHN0ZWVyaW5nKTtcbiAgICB9XG5cbiAgICAvLyBkb2Vzbid0IHdvcmsgd2VsbFxuICAgIGlmICh0aGlzLl93YWxsQXZvaWRhbmNlKSB7XG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLndhbGxBdm9pZGFuY2UodGhpcy52ZWhpY2xlLndvcmxkLndhbGxzKVxuICAgICAgc3RlZXJpbmcubXVsdGlwbHkodHdlYWtlcnMud2FsbEF2b2lkYW5jZSk7XG4gICAgICBzdGVlcmluZ3MuYWRkKHN0ZWVyaW5nKTtcbiAgICB9XG5cbiAgICBpZiAodGhpcy5fZmxvY2spIHtcbiAgICAgIC8vIHRhZyBuZWlnaGJvcnNcbiAgICAgIHRoaXMudmVoaWNsZS53b3JsZC50YWdCb2lkc1dpdGhpblJhbmdlKHRoaXMudmVoaWNsZSk7XG5cbiAgICAgIHZhciBmbG9ja3MgPSBuZXcgVmVjdG9yKCk7XG4gICAgICBmbG9ja3MuYWRkKHRoaXMuc2VwYXJhdGlvbih0aGlzLnZlaGljbGUubmVpZ2hib3JzKS5tdWx0aXBseSh0d2Vha2Vycy5zZXBhcmF0aW9uKSk7XG4gICAgICBmbG9ja3MuYWRkKHRoaXMuYWxpZ25tZW50KHRoaXMudmVoaWNsZS5uZWlnaGJvcnMpLm11bHRpcGx5KHR3ZWFrZXJzLmFsaWdubWVudCkpO1xuICAgICAgZmxvY2tzLmFkZCh0aGlzLmNvaGVzaW9uKHRoaXMudmVoaWNsZS5uZWlnaGJvcnMpLm11bHRpcGx5KHR3ZWFrZXJzLmNvaGVzaW9uKSk7XG5cbiAgICAgIHN0ZWVyaW5ncy5hZGQoZmxvY2tzKTtcbiAgICB9XG5cblxuICAgIHJldHVybiBzdGVlcmluZ3M7XG4gIH1cblxuICAvLyBiZWhhdmlvcnMgT24vT2ZmXG4gIC8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG5cbiAgc2Vla09uKCkgeyB0aGlzLl9zZWVrID0gdHJ1ZTsgfVxuICBzZWVrT2ZmKCkgeyB0aGlzLl9zZWVrID0gZmFsc2U7IH1cblxuICBmbGVlT24oKSB7IHRoaXMuX2ZsZWUgPSB0cnVlOyB9XG4gIGZsZWVPZmYoKSB7IHRoaXMuX2ZsZWUgPSBmYWxzZTsgfVxuXG4gIGFycml2ZU9uKCkgeyB0aGlzLl9hcnJpdmUgPSB0cnVlOyB9XG4gIGFycml2ZU9mZigpIHsgdGhpcy5fYXJyaXZlID0gZmFsc2U7IH1cblxuICBwdXJzdWl0T24oKSB7IHRoaXMuX3B1cnN1aXQgPSB0cnVlOyB9XG4gIHB1cnN1aXRPZmYoKSB7IHRoaXMuX3B1cnN1aXQgPSBmYWxzZTsgfVxuXG4gIGV2YWRlT24oKSB7IHRoaXMuX2V2YWRlID0gdHJ1ZTsgfVxuICBldmFkZU9mZigpIHsgdGhpcy5fZXZhZGUgPSBmYWxzZTsgfVxuXG4gIHdhbmRlck9uKCkgeyB0aGlzLl93YW5kZXIgPSB0cnVlOyB9XG4gIHdhbmRlck9mZigpIHsgdGhpcy5fd2FuZGVyID0gZmFsc2U7IH1cblxuICBvYnN0YWNsZUF2b2lkYW5jZU9uKCkgeyB0aGlzLl9vYnN0YWNsZUF2b2lkYW5jZSA9IHRydWU7IH1cbiAgb2JzdGFjbGVBdm9pZGFuY2VPZmYoKSB7IHRoaXMuX29ic3RhY2xlQXZvaWRhbmNlID0gZmFsc2U7IH1cblxuICB3YWxsQXZvaWRhbmNlT24oKSB7IHRoaXMuX3dhbGxBdm9pZGFuY2UgPSB0cnVlOyB9XG4gIHdhbGxBdm9pZGFuY2VPZmYoKSB7IHRoaXMuX3dhbGxBdm9pZGFuY2UgPSBmYWxzZTsgfVxuXG4gIGZsb2NrT24oKSB7IHRoaXMuX2Zsb2NrID0gdHJ1ZTsgfVxuICBmbG9ja09mZigpIHsgdGhpcy5fZmxvY2sgPSBmYWxzZTsgfVxuXG4gIC8vIGJlaGF2aW9yc1xuICAvLyAtLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuXG4gIHNlZWsodGFyZ2V0UG9zaXRpb24pIHtcbiAgICB2YXIgZGVzaXJlZFZlbG9jaXR5ID0gVmVjdG9yLnN1YnN0cmFjdCh0YXJnZXRQb3NpdGlvbiwgdGhpcy52ZWhpY2xlLnBvc2l0aW9uKVxuICAgICAgLm5vcm1hbGl6ZSgpXG4gICAgICAubXVsdGlwbHkodGhpcy52ZWhpY2xlLm1heFNwZWVkKTtcblxuICAgIHZhciBzdGVlcmluZyA9IFZlY3Rvci5zdWJzdHJhY3QoZGVzaXJlZFZlbG9jaXR5LCB0aGlzLnZlaGljbGUudmVsb2NpdHkpO1xuICAgIHJldHVybiBzdGVlcmluZztcbiAgfVxuXG4gIGZsZWUodGFyZ2V0UG9zaXRpb24pIHtcbiAgICB2YXIgcGFuaWNEaXN0YW5jZSA9IDEwMCAqIDEwMDsgLy8gdXNlIHNxdWFyZSBkb21haW4gdG8gc2F2ZSBjb21wdXRhdGlvbnNcbiAgICBpZiAoVmVjdG9yLmRpc3RhbmNlU3FydCh0aGlzLnZlaGljbGUucG9zaXRpb24sIHRhcmdldFBvc2l0aW9uKSA+IHBhbmljRGlzdGFuY2UpIHtcbiAgICAgIHJldHVybiBuZXcgVmVjdG9yKCk7XG4gICAgfVxuXG4gICAgdmFyIGRlc2lyZWRWZWxvY2l0eSA9IFZlY3Rvci5zdWJzdHJhY3QodGhpcy52ZWhpY2xlLnBvc2l0aW9uLCB0YXJnZXRQb3NpdGlvbilcbiAgICAgIC5ub3JtYWxpemUoKVxuICAgICAgLm11bHRpcGx5KHRoaXMudmVoaWNsZS5tYXhTcGVlZCk7XG5cbiAgICB2YXIgc3RlZXJpbmcgPSBWZWN0b3Iuc3Vic3RyYWN0KGRlc2lyZWRWZWxvY2l0eSwgdGhpcy52ZWhpY2xlLnZlbG9jaXR5KTtcbiAgICByZXR1cm4gc3RlZXJpbmc7XG4gIH1cblxuICAvLyBkZWNlbGVyYXRpb24gaXMgYW4gZW51bSAoc2xvdyA9IDEsIG5vcm1hbCA9IDIsIGZhc3QgPSAzKVxuICAvLyBpdCBkZXNjcmliZXMgdGhlIHRpbWUgdGhlIGFnZW50IHNob3VsZCB0YWtlIHRvIGFycml2ZSBhdCBkZXN0aW5hdGlvblxuICBhcnJpdmUodGFyZ2V0UG9zaXRpb24sIGRlY2VsZXJhdGlvbikge1xuICAgIHZhciB0b1RhcmdldCA9IFZlY3Rvci5zdWJzdHJhY3QodGFyZ2V0UG9zaXRpb24sIHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gICAgdmFyIGRpc3RhbmNlID0gdG9UYXJnZXQubWFnbml0dWRlKCk7XG5cbiAgICBpZiAoZGlzdGFuY2UgPiAwKSB7XG4gICAgICAvLyBhbGxvdyB0byB0d2VhayBkZWNlbGVyYXRpb25cbiAgICAgIHZhciBkZWNlbGVyYXRpb25Ud2Vha2VyID0gMC4zO1xuICAgICAgLy8gZGVmaW5lIHRoZSBzcGVlZCB0aGUgYWdlbnQgc2hvdWxkIGhhdmUgdG8gYXJyaXZlIGF0IGRlc3RpbmF0aW9uXG4gICAgICB2YXIgc3BlZWQgPSBkaXN0YW5jZSAvIChkZWNlbGVyYXRpb24gKiBkZWNlbGVyYXRpb25Ud2Vha2VyKTtcbiAgICAgIC8vIHNwZWVkIHNob3VsZG4ndCBleGNlZWQgbWF4U3BlZWRcbiAgICAgIHNwZWVkID0gTWF0aC5taW4oc3BlZWQsIHRoaXMudmVoaWNsZS5tYXhTcGVlZCk7XG4gICAgICAvLyBuZXh0IHN0ZXBzIGFyZSBzYW1lIGFzIHNlZWtcbiAgICAgIHZhciBkZXNpcmVkVmVsb2NpdHkgPSB0b1RhcmdldFxuICAgICAgICAuZGl2aWRlKGRpc3RhbmNlKSAvLyA8PT4gdG9UYXJnZXQubm9ybWFsaXplKCk7XG4gICAgICAgIC5tdWx0aXBseShzcGVlZCk7XG5cbiAgICAgIHZhciBzdGVlcmluZyA9IFZlY3Rvci5zdWJzdHJhY3QoZGVzaXJlZFZlbG9jaXR5LCB0aGlzLnZlaGljbGUudmVsb2NpdHkpO1xuICAgICAgcmV0dXJuIHN0ZWVyaW5nO1xuICAgIH1cblxuICAgIHJldHVybiBuZXcgVmVjdG9yKCk7XG4gIH1cblxuICBwdXJzdWl0KGV2YWRlcikge1xuICAgIHZhciB0b0V2YWRlciA9IFZlY3Rvci5zdWJzdHJhY3QoZXZhZGVyLnBvc2l0aW9uLCB0aGlzLnZlaGljbGUucG9zaXRpb24pO1xuICAgIC8vIGNvc2luZSBvZiB0aGUgYW5nbGUgYmV0d2VlbiB0aGUgMiBhZ2VudHMgaGVhZGluZ3NcbiAgICB2YXIgcmVsYXRpdmVIZWFkaW5nID0gVmVjdG9yLmRvdChldmFkZXIuaGVhZGluZywgdGhpcy52ZWhpY2xlLmhlYWRpbmcpO1xuXG4gICAgaWYgKFxuICAgICAgLy8gaWYgdGhlIHRoZSBldmFkZXIgaXMgaW4gZnJvbnQgb2YgdGhlIHB1cnN1ZXJcbiAgICAgIChWZWN0b3IuZG90KHRoaXMudmVoaWNsZS5oZWFkaW5nLCB0b0V2YWRlcikgPiAwKSAmJlxuICAgICAgLy8gYW5kIHRoZSB0d28gYWdlbnRzIGFyZSBhcHByb3ggZmFjZSB0byBmYWNlXG4gICAgICAocmVsYXRpdmVIZWFkaW5nIDwgLTAuOTUpIC8vIGFjb3MoMC45NSkgPSAxOGRlZ1xuICAgICkge1xuICAgICAgcmV0dXJuIHRoaXMuc2VlayhldmFkZXIucG9zaXRpb24pO1xuICAgIH1cblxuICAgIC8vIG5vdCBjb25zaXJkZXJlZCBhaGVhZCBzbyB3ZSBoYXZlIHRvXG4gICAgLy8gcHJlZGljdCB0aGUgZnV0dXJlIHBvc2l0aW9uIG9mIHRoZSBldmFkZXJcblxuICAgIC8vIHRoZSBsb29rQWhlYWRUaW1lIHNob3VsZCBiZSBwcm9wb3J0aW9ubmFsIHRvIHRoZSBkaXN0YW5jZSxcbiAgICAvLyBhbmQgaW52ZXJzbHkgcHJvcG9ydGlvbm5hbCB0byB0aGUgc3BlZWQgb2YgdGhlIGFnZW50c1xuICAgIHZhciBsb29rQWhlYWRUaW1lID0gdG9FdmFkZXIubWFnbml0dWRlKCkgLyAodGhpcy52ZWhpY2xlLm1heFNwZWVkICsgZXZhZGVyLnNwZWVkKCkpO1xuICAgIC8vIG5vdyBzZWVrIHRvIGEgcHJlZGljdGlvbiBvZiB0aGUgZXZhZGVyIHBvc2l0aW9uXG4gICAgdmFyIHByZWRpY3RlZFBvc2l0aW9uID0gVmVjdG9yLmFkZChldmFkZXIucG9zaXRpb24sIFZlY3Rvci5tdWx0aXBseShldmFkZXIudmVsb2NpdHksIGxvb2tBaGVhZFRpbWUpKTtcbiAgICByZXR1cm4gdGhpcy5zZWVrKHByZWRpY3RlZFBvc2l0aW9uKTtcbiAgfVxuXG4gIGV2YWRlKHB1cnN1ZXIpIHtcbiAgICAvLyBubyBuZWVkIHRvIGNoZWNrIGlmIHRoZSBhZ2VudHMgYXJlIGZhY2luZ1xuICAgIHZhciB0b1B1cnN1ZXIgPSBWZWN0b3Iuc3Vic3RyYWN0KHB1cnN1ZXIucG9zaXRpb24sIHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gICAgLy8gdGhlbiBzYW1lIGFzIHB1cnN1aXQgYnV0IHJlcGxhY2Ugc2VlayB3aXRoIGZsZWVcbiAgICB2YXIgbG9va0FoZWFkVGltZSA9IHRvUHVyc3Vlci5tYWduaXR1ZGUoKSAvICh0aGlzLnZlaGljbGUubWF4U3BlZWQgKyBwdXJzdWVyLnNwZWVkKCkpO1xuICAgIHZhciBwcmVkaWN0ZWRQb3NpdGlvbiA9IFZlY3Rvci5hZGQocHVyc3Vlci5wb3NpdGlvbiwgVmVjdG9yLm11bHRpcGx5KHB1cnN1ZXIudmVsb2NpdHksIGxvb2tBaGVhZFRpbWUpKTtcbiAgICByZXR1cm4gdGhpcy5mbGVlKHByZWRpY3RlZFBvc2l0aW9uKTtcbiAgfVxuXG4gIHdhbmRlcigpIHtcbiAgICAvLyBhZGQgYSBzbWFsbCByYW5kb20gdmVjdG9yIHRvIHRoZSB3YW5kZXJUYXJnZXRcbiAgICB2YXIgcmFuZG9tVmVjdG9yID0gbmV3IFZlY3Rvcih1dGlscy5yYW5kQ2xhbXBlZCgpICogdGhpcy53YW5kZXJKaXR0ZXIsXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdXRpbHMucmFuZENsYW1wZWQoKSAqIHRoaXMud2FuZGVySml0dGVyKTtcbiAgICB0aGlzLndhbmRlclRhcmdldC5hZGQocmFuZG9tVmVjdG9yKTtcbiAgICAvLyByZXByb2plY3QgdGhlIHdhbmRlclRhcmdldCBvbiB0aGUgd2FuZGVyIGNpcmNsZVxuICAgIHRoaXMud2FuZGVyVGFyZ2V0XG4gICAgICAubm9ybWFsaXplKClcbiAgICAgIC5tdWx0aXBseSh0aGlzLndhbmRlclJhZGl1cyk7XG4gICAgLy8gcHJvamVjdCB0aGUgd2FuZGVyIGNpcmNsZSBpbiBmcm9udCBvZiB0aGUgYWdlbnRcbiAgICB2YXIgdGFyZ2V0TG9jYWwgPSBWZWN0b3IuYWRkKHRoaXMud2FuZGVyVGFyZ2V0LCBuZXcgVmVjdG9yKHRoaXMud2FuZGVyRGlzdGFuY2UsIDApKTtcbiAgICAvLyBAVE9ETyBzaG91bGQgYmUgYW4gbWF0cml4IHRyYW5zZm9ybXMgdXRpbHNcbiAgICAvLyBwcm9qZWN0IHRoZSB0YXJnZXQgaW4gd29ybGQgc3BhY2UgLSBjaGFuZ2UgbmFtZSBmb3IgdW5kZXJzdGFuZGFiaWxpdHlcbiAgICB2YXIgdGFyZ2V0V29ybGQgPSB0YXJnZXRMb2NhbDtcbiAgICAvLyByb3RhdGVcbiAgICB0YXJnZXRXb3JsZC5yb3RhdGUodGhpcy52ZWhpY2xlLmhlYWRpbmcuZGlyZWN0aW9uKCkpO1xuICAgIC8vIHRyYW5zbGF0ZVxuICAgIHRhcmdldFdvcmxkLmFkZCh0aGlzLnZlaGljbGUucG9zaXRpb24pO1xuICAgIC8vIHN0ZWVyIHRvd2FyZCB0aGlzIHRhcmdldFxuICAgIHJldHVybiBWZWN0b3Iuc3Vic3RyYWN0KHRhcmdldFdvcmxkLCB0aGlzLnZlaGljbGUucG9zaXRpb24pO1xuICB9XG5cbiAgLy8gcHJldmVudCB0aGUgYWdlbnQgY29sbGlkaW5nIHdpdGggdGhlIGNsb3Nlc3Qgb2JzdGFjbGVcbiAgLy8gQE5PVEUgLSBub3QgdGVzdGVkIHByb3Blcmx5XG4gIG9ic3RhY2xlQXZvaWRhbmNlKG9ic3RhY2xlcykge1xuICAgIC8vIGNyZWF0ZSBhIGRldGVjdGlvbiBib3ggcHJvcG9ydGlvbm5hbCB0byB0aGUgYWdlbnQgdmVsb2NpdHlcbiAgICB2YXIgYm94TGVuZ3RoID0gdGhpcy5wYXJhbXMubWluRGV0ZWN0aW9uQm94TGVuZ3RoICtcbiAgICAgICAgICAgICAgICAgICAgdGhpcy52ZWhpY2xlLnNwZWVkKCkgLyB0aGlzLnZlaGljbGUubWF4U3BlZWQgKlxuICAgICAgICAgICAgICAgICAgICB0aGlzLnBhcmFtcy5taW5EZXRlY3Rpb25Cb3hMZW5ndGg7XG5cbiAgICB0aGlzLnZlaGljbGUud29ybGQudGFnT2JzdGFjbGVzV2l0aGluUmFuZ2UodGhpcy52ZWhpY2xlLCBib3hMZW5ndGgpO1xuICAgIC8vIGNsb3Nlc3QgaW50ZXJzZWN0aW5nIG9ic3RhY2xlIChDSU8pXG4gICAgdmFyIGNsb3Nlc3RJbnRlcmNlcHRpbmdPYnN0YWNsZSA9IG51bGw7XG4gICAgdmFyIGRpc3RhbmNlVG9DSU8gPSArSW5maW5pdHk7XG4gICAgdmFyIGxvY2FsUG9zaXRpb25PZkNJTyA9IG51bGw7XG5cbiAgICBvYnN0YWNsZXMuZm9yRWFjaChmdW5jdGlvbihvYnN0YWNsZSkge1xuICAgICAgaWYgKCFvYnN0YWNsZS5pc1RhZ2dlZCgpKSB7IHJldHVybjsgfVxuICAgICAgLy8gZmluZCBsb2NhbCBjb29yZGluYXRlcyBvZiB0aGUgb2JzdGFjbGVcbiAgICAgIHZhciBsb2NhbFBvcyA9IG9ic3RhY2xlLnBvc2l0aW9uLmNsb25lKCk7XG4gICAgICAvLyByb3RhdGVcbiAgICAgIGxvY2FsUG9zLnJvdGF0ZSh0aGlzLnZlaGljbGUuaGVhZGluZy5kaXJlY3Rpb24oKSAqIC0xKTtcbiAgICAgIC8vIHRyYW5zbGF0ZVxuICAgICAgbG9jYWxQb3Muc3Vic3RyYWN0KHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gICAgICAvLyBpZiB0aGUgbG9jYWwgeCB2YWx1ZSBpcyBuZWdhdGl2ZSwgdGhlIG9ic3RhY2xlIGlzIGJlaGluZCB0aGUgYWdlbnRcbiAgICAgIGlmIChsb2NhbFBvcy54IDwgMCkgeyByZXR1cm47IH1cblxuICAgICAgLy8gaWYgdGhlIGRpc3RhbmNlIGJldHdlZW4gdGhlIHggYXhpcyB0byB0aGUgb2plY3QgbG9jYWwgcG9zaXRpb24gaXMgbGVzcyB0aGFuXG4gICAgICAvLyBpdHMgcmFkaXVzICsgdGhlIHJhZGl1cyBvZiB0aGlzLnZlaGljbGUsIHRoZXJlIGlzIGEgcG9zc2libGUgY29sbGlzaW9uXG4gICAgICB2YXIgZXhwYW5kZWRSYWRpdXMgPSBvYnN0YWNsZS5ib3VuZGluZ1JhZGl1cyArIHRoaXMudmVoaWNsZS5ib3VuZGluZ1JhZGl1cztcblxuICAgICAgaWYgKGxvY2FsUG9zLnkgPiBleHBhbmRlZFJhZGl1cykgeyByZXR1cm47IH1cblxuICAgICAgLy8gbm93IHRvIGRvIGEgbGluZS9jaXJjbGUgaW50ZXJzZWN0aW9uIHRlc3QuIFRoZSBjZW50ZXIgb2YgdGhlXG4gICAgICAvLyBjaXJjbGUgaXMgcmVwcmVzZW50ZWQgYnkgKGNYLCBjWSkuIFRoZSBpbnRlcnNlY3Rpb24gcG9pbnRzIGFyZVxuICAgICAgLy8gZ2l2ZW4gYnkgdGhlIGZvcm11bGEgeCA9IGNYICsvLXNxcnQocl4yLWNZXjIpIGZvciB5PTAuXG4gICAgICAvLyBXZSBvbmx5IG5lZWQgdG8gbG9vayBhdCB0aGUgc21hbGxlc3QgcG9zaXRpdmUgdmFsdWUgb2YgeCBiZWNhdXNlXG4gICAgICAvLyB0aGF0IHdpbGwgYmUgdGhlIGNsb3Nlc3QgcG9pbnQgb2YgaW50ZXJzZWN0aW9uLlxuICAgICAgdmFyIGNYID0gbG9jYWxQb3MueDtcbiAgICAgIHZhciBjWSA9IGxvY2FsUG9zLnk7XG5cbiAgICAgIC8vIGNhbGN1bGUgdGhlIHNxcnQgcGFydCBvZiB0aGUgZXF1YXRpb24gb25seSBvbmNlXG4gICAgICB2YXIgc3FydFBhcnQgPSBNYXRoLnNxcnQoZXhwYW5kZWRSYWRpdXMgKiBleHBhbmRlZFJhZGl1cyAtIGNZICogY1kpO1xuXG4gICAgICB2YXIgaXAgPSBjWCAtIHNxcnRQYXJ0O1xuICAgICAgaWYgKGlwIDwgMCkgeyBpcCA9IGNYICsgc3FydFBhcnQ7IH1cblxuICAgICAgLy8gaWYgY2xvc2VzdCBzbyBmYXIgLSBzdG9yZSBhbGwgaXRzIHZhbHVlXG4gICAgICBpZiAoaXAgPCBkaXN0YW5jZVRvQ0lPKSB7XG4gICAgICAgIGRpc3RhbmNlVG9DSU8gPSBpcDtcbiAgICAgICAgY2xvc2VzdEludGVyY2VwdGluZ09ic3RhY2xlID0gb2JzdGFjbGU7XG4gICAgICAgIGxvY2FsUG9zaXRpb25PZkNJTyA9IGxvY2FsUG9zO1xuICAgICAgfVxuICAgIH0sIHRoaXMpO1xuXG4gICAgLy8gc3RpbGwgaW4gbG9jYWwgc3BhY2VcbiAgICB2YXIgc3RlZXJpbmcgPSBuZXcgVmVjdG9yKCk7XG4gICAgLy8gaWYgd2UgZm91bmQgc29tZSBvYnRhY2xlLCBjYWxjdWxhdGUgYSBzdGVlcmluZyBmb3JjZSBhd2F5IGZyb20gaXRcbiAgICBpZiAoY2xvc2VzdEludGVyY2VwdGluZ09ic3RhY2xlKSB7XG4gICAgICAvLyB0aGUgY2xvc2VyIGFuIGFnZW50IGlzIHRvIGFuIG9iamVjdCwgdGhlIHN0cm9uZ2VyIHRoZSBzdGVlcmluZyAoYmV0d2VlbiAxIGFuZCAyIFs/XSlcbiAgICAgIHZhciBtdWx0aXBsaWVyID0gMSArIChib3hMZW5ndGggLSBsb2NhbFBvc2l0aW9uT2ZDSU8ueCkgLyBib3hMZW5ndGg7XG5cbiAgICAgIHN0ZWVyaW5nLnkgPSAoY2xvc2VzdEludGVyY2VwdGluZ09ic3RhY2xlLmJvdW5kaW5nUmFkaXVzIC1cbiAgICAgICAgICAgICAgICAgICAgbG9jYWxQb3NpdGlvbk9mQ0lPLnkpICogbXVsdGlwbGllcjtcblxuICAgICAgdmFyIGJyYWtpbmdXZWlnaHQgPSAwLjI7XG5cbiAgICAgIHN0ZWVyaW5nLnggPSAoY2xvc2VzdEludGVyY2VwdGluZ09ic3RhY2xlLmJvdW5kaW5nUmFkaXVzIC1cbiAgICAgICAgICAgICAgICAgICAgbG9jYWxQb3NpdGlvbk9mQ0lPLngpICogYnJha2luZ1dlaWdodDtcbiAgICB9XG5cbiAgICAvLyByb3RhdGUgdG8gZ28gYmFjayB0byB3b3JsZCBzcGFjZVxuICAgIHN0ZWVyaW5nLnJvdGF0ZSh0aGlzLnZlaGljbGUuaGVhZGluZy5kaXJlY3Rpb24oKSk7XG4gICAgcmV0dXJuIHN0ZWVyaW5nO1xuICB9XG5cbiAgd2FsbEF2b2lkYW5jZSh3YWxscykge1xuICAgIHRoaXMuY3JlYXRlRmVlbGVycygpO1xuXG4gICAgLy8gSVA6IEludGVyc2VjdGlvbiBQb2ludFxuICAgIHZhciBkaXN0YW5jZVRvQ2xvc2VzdElQID0gK0luZmluaXR5O1xuICAgIHZhciBjbG9zZXN0UG9pbnQgPSBudWxsO1xuICAgIHZhciBjbG9zZXN0V2FsbCA9IG51bGw7XG5cbiAgICB2YXIgc3RlZXJpbmcgPSBuZXcgVmVjdG9yKCk7XG4gICAgdmFyIGRpc3RhbmNlLCBwb2ludDtcblxuICAgIC8vIGdlb21ldHJ5LmggKGxpbmUgMjg0KSBMaW5lSW50ZXJzZWN0aW9uMmQgKD8/PyB3aGF0IGFwcGVuZHMgaGVyZSA/Pz8pXG4gICAgdmFyIGludGVyc2VjdCA9IGZ1bmN0aW9uKGYxLCB0MSwgZjIsIHQyKSB7XG4gICAgICB2YXIgclRvcCA9IChmMS55LWYyLnkpKih0Mi54LWYyLngpLShmMS54LWYyLngpKih0Mi55LWYyLnkpO1xuICAgICAgdmFyIHJCb3QgPSAodDEueC1mMS54KSoodDIueS1mMi55KS0odDEueS1mMS55KSoodDIueC1mMi54KTtcblxuICAgICAgdmFyIHNUb3AgPSAoZjEueS1mMi55KSoodDEueC1mMS54KS0oZjEueC1mMi54KSoodDEueS1mMS55KTtcbiAgICAgIHZhciBzQm90ID0gKHQxLngtZjEueCkqKHQyLnktZjIueSktKHQxLnktZjEueSkqKHQyLngtZjIueCk7XG5cbiAgICAgIGlmICggKHJCb3QgPT09IDApIHx8wqAoc0JvdCA9PT0gMCkgKSB7XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgIH1cblxuICAgICAgdmFyIHIgPSByVG9wL3JCb3Q7XG4gICAgICB2YXIgcyA9IHNUb3Avc0JvdDtcblxuICAgICAgaWYgKCAociA+IDApICYmIChyIDwgMSkgJiYgKHMgPiAwKSAmJiAocyA8IDEpICkge1xuICAgICAgICBkaXN0YW5jZSA9IFZlY3Rvci5kaXN0YW5jZShmMSwgdDEpICogcjtcbiAgICAgICAgLy8gY29uc29sZS5sb2cociwgZGlzdGFuY2UpO1xuICAgICAgICBwb2ludCA9IG5ldyBWZWN0b3IoZjEueCArIHIsIGYxLnkgKyByKTtcbiAgICAgICAgLy8gY2hhbmdlIGZyb20gYysrIGNvZGUgaGVyZSwgdG9vIG11Y2ggZm9yY2Ugc29tZXRoaW5nIGlzIHdyb25nICguLi5wcm9iYWJseSBtZSlcbiAgICAgICAgLy8gcG9pbnQubXVsdGlwbHkoVmVjdG9yLnN1YnN0cmFjdCh0MSwgZjEpKTtcbiAgICAgICAgcG9pbnQubXVsdGlwbHkoMS4wOCk7XG5cbiAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBkaXN0YW5jZSA9IDA7XG5cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgfVxuICAgIH1cblxuICAgIC8vIGV4YW1pbmUgZWFjaCBmZWVsZXIgb25uIGVhY2ggd2FsbFxuICAgIHRoaXMuZmVlbGVycy5mb3JFYWNoKGZ1bmN0aW9uKGZlZWxlcikge1xuICAgICAgd2FsbHMuZm9yRWFjaChmdW5jdGlvbih3YWxsKSB7XG4gICAgICAgIGlmIChpbnRlcnNlY3QodGhpcy52ZWhpY2xlLnBvc2l0aW9uLCBmZWVsZXIsIHdhbGwuZnJvbSwgd2FsbC50bykpIHtcbiAgICAgICAgICBpZiAoZGlzdGFuY2UgPCBkaXN0YW5jZVRvQ2xvc2VzdElQKSB7XG4gICAgICAgICAgICBkaXN0YW5jZVRvQ2xvc2VzdElQID0gZGlzdGFuY2U7XG4gICAgICAgICAgICBjbG9zZXN0V2FsbCA9IHdhbGw7XG4gICAgICAgICAgICBjbG9zZXN0UG9pbnQgPSBwb2ludDtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgIH0sIHRoaXMpO1xuXG5cbiAgICAgIGlmIChjbG9zZXN0V2FsbCkge1xuICAgICAgICB2YXIgb3ZlclNob290ID0gVmVjdG9yLnN1YnN0cmFjdChmZWVsZXIsIGNsb3Nlc3RQb2ludCk7XG4gICAgICAgIC8vIGNvbnNvbGUubG9nKG92ZXJTaG9vdC5tYWduaXR1ZGUoKSlcbiAgICAgICAgc3RlZXJpbmcgPSBWZWN0b3IubXVsdGlwbHkoY2xvc2VzdFdhbGwubm9ybWFsLCBvdmVyU2hvb3QubWFnbml0dWRlKCkpO1xuICAgICAgfVxuICAgIH0sIHRoaXMpO1xuXG4gICAgcmV0dXJuIHN0ZWVyaW5nO1xuICB9XG5cbiAgLy8gY3JlYXRlIGFudGVubmEgdXNlZCBpbiB3YWxsQXZvaWRhbmNlIC0gaW4gd29ybGQgY29vcmRpbmF0ZXNcbiAgY3JlYXRlRmVlbGVycygpIHtcbiAgICAvLyBjZW50ZXJcbiAgICB2YXIgdG1wID0gVmVjdG9yLm11bHRpcGx5KHRoaXMudmVoaWNsZS5oZWFkaW5nLCB0aGlzLnBhcmFtcy53YWxsRGV0ZWN0aW9uRmVlbGVyTGVuZ3RoKTtcbiAgICB0aGlzLmZlZWxlcnNbMF0gPSBWZWN0b3IuYWRkKHRoaXMudmVoaWNsZS5wb3NpdGlvbiwgdG1wKTtcbiAgICAvLyBsZWZ0XG4gICAgdmFyIHRtcCA9IHRoaXMudmVoaWNsZS5oZWFkaW5nLmNsb25lKCk7XG4gICAgdG1wLnJvdGF0ZShNYXRoLlBJIC8gMiAqIDMuNSk7XG4gICAgdG1wID0gVmVjdG9yLm11bHRpcGx5KHRtcCwgdGhpcy5wYXJhbXMud2FsbERldGVjdGlvbkZlZWxlckxlbmd0aCAvIDIpO1xuICAgIHRoaXMuZmVlbGVyc1sxXSA9IFZlY3Rvci5hZGQodGhpcy52ZWhpY2xlLnBvc2l0aW9uLCB0bXApO1xuICAgIC8vIHJpZ2h0XG4gICAgdmFyIHRtcCA9IHRoaXMudmVoaWNsZS5oZWFkaW5nLmNsb25lKCk7XG4gICAgdG1wLnJvdGF0ZShNYXRoLlBJIC8gMiAqIDAuNSk7XG4gICAgdG1wID0gVmVjdG9yLm11bHRpcGx5KHRtcCwgdGhpcy5wYXJhbXMud2FsbERldGVjdGlvbkZlZWxlckxlbmd0aCAvIDIpO1xuICAgIHRoaXMuZmVlbGVyc1syXSA9IFZlY3Rvci5hZGQodGhpcy52ZWhpY2xlLnBvc2l0aW9uLCB0bXApO1xuICB9XG5cbiAgLy8gRkxPQ0tTXG4gIC8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS1cblxuICBzZXBhcmF0aW9uKG5laWdoYm9ycykge1xuICAgIHZhciBzdGVlcmluZyA9IG5ldyBWZWN0b3IoKTtcblxuICAgIG5laWdoYm9ycy5mb3JFYWNoKGZ1bmN0aW9uKG5laWdoYm9yLCBpbmRleCkge1xuICAgICAgdmFyIHRvQWdlbnQgPSBWZWN0b3Iuc3Vic3RyYWN0KHRoaXMudmVoaWNsZS5wb3NpdGlvbiwgbmVpZ2hib3IucG9zaXRpb24pO1xuICAgICAgLy8gc2NhbGUgdGhlIGZvcmNlIGludmVyc2VseSBwcm9wb3J0aW9ubmFsIHRvIHRoZSBhZ2VudCBkaXN0YW5jZSBmcm9tIGl0cyBuZWlnaGJvclxuICAgICAgdmFyIGRpc3RhbmNlID0gdG9BZ2VudC5tYWduaXR1ZGUoKTtcbiAgICAgIHRvQWdlbnQubm9ybWFsaXplKCkuZGl2aWRlKGRpc3RhbmNlKTsgLy8gLm11bHRpcGx5KDEwMCk7XG5cbiAgICAgIHN0ZWVyaW5nLmFkZCh0b0FnZW50KTtcbiAgICB9LCB0aGlzKTtcblxuICAgIC8vIGlmICh0aGlzLnZlaGljbGUuaXNUZXN0KSB7IGNvbnNvbGUubG9nKHN0ZWVyaW5nKTsgfVxuICAgIHJldHVybiBzdGVlcmluZztcbiAgfVxuXG4gIGFsaWdubWVudChuZWlnaGJvcnMpIHtcbiAgICB2YXIgYXZlcmFnZUhlYWRpbmcgPSBuZXcgVmVjdG9yKCk7XG4gICAgdmFyIG5laWdoYm9yQ291bnQgPSBuZWlnaGJvcnMubGVuZ3RoO1xuXG4gICAgbmVpZ2hib3JzLmZvckVhY2goZnVuY3Rpb24obmVpZ2hib3IpIHtcbiAgICAgIGF2ZXJhZ2VIZWFkaW5nLmFkZChuZWlnaGJvci5oZWFkaW5nKTtcbiAgICB9KTtcblxuICAgIGlmIChuZWlnaGJvckNvdW50ID4gMCkge1xuICAgICAgYXZlcmFnZUhlYWRpbmcuZGl2aWRlKG5laWdoYm9yQ291bnQpO1xuICAgICAgYXZlcmFnZUhlYWRpbmcuc3Vic3RyYWN0KHRoaXMudmVoaWNsZS5oZWFkaW5nKTtcbiAgICB9XG5cbiAgICByZXR1cm4gYXZlcmFnZUhlYWRpbmc7XG4gIH1cblxuICBjb2hlc2lvbihuZWlnaGJvcnMpIHtcbiAgICB2YXIgY2VudGVyT2ZNYXNzID0gbmV3IFZlY3RvcigpO1xuICAgIHZhciBzdGVlcmluZyA9IG5ldyBWZWN0b3IoKTtcbiAgICB2YXIgbmVpZ2hib3JDb3VudCA9IG5laWdoYm9ycy5sZW5ndGg7XG5cbiAgICBuZWlnaGJvcnMuZm9yRWFjaChmdW5jdGlvbihuZWlnaGJvcikge1xuICAgICAgY2VudGVyT2ZNYXNzLmFkZChuZWlnaGJvci5wb3NpdGlvbik7XG4gICAgfSk7XG5cbiAgICBpZiAobmVpZ2hib3JDb3VudCkge1xuICAgICAgY2VudGVyT2ZNYXNzLmRpdmlkZShuZWlnaGJvckNvdW50KTtcbiAgICAgIHN0ZWVyaW5nID0gdGhpcy5zZWVrKGNlbnRlck9mTWFzcyk7XG4gICAgfVxuXG4gICAgcmV0dXJuIHN0ZWVyaW5nO1xuICB9XG5cbiAgLy8gLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuICAvLyBERUJVRyBWSVNVQUxJWkFUSU9OXG4gIC8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS1cblxuICBkZWJ1Z0ZlZWxlcnMoY3R4KSB7XG4gICAgdGhpcy5mZWVsZXJzLmZvckVhY2goZnVuY3Rpb24oZmVlbGVyKSB7XG4gICAgICBjdHguc2F2ZSgpO1xuICAgICAgY3R4LmJlZ2luUGF0aCgpO1xuXG4gICAgICBjdHguc3Ryb2tlU3R5bGUgPSAnI2FjYWNhYyc7XG4gICAgICBjdHgubW92ZVRvKHRoaXMudmVoaWNsZS5wb3NpdGlvbi54LCB0aGlzLnZlaGljbGUucG9zaXRpb24ueSk7XG4gICAgICBjdHgubGluZVRvKGZlZWxlci54LCBmZWVsZXIueSk7XG4gICAgICBjdHguc3Ryb2tlKCk7XG5cbiAgICAgIGN0eC5jbG9zZVBhdGgoKTtcbiAgICAgIGN0eC5yZXN0b3JlKCk7XG4gICAgfSwgdGhpcyk7XG4gIH1cblxuICBkZWJ1Z1dhbmRlcihjdHgpIHtcbiAgICBjdHguc2F2ZSgpO1xuICAgIC8vIG1vdmUgdG8gd2FuZGVyIGNpcmNsZSBjZW50ZXJcbiAgICBjdHgudHJhbnNsYXRlKHRoaXMud2FuZGVyRGlzdGFuY2UsIDApO1xuXG4gICAgY3R4LnN0cm9rZVN0eWxlID0gJ3JlZCc7XG4gICAgY3R4LmZpbGxTdHlsZSA9ICdyZWQnO1xuICAgIC8vIHdhbmRlciBjaXJjbGVcbiAgICBjdHguYmVnaW5QYXRoKCk7XG4gICAgY3R4LmFyYygwLCAwLCB0aGlzLndhbmRlclJhZGl1cywgMCwgdXRpbHMuVHdvUEksIGZhbHNlKTtcbiAgICBjdHguc3Ryb2tlKCk7XG4gICAgY3R4LmNsb3NlUGF0aCgpO1xuICAgIC8vIHdhbmRlciB0YXJnZXRcbiAgICBjdHguYmVnaW5QYXRoKCk7XG4gICAgY3R4LmFyYyh0aGlzLndhbmRlclRhcmdldC54LCB0aGlzLndhbmRlclRhcmdldC55LCAyLCAwLCB1dGlscy5Ud29QSSwgZmFsc2UpO1xuICAgIGN0eC5zdHJva2UoKTtcbiAgICBjdHguY2xvc2VQYXRoKCk7XG5cbiAgICBjdHgucmVzdG9yZSgpO1xuICB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gU3RlZXJpbmdCZWhhdmlvcnM7XG4iXX0=