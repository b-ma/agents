'use strict';

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var Vector = require('vector');
var utils = require('./utils');

var SteeringBehaviors = (function () {
  function SteeringBehaviors(vehicle) {
    _classCallCheck(this, SteeringBehaviors);

    this.vehicle = vehicle;

    this.params = {
      minDetectionBoxLength: 40,
      wallDetectionFeelerLength: 40

    };

    // antenna used in wall avoidance
    this.feelers = [];
    // variables for wander behavior
    this.wanderRadius = 6;
    this.wanderDistance = 15;
    this.wanderJitter = 2;
    // initialize target -> circle of wanderRadius centered on the agent
    var theta = utils.rand() * utils.TwoPI;
    // project theta on the wander circle
    this.wanderTarget = new Vector(this.wanderRadius * Math.cos(theta), this.wanderRadius * Math.sin(theta));
  }

  _createClass(SteeringBehaviors, [{
    key: 'calculate',
    value: function calculate() {
      if (!this.vehicle.world.isStarted) {
        return new Vector();
      }

      var tweakers = this.vehicle.world.steeringTweakers;
      var steerings = new Vector();

      if (this._seek && this.vehicle.world.target) {
        var steering = this.seek(this.vehicle.world.target);
        steering.multiply(tweakers.seek);
        steerings.add(steering);
      }

      if (this._flee && this.vehicle.world.target) {
        var steering = this.flee(this.vehicle.world.target);
        steering.multiply(tweakers.flee);
        steerings.add(steering);
      }

      if (this._arrive && this.vehicle.world.target) {
        var steering = this.arrive(this.vehicle.world.target, 3);
        steering.multiply(tweakers.arrive);
        steerings.add(steering);
      }

      if (this._pursuit) {
        var steering = this.pursuit(this.vehicle.world.evader);
        steering.multiply(tweakers.pursuit);
        steerings.add(steering);
      }

      if (this._evade) {
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
      ctx.translate(this.vehicle.position.x, this.vehicle.position.y);
      ctx.rotate(this.vehicle.heading.direction());

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
//# sourceMappingURL=data:application/json;base64,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QyxFQUFFLENBQUMsQ0FBQyxDQUFBLElBQUcsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLEFBQUMsQ0FBQzs7QUFFM0QsWUFBSSxJQUFJLEdBQUcsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsSUFBRyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsQUFBQyxHQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLElBQUcsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLEFBQUMsQ0FBQztBQUMzRCxZQUFJLElBQUksR0FBRyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxJQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxBQUFDLEdBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsSUFBRyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsQUFBQyxDQUFDOztBQUUzRCxZQUFLLEFBQUMsSUFBSSxLQUFLLENBQUMsSUFBTSxJQUFJLEtBQUssQ0FBQyxBQUFDLEVBQUc7QUFDbEMsaUJBQU8sS0FBSyxDQUFDO1NBQ2Q7O0FBRUQsWUFBSSxDQUFDLEdBQUcsSUFBSSxHQUFDLElBQUksQ0FBQztBQUNsQixZQUFJLENBQUMsR0FBRyxJQUFJLEdBQUMsSUFBSSxDQUFDOztBQUVsQixZQUFLLEFBQUMsQ0FBQyxHQUFHLENBQUMsSUFBTSxDQUFDLEdBQUcsQ0FBQyxBQUFDLElBQUssQ0FBQyxHQUFHLENBQUMsQUFBQyxJQUFLLENBQUMsR0FBRyxDQUFDLEFBQUMsRUFBRztBQUM5QyxrQkFBUSxHQUFHLE1BQU0sQ0FBQyxRQUFRLENBQUMsRUFBRSxFQUFFLEVBQUUsQ0FBQyxHQUFHLENBQUMsQ0FBQzs7QUFFdkMsZUFBSyxHQUFHLElBQUksTUFB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TyxDQUFDLFNBQVMsRUFBRSxDQUFDLENBQUM7OztBQUc3QyxTQUFHLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxjQUFjLEVBQUUsQ0FBQyxDQUFDLENBQUM7O0FBRXRDLFNBQUcsQ0FBQyxXQUFXLEdBQUcsS0FBSyxDQUFDO0FBQ3hCLFNBQUcsQ0FBQyxTQUFTLEdBQUcsS0FBSyxDQUFDOztBQUV0QixTQUFHLENBQUMsU0FBUyxFQUFFLENBQUM7QUFDaEIsU0FBRyxDQUFDLEdBQUcsQ0FBQyxDQUFDLEVBQUUsQ0FBQyxFQUFFLElBQUksQ0FBQyxZQUFZLEVBQUUsQ0FBQyxFQUFFLEtBQUssQ0FBQyxLQUFLLEVBQUUsS0FBSyxDQUFDLENBQUM7QUFDeEQsU0FBRyxDQUFDLE1BQU0sRUFBRSxDQUFDO0FBQ2IsU0FBRyxDQUFDLFNBQVMsRUFBRSxDQUFDOztBQUVoQixTQUFHLENBQUMsU0FBUyxFQUFFLENBQUM7QUFDaEIsU0FBRyxDQUFDLEdBQUcsQ0FBQyxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUMsRUFBRSxJQUFJLENBQUMsWUFBWSxDQUFDLENBQUMsRUFBRSxDQUFDLEVBQUUsQ0FBQyxFQUFFLEtBQUssQ0FBQyxLQUFLLEVBQUUsS0FBSyxDQUFDLENBQUM7QUFDNUUsU0FBRyxDQUFDLE1BQU0sRUFBRSxDQUFDO0FBQ2IsU0FBRyxDQUFDLFNBQVMsRUFBRSxDQUFDOztBQUVoQixTQUFHLENBQUMsT0FBTyxFQUFFLENBQUM7S0FDZjs7O1NBcGVHLGlCQUFpQjs7O0FBdWV2QixNQUFNLENBQUMsT0FBTyxHQUFHLGlCQUFpQixDQUFDIiwiZmlsZSI6InNyYy92ZWhpY2xlLmVzNi5qcyIsInNvdXJjZXNDb250ZW50IjpbInZhciBWZWN0b3IgPSByZXF1aXJlKCd2ZWN0b3InKTtcbnZhciB1dGlscyAgPSByZXF1aXJlKCcuL3V0aWxzJyk7XG5cbmNsYXNzIFN0ZWVyaW5nQmVoYXZpb3JzIHtcbiAgY29uc3RydWN0b3IodmVoaWNsZSkge1xuICAgIHRoaXMudmVoaWNsZSA9IHZlaGljbGU7XG5cbiAgICB0aGlzLnBhcmFtcyA9IHtcbiAgICAgIG1pbkRldGVjdGlvbkJveExlbmd0aDogNDAsXG4gICAgICB3YWxsRGV0ZWN0aW9uRmVlbGVyTGVuZ3RoOiA0MFxuXG4gICAgfTtcblxuICAgIC8vIGFudGVubmEgdXNlZCBpbiB3YWxsIGF2b2lkYW5jZVxuICAgIHRoaXMuZmVlbGVycyA9IFtdO1xuICAgIC8vIHZhcmlhYmxlcyBmb3Igd2FuZGVyIGJlaGF2aW9yXG4gICAgdGhpcy53YW5kZXJSYWRpdXMgPSA2O1xuICAgIHRoaXMud2FuZGVyRGlzdGFuY2UgPSAxNTtcbiAgICB0aGlzLndhbmRlckppdHRlciA9IDI7XG4gICAgLy8gaW5pdGlhbGl6ZSB0YXJnZXQgLT4gY2lyY2xlIG9mIHdhbmRlclJhZGl1cyBjZW50ZXJlZCBvbiB0aGUgYWdlbnRcbiAgICB2YXIgdGhldGEgPSB1dGlscy5yYW5kKCkgKiB1dGlscy5Ud29QSTtcbiAgICAvLyBwcm9qZWN0IHRoZXRhIG9uIHRoZSB3YW5kZXIgY2lyY2xlXG4gICAgdGhpcy53YW5kZXJUYXJnZXQgPSBuZXcgVmVjdG9yKHRoaXMud2FuZGVyUmFkaXVzICogTWF0aC5jb3ModGhldGEpLFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLndhbmRlclJhZGl1cyAqIE1hdGguc2luKHRoZXRhKSk7XG5cbiAgfVxuXG4gIGNhbGN1bGF0ZSgpIHtcbiAgICBpZiAoIXRoaXMudmVoaWNsZS53b3JsZC5pc1N0YXJ0ZWQpIHtcbiAgICAgIHJldHVybiBuZXcgVmVjdG9yKCk7XG4gICAgfVxuXG4gICAgdmFyIHR3ZWFrZXJzID0gdGhpcy52ZWhpY2xlLndvcmxkLnN0ZWVyaW5nVHdlYWtlcnM7XG4gICAgdmFyIHN0ZWVyaW5ncyA9IG5ldyBWZWN0b3IoKTtcblxuICAgIGlmICh0aGlzLl9zZWVrICYmIHRoaXMudmVoaWNsZS53b3JsZC50YXJnZXQpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMuc2Vlayh0aGlzLnZlaGljbGUud29ybGQudGFyZ2V0KVxuICAgICAgc3RlZXJpbmcubXVsdGlwbHkodHdlYWtlcnMuc2Vlayk7XG4gICAgICBzdGVlcmluZ3MuYWRkKHN0ZWVyaW5nKTtcbiAgICB9XG5cbiAgICBpZiAodGhpcy5fZmxlZSAmJiB0aGlzLnZlaGljbGUud29ybGQudGFyZ2V0KSB7XG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLmZsZWUodGhpcy52ZWhpY2xlLndvcmxkLnRhcmdldClcbiAgICAgIHN0ZWVyaW5nLm11bHRpcGx5KHR3ZWFrZXJzLmZsZWUpO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgaWYgKHRoaXMuX2Fycml2ZSAmJiB0aGlzLnZlaGljbGUud29ybGQudGFyZ2V0KSB7XG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLmFycml2ZSh0aGlzLnZlaGljbGUud29ybGQudGFyZ2V0LCAzKVxuICAgICAgc3RlZXJpbmcubXVsdGlwbHkodHdlYWtlcnMuYXJyaXZlKTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIGlmICh0aGlzLl9wdXJzdWl0KSB7XG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLnB1cnN1aXQodGhpcy52ZWhpY2xlLndvcmxkLmV2YWRlcilcbiAgICAgIHN0ZWVyaW5nLm11bHRpcGx5KHR3ZWFrZXJzLnB1cnN1aXQpO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgaWYgKHRoaXMuIF9ldmFkZSkge1xuICAgICAgdmFyIHN0ZWVyaW5nID0gdGhpcy5ldmFkZSh0aGlzLnZlaGljbGUud29ybGQucHVyc3VlcilcbiAgICAgIHN0ZWVyaW5nLm11bHRpcGx5KHR3ZWFrZXJzLmV2YWRlKTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIGlmICh0aGlzLl93YW5kZXIpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMud2FuZGVyKClcbiAgICAgIHN0ZWVyaW5nLm11bHRpcGx5KHR3ZWFrZXJzLndhbmRlcik7XG4gICAgICBzdGVlcmluZ3MuYWRkKHN0ZWVyaW5nKTtcbiAgICB9XG5cbiAgICAvLyBkb2Vzbid0IHdvcmsuLi5cbiAgICBpZiAodGhpcy5fb2JzdGFjbGVBdm9pZGFuY2UpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMub2JzdGFjbGVBdm9pZGFuY2UodGhpcy52ZWhpY2xlLndvcmxkLm9ic3RhY2xlcylcbiAgICAgIHN0ZWVyaW5nLm11bHRpcGx5KHR3ZWFrZXJzLm9ic3RhY2xlQXZvaWRhbmNlKTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIC8vIGRvZXNuJ3Qgd29yayB3ZWxsXG4gICAgaWYgKHRoaXMuX3dhbGxBdm9pZGFuY2UpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMud2FsbEF2b2lkYW5jZSh0aGlzLnZlaGljbGUud29ybGQud2FsbHMpXG4gICAgICBzdGVlcmluZy5tdWx0aXBseSh0d2Vha2Vycy53YWxsQXZvaWRhbmNlKTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIGlmICh0aGlzLl9mbG9jaykge1xuICAgICAgLy8gdGFnIG5laWdoYm9yc1xuICAgICAgdGhpcy52ZWhpY2xlLndvcmxkLnRhZ0JvaWRzV2l0aGluUmFuZ2UodGhpcy52ZWhpY2xlKTtcblxuICAgICAgdmFyIGZsb2NrcyA9IG5ldyBWZWN0b3IoKTtcbiAgICAgIGZsb2Nrcy5hZGQodGhpcy5zZXBhcmF0aW9uKHRoaXMudmVoaWNsZS5uZWlnaGJvcnMpLm11bHRpcGx5KHR3ZWFrZXJzLnNlcGFyYXRpb24pKTtcbiAgICAgIGZsb2Nrcy5hZGQodGhpcy5hbGlnbm1lbnQodGhpcy52ZWhpY2xlLm5laWdoYm9ycykubXVsdGlwbHkodHdlYWtlcnMuYWxpZ25tZW50KSk7XG4gICAgICBmbG9ja3MuYWRkKHRoaXMuY29oZXNpb24odGhpcy52ZWhpY2xlLm5laWdoYm9ycykubXVsdGlwbHkodHdlYWtlcnMuY29oZXNpb24pKTtcblxuICAgICAgc3RlZXJpbmdzLmFkZChmbG9ja3MpO1xuICAgIH1cblxuXG4gICAgcmV0dXJuIHN0ZWVyaW5ncztcbiAgfVxuXG4gIC8vIGJlaGF2aW9ycyBPbi9PZmZcbiAgLy8gLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS1cblxuICBzZWVrT24oKSB7IHRoaXMuX3NlZWsgPSB0cnVlOyB9XG4gIHNlZWtPZmYoKSB7IHRoaXMuX3NlZWsgPSBmYWxzZTsgfVxuXG4gIGZsZWVPbigpIHsgdGhpcy5fZmxlZSA9IHRydWU7IH1cbiAgZmxlZU9mZigpIHsgdGhpcy5fZmxlZSA9IGZhbHNlOyB9XG5cbiAgYXJyaXZlT24oKSB7IHRoaXMuX2Fycml2ZSA9IHRydWU7IH1cbiAgYXJyaXZlT2ZmKCkgeyB0aGlzLl9hcnJpdmUgPSBmYWxzZTsgfVxuXG4gIHB1cnN1aXRPbigpIHsgdGhpcy5fcHVyc3VpdCA9IHRydWU7IH1cbiAgcHVyc3VpdE9mZigpIHsgdGhpcy5fcHVyc3VpdCA9IGZhbHNlOyB9XG5cbiAgZXZhZGVPbigpIHsgdGhpcy5fZXZhZGUgPSB0cnVlOyB9XG4gIGV2YWRlT2ZmKCkgeyB0aGlzLl9ldmFkZSA9IGZhbHNlOyB9XG5cbiAgd2FuZGVyT24oKSB7IHRoaXMuX3dhbmRlciA9IHRydWU7IH1cbiAgd2FuZGVyT2ZmKCkgeyB0aGlzLl93YW5kZXIgPSBmYWxzZTsgfVxuXG4gIG9ic3RhY2xlQXZvaWRhbmNlT24oKSB7IHRoaXMuX29ic3RhY2xlQXZvaWRhbmNlID0gdHJ1ZTsgfVxuICBvYnN0YWNsZUF2b2lkYW5jZU9mZigpIHsgdGhpcy5fb2JzdGFjbGVBdm9pZGFuY2UgPSBmYWxzZTsgfVxuXG4gIHdhbGxBdm9pZGFuY2VPbigpIHsgdGhpcy5fd2FsbEF2b2lkYW5jZSA9IHRydWU7IH1cbiAgd2FsbEF2b2lkYW5jZU9mZigpIHsgdGhpcy5fd2FsbEF2b2lkYW5jZSA9IGZhbHNlOyB9XG5cbiAgZmxvY2tPbigpIHsgdGhpcy5fZmxvY2sgPSB0cnVlOyB9XG4gIGZsb2NrT2ZmKCkgeyB0aGlzLl9mbG9jayA9IGZhbHNlOyB9XG5cbiAgLy8gYmVoYXZpb3JzXG4gIC8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG5cbiAgc2Vlayh0YXJnZXRQb3NpdGlvbikge1xuICAgIHZhciBkZXNpcmVkVmVsb2NpdHkgPSBWZWN0b3Iuc3Vic3RyYWN0KHRhcmdldFBvc2l0aW9uLCB0aGlzLnZlaGljbGUucG9zaXRpb24pXG4gICAgICAubm9ybWFsaXplKClcbiAgICAgIC5tdWx0aXBseSh0aGlzLnZlaGljbGUubWF4U3BlZWQpO1xuXG4gICAgdmFyIHN0ZWVyaW5nID0gVmVjdG9yLnN1YnN0cmFjdChkZXNpcmVkVmVsb2NpdHksIHRoaXMudmVoaWNsZS52ZWxvY2l0eSk7XG4gICAgcmV0dXJuIHN0ZWVyaW5nO1xuICB9XG5cbiAgZmxlZSh0YXJnZXRQb3NpdGlvbikge1xuICAgIHZhciBwYW5pY0Rpc3RhbmNlID0gMTAwICogMTAwOyAvLyB1c2Ugc3F1YXJlIGRvbWFpbiB0byBzYXZlIGNvbXB1dGF0aW9uc1xuICAgIGlmIChWZWN0b3IuZGlzdGFuY2VTcXJ0KHRoaXMudmVoaWNsZS5wb3NpdGlvbiwgdGFyZ2V0UG9zaXRpb24pID4gcGFuaWNEaXN0YW5jZSkge1xuICAgICAgcmV0dXJuIG5ldyBWZWN0b3IoKTtcbiAgICB9XG5cbiAgICB2YXIgZGVzaXJlZFZlbG9jaXR5ID0gVmVjdG9yLnN1YnN0cmFjdCh0aGlzLnZlaGljbGUucG9zaXRpb24sIHRhcmdldFBvc2l0aW9uKVxuICAgICAgLm5vcm1hbGl6ZSgpXG4gICAgICAubXVsdGlwbHkodGhpcy52ZWhpY2xlLm1heFNwZWVkKTtcblxuICAgIHZhciBzdGVlcmluZyA9IFZlY3Rvci5zdWJzdHJhY3QoZGVzaXJlZFZlbG9jaXR5LCB0aGlzLnZlaGljbGUudmVsb2NpdHkpO1xuICAgIHJldHVybiBzdGVlcmluZztcbiAgfVxuXG4gIC8vIGRlY2VsZXJhdGlvbiBpcyBhbiBlbnVtIChzbG93ID0gMSwgbm9ybWFsID0gMiwgZmFzdCA9IDMpXG4gIC8vIGl0IGRlc2NyaWJlcyB0aGUgdGltZSB0aGUgYWdlbnQgc2hvdWxkIHRha2UgdG8gYXJyaXZlIGF0IGRlc3RpbmF0aW9uXG4gIGFycml2ZSh0YXJnZXRQb3NpdGlvbiwgZGVjZWxlcmF0aW9uKSB7XG4gICAgdmFyIHRvVGFyZ2V0ID0gVmVjdG9yLnN1YnN0cmFjdCh0YXJnZXRQb3NpdGlvbiwgdGhpcy52ZWhpY2xlLnBvc2l0aW9uKTtcbiAgICB2YXIgZGlzdGFuY2UgPSB0b1RhcmdldC5tYWduaXR1ZGUoKTtcblxuICAgIGlmIChkaXN0YW5jZSA+IDApIHtcbiAgICAgIC8vIGFsbG93IHRvIHR3ZWFrIGRlY2VsZXJhdGlvblxuICAgICAgdmFyIGRlY2VsZXJhdGlvblR3ZWFrZXIgPSAwLjM7XG4gICAgICAvLyBkZWZpbmUgdGhlIHNwZWVkIHRoZSBhZ2VudCBzaG91bGQgaGF2ZSB0byBhcnJpdmUgYXQgZGVzdGluYXRpb25cbiAgICAgIHZhciBzcGVlZCA9IGRpc3RhbmNlIC8gKGRlY2VsZXJhdGlvbiAqIGRlY2VsZXJhdGlvblR3ZWFrZXIpO1xuICAgICAgLy8gc3BlZWQgc2hvdWxkbid0IGV4Y2VlZCBtYXhTcGVlZFxuICAgICAgc3BlZWQgPSBNYXRoLm1pbihzcGVlZCwgdGhpcy52ZWhpY2xlLm1heFNwZWVkKTtcbiAgICAgIC8vIG5leHQgc3RlcHMgYXJlIHNhbWUgYXMgc2Vla1xuICAgICAgdmFyIGRlc2lyZWRWZWxvY2l0eSA9IHRvVGFyZ2V0XG4gICAgICAgIC5kaXZpZGUoZGlzdGFuY2UpIC8vIDw9PiB0b1RhcmdldC5ub3JtYWxpemUoKTtcbiAgICAgICAgLm11bHRpcGx5KHNwZWVkKTtcblxuICAgICAgdmFyIHN0ZWVyaW5nID0gVmVjdG9yLnN1YnN0cmFjdChkZXNpcmVkVmVsb2NpdHksIHRoaXMudmVoaWNsZS52ZWxvY2l0eSk7XG4gICAgICByZXR1cm4gc3RlZXJpbmc7XG4gICAgfVxuXG4gICAgcmV0dXJuIG5ldyBWZWN0b3IoKTtcbiAgfVxuXG4gIHB1cnN1aXQoZXZhZGVyKSB7XG4gICAgdmFyIHRvRXZhZGVyID0gVmVjdG9yLnN1YnN0cmFjdChldmFkZXIucG9zaXRpb24sIHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gICAgLy8gY29zaW5lIG9mIHRoZSBhbmdsZSBiZXR3ZWVuIHRoZSAyIGFnZW50cyBoZWFkaW5nc1xuICAgIHZhciByZWxhdGl2ZUhlYWRpbmcgPSBWZWN0b3IuZG90KGV2YWRlci5oZWFkaW5nLCB0aGlzLnZlaGljbGUuaGVhZGluZyk7XG5cbiAgICBpZiAoXG4gICAgICAvLyBpZiB0aGUgdGhlIGV2YWRlciBpcyBpbiBmcm9udCBvZiB0aGUgcHVyc3VlclxuICAgICAgKFZlY3Rvci5kb3QodGhpcy52ZWhpY2xlLmhlYWRpbmcsIHRvRXZhZGVyKSA+IDApICYmXG4gICAgICAvLyBhbmQgdGhlIHR3byBhZ2VudHMgYXJlIGFwcHJveCBmYWNlIHRvIGZhY2VcbiAgICAgIChyZWxhdGl2ZUhlYWRpbmcgPCAtMC45NSkgLy8gYWNvcygwLjk1KSA9IDE4ZGVnXG4gICAgKSB7XG4gICAgICByZXR1cm4gdGhpcy5zZWVrKGV2YWRlci5wb3NpdGlvbik7XG4gICAgfVxuXG4gICAgLy8gbm90IGNvbnNpcmRlcmVkIGFoZWFkIHNvIHdlIGhhdmUgdG9cbiAgICAvLyBwcmVkaWN0IHRoZSBmdXR1cmUgcG9zaXRpb24gb2YgdGhlIGV2YWRlclxuXG4gICAgLy8gdGhlIGxvb2tBaGVhZFRpbWUgc2hvdWxkIGJlIHByb3BvcnRpb25uYWwgdG8gdGhlIGRpc3RhbmNlLFxuICAgIC8vIGFuZCBpbnZlcnNseSBwcm9wb3J0aW9ubmFsIHRvIHRoZSBzcGVlZCBvZiB0aGUgYWdlbnRzXG4gICAgdmFyIGxvb2tBaGVhZFRpbWUgPSB0b0V2YWRlci5tYWduaXR1ZGUoKSAvICh0aGlzLnZlaGljbGUubWF4U3BlZWQgKyBldmFkZXIuc3BlZWQoKSk7XG4gICAgLy8gbm93IHNlZWsgdG8gYSBwcmVkaWN0aW9uIG9mIHRoZSBldmFkZXIgcG9zaXRpb25cbiAgICB2YXIgcHJlZGljdGVkUG9zaXRpb24gPSBWZWN0b3IuYWRkKGV2YWRlci5wb3NpdGlvbiwgVmVjdG9yLm11bHRpcGx5KGV2YWRlci52ZWxvY2l0eSwgbG9va0FoZWFkVGltZSkpO1xuICAgIHJldHVybiB0aGlzLnNlZWsocHJlZGljdGVkUG9zaXRpb24pO1xuICB9XG5cbiAgZXZhZGUocHVyc3Vlcikge1xuICAgIC8vIG5vIG5lZWQgdG8gY2hlY2sgaWYgdGhlIGFnZW50cyBhcmUgZmFjaW5nXG4gICAgdmFyIHRvUHVyc3VlciA9IFZlY3Rvci5zdWJzdHJhY3QocHVyc3Vlci5wb3NpdGlvbiwgdGhpcy52ZWhpY2xlLnBvc2l0aW9uKTtcbiAgICAvLyB0aGVuIHNhbWUgYXMgcHVyc3VpdCBidXQgcmVwbGFjZSBzZWVrIHdpdGggZmxlZVxuICAgIHZhciBsb29rQWhlYWRUaW1lID0gdG9QdXJzdWVyLm1hZ25pdHVkZSgpIC8gKHRoaXMudmVoaWNsZS5tYXhTcGVlZCArIHB1cnN1ZXIuc3BlZWQoKSk7XG4gICAgdmFyIHByZWRpY3RlZFBvc2l0aW9uID0gVmVjdG9yLmFkZChwdXJzdWVyLnBvc2l0aW9uLCBWZWN0b3IubXVsdGlwbHkocHVyc3Vlci52ZWxvY2l0eSwgbG9va0FoZWFkVGltZSkpO1xuICAgIHJldHVybiB0aGlzLmZsZWUocHJlZGljdGVkUG9zaXRpb24pO1xuICB9XG5cbiAgd2FuZGVyKCkge1xuICAgIC8vIGFkZCBhIHNtYWxsIHJhbmRvbSB2ZWN0b3IgdG8gdGhlIHdhbmRlclRhcmdldFxuICAgIHZhciByYW5kb21WZWN0b3IgPSBuZXcgVmVjdG9yKHV0aWxzLnJhbmRDbGFtcGVkKCkgKiB0aGlzLndhbmRlckppdHRlcixcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICB1dGlscy5yYW5kQ2xhbXBlZCgpICogdGhpcy53YW5kZXJKaXR0ZXIpO1xuICAgIHRoaXMud2FuZGVyVGFyZ2V0LmFkZChyYW5kb21WZWN0b3IpO1xuICAgIC8vIHJlcHJvamVjdCB0aGUgd2FuZGVyVGFyZ2V0IG9uIHRoZSB3YW5kZXIgY2lyY2xlXG4gICAgdGhpcy53YW5kZXJUYXJnZXRcbiAgICAgIC5ub3JtYWxpemUoKVxuICAgICAgLm11bHRpcGx5KHRoaXMud2FuZGVyUmFkaXVzKTtcbiAgICAvLyBwcm9qZWN0IHRoZSB3YW5kZXIgY2lyY2xlIGluIGZyb250IG9mIHRoZSBhZ2VudFxuICAgIHZhciB0YXJnZXRMb2NhbCA9IFZlY3Rvci5hZGQodGhpcy53YW5kZXJUYXJnZXQsIG5ldyBWZWN0b3IodGhpcy53YW5kZXJEaXN0YW5jZSwgMCkpO1xuICAgIC8vIEBUT0RPIHNob3VsZCBiZSBhbiBtYXRyaXggdHJhbnNmb3JtcyB1dGlsc1xuICAgIC8vIHByb2plY3QgdGhlIHRhcmdldCBpbiB3b3JsZCBzcGFjZSAtIGNoYW5nZSBuYW1lIGZvciB1bmRlcnN0YW5kYWJpbGl0eVxuICAgIHZhciB0YXJnZXRXb3JsZCA9IHRhcmdldExvY2FsO1xuICAgIC8vIHJvdGF0ZVxuICAgIHRhcmdldFdvcmxkLnJvdGF0ZSh0aGlzLnZlaGljbGUuaGVhZGluZy5kaXJlY3Rpb24oKSk7XG4gICAgLy8gdHJhbnNsYXRlXG4gICAgdGFyZ2V0V29ybGQuYWRkKHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gICAgLy8gc3RlZXIgdG93YXJkIHRoaXMgdGFyZ2V0XG4gICAgcmV0dXJuIFZlY3Rvci5zdWJzdHJhY3QodGFyZ2V0V29ybGQsIHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gIH1cblxuICAvLyBwcmV2ZW50IHRoZSBhZ2VudCBjb2xsaWRpbmcgd2l0aCB0aGUgY2xvc2VzdCBvYnN0YWNsZVxuICAvLyBATk9URSAtIG5vdCB0ZXN0ZWQgcHJvcGVybHlcbiAgb2JzdGFjbGVBdm9pZGFuY2Uob2JzdGFjbGVzKSB7XG4gICAgLy8gY3JlYXRlIGEgZGV0ZWN0aW9uIGJveCBwcm9wb3J0aW9ubmFsIHRvIHRoZSBhZ2VudCB2ZWxvY2l0eVxuICAgIHZhciBib3hMZW5ndGggPSB0aGlzLnBhcmFtcy5taW5EZXRlY3Rpb25Cb3hMZW5ndGggK1xuICAgICAgICAgICAgICAgICAgICB0aGlzLnZlaGljbGUuc3BlZWQoKSAvIHRoaXMudmVoaWNsZS5tYXhTcGVlZCAqXG4gICAgICAgICAgICAgICAgICAgIHRoaXMucGFyYW1zLm1pbkRldGVjdGlvbkJveExlbmd0aDtcblxuICAgIHRoaXMudmVoaWNsZS53b3JsZC50YWdPYnN0YWNsZXNXaXRoaW5SYW5nZSh0aGlzLnZlaGljbGUsIGJveExlbmd0aCk7XG4gICAgLy8gY2xvc2VzdCBpbnRlcnNlY3Rpbmcgb2JzdGFjbGUgKENJTylcbiAgICB2YXIgY2xvc2VzdEludGVyY2VwdGluZ09ic3RhY2xlID0gbnVsbDtcbiAgICB2YXIgZGlzdGFuY2VUb0NJTyA9ICtJbmZpbml0eTtcbiAgICB2YXIgbG9jYWxQb3NpdGlvbk9mQ0lPID0gbnVsbDtcblxuICAgIG9ic3RhY2xlcy5mb3JFYWNoKGZ1bmN0aW9uKG9ic3RhY2xlKSB7XG4gICAgICBpZiAoIW9ic3RhY2xlLmlzVGFnZ2VkKCkpIHsgcmV0dXJuOyB9XG4gICAgICAvLyBmaW5kIGxvY2FsIGNvb3JkaW5hdGVzIG9mIHRoZSBvYnN0YWNsZVxuICAgICAgdmFyIGxvY2FsUG9zID0gb2JzdGFjbGUucG9zaXRpb24uY2xvbmUoKTtcbiAgICAgIC8vIHJvdGF0ZVxuICAgICAgbG9jYWxQb3Mucm90YXRlKHRoaXMudmVoaWNsZS5oZWFkaW5nLmRpcmVjdGlvbigpICogLTEpO1xuICAgICAgLy8gdHJhbnNsYXRlXG4gICAgICBsb2NhbFBvcy5zdWJzdHJhY3QodGhpcy52ZWhpY2xlLnBvc2l0aW9uKTtcbiAgICAgIC8vIGlmIHRoZSBsb2NhbCB4IHZhbHVlIGlzIG5lZ2F0aXZlLCB0aGUgb2JzdGFjbGUgaXMgYmVoaW5kIHRoZSBhZ2VudFxuICAgICAgaWYgKGxvY2FsUG9zLnggPCAwKSB7IHJldHVybjsgfVxuXG4gICAgICAvLyBpZiB0aGUgZGlzdGFuY2UgYmV0d2VlbiB0aGUgeCBheGlzIHRvIHRoZSBvamVjdCBsb2NhbCBwb3NpdGlvbiBpcyBsZXNzIHRoYW5cbiAgICAgIC8vIGl0cyByYWRpdXMgKyB0aGUgcmFkaXVzIG9mIHRoaXMudmVoaWNsZSwgdGhlcmUgaXMgYSBwb3NzaWJsZSBjb2xsaXNpb25cbiAgICAgIHZhciBleHBhbmRlZFJhZGl1cyA9IG9ic3RhY2xlLmJvdW5kaW5nUmFkaXVzICsgdGhpcy52ZWhpY2xlLmJvdW5kaW5nUmFkaXVzO1xuXG4gICAgICBpZiAobG9jYWxQb3MueSA+IGV4cGFuZGVkUmFkaXVzKSB7IHJldHVybjsgfVxuXG4gICAgICAvLyBub3cgdG8gZG8gYSBsaW5lL2NpcmNsZSBpbnRlcnNlY3Rpb24gdGVzdC4gVGhlIGNlbnRlciBvZiB0aGVcbiAgICAgIC8vIGNpcmNsZSBpcyByZXByZXNlbnRlZCBieSAoY1gsIGNZKS4gVGhlIGludGVyc2VjdGlvbiBwb2ludHMgYXJlXG4gICAgICAvLyBnaXZlbiBieSB0aGUgZm9ybXVsYSB4ID0gY1ggKy8tc3FydChyXjItY1leMikgZm9yIHk9MC5cbiAgICAgIC8vIFdlIG9ubHkgbmVlZCB0byBsb29rIGF0IHRoZSBzbWFsbGVzdCBwb3NpdGl2ZSB2YWx1ZSBvZiB4IGJlY2F1c2VcbiAgICAgIC8vIHRoYXQgd2lsbCBiZSB0aGUgY2xvc2VzdCBwb2ludCBvZiBpbnRlcnNlY3Rpb24uXG4gICAgICB2YXIgY1ggPSBsb2NhbFBvcy54O1xuICAgICAgdmFyIGNZID0gbG9jYWxQb3MueTtcblxuICAgICAgLy8gY2FsY3VsZSB0aGUgc3FydCBwYXJ0IG9mIHRoZSBlcXVhdGlvbiBvbmx5IG9uY2VcbiAgICAgIHZhciBzcXJ0UGFydCA9IE1hdGguc3FydChleHBhbmRlZFJhZGl1cyAqIGV4cGFuZGVkUmFkaXVzIC0gY1kgKiBjWSk7XG5cbiAgICAgIHZhciBpcCA9IGNYIC0gc3FydFBhcnQ7XG4gICAgICBpZiAoaXAgPCAwKSB7IGlwID0gY1ggKyBzcXJ0UGFydDsgfVxuXG4gICAgICAvLyBpZiBjbG9zZXN0IHNvIGZhciAtIHN0b3JlIGFsbCBpdHMgdmFsdWVcbiAgICAgIGlmIChpcCA8IGRpc3RhbmNlVG9DSU8pIHtcbiAgICAgICAgZGlzdGFuY2VUb0NJTyA9IGlwO1xuICAgICAgICBjbG9zZXN0SW50ZXJjZXB0aW5nT2JzdGFjbGUgPSBvYnN0YWNsZTtcbiAgICAgICAgbG9jYWxQb3NpdGlvbk9mQ0lPID0gbG9jYWxQb3M7XG4gICAgICB9XG4gICAgfSwgdGhpcyk7XG5cbiAgICAvLyBzdGlsbCBpbiBsb2NhbCBzcGFjZVxuICAgIHZhciBzdGVlcmluZyA9IG5ldyBWZWN0b3IoKTtcbiAgICAvLyBpZiB3ZSBmb3VuZCBzb21lIG9idGFjbGUsIGNhbGN1bGF0ZSBhIHN0ZWVyaW5nIGZvcmNlIGF3YXkgZnJvbSBpdFxuICAgIGlmIChjbG9zZXN0SW50ZXJjZXB0aW5nT2JzdGFjbGUpIHtcbiAgICAgIC8vIHRoZSBjbG9zZXIgYW4gYWdlbnQgaXMgdG8gYW4gb2JqZWN0LCB0aGUgc3Ryb25nZXIgdGhlIHN0ZWVyaW5nIChiZXR3ZWVuIDEgYW5kIDIgWz9dKVxuICAgICAgdmFyIG11bHRpcGxpZXIgPSAxICsgKGJveExlbmd0aCAtIGxvY2FsUG9zaXRpb25PZkNJTy54KSAvIGJveExlbmd0aDtcblxuICAgICAgc3RlZXJpbmcueSA9IChjbG9zZXN0SW50ZXJjZXB0aW5nT2JzdGFjbGUuYm91bmRpbmdSYWRpdXMgLVxuICAgICAgICAgICAgICAgICAgICBsb2NhbFBvc2l0aW9uT2ZDSU8ueSkgKiBtdWx0aXBsaWVyO1xuXG4gICAgICB2YXIgYnJha2luZ1dlaWdodCA9IDAuMjtcblxuICAgICAgc3RlZXJpbmcueCA9IChjbG9zZXN0SW50ZXJjZXB0aW5nT2JzdGFjbGUuYm91bmRpbmdSYWRpdXMgLVxuICAgICAgICAgICAgICAgICAgICBsb2NhbFBvc2l0aW9uT2ZDSU8ueCkgKiBicmFraW5nV2VpZ2h0O1xuICAgIH1cblxuICAgIC8vIHJvdGF0ZSB0byBnbyBiYWNrIHRvIHdvcmxkIHNwYWNlXG4gICAgc3RlZXJpbmcucm90YXRlKHRoaXMudmVoaWNsZS5oZWFkaW5nLmRpcmVjdGlvbigpKTtcbiAgICByZXR1cm4gc3RlZXJpbmc7XG4gIH1cblxuICB3YWxsQXZvaWRhbmNlKHdhbGxzKSB7XG4gICAgdGhpcy5jcmVhdGVGZWVsZXJzKCk7XG5cbiAgICAvLyBJUDogSW50ZXJzZWN0aW9uIFBvaW50XG4gICAgdmFyIGRpc3RhbmNlVG9DbG9zZXN0SVAgPSArSW5maW5pdHk7XG4gICAgdmFyIGNsb3Nlc3RQb2ludCA9IG51bGw7XG4gICAgdmFyIGNsb3Nlc3RXYWxsID0gbnVsbDtcblxuICAgIHZhciBzdGVlcmluZyA9IG5ldyBWZWN0b3IoKTtcbiAgICB2YXIgZGlzdGFuY2UsIHBvaW50O1xuXG4gICAgLy8gZ2VvbWV0cnkuaCAobGluZSAyODQpIExpbmVJbnRlcnNlY3Rpb24yZCAoPz8/IHdoYXQgYXBwZW5kcyBoZXJlID8/PylcbiAgICB2YXIgaW50ZXJzZWN0ID0gZnVuY3Rpb24oZjEsIHQxLCBmMiwgdDIpIHtcbiAgICAgIHZhciByVG9wID0gKGYxLnktZjIueSkqKHQyLngtZjIueCktKGYxLngtZjIueCkqKHQyLnktZjIueSk7XG4gICAgICB2YXIgckJvdCA9ICh0MS54LWYxLngpKih0Mi55LWYyLnkpLSh0MS55LWYxLnkpKih0Mi54LWYyLngpO1xuXG4gICAgICB2YXIgc1RvcCA9IChmMS55LWYyLnkpKih0MS54LWYxLngpLShmMS54LWYyLngpKih0MS55LWYxLnkpO1xuICAgICAgdmFyIHNCb3QgPSAodDEueC1mMS54KSoodDIueS1mMi55KS0odDEueS1mMS55KSoodDIueC1mMi54KTtcblxuICAgICAgaWYgKCAockJvdCA9PT0gMCkgfHzCoChzQm90ID09PSAwKSApIHtcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgfVxuXG4gICAgICB2YXIgciA9IHJUb3AvckJvdDtcbiAgICAgIHZhciBzID0gc1RvcC9zQm90O1xuXG4gICAgICBpZiAoIChyID4gMCkgJiYgKHIgPCAxKSAmJiAocyA+IDApICYmIChzIDwgMSkgKSB7XG4gICAgICAgIGRpc3RhbmNlID0gVmVjdG9yLmRpc3RhbmNlKGYxLCB0MSkgKiByO1xuICAgICAgICAvLyBjb25zb2xlLmxvZyhyLCBkaXN0YW5jZSk7XG4gICAgICAgIHBvaW50ID0gbmV3IFZlY3RvcihmMS54ICsgciwgZjEueSArIHIpO1xuICAgICAgICAvLyBjaGFuZ2UgZnJvbSBjKysgY29kZSBoZXJlLCB0b28gbXVjaCBmb3JjZSBzb21ldGhpbmcgaXMgd3JvbmcgKC4uLnByb2JhYmx5IG1lKVxuICAgICAgICAvLyBwb2ludC5tdWx0aXBseShWZWN0b3Iuc3Vic3RyYWN0KHQxLCBmMSkpO1xuICAgICAgICBwb2ludC5tdWx0aXBseSgxLjA4KTtcblxuICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIGRpc3RhbmNlID0gMDtcblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG4gICAgfVxuXG4gICAgLy8gZXhhbWluZSBlYWNoIGZlZWxlciBvbm4gZWFjaCB3YWxsXG4gICAgdGhpcy5mZWVsZXJzLmZvckVhY2goZnVuY3Rpb24oZmVlbGVyKSB7XG4gICAgICB3YWxscy5mb3JFYWNoKGZ1bmN0aW9uKHdhbGwpIHtcbiAgICAgICAgaWYgKGludGVyc2VjdCh0aGlzLnZlaGljbGUucG9zaXRpb24sIGZlZWxlciwgd2FsbC5mcm9tLCB3YWxsLnRvKSkge1xuICAgICAgICAgIGlmIChkaXN0YW5jZSA8IGRpc3RhbmNlVG9DbG9zZXN0SVApIHtcbiAgICAgICAgICAgIGRpc3RhbmNlVG9DbG9zZXN0SVAgPSBkaXN0YW5jZTtcbiAgICAgICAgICAgIGNsb3Nlc3RXYWxsID0gd2FsbDtcbiAgICAgICAgICAgIGNsb3Nlc3RQb2ludCA9IHBvaW50O1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgfSwgdGhpcyk7XG5cblxuICAgICAgaWYgKGNsb3Nlc3RXYWxsKSB7XG4gICAgICAgIHZhciBvdmVyU2hvb3QgPSBWZWN0b3Iuc3Vic3RyYWN0KGZlZWxlciwgY2xvc2VzdFBvaW50KTtcbiAgICAgICAgLy8gY29uc29sZS5sb2cob3ZlclNob290Lm1hZ25pdHVkZSgpKVxuICAgICAgICBzdGVlcmluZyA9IFZlY3Rvci5tdWx0aXBseShjbG9zZXN0V2FsbC5ub3JtYWwsIG92ZXJTaG9vdC5tYWduaXR1ZGUoKSk7XG4gICAgICB9XG4gICAgfSwgdGhpcyk7XG5cbiAgICByZXR1cm4gc3RlZXJpbmc7XG4gIH1cblxuICAvLyBjcmVhdGUgYW50ZW5uYSB1c2VkIGluIHdhbGxBdm9pZGFuY2UgLSBpbiB3b3JsZCBjb29yZGluYXRlc1xuICBjcmVhdGVGZWVsZXJzKCkge1xuICAgIC8vIGNlbnRlclxuICAgIHZhciB0bXAgPSBWZWN0b3IubXVsdGlwbHkodGhpcy52ZWhpY2xlLmhlYWRpbmcsIHRoaXMucGFyYW1zLndhbGxEZXRlY3Rpb25GZWVsZXJMZW5ndGgpO1xuICAgIHRoaXMuZmVlbGVyc1swXSA9IFZlY3Rvci5hZGQodGhpcy52ZWhpY2xlLnBvc2l0aW9uLCB0bXApO1xuICAgIC8vIGxlZnRcbiAgICB2YXIgdG1wID0gdGhpcy52ZWhpY2xlLmhlYWRpbmcuY2xvbmUoKTtcbiAgICB0bXAucm90YXRlKE1hdGguUEkgLyAyICogMy41KTtcbiAgICB0bXAgPSBWZWN0b3IubXVsdGlwbHkodG1wLCB0aGlzLnBhcmFtcy53YWxsRGV0ZWN0aW9uRmVlbGVyTGVuZ3RoIC8gMik7XG4gICAgdGhpcy5mZWVsZXJzWzFdID0gVmVjdG9yLmFkZCh0aGlzLnZlaGljbGUucG9zaXRpb24sIHRtcCk7XG4gICAgLy8gcmlnaHRcbiAgICB2YXIgdG1wID0gdGhpcy52ZWhpY2xlLmhlYWRpbmcuY2xvbmUoKTtcbiAgICB0bXAucm90YXRlKE1hdGguUEkgLyAyICogMC41KTtcbiAgICB0bXAgPSBWZWN0b3IubXVsdGlwbHkodG1wLCB0aGlzLnBhcmFtcy53YWxsRGV0ZWN0aW9uRmVlbGVyTGVuZ3RoIC8gMik7XG4gICAgdGhpcy5mZWVsZXJzWzJdID0gVmVjdG9yLmFkZCh0aGlzLnZlaGljbGUucG9zaXRpb24sIHRtcCk7XG4gIH1cblxuICAvLyBGTE9DS1NcbiAgLy8gLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuXG4gIHNlcGFyYXRpb24obmVpZ2hib3JzKSB7XG4gICAgdmFyIHN0ZWVyaW5nID0gbmV3IFZlY3RvcigpO1xuXG4gICAgbmVpZ2hib3JzLmZvckVhY2goZnVuY3Rpb24obmVpZ2hib3IsIGluZGV4KSB7XG4gICAgICB2YXIgdG9BZ2VudCA9IFZlY3Rvci5zdWJzdHJhY3QodGhpcy52ZWhpY2xlLnBvc2l0aW9uLCBuZWlnaGJvci5wb3NpdGlvbik7XG4gICAgICAvLyBzY2FsZSB0aGUgZm9yY2UgaW52ZXJzZWx5IHByb3BvcnRpb25uYWwgdG8gdGhlIGFnZW50IGRpc3RhbmNlIGZyb20gaXRzIG5laWdoYm9yXG4gICAgICB2YXIgZGlzdGFuY2UgPSB0b0FnZW50Lm1hZ25pdHVkZSgpO1xuICAgICAgdG9BZ2VudC5ub3JtYWxpemUoKS5kaXZpZGUoZGlzdGFuY2UpOyAvLyAubXVsdGlwbHkoMTAwKTtcblxuICAgICAgc3RlZXJpbmcuYWRkKHRvQWdlbnQpO1xuICAgIH0sIHRoaXMpO1xuXG4gICAgLy8gaWYgKHRoaXMudmVoaWNsZS5pc1Rlc3QpIHsgY29uc29sZS5sb2coc3RlZXJpbmcpOyB9XG4gICAgcmV0dXJuIHN0ZWVyaW5nO1xuICB9XG5cbiAgYWxpZ25tZW50KG5laWdoYm9ycykge1xuICAgIHZhciBhdmVyYWdlSGVhZGluZyA9IG5ldyBWZWN0b3IoKTtcbiAgICB2YXIgbmVpZ2hib3JDb3VudCA9IG5laWdoYm9ycy5sZW5ndGg7XG5cbiAgICBuZWlnaGJvcnMuZm9yRWFjaChmdW5jdGlvbihuZWlnaGJvcikge1xuICAgICAgYXZlcmFnZUhlYWRpbmcuYWRkKG5laWdoYm9yLmhlYWRpbmcpO1xuICAgIH0pO1xuXG4gICAgaWYgKG5laWdoYm9yQ291bnQgPiAwKSB7XG4gICAgICBhdmVyYWdlSGVhZGluZy5kaXZpZGUobmVpZ2hib3JDb3VudCk7XG4gICAgICBhdmVyYWdlSGVhZGluZy5zdWJzdHJhY3QodGhpcy52ZWhpY2xlLmhlYWRpbmcpO1xuICAgIH1cblxuICAgIHJldHVybiBhdmVyYWdlSGVhZGluZztcbiAgfVxuXG4gIGNvaGVzaW9uKG5laWdoYm9ycykge1xuICAgIHZhciBjZW50ZXJPZk1hc3MgPSBuZXcgVmVjdG9yKCk7XG4gICAgdmFyIHN0ZWVyaW5nID0gbmV3IFZlY3RvcigpO1xuICAgIHZhciBuZWlnaGJvckNvdW50ID0gbmVpZ2hib3JzLmxlbmd0aDtcblxuICAgIG5laWdoYm9ycy5mb3JFYWNoKGZ1bmN0aW9uKG5laWdoYm9yKSB7XG4gICAgICBjZW50ZXJPZk1hc3MuYWRkKG5laWdoYm9yLnBvc2l0aW9uKTtcbiAgICB9KTtcblxuICAgIGlmIChuZWlnaGJvckNvdW50KSB7XG4gICAgICBjZW50ZXJPZk1hc3MuZGl2aWRlKG5laWdoYm9yQ291bnQpO1xuICAgICAgc3RlZXJpbmcgPSB0aGlzLnNlZWsoY2VudGVyT2ZNYXNzKTtcbiAgICB9XG5cbiAgICByZXR1cm4gc3RlZXJpbmc7XG4gIH1cblxuICAvLyAtLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG4gIC8vIERFQlVHIFZJU1VBTElaQVRJT05cbiAgLy8gLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuXG4gIGRlYnVnRmVlbGVycyhjdHgpIHtcbiAgICB0aGlzLmZlZWxlcnMuZm9yRWFjaChmdW5jdGlvbihmZWVsZXIpIHtcbiAgICAgIGN0eC5zYXZlKCk7XG4gICAgICBjdHguYmVnaW5QYXRoKCk7XG5cbiAgICAgIGN0eC5zdHJva2VTdHlsZSA9ICcjYWNhY2FjJztcbiAgICAgIGN0eC5tb3ZlVG8odGhpcy52ZWhpY2xlLnBvc2l0aW9uLngsIHRoaXMudmVoaWNsZS5wb3NpdGlvbi55KTtcbiAgICAgIGN0eC5saW5lVG8oZmVlbGVyLngsIGZlZWxlci55KTtcbiAgICAgIGN0eC5zdHJva2UoKTtcblxuICAgICAgY3R4LmNsb3NlUGF0aCgpO1xuICAgICAgY3R4LnJlc3RvcmUoKTtcbiAgICB9LCB0aGlzKTtcbiAgfVxuXG4gIGRlYnVnV2FuZGVyKGN0eCkge1xuICAgIGN0eC5zYXZlKCk7XG4gICAgY3R4LnRyYW5zbGF0ZSh0aGlzLnZlaGljbGUucG9zaXRpb24ueCwgdGhpcy52ZWhpY2xlLnBvc2l0aW9uLnkpO1xuICAgIGN0eC5yb3RhdGUodGhpcy52ZWhpY2xlLmhlYWRpbmcuZGlyZWN0aW9uKCkpO1xuXG4gICAgLy8gbW92ZSB0byB3YW5kZXIgY2lyY2xlIGNlbnRlclxuICAgIGN0eC50cmFuc2xhdGUodGhpcy53YW5kZXJEaXN0YW5jZSwgMCk7XG5cbiAgICBjdHguc3Ryb2tlU3R5bGUgPSAncmVkJztcbiAgICBjdHguZmlsbFN0eWxlID0gJ3JlZCc7XG4gICAgLy8gd2FuZGVyIGNpcmNsZVxuICAgIGN0eC5iZWdpblBhdGgoKTtcbiAgICBjdHguYXJjKDAsIDAsIHRoaXMud2FuZGVyUmFkaXVzLCAwLCB1dGlscy5Ud29QSSwgZmFsc2UpO1xuICAgIGN0eC5zdHJva2UoKTtcbiAgICBjdHguY2xvc2VQYXRoKCk7XG4gICAgLy8gd2FuZGVyIHRhcmdldFxuICAgIGN0eC5iZWdpblBhdGgoKTtcbiAgICBjdHguYXJjKHRoaXMud2FuZGVyVGFyZ2V0LngsIHRoaXMud2FuZGVyVGFyZ2V0LnksIDIsIDAsIHV0aWxzLlR3b1BJLCBmYWxzZSk7XG4gICAgY3R4LnN0cm9rZSgpO1xuICAgIGN0eC5jbG9zZVBhdGgoKTtcblxuICAgIGN0eC5yZXN0b3JlKCk7XG4gIH1cbn1cblxubW9kdWxlLmV4cG9ydHMgPSBTdGVlcmluZ0JlaGF2aW9ycztcbiJdfQ==