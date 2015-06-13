(function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
var agents = {
  BaseGameEntity: require('./dist/base-game-entity'),
  MovingEntity: require('./dist/moving-entity'),
  Vehicle: require('./dist/vehicle'),
  SteeringBehaviors: require('./dist/steering-behaviors')
};

module.exports = agents;

},{"./dist/base-game-entity":2,"./dist/moving-entity":3,"./dist/steering-behaviors":4,"./dist/vehicle":6}],2:[function(require,module,exports){
'use strict';

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var vector = require('vector');

var defaultEntityType = -1;
var id = 0;

var BaseGameEntity = (function () {
  function BaseGameEntity() {
    var entityType = arguments[0] === undefined ? defaultEntityType : arguments[0];
    var position = arguments[1] === undefined ? vector(0, 0) : arguments[1];
    var boundingRadius = arguments[2] === undefined ? 0 : arguments[2];

    _classCallCheck(this, BaseGameEntity);

    this.id = id++;
    this.entityType = entityType;
    this.position = position;
    this.boundingRadius = boundingRadius;
    // general use tag
    this._tag = false;
  }

  _createClass(BaseGameEntity, [{
    key: 'update',

    // interface
    value: function update(dt) {}
  }, {
    key: 'render',
    value: function render(ctx, buffers) {}
  }, {
    key: 'handleMessage',
    value: function handleMessage(message) {
      return false;
    }
  }, {
    key: 'isTagged',
    value: function isTagged() {
      return this._tag;
    }
  }, {
    key: 'tag',
    value: function tag() {
      this._tag = true;
    }
  }, {
    key: 'unTag',
    value: function unTag() {
      this._tag = false;
    }
  }]);

  return BaseGameEntity;
})();

module.exports = BaseGameEntity;

},{"babel-runtime/helpers/class-call-check":11,"babel-runtime/helpers/create-class":12,"vector":7}],3:[function(require,module,exports){
'use strict';

var _inherits = require('babel-runtime/helpers/inherits')['default'];

var _get = require('babel-runtime/helpers/get')['default'];

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var vector = require('vector');
var BaseGameEntity = require('./base-game-entity');

var entityType = 0;

var MovingEntity = (function (_BaseGameEntity) {
  function MovingEntity(position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    _classCallCheck(this, MovingEntity);

    _get(Object.getPrototypeOf(MovingEntity.prototype), 'constructor', this).call(this, entityType, position, boundingRadius);

    this.velocity = velocity;
    // normalized vector pointing in the direction the entity is moving
    this.heading = heading;
    // vector perpendicular to the heading vector
    this.side = null;
    this.mass = mass;
    // maximum speed (float)
    this.maxSpeed = maxSpeed;
    // maximum force the entity can produce to power itself (think motor engine)
    this.maxForce = maxForce;
    // maximum rate at which the entity can rotate (rad/sec)
    this.maxTurnRate = maxTurnRate;
    // ignore scale
  }

  _inherits(MovingEntity, _BaseGameEntity);

  _createClass(MovingEntity, [{
    key: 'speed',
    value: function speed() {
      return this.velocity.magnitude();
    }
  }, {
    key: 'speedSqrt',
    value: function speedSqrt() {
      return this.velocity.magnitudeSqrt();
    }
  }]);

  return MovingEntity;
})(BaseGameEntity);

module.exports = MovingEntity;

},{"./base-game-entity":2,"babel-runtime/helpers/class-call-check":11,"babel-runtime/helpers/create-class":12,"babel-runtime/helpers/get":13,"babel-runtime/helpers/inherits":14,"vector":7}],4:[function(require,module,exports){
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

},{"./utils":5,"babel-runtime/helpers/class-call-check":11,"babel-runtime/helpers/create-class":12,"vector":7}],5:[function(require,module,exports){
"use strict";

var utils = {

  TwoPI: 2 * Math.PI,

  rand: function rand() {
    return Math.random();
  },

  degsToRads: function degsToRads(degs) {
    return this.TwoPI * (degs / 360);
  },

  // return val between -1 and 1
  randClamped: function randClamped() {
    // return Math.random() * Math.random()
    return Math.random() * 2 - 1;
  },

  randBool: function randBool() {
    return Math.random() > 0.5;
  }

};

module.exports = utils;

},{}],6:[function(require,module,exports){
'use strict';

var _inherits = require('babel-runtime/helpers/inherits')['default'];

var _get = require('babel-runtime/helpers/get')['default'];

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var Vector = require('vector');
var MovingEntity = require('./moving-entity');
var SteeringBehaviors = require('./steering-behaviors');

var Vehicle = (function (_MovingEntity) {
  function Vehicle(world, position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    _classCallCheck(this, Vehicle);

    _get(Object.getPrototypeOf(Vehicle.prototype), 'constructor', this).call(this, position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce);
    // call some kind of super
    this.world = world;
    this.steerings = new SteeringBehaviors(this);
  }

  _inherits(Vehicle, _MovingEntity);

  _createClass(Vehicle, [{
    key: 'update',
    value: function update(dt) {
      // define all the forces applied on the agent
      var steeringForces = this.steerings.calculate();
      // console.log(steeringForces);
      // newton
      this.acceleration = Vector.divide(steeringForces, this.mass);
      // reverse fourier integration - could be a service
      this.velocity.add(Vector.multiply(this.acceleration, dt));
      // vehicle cannot go beyong max speed
      this.velocity.truncate(this.maxSpeed);
      // update location
      this.position.add(Vector.multiply(this.velocity, dt));

      // prevent stupid display behavior (turning when stopped)
      // console.log(this.velocity.magnitudeSqrt());
      if (this.velocity.magnitudeSqrt() > 0.00001) {
        this.heading = Vector.normalize(this.velocity);
        this.side = Vector.orthogonal(this.heading);
      }
    }
  }, {
    key: 'render',
    value: function render(ctx, buffers /*, worldSize */) {}
  }]);

  return Vehicle;
})(MovingEntity);

module.exports = Vehicle;

},{"./moving-entity":3,"./steering-behaviors":4,"babel-runtime/helpers/class-call-check":11,"babel-runtime/helpers/create-class":12,"babel-runtime/helpers/get":13,"babel-runtime/helpers/inherits":14,"vector":7}],7:[function(require,module,exports){
// 2d vectors
var Vector = function(x, y) {
    this.x = !isNaN(x) ? x : 0;
    this.y = !isNaN(y) ? y : 0;
};

// static function - returns new vectors
Vector.add = function(v1, v2) {
    return new this(v1.x + v2.x, v1.y + v2.y);
}

Vector.substract = function(v1, v2) {
    return new this(v1.x - v2.x, v1.y - v2.y);
}

Vector.multiply = function(v1, val) {
    return new this(v1.x * val, v1.y * val);
}

Vector.divide = function(v1, val) {
    return new this(v1.x / val, v1.y / val);
}

Vector.distance = function(v1, v2) {
  var v = this.substract(v2, v1);
  return v.magnitude();
}

Vector.distanceSqrt = function(v1, v2) {
  var v = this.substract(v2, v1);
  return v.magnitudeSqrt();
}

Vector.clone = function(v) {
    return new this(v.x, v.y);
}

Vector.orthogonal = function(v) {
  return new this(-v.y, v.x);
}

// returns a normalized vector orthogonal to the line
// defined by the vector passed as arguments
Vector.normal = function(v1, v2) {
  var temp = Vector.substract(v2, v1).normalize();

  return new this(-temp.y, temp.x);
}

Vector.normalize = function(v) {
  return v.clone().normalize();
}

Vector.dot = function(v1, v2) {
  v1.normalize();
  v2.normalize();
  return (v1.x * v2.x) + (v1.y * v2.y);
}

// instance methods
Vector.prototype.add = function(v) {
    this.x += v.x;
    this.y += v.y;
    return this;
}

Vector.prototype.substract = function(v) {
    this.x -= v.x;
    this.y -= v.y;
    return this;
}

Vector.prototype.multiply = function(value) {
    if (!(value instanceof Vector)) {
      this.x *= value;
      this.y *= value;
    } else {
      this.x *= value.x
      this.y *= value.y
    }
    return this;
}
Vector.prototype.divide = function(value) {
    return this.multiply(1/value);
}

Vector.prototype.truncate = function(value) {
    if (this.magnitude() > value) {
        this.normalize(value);
    }
    return this;
}

Vector.prototype.normalize = function(multiplier) {
    var multiplier = multiplier ? multiplier : 1;
    var mag = this.magnitude();
    if (mag === 0) { return this; }

    this.x = (this.x / mag);
    this.y = (this.y / mag);
    this.multiply(multiplier);
    return this;
}

Vector.prototype.rotate = function(theta) {
    var finalTheta = this.direction() + theta;
    this.setAngle(finalTheta);
}

Vector.prototype.setAngle = function(theta) {
    var magnitude = this.magnitude();
    this.normalize();
    this.x = Math.cos(theta);
    this.y = Math.sin(theta);
    this.multiply(magnitude);
    return this;
}

Vector.prototype.magnitude = function() {
  return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
}
// same as magnitude in square domain, allow to save calcualtion when comparing distances
Vector.prototype.magnitudeSqrt = function() {
  return Math.pow(this.x, 2) + Math.pow(this.y, 2);
}

Vector.prototype.direction = function() {
  // cf. https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Math/atan2
  return Math.atan2(this.y, this.x);
}

Vector.prototype.clone = function() {
  return new this.constructor(this.x, this.y);
}

module.exports = Vector;

},{}],8:[function(require,module,exports){
module.exports = { "default": require("core-js/library/fn/object/create"), __esModule: true };
},{"core-js/library/fn/object/create":15}],9:[function(require,module,exports){
module.exports = { "default": require("core-js/library/fn/object/define-property"), __esModule: true };
},{"core-js/library/fn/object/define-property":16}],10:[function(require,module,exports){
module.exports = { "default": require("core-js/library/fn/object/get-own-property-descriptor"), __esModule: true };
},{"core-js/library/fn/object/get-own-property-descriptor":17}],11:[function(require,module,exports){
"use strict";

exports["default"] = function (instance, Constructor) {
  if (!(instance instanceof Constructor)) {
    throw new TypeError("Cannot call a class as a function");
  }
};

exports.__esModule = true;
},{}],12:[function(require,module,exports){
"use strict";

var _Object$defineProperty = require("babel-runtime/core-js/object/define-property")["default"];

exports["default"] = (function () {
  function defineProperties(target, props) {
    for (var i = 0; i < props.length; i++) {
      var descriptor = props[i];
      descriptor.enumerable = descriptor.enumerable || false;
      descriptor.configurable = true;
      if ("value" in descriptor) descriptor.writable = true;

      _Object$defineProperty(target, descriptor.key, descriptor);
    }
  }

  return function (Constructor, protoProps, staticProps) {
    if (protoProps) defineProperties(Constructor.prototype, protoProps);
    if (staticProps) defineProperties(Constructor, staticProps);
    return Constructor;
  };
})();

exports.__esModule = true;
},{"babel-runtime/core-js/object/define-property":9}],13:[function(require,module,exports){
"use strict";

var _Object$getOwnPropertyDescriptor = require("babel-runtime/core-js/object/get-own-property-descriptor")["default"];

exports["default"] = function get(_x, _x2, _x3) {
  var _again = true;

  _function: while (_again) {
    var object = _x,
        property = _x2,
        receiver = _x3;
    desc = parent = getter = undefined;
    _again = false;

    var desc = _Object$getOwnPropertyDescriptor(object, property);

    if (desc === undefined) {
      var parent = Object.getPrototypeOf(object);

      if (parent === null) {
        return undefined;
      } else {
        _x = parent;
        _x2 = property;
        _x3 = receiver;
        _again = true;
        continue _function;
      }
    } else if ("value" in desc) {
      return desc.value;
    } else {
      var getter = desc.get;

      if (getter === undefined) {
        return undefined;
      }

      return getter.call(receiver);
    }
  }
};

exports.__esModule = true;
},{"babel-runtime/core-js/object/get-own-property-descriptor":10}],14:[function(require,module,exports){
"use strict";

var _Object$create = require("babel-runtime/core-js/object/create")["default"];

exports["default"] = function (subClass, superClass) {
  if (typeof superClass !== "function" && superClass !== null) {
    throw new TypeError("Super expression must either be null or a function, not " + typeof superClass);
  }

  subClass.prototype = _Object$create(superClass && superClass.prototype, {
    constructor: {
      value: subClass,
      enumerable: false,
      writable: true,
      configurable: true
    }
  });
  if (superClass) subClass.__proto__ = superClass;
};

exports.__esModule = true;
},{"babel-runtime/core-js/object/create":8}],15:[function(require,module,exports){
var $ = require('../../modules/$');
module.exports = function create(P, D){
  return $.create(P, D);
};
},{"../../modules/$":21}],16:[function(require,module,exports){
var $ = require('../../modules/$');
module.exports = function defineProperty(it, key, desc){
  return $.setDesc(it, key, desc);
};
},{"../../modules/$":21}],17:[function(require,module,exports){
var $ = require('../../modules/$');
require('../../modules/es6.object.statics-accept-primitives');
module.exports = function getOwnPropertyDescriptor(it, key){
  return $.getDesc(it, key);
};
},{"../../modules/$":21,"../../modules/es6.object.statics-accept-primitives":22}],18:[function(require,module,exports){
var $          = require('./$')
  , global     = $.g
  , core       = $.core
  , isFunction = $.isFunction;
function ctx(fn, that){
  return function(){
    return fn.apply(that, arguments);
  };
}
// type bitmap
$def.F = 1;  // forced
$def.G = 2;  // global
$def.S = 4;  // static
$def.P = 8;  // proto
$def.B = 16; // bind
$def.W = 32; // wrap
function $def(type, name, source){
  var key, own, out, exp
    , isGlobal = type & $def.G
    , isProto  = type & $def.P
    , target   = isGlobal ? global : type & $def.S
        ? global[name] : (global[name] || {}).prototype
    , exports  = isGlobal ? core : core[name] || (core[name] = {});
  if(isGlobal)source = name;
  for(key in source){
    // contains in native
    own = !(type & $def.F) && target && key in target;
    if(own && key in exports)continue;
    // export native or passed
    out = own ? target[key] : source[key];
    // prevent global pollution for namespaces
    if(isGlobal && !isFunction(target[key]))exp = source[key];
    // bind timers to global for call from export context
    else if(type & $def.B && own)exp = ctx(out, global);
    // wrap global constructors for prevent change them in library
    else if(type & $def.W && target[key] == out)!function(C){
      exp = function(param){
        return this instanceof C ? new C(param) : C(param);
      };
      exp.prototype = C.prototype;
    }(out);
    else exp = isProto && isFunction(out) ? ctx(Function.call, out) : out;
    // export
    exports[key] = exp;
    if(isProto)(exports.prototype || (exports.prototype = {}))[key] = out;
  }
}
module.exports = $def;
},{"./$":21}],19:[function(require,module,exports){
module.exports = function($){
  $.FW   = false;
  $.path = $.core;
  return $;
};
},{}],20:[function(require,module,exports){
// fallback for IE11 buggy Object.getOwnPropertyNames with iframe and window
var $ = require('./$')
  , toString = {}.toString
  , getNames = $.getNames;

var windowNames = typeof window == 'object' && Object.getOwnPropertyNames
  ? Object.getOwnPropertyNames(window) : [];

function getWindowNames(it){
  try {
    return getNames(it);
  } catch(e){
    return windowNames.slice();
  }
}

module.exports.get = function getOwnPropertyNames(it){
  if(windowNames && toString.call(it) == '[object Window]')return getWindowNames(it);
  return getNames($.toObject(it));
};
},{"./$":21}],21:[function(require,module,exports){
'use strict';
var global = typeof self != 'undefined' ? self : Function('return this')()
  , core   = {}
  , defineProperty = Object.defineProperty
  , hasOwnProperty = {}.hasOwnProperty
  , ceil  = Math.ceil
  , floor = Math.floor
  , max   = Math.max
  , min   = Math.min;
// The engine works fine with descriptors? Thank's IE8 for his funny defineProperty.
var DESC = !!function(){
  try {
    return defineProperty({}, 'a', {get: function(){ return 2; }}).a == 2;
  } catch(e){ /* empty */ }
}();
var hide = createDefiner(1);
// 7.1.4 ToInteger
function toInteger(it){
  return isNaN(it = +it) ? 0 : (it > 0 ? floor : ceil)(it);
}
function desc(bitmap, value){
  return {
    enumerable  : !(bitmap & 1),
    configurable: !(bitmap & 2),
    writable    : !(bitmap & 4),
    value       : value
  };
}
function simpleSet(object, key, value){
  object[key] = value;
  return object;
}
function createDefiner(bitmap){
  return DESC ? function(object, key, value){
    return $.setDesc(object, key, desc(bitmap, value));
  } : simpleSet;
}

function isObject(it){
  return it !== null && (typeof it == 'object' || typeof it == 'function');
}
function isFunction(it){
  return typeof it == 'function';
}
function assertDefined(it){
  if(it == undefined)throw TypeError("Can't call method on  " + it);
  return it;
}

var $ = module.exports = require('./$.fw')({
  g: global,
  core: core,
  html: global.document && document.documentElement,
  // http://jsperf.com/core-js-isobject
  isObject:   isObject,
  isFunction: isFunction,
  that: function(){
    return this;
  },
  // 7.1.4 ToInteger
  toInteger: toInteger,
  // 7.1.15 ToLength
  toLength: function(it){
    return it > 0 ? min(toInteger(it), 0x1fffffffffffff) : 0; // pow(2, 53) - 1 == 9007199254740991
  },
  toIndex: function(index, length){
    index = toInteger(index);
    return index < 0 ? max(index + length, 0) : min(index, length);
  },
  has: function(it, key){
    return hasOwnProperty.call(it, key);
  },
  create:     Object.create,
  getProto:   Object.getPrototypeOf,
  DESC:       DESC,
  desc:       desc,
  getDesc:    Object.getOwnPropertyDescriptor,
  setDesc:    defineProperty,
  setDescs:   Object.defineProperties,
  getKeys:    Object.keys,
  getNames:   Object.getOwnPropertyNames,
  getSymbols: Object.getOwnPropertySymbols,
  assertDefined: assertDefined,
  // Dummy, fix for not array-like ES3 string in es5 module
  ES5Object: Object,
  toObject: function(it){
    return $.ES5Object(assertDefined(it));
  },
  hide: hide,
  def: createDefiner(0),
  set: global.Symbol ? simpleSet : hide,
  each: [].forEach
});
/* eslint-disable no-undef */
if(typeof __e != 'undefined')__e = core;
if(typeof __g != 'undefined')__g = global;
},{"./$.fw":19}],22:[function(require,module,exports){
var $        = require('./$')
  , $def     = require('./$.def')
  , isObject = $.isObject
  , toObject = $.toObject;
$.each.call(('freeze,seal,preventExtensions,isFrozen,isSealed,isExtensible,' +
  'getOwnPropertyDescriptor,getPrototypeOf,keys,getOwnPropertyNames').split(',')
, function(KEY, ID){
  var fn     = ($.core.Object || {})[KEY] || Object[KEY]
    , forced = 0
    , method = {};
  method[KEY] = ID == 0 ? function freeze(it){
    return isObject(it) ? fn(it) : it;
  } : ID == 1 ? function seal(it){
    return isObject(it) ? fn(it) : it;
  } : ID == 2 ? function preventExtensions(it){
    return isObject(it) ? fn(it) : it;
  } : ID == 3 ? function isFrozen(it){
    return isObject(it) ? fn(it) : true;
  } : ID == 4 ? function isSealed(it){
    return isObject(it) ? fn(it) : true;
  } : ID == 5 ? function isExtensible(it){
    return isObject(it) ? fn(it) : false;
  } : ID == 6 ? function getOwnPropertyDescriptor(it, key){
    return fn(toObject(it), key);
  } : ID == 7 ? function getPrototypeOf(it){
    return fn(Object($.assertDefined(it)));
  } : ID == 8 ? function keys(it){
    return fn(toObject(it));
  } : require('./$.get-names').get;
  try {
    fn('z');
  } catch(e){
    forced = 1;
  }
  $def($def.S + $def.F * forced, 'Object', method);
});
},{"./$":21,"./$.def":18,"./$.get-names":20}]},{},[1])
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyaWZ5L25vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJhZ2VudHMuanMiLCJzcmMvdmVoaWNsZS5lczYuanMiLCJub2RlX21vZHVsZXMvdmVjdG9yL3ZlY3Rvci5qcyIsIi4uLy4uLy4uLy4uLy4uLy4uL3Vzci9sb2NhbC9saWIvbm9kZV9tb2R1bGVzL2JhYmVsLXJ1bnRpbWUvY29yZS1qcy9vYmplY3QvY3JlYXRlLmpzIiwiLi4vLi4vLi4vLi4vLi4vLi4vdXNyL2xvY2FsL2xpYi9ub2RlX21vZHVsZXMvYmFiZWwtcnVudGltZS9jb3JlLWpzL29iamVjdC9kZWZpbmUtcHJvcGVydHkuanMiLCIuLi8uLi8uLi8uLi8uLi8uLi91c3IvbG9jYWwvbGliL25vZGVfbW9kdWxlcy9iYWJlbC1ydW50aW1lL2NvcmUtanMvb2JqZWN0L2dldC1vd24tcHJvcGVydHktZGVzY3JpcHRvci5qcyIsIi4uLy4uLy4uLy4uLy4uLy4uL3Vzci9sb2NhbC9saWIvbm9kZV9tb2R1bGVzL2JhYmVsLXJ1bnRpbWUvaGVscGVycy9jbGFzcy1jYWxsLWNoZWNrLmpzIiwiLi4vLi4vLi4vLi4vLi4vLi4vdXNyL2xvY2FsL2xpYi9ub2RlX21vZHVsZXMvYmFiZWwtcnVudGltZS9oZWxwZXJzL2NyZWF0ZS1jbGFzcy5qcyIsIi4uLy4uLy4uLy4uLy4uLy4uL3Vzci9sb2NhbC9saWIvbm9kZV9tb2R1bGVzL2JhYmVsLXJ1bnRpbWUvaGVscGVycy9nZXQuanMiLCIuLi8uLi8uLi8uLi8uLi8uLi91c3IvbG9jYWwvbGliL25vZGVfbW9kdWxlcy9iYWJlbC1ydW50aW1lL2hlbHBlcnMvaW5oZXJpdHMuanMiLCIuLi8uLi8uLi8uLi8uLi8uLi91c3IvbG9jYWwvbGliL25vZGVfbW9kdWxlcy9iYWJlbC1ydW50aW1lL25vZGVfbW9kdWxlcy9jb3JlLWpzL2xpYnJhcnkvZm4vb2JqZWN0L2NyZWF0ZS5qcyIsIi4uLy4uLy4uLy4uLy4uLy4uL3Vzci9sb2NhbC9saWIvbm9kZV9tb2R1bGVzL2JhYmVsLXJ1bnRpbWUvbm9kZV9tb2R1bGVzL2NvcmUtanMvbGlicmFyeS9mbi9vYmplY3QvZGVmaW5lLXByb3BlcnR5LmpzIiwiLi4vLi4vLi4vLi4vLi4vLi4vdXNyL2xvY2FsL2xpYi9ub2RlX21vZHVsZXMvYmFiZWwtcnVudGltZS9ub2RlX21vZHVsZXMvY29yZS1qcy9saWJyYXJ5L2ZuL29iamVjdC9nZXQtb3duLXByb3BlcnR5LWRlc2NyaXB0b3IuanMiLCIuLi8uLi8uLi8uLi8uLi8uLi91c3IvbG9jYWwvbGliL25vZGVfbW9kdWxlcy9iYWJlbC1ydW50aW1lL25vZGVfbW9kdWxlcy9jb3JlLWpzL2xpYnJhcnkvbW9kdWxlcy8kLmRlZi5qcyIsIi4uLy4uLy4uLy4uLy4uLy4uL3Vzci9sb2NhbC9saWIvbm9kZV9tb2R1bGVzL2JhYmVsLXJ1bnRpbWUvbm9kZV9tb2R1bGVzL2NvcmUtanMvbGlicmFyeS9tb2R1bGVzLyQuZncuanMiLCIuLi8uLi8uLi8uLi8uLi8uLi91c3IvbG9jYWwvbGliL25vZGVfbW9kdWxlcy9iYWJlbC1ydW50aW1lL25vZGVfbW9kdWxlcy9jb3JlLWpzL2xpYnJhcnkvbW9kdWxlcy8kLmdldC1uYW1lcy5qcyIsIi4uLy4uLy4uLy4uLy4uLy4uL3Vzci9sb2NhbC9saWIvbm9kZV9tb2R1bGVzL2JhYmVsLXJ1bnRpbWUvbm9kZV9tb2R1bGVzL2NvcmUtanMvbGlicmFyeS9tb2R1bGVzLyQuanMiLCIuLi8uLi8uLi8uLi8uLi8uLi91c3IvbG9jYWwvbGliL25vZGVfbW9kdWxlcy9iYWJlbC1ydW50aW1lL25vZGVfbW9kdWxlcy9jb3JlLWpzL2xpYnJhcnkvbW9kdWxlcy9lczYub2JqZWN0LnN0YXRpY3MtYWNjZXB0LXByaW1pdGl2ZXMuanMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUE7QUNBQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7Ozs7Ozs7O0FDUkEsSUFBSSxNQUFNLEdBQUcs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USxDQUFDO0FBQ3BDLFVBQUksWUFBWSxHQUFHLElBQUksQ0FBQztBQUN4QixVQUFJLFdBQVcsR0FBRyxJQUFJLENBQUM7O0FBRXZCLFVBQUksUUFBUSxHQUFHLElBQUksTUFBTSxFQUFFLENBQUM7QUFDNUIsVUFBSSxRQUFRLEVBQUUsS0FBSyxDQUFDOzs7QUFHcEIsVUFBSSxTQUFTLEdBQUcsU0FBWixTQUFTLENBQVksRUFBRSxFQUFFLEVBQUUsRUFBRSxFQUFFLEVBQUUsRUFBRSxFQUFFO0FBQ3ZDLFlBQUksSUFBSSxHQUFHLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLElBQUcsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLEFBQUMsR0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxJQUFHLEVBQUUsQ0FBQyxDQUFDLEdBQUMsRUFBRSxDQUFDLENBQUMsQ0FBQSxBQUFDLENBQUM7QUFDM0QsWUFBSSxJQUFJLEdBQUcsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsSUFBRyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsQUFBQyxHQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLElBQUcsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLEFBQUMsQ0FBQzs7QUFFM0QsWUFBSSxJQUFJLEdBQUcsQ0FBQyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsSUFBRyxFQUFFLENBQUMsQ0FBQyxHQUFDLEVBQUUsQ0FBQyxDQUFDLENBQUEsQUFBQyxHQUFDLENBQUMsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLElBQUcsRUFBRSxDQUFDLENBQUMsR0FBQyxFQUFFLENBQUMsQ0FBQyxDQUFBLEFBQUMsQ0FBQztBQUMzRCxZQUFJLElBQUks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LE9BQU8sRUFBRTtBQUMzQyxZQUFJLENBQUMsT0FBTyxHQUFHLE1BQU0sQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDO0FBQy9DLFlBQUksQ0FBQyxJQUFJLEdBQUcsTUFBTSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLENBQUM7T0FDN0M7S0FDRjs7O1dBRUssZ0JBQUMsR0FBRyxFQUFFLE9BQU8sbUJBQWtCLEVBQUU7OztTQTdCbkMsT0FBTztHQUFTLFlBQVk7O0FBZ0NsQyxNQUFNLENBQUMsT0FBTyxHQUFHLE9BQU8sQ0FBQzs7O0FDcEN6QjtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3hJQTs7QUNBQTs7QUNBQTs7QUNBQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDUkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3ZCQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMxQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3BCQTtBQUNBO0FBQ0E7QUFDQTs7QUNIQTtBQUNBO0FBQ0E7QUFDQTs7QUNIQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ0pBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMvQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNKQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ25CQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDL0ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSIsImZpbGUiOiJnZW5lcmF0ZWQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlc0NvbnRlbnQiOlsiKGZ1bmN0aW9uIGUodCxuLHIpe2Z1bmN0aW9uIHMobyx1KXtpZighbltvXSl7aWYoIXRbb10pe3ZhciBhPXR5cGVvZiByZXF1aXJlPT1cImZ1bmN0aW9uXCImJnJlcXVpcmU7aWYoIXUmJmEpcmV0dXJuIGEobywhMCk7aWYoaSlyZXR1cm4gaShvLCEwKTt2YXIgZj1uZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiK28rXCInXCIpO3Rocm93IGYuY29kZT1cIk1PRFVMRV9OT1RfRk9VTkRcIixmfXZhciBsPW5bb109e2V4cG9ydHM6e319O3Rbb11bMF0uY2FsbChsLmV4cG9ydHMsZnVuY3Rpb24oZSl7dmFyIG49dFtvXVsxXVtlXTtyZXR1cm4gcyhuP246ZSl9LGwsbC5leHBvcnRzLGUsdCxuLHIpfXJldHVybiBuW29dLmV4cG9ydHN9dmFyIGk9dHlwZW9mIHJlcXVpcmU9PVwiZnVuY3Rpb25cIiYmcmVxdWlyZTtmb3IodmFyIG89MDtvPHIubGVuZ3RoO28rKylzKHJbb10pO3JldHVybiBzfSkiLCJ2YXIgYWdlbnRzID0ge1xuICBCYXNlR2FtZUVudGl0eTogcmVxdWlyZSgnLi9kaXN0L2Jhc2UtZ2FtZS1lbnRpdHknKSxcbiAgTW92aW5nRW50aXR5OiByZXF1aXJlKCcuL2Rpc3QvbW92aW5nLWVudGl0eScpLFxuICBWZWhpY2xlOiByZXF1aXJlKCcuL2Rpc3QvdmVoaWNsZScpLFxuICBTdGVlcmluZ0JlaGF2aW9yczogcmVxdWlyZSgnLi9kaXN0L3N0ZWVyaW5nLWJlaGF2aW9ycycpXG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IGFnZW50cztcbiIsInZhciBWZWN0b3IgPSByZXF1aXJlKCd2ZWN0b3InKTtcbnZhciBNb3ZpbmdFbnRpdHkgPSByZXF1aXJlKCcuL21vdmluZy1lbnRpdHknKTtcbnZhciBTdGVlcmluZ0JlaGF2aW9ycyA9IHJlcXVpcmUoJy4vc3RlZXJpbmctYmVoYXZpb3JzJyk7XG5cbmNsYXNzIFZlaGljbGUgZXh0ZW5kcyBNb3ZpbmdFbnRpdHkge1xuICBjb25zdHJ1Y3Rvcih3b3JsZCwgcG9zaXRpb24sIGJvdW5kaW5nUmFkaXVzLCB2ZWxvY2l0eSwgbWF4U3BlZWQsIGhlYWRpbmcsIG1hc3MsIG1heFR1cm5SYXRlLCBtYXhGb3JjZSkge1xuICAgIHN1cGVyKHBvc2l0aW9uLCBib3VuZGluZ1JhZGl1cywgdmVsb2NpdHksIG1heFNwZWVkLCBoZWFkaW5nLCBtYXNzLCBtYXhUdXJuUmF0ZSwgbWF4Rm9yY2UpO1xuICAgIC8vIGNhbGwgc29tZSBraW5kIG9mIHN1cGVyXG4gICAgdGhpcy53b3JsZCA9IHdvcmxkO1xuICAgIHRoaXMuc3RlZXJpbmdzID0gbmV3IFN0ZWVyaW5nQmVoYXZpb3JzKHRoaXMpO1xuICB9XG5cbiAgdXBkYXRlKGR0KSB7XG4gICAgLy8gZGVmaW5lIGFsbCB0aGUgZm9yY2VzIGFwcGxpZWQgb24gdGhlIGFnZW50XG4gICAgdmFyIHN0ZWVyaW5nRm9yY2VzID0gdGhpcy5zdGVlcmluZ3MuY2FsY3VsYXRlKCk7XG4gICAgLy8gY29uc29sZS5sb2coc3RlZXJpbmdGb3JjZXMpO1xuICAgIC8vIG5ld3RvblxuICAgIHRoaXMuYWNjZWxlcmF0aW9uID0gVmVjdG9yLmRpdmlkZShzdGVlcmluZ0ZvcmNlcywgdGhpcy5tYXNzKTtcbiAgICAvLyByZXZlcnNlIGZvdXJpZXIgaW50ZWdyYXRpb24gLSBjb3VsZCBiZSBhIHNlcnZpY2VcbiAgICB0aGlzLnZlbG9jaXR5LmFkZChWZWN0b3IubXVsdGlwbHkodGhpcy5hY2NlbGVyYXRpb24sIGR0KSk7XG4gICAgLy8gdmVoaWNsZSBjYW5ub3QgZ28gYmV5b25nIG1heCBzcGVlZFxuICAgIHRoaXMudmVsb2NpdHkudHJ1bmNhdGUodGhpcy5tYXhTcGVlZCk7XG4gICAgLy8gdXBkYXRlIGxvY2F0aW9uXG4gICAgdGhpcy5wb3NpdGlvbi5hZGQoVmVjdG9yLm11bHRpcGx5KHRoaXMudmVsb2NpdHksIGR0KSk7XG5cbiAgICAvLyBwcmV2ZW50IHN0dXBpZCBkaXNwbGF5IGJlaGF2aW9yICh0dXJuaW5nIHdoZW4gc3RvcHBlZClcbiAgICAvLyBjb25zb2xlLmxvZyh0aGlzLnZlbG9jaXR5Lm1hZ25pdHVkZVNxcnQoKSk7XG4gICAgaWYgKHRoaXMudmVsb2NpdHkubWFnbml0dWRlU3FydCgpID4gMC4wMDAwMSkge1xuICAgICAgdGhpcy5oZWFkaW5nID0gVmVjdG9yLm5vcm1hbGl6ZSh0aGlzLnZlbG9jaXR5KTtcbiAgICAgIHRoaXMuc2lkZSA9IFZlY3Rvci5vcnRob2dvbmFsKHRoaXMuaGVhZGluZyk7XG4gICAgfVxuICB9XG5cbiAgcmVuZGVyKGN0eCwgYnVmZmVycy8qLCB3b3JsZFNpemUgKi8pIHt9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gVmVoaWNsZTtcbiIsIi8vIDJkIHZlY3RvcnNcbnZhciBWZWN0b3IgPSBmdW5jdGlvbih4LCB5KSB7XG4gICAgdGhpcy54ID0gIWlzTmFOKHgpID8geCA6IDA7XG4gICAgdGhpcy55ID0gIWlzTmFOKHkpID8geSA6IDA7XG59O1xuXG4vLyBzdGF0aWMgZnVuY3Rpb24gLSByZXR1cm5zIG5ldyB2ZWN0b3JzXG5WZWN0b3IuYWRkID0gZnVuY3Rpb24odjEsIHYyKSB7XG4gICAgcmV0dXJuIG5ldyB0aGlzKHYxLnggKyB2Mi54LCB2MS55ICsgdjIueSk7XG59XG5cblZlY3Rvci5zdWJzdHJhY3QgPSBmdW5jdGlvbih2MSwgdjIpIHtcbiAgICByZXR1cm4gbmV3IHRoaXModjEueCAtIHYyLngsIHYxLnkgLSB2Mi55KTtcbn1cblxuVmVjdG9yLm11bHRpcGx5ID0gZnVuY3Rpb24odjEsIHZhbCkge1xuICAgIHJldHVybiBuZXcgdGhpcyh2MS54ICogdmFsLCB2MS55ICogdmFsKTtcbn1cblxuVmVjdG9yLmRpdmlkZSA9IGZ1bmN0aW9uKHYxLCB2YWwpIHtcbiAgICByZXR1cm4gbmV3IHRoaXModjEueCAvIHZhbCwgdjEueSAvIHZhbCk7XG59XG5cblZlY3Rvci5kaXN0YW5jZSA9IGZ1bmN0aW9uKHYxLCB2Mikge1xuICB2YXIgdiA9IHRoaXMuc3Vic3RyYWN0KHYyLCB2MSk7XG4gIHJldHVybiB2Lm1hZ25pdHVkZSgpO1xufVxuXG5WZWN0b3IuZGlzdGFuY2VTcXJ0ID0gZnVuY3Rpb24odjEsIHYyKSB7XG4gIHZhciB2ID0gdGhpcy5zdWJzdHJhY3QodjIsIHYxKTtcbiAgcmV0dXJuIHYubWFnbml0dWRlU3FydCgpO1xufVxuXG5WZWN0b3IuY2xvbmUgPSBmdW5jdGlvbih2KSB7XG4gICAgcmV0dXJuIG5ldyB0aGlzKHYueCwgdi55KTtcbn1cblxuVmVjdG9yLm9ydGhvZ29uYWwgPSBmdW5jdGlvbih2KSB7XG4gIHJldHVybiBuZXcgdGhpcygtdi55LCB2LngpO1xufVxuXG4vLyByZXR1cm5zIGEgbm9ybWFsaXplZCB2ZWN0b3Igb3J0aG9nb25hbCB0byB0aGUgbGluZVxuLy8gZGVmaW5lZCBieSB0aGUgdmVjdG9yIHBhc3NlZCBhcyBhcmd1bWVudHNcblZlY3Rvci5ub3JtYWwgPSBmdW5jdGlvbih2MSwgdjIpIHtcbiAgdmFyIHRlbXAgPSBWZWN0b3Iuc3Vic3RyYWN0KHYyLCB2MSkubm9ybWFsaXplKCk7XG5cbiAgcmV0dXJuIG5ldyB0aGlzKC10ZW1wLnksIHRlbXAueCk7XG59XG5cblZlY3Rvci5ub3JtYWxpemUgPSBmdW5jdGlvbih2KSB7XG4gIHJldHVybiB2LmNsb25lKCkubm9ybWFsaXplKCk7XG59XG5cblZlY3Rvci5kb3QgPSBmdW5jdGlvbih2MSwgdjIpIHtcbiAgdjEubm9ybWFsaXplKCk7XG4gIHYyLm5vcm1hbGl6ZSgpO1xuICByZXR1cm4gKHYxLnggKiB2Mi54KSArICh2MS55ICogdjIueSk7XG59XG5cbi8vIGluc3RhbmNlIG1ldGhvZHNcblZlY3Rvci5wcm90b3R5cGUuYWRkID0gZnVuY3Rpb24odikge1xuICAgIHRoaXMueCArPSB2Lng7XG4gICAgdGhpcy55ICs9IHYueTtcbiAgICByZXR1cm4gdGhpcztcbn1cblxuVmVjdG9yLnByb3RvdHlwZS5zdWJzdHJhY3QgPSBmdW5jdGlvbih2KSB7XG4gICAgdGhpcy54IC09IHYueDtcbiAgICB0aGlzLnkgLT0gdi55O1xuICAgIHJldHVybiB0aGlzO1xufVxuXG5WZWN0b3IucHJvdG90eXBlLm11bHRpcGx5ID0gZnVuY3Rpb24odmFsdWUpIHtcbiAgICBpZiAoISh2YWx1ZSBpbnN0YW5jZW9mIFZlY3RvcikpIHtcbiAgICAgIHRoaXMueCAqPSB2YWx1ZTtcbiAgICAgIHRoaXMueSAqPSB2YWx1ZTtcbiAgICB9IGVsc2Uge1xuICAgICAgdGhpcy54ICo9IHZhbHVlLnhcbiAgICAgIHRoaXMueSAqPSB2YWx1ZS55XG4gICAgfVxuICAgIHJldHVybiB0aGlzO1xufVxuVmVjdG9yLnByb3RvdHlwZS5kaXZpZGUgPSBmdW5jdGlvbih2YWx1ZSkge1xuICAgIHJldHVybiB0aGlzLm11bHRpcGx5KDEvdmFsdWUpO1xufVxuXG5WZWN0b3IucHJvdG90eXBlLnRydW5jYXRlID0gZnVuY3Rpb24odmFsdWUpIHtcbiAgICBpZiAodGhpcy5tYWduaXR1ZGUoKSA+IHZhbHVlKSB7XG4gICAgICAgIHRoaXMubm9ybWFsaXplKHZhbHVlKTtcbiAgICB9XG4gICAgcmV0dXJuIHRoaXM7XG59XG5cblZlY3Rvci5wcm90b3R5cGUubm9ybWFsaXplID0gZnVuY3Rpb24obXVsdGlwbGllcikge1xuICAgIHZhciBtdWx0aXBsaWVyID0gbXVsdGlwbGllciA/IG11bHRpcGxpZXIgOiAxO1xuICAgIHZhciBtYWcgPSB0aGlzLm1hZ25pdHVkZSgpO1xuICAgIGlmIChtYWcgPT09IDApIHsgcmV0dXJuIHRoaXM7IH1cblxuICAgIHRoaXMueCA9ICh0aGlzLnggLyBtYWcpO1xuICAgIHRoaXMueSA9ICh0aGlzLnkgLyBtYWcpO1xuICAgIHRoaXMubXVsdGlwbHkobXVsdGlwbGllcik7XG4gICAgcmV0dXJuIHRoaXM7XG59XG5cblZlY3Rvci5wcm90b3R5cGUucm90YXRlID0gZnVuY3Rpb24odGhldGEpIHtcbiAgICB2YXIgZmluYWxUaGV0YSA9IHRoaXMuZGlyZWN0aW9uKCkgKyB0aGV0YTtcbiAgICB0aGlzLnNldEFuZ2xlKGZpbmFsVGhldGEpO1xufVxuXG5WZWN0b3IucHJvdG90eXBlLnNldEFuZ2xlID0gZnVuY3Rpb24odGhldGEpIHtcbiAgICB2YXIgbWFnbml0dWRlID0gdGhpcy5tYWduaXR1ZGUoKTtcbiAgICB0aGlzLm5vcm1hbGl6ZSgpO1xuICAgIHRoaXMueCA9IE1hdGguY29zKHRoZXRhKTtcbiAgICB0aGlzLnkgPSBNYXRoLnNpbih0aGV0YSk7XG4gICAgdGhpcy5tdWx0aXBseShtYWduaXR1ZGUpO1xuICAgIHJldHVybiB0aGlzO1xufVxuXG5WZWN0b3IucHJvdG90eXBlLm1hZ25pdHVkZSA9IGZ1bmN0aW9uKCkge1xuICByZXR1cm4gTWF0aC5zcXJ0KE1hdGgucG93KHRoaXMueCwgMikgKyBNYXRoLnBvdyh0aGlzLnksIDIpKTtcbn1cbi8vIHNhbWUgYXMgbWFnbml0dWRlIGluIHNxdWFyZSBkb21haW4sIGFsbG93IHRvIHNhdmUgY2FsY3VhbHRpb24gd2hlbiBjb21wYXJpbmcgZGlzdGFuY2VzXG5WZWN0b3IucHJvdG90eXBlLm1hZ25pdHVkZVNxcnQgPSBmdW5jdGlvbigpIHtcbiAgcmV0dXJuIE1hdGgucG93KHRoaXMueCwgMikgKyBNYXRoLnBvdyh0aGlzLnksIDIpO1xufVxuXG5WZWN0b3IucHJvdG90eXBlLmRpcmVjdGlvbiA9IGZ1bmN0aW9uKCkge1xuICAvLyBjZi4gaHR0cHM6Ly9kZXZlbG9wZXIubW96aWxsYS5vcmcvZW4tVVMvZG9jcy9XZWIvSmF2YVNjcmlwdC9SZWZlcmVuY2UvR2xvYmFsX09iamVjdHMvTWF0aC9hdGFuMlxuICByZXR1cm4gTWF0aC5hdGFuMih0aGlzLnksIHRoaXMueCk7XG59XG5cblZlY3Rvci5wcm90b3R5cGUuY2xvbmUgPSBmdW5jdGlvbigpIHtcbiAgcmV0dXJuIG5ldyB0aGlzLmNvbnN0cnVjdG9yKHRoaXMueCwgdGhpcy55KTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBWZWN0b3I7XG4iLCJtb2R1bGUuZXhwb3J0cyA9IHsgXCJkZWZhdWx0XCI6IHJlcXVpcmUoXCJjb3JlLWpzL2xpYnJhcnkvZm4vb2JqZWN0L2NyZWF0ZVwiKSwgX19lc01vZHVsZTogdHJ1ZSB9OyIsIm1vZHVsZS5leHBvcnRzID0geyBcImRlZmF1bHRcIjogcmVxdWlyZShcImNvcmUtanMvbGlicmFyeS9mbi9vYmplY3QvZGVmaW5lLXByb3BlcnR5XCIpLCBfX2VzTW9kdWxlOiB0cnVlIH07IiwibW9kdWxlLmV4cG9ydHMgPSB7IFwiZGVmYXVsdFwiOiByZXF1aXJlKFwiY29yZS1qcy9saWJyYXJ5L2ZuL29iamVjdC9nZXQtb3duLXByb3BlcnR5LWRlc2NyaXB0b3JcIiksIF9fZXNNb2R1bGU6IHRydWUgfTsiLCJcInVzZSBzdHJpY3RcIjtcblxuZXhwb3J0c1tcImRlZmF1bHRcIl0gPSBmdW5jdGlvbiAoaW5zdGFuY2UsIENvbnN0cnVjdG9yKSB7XG4gIGlmICghKGluc3RhbmNlIGluc3RhbmNlb2YgQ29uc3RydWN0b3IpKSB7XG4gICAgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkNhbm5vdCBjYWxsIGEgY2xhc3MgYXMgYSBmdW5jdGlvblwiKTtcbiAgfVxufTtcblxuZXhwb3J0cy5fX2VzTW9kdWxlID0gdHJ1ZTsiLCJcInVzZSBzdHJpY3RcIjtcblxudmFyIF9PYmplY3QkZGVmaW5lUHJvcGVydHkgPSByZXF1aXJlKFwiYmFiZWwtcnVudGltZS9jb3JlLWpzL29iamVjdC9kZWZpbmUtcHJvcGVydHlcIilbXCJkZWZhdWx0XCJdO1xuXG5leHBvcnRzW1wiZGVmYXVsdFwiXSA9IChmdW5jdGlvbiAoKSB7XG4gIGZ1bmN0aW9uIGRlZmluZVByb3BlcnRpZXModGFyZ2V0LCBwcm9wcykge1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgcHJvcHMubGVuZ3RoOyBpKyspIHtcbiAgICAgIHZhciBkZXNjcmlwdG9yID0gcHJvcHNbaV07XG4gICAgICBkZXNjcmlwdG9yLmVudW1lcmFibGUgPSBkZXNjcmlwdG9yLmVudW1lcmFibGUgfHwgZmFsc2U7XG4gICAgICBkZXNjcmlwdG9yLmNvbmZpZ3VyYWJsZSA9IHRydWU7XG4gICAgICBpZiAoXCJ2YWx1ZVwiIGluIGRlc2NyaXB0b3IpIGRlc2NyaXB0b3Iud3JpdGFibGUgPSB0cnVlO1xuXG4gICAgICBfT2JqZWN0JGRlZmluZVByb3BlcnR5KHRhcmdldCwgZGVzY3JpcHRvci5rZXksIGRlc2NyaXB0b3IpO1xuICAgIH1cbiAgfVxuXG4gIHJldHVybiBmdW5jdGlvbiAoQ29uc3RydWN0b3IsIHByb3RvUHJvcHMsIHN0YXRpY1Byb3BzKSB7XG4gICAgaWYgKHByb3RvUHJvcHMpIGRlZmluZVByb3BlcnRpZXMoQ29uc3RydWN0b3IucHJvdG90eXBlLCBwcm90b1Byb3BzKTtcbiAgICBpZiAoc3RhdGljUHJvcHMpIGRlZmluZVByb3BlcnRpZXMoQ29uc3RydWN0b3IsIHN0YXRpY1Byb3BzKTtcbiAgICByZXR1cm4gQ29uc3RydWN0b3I7XG4gIH07XG59KSgpO1xuXG5leHBvcnRzLl9fZXNNb2R1bGUgPSB0cnVlOyIsIlwidXNlIHN0cmljdFwiO1xuXG52YXIgX09iamVjdCRnZXRPd25Qcm9wZXJ0eURlc2NyaXB0b3IgPSByZXF1aXJlKFwiYmFiZWwtcnVudGltZS9jb3JlLWpzL29iamVjdC9nZXQtb3duLXByb3BlcnR5LWRlc2NyaXB0b3JcIilbXCJkZWZhdWx0XCJdO1xuXG5leHBvcnRzW1wiZGVmYXVsdFwiXSA9IGZ1bmN0aW9uIGdldChfeCwgX3gyLCBfeDMpIHtcbiAgdmFyIF9hZ2FpbiA9IHRydWU7XG5cbiAgX2Z1bmN0aW9uOiB3aGlsZSAoX2FnYWluKSB7XG4gICAgdmFyIG9iamVjdCA9IF94LFxuICAgICAgICBwcm9wZXJ0eSA9IF94MixcbiAgICAgICAgcmVjZWl2ZXIgPSBfeDM7XG4gICAgZGVzYyA9IHBhcmVudCA9IGdldHRlciA9IHVuZGVmaW5lZDtcbiAgICBfYWdhaW4gPSBmYWxzZTtcblxuICAgIHZhciBkZXNjID0gX09iamVjdCRnZXRPd25Qcm9wZXJ0eURlc2NyaXB0b3Iob2JqZWN0LCBwcm9wZXJ0eSk7XG5cbiAgICBpZiAoZGVzYyA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICB2YXIgcGFyZW50ID0gT2JqZWN0LmdldFByb3RvdHlwZU9mKG9iamVjdCk7XG5cbiAgICAgIGlmIChwYXJlbnQgPT09IG51bGwpIHtcbiAgICAgICAgcmV0dXJuIHVuZGVmaW5lZDtcbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIF94ID0gcGFyZW50O1xuICAgICAgICBfeDIgPSBwcm9wZXJ0eTtcbiAgICAgICAgX3gzID0gcmVjZWl2ZXI7XG4gICAgICAgIF9hZ2FpbiA9IHRydWU7XG4gICAgICAgIGNvbnRpbnVlIF9mdW5jdGlvbjtcbiAgICAgIH1cbiAgICB9IGVsc2UgaWYgKFwidmFsdWVcIiBpbiBkZXNjKSB7XG4gICAgICByZXR1cm4gZGVzYy52YWx1ZTtcbiAgICB9IGVsc2Uge1xuICAgICAgdmFyIGdldHRlciA9IGRlc2MuZ2V0O1xuXG4gICAgICBpZiAoZ2V0dGVyID09PSB1bmRlZmluZWQpIHtcbiAgICAgICAgcmV0dXJuIHVuZGVmaW5lZDtcbiAgICAgIH1cblxuICAgICAgcmV0dXJuIGdldHRlci5jYWxsKHJlY2VpdmVyKTtcbiAgICB9XG4gIH1cbn07XG5cbmV4cG9ydHMuX19lc01vZHVsZSA9IHRydWU7IiwiXCJ1c2Ugc3RyaWN0XCI7XG5cbnZhciBfT2JqZWN0JGNyZWF0ZSA9IHJlcXVpcmUoXCJiYWJlbC1ydW50aW1lL2NvcmUtanMvb2JqZWN0L2NyZWF0ZVwiKVtcImRlZmF1bHRcIl07XG5cbmV4cG9ydHNbXCJkZWZhdWx0XCJdID0gZnVuY3Rpb24gKHN1YkNsYXNzLCBzdXBlckNsYXNzKSB7XG4gIGlmICh0eXBlb2Ygc3VwZXJDbGFzcyAhPT0gXCJmdW5jdGlvblwiICYmIHN1cGVyQ2xhc3MgIT09IG51bGwpIHtcbiAgICB0aHJvdyBuZXcgVHlwZUVycm9yKFwiU3VwZXIgZXhwcmVzc2lvbiBtdXN0IGVpdGhlciBiZSBudWxsIG9yIGEgZnVuY3Rpb24sIG5vdCBcIiArIHR5cGVvZiBzdXBlckNsYXNzKTtcbiAgfVxuXG4gIHN1YkNsYXNzLnByb3RvdHlwZSA9IF9PYmplY3QkY3JlYXRlKHN1cGVyQ2xhc3MgJiYgc3VwZXJDbGFzcy5wcm90b3R5cGUsIHtcbiAgICBjb25zdHJ1Y3Rvcjoge1xuICAgICAgdmFsdWU6IHN1YkNsYXNzLFxuICAgICAgZW51bWVyYWJsZTogZmFsc2UsXG4gICAgICB3cml0YWJsZTogdHJ1ZSxcbiAgICAgIGNvbmZpZ3VyYWJsZTogdHJ1ZVxuICAgIH1cbiAgfSk7XG4gIGlmIChzdXBlckNsYXNzKSBzdWJDbGFzcy5fX3Byb3RvX18gPSBzdXBlckNsYXNzO1xufTtcblxuZXhwb3J0cy5fX2VzTW9kdWxlID0gdHJ1ZTsiLCJ2YXIgJCA9IHJlcXVpcmUoJy4uLy4uL21vZHVsZXMvJCcpO1xubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiBjcmVhdGUoUCwgRCl7XG4gIHJldHVybiAkLmNyZWF0ZShQLCBEKTtcbn07IiwidmFyICQgPSByZXF1aXJlKCcuLi8uLi9tb2R1bGVzLyQnKTtcbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24gZGVmaW5lUHJvcGVydHkoaXQsIGtleSwgZGVzYyl7XG4gIHJldHVybiAkLnNldERlc2MoaXQsIGtleSwgZGVzYyk7XG59OyIsInZhciAkID0gcmVxdWlyZSgnLi4vLi4vbW9kdWxlcy8kJyk7XG5yZXF1aXJlKCcuLi8uLi9tb2R1bGVzL2VzNi5vYmplY3Quc3RhdGljcy1hY2NlcHQtcHJpbWl0aXZlcycpO1xubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiBnZXRPd25Qcm9wZXJ0eURlc2NyaXB0b3IoaXQsIGtleSl7XG4gIHJldHVybiAkLmdldERlc2MoaXQsIGtleSk7XG59OyIsInZhciAkICAgICAgICAgID0gcmVxdWlyZSgnLi8kJylcbiAgLCBnbG9iYWwgICAgID0gJC5nXG4gICwgY29yZSAgICAgICA9ICQuY29yZVxuICAsIGlzRnVuY3Rpb24gPSAkLmlzRnVuY3Rpb247XG5mdW5jdGlvbiBjdHgoZm4sIHRoYXQpe1xuICByZXR1cm4gZnVuY3Rpb24oKXtcbiAgICByZXR1cm4gZm4uYXBwbHkodGhhdCwgYXJndW1lbnRzKTtcbiAgfTtcbn1cbi8vIHR5cGUgYml0bWFwXG4kZGVmLkYgPSAxOyAgLy8gZm9yY2VkXG4kZGVmLkcgPSAyOyAgLy8gZ2xvYmFsXG4kZGVmLlMgPSA0OyAgLy8gc3RhdGljXG4kZGVmLlAgPSA4OyAgLy8gcHJvdG9cbiRkZWYuQiA9IDE2OyAvLyBiaW5kXG4kZGVmLlcgPSAzMjsgLy8gd3JhcFxuZnVuY3Rpb24gJGRlZih0eXBlLCBuYW1lLCBzb3VyY2Upe1xuICB2YXIga2V5LCBvd24sIG91dCwgZXhwXG4gICAgLCBpc0dsb2JhbCA9IHR5cGUgJiAkZGVmLkdcbiAgICAsIGlzUHJvdG8gID0gdHlwZSAmICRkZWYuUFxuICAgICwgdGFyZ2V0ICAgPSBpc0dsb2JhbCA/IGdsb2JhbCA6IHR5cGUgJiAkZGVmLlNcbiAgICAgICAgPyBnbG9iYWxbbmFtZV0gOiAoZ2xvYmFsW25hbWVdIHx8IHt9KS5wcm90b3R5cGVcbiAgICAsIGV4cG9ydHMgID0gaXNHbG9iYWwgPyBjb3JlIDogY29yZVtuYW1lXSB8fCAoY29yZVtuYW1lXSA9IHt9KTtcbiAgaWYoaXNHbG9iYWwpc291cmNlID0gbmFtZTtcbiAgZm9yKGtleSBpbiBzb3VyY2Upe1xuICAgIC8vIGNvbnRhaW5zIGluIG5hdGl2ZVxuICAgIG93biA9ICEodHlwZSAmICRkZWYuRikgJiYgdGFyZ2V0ICYmIGtleSBpbiB0YXJnZXQ7XG4gICAgaWYob3duICYmIGtleSBpbiBleHBvcnRzKWNvbnRpbnVlO1xuICAgIC8vIGV4cG9ydCBuYXRpdmUgb3IgcGFzc2VkXG4gICAgb3V0ID0gb3duID8gdGFyZ2V0W2tleV0gOiBzb3VyY2Vba2V5XTtcbiAgICAvLyBwcmV2ZW50IGdsb2JhbCBwb2xsdXRpb24gZm9yIG5hbWVzcGFjZXNcbiAgICBpZihpc0dsb2JhbCAmJiAhaXNGdW5jdGlvbih0YXJnZXRba2V5XSkpZXhwID0gc291cmNlW2tleV07XG4gICAgLy8gYmluZCB0aW1lcnMgdG8gZ2xvYmFsIGZvciBjYWxsIGZyb20gZXhwb3J0IGNvbnRleHRcbiAgICBlbHNlIGlmKHR5cGUgJiAkZGVmLkIgJiYgb3duKWV4cCA9IGN0eChvdXQsIGdsb2JhbCk7XG4gICAgLy8gd3JhcCBnbG9iYWwgY29uc3RydWN0b3JzIGZvciBwcmV2ZW50IGNoYW5nZSB0aGVtIGluIGxpYnJhcnlcbiAgICBlbHNlIGlmKHR5cGUgJiAkZGVmLlcgJiYgdGFyZ2V0W2tleV0gPT0gb3V0KSFmdW5jdGlvbihDKXtcbiAgICAgIGV4cCA9IGZ1bmN0aW9uKHBhcmFtKXtcbiAgICAgICAgcmV0dXJuIHRoaXMgaW5zdGFuY2VvZiBDID8gbmV3IEMocGFyYW0pIDogQyhwYXJhbSk7XG4gICAgICB9O1xuICAgICAgZXhwLnByb3RvdHlwZSA9IEMucHJvdG90eXBlO1xuICAgIH0ob3V0KTtcbiAgICBlbHNlIGV4cCA9IGlzUHJvdG8gJiYgaXNGdW5jdGlvbihvdXQpID8gY3R4KEZ1bmN0aW9uLmNhbGwsIG91dCkgOiBvdXQ7XG4gICAgLy8gZXhwb3J0XG4gICAgZXhwb3J0c1trZXldID0gZXhwO1xuICAgIGlmKGlzUHJvdG8pKGV4cG9ydHMucHJvdG90eXBlIHx8IChleHBvcnRzLnByb3RvdHlwZSA9IHt9KSlba2V5XSA9IG91dDtcbiAgfVxufVxubW9kdWxlLmV4cG9ydHMgPSAkZGVmOyIsIm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24oJCl7XG4gICQuRlcgICA9IGZhbHNlO1xuICAkLnBhdGggPSAkLmNvcmU7XG4gIHJldHVybiAkO1xufTsiLCIvLyBmYWxsYmFjayBmb3IgSUUxMSBidWdneSBPYmplY3QuZ2V0T3duUHJvcGVydHlOYW1lcyB3aXRoIGlmcmFtZSBhbmQgd2luZG93XHJcbnZhciAkID0gcmVxdWlyZSgnLi8kJylcclxuICAsIHRvU3RyaW5nID0ge30udG9TdHJpbmdcclxuICAsIGdldE5hbWVzID0gJC5nZXROYW1lcztcclxuXHJcbnZhciB3aW5kb3dOYW1lcyA9IHR5cGVvZiB3aW5kb3cgPT0gJ29iamVjdCcgJiYgT2JqZWN0LmdldE93blByb3BlcnR5TmFtZXNcclxuICA/IE9iamVjdC5nZXRPd25Qcm9wZXJ0eU5hbWVzKHdpbmRvdykgOiBbXTtcclxuXHJcbmZ1bmN0aW9uIGdldFdpbmRvd05hbWVzKGl0KXtcclxuICB0cnkge1xyXG4gICAgcmV0dXJuIGdldE5hbWVzKGl0KTtcclxuICB9IGNhdGNoKGUpe1xyXG4gICAgcmV0dXJuIHdpbmRvd05hbWVzLnNsaWNlKCk7XHJcbiAgfVxyXG59XHJcblxyXG5tb2R1bGUuZXhwb3J0cy5nZXQgPSBmdW5jdGlvbiBnZXRPd25Qcm9wZXJ0eU5hbWVzKGl0KXtcclxuICBpZih3aW5kb3dOYW1lcyAmJiB0b1N0cmluZy5jYWxsKGl0KSA9PSAnW29iamVjdCBXaW5kb3ddJylyZXR1cm4gZ2V0V2luZG93TmFtZXMoaXQpO1xyXG4gIHJldHVybiBnZXROYW1lcygkLnRvT2JqZWN0KGl0KSk7XHJcbn07IiwiJ3VzZSBzdHJpY3QnO1xudmFyIGdsb2JhbCA9IHR5cGVvZiBzZWxmICE9ICd1bmRlZmluZWQnID8gc2VsZiA6IEZ1bmN0aW9uKCdyZXR1cm4gdGhpcycpKClcbiAgLCBjb3JlICAgPSB7fVxuICAsIGRlZmluZVByb3BlcnR5ID0gT2JqZWN0LmRlZmluZVByb3BlcnR5XG4gICwgaGFzT3duUHJvcGVydHkgPSB7fS5oYXNPd25Qcm9wZXJ0eVxuICAsIGNlaWwgID0gTWF0aC5jZWlsXG4gICwgZmxvb3IgPSBNYXRoLmZsb29yXG4gICwgbWF4ICAgPSBNYXRoLm1heFxuICAsIG1pbiAgID0gTWF0aC5taW47XG4vLyBUaGUgZW5naW5lIHdvcmtzIGZpbmUgd2l0aCBkZXNjcmlwdG9ycz8gVGhhbmsncyBJRTggZm9yIGhpcyBmdW5ueSBkZWZpbmVQcm9wZXJ0eS5cbnZhciBERVNDID0gISFmdW5jdGlvbigpe1xuICB0cnkge1xuICAgIHJldHVybiBkZWZpbmVQcm9wZXJ0eSh7fSwgJ2EnLCB7Z2V0OiBmdW5jdGlvbigpeyByZXR1cm4gMjsgfX0pLmEgPT0gMjtcbiAgfSBjYXRjaChlKXsgLyogZW1wdHkgKi8gfVxufSgpO1xudmFyIGhpZGUgPSBjcmVhdGVEZWZpbmVyKDEpO1xuLy8gNy4xLjQgVG9JbnRlZ2VyXG5mdW5jdGlvbiB0b0ludGVnZXIoaXQpe1xuICByZXR1cm4gaXNOYU4oaXQgPSAraXQpID8gMCA6IChpdCA+IDAgPyBmbG9vciA6IGNlaWwpKGl0KTtcbn1cbmZ1bmN0aW9uIGRlc2MoYml0bWFwLCB2YWx1ZSl7XG4gIHJldHVybiB7XG4gICAgZW51bWVyYWJsZSAgOiAhKGJpdG1hcCAmIDEpLFxuICAgIGNvbmZpZ3VyYWJsZTogIShiaXRtYXAgJiAyKSxcbiAgICB3cml0YWJsZSAgICA6ICEoYml0bWFwICYgNCksXG4gICAgdmFsdWUgICAgICAgOiB2YWx1ZVxuICB9O1xufVxuZnVuY3Rpb24gc2ltcGxlU2V0KG9iamVjdCwga2V5LCB2YWx1ZSl7XG4gIG9iamVjdFtrZXldID0gdmFsdWU7XG4gIHJldHVybiBvYmplY3Q7XG59XG5mdW5jdGlvbiBjcmVhdGVEZWZpbmVyKGJpdG1hcCl7XG4gIHJldHVybiBERVNDID8gZnVuY3Rpb24ob2JqZWN0LCBrZXksIHZhbHVlKXtcbiAgICByZXR1cm4gJC5zZXREZXNjKG9iamVjdCwga2V5LCBkZXNjKGJpdG1hcCwgdmFsdWUpKTtcbiAgfSA6IHNpbXBsZVNldDtcbn1cblxuZnVuY3Rpb24gaXNPYmplY3QoaXQpe1xuICByZXR1cm4gaXQgIT09IG51bGwgJiYgKHR5cGVvZiBpdCA9PSAnb2JqZWN0JyB8fCB0eXBlb2YgaXQgPT0gJ2Z1bmN0aW9uJyk7XG59XG5mdW5jdGlvbiBpc0Z1bmN0aW9uKGl0KXtcbiAgcmV0dXJuIHR5cGVvZiBpdCA9PSAnZnVuY3Rpb24nO1xufVxuZnVuY3Rpb24gYXNzZXJ0RGVmaW5lZChpdCl7XG4gIGlmKGl0ID09IHVuZGVmaW5lZCl0aHJvdyBUeXBlRXJyb3IoXCJDYW4ndCBjYWxsIG1ldGhvZCBvbiAgXCIgKyBpdCk7XG4gIHJldHVybiBpdDtcbn1cblxudmFyICQgPSBtb2R1bGUuZXhwb3J0cyA9IHJlcXVpcmUoJy4vJC5mdycpKHtcbiAgZzogZ2xvYmFsLFxuICBjb3JlOiBjb3JlLFxuICBodG1sOiBnbG9iYWwuZG9jdW1lbnQgJiYgZG9jdW1lbnQuZG9jdW1lbnRFbGVtZW50LFxuICAvLyBodHRwOi8vanNwZXJmLmNvbS9jb3JlLWpzLWlzb2JqZWN0XG4gIGlzT2JqZWN0OiAgIGlzT2JqZWN0LFxuICBpc0Z1bmN0aW9uOiBpc0Z1bmN0aW9uLFxuICB0aGF0OiBmdW5jdGlvbigpe1xuICAgIHJldHVybiB0aGlzO1xuICB9LFxuICAvLyA3LjEuNCBUb0ludGVnZXJcbiAgdG9JbnRlZ2VyOiB0b0ludGVnZXIsXG4gIC8vIDcuMS4xNSBUb0xlbmd0aFxuICB0b0xlbmd0aDogZnVuY3Rpb24oaXQpe1xuICAgIHJldHVybiBpdCA+IDAgPyBtaW4odG9JbnRlZ2VyKGl0KSwgMHgxZmZmZmZmZmZmZmZmZikgOiAwOyAvLyBwb3coMiwgNTMpIC0gMSA9PSA5MDA3MTk5MjU0NzQwOTkxXG4gIH0sXG4gIHRvSW5kZXg6IGZ1bmN0aW9uKGluZGV4LCBsZW5ndGgpe1xuICAgIGluZGV4ID0gdG9JbnRlZ2VyKGluZGV4KTtcbiAgICByZXR1cm4gaW5kZXggPCAwID8gbWF4KGluZGV4ICsgbGVuZ3RoLCAwKSA6IG1pbihpbmRleCwgbGVuZ3RoKTtcbiAgfSxcbiAgaGFzOiBmdW5jdGlvbihpdCwga2V5KXtcbiAgICByZXR1cm4gaGFzT3duUHJvcGVydHkuY2FsbChpdCwga2V5KTtcbiAgfSxcbiAgY3JlYXRlOiAgICAgT2JqZWN0LmNyZWF0ZSxcbiAgZ2V0UHJvdG86ICAgT2JqZWN0LmdldFByb3RvdHlwZU9mLFxuICBERVNDOiAgICAgICBERVNDLFxuICBkZXNjOiAgICAgICBkZXNjLFxuICBnZXREZXNjOiAgICBPYmplY3QuZ2V0T3duUHJvcGVydHlEZXNjcmlwdG9yLFxuICBzZXREZXNjOiAgICBkZWZpbmVQcm9wZXJ0eSxcbiAgc2V0RGVzY3M6ICAgT2JqZWN0LmRlZmluZVByb3BlcnRpZXMsXG4gIGdldEtleXM6ICAgIE9iamVjdC5rZXlzLFxuICBnZXROYW1lczogICBPYmplY3QuZ2V0T3duUHJvcGVydHlOYW1lcyxcbiAgZ2V0U3ltYm9sczogT2JqZWN0LmdldE93blByb3BlcnR5U3ltYm9scyxcbiAgYXNzZXJ0RGVmaW5lZDogYXNzZXJ0RGVmaW5lZCxcbiAgLy8gRHVtbXksIGZpeCBmb3Igbm90IGFycmF5LWxpa2UgRVMzIHN0cmluZyBpbiBlczUgbW9kdWxlXG4gIEVTNU9iamVjdDogT2JqZWN0LFxuICB0b09iamVjdDogZnVuY3Rpb24oaXQpe1xuICAgIHJldHVybiAkLkVTNU9iamVjdChhc3NlcnREZWZpbmVkKGl0KSk7XG4gIH0sXG4gIGhpZGU6IGhpZGUsXG4gIGRlZjogY3JlYXRlRGVmaW5lcigwKSxcbiAgc2V0OiBnbG9iYWwuU3ltYm9sID8gc2ltcGxlU2V0IDogaGlkZSxcbiAgZWFjaDogW10uZm9yRWFjaFxufSk7XG4vKiBlc2xpbnQtZGlzYWJsZSBuby11bmRlZiAqL1xuaWYodHlwZW9mIF9fZSAhPSAndW5kZWZpbmVkJylfX2UgPSBjb3JlO1xuaWYodHlwZW9mIF9fZyAhPSAndW5kZWZpbmVkJylfX2cgPSBnbG9iYWw7IiwidmFyICQgICAgICAgID0gcmVxdWlyZSgnLi8kJylcbiAgLCAkZGVmICAgICA9IHJlcXVpcmUoJy4vJC5kZWYnKVxuICAsIGlzT2JqZWN0ID0gJC5pc09iamVjdFxuICAsIHRvT2JqZWN0ID0gJC50b09iamVjdDtcbiQuZWFjaC5jYWxsKCgnZnJlZXplLHNlYWwscHJldmVudEV4dGVuc2lvbnMsaXNGcm96ZW4saXNTZWFsZWQsaXNFeHRlbnNpYmxlLCcgK1xuICAnZ2V0T3duUHJvcGVydHlEZXNjcmlwdG9yLGdldFByb3RvdHlwZU9mLGtleXMsZ2V0T3duUHJvcGVydHlOYW1lcycpLnNwbGl0KCcsJylcbiwgZnVuY3Rpb24oS0VZLCBJRCl7XG4gIHZhciBmbiAgICAgPSAoJC5jb3JlLk9iamVjdCB8fCB7fSlbS0VZXSB8fCBPYmplY3RbS0VZXVxuICAgICwgZm9yY2VkID0gMFxuICAgICwgbWV0aG9kID0ge307XG4gIG1ldGhvZFtLRVldID0gSUQgPT0gMCA/IGZ1bmN0aW9uIGZyZWV6ZShpdCl7XG4gICAgcmV0dXJuIGlzT2JqZWN0KGl0KSA/IGZuKGl0KSA6IGl0O1xuICB9IDogSUQgPT0gMSA/IGZ1bmN0aW9uIHNlYWwoaXQpe1xuICAgIHJldHVybiBpc09iamVjdChpdCkgPyBmbihpdCkgOiBpdDtcbiAgfSA6IElEID09IDIgPyBmdW5jdGlvbiBwcmV2ZW50RXh0ZW5zaW9ucyhpdCl7XG4gICAgcmV0dXJuIGlzT2JqZWN0KGl0KSA/IGZuKGl0KSA6IGl0O1xuICB9IDogSUQgPT0gMyA/IGZ1bmN0aW9uIGlzRnJvemVuKGl0KXtcbiAgICByZXR1cm4gaXNPYmplY3QoaXQpID8gZm4oaXQpIDogdHJ1ZTtcbiAgfSA6IElEID09IDQgPyBmdW5jdGlvbiBpc1NlYWxlZChpdCl7XG4gICAgcmV0dXJuIGlzT2JqZWN0KGl0KSA/IGZuKGl0KSA6IHRydWU7XG4gIH0gOiBJRCA9PSA1ID8gZnVuY3Rpb24gaXNFeHRlbnNpYmxlKGl0KXtcbiAgICByZXR1cm4gaXNPYmplY3QoaXQpID8gZm4oaXQpIDogZmFsc2U7XG4gIH0gOiBJRCA9PSA2ID8gZnVuY3Rpb24gZ2V0T3duUHJvcGVydHlEZXNjcmlwdG9yKGl0LCBrZXkpe1xuICAgIHJldHVybiBmbih0b09iamVjdChpdCksIGtleSk7XG4gIH0gOiBJRCA9PSA3ID8gZnVuY3Rpb24gZ2V0UHJvdG90eXBlT2YoaXQpe1xuICAgIHJldHVybiBmbihPYmplY3QoJC5hc3NlcnREZWZpbmVkKGl0KSkpO1xuICB9IDogSUQgPT0gOCA/IGZ1bmN0aW9uIGtleXMoaXQpe1xuICAgIHJldHVybiBmbih0b09iamVjdChpdCkpO1xuICB9IDogcmVxdWlyZSgnLi8kLmdldC1uYW1lcycpLmdldDtcbiAgdHJ5IHtcbiAgICBmbigneicpO1xuICB9IGNhdGNoKGUpe1xuICAgIGZvcmNlZCA9IDE7XG4gIH1cbiAgJGRlZigkZGVmLlMgKyAkZGVmLkYgKiBmb3JjZWQsICdPYmplY3QnLCBtZXRob2QpO1xufSk7Il19
