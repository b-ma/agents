"use strict";

var _babelHelpers = require("babel-runtime/helpers")["default"];

var Vector = require("vector");
var utils = require("./utils");

var SteeringBehaviors = (function () {
  function SteeringBehaviors(vehicle) {
    _babelHelpers.classCallCheck(this, SteeringBehaviors);

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

  _babelHelpers.prototypeProperties(SteeringBehaviors, null, {
    calculate: {
      value: function calculate() {
        if (!this.vehicle.world.start) {
          return new Vector();
        }

        var steerings = new Vector();

        if (this._seek && this.vehicle.world.target) {
          var steering = this.seek(this.vehicle.world.target);
          steerings.add(steering);
        }

        if (this._flee && this.vehicle.world.target) {
          var steering = this.flee(this.vehicle.world.target);
          steerings.add(steering);
        }

        if (this._arrive && this.vehicle.world.target) {
          var steering = this.arrive(this.vehicle.world.target, 3);
          steerings.add(steering);
        }

        if (this._pursuit) {
          var steering = this.pursuit(this.vehicle.world.evader);
          steerings.add(steering);
        }

        if (this._evade) {
          var steering = this.evade(this.vehicle.world.pursuer);
          steerings.add(steering);
        }

        if (this._wander) {
          var steering = this.wander();
          steerings.add(steering);
        }

        if (this._wallAvoidance) {
          var steering = this.wallAvoidance(this.vehicle.world.walls);
          steerings.add(steering);
        }

        return steerings;
      },
      writable: true,
      configurable: true
    },
    seekOn: {

      // behaviors On/Off
      // --------------------------------------------

      value: function seekOn() {
        this._seek = true;
      },
      writable: true,
      configurable: true
    },
    seekOff: {
      value: function seekOff() {
        this._seek = false;
      },
      writable: true,
      configurable: true
    },
    fleeOn: {
      value: function fleeOn() {
        this._flee = true;
      },
      writable: true,
      configurable: true
    },
    fleeOff: {
      value: function fleeOff() {
        this._flee = false;
      },
      writable: true,
      configurable: true
    },
    arriveOn: {
      value: function arriveOn() {
        this._arrive = true;
      },
      writable: true,
      configurable: true
    },
    arriveOff: {
      value: function arriveOff() {
        this._arrive = false;
      },
      writable: true,
      configurable: true
    },
    pursuitOn: {
      value: function pursuitOn() {
        this._pursuit = true;
      },
      writable: true,
      configurable: true
    },
    pursuitOff: {
      value: function pursuitOff() {
        this._pursuit = false;
      },
      writable: true,
      configurable: true
    },
    evadeOn: {
      value: function evadeOn() {
        this._evade = true;
      },
      writable: true,
      configurable: true
    },
    evadeOff: {
      value: function evadeOff() {
        this._evade = false;
      },
      writable: true,
      configurable: true
    },
    wanderOn: {
      value: function wanderOn() {
        this._wander = true;
      },
      writable: true,
      configurable: true
    },
    wanderOff: {
      value: function wanderOff() {
        this._wander = false;
      },
      writable: true,
      configurable: true
    },
    wallAvoidanceOn: {
      value: function wallAvoidanceOn() {
        this._wallAvoidance = true;
      },
      writable: true,
      configurable: true
    },
    wallAvoidanceOff: {
      value: function wallAvoidanceOff() {
        this._wallAvoidance = false;
      },
      writable: true,
      configurable: true
    },
    seek: {

      // behaviors
      // --------------------------------------------

      value: function seek(targetPosition) {
        var desiredVelocity = Vector.substract(targetPosition, this.vehicle.position).normalize().multiply(this.vehicle.maxSpeed);

        var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
        return steering;
      },
      writable: true,
      configurable: true
    },
    flee: {
      value: function flee(targetPosition) {
        var panicDistance = 100 * 100; // use square domain to save computations
        if (Vector.distanceSqrt(this.vehicle.position, targetPosition) > panicDistance) {
          return new Vector();
        }

        var desiredVelocity = Vector.substract(this.vehicle.position, targetPosition).normalize().multiply(this.vehicle.maxSpeed);

        var steering = Vector.substract(desiredVelocity, this.vehicle.velocity);
        return steering;
      },
      writable: true,
      configurable: true
    },
    arrive: {

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
      },
      writable: true,
      configurable: true
    },
    pursuit: {
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
      },
      writable: true,
      configurable: true
    },
    evade: {
      value: function evade(pursuer) {
        // no need to check if the agents are facing
        var toPursuer = Vector.substract(pursuer.position, this.vehicle.position);
        // then same as pursuit but replace seek with flee
        var lookAheadTime = toPursuer.magnitude() / (this.vehicle.maxSpeed + pursuer.speed());
        var predictedPosition = Vector.add(pursuer.position, Vector.multiply(pursuer.velocity, lookAheadTime));
        return this.flee(predictedPosition);
      },
      writable: true,
      configurable: true
    },
    wander: {
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
      },
      writable: true,
      configurable: true
    },
    obstacleAvoidance: {

      // prevent the agent colliding with the closest obstacle

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
          var sqrtPart = Math.sqrt(expendedRadius * expendedRadius - cY * cY);

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
      },
      writable: true,
      configurable: true
    },
    wallAvoidance: {
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
      },
      writable: true,
      configurable: true
    },
    createFeelers: {

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
      },
      writable: true,
      configurable: true
    },
    debugFeelers: {

      // visualize - debug tools
      // all debug stuff should be in sterring

      value: function debugFeelers(ctx) {
        this.feelers.forEach(function (feeler) {
          ctx.save();
          ctx.beginPath();

          ctx.strokeStyle = "#acacac";
          ctx.moveTo(this.vehicle.position.x, this.vehicle.position.y);
          ctx.lineTo(feeler.x, feeler.y);
          ctx.stroke();

          ctx.closePath();
          ctx.restore();
        }, this);
      },
      writable: true,
      configurable: true
    },
    debugWander: {
      value: function debugWander(ctx) {
        ctx.save();
        ctx.translate(this.vehicle.position.x, this.vehicle.position.y);
        ctx.rotate(this.vehicle.heading.direction());

        // move to wander circle center
        ctx.translate(this.wanderDistance, 0);

        ctx.strokeStyle = "red";
        ctx.fillStyle = "red";
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
      },
      writable: true,
      configurable: true
    }
  });

  return SteeringBehaviors;
})();

module.exports = SteeringBehaviors;

// and the two agents are approx face to face

//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4vbGliL3N0ZWVyaW5nLWJlaGF2aW9ycy5lczYuanMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7OztBQUFBLElBQUksTUFBTSxHQUFHLE9BQU8sQ0FBQyxRQUFRLENBQUMsQ0FBQztBQUMvQixJQUFJLEtBQUssR0FBSSxPQUFPLENBQUMsU0FBUyxDQUFDLENBQUM7O0lBRTFCLGlCQUFpQjtBQUNWLFdBRFAsaUJBQWlCLENBQ1QsT0FBTzt1Q0FEZixpQkFBaUI7O0FBRW5CLFFBQUksQ0FBQyxPQUFPLEdBQUcsT0FBTyxDQUFDOztBQUV2QixRQUFJLENBQUMsTUFBTSxHQUFHO0FBQ1osMkJBQXFCLEVBQUUsRUFBRTtBQUN6QiwrQkFBeUIsRUFBRSxFQUFFOztLQUU5QixDQUFDOzs7QUFHRixRQUFJLENBQUMsT0FBTyxHQUFHLEVBQUUsQ0FBQzs7QUFFbEIsUUFBSSxDQUFDLFlBQVksR0FBRyxDQUFDLENBQUM7QUFDdEIsUUFBSSxDQUFDLGNBQWMsR0FBRyxFQUFFLENBQUM7QUFDekIsUUFBSSxDQUFDLFlBQVksR0FBRyxDQUFDLENBQUM7O0FBRXRCLFFBQUksS0FBSyxHQUFHLEtBQUssQ0FBQyxJQUFJLEVBQUUsR0FBRyxLQUFLLENBQUMsS0FBSyxDQUFDOztBQUV2QyxRQUFJLENBQUMsWUFBWSxHQUFHLElBQUksTUFBTSxDQUFDLElBQUksQ0FBQyxZQUFZLEdBQUcsSUFBSSxDQUFDLEdBQUcsQ0FBQyxLQUFLLENBQUMsRUFDbkMsSUFBSSxDQUFDLFlBQVksR0FBRyxJQUFJLENBQUMsR0FBRyxDQUFDLEtBQUssQ0FBQyxDQUFDLENBQUM7R0FFckU7O29DQXRCRyxpQkFBaUI7QUF3QnJCLGFBQVM7YUFBQSxxQkFBRztBQUNWLFlBQUksQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLEtBQUssQ0FBQyxLQUFLLEVBQUU7QUFDN0IsaUJBQU8sSUFBSSxNQUFNLEVBQUUsQ0FBQztTQUNyQjs7QUFFRCxZQUFJLFNBQVMsR0FBRyxJQUFJLE1BQU0sRUFBRSxDQUFD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QyxDQUFDO0FBQzVFLFdBQUcsQ0FBQyxNQUFNLEVBQUUsQ0FBQztBQUNiLFdBQUcsQ0FBQyxTQUFTLEVBQUUsQ0FBQzs7QUFFaEIsV0FBRyxDQUFDLE9BQU8sRUFBRSxDQUFDO09BQ2Y7Ozs7OztTQTNZRyxpQkFBaUI7OztBQThZdkIsTUFBTSxDQUFDLE9BQU8sR0FBRyxpQkFBaUIsQ0FBQyIsImZpbGUiOiIuL2xpYi9zdGVlcmluZy1iZWhhdmlvcnMuanMiLCJzb3VyY2VzQ29udGVudCI6WyJ2YXIgVmVjdG9yID0gcmVxdWlyZSgndmVjdG9yJyk7XG52YXIgdXRpbHMgID0gcmVxdWlyZSgnLi91dGlscycpO1xuXG5jbGFzcyBTdGVlcmluZ0JlaGF2aW9ycyB7XG4gIGNvbnN0cnVjdG9yKHZlaGljbGUpIHtcbiAgICB0aGlzLnZlaGljbGUgPSB2ZWhpY2xlO1xuXG4gICAgdGhpcy5wYXJhbXMgPSB7XG4gICAgICBtaW5EZXRlY3Rpb25Cb3hMZW5ndGg6IDQwLFxuICAgICAgd2FsbERldGVjdGlvbkZlZWxlckxlbmd0aDogNDBcblxuICAgIH07XG5cbiAgICAvLyBhbnRlbm5hIHVzZWQgaW4gd2FsbCBhdm9pZGFuY2VcbiAgICB0aGlzLmZlZWxlcnMgPSBbXTtcbiAgICAvLyB2YXJpYWJsZXMgZm9yIHdhbmRlciBiZWhhdmlvclxuICAgIHRoaXMud2FuZGVyUmFkaXVzID0gNjtcbiAgICB0aGlzLndhbmRlckRpc3RhbmNlID0gMTU7XG4gICAgdGhpcy53YW5kZXJKaXR0ZXIgPSAyO1xuICAgIC8vIGluaXRpYWxpemUgdGFyZ2V0IC0+IGNpcmNsZSBvZiB3YW5kZXJSYWRpdXMgY2VudGVyZWQgb24gdGhlIGFnZW50XG4gICAgdmFyIHRoZXRhID0gdXRpbHMucmFuZCgpICogdXRpbHMuVHdvUEk7XG4gICAgLy8gcHJvamVjdCB0aGV0YSBvbiB0aGUgd2FuZGVyIGNpcmNsZVxuICAgIHRoaXMud2FuZGVyVGFyZ2V0ID0gbmV3IFZlY3Rvcih0aGlzLndhbmRlclJhZGl1cyAqIE1hdGguY29zKHRoZXRhKSxcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy53YW5kZXJSYWRpdXMgKiBNYXRoLnNpbih0aGV0YSkpO1xuXG4gIH1cblxuICBjYWxjdWxhdGUoKSB7XG4gICAgaWYgKCF0aGlzLnZlaGljbGUud29ybGQuc3RhcnQpIHtcbiAgICAgIHJldHVybiBuZXcgVmVjdG9yKCk7XG4gICAgfVxuXG4gICAgdmFyIHN0ZWVyaW5ncyA9IG5ldyBWZWN0b3IoKTtcblxuICAgIGlmICh0aGlzLl9zZWVrICYmIHRoaXMudmVoaWNsZS53b3JsZC50YXJnZXQpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMuc2Vlayh0aGlzLnZlaGljbGUud29ybGQudGFyZ2V0KTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIGlmICh0aGlzLl9mbGVlICYmIHRoaXMudmVoaWNsZS53b3JsZC50YXJnZXQpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMuZmxlZSh0aGlzLnZlaGljbGUud29ybGQudGFyZ2V0KTtcbiAgICAgIHN0ZWVyaW5ncy5hZGQoc3RlZXJpbmcpO1xuICAgIH1cblxuICAgIGlmICh0aGlzLl9hcnJpdmUgJiYgdGhpcy52ZWhpY2xlLndvcmxkLnRhcmdldCkge1xuICAgICAgdmFyIHN0ZWVyaW5nID0gdGhpcy5hcnJpdmUodGhpcy52ZWhpY2xlLndvcmxkLnRhcmdldCwgMyk7XG4gICAgICBzdGVlcmluZ3MuYWRkKHN0ZWVyaW5nKTtcbiAgICB9XG5cbiAgICBpZiAodGhpcy5fcHVyc3VpdCkge1xuICAgICAgdmFyIHN0ZWVyaW5nID0gdGhpcy5wdXJzdWl0KHRoaXMudmVoaWNsZS53b3JsZC5ldmFkZXIpO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgaWYgKHRoaXMuIF9ldmFkZSkge1xuICAgICAgdmFyIHN0ZWVyaW5nID0gdGhpcy5ldmFkZSh0aGlzLnZlaGljbGUud29ybGQucHVyc3Vlcik7XG4gICAgICBzdGVlcmluZ3MuYWRkKHN0ZWVyaW5nKTtcbiAgICB9XG5cbiAgICBpZiAodGhpcy5fd2FuZGVyKSB7XG4gICAgICB2YXIgc3RlZXJpbmcgPSB0aGlzLndhbmRlcigpO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgaWYgKHRoaXMuX3dhbGxBdm9pZGFuY2UpIHtcbiAgICAgIHZhciBzdGVlcmluZyA9IHRoaXMud2FsbEF2b2lkYW5jZSh0aGlzLnZlaGljbGUud29ybGQud2FsbHMpO1xuICAgICAgc3RlZXJpbmdzLmFkZChzdGVlcmluZyk7XG4gICAgfVxuXG4gICAgcmV0dXJuIHN0ZWVyaW5ncztcbiAgfVxuXG4gIC8vIGJlaGF2aW9ycyBPbi9PZmZcbiAgLy8gLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS1cblxuICBzZWVrT24oKSB7IHRoaXMuX3NlZWsgPSB0cnVlOyB9XG4gIHNlZWtPZmYoKSB7IHRoaXMuX3NlZWsgPSBmYWxzZTsgfVxuXG4gIGZsZWVPbigpIHsgdGhpcy5fZmxlZSA9IHRydWU7IH1cbiAgZmxlZU9mZigpIHsgdGhpcy5fZmxlZSA9IGZhbHNlOyB9XG5cbiAgYXJyaXZlT24oKSB7IHRoaXMuX2Fycml2ZSA9IHRydWU7IH1cbiAgYXJyaXZlT2ZmKCkgeyB0aGlzLl9hcnJpdmUgPSBmYWxzZTsgfVxuXG4gIHB1cnN1aXRPbigpIHsgdGhpcy5fcHVyc3VpdCA9IHRydWU7IH1cbiAgcHVyc3VpdE9mZigpIHsgdGhpcy5fcHVyc3VpdCA9IGZhbHNlOyB9XG5cbiAgZXZhZGVPbigpIHsgdGhpcy5fZXZhZGUgPSB0cnVlOyB9XG4gIGV2YWRlT2ZmKCkgeyB0aGlzLl9ldmFkZSA9IGZhbHNlOyB9XG5cbiAgd2FuZGVyT24oKSB7IHRoaXMuX3dhbmRlciA9IHRydWU7IH1cbiAgd2FuZGVyT2ZmKCkgeyB0aGlzLl93YW5kZXIgPSBmYWxzZTsgfVxuXG4gIHdhbGxBdm9pZGFuY2VPbigpIHsgdGhpcy5fd2FsbEF2b2lkYW5jZSA9IHRydWU7IH1cbiAgd2FsbEF2b2lkYW5jZU9mZigpIHsgdGhpcy5fd2FsbEF2b2lkYW5jZSA9IGZhbHNlOyB9XG5cbiAgLy8gYmVoYXZpb3JzXG4gIC8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG5cbiAgc2Vlayh0YXJnZXRQb3NpdGlvbikge1xuICAgIHZhciBkZXNpcmVkVmVsb2NpdHkgPSBWZWN0b3Iuc3Vic3RyYWN0KHRhcmdldFBvc2l0aW9uLCB0aGlzLnZlaGljbGUucG9zaXRpb24pXG4gICAgICAubm9ybWFsaXplKClcbiAgICAgIC5tdWx0aXBseSh0aGlzLnZlaGljbGUubWF4U3BlZWQpO1xuXG4gICAgdmFyIHN0ZWVyaW5nID0gVmVjdG9yLnN1YnN0cmFjdChkZXNpcmVkVmVsb2NpdHksIHRoaXMudmVoaWNsZS52ZWxvY2l0eSk7XG4gICAgcmV0dXJuIHN0ZWVyaW5nO1xuICB9XG5cbiAgZmxlZSh0YXJnZXRQb3NpdGlvbikge1xuICAgIHZhciBwYW5pY0Rpc3RhbmNlID0gMTAwICogMTAwOyAvLyB1c2Ugc3F1YXJlIGRvbWFpbiB0byBzYXZlIGNvbXB1dGF0aW9uc1xuICAgIGlmIChWZWN0b3IuZGlzdGFuY2VTcXJ0KHRoaXMudmVoaWNsZS5wb3NpdGlvbiwgdGFyZ2V0UG9zaXRpb24pID4gcGFuaWNEaXN0YW5jZSkge1xuICAgICAgcmV0dXJuIG5ldyBWZWN0b3IoKTtcbiAgICB9XG5cbiAgICB2YXIgZGVzaXJlZFZlbG9jaXR5ID0gVmVjdG9yLnN1YnN0cmFjdCh0aGlzLnZlaGljbGUucG9zaXRpb24sIHRhcmdldFBvc2l0aW9uKVxuICAgICAgLm5vcm1hbGl6ZSgpXG4gICAgICAubXVsdGlwbHkodGhpcy52ZWhpY2xlLm1heFNwZWVkKTtcblxuICAgIHZhciBzdGVlcmluZyA9IFZlY3Rvci5zdWJzdHJhY3QoZGVzaXJlZFZlbG9jaXR5LCB0aGlzLnZlaGljbGUudmVsb2NpdHkpO1xuICAgIHJldHVybiBzdGVlcmluZztcbiAgfVxuXG4gIC8vIGRlY2VsZXJhdGlvbiBpcyBhbiBlbnVtIChzbG93ID0gMSwgbm9ybWFsID0gMiwgZmFzdCA9IDMpXG4gIC8vIGl0IGRlc2NyaWJlcyB0aGUgdGltZSB0aGUgYWdlbnQgc2hvdWxkIHRha2UgdG8gYXJyaXZlIGF0IGRlc3RpbmF0aW9uXG4gIGFycml2ZSh0YXJnZXRQb3NpdGlvbiwgZGVjZWxlcmF0aW9uKSB7XG4gICAgdmFyIHRvVGFyZ2V0ID0gVmVjdG9yLnN1YnN0cmFjdCh0YXJnZXRQb3NpdGlvbiwgdGhpcy52ZWhpY2xlLnBvc2l0aW9uKTtcbiAgICB2YXIgZGlzdGFuY2UgPSB0b1RhcmdldC5tYWduaXR1ZGUoKTtcblxuICAgIGlmIChkaXN0YW5jZSA+IDApIHtcbiAgICAgIC8vIGFsbG93IHRvIHR3ZWFrIGRlY2VsZXJhdGlvblxuICAgICAgdmFyIGRlY2VsZXJhdGlvblR3ZWFrZXIgPSAwLjM7XG4gICAgICAvLyBkZWZpbmUgdGhlIHNwZWVkIHRoZSBhZ2VudCBzaG91bGQgaGF2ZSB0byBhcnJpdmUgYXQgZGVzdGluYXRpb25cbiAgICAgIHZhciBzcGVlZCA9IGRpc3RhbmNlIC8gKGRlY2VsZXJhdGlvbiAqIGRlY2VsZXJhdGlvblR3ZWFrZXIpO1xuICAgICAgLy8gc3BlZWQgc2hvdWxkbid0IGV4Y2VlZCBtYXhTcGVlZFxuICAgICAgc3BlZWQgPSBNYXRoLm1pbihzcGVlZCwgdGhpcy52ZWhpY2xlLm1heFNwZWVkKTtcbiAgICAgIC8vIG5leHQgc3RlcHMgYXJlIHNhbWUgYXMgc2Vla1xuICAgICAgdmFyIGRlc2lyZWRWZWxvY2l0eSA9IHRvVGFyZ2V0XG4gICAgICAgIC5kaXZpZGUoZGlzdGFuY2UpIC8vIDw9PiB0b1RhcmdldC5ub3JtYWxpemUoKTtcbiAgICAgICAgLm11bHRpcGx5KHNwZWVkKTtcblxuICAgICAgdmFyIHN0ZWVyaW5nID0gVmVjdG9yLnN1YnN0cmFjdChkZXNpcmVkVmVsb2NpdHksIHRoaXMudmVoaWNsZS52ZWxvY2l0eSk7XG4gICAgICByZXR1cm4gc3RlZXJpbmc7XG4gICAgfVxuXG4gICAgcmV0dXJuIG5ldyBWZWN0b3IoKTtcbiAgfVxuXG4gIHB1cnN1aXQoZXZhZGVyKSB7XG4gICAgdmFyIHRvRXZhZGVyID0gVmVjdG9yLnN1YnN0cmFjdChldmFkZXIucG9zaXRpb24sIHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gICAgLy8gY29zaW5lIG9mIHRoZSBhbmdsZSBiZXR3ZWVuIHRoZSAyIGFnZW50cyBoZWFkaW5nc1xuICAgIHZhciByZWxhdGl2ZUhlYWRpbmcgPSBWZWN0b3IuZG90KGV2YWRlci5oZWFkaW5nLCB0aGlzLnZlaGljbGUuaGVhZGluZyk7XG5cbiAgICBpZiAoXG4gICAgICAvLyBpZiB0aGUgdGhlIGV2YWRlciBpcyBpbiBmcm9udCBvZiB0aGUgcHVyc3VlclxuICAgICAgKFZlY3Rvci5kb3QodGhpcy52ZWhpY2xlLmhlYWRpbmcsIHRvRXZhZGVyKSA+IDApICYmXG4gICAgICAvLyBhbmQgdGhlIHR3byBhZ2VudHMgYXJlIGFwcHJveCBmYWNlIHRvIGZhY2VcbiAgICAgIChyZWxhdGl2ZUhlYWRpbmcgPCAtMC45NSkgLy8gYWNvcygwLjk1KSA9IDE4ZGVnXG4gICAgKSB7XG4gICAgICByZXR1cm4gdGhpcy5zZWVrKGV2YWRlci5wb3NpdGlvbik7XG4gICAgfVxuXG4gICAgLy8gbm90IGNvbnNpcmRlcmVkIGFoZWFkIHNvIHdlIGhhdmUgdG9cbiAgICAvLyBwcmVkaWN0IHRoZSBmdXR1cmUgcG9zaXRpb24gb2YgdGhlIGV2YWRlclxuXG4gICAgLy8gdGhlIGxvb2tBaGVhZFRpbWUgc2hvdWxkIGJlIHByb3BvcnRpb25uYWwgdG8gdGhlIGRpc3RhbmNlLFxuICAgIC8vIGFuZCBpbnZlcnNseSBwcm9wb3J0aW9ubmFsIHRvIHRoZSBzcGVlZCBvZiB0aGUgYWdlbnRzXG4gICAgdmFyIGxvb2tBaGVhZFRpbWUgPSB0b0V2YWRlci5tYWduaXR1ZGUoKSAvICh0aGlzLnZlaGljbGUubWF4U3BlZWQgKyBldmFkZXIuc3BlZWQoKSk7XG4gICAgLy8gbm93IHNlZWsgdG8gYSBwcmVkaWN0aW9uIG9mIHRoZSBldmFkZXIgcG9zaXRpb25cbiAgICB2YXIgcHJlZGljdGVkUG9zaXRpb24gPSBWZWN0b3IuYWRkKGV2YWRlci5wb3NpdGlvbiwgVmVjdG9yLm11bHRpcGx5KGV2YWRlci52ZWxvY2l0eSwgbG9va0FoZWFkVGltZSkpO1xuICAgIHJldHVybiB0aGlzLnNlZWsocHJlZGljdGVkUG9zaXRpb24pO1xuICB9XG5cbiAgZXZhZGUocHVyc3Vlcikge1xuICAgIC8vIG5vIG5lZWQgdG8gY2hlY2sgaWYgdGhlIGFnZW50cyBhcmUgZmFjaW5nXG4gICAgdmFyIHRvUHVyc3VlciA9IFZlY3Rvci5zdWJzdHJhY3QocHVyc3Vlci5wb3NpdGlvbiwgdGhpcy52ZWhpY2xlLnBvc2l0aW9uKTtcbiAgICAvLyB0aGVuIHNhbWUgYXMgcHVyc3VpdCBidXQgcmVwbGFjZSBzZWVrIHdpdGggZmxlZVxuICAgIHZhciBsb29rQWhlYWRUaW1lID0gdG9QdXJzdWVyLm1hZ25pdHVkZSgpIC8gKHRoaXMudmVoaWNsZS5tYXhTcGVlZCArIHB1cnN1ZXIuc3BlZWQoKSk7XG4gICAgdmFyIHByZWRpY3RlZFBvc2l0aW9uID0gVmVjdG9yLmFkZChwdXJzdWVyLnBvc2l0aW9uLCBWZWN0b3IubXVsdGlwbHkocHVyc3Vlci52ZWxvY2l0eSwgbG9va0FoZWFkVGltZSkpO1xuICAgIHJldHVybiB0aGlzLmZsZWUocHJlZGljdGVkUG9zaXRpb24pO1xuICB9XG5cbiAgd2FuZGVyKCkge1xuICAgIC8vIGFkZCBhIHNtYWxsIHJhbmRvbSB2ZWN0b3IgdG8gdGhlIHdhbmRlclRhcmdldFxuICAgIHZhciByYW5kb21WZWN0b3IgPSBuZXcgVmVjdG9yKHV0aWxzLnJhbmRDbGFtcGVkKCkgKiB0aGlzLndhbmRlckppdHRlcixcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICB1dGlscy5yYW5kQ2xhbXBlZCgpICogdGhpcy53YW5kZXJKaXR0ZXIpO1xuICAgIHRoaXMud2FuZGVyVGFyZ2V0LmFkZChyYW5kb21WZWN0b3IpO1xuICAgIC8vIHJlcHJvamVjdCB0aGUgd2FuZGVyVGFyZ2V0IG9uIHRoZSB3YW5kZXIgY2lyY2xlXG4gICAgdGhpcy53YW5kZXJUYXJnZXRcbiAgICAgIC5ub3JtYWxpemUoKVxuICAgICAgLm11bHRpcGx5KHRoaXMud2FuZGVyUmFkaXVzKTtcbiAgICAvLyBwcm9qZWN0IHRoZSB3YW5kZXIgY2lyY2xlIGluIGZyb250IG9mIHRoZSBhZ2VudFxuICAgIHZhciB0YXJnZXRMb2NhbCA9IFZlY3Rvci5hZGQodGhpcy53YW5kZXJUYXJnZXQsIG5ldyBWZWN0b3IodGhpcy53YW5kZXJEaXN0YW5jZSwgMCkpO1xuICAgIC8vIEBUT0RPIHNob3VsZCBiZSBhbiBtYXRyaXggdHJhbnNmb3JtcyB1dGlsc1xuICAgIC8vIHByb2plY3QgdGhlIHRhcmdldCBpbiB3b3JsZCBzcGFjZSAtIGNoYW5nZSBuYW1lIGZvciB1bmRlcnN0YW5kYWJpbGl0eVxuICAgIHZhciB0YXJnZXRXb3JsZCA9IHRhcmdldExvY2FsO1xuICAgIC8vIHJvdGF0ZVxuICAgIHRhcmdldFdvcmxkLnJvdGF0ZSh0aGlzLnZlaGljbGUuaGVhZGluZy5kaXJlY3Rpb24oKSk7XG4gICAgLy8gdHJhbnNsYXRlXG4gICAgdGFyZ2V0V29ybGQuYWRkKHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gICAgLy8gc3RlZXIgdG93YXJkIHRoaXMgdGFyZ2V0XG4gICAgcmV0dXJuIFZlY3Rvci5zdWJzdHJhY3QodGFyZ2V0V29ybGQsIHRoaXMudmVoaWNsZS5wb3NpdGlvbik7XG4gIH1cblxuICAvLyBwcmV2ZW50IHRoZSBhZ2VudCBjb2xsaWRpbmcgd2l0aCB0aGUgY2xvc2VzdCBvYnN0YWNsZVxuICBvYnN0YWNsZUF2b2lkYW5jZShvYnN0YWNsZXMpIHtcbiAgICAvLyBjcmVhdGUgYSBkZXRlY3Rpb24gYm94IHByb3BvcnRpb25uYWwgdG8gdGhlIGFnZW50IHZlbG9jaXR5XG4gICAgdmFyIGJveExlbmd0aCA9IHRoaXMucGFyYW1zLm1pbkRldGVjdGlvbkJveExlbmd0aCArXG4gICAgICAgICAgICAgICAgICAgIHRoaXMudmVoaWNsZS5zcGVlZCgpIC8gdGhpcy52ZWhpY2xlLm1heFNwZWVkICpcbiAgICAgICAgICAgICAgICAgICAgdGhpcy5wYXJhbXMubWluRGV0ZWN0aW9uQm94TGVuZ3RoO1xuXG4gICAgdGhpcy52ZWhpY2xlLndvcmxkLnRhZ09ic3RhY2xlc1dpdGhpblJhbmdlKHRoaXMudmVoaWNsZSwgYm94TGVuZ3RoKTtcbiAgICAvLyBjbG9zZXN0IGludGVyc2VjdGluZyBvYnN0YWNsZSAoQ0lPKVxuICAgIHZhciBjbG9zZXN0SW50ZXJjZXB0aW5nT2JzdGFjbGUgPSBudWxsO1xuICAgIHZhciBkaXN0YW5jZVRvQ0lPID0gK0luZmluaXR5O1xuICAgIHZhciBsb2NhbFBvc2l0aW9uT2ZDSU8gPSBudWxsO1xuXG4gICAgb2JzdGFjbGVzLmZvckVhY2goZnVuY3Rpb24ob2JzdGFjbGUpIHtcbiAgICAgIGlmICghb2JzdGFjbGUuaXNUYWdnZWQoKSkgeyByZXR1cm47IH1cbiAgICAgIC8vIGZpbmQgbG9jYWwgY29vcmRpbmF0ZXMgb2YgdGhlIG9ic3RhY2xlXG4gICAgICB2YXIgbG9jYWxQb3MgPSBvYnN0YWNsZS5wb3NpdGlvbi5jbG9uZSgpO1xuICAgICAgLy8gcm90YXRlXG4gICAgICBsb2NhbFBvcy5yb3RhdGUodGhpcy52ZWhpY2xlLmhlYWRpbmcuZGlyZWN0aW9uKCkgKiAtMSk7XG4gICAgICAvLyB0cmFuc2xhdGVcbiAgICAgIGxvY2FsUG9zLnN1YnN0cmFjdCh0aGlzLnZlaGljbGUucG9zaXRpb24pO1xuICAgICAgLy8gaWYgdGhlIGxvY2FsIHggdmFsdWUgaXMgbmVnYXRpdmUsIHRoZSBvYnN0YWNsZSBpcyBiZWhpbmQgdGhlIGFnZW50XG4gICAgICBpZiAobG9jYWxQb3MueCA8IDApIHsgcmV0dXJuOyB9XG5cbiAgICAgIC8vIGlmIHRoZSBkaXN0YW5jZSBiZXR3ZWVuIHRoZSB4IGF4aXMgdG8gdGhlIG9qZWN0IGxvY2FsIHBvc2l0aW9uIGlzIGxlc3MgdGhhblxuICAgICAgLy8gaXRzIHJhZGl1cyArIHRoZSByYWRpdXMgb2YgdGhpcy52ZWhpY2xlLCB0aGVyZSBpcyBhIHBvc3NpYmxlIGNvbGxpc2lvblxuICAgICAgdmFyIGV4cGFuZGVkUmFkaXVzID0gb2JzdGFjbGUuYm91bmRpbmdSYWRpdXMgKyB0aGlzLnZlaGljbGUuYm91bmRpbmdSYWRpdXM7XG5cbiAgICAgIGlmIChsb2NhbFBvcy55ID4gZXhwYW5kZWRSYWRpdXMpIHsgcmV0dXJuOyB9XG5cbiAgICAgIC8vIG5vdyB0byBkbyBhIGxpbmUvY2lyY2xlIGludGVyc2VjdGlvbiB0ZXN0LiBUaGUgY2VudGVyIG9mIHRoZVxuICAgICAgLy8gY2lyY2xlIGlzIHJlcHJlc2VudGVkIGJ5IChjWCwgY1kpLiBUaGUgaW50ZXJzZWN0aW9uIHBvaW50cyBhcmVcbiAgICAgIC8vIGdpdmVuIGJ5IHRoZSBmb3JtdWxhIHggPSBjWCArLy1zcXJ0KHJeMi1jWV4yKSBmb3IgeT0wLlxuICAgICAgLy8gV2Ugb25seSBuZWVkIHRvIGxvb2sgYXQgdGhlIHNtYWxsZXN0IHBvc2l0aXZlIHZhbHVlIG9mIHggYmVjYXVzZVxuICAgICAgLy8gdGhhdCB3aWxsIGJlIHRoZSBjbG9zZXN0IHBvaW50IG9mIGludGVyc2VjdGlvbi5cbiAgICAgIHZhciBjWCA9IGxvY2FsUG9zLng7XG4gICAgICB2YXIgY1kgPSBsb2NhbFBvcy55O1xuXG4gICAgICAvLyBjYWxjdWxlIHRoZSBzcXJ0IHBhcnQgb2YgdGhlIGVxdWF0aW9uIG9ubHkgb25jZVxuICAgICAgdmFyIHNxcnRQYXJ0ID0gTWF0aC5zcXJ0KGV4cGVuZGVkUmFkaXVzICogZXhwZW5kZWRSYWRpdXMgLSBjWSAqIGNZKTtcblxuICAgICAgdmFyIGlwID0gY1ggLSBzcXJ0UGFydDtcbiAgICAgIGlmIChpcCA8IDApIHsgaXAgPSBjWCArIHNxcnRQYXJ0OyB9XG5cbiAgICAgIC8vIGlmIGNsb3Nlc3Qgc28gZmFyIC0gc3RvcmUgYWxsIGl0cyB2YWx1ZVxuICAgICAgaWYgKGlwIDwgZGlzdGFuY2VUb0NJTykge1xuICAgICAgICBkaXN0YW5jZVRvQ0lPID0gaXA7XG4gICAgICAgIGNsb3Nlc3RJbnRlcmNlcHRpbmdPYnN0YWNsZSA9IG9ic3RhY2xlO1xuICAgICAgICBsb2NhbFBvc2l0aW9uT2ZDSU8gPSBsb2NhbFBvcztcbiAgICAgIH1cbiAgICB9LCB0aGlzKTtcblxuICAgIC8vIHN0aWxsIGluIGxvY2FsIHNwYWNlXG4gICAgdmFyIHN0ZWVyaW5nID0gbmV3IFZlY3RvcigpO1xuICAgIC8vIGlmIHdlIGZvdW5kIHNvbWUgb2J0YWNsZSwgY2FsY3VsYXRlIGEgc3RlZXJpbmcgZm9yY2UgYXdheSBmcm9tIGl0XG4gICAgaWYgKGNsb3Nlc3RJbnRlcmNlcHRpbmdPYnN0YWNsZSkge1xuICAgICAgLy8gdGhlIGNsb3NlciBhbiBhZ2VudCBpcyB0byBhbiBvYmplY3QsIHRoZSBzdHJvbmdlciB0aGUgc3RlZXJpbmcgKGJldHdlZW4gMSBhbmQgMiBbP10pXG4gICAgICB2YXIgbXVsdGlwbGllciA9IDEgKyAoYm94TGVuZ3RoIC0gbG9jYWxQb3NpdGlvbk9mQ0lPLngpIC8gYm94TGVuZ3RoO1xuXG4gICAgICBzdGVlcmluZy55ID0gKGNsb3Nlc3RJbnRlcmNlcHRpbmdPYnN0YWNsZS5ib3VuZGluZ1JhZGl1cyAtXG4gICAgICAgICAgICAgICAgICAgIGxvY2FsUG9zaXRpb25PZkNJTy55KSAqIG11bHRpcGxpZXI7XG5cbiAgICAgIHZhciBicmFraW5nV2VpZ2h0ID0gMC4yO1xuXG4gICAgICBzdGVlcmluZy54ID0gKGNsb3Nlc3RJbnRlcmNlcHRpbmdPYnN0YWNsZS5ib3VuZGluZ1JhZGl1cyAtXG4gICAgICAgICAgICAgICAgICAgIGxvY2FsUG9zaXRpb25PZkNJTy54KSAqIGJyYWtpbmdXZWlnaHQ7XG4gICAgfVxuXG4gICAgLy8gcm90YXRlIHRvIGdvIGJhY2sgdG8gd29ybGQgc3BhY2VcbiAgICBzdGVlcmluZy5yb3RhdGUodGhpcy52ZWhpY2xlLmhlYWRpbmcuZGlyZWN0aW9uKCkpO1xuICAgIHJldHVybiBzdGVlcmluZztcbiAgfVxuXG4gIHdhbGxBdm9pZGFuY2Uod2FsbHMpIHtcbiAgICB0aGlzLmNyZWF0ZUZlZWxlcnMoKTtcblxuICAgIC8vIElQOiBJbnRlcnNlY3Rpb24gUG9pbnRcbiAgICB2YXIgZGlzdGFuY2VUb0Nsb3Nlc3RJUCA9ICtJbmZpbml0eTtcbiAgICB2YXIgY2xvc2VzdFBvaW50ID0gbnVsbDtcbiAgICB2YXIgY2xvc2VzdFdhbGwgPSBudWxsO1xuXG4gICAgdmFyIHN0ZWVyaW5nID0gbmV3IFZlY3RvcigpO1xuICAgIHZhciBkaXN0YW5jZSwgcG9pbnQ7XG5cbiAgICAvLyBnZW9tZXRyeS5oIChsaW5lIDI4NCkgTGluZUludGVyc2VjdGlvbjJkICg/Pz8gd2hhdCBhcHBlbmRzIGhlcmUgPz8/KVxuICAgIHZhciBpbnRlcnNlY3QgPSBmdW5jdGlvbihmMSwgdDEsIGYyLCB0Mikge1xuICAgICAgdmFyIHJUb3AgPSAoZjEueS1mMi55KSoodDIueC1mMi54KS0oZjEueC1mMi54KSoodDIueS1mMi55KTtcbiAgICAgIHZhciByQm90ID0gKHQxLngtZjEueCkqKHQyLnktZjIueSktKHQxLnktZjEueSkqKHQyLngtZjIueCk7XG5cbiAgICAgIHZhciBzVG9wID0gKGYxLnktZjIueSkqKHQxLngtZjEueCktKGYxLngtZjIueCkqKHQxLnktZjEueSk7XG4gICAgICB2YXIgc0JvdCA9ICh0MS54LWYxLngpKih0Mi55LWYyLnkpLSh0MS55LWYxLnkpKih0Mi54LWYyLngpO1xuXG4gICAgICBpZiAoIChyQm90ID09PSAwKSB8fMKgKHNCb3QgPT09IDApICkge1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICB9XG5cbiAgICAgIHZhciByID0gclRvcC9yQm90O1xuICAgICAgdmFyIHMgPSBzVG9wL3NCb3Q7XG5cbiAgICAgIGlmICggKHIgPiAwKSAmJiAociA8IDEpICYmIChzID4gMCkgJiYgKHMgPCAxKSApIHtcbiAgICAgICAgZGlzdGFuY2UgPSBWZWN0b3IuZGlzdGFuY2UoZjEsIHQxKSAqIHI7XG4gICAgICAgIC8vIGNvbnNvbGUubG9nKHIsIGRpc3RhbmNlKTtcbiAgICAgICAgcG9pbnQgPSBuZXcgVmVjdG9yKGYxLnggKyByLCBmMS55ICsgcik7XG4gICAgICAgIC8vIGNoYW5nZSBmcm9tIGMrKyBjb2RlIGhlcmUsIHRvbyBtdWNoIGZvcmNlIHNvbWV0aGluZyBpcyB3cm9uZyAoLi4ucHJvYmFibHkgbWUpXG4gICAgICAgIC8vIHBvaW50Lm11bHRpcGx5KFZlY3Rvci5zdWJzdHJhY3QodDEsIGYxKSk7XG4gICAgICAgIHBvaW50Lm11bHRpcGx5KDEuMDgpO1xuXG4gICAgICAgIHJldHVybiB0cnVlO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgZGlzdGFuY2UgPSAwO1xuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICAvLyBleGFtaW5lIGVhY2ggZmVlbGVyIG9ubiBlYWNoIHdhbGxcbiAgICB0aGlzLmZlZWxlcnMuZm9yRWFjaChmdW5jdGlvbihmZWVsZXIpIHtcbiAgICAgIHdhbGxzLmZvckVhY2goZnVuY3Rpb24od2FsbCkge1xuICAgICAgICBpZiAoaW50ZXJzZWN0KHRoaXMudmVoaWNsZS5wb3NpdGlvbiwgZmVlbGVyLCB3YWxsLmZyb20sIHdhbGwudG8pKSB7XG4gICAgICAgICAgaWYgKGRpc3RhbmNlIDwgZGlzdGFuY2VUb0Nsb3Nlc3RJUCkge1xuICAgICAgICAgICAgZGlzdGFuY2VUb0Nsb3Nlc3RJUCA9IGRpc3RhbmNlO1xuICAgICAgICAgICAgY2xvc2VzdFdhbGwgPSB3YWxsO1xuICAgICAgICAgICAgY2xvc2VzdFBvaW50ID0gcG9pbnQ7XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9LCB0aGlzKTtcblxuXG4gICAgICBpZiAoY2xvc2VzdFdhbGwpIHtcbiAgICAgICAgdmFyIG92ZXJTaG9vdCA9IFZlY3Rvci5zdWJzdHJhY3QoZmVlbGVyLCBjbG9zZXN0UG9pbnQpO1xuICAgICAgICAvLyBjb25zb2xlLmxvZyhvdmVyU2hvb3QubWFnbml0dWRlKCkpXG4gICAgICAgIHN0ZWVyaW5nID0gVmVjdG9yLm11bHRpcGx5KGNsb3Nlc3RXYWxsLm5vcm1hbCwgb3ZlclNob290Lm1hZ25pdHVkZSgpKTtcbiAgICAgIH1cbiAgICB9LCB0aGlzKTtcblxuICAgIHJldHVybiBzdGVlcmluZztcbiAgfVxuXG4gIC8vIGNyZWF0ZSBhbnRlbm5hIHVzZWQgaW4gd2FsbEF2b2lkYW5jZSAtIGluIHdvcmxkIGNvb3JkaW5hdGVzXG4gIGNyZWF0ZUZlZWxlcnMoKSB7XG4gICAgLy8gY2VudGVyXG4gICAgdmFyIHRtcCA9IFZlY3Rvci5tdWx0aXBseSh0aGlzLnZlaGljbGUuaGVhZGluZywgdGhpcy5wYXJhbXMud2FsbERldGVjdGlvbkZlZWxlckxlbmd0aCk7XG4gICAgdGhpcy5mZWVsZXJzWzBdID0gVmVjdG9yLmFkZCh0aGlzLnZlaGljbGUucG9zaXRpb24sIHRtcCk7XG4gICAgLy8gbGVmdFxuICAgIHZhciB0bXAgPSB0aGlzLnZlaGljbGUuaGVhZGluZy5jbG9uZSgpO1xuICAgIHRtcC5yb3RhdGUoTWF0aC5QSSAvIDIgKiAzLjUpO1xuICAgIHRtcCA9IFZlY3Rvci5tdWx0aXBseSh0bXAsIHRoaXMucGFyYW1zLndhbGxEZXRlY3Rpb25GZWVsZXJMZW5ndGggLyAyKTtcbiAgICB0aGlzLmZlZWxlcnNbMV0gPSBWZWN0b3IuYWRkKHRoaXMudmVoaWNsZS5wb3NpdGlvbiwgdG1wKTtcbiAgICAvLyByaWdodFxuICAgIHZhciB0bXAgPSB0aGlzLnZlaGljbGUuaGVhZGluZy5jbG9uZSgpO1xuICAgIHRtcC5yb3RhdGUoTWF0aC5QSSAvIDIgKiAwLjUpO1xuICAgIHRtcCA9IFZlY3Rvci5tdWx0aXBseSh0bXAsIHRoaXMucGFyYW1zLndhbGxEZXRlY3Rpb25GZWVsZXJMZW5ndGggLyAyKTtcbiAgICB0aGlzLmZlZWxlcnNbMl0gPSBWZWN0b3IuYWRkKHRoaXMudmVoaWNsZS5wb3NpdGlvbiwgdG1wKTtcbiAgfVxuXG5cbiAgLy8gdmlzdWFsaXplIC0gZGVidWcgdG9vbHNcbiAgLy8gYWxsIGRlYnVnIHN0dWZmIHNob3VsZCBiZSBpbiBzdGVycmluZ1xuICBkZWJ1Z0ZlZWxlcnMoY3R4KSB7XG4gICAgdGhpcy5mZWVsZXJzLmZvckVhY2goZnVuY3Rpb24oZmVlbGVyKSB7XG4gICAgICBjdHguc2F2ZSgpO1xuICAgICAgY3R4LmJlZ2luUGF0aCgpO1xuXG4gICAgICBjdHguc3Ryb2tlU3R5bGUgPSAnI2FjYWNhYyc7XG4gICAgICBjdHgubW92ZVRvKHRoaXMudmVoaWNsZS5wb3NpdGlvbi54LCB0aGlzLnZlaGljbGUucG9zaXRpb24ueSk7XG4gICAgICBjdHgubGluZVRvKGZlZWxlci54LCBmZWVsZXIueSk7XG4gICAgICBjdHguc3Ryb2tlKCk7XG5cbiAgICAgIGN0eC5jbG9zZVBhdGgoKTtcbiAgICAgIGN0eC5yZXN0b3JlKCk7XG4gICAgfSwgdGhpcyk7XG4gIH1cblxuICBkZWJ1Z1dhbmRlcihjdHgpIHtcbiAgICBjdHguc2F2ZSgpO1xuICAgIGN0eC50cmFuc2xhdGUodGhpcy52ZWhpY2xlLnBvc2l0aW9uLngsIHRoaXMudmVoaWNsZS5wb3NpdGlvbi55KTtcbiAgICBjdHgucm90YXRlKHRoaXMudmVoaWNsZS5oZWFkaW5nLmRpcmVjdGlvbigpKTtcblxuICAgIC8vIG1vdmUgdG8gd2FuZGVyIGNpcmNsZSBjZW50ZXJcbiAgICBjdHgudHJhbnNsYXRlKHRoaXMud2FuZGVyRGlzdGFuY2UsIDApO1xuXG4gICAgY3R4LnN0cm9rZVN0eWxlID0gJ3JlZCc7XG4gICAgY3R4LmZpbGxTdHlsZSA9ICdyZWQnO1xuICAgIC8vIHdhbmRlciBjaXJjbGVcbiAgICBjdHguYmVnaW5QYXRoKCk7XG4gICAgY3R4LmFyYygwLCAwLCB0aGlzLndhbmRlclJhZGl1cywgMCwgdXRpbHMuVHdvUEksIGZhbHNlKTtcbiAgICBjdHguc3Ryb2tlKCk7XG4gICAgY3R4LmNsb3NlUGF0aCgpO1xuICAgIC8vIHdhbmRlciB0YXJnZXRcbiAgICBjdHguYmVnaW5QYXRoKCk7XG4gICAgY3R4LmFyYyh0aGlzLndhbmRlclRhcmdldC54LCB0aGlzLndhbmRlclRhcmdldC55LCAyLCAwLCB1dGlscy5Ud29QSSwgZmFsc2UpO1xuICAgIGN0eC5zdHJva2UoKTtcbiAgICBjdHguY2xvc2VQYXRoKCk7XG5cbiAgICBjdHgucmVzdG9yZSgpO1xuICB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gU3RlZXJpbmdCZWhhdmlvcnM7XG4iXX0=