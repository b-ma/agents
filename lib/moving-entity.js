"use strict";

var _babelHelpers = require("babel-runtime/helpers")["default"];

var _core = require("babel-runtime/core-js")["default"];

var vector = require("vector");
var BaseGameEntity = require("./base-game-entity");

var entityType = 0;

var MovingEntity = (function (BaseGameEntity) {
  function MovingEntity(position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    _babelHelpers.classCallCheck(this, MovingEntity);

    _babelHelpers.get(_core.Object.getPrototypeOf(MovingEntity.prototype), "constructor", this).call(this, entityType, position, boundingRadius);

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

  _babelHelpers.inherits(MovingEntity, BaseGameEntity);

  _babelHelpers.prototypeProperties(MovingEntity, null, {
    speed: {
      value: function speed() {
        return this.velocity.magnitude();
      },
      writable: true,
      configurable: true
    },
    speedSqrt: {
      value: function speedSqrt() {
        return this.velocity.magnitudeSqrt();
      },
      writable: true,
      configurable: true
    }
  });

  return MovingEntity;
})(BaseGameEntity);

module.exports = MovingEntity;

//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4vbGliL21vdmluZy1lbnRpdHkuZXM2LmpzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7Ozs7OztBQUFBLElBQUksTUFBTSxHQUFHLE9BQU8sQ0FBQyxRQUFRLENBQUMsQ0FBQztBQUMvQixJQUFJLGNBQWMsR0FBRyxPQUFPLENBQUMsb0JBQW9CLENBQUMsQ0FBQzs7QUFFbkQsSUFBSSxVQUFVLEdBQUcsQ0FBQyxDQUFDOztJQUViLFlBQVksY0FBUyxjQUFjO0FBRTVCLFdBRlAsWUFBWSxDQUVKLFFBQVEsRUFBRSxjQUFjLEVBQUUsUUFBUSxFQUFFLFFBQVEsRUFBRSxPQUFPLEVBQUUsSUFBSSxFQUFFLFdBQVcsRUFBRSxRQUFRO3VDQUYxRixZQUFZOztBQUdkLGtEQUhFLFlBQVksNkNBR1IsVUFBVSxFQUFFLFFBQVEsRUFBRSxjQUFjLEVBQUU7O0FBRTVDLFFBQUksQ0FBQyxRQUFRLEdBQUcsUUFBUSxDQUFDOztBQUV6QixRQUFJLENBQUMsT0FBTyxHQUFHLE9BQU8sQ0FBQzs7QUFFdkIsUUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUM7QUFDakIsUUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUM7O0FBRWpCLFFBQUksQ0FBQyxRQUFRLEdBQUcsUUFBUSxDQUFDOztBQUV6QixRQUFJLENBQUMsUUFBUSxHQUFHLFFBQVEsQ0FBQzs7QUFFekIsUUFBSSxDQUFDLFdBQVcsR0FBRyxXQUFXLENBQUM7O0dBRWhDOzt5QkFsQkcsWUFBWSxFQUFTLGNBQWM7O29DQUFuQyxZQUFZO0FBb0JoQixTQUFLO2FBQUEsaUJBQUc7QUFBRSxlQUFPLElBQUksQ0FBQyxRQUFRLENBQUMsU0FBUyxFQUFFLENBQUM7T0FBRTs7OztBQUM3QyxhQUFTO2FBQUEscUJBQUc7QUFBRSxlQUFPLElBQUksQ0FBQyxRQUFRLENBQUMsYUFBYSxFQUFFLENBQUM7T0FBRTs7Ozs7O1NBckJqRCxZQUFZO0dBQVMsY0FBYzs7QUF3QnpDLE1BQU0sQ0FBQyxPQUFPLEdBQUcsWUFBWSxDQUFBIiwiZmlsZSI6Ii4vbGliL21vdmluZy1lbnRpdHkuanMiLCJzb3VyY2VzQ29udGVudCI6WyJ2YXIgdmVjdG9yID0gcmVxdWlyZSgndmVjdG9yJyk7XG52YXIgQmFzZUdhbWVFbnRpdHkgPSByZXF1aXJlKCcuL2Jhc2UtZ2FtZS1lbnRpdHknKTtcblxudmFyIGVudGl0eVR5cGUgPSAwO1xuXG5jbGFzcyBNb3ZpbmdFbnRpdHkgZXh0ZW5kcyBCYXNlR2FtZUVudGl0eSB7XG5cbiAgY29uc3RydWN0b3IocG9zaXRpb24sIGJvdW5kaW5nUmFkaXVzLCB2ZWxvY2l0eSwgbWF4U3BlZWQsIGhlYWRpbmcsIG1hc3MsIG1heFR1cm5SYXRlLCBtYXhGb3JjZSkge1xuICAgIHN1cGVyKGVudGl0eVR5cGUsIHBvc2l0aW9uLCBib3VuZGluZ1JhZGl1cyk7XG5cbiAgICB0aGlzLnZlbG9jaXR5ID0gdmVsb2NpdHk7XG4gICAgLy8gbm9ybWFsaXplZCB2ZWN0b3IgcG9pbnRpbmcgaW4gdGhlIGRpcmVjdGlvbiB0aGUgZW50aXR5IGlzIG1vdmluZ1xuICAgIHRoaXMuaGVhZGluZyA9IGhlYWRpbmc7XG4gICAgLy8gdmVjdG9yIHBlcnBlbmRpY3VsYXIgdG8gdGhlIGhlYWRpbmcgdmVjdG9yXG4gICAgdGhpcy5zaWRlID0gbnVsbDtcbiAgICB0aGlzLm1hc3MgPSBtYXNzO1xuICAgIC8vIG1heGltdW0gc3BlZWQgKGZsb2F0KVxuICAgIHRoaXMubWF4U3BlZWQgPSBtYXhTcGVlZDtcbiAgICAvLyBtYXhpbXVtIGZvcmNlIHRoZSBlbnRpdHkgY2FuIHByb2R1Y2UgdG8gcG93ZXIgaXRzZWxmICh0aGluayBtb3RvciBlbmdpbmUpXG4gICAgdGhpcy5tYXhGb3JjZSA9IG1heEZvcmNlO1xuICAgIC8vIG1heGltdW0gcmF0ZSBhdCB3aGljaCB0aGUgZW50aXR5IGNhbiByb3RhdGUgKHJhZC9zZWMpXG4gICAgdGhpcy5tYXhUdXJuUmF0ZSA9IG1heFR1cm5SYXRlO1xuICAgIC8vIGlnbm9yZSBzY2FsZVxuICB9XG5cbiAgc3BlZWQoKSB7IHJldHVybiB0aGlzLnZlbG9jaXR5Lm1hZ25pdHVkZSgpOyB9XG4gIHNwZWVkU3FydCgpIHsgcmV0dXJuIHRoaXMudmVsb2NpdHkubWFnbml0dWRlU3FydCgpOyB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gTW92aW5nRW50aXR5XG4iXX0=