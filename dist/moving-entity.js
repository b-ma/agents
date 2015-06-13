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
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNyYy92ZWhpY2xlLmVzNi5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiOzs7Ozs7Ozs7O0FBQUEsSUFBSSxNQUFNLEdBQUcsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDO0FBQy9CLElBQUksY0FBYyxHQUFHLE9BQU8sQ0FBQyxvQkFBb0IsQ0FBQyxDQUFDOztBQUVuRCxJQUFJLFVBQVUsR0FBRyxDQUFDLENBQUM7O0lBRWIsWUFBWTtBQUVMLFdBRlAsWUFBWSxDQUVKLFFBQVEsRUFBRSxjQUFjLEVBQUUsUUFBUSxFQUFFLFFBQVEsRUFBRSxPQUFPLEVBQUUsSUFBSSxFQUFFLFdBQVcsRUFBRSxRQUFRLEVBQUU7MEJBRjVGLFlBQVk7O0FBR2QsK0JBSEUsWUFBWSw2Q0FHUixVQUFVLEVBQUUsUUFBUSxFQUFFLGNBQWMsRUFBRTs7QUFFNUMsUUFBSSxDQUFDLFFBQVEsR0FBRyxRQUFRLENBQUM7O0FBRXpCLFFBQUksQ0FBQyxPQUFPLEdBQUcsT0FBTyxDQUFDOztBQUV2QixRQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQztBQUNqQixRQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQzs7QUFFakIsUUFBSSxDQUFDLFFBQVEsR0FBRyxRQUFRLENBQUM7O0FBRXpCLFFBQUksQ0FBQyxRQUFRLEdBQUcsUUFBUSxDQUFDOztBQUV6QixRQUFJLENBQUMsV0FBVyxHQUFHLFdBQVcsQ0FBQzs7R0FFaEM7O1lBbEJHLFlBQVk7O2VBQVosWUFBWTs7V0FvQlgsaUJBQUc7QUFBRSxhQUFPLElBQUksQ0FBQyxRQUFRLENBQUMsU0FBUyxFQUFFLENBQUM7S0FBRTs7O1dBQ3BDLHFCQUFHO0FBQUUsYUFBTyxJQUFJLENBQUMsUUFBUSxDQUFDLGFBQWEsRUFBRSxDQUFDO0tBQUU7OztTQXJCakQsWUFBWTtHQUFTLGNBQWM7O0FBd0J6QyxNQUFNLENBQUMsT0FBTyxHQUFHLFlBQVksQ0FBQSIsImZpbGUiOiJzcmMvdmVoaWNsZS5lczYuanMiLCJzb3VyY2VzQ29udGVudCI6WyJ2YXIgdmVjdG9yID0gcmVxdWlyZSgndmVjdG9yJyk7XG52YXIgQmFzZUdhbWVFbnRpdHkgPSByZXF1aXJlKCcuL2Jhc2UtZ2FtZS1lbnRpdHknKTtcblxudmFyIGVudGl0eVR5cGUgPSAwO1xuXG5jbGFzcyBNb3ZpbmdFbnRpdHkgZXh0ZW5kcyBCYXNlR2FtZUVudGl0eSB7XG5cbiAgY29uc3RydWN0b3IocG9zaXRpb24sIGJvdW5kaW5nUmFkaXVzLCB2ZWxvY2l0eSwgbWF4U3BlZWQsIGhlYWRpbmcsIG1hc3MsIG1heFR1cm5SYXRlLCBtYXhGb3JjZSkge1xuICAgIHN1cGVyKGVudGl0eVR5cGUsIHBvc2l0aW9uLCBib3VuZGluZ1JhZGl1cyk7XG5cbiAgICB0aGlzLnZlbG9jaXR5ID0gdmVsb2NpdHk7XG4gICAgLy8gbm9ybWFsaXplZCB2ZWN0b3IgcG9pbnRpbmcgaW4gdGhlIGRpcmVjdGlvbiB0aGUgZW50aXR5IGlzIG1vdmluZ1xuICAgIHRoaXMuaGVhZGluZyA9IGhlYWRpbmc7XG4gICAgLy8gdmVjdG9yIHBlcnBlbmRpY3VsYXIgdG8gdGhlIGhlYWRpbmcgdmVjdG9yXG4gICAgdGhpcy5zaWRlID0gbnVsbDtcbiAgICB0aGlzLm1hc3MgPSBtYXNzO1xuICAgIC8vIG1heGltdW0gc3BlZWQgKGZsb2F0KVxuICAgIHRoaXMubWF4U3BlZWQgPSBtYXhTcGVlZDtcbiAgICAvLyBtYXhpbXVtIGZvcmNlIHRoZSBlbnRpdHkgY2FuIHByb2R1Y2UgdG8gcG93ZXIgaXRzZWxmICh0aGluayBtb3RvciBlbmdpbmUpXG4gICAgdGhpcy5tYXhGb3JjZSA9IG1heEZvcmNlO1xuICAgIC8vIG1heGltdW0gcmF0ZSBhdCB3aGljaCB0aGUgZW50aXR5IGNhbiByb3RhdGUgKHJhZC9zZWMpXG4gICAgdGhpcy5tYXhUdXJuUmF0ZSA9IG1heFR1cm5SYXRlO1xuICAgIC8vIGlnbm9yZSBzY2FsZVxuICB9XG5cbiAgc3BlZWQoKSB7IHJldHVybiB0aGlzLnZlbG9jaXR5Lm1hZ25pdHVkZSgpOyB9XG4gIHNwZWVkU3FydCgpIHsgcmV0dXJuIHRoaXMudmVsb2NpdHkubWFnbml0dWRlU3FydCgpOyB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gTW92aW5nRW50aXR5XG4iXX0=