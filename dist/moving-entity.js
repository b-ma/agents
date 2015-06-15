'use strict';

var _inherits = require('babel-runtime/helpers/inherits')['default'];

var _get = require('babel-runtime/helpers/get')['default'];

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var Vector = require('vector');
var BaseGameEntity = require('./base-game-entity');
var SteeringBehaviors = require('./steering-behaviors');

var entityType = 0;

var MovingEntity = (function (_BaseGameEntity) {
  function MovingEntity(world, position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    _classCallCheck(this, MovingEntity);

    _get(Object.getPrototypeOf(MovingEntity.prototype), 'constructor', this).call(this, entityType, position, boundingRadius);

    // call some kind of super
    this.world = world;
    this.steerings = new SteeringBehaviors(this);

    this.velocity = velocity;
    // normalized vector pointing in the direction the entity is moving
    this.heading = heading;
    // vector perpendicular to the heading vector
    this.side = null;
    this.mass = mass;
    // maximum speed (float)
    this.maxSpeed = maxSpeed;

    // @NOTE to implement
    // maximum force the entity can produce to power itself (think motor engine)
    this.maxForce = maxForce;
    // maximum rate at which the entity can rotate (rad/sec)
    this.maxTurnRate = maxTurnRate;

    // used in steeringBehaviors
    this.neighbors = [];
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
  }, {
    key: 'update',
    value: function update(dt) {
      // define all the forces applied on the agent
      var steeringForces = this.steerings.calculate();
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

  return MovingEntity;
})(BaseGameEntity);

module.exports = MovingEntity;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNyYy9tb3ZpbmctZW50aXR5LmVzNi5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiOzs7Ozs7Ozs7O0FBQUEsSUFBSSxNQUFNLEdBQUcsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDO0FBQy9CLElBQUksY0FBYyxHQUFHLE9BQU8sQ0FBQyxvQkFBb0IsQ0FBQyxDQUFDO0FBQ25ELElBQUksaUJBQWlCLEdBQUcsT0FBTyxDQUFDLHNCQUFzQixDQUFDLENBQUM7O0FBRXhELElBQUksVUFBVSxHQUFHLENBQUMsQ0FBQzs7SUFFYixZQUFZO0FBRUwsV0FGUCxZQUFZLENBRUosS0FBSyxFQUFFLFFBQVEsRUFBRSxjQUFjLEVBQUUsUUFBUSxFQUFFLFFBQVEsRUFBRSxPQUFPLEVBQUUsSUFBSSxFQUFFLFdBQVcsRUFBRSxRQUFRLEVBQUU7MEJBRm5HLFlBQVk7O0FBR2QsK0JBSEUsWUFBWSw2Q0FHUixVQUFVLEVBQUUsUUFBUSxFQUFFLGNBQWMsRUFBRTs7O0FBRzVDLFFBQUksQ0FBQyxLQUFLLEdBQUcsS0FBSyxDQUFDO0FBQ25CLFFBQUksQ0FBQyxTQUFTLEdBQUcsSUFBSSxpQkFBaUIsQ0FBQyxJQUFJLENBQUMsQ0FBQzs7QUFFN0MsUUFBSSxDQUFDLFFBQVEsR0FBRyxRQUFRLENBQUM7O0FBRXpCLFFBQUksQ0FBQyxPQUFPLEdBQUcsT0FBTyxDQUFDOztBQUV2QixRQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQztBQUNqQixRQUFJLENBQUMsSUFBSSxHQUFHLElBQUksQ0FBQzs7QUFFakIsUUFBSSxDQUFDLFFBQVEsR0FBRyxRQUFRLENBQUM7Ozs7QUFJekIsUUFBSSxDQUFDLFFBQVEsR0FBRyxRQUFRLENBQUM7O0FBRXpCLFFBQUksQ0FBQyxXQUFXLEdBQUcsV0FBVyxDQUFDOzs7QUFHL0IsUUFBSSxDQUFDLFNBQVMsR0FBRyxFQUFFLENBQUM7R0FDckI7O1lBMUJHLFlBQVk7O2VBQVosWUFBWTs7V0E0QlgsaUJBQUc7QUFBRSxhQUFPLElBQUksQ0FBQyxRQUFRLENBQUMsU0FBUyxFQUFFLENBQUM7S0FBRTs7O1dBQ3BDLHFCQUFHO0FBQUUsYUFBTyxJQUFJLENBQUMsUUFBUSxDQUFDLGFBQWEsRUFBRSxDQUFDO0tBQUU7OztXQUUvQyxnQkFBQyxFQUFFLEVBQUU7O0FBRVQsVUFBSSxjQUFjLEdBQUcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxTQUFTLEVBQUUsQ0FBQzs7QUFFaEQsVUFBSSxDQUFDLFlBQVksR0FBRyxNQUFNLENBQUMsTUFBTSxDQUFDLGNBQWMsRUFBRSxJQUFJLENBQUMsSUFBSSxDQUFDLENBQUM7O0FBRTdELFVBQUksQ0FBQyxRQUFRLENBQUMsR0FBRyxDQUFDLE1BQU0sQ0FBQyxRQUFRLENBQUMsSUFBSSxDQUFDLFlBQVksRUFBRSxFQUFFLENBQUMsQ0FBQyxDQUFDOztBQUUxRCxVQUFJLENBQUMsUUFBUSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUM7O0FBRXRDLFVBQUksQ0FBQyxRQUFRLENBQUMsR0FBRyxDQUFDLE1BQU0sQ0FBQyxRQUFRLENBQUMsSUFBSSxDQUFDLFFBQVEsRUFBRSxFQUFFLENBQUMsQ0FBQyxDQUFDOzs7O0FBSXRELFVBQUksSUFBSSxDQUFDLFFBQVEsQ0FBQyxhQUFhLEVBQUUsR0FBRyxPQUFPLEVBQUU7QUFDM0MsWUFBSSxDQUFDLE9BQU8sR0FBRyxNQUFNLENBQUMsU0FBUyxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQztBQUMvQyxZQUFJLENBQUMsSUFBSSxHQUFHLE1BQU0sQ0FBQyxVQUFVLENBQUMsSUFBSSxDQUFDLE9BQU8sQ0FBQyxDQUFDO09BQzdDO0tBQ0Y7OztXQUVLLGdCQUFDLEdBQUcsRUFBRSxPQUFPLG1CQUFrQixFQUFFOzs7U0FuRG5DLFlBQVk7R0FBUyxjQUFjOztBQXNEekMsTUFBTSxDQUFDLE9BQU8sR0FBRyxZQUFZLENBQUMiLCJmaWxlIjoic3JjL21vdmluZy1lbnRpdHkuZXM2LmpzIiwic291cmNlc0NvbnRlbnQiOlsidmFyIFZlY3RvciA9IHJlcXVpcmUoJ3ZlY3RvcicpO1xudmFyIEJhc2VHYW1lRW50aXR5ID0gcmVxdWlyZSgnLi9iYXNlLWdhbWUtZW50aXR5Jyk7XG52YXIgU3RlZXJpbmdCZWhhdmlvcnMgPSByZXF1aXJlKCcuL3N0ZWVyaW5nLWJlaGF2aW9ycycpO1xuXG52YXIgZW50aXR5VHlwZSA9IDA7XG5cbmNsYXNzIE1vdmluZ0VudGl0eSBleHRlbmRzIEJhc2VHYW1lRW50aXR5IHtcblxuICBjb25zdHJ1Y3Rvcih3b3JsZCwgcG9zaXRpb24sIGJvdW5kaW5nUmFkaXVzLCB2ZWxvY2l0eSwgbWF4U3BlZWQsIGhlYWRpbmcsIG1hc3MsIG1heFR1cm5SYXRlLCBtYXhGb3JjZSkge1xuICAgIHN1cGVyKGVudGl0eVR5cGUsIHBvc2l0aW9uLCBib3VuZGluZ1JhZGl1cyk7XG5cbiAgICAvLyBjYWxsIHNvbWUga2luZCBvZiBzdXBlclxuICAgIHRoaXMud29ybGQgPSB3b3JsZDtcbiAgICB0aGlzLnN0ZWVyaW5ncyA9IG5ldyBTdGVlcmluZ0JlaGF2aW9ycyh0aGlzKTtcblxuICAgIHRoaXMudmVsb2NpdHkgPSB2ZWxvY2l0eTtcbiAgICAvLyBub3JtYWxpemVkIHZlY3RvciBwb2ludGluZyBpbiB0aGUgZGlyZWN0aW9uIHRoZSBlbnRpdHkgaXMgbW92aW5nXG4gICAgdGhpcy5oZWFkaW5nID0gaGVhZGluZztcbiAgICAvLyB2ZWN0b3IgcGVycGVuZGljdWxhciB0byB0aGUgaGVhZGluZyB2ZWN0b3JcbiAgICB0aGlzLnNpZGUgPSBudWxsO1xuICAgIHRoaXMubWFzcyA9IG1hc3M7XG4gICAgLy8gbWF4aW11bSBzcGVlZCAoZmxvYXQpXG4gICAgdGhpcy5tYXhTcGVlZCA9IG1heFNwZWVkO1xuXG4gICAgLy8gQE5PVEUgdG8gaW1wbGVtZW50XG4gICAgLy8gbWF4aW11bSBmb3JjZSB0aGUgZW50aXR5IGNhbiBwcm9kdWNlIHRvIHBvd2VyIGl0c2VsZiAodGhpbmsgbW90b3IgZW5naW5lKVxuICAgIHRoaXMubWF4Rm9yY2UgPSBtYXhGb3JjZTtcbiAgICAvLyBtYXhpbXVtIHJhdGUgYXQgd2hpY2ggdGhlIGVudGl0eSBjYW4gcm90YXRlIChyYWQvc2VjKVxuICAgIHRoaXMubWF4VHVyblJhdGUgPSBtYXhUdXJuUmF0ZTtcblxuICAgIC8vIHVzZWQgaW4gc3RlZXJpbmdCZWhhdmlvcnNcbiAgICB0aGlzLm5laWdoYm9ycyA9IFtdO1xuICB9XG5cbiAgc3BlZWQoKSB7IHJldHVybiB0aGlzLnZlbG9jaXR5Lm1hZ25pdHVkZSgpOyB9XG4gIHNwZWVkU3FydCgpIHsgcmV0dXJuIHRoaXMudmVsb2NpdHkubWFnbml0dWRlU3FydCgpOyB9XG5cbiAgdXBkYXRlKGR0KSB7XG4gICAgLy8gZGVmaW5lIGFsbCB0aGUgZm9yY2VzIGFwcGxpZWQgb24gdGhlIGFnZW50XG4gICAgdmFyIHN0ZWVyaW5nRm9yY2VzID0gdGhpcy5zdGVlcmluZ3MuY2FsY3VsYXRlKCk7XG4gICAgLy8gbmV3dG9uXG4gICAgdGhpcy5hY2NlbGVyYXRpb24gPSBWZWN0b3IuZGl2aWRlKHN0ZWVyaW5nRm9yY2VzLCB0aGlzLm1hc3MpO1xuICAgIC8vIHJldmVyc2UgZm91cmllciBpbnRlZ3JhdGlvbiAtIGNvdWxkIGJlIGEgc2VydmljZVxuICAgIHRoaXMudmVsb2NpdHkuYWRkKFZlY3Rvci5tdWx0aXBseSh0aGlzLmFjY2VsZXJhdGlvbiwgZHQpKTtcbiAgICAvLyB2ZWhpY2xlIGNhbm5vdCBnbyBiZXlvbmcgbWF4IHNwZWVkXG4gICAgdGhpcy52ZWxvY2l0eS50cnVuY2F0ZSh0aGlzLm1heFNwZWVkKTtcbiAgICAvLyB1cGRhdGUgbG9jYXRpb25cbiAgICB0aGlzLnBvc2l0aW9uLmFkZChWZWN0b3IubXVsdGlwbHkodGhpcy52ZWxvY2l0eSwgZHQpKTtcblxuICAgIC8vIHByZXZlbnQgc3R1cGlkIGRpc3BsYXkgYmVoYXZpb3IgKHR1cm5pbmcgd2hlbiBzdG9wcGVkKVxuICAgIC8vIGNvbnNvbGUubG9nKHRoaXMudmVsb2NpdHkubWFnbml0dWRlU3FydCgpKTtcbiAgICBpZiAodGhpcy52ZWxvY2l0eS5tYWduaXR1ZGVTcXJ0KCkgPiAwLjAwMDAxKSB7XG4gICAgICB0aGlzLmhlYWRpbmcgPSBWZWN0b3Iubm9ybWFsaXplKHRoaXMudmVsb2NpdHkpO1xuICAgICAgdGhpcy5zaWRlID0gVmVjdG9yLm9ydGhvZ29uYWwodGhpcy5oZWFkaW5nKTtcbiAgICB9XG4gIH1cblxuICByZW5kZXIoY3R4LCBidWZmZXJzLyosIHdvcmxkU2l6ZSAqLykge31cbn1cblxubW9kdWxlLmV4cG9ydHMgPSBNb3ZpbmdFbnRpdHk7XG4iXX0=