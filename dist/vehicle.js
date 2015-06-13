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
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNyYy92ZWhpY2xlLmVzNi5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiOzs7Ozs7Ozs7O0FBQUEsSUFBSSxNQUFNLEdBQUcsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDO0FBQy9CLElBQUksWUFBWSxHQUFHLE9BQU8sQ0FBQyxpQkFBaUIsQ0FBQyxDQUFDO0FBQzlDLElBQUksaUJBQWlCLEdBQUcsT0FBTyxDQUFDLHNCQUFzQixDQUFDLENBQUM7O0lBRWxELE9BQU87QUFDQSxXQURQLE9BQU8sQ0FDQyxLQUFLLEVBQUUsUUFBUSxFQUFFLGNBQWMsRUFBRSxRQUFRLEVBQUUsUUFBUSxFQUFFLE9BQU8sRUFBRSxJQUFJLEVBQUUsV0FBVyxFQUFFLFFBQVEsRUFBRTswQkFEbkcsT0FBTzs7QUFFVCwrQkFGRSxPQUFPLDZDQUVILFFBQVEsRUFBRSxjQUFjLEVBQUUsUUFBUSxFQUFFLFFBQVEsRUFBRSxPQUFPLEVBQUUsSUFBSSxFQUFFLFdBQVcsRUFBRSxRQUFRLEVBQUU7O0FBRTFGLFFBQUksQ0FBQyxLQUFLLEdBQUcsS0FBSyxDQUFDO0FBQ25CLFFBQUksQ0FBQyxTQUFTLEdBQUcsSUFBSSxpQkFBaUIsQ0FBQyxJQUFJLENBQUMsQ0FBQztHQUM5Qzs7WUFORyxPQUFPOztlQUFQLE9BQU87O1dBUUwsZ0JBQUMsRUFBRSxFQUFFOztBQUVULFVBQUksY0FBYyxHQUFHLElBQUksQ0FBQyxTQUFTLENBQUMsU0FBUyxFQUFFLENBQUM7OztBQUdoRCxVQUFJLENBQUMsWUFBWSxHQUFHLE1BQU0sQ0FBQyxNQUFNLENBQUMsY0FBYyxFQUFFLElBQUksQ0FBQyxJQUFJLENBQUMsQ0FBQzs7QUFFN0QsVUFBSSxDQUFDLFFBQVEsQ0FBQyxHQUFHLENBQUMsTUFBTSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsWUFBWSxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUM7O0FBRTFELFVBQUksQ0FBQyxRQUFRLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxRQUFRLENBQUMsQ0FBQzs7QUFFdEMsVUFBSSxDQUFDLFFBQVEsQ0FBQyxHQUFHLENBQUMsTUFBTSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsUUFBUSxFQUFFLEVBQUUsQ0FBQyxDQUFDLENBQUM7Ozs7QUFJdEQsVUFBSSxJQUFJLENBQUMsUUFBUSxDQUFDLGFBQWEsRUFBRSxHQUFHLE9BQU8sRUFBRTtBQUMzQyxZQUFJLENBQUMsT0FBTyxHQUFHLE1BQU0sQ0FBQyxTQUFTLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDO0FBQy9DLFlBQUksQ0FBQyxJQUFJLEdBQUcsTUFBTSxDQUFDLFVBQVUsQ0FBQyxJQUFJLENBQUMsT0FBTyxDQUFDLENBQUM7T0FDN0M7S0FDRjs7O1dBRUssZ0JBQUMsR0FBRyxFQUFFLE9BQU8sbUJBQWtCLEVBQUU7OztTQTdCbkMsT0FBTztHQUFTLFlBQVk7O0FBZ0NsQyxNQUFNLENBQUMsT0FBTyxHQUFHLE9BQU8sQ0FBQyIsImZpbGUiOiJzcmMvdmVoaWNsZS5lczYuanMiLCJzb3VyY2VzQ29udGVudCI6WyJ2YXIgVmVjdG9yID0gcmVxdWlyZSgndmVjdG9yJyk7XG52YXIgTW92aW5nRW50aXR5ID0gcmVxdWlyZSgnLi9tb3ZpbmctZW50aXR5Jyk7XG52YXIgU3RlZXJpbmdCZWhhdmlvcnMgPSByZXF1aXJlKCcuL3N0ZWVyaW5nLWJlaGF2aW9ycycpO1xuXG5jbGFzcyBWZWhpY2xlIGV4dGVuZHMgTW92aW5nRW50aXR5IHtcbiAgY29uc3RydWN0b3Iod29ybGQsIHBvc2l0aW9uLCBib3VuZGluZ1JhZGl1cywgdmVsb2NpdHksIG1heFNwZWVkLCBoZWFkaW5nLCBtYXNzLCBtYXhUdXJuUmF0ZSwgbWF4Rm9yY2UpIHtcbiAgICBzdXBlcihwb3NpdGlvbiwgYm91bmRpbmdSYWRpdXMsIHZlbG9jaXR5LCBtYXhTcGVlZCwgaGVhZGluZywgbWFzcywgbWF4VHVyblJhdGUsIG1heEZvcmNlKTtcbiAgICAvLyBjYWxsIHNvbWUga2luZCBvZiBzdXBlclxuICAgIHRoaXMud29ybGQgPSB3b3JsZDtcbiAgICB0aGlzLnN0ZWVyaW5ncyA9IG5ldyBTdGVlcmluZ0JlaGF2aW9ycyh0aGlzKTtcbiAgfVxuXG4gIHVwZGF0ZShkdCkge1xuICAgIC8vIGRlZmluZSBhbGwgdGhlIGZvcmNlcyBhcHBsaWVkIG9uIHRoZSBhZ2VudFxuICAgIHZhciBzdGVlcmluZ0ZvcmNlcyA9IHRoaXMuc3RlZXJpbmdzLmNhbGN1bGF0ZSgpO1xuICAgIC8vIGNvbnNvbGUubG9nKHN0ZWVyaW5nRm9yY2VzKTtcbiAgICAvLyBuZXd0b25cbiAgICB0aGlzLmFjY2VsZXJhdGlvbiA9IFZlY3Rvci5kaXZpZGUoc3RlZXJpbmdGb3JjZXMsIHRoaXMubWFzcyk7XG4gICAgLy8gcmV2ZXJzZSBmb3VyaWVyIGludGVncmF0aW9uIC0gY291bGQgYmUgYSBzZXJ2aWNlXG4gICAgdGhpcy52ZWxvY2l0eS5hZGQoVmVjdG9yLm11bHRpcGx5KHRoaXMuYWNjZWxlcmF0aW9uLCBkdCkpO1xuICAgIC8vIHZlaGljbGUgY2Fubm90IGdvIGJleW9uZyBtYXggc3BlZWRcbiAgICB0aGlzLnZlbG9jaXR5LnRydW5jYXRlKHRoaXMubWF4U3BlZWQpO1xuICAgIC8vIHVwZGF0ZSBsb2NhdGlvblxuICAgIHRoaXMucG9zaXRpb24uYWRkKFZlY3Rvci5tdWx0aXBseSh0aGlzLnZlbG9jaXR5LCBkdCkpO1xuXG4gICAgLy8gcHJldmVudCBzdHVwaWQgZGlzcGxheSBiZWhhdmlvciAodHVybmluZyB3aGVuIHN0b3BwZWQpXG4gICAgLy8gY29uc29sZS5sb2codGhpcy52ZWxvY2l0eS5tYWduaXR1ZGVTcXJ0KCkpO1xuICAgIGlmICh0aGlzLnZlbG9jaXR5Lm1hZ25pdHVkZVNxcnQoKSA+IDAuMDAwMDEpIHtcbiAgICAgIHRoaXMuaGVhZGluZyA9IFZlY3Rvci5ub3JtYWxpemUodGhpcy52ZWxvY2l0eSk7XG4gICAgICB0aGlzLnNpZGUgPSBWZWN0b3Iub3J0aG9nb25hbCh0aGlzLmhlYWRpbmcpO1xuICAgIH1cbiAgfVxuXG4gIHJlbmRlcihjdHgsIGJ1ZmZlcnMvKiwgd29ybGRTaXplICovKSB7fVxufVxuXG5tb2R1bGUuZXhwb3J0cyA9IFZlaGljbGU7XG4iXX0=