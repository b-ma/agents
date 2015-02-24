"use strict";

var _babelHelpers = require("babel-runtime/helpers")["default"];

var _core = require("babel-runtime/core-js")["default"];

var Vector = require("vector");
var MovingEntity = require("./moving-entity");
var SteeringBehaviors = require("./steering-behaviors");

var Vehicle = (function (MovingEntity) {
  function Vehicle(world, position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    _babelHelpers.classCallCheck(this, Vehicle);

    _babelHelpers.get(_core.Object.getPrototypeOf(Vehicle.prototype), "constructor", this).call(this, position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce);
    // call some kind of super
    this.world = world;
    this.steerings = new SteeringBehaviors(this);
  }

  _babelHelpers.inherits(Vehicle, MovingEntity);

  _babelHelpers.prototypeProperties(Vehicle, null, {
    update: {
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
      },
      writable: true,
      configurable: true
    },
    render: {
      value: function render(ctx, buffers /*, worldSize */) {},
      writable: true,
      configurable: true
    }
  });

  return Vehicle;
})(MovingEntity);

module.exports = Vehicle;

//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4vbGliL3ZlaGljbGUuZXM2LmpzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7Ozs7OztBQUFBLElBQUksTUFBTSxHQUFHLE9BQU8sQ0FBQyxRQUFRLENBQUMsQ0FBQztBQUMvQixJQUFJLFlBQVksR0FBRyxPQUFPLENBQUMsaUJBQWlCLENBQUMsQ0FBQztBQUM5QyxJQUFJLGlCQUFpQixHQUFHLE9BQU8sQ0FBQyxzQkFBc0IsQ0FBQyxDQUFDOztJQUVsRCxPQUFPLGNBQVMsWUFBWTtBQUNyQixXQURQLE9BQU8sQ0FDQyxLQUFLLEVBQUUsUUFBUSxFQUFFLGNBQWMsRUFBRSxRQUFRLEVBQUUsUUFBUSxFQUFFLE9BQU8sRUFBRSxJQUFJLEVBQUUsV0FBVyxFQUFFLFFBQVE7dUNBRGpHLE9BQU87O0FBRVQsa0RBRkUsT0FBTyw2Q0FFSCxRQUFRLEVBQUUsY0FBYyxFQUFFLFFBQVEsRUFBRSxRQUFRLEVBQUUsT0FBTyxFQUFFLElBQUksRUFBRSxXQUFXLEVBQUUsUUFBUSxFQUFFOztBQUUxRixRQUFJLENBQUMsS0FBSyxHQUFHLEtBQUssQ0FBQztBQUNuQixRQUFJLENBQUMsU0FBUyxHQUFHLElBQUksaUJBQWlCLENBQUMsSUFBSSxDQUFDLENBQUM7R0FDOUM7O3lCQU5HLE9BQU8sRUFBUyxZQUFZOztvQ0FBNUIsT0FBTztBQVFYLFVBQU07YUFBQSxnQkFBQyxFQUFFLEVBQUU7O0FBRVQsWUFBSSxjQUFjLEdBQUcsSUFBSSxDQUFDLFNBQVMsQ0FBQyxTQUFTLEVBQUUsQ0FBQzs7O0FBR2hELFlBQUksQ0FBQyxZQUFZLEdBQUcsTUFBTSxDQUFDLE1BQU0sQ0FBQyxjQUFjLEVBQUUsSUFBSSxDQUFDLElBQUksQ0FBQyxDQUFDOztBQUU3RCxZQUFJLENBQUMsUUFBUSxDQUFDLEdBQUcsQ0FBQyxNQUFNLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxZQUFZLEVBQUUsRUFBRSxDQUFDLENBQUMsQ0FBQzs7QUFFMUQsWUFBSSxDQUFDLFFBQVEsQ0FBQyxRQUFRLENBQUMsSUFBSSxDQUFDLFFBQVEsQ0FBQyxDQUFDOztBQUV0QyxZQUFJLENBQUMsUUFBUSxDQUFDLEdBQUcsQ0FBQyxNQUFNLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxRQUFRLEVBQUUsRUFBRSxDQUFDLENBQUMsQ0FBQzs7OztBQUl0RCxZQUFJLElBQUksQ0FBQyxRQUFRLENBQUMsYUFBYSxFQUFFLEdBQUcsT0FBTyxFQUFFO0FBQzNDLGNBQUksQ0FBQyxPQUFPLEdBQUcsTUFBTSxDQUFDLFNBQVMsQ0FBQyxJQUFJLENBQUMsUUFBUSxDQUFDLENBQUM7QUFDL0MsY0FBSSxDQUFDLElBQUksR0FBRyxNQUFNLENBQUMsVUFBVSxDQUFDLElBQUksQ0FBQyxPQUFPLENBQUMsQ0FBQztTQUM3QztPQUNGOzs7O0FBRUQsVUFBTTthQUFBLGdCQUFDLEdBQUcsRUFBRSxPQUFPLG1CQUFrQixFQUFFOzs7Ozs7U0E3Qm5DLE9BQU87R0FBUyxZQUFZOztBQWdDbEMsTUFBTSxDQUFDLE9BQU8sR0FBRyxPQUFPLENBQUMiLCJmaWxlIjoiLi9saWIvdmVoaWNsZS5qcyIsInNvdXJjZXNDb250ZW50IjpbInZhciBWZWN0b3IgPSByZXF1aXJlKCd2ZWN0b3InKTtcbnZhciBNb3ZpbmdFbnRpdHkgPSByZXF1aXJlKCcuL21vdmluZy1lbnRpdHknKTtcbnZhciBTdGVlcmluZ0JlaGF2aW9ycyA9IHJlcXVpcmUoJy4vc3RlZXJpbmctYmVoYXZpb3JzJyk7XG5cbmNsYXNzIFZlaGljbGUgZXh0ZW5kcyBNb3ZpbmdFbnRpdHkge1xuICBjb25zdHJ1Y3Rvcih3b3JsZCwgcG9zaXRpb24sIGJvdW5kaW5nUmFkaXVzLCB2ZWxvY2l0eSwgbWF4U3BlZWQsIGhlYWRpbmcsIG1hc3MsIG1heFR1cm5SYXRlLCBtYXhGb3JjZSkge1xuICAgIHN1cGVyKHBvc2l0aW9uLCBib3VuZGluZ1JhZGl1cywgdmVsb2NpdHksIG1heFNwZWVkLCBoZWFkaW5nLCBtYXNzLCBtYXhUdXJuUmF0ZSwgbWF4Rm9yY2UpO1xuICAgIC8vIGNhbGwgc29tZSBraW5kIG9mIHN1cGVyXG4gICAgdGhpcy53b3JsZCA9IHdvcmxkO1xuICAgIHRoaXMuc3RlZXJpbmdzID0gbmV3IFN0ZWVyaW5nQmVoYXZpb3JzKHRoaXMpO1xuICB9XG5cbiAgdXBkYXRlKGR0KSB7XG4gICAgLy8gZGVmaW5lIGFsbCB0aGUgZm9yY2VzIGFwcGxpZWQgb24gdGhlIGFnZW50XG4gICAgdmFyIHN0ZWVyaW5nRm9yY2VzID0gdGhpcy5zdGVlcmluZ3MuY2FsY3VsYXRlKCk7XG4gICAgLy8gY29uc29sZS5sb2coc3RlZXJpbmdGb3JjZXMpO1xuICAgIC8vIG5ld3RvblxuICAgIHRoaXMuYWNjZWxlcmF0aW9uID0gVmVjdG9yLmRpdmlkZShzdGVlcmluZ0ZvcmNlcywgdGhpcy5tYXNzKTtcbiAgICAvLyByZXZlcnNlIGZvdXJpZXIgaW50ZWdyYXRpb24gLSBjb3VsZCBiZSBhIHNlcnZpY2VcbiAgICB0aGlzLnZlbG9jaXR5LmFkZChWZWN0b3IubXVsdGlwbHkodGhpcy5hY2NlbGVyYXRpb24sIGR0KSk7XG4gICAgLy8gdmVoaWNsZSBjYW5ub3QgZ28gYmV5b25nIG1heCBzcGVlZFxuICAgIHRoaXMudmVsb2NpdHkudHJ1bmNhdGUodGhpcy5tYXhTcGVlZCk7XG4gICAgLy8gdXBkYXRlIGxvY2F0aW9uXG4gICAgdGhpcy5wb3NpdGlvbi5hZGQoVmVjdG9yLm11bHRpcGx5KHRoaXMudmVsb2NpdHksIGR0KSk7XG5cbiAgICAvLyBwcmV2ZW50IHN0dXBpZCBkaXNwbGF5IGJlaGF2aW9yICh0dXJuaW5nIHdoZW4gc3RvcHBlZClcbiAgICAvLyBjb25zb2xlLmxvZyh0aGlzLnZlbG9jaXR5Lm1hZ25pdHVkZVNxcnQoKSk7XG4gICAgaWYgKHRoaXMudmVsb2NpdHkubWFnbml0dWRlU3FydCgpID4gMC4wMDAwMSkge1xuICAgICAgdGhpcy5oZWFkaW5nID0gVmVjdG9yLm5vcm1hbGl6ZSh0aGlzLnZlbG9jaXR5KTtcbiAgICAgIHRoaXMuc2lkZSA9IFZlY3Rvci5vcnRob2dvbmFsKHRoaXMuaGVhZGluZyk7XG4gICAgfVxuICB9XG5cbiAgcmVuZGVyKGN0eCwgYnVmZmVycy8qLCB3b3JsZFNpemUgKi8pIHt9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gVmVoaWNsZTtcbiJdfQ==