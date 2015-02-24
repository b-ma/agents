var Vector = require('vector');
var MovingEntity = require('./moving-entity');
var SteeringBehaviors = require('./steering-behaviors');

class Vehicle extends MovingEntity {
  constructor(world, position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    super(position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce);
    // call some kind of super
    this.world = world;
    this.steerings = new SteeringBehaviors(this);
  }

  update(dt) {
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

  render(ctx, buffers/*, worldSize */) {}
}

module.exports = Vehicle;
