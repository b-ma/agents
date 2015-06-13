var vector = require('vector');
var BaseGameEntity = require('./base-game-entity');
var SteeringBehaviors = require('./steering-behaviors');

var entityType = 0;

class MovingEntity extends BaseGameEntity {

  constructor(world, position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    super(entityType, position, boundingRadius);

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
    // maximum force the entity can produce to power itself (think motor engine)
    this.maxForce = maxForce;
    // maximum rate at which the entity can rotate (rad/sec)
    this.maxTurnRate = maxTurnRate;
    // ignore scale
  }

  speed() { return this.velocity.magnitude(); }
  speedSqrt() { return this.velocity.magnitudeSqrt(); }

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

module.exports = MovingEntity
