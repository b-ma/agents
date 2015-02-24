var vector = require('vector');
var BaseGameEntity = require('./base-game-entity');

var entityType = 0;

class MovingEntity extends BaseGameEntity {

  constructor(position, boundingRadius, velocity, maxSpeed, heading, mass, maxTurnRate, maxForce) {
    super(entityType, position, boundingRadius);

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
}

module.exports = MovingEntity
