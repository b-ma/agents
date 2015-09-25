
class BaseWorld {
  constructor() {
    this.isStarted  = false;
    // multipliers to teak steering
    this.steeringTweakers = {
      seek: 0,
      flee: 0,
      arrive: 0,
      pursuit: 0,
      evade: 0,
      wander: 0,
      obstacleAvoidance: 0,
      wallAvoidance: 0,
      separation: 0,
      alignment: 0,
      cohesion: 0
    };

    this.boids = [];
    this.obstacles = [];
    this.walls = [];

    // these should be in the boid
    this.target = null;
    this.pursuer = null;
    this.evader = null;
  }

  // shouldn't be here, used in steering behavior
  tagObstaclesWithinRange(entity, range) {
    this.tagNeighbors(entity, this.obstacles, range);
  },

  tagBoidsWithinRange(entity) {
    this.tagNeighbors(entity, this.boids, params.viewDistance);
  },

  tagNeighbors(entity, entities, radius) {
    // reset neighbors
    entity.neighbors.length = 0;
    var currentEntity = null;
    // entities.forEach(function(currentEntity) {
    for (var i = 0, l = entities.length; i < l; i++) {
      let currentEntity = entities[i];
      // clear any current tag
      currentEntity.unTag();

      if (currentEntity === entity) { continue; }

      var to = Vector.substract(currentEntity.position, entity.position);
      // the bounding radius of the other must be taken in account for the range
      var range = radius + currentEntity.boundingRadius;
      // if entity is within range tag it for futher consideration
      // use square space to save memory
      var shouldBeTagged = (to.magnitudeSqrt() < (range * range));

      if (shouldBeTagged) {
        currentEntity.tag();
        entity.neighbors.push(currentEntity);
      }

      // debug
      if (entity.isLeader) {
        currentEntity.leaderTagged = false;
        if (shouldBeTagged) { currentEntity.leaderTagged = true; }
      }
    };
    // }, this);
  },
}
