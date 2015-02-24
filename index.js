var BaseGameEntity = require('./lib/base-game-entity');
var MovingEntity = require('./lib/moving-entity');
var Vehicle = require('./lib/vehicle');

var agents = {
  BaseGameEntity: BaseGameEntity,
  MovingEntity: MovingEntity,
  Vehicle: Vehicle
};

// console.log(agents);

module.exports = agents;
