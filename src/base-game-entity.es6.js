var vector = require('vector');

var defaultEntityType = -1;
var id = 0;

class BaseGameEntity {
  constructor(entityType = defaultEntityType, position = vector(0, 0), boundingRadius = 0) {
    this.id = id++;
    this.entityType = entityType;
    this.position = position;
    this.boundingRadius = boundingRadius;
    // general use tag
    this._tag = false;
  }

  // interface
  update(dt) {}
  render(ctx, buffers) {}
  handleMessage(message) { return false; }

  isTagged() { return this._tag; }
  tag() { this._tag = true; }
  unTag() { this._tag = false; }
}

module.exports = BaseGameEntity;
