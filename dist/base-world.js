'use strict';

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var Vector = require('vector');

var BaseWorld = (function () {
  function BaseWorld(size) {
    _classCallCheck(this, BaseWorld);

    this.isStarted = false;
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

    this.params = {
      viewDistance: 40
    };

    this.boids = [];
    this.obstacles = [];
    this.walls = [];

    // these should be in the boid
    this.target = null;
    this.pursuer = null;
    this.evader = null;
  }

  _createClass(BaseWorld, [{
    key: 'tagObstaclesWithinRange',

    // shouldn't be here, used in steering behavior
    value: function tagObstaclesWithinRange(entity, range) {
      this.tagNeighbors(entity, this.obstacles, range);
    }
  }, {
    key: 'tagBoidsWithinRange',
    value: function tagBoidsWithinRange(entity) {
      this.tagNeighbors(entity, this.boids, this.params.viewDistance);
    }
  }, {
    key: 'tagNeighbors',
    value: function tagNeighbors(entity, entities, radius) {
      // reset neighbors
      entity.neighbors.length = 0;
      var currentEntity = null;
      // entities.forEach(function(currentEntity) {
      for (var i = 0, l = entities.length; i < l; i++) {
        var _currentEntity = entities[i];
        // clear any current tag
        _currentEntity.unTag();

        if (_currentEntity === entity) {
          continue;
        }

        var to = Vector.substract(_currentEntity.position, entity.position);
        // the bounding radius of the other must be taken in account for the range
        var range = radius + _currentEntity.boundingRadius;
        // if entity is within range tag it for futher consideration
        // use square space to save memory
        var shouldBeTagged = to.magnitudeSqrt() < range * range;

        if (shouldBeTagged) {
          _currentEntity.tag();
          entity.neighbors.push(_currentEntity);
        }
      }

      // debug - this can't work...
      // if (entity.isDebugged) {
      //   currentEntity.debugTagged = false;
      //   if (shouldBeTagged) { currentEntity.debugTagged = true; }
      // }
    }
  }, {
    key: 'update',
    value: function update(dt) {}
  }, {
    key: 'render',
    value: function render(ctx, buffers) {}
  }]);

  return BaseWorld;
})();

module.exports = BaseWorld;
//# sourceMappingURL=data:application/json;base64,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QUMsT0FBTyxHQUFHLFNBQVMsQ0FBQyIsImZpbGUiOiJzcmMvYmFzZS13b3JsZC5lczYuanMiLCJzb3VyY2VzQ29udGVudCI6WyJjb25zdCBWZWN0b3IgPSByZXF1aXJlKCd2ZWN0b3InKTtcblxuY2xhc3MgQmFzZVdvcmxkIHtcbiAgY29uc3RydWN0b3Ioc2l6ZSkge1xuICAgIHRoaXMuaXNTdGFydGVkICA9IGZhbHNlO1xuICAgIC8vIG11bHRpcGxpZXJzIHRvIHRlYWsgc3RlZXJpbmdcbiAgICB0aGlzLnN0ZWVyaW5nVHdlYWtlcnMgPSB7XG4gICAgICBzZWVrOiAwLFxuICAgICAgZmxlZTogMCxcbiAgICAgIGFycml2ZTogMCxcbiAgICAgIHB1cnN1aXQ6IDAsXG4gICAgICBldmFkZTogMCxcbiAgICAgIHdhbmRlcjogMCxcbiAgICAgIG9ic3RhY2xlQXZvaWRhbmNlOiAwLFxuICAgICAgd2FsbEF2b2lkYW5jZTogMCxcbiAgICAgIHNlcGFyYXRpb246IDAsXG4gICAgICBhbGlnbm1lbnQ6IDAsXG4gICAgICBjb2hlc2lvbjogMFxuICAgIH07XG5cbiAgICB0aGlzLnBhcmFtcyA9IHtcbiAgICAgIHZpZXdEaXN0YW5jZTogNDBcbiAgICB9O1xuXG4gICAgdGhpcy5ib2lkcyA9IFtdO1xuICAgIHRoaXMub2JzdGFjbGVzID0gW107XG4gICAgdGhpcy53YWxscyA9IFtdO1xuXG4gICAgLy8gdGhlc2Ugc2hvdWxkIGJlIGluIHRoZSBib2lkXG4gICAgdGhpcy50YXJnZXQgPSBudWxsO1xuICAgIHRoaXMucHVyc3VlciA9IG51bGw7XG4gICAgdGhpcy5ldmFkZXIgPSBudWxsO1xuICB9XG5cbiAgLy8gc2hvdWxkbid0IGJlIGhlcmUsIHVzZWQgaW4gc3RlZXJpbmcgYmVoYXZpb3JcbiAgdGFnT2JzdGFjbGVzV2l0aGluUmFuZ2UoZW50aXR5LCByYW5nZSkge1xuICAgIHRoaXMudGFnTmVpZ2hib3JzKGVudGl0eSwgdGhpcy5vYnN0YWNsZXMsIHJhbmdlKTtcbiAgfVxuXG4gIHRhZ0JvaWRzV2l0aGluUmFuZ2UoZW50aXR5KSB7XG4gICAgdGhpcy50YWdOZWlnaGJvcnMoZW50aXR5LCB0aGlzLmJvaWRzLCB0aGlzLnBhcmFtcy52aWV3RGlzdGFuY2UpO1xuICB9XG5cbiAgdGFnTmVpZ2hib3JzKGVudGl0eSwgZW50aXRpZXMsIHJhZGl1cykge1xuICAgIC8vIHJlc2V0IG5laWdoYm9yc1xuICAgIGVudGl0eS5uZWlnaGJvcnMubGVuZ3RoID0gMDtcbiAgICB2YXIgY3VycmVudEVudGl0eSA9IG51bGw7XG4gICAgLy8gZW50aXRpZXMuZm9yRWFjaChmdW5jdGlvbihjdXJyZW50RW50aXR5KSB7XG4gICAgZm9yICh2YXIgaSA9IDAsIGwgPSBlbnRpdGllcy5sZW5ndGg7IGkgPCBsOyBpKyspIHtcbiAgICAgIGxldCBjdXJyZW50RW50aXR5ID0gZW50aXRpZXNbaV07XG4gICAgICAvLyBjbGVhciBhbnkgY3VycmVudCB0YWdcbiAgICAgIGN1cnJlbnRFbnRpdHkudW5UYWcoKTtcblxuICAgICAgaWYgKGN1cnJlbnRFbnRpdHkgPT09IGVudGl0eSkgeyBjb250aW51ZTsgfVxuXG4gICAgICB2YXIgdG8gPSBWZWN0b3Iuc3Vic3RyYWN0KGN1cnJlbnRFbnRpdHkucG9zaXRpb24sIGVudGl0eS5wb3NpdGlvbik7XG4gICAgICAvLyB0aGUgYm91bmRpbmcgcmFkaXVzIG9mIHRoZSBvdGhlciBtdXN0IGJlIHRha2VuIGluIGFjY291bnQgZm9yIHRoZSByYW5nZVxuICAgICAgdmFyIHJhbmdlID0gcmFkaXVzICsgY3VycmVudEVudGl0eS5ib3VuZGluZ1JhZGl1cztcbiAgICAgIC8vIGlmIGVudGl0eSBpcyB3aXRoaW4gcmFuZ2UgdGFnIGl0IGZvciBmdXRoZXIgY29uc2lkZXJhdGlvblxuICAgICAgLy8gdXNlIHNxdWFyZSBzcGFjZSB0byBzYXZlIG1lbW9yeVxuICAgICAgdmFyIHNob3VsZEJlVGFnZ2VkID0gKHRvLm1hZ25pdHVkZVNxcnQoKSA8IChyYW5nZSAqIHJhbmdlKSk7XG5cbiAgICAgIGlmIChzaG91bGRCZVRhZ2dlZCkge1xuICAgICAgICBjdXJyZW50RW50aXR5LnRhZygpO1xuICAgICAgICBlbnRpdHkubmVpZ2hib3JzLnB1c2goY3VycmVudEVudGl0eSk7XG4gICAgICB9XG4gICAgfVxuXG4gICAgLy8gZGVidWcgLSB0aGlzIGNhbid0IHdvcmsuLi5cbiAgICAvLyBpZiAoZW50aXR5LmlzRGVidWdnZWQpIHtcbiAgICAvLyAgIGN1cnJlbnRFbnRpdHkuZGVidWdUYWdnZWQgPSBmYWxzZTtcbiAgICAvLyAgIGlmIChzaG91bGRCZVRhZ2dlZCkgeyBjdXJyZW50RW50aXR5LmRlYnVnVGFnZ2VkID0gdHJ1ZTsgfVxuICAgIC8vIH1cbiAgfVxuXG4gIHVwZGF0ZShkdCkge31cblxuICByZW5kZXIoY3R4LCBidWZmZXJzKSB7fVxufVxuXG5tb2R1bGUuZXhwb3J0cyA9IEJhc2VXb3JsZDtcblxuIl19