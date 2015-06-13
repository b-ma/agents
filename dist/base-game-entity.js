'use strict';

var _createClass = require('babel-runtime/helpers/create-class')['default'];

var _classCallCheck = require('babel-runtime/helpers/class-call-check')['default'];

var vector = require('vector');

var defaultEntityType = -1;
var id = 0;

var BaseGameEntity = (function () {
  function BaseGameEntity() {
    var entityType = arguments[0] === undefined ? defaultEntityType : arguments[0];
    var position = arguments[1] === undefined ? vector(0, 0) : arguments[1];
    var boundingRadius = arguments[2] === undefined ? 0 : arguments[2];

    _classCallCheck(this, BaseGameEntity);

    this.id = id++;
    this.entityType = entityType;
    this.position = position;
    this.boundingRadius = boundingRadius;
    // general use tag
    this._tag = false;
  }

  _createClass(BaseGameEntity, [{
    key: 'update',

    // interface
    value: function update(dt) {}
  }, {
    key: 'render',
    value: function render(ctx, buffers) {}
  }, {
    key: 'handleMessage',
    value: function handleMessage(message) {
      return false;
    }
  }, {
    key: 'isTagged',
    value: function isTagged() {
      return this._tag;
    }
  }, {
    key: 'tag',
    value: function tag() {
      this._tag = true;
    }
  }, {
    key: 'unTag',
    value: function unTag() {
      this._tag = false;
    }
  }]);

  return BaseGameEntity;
})();

module.exports = BaseGameEntity;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNyYy92ZWhpY2xlLmVzNi5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiOzs7Ozs7QUFBQSxJQUFJLE1BQU0sR0FBRyxPQUFPLENBQUMsUUFBUSxDQUFDLENBQUM7O0FBRS9CLElBQUksaUJBQWlCLEdBQUcsQ0FBQyxDQUFDLENBQUM7QUFDM0IsSUFBSSxFQUFFLEdBQUcsQ0FBQyxDQUFDOztJQUVMLGNBQWM7QUFDUCxXQURQLGNBQWMsR0FDdUU7UUFBN0UsVUFBVSxnQ0FBRyxpQkFBaUI7UUFBRSxRQUFRLGdDQUFHLE1BQU0sQ0FBQyxDQUFDLEVBQUUsQ0FBQyxDQUFDO1FBQUUsY0FBYyxnQ0FBRyxDQUFDOzswQkFEbkYsY0FBYzs7QUFFaEIsUUFBSSxDQUFDLEVBQUUsR0FBRyxFQUFFLEVBQUUsQ0FBQztBQUNmLFFBQUksQ0FBQyxVQUFVLEdBQUcsVUFBVSxDQUFDO0FBQzdCLFFBQUksQ0FBQyxRQUFRLEdBQUcsUUFBUSxDQUFDO0FBQ3pCLFFBQUksQ0FBQyxjQUFjLEdBQUcsY0FBYyxDQUFDOztBQUVyQyxRQUFJLENBQUMsSUFBSSxHQUFHLEtBQUssQ0FBQztHQUNuQjs7ZUFSRyxjQUFjOzs7O1dBV1osZ0JBQUMsRUFBRSxFQUFFLEVBQUU7OztXQUNQLGdCQUFDLEdBQUcsRUFBRSxPQUFPLEVBQUUsRUFBRTs7O1dBQ1YsdUJBQUMsT0FBTyxFQUFFO0FBQUUsYUFBTyxLQUFLLENBQUM7S0FBRTs7O1dBRWhDLG9CQUFHO0FBQUUsYUFBTyxJQUFJLENBQUMsSUFBSSxDQUFDO0tBQUU7OztXQUM3QixlQUFHO0FBQUUsVUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUM7S0FBRTs7O1dBQ3RCLGlCQUFHO0FBQUUsVUFBSSxDQUFDLElBQUksR0FBRyxLQUFLLENBQUM7S0FBRTs7O1NBakIxQixjQUFjOzs7QUFvQnBCLE1BQU0sQ0FBQyxPQUFPLEdBQUcsY0FBYyxDQUFDIiwiZmlsZSI6InNyYy92ZWhpY2xlLmVzNi5qcyIsInNvdXJjZXNDb250ZW50IjpbInZhciB2ZWN0b3IgPSByZXF1aXJlKCd2ZWN0b3InKTtcblxudmFyIGRlZmF1bHRFbnRpdHlUeXBlID0gLTE7XG52YXIgaWQgPSAwO1xuXG5jbGFzcyBCYXNlR2FtZUVudGl0eSB7XG4gIGNvbnN0cnVjdG9yKGVudGl0eVR5cGUgPSBkZWZhdWx0RW50aXR5VHlwZSwgcG9zaXRpb24gPSB2ZWN0b3IoMCwgMCksIGJvdW5kaW5nUmFkaXVzID0gMCkge1xuICAgIHRoaXMuaWQgPSBpZCsrO1xuICAgIHRoaXMuZW50aXR5VHlwZSA9IGVudGl0eVR5cGU7XG4gICAgdGhpcy5wb3NpdGlvbiA9IHBvc2l0aW9uO1xuICAgIHRoaXMuYm91bmRpbmdSYWRpdXMgPSBib3VuZGluZ1JhZGl1cztcbiAgICAvLyBnZW5lcmFsIHVzZSB0YWdcbiAgICB0aGlzLl90YWcgPSBmYWxzZTtcbiAgfVxuXG4gIC8vIGludGVyZmFjZVxuICB1cGRhdGUoZHQpIHt9XG4gIHJlbmRlcihjdHgsIGJ1ZmZlcnMpIHt9XG4gIGhhbmRsZU1lc3NhZ2UobWVzc2FnZSkgeyByZXR1cm4gZmFsc2U7IH1cblxuICBpc1RhZ2dlZCgpIHsgcmV0dXJuIHRoaXMuX3RhZzsgfVxuICB0YWcoKSB7IHRoaXMuX3RhZyA9IHRydWU7IH1cbiAgdW5UYWcoKSB7IHRoaXMuX3RhZyA9IGZhbHNlOyB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gQmFzZUdhbWVFbnRpdHk7XG4iXX0=