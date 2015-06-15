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
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbInNyYy91dGlscy5lczYuanMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7Ozs7O0FBQUEsSUFBSSxNQUFNLEdBQUcsT0FBTyxDQUFDLFFBQVEsQ0FBQyxDQUFDOztBQUUvQixJQUFJLGlCQUFpQixHQUFHLENBQUMsQ0FBQyxDQUFDO0FBQzNCLElBQUksRUFBRSxHQUFHLENBQUMsQ0FBQzs7SUFFTCxjQUFjO0FBQ1AsV0FEUCxjQUFjLEdBQ3VFO1FBQTdFLFVBQVUsZ0NBQUcsaUJBQWlCO1FBQUUsUUFBUSxnQ0FBRyxNQUFNLENBQUMsQ0FBQyxFQUFFLENBQUMsQ0FBQztRQUFFLGNBQWMsZ0NBQUcsQ0FBQzs7MEJBRG5GLGNBQWM7O0FBRWhCLFFBQUksQ0FBQyxFQUFFLEdBQUcsRUFBRSxFQUFFLENBQUM7QUFDZixRQUFJLENBQUMsVUFBVSxHQUFHLFVBQVUsQ0FBQztBQUM3QixRQUFJLENBQUMsUUFBUSxHQUFHLFFBQVEsQ0FBQztBQUN6QixRQUFJLENBQUMsY0FBYyxHQUFHLGNBQWMsQ0FBQzs7QUFFckMsUUFBSSxDQUFDLElBQUksR0FBRyxLQUFLLENBQUM7R0FDbkI7O2VBUkcsY0FBYzs7OztXQVdaLGdCQUFDLEVBQUUsRUFBRSxFQUFFOzs7V0FDUCxnQkFBQyxHQUFHLEVBQUUsT0FBTyxFQUFFLEVBQUU7OztXQUNWLHVCQUFDLE9BQU8sRUFBRTtBQUFFLGFBQU8sS0FBSyxDQUFDO0tBQUU7OztXQUVoQyxvQkFBRztBQUFFLGFBQU8sSUFBSSxDQUFDLElBQUksQ0FBQztLQUFFOzs7V0FDN0IsZUFBRztBQUFFLFVBQUksQ0FBQyxJQUFJLEdBQUcsSUFBSSxDQUFDO0tBQUU7OztXQUN0QixpQkFBRztBQUFFLFVBQUksQ0FBQyxJQUFJLEdBQUcsS0FBSyxDQUFDO0tBQUU7OztTQWpCMUIsY0FBYzs7O0FBb0JwQixNQUFNLENBQUMsT0FBTyxHQUFHLGNBQWMsQ0FBQyIsImZpbGUiOiJzcmMvdXRpbHMuZXM2LmpzIiwic291cmNlc0NvbnRlbnQiOlsidmFyIHZlY3RvciA9IHJlcXVpcmUoJ3ZlY3RvcicpO1xuXG52YXIgZGVmYXVsdEVudGl0eVR5cGUgPSAtMTtcbnZhciBpZCA9IDA7XG5cbmNsYXNzIEJhc2VHYW1lRW50aXR5IHtcbiAgY29uc3RydWN0b3IoZW50aXR5VHlwZSA9IGRlZmF1bHRFbnRpdHlUeXBlLCBwb3NpdGlvbiA9IHZlY3RvcigwLCAwKSwgYm91bmRpbmdSYWRpdXMgPSAwKSB7XG4gICAgdGhpcy5pZCA9IGlkKys7XG4gICAgdGhpcy5lbnRpdHlUeXBlID0gZW50aXR5VHlwZTtcbiAgICB0aGlzLnBvc2l0aW9uID0gcG9zaXRpb247XG4gICAgdGhpcy5ib3VuZGluZ1JhZGl1cyA9IGJvdW5kaW5nUmFkaXVzO1xuICAgIC8vIGdlbmVyYWwgdXNlIHRhZ1xuICAgIHRoaXMuX3RhZyA9IGZhbHNlO1xuICB9XG5cbiAgLy8gaW50ZXJmYWNlXG4gIHVwZGF0ZShkdCkge31cbiAgcmVuZGVyKGN0eCwgYnVmZmVycykge31cbiAgaGFuZGxlTWVzc2FnZShtZXNzYWdlKSB7IHJldHVybiBmYWxzZTsgfVxuXG4gIGlzVGFnZ2VkKCkgeyByZXR1cm4gdGhpcy5fdGFnOyB9XG4gIHRhZygpIHsgdGhpcy5fdGFnID0gdHJ1ZTsgfVxuICB1blRhZygpIHsgdGhpcy5fdGFnID0gZmFsc2U7IH1cbn1cblxubW9kdWxlLmV4cG9ydHMgPSBCYXNlR2FtZUVudGl0eTtcbiJdfQ==