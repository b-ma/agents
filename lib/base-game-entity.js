"use strict";

var _babelHelpers = require("babel-runtime/helpers")["default"];

var vector = require("vector");

var defaultEntityType = -1;
var id = 0;

var BaseGameEntity = (function () {
  function BaseGameEntity() {
    var entityType = arguments[0] === undefined ? defaultEntityType : arguments[0];
    var position = arguments[1] === undefined ? vector(0, 0) : arguments[1];
    var boundingRadius = arguments[2] === undefined ? 0 : arguments[2];

    _babelHelpers.classCallCheck(this, BaseGameEntity);

    this.id = id++;
    this.entityType = entityType;
    this.position = position;
    this.boundingRadius = boundingRadius;
    // general use tag
    this._tag = false;
  }

  _babelHelpers.prototypeProperties(BaseGameEntity, null, {
    update: {

      // interface

      value: function update(dt) {},
      writable: true,
      configurable: true
    },
    render: {
      value: function render(ctx, buffers) {},
      writable: true,
      configurable: true
    },
    handleMessage: {
      value: function handleMessage(message) {
        return false;
      },
      writable: true,
      configurable: true
    },
    isTagged: {
      value: function isTagged() {
        return this._tag;
      },
      writable: true,
      configurable: true
    },
    tag: {
      value: function tag() {
        this._tag = true;
      },
      writable: true,
      configurable: true
    },
    untag: {
      value: function untag() {
        this._tag = false;
      },
      writable: true,
      configurable: true
    }
  });

  return BaseGameEntity;
})();

module.exports = BaseGameEntity;

//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4vbGliL2Jhc2UtZ2FtZS1lbnRpdHkuZXM2LmpzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiI7Ozs7QUFBQSxJQUFJLE1BQU0sR0FBRyxPQUFPLENBQUMsUUFBUSxDQUFDLENBQUM7O0FBRS9CLElBQUksaUJBQWlCLEdBQUcsQ0FBQyxDQUFDLENBQUM7QUFDM0IsSUFBSSxFQUFFLEdBQUcsQ0FBQyxDQUFDOztJQUVMLGNBQWM7QUFDUCxXQURQLGNBQWM7UUFDTixVQUFVLGdDQUFHLGlCQUFpQjtRQUFFLFFBQVEsZ0NBQUcsTUFBTSxDQUFDLENBQUMsRUFBRSxDQUFDLENBQUM7UUFBRSxjQUFjLGdDQUFHLENBQUM7O3VDQURuRixjQUFjOztBQUVoQixRQUFJLENBQUMsRUFBRSxHQUFHLEVBQUUsRUFBRSxDQUFDO0FBQ2YsUUFBSSxDQUFDLFVBQVUsR0FBRyxVQUFVLENBQUM7QUFDN0IsUUFBSSxDQUFDLFFBQVEsR0FBRyxRQUFRLENBQUM7QUFDekIsUUFBSSxDQUFDLGNBQWMsR0FBRyxjQUFjLENBQUM7O0FBRXJDLFFBQUksQ0FBQyxJQUFJLEdBQUcsS0FBSyxDQUFDO0dBQ25COztvQ0FSRyxjQUFjO0FBV2xCLFVBQU07Ozs7YUFBQSxnQkFBQyxFQUFFLEVBQUUsRUFBRTs7OztBQUNiLFVBQU07YUFBQSxnQkFBQyxHQUFHLEVBQUUsT0FBTyxFQUFFLEVBQUU7Ozs7QUFDdkIsaUJBQWE7YUFBQSx1QkFBQyxPQUFPLEVBQUU7QUFBRSxlQUFPLEtBQUssQ0FBQztPQUFFOzs7O0FBRXhDLFlBQVE7YUFBQSxvQkFBRztBQUFFLGVBQU8sSUFBSSxDQUFDLElBQUksQ0FBQztPQUFFOzs7O0FBQ2hDLE9BQUc7YUFBQSxlQUFHO0FBQUUsWUFBSSxDQUFDLElBQUksR0FBRyxJQUFJLENBQUM7T0FBRTs7OztBQUMzQixTQUFLO2FBQUEsaUJBQUc7QUFBRSxZQUFJLENBQUMsSUFBSSxHQUFHLEtBQUssQ0FBQztPQUFFOzs7Ozs7U0FqQjFCLGNBQWM7OztBQW9CcEIsTUFBTSxDQUFDLE9BQU8sR0FBRyxjQUFjLENBQUMiLCJmaWxlIjoiLi9saWIvYmFzZS1nYW1lLWVudGl0eS5qcyIsInNvdXJjZXNDb250ZW50IjpbInZhciB2ZWN0b3IgPSByZXF1aXJlKCd2ZWN0b3InKTtcblxudmFyIGRlZmF1bHRFbnRpdHlUeXBlID0gLTE7XG52YXIgaWQgPSAwO1xuXG5jbGFzcyBCYXNlR2FtZUVudGl0eSB7XG4gIGNvbnN0cnVjdG9yKGVudGl0eVR5cGUgPSBkZWZhdWx0RW50aXR5VHlwZSwgcG9zaXRpb24gPSB2ZWN0b3IoMCwgMCksIGJvdW5kaW5nUmFkaXVzID0gMCkge1xuICAgIHRoaXMuaWQgPSBpZCsrO1xuICAgIHRoaXMuZW50aXR5VHlwZSA9IGVudGl0eVR5cGU7XG4gICAgdGhpcy5wb3NpdGlvbiA9IHBvc2l0aW9uO1xuICAgIHRoaXMuYm91bmRpbmdSYWRpdXMgPSBib3VuZGluZ1JhZGl1cztcbiAgICAvLyBnZW5lcmFsIHVzZSB0YWdcbiAgICB0aGlzLl90YWcgPSBmYWxzZTtcbiAgfVxuXG4gIC8vIGludGVyZmFjZVxuICB1cGRhdGUoZHQpIHt9XG4gIHJlbmRlcihjdHgsIGJ1ZmZlcnMpIHt9XG4gIGhhbmRsZU1lc3NhZ2UobWVzc2FnZSkgeyByZXR1cm4gZmFsc2U7IH1cblxuICBpc1RhZ2dlZCgpIHsgcmV0dXJuIHRoaXMuX3RhZzsgfVxuICB0YWcoKSB7IHRoaXMuX3RhZyA9IHRydWU7IH1cbiAgdW50YWcoKSB7IHRoaXMuX3RhZyA9IGZhbHNlOyB9XG59XG5cbm1vZHVsZS5leHBvcnRzID0gQmFzZUdhbWVFbnRpdHk7XG4iXX0=