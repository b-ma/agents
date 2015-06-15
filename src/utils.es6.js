var utils = {

  TwoPI: (2 * Math.PI),

  rand: function() {
    return Math.random();
  },

  degsToRads: function(degs) {
    return this.TwoPI * (degs / 360);
  },

  // return val between -1 and 1
  randClamped: function() {
    // return Math.random() * Math.random()
    return Math.random() * 2 - 1;
  },

  randBool:  function() {
    return (Math.random() > 0.5);
  }

}

module.exports = utils;
