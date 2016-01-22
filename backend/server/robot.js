// The implementation of the primitives.
Robot = function() {
  
  var displayMessage = Meteor.wrapAsync(function(h1text, h2text, timeout, callback) {
    var display = new ROSLIB.Topic({
      ros: ROS,
      name: '/code_it/display',
      messageType: 'code_it/DisplayParams'
    });
    var msg = new ROSLIB.Message({
      data: h1text
    });
    display.publish(msg);

    if (timeout > 0) {
      Meteor.setTimeout(function() {
        callback(null, null); // err, result
      }, timeout * 1000);
    } else {
      callback(null, null);
    }
  });

  return {
    displayMessage: displayMessage,
  };
}();
