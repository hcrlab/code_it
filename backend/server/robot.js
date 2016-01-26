// The implementation of the primitives.
Robot = function() {
  
  var displayMessage = Meteor.wrapAsync(function(h1text, h2text, timeout, callback) {
    console.log('Displaying h1: ' + h1text + ', h2: ' + h2text);
    //var display = new ROSLIB.Topic({
    //  ros: ROS,
    //  name: '/code_it/display',
    //  messageType: 'code_it/DisplayParams'
    //});
    //var msg = new ROSLIB.Message({
    //  display_type: 'display_message',
    //  h1_text: h1text,
    //  h2_text: h2text,
    //  timeout: timeout
    //});
    //display.publish(msg);

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
