// The implementation of the primitives.
Robot = function() {
  
  var displayMessage = Meteor.wrapAsync(function(h1text, h2text, timeout, callback) {
    console.log('Displaying h1: ' + h1text + ', h2: ' + h2text);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/display_message',
      serviceType : 'code_it/DisplayMessage'
    });

    var request = new ROSLIB.ServiceRequest({
      h1_text: h1text,
      h2_text: h2text,
      timeout: timeout
    });

    client.callService(request, function(result) {
    });

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
