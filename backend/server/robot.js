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

  var askMultipleChoice = Meteor.wrapAsync(function(question, choices, timeout, callback) {
    console.log('Asking: ' + question + ', choices: ' + choices);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/ask_multiple_choice',
      serviceType : 'code_it/AskMultipleChoice'
    });

    var request = new ROSLIB.ServiceRequest({
      question: question,
      choices: choices,
    });

    var funcCall = this;
    funcCall.isRunning = true;
    client.callService(request, function(result) {
      funcCall.isRunning = false;
      callback(null, result.choice);
    });

    if (timeout > 0) {
      Meteor.setTimeout(function() {
        if (funcCall.isRunning) {
          callback(null, null); // err, result
        }
      }, timeout * 1000);
    }
  });

  var goTo = Meteor.wrapAsync(function(location, callback) {
    console.log('Going to: ' + location);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/go_to',
      serviceType : 'code_it/GoTo'
    });

    var request = new ROSLIB.ServiceRequest({
      location: location
    });

    client.callService(request, function(result) {
      console.log('Done navigating to ' + location + ', result:');
      console.log(result);
      if (result.error !== '') { // Navigation failed
        callback(null, false); // result = false
      }
      callback(null, true); // Success
    }, function(error) {
      // Failure callback
      console.log('GoTo service call failed.');
      console.log(error);
      callback(null, false);
    });
  });

  var goToDock = Meteor.wrapAsync(function(callback) {
    console.log('Going to dock');
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/go_to_dock',
      serviceType : 'code_it/GoToDock'
    });

    var request = new ROSLIB.ServiceRequest({
    });

    client.callService(request, function(result) {
      console.log('Done docking');
      console.log(result);
      if (result.error !== '') { // Docking failed
        callback(null, false); // result = false
      }
      callback(null, true); // Success
    }, function(error) {
      // Failure callback
      console.log('GoToDock service call failed.');
      console.log(error);
      callback(null, false);
    });
  });

  return {
    displayMessage: displayMessage,
    askMultipleChoice: askMultipleChoice,
    goTo: goTo,
    goToDock: goToDock,
  };
}();
