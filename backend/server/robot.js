// The implementation of the primitives.
Robot = function() {
  var askMultipleChoice = Meteor.wrapAsync(function(question, choices, timeout, callback) {
    console.log('Asking: ' + question + ', choices: ' + choices);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/ask_multiple_choice',
      serviceType : 'code_it_msgs/AskMultipleChoice'
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
    }, function(error) {
      callback(true, null);
    });

    if (timeout > 0) {
      Meteor.setTimeout(function() {
        if (funcCall.isRunning) {
          callback(null, null); // err, result
        }
      }, timeout * 1000);
    }
  });

  var displayMessage = Meteor.wrapAsync(function(h1text, h2text, timeout, callback) {
    console.log('Displaying h1: ' + h1text + ', h2: ' + h2text);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/display_message',
      serviceType : 'code_it_msgs/DisplayMessage'
    });

    var request = new ROSLIB.ServiceRequest({
      h1_text: h1text,
      h2_text: h2text,
      timeout: timeout
    });

    client.callService(request, function(result) {
    }, function(error) {
      callback(true, null);
    });

    if (timeout > 0) {
      Meteor.setTimeout(function() {
        callback(null, null); // err, result
      }, timeout * 1000);
    } else {
      callback(null, null);
    }
  });

  var findObjects = Meteor.wrapAsync(function(callback) {
    console.log('Finding objects');
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/find_objects',
      serviceType : 'code_it_msgs/FindObjects'
    });

    var request = new ROSLIB.ServiceRequest({});
    client.callService(request, function(result) {
      callback(null, result.objects);
    }, function() {
      callback(true, []);
    });
  });

  var goTo = Meteor.wrapAsync(function(location, callback) {
    console.log('Going to: ' + location);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/go_to',
      serviceType : 'code_it_msgs/GoTo'
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
      serviceType : 'code_it_msgs/GoToDock'
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

  var lookAt = Meteor.wrapAsync(function(x, y, z, frame_id, callback) {
    console.log('Looking at: (' + x + ', ' + y + ', ' + z + ', ' + frame_id + ')');
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/look_at',
      serviceType : 'code_it_msgs/LookAt'
    });

    var request = new ROSLIB.ServiceRequest({
      target: {
        point: {
          x: x,
          y: y,
          z: z,
        },
        header: {
          frame_id: frame_id,
        },
      },
    });

    client.callService(request, function(result) {
      callback(null, null);
    }, function(error) {
      callback(null, null);
    });
  });

  var DEGS_TO_RADS = Math.PI / 180;
  var lookAtDegrees = Meteor.wrapAsync(function(up, left, callback) {
    console.log('Looking at: (' + up + ', ' + left  + ') degrees');
    var x = Math.cos(DEGS_TO_RADS * left);
    var y = Math.sin(DEGS_TO_RADS * left);
    var z = Math.sin(DEGS_TO_RADS * up);
    // Empty string for the frame_id defaults to a frame that is located
    // near the head, facing the same direction as the robot.
    lookAt(x, y, z, '', callback);
  });

  var pick = Meteor.wrapAsync(function(obj, arm_id, callback) {
    console.log('Picking object with arm_id ' + arm_id);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/pick',
      serviceType : 'code_it_msgs/Pick'
    });

    var request = new ROSLIB.ServiceRequest({
      object: obj,
      arm: {
        arm_id: arm_id
      }
    });

    client.callService(request, function(result) {
      callback(null, true);
    }, function(error) {
      callback(null, false);
    });
  });

  var place = Meteor.wrapAsync(function(arm_id, callback) {
    console.log('Placing object with arm_id ' + arm_id);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/place',
      serviceType : 'code_it_msgs/Place'
    });

    var request = new ROSLIB.ServiceRequest({
      arm: {
        arm_id: arm_id
      }
    });

    client.callService(request, function(result) {
      callback(null, true);
    }, function(error) {
      callback(null, false);
    });
  });

  var say = Meteor.wrapAsync(function(text, callback) {
    console.log('Saying: ' + text);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/say',
      serviceType : 'code_it_msgs/Say'
    });

    var request = new ROSLIB.ServiceRequest({
      text: text
    });

    // Some implementations may not know when the sound has finished.
    // Keep in mind that this block may not be synchronous.
    client.callService(request, function(result) {
      callback(null, null);
    }, function(error) {
      callback(null, null);
    });
  });

  var tuckArms = Meteor.wrapAsync(function(tuck_left, tuck_right, callback) {
    console.log('Setting arms, tuck left: ' + tuck_left + ', tuck_right: ' + tuck_right);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/tuck_arms',
      serviceType : 'code_it_msgs/TuckArms'
    });

    var request = new ROSLIB.ServiceRequest({
      tuck_left: tuck_left,
      tuck_right: tuck_right
    });

    client.callService(request, function(result) {
      callback(null, null);
    }, function(error) {
      console.log(error);
      callback(null, null);
    });
  });

  return {
    askMultipleChoice: askMultipleChoice,
    displayMessage: displayMessage,
    findObjects: findObjects,
    goTo: goTo,
    goToDock: goToDock,
    lookAt: lookAt,
    lookAtDegrees: lookAtDegrees,
    pick: pick,
    place: place,
    say: say,
    tuckArms: tuckArms,
  };
}();
