// The implementation of the primitives.
Robot = function() {
  var error = ''; // Most recent error message, empty string for no error.

  var getError = function() {
    return error;
  }

  var setError = function(message) {
    error = message;
  }

  var askMultipleChoice = Meteor.wrapAsync(function(question, choices, timeout, callback) {
    console.log('Asking: ' + question + ', choices: ' + choices);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/ask_multiple_choice',
      serviceType: 'code_it_msgs/AskMultipleChoice'
    });

    var request = new ROSLIB.ServiceRequest({
      question: question,
      choices: choices,
    });

    var funcCall = this;
    funcCall.isRunning = true;
    client.callService(request, function(result) {
      if (funcCall.isRunning) {
        funcCall.isRunning = false;
        setError(result.error);
        if (result.error) {
          callback(result.error, null); // Must quit program, need user input to proceed.
        } else {
          console.log('Choice was: ' + result.choice);
          callback(null, result.choice);
        }
      }
    }, function(error) {
      if (funcCall.isRunning) {
        funcCall.isRunning = false;
        callback(error ? error : 'Failed to ask multiple choice question.', null);
      }
    });

    if (timeout > 0) {
      Meteor.setTimeout(function() {
        if (funcCall.isRunning) {
          funcCall.isRunning = false;
          callback(null, null); // err, result
        }
      }, timeout * 1000);
    }
  });

  var displayMessage = Meteor.wrapAsync(function(h1text, h2text, timeout, callback) {
    console.log('Displaying h1: ' + h1text + ', h2: ' + h2text);
    if (timeout > 0) {
      setError('Warning: use "wait for seconds" instead of timeout field.');
      console.log('WARNING: displayMessage\'s timeout field is no longer used. Use waitForDuration instead.');
    }
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/display_message',
      serviceType: 'code_it_msgs/DisplayMessage'
    });

    var request = new ROSLIB.ServiceRequest({
      h1_text: h1text,
      h2_text: h2text,
    });

    var funcCall = this;
    funcCall.isRunning = true;

    client.callService(request, function(result) {
        funcCall.isRunning = false;
        setError(result.error); // Failure is probaby not critical enough to quit over.
        callback(null, null);
    }, function(error) {
      if (funcCall.isRunning) {
        funcCall.isRunning = false;
        callback(error ? error : 'Failed to display message.', null);
      }
    });

  });

  var findCustomLandmark = Meteor.wrapAsync(function(db_id, is_tabletop, callback) {
    console.log('Finding custom landmark ' + db_id + ', on tabletop: ' + is_tabletop);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/find_custom_landmark',
      serviceType: 'code_it_msgs/FindCustomLandmarks'
    });

    var request = new ROSLIB.ServiceRequest({
      db_id: db_id,
      is_tabletop: is_tabletop
    });
    console.log(request);
    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(null, []); // Return empty list on error.
      } else {
        callback(null, result.landmarks);
      }
    }, function() {
      callback(error ? error : 'Failed to search for custom landmark.', []);
    });
  });

  var findObjects = Meteor.wrapAsync(function(callback) {
    console.log('Finding objects');
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/find_objects',
      serviceType: 'code_it_msgs/FindObjects'
    });

    var request = new ROSLIB.ServiceRequest({});
    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(null, []); // Return empty list on error.
      } else {
        callback(null, result.objects);
      }
    }, function() {
      callback(error ? error : 'Failed to look for objects.', []);
    });
  });

  var goTo = Meteor.wrapAsync(function(location, callback) {
    console.log('Going to: ' + location);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/go_to',
      serviceType: 'code_it_msgs/GoTo'
    });

    var request = new ROSLIB.ServiceRequest({
      location: location
    });

    client.callService(request, function(result) {
      console.log('Done navigating to ' + location + ', result:');
      console.log(result);
      setError(result.error);
      if (result.error !== '') { // Navigation failed
        callback(null, false); // result = false
      }
      callback(null, true); // Success
    }, function(error) {
      callback(error ? error : 'Failed to run navigation.', null);
    });
  });

  var goToDock = Meteor.wrapAsync(function(callback) {
    console.log('Going to dock');
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/go_to_dock',
      serviceType: 'code_it_msgs/GoToDock'
    });

    var request = new ROSLIB.ServiceRequest({
    });

    client.callService(request, function(result) {
      console.log('Done docking');
      console.log(result);
      setError(result.error);
      if (result.error !== '') { // Docking failed
        callback(null, false); // result = false
      }
      callback(null, true); // Success
    }, function(error) {
      callback(error ? error : 'Failed to run navigation to dock.', null);
    });
  });

  var isGripperOpen = Meteor.wrapAsync(function(gripper, callback) {
    console.log('Checking ' + gripper + ' gripper state.');
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/is_gripper_open',
      serviceType: 'code_it_msgs/IsGripperOpen'
    });

    var gripper_id = 0;
    if (gripper === 'LEFT') {
      gripper_id = 1;
    } else if (gripper === 'RIGHT') {
      gripper_id = 2;
    }

    var request = new ROSLIB.ServiceRequest({
      gripper: {
        id: gripper_id
      }
    });

    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(result.error, null); // No recovery if error with gripper service.
      } else {
        callback(null, result.is_open);
      }
    }, function(error) {
      callback(error ? error : 'Failed to check gripper state.', null);
    });
  });

  var lookAt = Meteor.wrapAsync(function(x, y, z, frame_id, callback) {
    console.log('Looking at: (' + x + ', ' + y + ', ' + z + ', ' + frame_id + ')');
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/look_at',
      serviceType: 'code_it_msgs/LookAt'
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
      setError(result.error);
      if (result.error) {
        callback(result.error, null); // No recovery.
      }
      callback(null, null);
    }, function(error) {
      callback(error ? error : 'Failed to look at target.', null);
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
      serviceType: 'code_it_msgs/Pick'
    });

    var request = new ROSLIB.ServiceRequest({
      object: obj,
      arm: {
        arm_id: arm_id
      }
    });

    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(null, false);
      } else {
        callback(null, true);
      }
    }, function(error) {
      callback(error ? error : 'Failed to run object picking.', false);
    });
  });

  var place = Meteor.wrapAsync(function(arm_id, callback) {
    console.log('Placing object with arm_id ' + arm_id);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/place',
      serviceType: 'code_it_msgs/Place'
    });

    var request = new ROSLIB.ServiceRequest({
      arm: {
        arm_id: arm_id
      }
    });

    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(null, false);
      } else {
        callback(null, true);
      }
    }, function(error) {
      callback(error ? error : 'Failed to run object placing.', false);
    });
  });

  var runPbdAction = Meteor.wrapAsync(function(actionId, preregisteredLandmarks, callback) {
    console.log('Running PbD action: ' + actionId);
    console.log('Preregistered landmarks', preregisteredLandmarks);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/run_pbd_action',
      serviceType: 'code_it_msgs/RunPbdAction'
    });

    var preregistered = [];
    for (var i=0; i<preregisteredLandmarks.length; i++) {
      if (preregisteredLandmarks[i]) {
        preregistered.push(preregisteredLandmarks[i]);
      }
    }

    var request = new ROSLIB.ServiceRequest({
      action_id: actionId,
      landmarks: preregistered
    });

    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(null, false); // There was an error, return false.
      } else {
        callback(null, true); // Return success = true.
      }
    }, function(error) {
      callback(error ? error : 'PbD action failed to run.', null);
    });
  });

  var say = Meteor.wrapAsync(function(text, callback) {
    console.log('Saying: ' + text);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/say',
      serviceType: 'code_it_msgs/Say'
    });

    var request = new ROSLIB.ServiceRequest({
      text: text
    });

    // Some implementations may not know when the sound has finished.
    // Keep in mind that this block may not be synchronous.
    client.callService(request, function(result) {
      setError(result.error); // Sound not playing is probably not fatal enough to quit the program over.
      callback(null, null);
    }, function(error) {
      callback(error ? error : 'Speech failed to run.', null);
    });
  });

  var setGripper = Meteor.wrapAsync(function(side, action, max_effort, callback) {
    console.log('Setting gripper, side: ' + side + ', ' + action);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/set_gripper',
      serviceType: 'code_it_msgs/SetGripper'
    });

    var request = new ROSLIB.ServiceRequest({
      gripper: {
        id: side
      },
      action: action,
      max_effort: max_effort
    });

    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(result.error, null); // No recovery.
      } else {
        callback(null, null);
      }
    }, function(error) {
      callback(error ? error : 'Failed to set gripper.', null);
    });
  });

  var tuckArms = Meteor.wrapAsync(function(tuck_left, tuck_right, callback) {
    console.log('Setting arms, tuck left: ' + tuck_left + ', tuck_right: ' + tuck_right);
    var client = new ROSLIB.Service({
      ros: ROS,
      name: '/code_it/api/tuck_arms',
      serviceType: 'code_it_msgs/TuckArms'
    });

    var request = new ROSLIB.ServiceRequest({
      tuck_left: tuck_left,
      tuck_right: tuck_right
    });

    client.callService(request, function(result) {
      setError(result.error);
      if (result.error) {
        callback(result.error, null);
      } else {
        callback(null, null);
      }
    }, function(error) {
      callback(error ? error : 'Failed to tuck arms.', null);
    });
  });

  var waitForDuration = Meteor.wrapAsync(function(seconds, callback) {
    if (seconds <= 0) {
      return;
    }
    Meteor.setTimeout(function() {
      callback(null, null); // err, result
    }, seconds * 1000);
  });

  return {
    askMultipleChoice: askMultipleChoice,
    displayMessage: displayMessage,
    findCustomLandmark: findCustomLandmark,
    findObjects: findObjects,
    getError: getError,
    goTo: goTo,
    goToDock: goToDock,
    isGripperOpen: isGripperOpen,
    lookAt: lookAt,
    lookAtDegrees: lookAtDegrees,
    pick: pick,
    place: place,
    runPbdAction: runPbdAction,
    say: say,
    setError: setError,
    setGripper: setGripper,
    tuckArms: tuckArms,
    waitForDuration: waitForDuration,
  };
}();
