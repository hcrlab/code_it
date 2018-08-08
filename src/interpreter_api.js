'use strict';

// Add hooks for each primitive to the interpreter.
function interpreterApi(interpreter, scope, robot) {
  const robotObj = interpreter.createObjectProto(interpreter.OBJECT_PROTO);
  interpreter.setProperty(scope, 'robot', robotObj);

  let wrapper = function(name, is_tabletop, callback) {
    function nativeToPseudoCallback(result) {
      callback(interpreter.nativeToPseudo(result));
    }
    robot.findCustomLandmark(name, is_tabletop, nativeToPseudoCallback);
  };
  interpreter.setProperty(
      robotObj, 'findCustomLandmark', interpreter.createAsyncFunction(wrapper));

  wrapper = function(callback) {
    robot.goToDock(callback);
  };
  interpreter.setProperty(
      robotObj, 'goToDock', interpreter.createAsyncFunction(wrapper));

  wrapper = function(callback) {
    robot.isGripperOpen('LEFT', callback);
  };
  interpreter.setProperty(
      robotObj, 'isLeftGripperOpen', interpreter.createAsyncFunction(wrapper));

  wrapper = function(callback) {
    robot.isGripperOpen('RIGHT', callback);
  };
  interpreter.setProperty(
      robotObj, 'isRightGripperOpen', interpreter.createAsyncFunction(wrapper));

  wrapper = function(callback) {
    robot.isGripperOpen('', callback);
  };
  interpreter.setProperty(
      robotObj, 'isGripperOpen', interpreter.createAsyncFunction(wrapper));

  wrapper = function(up, left, callback) {
    robot.lookAtDegrees(up, left, callback);
  };
  interpreter.setProperty(
      robotObj, 'lookAtDegrees', interpreter.createAsyncFunction(wrapper));

  wrapper = function(name, preregisteredLandmarks, callback) {
    var preregisteredLandmarks = preregisteredLandmarks ?
        interpreter.pseudoToNative(preregisteredLandmarks) :
        {};
    robot.runPbdAction(name, preregisteredLandmarks, callback);
  };
  interpreter.setProperty(
      robotObj, 'runPbdAction', interpreter.createAsyncFunction(wrapper));

  wrapper = function(text, callback) {
    robot.say(text, callback);
  };
  interpreter.setProperty(
      robotObj, 'say', interpreter.createAsyncFunction(wrapper));

  wrapper = function(tuck_left, tuck_right, callback) {
    robot.tuckArms(tuck_left, tuck_right, callback);
  };
  interpreter.setProperty(
      robotObj, 'tuckArms', interpreter.createAsyncFunction(wrapper));

  wrapper = function(question, choices) {
    const choicesArr = interpreter.pseudoToNative(choices);
    robot.startAskMultipleChoice(question, choicesArr);
  };
  interpreter.setProperty(
      robotObj, 'startAskMultipleChoice',
      interpreter.createNativeFunction(wrapper));

  wrapper = function(h1text, h2text) {
    robot.startDisplayMessage(h1text, h2text);
  };
  interpreter.setProperty(
      robotObj, 'startDisplayMessage',
      interpreter.createNativeFunction(wrapper));

  wrapper = function(location) {
    robot.startGoTo(location);
  };
  interpreter.setProperty(
      robotObj, 'startGoTo', interpreter.createNativeFunction(wrapper));

  wrapper = function(pan, tilt) {
    robot.startHead(pan, tilt);
  };
  interpreter.setProperty(
      robotObj, 'startHead', interpreter.createNativeFunction(wrapper));

  wrapper = function(program) {
    robot.startRapidPbD(program);
  };
  interpreter.setProperty(
      robotObj, 'startRapidPbD', interpreter.createNativeFunction(wrapper));

  wrapper = function(force) {
    robot.startCloseGripper(force);
  };
  interpreter.setProperty(
      robotObj, 'startCloseGripper', interpreter.createNativeFunction(wrapper));

  wrapper = function() {
    robot.startOpenGripper();
  };
  interpreter.setProperty(
      robotObj, 'startOpenGripper', interpreter.createNativeFunction(wrapper));

  wrapper = function(height) {
    robot.startTorso(height);
  };
  interpreter.setProperty(
      robotObj, 'startTorso', interpreter.createNativeFunction(wrapper));

  wrapper = function(seconds) {
    robot.startTimer(seconds);
  };
  interpreter.setProperty(
      scope, 'startTimer', interpreter.createNativeFunction(wrapper));

  wrapper = function(resource) {
    return robot.isDone(resource);
  };
  interpreter.setProperty(
      robotObj, 'isDone', interpreter.createNativeFunction(wrapper));

  wrapper = function(resource) {
    return robot.getResult(resource);
  };
  interpreter.setProperty(
      robotObj, 'getResult', interpreter.createNativeFunction(wrapper));

  wrapper = function(resource) {
    robot.cancel(resource);
  };
  interpreter.setProperty(
      robotObj, 'cancel', interpreter.createNativeFunction(wrapper));

  wrapper = function(question, choices, callback) {
    const choicesArr = interpreter.pseudoToNative(choices);
    robot.askMultipleChoice(question, choicesArr, callback);
  };
  interpreter.setProperty(
      robotObj, 'askMultipleChoice', interpreter.createAsyncFunction(wrapper));

  wrapper = function(h1text, h2text, callback) {
    robot.displayMessage(h1text, h2text, callback);
  };
  interpreter.setProperty(
      robotObj, 'displayMessage', interpreter.createAsyncFunction(wrapper));

  wrapper = function(location, callback) {
    robot.goTo(location, callback);
  };
  interpreter.setProperty(
      robotObj, 'goTo', interpreter.createAsyncFunction(wrapper));

  wrapper = function(name, callback) {
    robot.runRapidPbdProgram(name, callback);
  };
  interpreter.setProperty(
      robotObj, 'runRapidPbdProgram', interpreter.createAsyncFunction(wrapper));

  wrapper = function(callback) {
    var side = 0;    // For Fetch only
    var action = 1;  // Open
    var max_effort = 0;
    robot.setGripper(side, action, max_effort, callback);
  };
  interpreter.setProperty(
      robotObj, 'openGripper', interpreter.createAsyncFunction(wrapper));

  wrapper = function(max_effort, callback) {
    var side = 0;    // For Fetch only
    var action = 2;  // Close
    var max_effort = max_effort;
    robot.setGripper(side, action, max_effort, callback);
  };
  interpreter.setProperty(
      robotObj, 'closeGripper', interpreter.createAsyncFunction(wrapper));

  wrapper = function(callback) {
    var side = 1;    // Left
    var action = 1;  // Open
    var max_effort = -1;
    robot.setGripper(side, action, max_effort, callback);
  };
  interpreter.setProperty(
      robotObj, 'openLeftGripper', interpreter.createAsyncFunction(wrapper));

  wrapper = function(max_effort, callback) {
    var side = 1;    // Left
    var action = 2;  // Close
    var max_effort = max_effort ? max_effort : -1;
    robot.setGripper(side, action, max_effort, callback);
  };
  interpreter.setProperty(
      robotObj, 'closeLeftGripper', interpreter.createAsyncFunction(wrapper));

  wrapper = function(callback) {
    var side = 2;    // Right
    var action = 1;  // Open
    var max_effort = -1;
    robot.setGripper(side, action, max_effort, callback);
  };
  interpreter.setProperty(
      robotObj, 'openRightGripper', interpreter.createAsyncFunction(wrapper));

  wrapper = function(max_effort, callback) {
    var side = 2;    // Right
    var action = 2;  // Close
    var max_effort = max_effort ? max_effort : -1;
    robot.setGripper(side, action, max_effort, callback);
  };
  interpreter.setProperty(
      robotObj, 'closeRightGripper', interpreter.createAsyncFunction(wrapper));

  wrapper = function(height, callback) {
    robot.setTorso(height, callback);
  };
  interpreter.setProperty(
      robotObj, 'setTorso', interpreter.createAsyncFunction(wrapper));

  wrapper = function(seconds, callback) {
    robot.waitForDuration(seconds, callback);
  };
  interpreter.setProperty(
      scope, 'waitForDuration', interpreter.createAsyncFunction(wrapper));

  // Misc functions.
  wrapper = function(blockId) {
    interpreter.blockId = blockId;
  };
  interpreter.setProperty(
      scope, 'highlightBlock', interpreter.createNativeFunction(wrapper));

  // getX, getY, getZ
  wrapper = function(landmark) {
    var landmark = landmark ? interpreter.pseudoToNative(landmark) : {};
    if (!landmark || !landmark.pose || !landmark.pose.pose ||
        !landmark.pose.pose.position || !landmark.pose.pose.position.x) {
      return null;
    }
    return landmark.pose.pose.position.x;
  };
  interpreter.setProperty(
      scope, 'getLandmarkX', interpreter.createNativeFunction(wrapper));

  wrapper = function(landmark) {
    var landmark = landmark ? interpreter.pseudoToNative(landmark) : {};
    if (!landmark || !landmark.pose || !landmark.pose.pose ||
        !landmark.pose.pose.position || !landmark.pose.pose.position.y) {
      return null;
    }
    return landmark.pose.pose.position.y;
  };
  interpreter.setProperty(
      scope, 'getLandmarkY', interpreter.createNativeFunction(wrapper));

  wrapper = function(landmark) {
    var landmark = landmark ? interpreter.pseudoToNative(landmark) : {};
    if (!landmark || !landmark.pose || !landmark.pose.pose ||
        !landmark.pose.pose.position || !landmark.pose.pose.position.z) {
      return null;
    }
    return landmark.pose.pose.position.z;
  };
  interpreter.setProperty(
      scope, 'getLandmarkZ', interpreter.createNativeFunction(wrapper));
	
  wrapper = function(callback){
      robot.slipGripper(callback);
  };
  interpreter.setProperty(
      robotObj, 'slipGripper', interpreter.createAsyncFunction(wrapper));
	
}

module.exports = interpreterApi;
