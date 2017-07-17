// Add hooks for each primitive to the interpreter.
function interpreterApi(interpreter, scope) {
  var myRobot = interpreter.createObject(interpreter.OBJECT);
  interpreter.setProperty(scope, 'robot', myRobot);

  var wrapper = function(question, choices, timeout) {
    var question = question ? question.toString() : '';
    var choices_arr = [];
    for (var i=0; i<choices.length; ++i) {
      choices_arr.push(choices.properties[i].toString());
    }
    var timeout = timeout ? timeout.toNumber() : 0;
    return interpreter.createPrimitive(Robot.askMultipleChoice(question, choices_arr, timeout));
  };
  interpreter.setProperty(myRobot, 'askMultipleChoice', interpreter.createNativeFunction(wrapper));

  var wrapper = function(h1text, h2text, timeout) {
    var h1text = h1text ? h1text.toString() : '';
    var h2text = h2text ? h2text.toString() : '';
    var timeout = timeout ? timeout.toNumber() : 0;
    return interpreter.createPrimitive(Robot.displayMessage(h1text, h2text, timeout));
  };
  interpreter.setProperty(myRobot, 'displayMessage', interpreter.createNativeFunction(wrapper));

  var wrapper = function(name, is_tabletop) {
    var name = name ? name.toString() : '';
    var is_tabletop = is_tabletop ? is_tabletop.toBoolean() : false;
    var landmarks = Robot.findCustomLandmark(name, is_tabletop);
    var landmarks_arr = interpreter.toPseudoObject(landmarks);
    return landmarks_arr;
  };
  interpreter.setProperty(myRobot, 'findCustomLandmark', interpreter.createNativeFunction(wrapper));

  var wrapper = function() {
    var objects = Robot.findObjects();
    var objects_arr = interpreter.toPseudoObject(objects);
    return objects_arr;
  };
  interpreter.setProperty(myRobot, 'findObjects', interpreter.createNativeFunction(wrapper));

  var wrapper = function(location) {
    var location = location ? location.toString() : '';
    return interpreter.createPrimitive(Robot.goTo(location));
  };
  interpreter.setProperty(myRobot, 'goTo', interpreter.createNativeFunction(wrapper));

  var wrapper = function() {
    return interpreter.createPrimitive(Robot.goToDock());
  };
  interpreter.setProperty(myRobot, 'goToDock', interpreter.createNativeFunction(wrapper));

  var wrapper = function(gripper) {
    var gripper = gripper ? gripper.toString() : '';
    return interpreter.createPrimitive(Robot.isGripperOpen(gripper));
  };
  interpreter.setProperty(myRobot, 'isGripperOpen', interpreter.createNativeFunction(wrapper));

  var wrapper = function(obj) {
    var obj = interpreter.toNativeObject(obj);
    var x = obj.pose.pose.position.x;
    var y = obj.pose.pose.position.y;
    var z = obj.pose.pose.position.z;
    var frame_id = 'base_footprint';
    return interpreter.createPrimitive(Robot.lookAt(x, y, z, frame_id));
  };
  interpreter.setProperty(myRobot, 'lookAt', interpreter.createNativeFunction(wrapper));

  var wrapper = function(up, left) {
    var up = up ? up.toNumber() : 0;
    var left = left ? left.toNumber() : 0;
    return interpreter.createPrimitive(Robot.lookAtDegrees(up, left));
  };
  interpreter.setProperty(myRobot, 'lookAtDegrees', interpreter.createNativeFunction(wrapper));

  var wrapper = function(obj, arm_id) {
    var obj = obj ? interpreter.toNativeObject(obj) : null;
    var arm_id = arm_id ? arm_id.toNumber() : 0;
    return interpreter.createPrimitive(Robot.pick(obj, arm_id));
  };
  interpreter.setProperty(myRobot, 'pick', interpreter.createNativeFunction(wrapper));

  var wrapper = function(arm_id) {
    var arm_id = arm_id ? arm_id.toNumber() : 0;
    return interpreter.createPrimitive(Robot.place(arm_id));
  };
  interpreter.setProperty(myRobot, 'place', interpreter.createNativeFunction(wrapper));

  var wrapper = function(name, preregisteredLandmarks) {
    var name = name ? name.toString() : '';
    var preregisteredLandmarks = preregisteredLandmarks ? interpreter.toNativeObject(preregisteredLandmarks) : {};
    return interpreter.createPrimitive(Robot.runPbdAction(name, preregisteredLandmarks));
  };
  interpreter.setProperty(myRobot, 'runPbdAction', interpreter.createNativeFunction(wrapper));

  var wrapper = function(text) {
    var text = text ? text.toString() : '';
    return interpreter.createPrimitive(Robot.say(text));
  };
  interpreter.setProperty(myRobot, 'say', interpreter.createNativeFunction(wrapper));

  var wrapper = function(side, action, max_effort) {
    var side = side ? side.toNumber() : 0;
    var arm_id = action ? action.toNumber() : 0;
    var max_effort = max_effort ? max_effort.toNumber() : -1;
    return interpreter.createPrimitive(Robot.setGripper(side, arm_id, max_effort));
  };
  interpreter.setProperty(myRobot, 'setGripper', interpreter.createNativeFunction(wrapper));

  var wrapper = function (height) {
    var height = height ? height.toNumber() : 0;
    return interpreter.createPrimitive(Robot.setTorso(height));
  };
  interpreter.setProperty(myRobot, 'setTorso', interpreter.createNativeFunction(wrapper));

  var wrapper = function(tuck_left, tuck_right) {
    var tuck_left = tuck_left ? tuck_left.toBoolean() : false;
    var tuck_right = tuck_right ? tuck_right.toBoolean() : false;
    return interpreter.createPrimitive(Robot.tuckArms(tuck_left, tuck_right));
  }
  interpreter.setProperty(myRobot, 'tuckArms', interpreter.createNativeFunction(wrapper));

  var wrapper = function(seconds) {
    var seconds = seconds ? seconds.toNumber() : 0;
    return interpreter.createPrimitive(Robot.waitForDuration(seconds));
  }
  interpreter.setProperty(scope, 'waitForDuration', interpreter.createNativeFunction(wrapper));

  // Misc functions.
  var wrapper = function(blockId) {
    var blockId = blockId ? blockId.toString() : "";
    return interpreter.createPrimitive(function() {
      interpreter.blockId = blockId;
    }());
  }
  interpreter.setProperty(scope, 'highlightBlock', interpreter.createNativeFunction(wrapper));

  // getX, getY, getZ
  var wrapper = function(landmark) {
    var landmark = landmark ? interpreter.toNativeObject(landmark) : {};
    return interpreter.createPrimitive(function() {
      if (!landmark || !landmark.pose || !landmark.pose.pose || !landmark.pose.pose.position || !landmark.pose.pose.position.x) {
        return null;
      }
      return landmark.pose.pose.position.x;
    }());
  }
  interpreter.setProperty(scope, 'getLandmarkX', interpreter.createNativeFunction(wrapper));

  var wrapper = function(landmark) {
    var landmark = landmark ? interpreter.toNativeObject(landmark) : {};
    return interpreter.createPrimitive(function() {
      if (!landmark || !landmark.pose || !landmark.pose.pose || !landmark.pose.pose.position || !landmark.pose.pose.position.y) {
        return null;
      }
      return landmark.pose.pose.position.y;
    }());
  }
  interpreter.setProperty(scope, 'getLandmarkY', interpreter.createNativeFunction(wrapper));

  var wrapper = function(landmark) {
    var landmark = landmark ? interpreter.toNativeObject(landmark) : {};
    return interpreter.createPrimitive(function() {
      if (!landmark || !landmark.pose || !landmark.pose.pose || !landmark.pose.pose.position || !landmark.pose.pose.position.z) {
        return null;
      }
      return landmark.pose.pose.position.z;
    }());
  }
  interpreter.setProperty(scope, 'getLandmarkZ', interpreter.createNativeFunction(wrapper));

  // User study aliases - actions, landmarks
  var actions = interpreter.createObject(interpreter.OBJECT);
  interpreter.setProperty(scope, 'ACTIONS_POKE_GLUE_BOTTLE', interpreter.createPrimitive('Poke glue bottle'));
  interpreter.setProperty(scope, 'ACTIONS_PUT_TOMATO_CAN_IN_BROWN_BOX', interpreter.createPrimitive('Put tomato can in brown box'));
  interpreter.setProperty(scope, 'ACTIONS_PUT_TOMATO_CAN_IN_GREEN_SQUARE', interpreter.createPrimitive('Put tomato can in green square'));
  interpreter.setProperty(scope, 'ACTIONS_PUT_LID_ON_CAN_IN_GREEN_SQUARE', interpreter.createPrimitive('Put lid on can in green square'));

  interpreter.setProperty(scope, 'LANDMARKS_GLUE_BOTTLE', interpreter.createPrimitive('Glue bottle'));
  interpreter.setProperty(scope, 'LANDMARKS_TOMATO_CAN', interpreter.createPrimitive('Tomato can'));
  interpreter.setProperty(scope, 'LANDMARKS_KROGER_CAN', interpreter.createPrimitive('Kroger can'));
  interpreter.setProperty(scope, 'LANDMARKS_CAN_LID', interpreter.createPrimitive('Can lid'));
};

Runtime = function() {
  var isRunning = false;
  var isRunningTopic = new ROSLIB.Topic({
    ros: ROS,
    name: 'code_it/is_program_running',
    messageType: 'std_msgs/Bool',
    latch: true
  });
  var errorTopic = new ROSLIB.Topic({
    ros: ROS,
    name: 'code_it/errors',
    messageType: 'std_msgs/String'
  });

  var publishError = function(message) {
    var msg = new ROSLIB.Message({data: message});
    console.log('Publishing error: ' + message);
    errorTopic.publish(msg);
  };

  var onProgramEnd = function() {
    var msg = new ROSLIB.Message({data: false});
    isRunningTopic.publish(msg);

    console.log('Notifying everyone that the program has ended.');
  };

  var runProgram = function(action, program) {
    var interpreter = new Interpreter(program, interpreterApi);
    isRunning = true;
    var msg = new ROSLIB.Message({data: true});
    isRunningTopic.publish(msg);

    function nextStep() {
      if (!isRunning) {
        if (action.currentGoal) {
          // We don't need onProgramEnd here because this should only be reached via stopProgram.
          action.setPreempted();
        }
        return;
      }
      try {
        if (interpreter.step()) {
          var error = Robot.getError();
          if (error) {
            publishError(error);
            Robot.setError('');
          }
          var feedback = {block_id: interpreter.blockId};
          if (action.currentGoal) {
            action.sendFeedback(feedback);
          }
          Meteor.setTimeout(function() {
            nextStep();
          }, 0);
        } else {
          console.log('Program complete.');
          if (action.currentGoal) {
            action.setSucceeded();
          }
          onProgramEnd();
        }
      } catch(e) {
        console.log('Error: ');
        console.log(e);
        e.stack && console.log(e.stack);
        action.setAborted(e.toString()); 
        publishError('There was an error while running the program: ' + e.toString());
        onProgramEnd();
      }
    }
    nextStep();
  }

  var stopProgram = function(action) {
    console.log('Program was stopped by the user.');
    onProgramEnd();
    isRunning = false;
    if (action.currentGoal) {
      action.setPreempted();
    }
  };

  return {
    runProgram: runProgram,
    stopProgram: stopProgram,
  };
}()

Meteor.startup(function() {
  var runAction = new ROSLIB.SimpleActionServer({
    ros: ROS,
    serverName: '/run_program',
    actionName: 'code_it_msgs/RunProgramAction'
  });

  runAction.on('goal', Meteor.bindEnvironment(function(goal) {
    Runtime.runProgram(runAction, goal.program);
  }));

  runAction.on('cancel', Meteor.bindEnvironment(function() {
    Runtime.stopProgram(runAction);
  }));
});
