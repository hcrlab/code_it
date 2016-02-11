// Add hooks for each primitive to the interpreter.
function interpreterApi(interpreter, scope) {
  var myRobot = interpreter.createObject(interpreter.OBJECT);
  interpreter.setProperty(scope, 'robot', myRobot);

  var wrapper = function(h1text, h2text, timeout) {
    var h1text = h1text ? h1text.toString() : '';
    var h2text = h2text ? h2text.toString() : '';
    var timeout = timeout ? timeout.toNumber() : 0;
    return interpreter.createPrimitive(Robot.displayMessage(h1text, h2text, timeout));
  };
  interpreter.setProperty(myRobot, 'displayMessage', interpreter.createNativeFunction(wrapper));

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

  var wrapper = function(location) {
    var location = location ? location.toString() : '';
    return interpreter.createPrimitive(Robot.goTo(location));
  };
  interpreter.setProperty(myRobot, 'goTo', interpreter.createNativeFunction(wrapper));

  var wrapper = function() {
    return interpreter.createPrimitive(Robot.goToDock());
  };
  interpreter.setProperty(myRobot, 'goToDock', interpreter.createNativeFunction(wrapper));

  var wrapper = function(text) {
    var text = text ? text.toString() : '';
    return interpreter.createPrimitive(Robot.say(text));
  };
  interpreter.setProperty(myRobot, 'say', interpreter.createNativeFunction(wrapper));
};

Runtime = function() {
  var isRunning = false;
  var isRunningTopic = new ROSLIB.Topic({
    ros: ROS,
    name: 'code_it/is_program_running',
    messageType: 'std_msgs/Bool',
    latch: true
  });

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

    function nextStep(stepNum) {
      if (!isRunning) {
        if (action.currentGoal) {
          // We don't need onProgramEnd here because this should only be reached via stopProgram.
          action.setPreempted();
        }
        return;
      }
      try {
        if (interpreter.step()) {
          // TODO(jstn): Report block number instead of step number
          var feedback = {block_id: stepNum};
          if (action.currentGoal) {
            action.sendFeedback(feedback);
          }
          Meteor.setTimeout(function() {
            nextStep(stepNum + 1);
          }, 0);
        } else {
          console.log('Program complete.');
          if (action.currentGoal) {
            action.setSucceeded();
          }
          onProgramEnd();
        }
      } catch(e) {
        console.log('Error running program ' + program);
        console.log(e.stack);
        action.setAborted(e.toString()); 
        onProgramEnd();
      }
    }
    nextStep(0);
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
    actionName: 'code_it/RunProgramAction'
  });

  runAction.on('goal', Meteor.bindEnvironment(function(goal) {
    Runtime.runProgram(runAction, goal.program);
  }));

  runAction.on('cancel', Meteor.bindEnvironment(function() {
    Runtime.stopProgram(runAction);
  }));
});
