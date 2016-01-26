// Add hooks for each primitive to the interpreter.
function interpreterApi(interpreter, scope) {
  var myRobot = interpreter.createObject(interpreter.OBJECT);
  interpreter.setProperty(scope, 'robot', myRobot);

  var wrapper = function(h1text, h2text, timeout) {
    h1text = h1text ? h1text.toString() : '';
    h2text = h2text ? h2text.toString() : '';
    timeout = timeout ? timeout.toNumber() : 0;
    return interpreter.createPrimitive(Robot.displayMessage(h1text, h2text, timeout));
  };
  interpreter.setProperty(myRobot, 'displayMessage', interpreter.createNativeFunction(wrapper));
};

Runtime = function() {
  var isRunning = false;

  var runProgram = function(action, program) {
    var interpreter = new Interpreter(program, interpreterApi);
    isRunning = true;
    function nextStep(stepNum) {
      if (!isRunning) {
        if (action.currentGoal) {
          action.setPreempted();
        }
        return;
      }
      try {
        if (interpreter.step()) {
          // TODO(jstn): Report block number instead of step number
          var feedback = {block_id: stepNum};
          action.sendFeedback(feedback);
          Meteor.setTimeout(function() {
            nextStep(stepNum + 1);
          }, 0);
        } else {
          action.setSucceeded();
        }
      } catch(e) {
        console.log('Error running program ' + program);
        console.log(e.stack);
        if (action.currentGoal) {
          action.setAborted(e); 
        }
      }
    }
    nextStep(0);
  }

  var stopProgram = function(action) {
    isRunning = false;
    if (action.currentGoal) {
      action.setPreempted();
    }
  }

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
