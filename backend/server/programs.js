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

  var onProgramEnd = function() {
    var topic = new ROSLIB.Topic({
      ros: ROS,
      name: '/code_it/stopped',
      messageType: 'std_msgs/Empty'
    });
    var msg = new ROSLIB.Message({});
    topic.publish(msg);
  };

  var runProgram = function(action, program) {
    var interpreter = new Interpreter(program, interpreterApi);
    isRunning = true;
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
          action.sendFeedback(feedback);
          Meteor.setTimeout(function() {
            nextStep(stepNum + 1);
          }, 0);
        } else {
          action.setSucceeded();
          onProgramEnd();
        }
      } catch(e) {
        console.log('Error running program ' + program);
        console.log(e.stack);
        action.setError(e.toString()); 
        onProgramEnd();
      }
    }
    nextStep(0);
  }

  var stopProgram = function(action) {
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
