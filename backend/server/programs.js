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

Meteor.startup(function() {
  Meteor.methods({
    runProgram: function(program) {
      if (program === undefined || program.length <= 0) {
        throw new Meteor.Error(400, 'No program given.');
      }
      var interpreter = new Interpreter(program, interpreterApi);
      try {
        interpreter.run();
        Meteor.call('endProgram');
      } catch(e) {
        throw new Meteor.Error(400, 'Program was invalid. ' + e);
      }
    },
    endProgram: function() {
    }
  });

  var listener = new ROSLIB.Topic({
    ros: ROS,
    name: '/code_it/program',
    messageType: 'std_msgs/String'
  });
  
  listener.subscribe(Meteor.bindEnvironment(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
    Meteor.call('runProgram', message.data);
  }));
});
