ROSLIB = Meteor.npmRequire('roslib');
ROS = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ROS.on('connection', function() {
  console.log('Connected to websocket server.');
});
ROS.on('error', function() {
  console.log('Error connecting to websocket server.');
});
ROS.on('close', function() {
  console.log('Closed connection to websocket server.');
});

ROSLIB.SimpleActionServer.prototype.setAborted = function(text) {
  var resultMessage = new ROSLIB.Message({
    status: {
      goal_id: this.currentGoal.goal_id,
      status: 4,
      text: text,
    },
  });
  this.resultPublisher.publish(resultMessage);
  if (this.nextGoal) {
    this.currentGoal = this.nextGoal;
    this.nextGoal = null;
    this.emit('goal', this.currentGoal.goal);
  } else {
    this.currentGoal = null;
  }
}
