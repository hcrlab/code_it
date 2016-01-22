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
