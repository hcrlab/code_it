#!/usr/bin/env node

'use strict';

const rosnodejs = require('rosnodejs');
const Runtime = require('../src/runtime.js');
const Robot = require('../src/robot.js');

if (require.main === module) {
  rosnodejs.initNode('code_it').then((nh) => {
    const isRunningPub = nh.advertise(
        'code_it/is_program_running', 'std_msgs/Bool', {latching: true, tcpNoDelay: true});
    const errorPub = nh.advertise('code_it/errors', 'std_msgs/String');

    const programServer = new rosnodejs.ActionServer(
        {nh, type: 'code_it_msgs/RunProgram', actionServer: 'run_program'});
    const robot = new Robot(nh);
    const runtime = new Runtime(robot, isRunningPub, errorPub);
    programServer.on('goal', (goal) => {
      runtime.execute(goal);
    });
    programServer.on('cancel', () => {
      runtime.stop();
    });
    rosnodejs.on('shutdown', () => {
      isRunningPub.publish({data: false});
      runtime.shutdown();
      programServer.shutdown();
    });
  });
}
