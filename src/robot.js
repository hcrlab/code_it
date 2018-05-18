'use strict';

const rosnodejs = require('rosnodejs');
const actionlib_msgs = rosnodejs.require('actionlib_msgs');
const code_it_msgs = rosnodejs.require('code_it_msgs');

// The implementation of the primitives.
class Robot {
  constructor(nh) {
    this._nh = nh;
    this.error = '';  // Most recent error message, empty string for no error.

    this.torsoClient = this._nh.actionClientInterface(
        '/code_it/api/set_torso', 'code_it_msgs/SetTorso');
    this.torsoClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.torsoStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.torsoStatus = msg.status_list[msg.status_list.length - 1].status;
      }
      if (msg.status_list.length > 1) {
        rosnodejs.log.warn(
            'There were ' + msg.status_list.length +
            ' goals in the torso status_list.');
      }
    });

    this.headClient = this._nh.actionClientInterface(
        '/code_it/api/move_head', 'code_it_msgs/MoveHead');
    this.headClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.headStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.headStatus = msg.status_list[msg.status_list.length - 1].status;
      }
      if (msg.status_list.length > 1) {
        rosnodejs.log.warn(
            'There were ' + msg.status_list.length +
            ' goals in the head status_list.');
      }
    });
  }

  askMultipleChoice(question, choices, callback) {
    rosnodejs.log.info('Asking: ' + question + ', choices: ' + choices);
    const service_name = '/code_it/api/ask_multiple_choice';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/AskMultipleChoice');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.AskMultipleChoice.Request({
          question: question,
          choices: choices,
        });
        client.call(request).then((response) => {
          this.error = response.error;
          callback(response.choice);
        });
      } else {
        this.error = 'AskMultipleChoice service not available!';
        callback('');
      }
    });
  }

  displayMessage(h1text, h2text, callback) {
    rosnodejs.log.info('Displaying h1: ' + h1text + ', h2: ' + h2text);
    const service_name = '/code_it/api/display_message';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/DisplayMessage');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.DisplayMessage.Request(
            {h1_text: h1text, h2_text: h2text});
        client.call(request).then((response) => {
          this.error = response.error;
          callback();
        });
      } else {
        this.error = 'DisplayMessage service not available!';
        callback();
      }
    });
  }

  findCustomLandmark(name, is_tabletop, callback) {
    rosnodejs.log.info(
        'Finding custom landmark ' + name + ', on tabletop: ' + is_tabletop);
    const service_name = '/code_it/api/find_custom_landmark';
    const client = this._nh.serviceClient(
        service_name, 'code_it_msgs/FindCustomLandmarks');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.FindCustomLandmarks.Request(
            {name: name, is_tabletop: is_tabletop});
        client.call(request).then((response) => {
          this.error = response.error;
          callback(response.landmarks);
        });
      } else {
        this.error = 'FindCustomLandmarks service not available!';
        callback([]);
      }
    });
  }

  goTo(location, callback) {
    rosnodejs.log.info('Going to: ' + location);
    const service_name = '/code_it/api/go_to';
    const client = this._nh.serviceClient(service_name, 'code_it_msgs/GoTo');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.GoTo.Request({location: location});
        client.call(request).then((response) => {
          this.error = response.error;
          callback(response.error === '');
        });
      } else {
        this.error = 'GoTo service not available!';
        callback(false);
      }
    });
  }

  goToDock(callback) {
    rosnodejs.log.info('Going to dock');
    const service_name = '/code_it/api/go_to_dock';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/GoToDock');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request =
            new code_it_msgs.srv.GoToDock.Request({location: location});
        client.call(request).then((response) => {
          this.error = response.error;
          callback(response.error === '');
        });
      } else {
        this.error = 'GoToDock service not available!';
        callback(false);
      }
    });
  }

  isGripperOpen(gripper, callback) {
    rosnodejs.log.info('Checking ' + gripper + ' gripper state.');
    const service_name = '/code_it/api/is_gripper_open';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/IsGripperOpen');

    let gripper_id = 0;
    if (gripper === 'LEFT') {
      gripper_id = 1;
    } else if (gripper === 'RIGHT') {
      gripper_id = 2;
    }

    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.IsGripperOpen.Request(
            {gripper: {id: gripper_id}});
        client.call(request).then((response) => {
          this.error = response.error;
          callback(response.is_open);
        });
      } else {
        this.error = 'IsGripperOpen service not available!';
        callback(false);
      }
    });
  }

  lookAt(x, y, z, frame_id, callback) {
    rosnodejs.log.info(
        'Looking at: (' + x + ', ' + y + ', ' + z + ', ' + frame_id + ')');
    const service_name = '/code_it/api/look_at';
    const client = this._nh.serviceClient(service_name, 'code_it_msgs/LookAt');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.LookAt.Request({
          target: {
            point: {
              x: x,
              y: y,
              z: z,
            },
            header: {
              frame_id: frame_id,
            }
          }
        });
        client.call(request).then((response) => {
          this.error = response.error;
          callback();
        });
      } else {
        this.error = 'LookAt service not available!';
        callback();
      }
    });
  }

  lookAtDegrees(up, left, callback) {
    rosnodejs.log.info('Looking at: (' + up + ', ' + left + ') degrees');
    var DEGS_TO_RADS = Math.PI / 180;
    var x = Math.cos(DEGS_TO_RADS * left);
    var y = Math.sin(DEGS_TO_RADS * left);
    var z = Math.sin(DEGS_TO_RADS * up);
    // Empty string for the frame_id defaults to a frame that is located
    // near the head, facing the same direction as the robot.
    this.lookAt(x, y, z, '', callback);
  }

  runPbdAction(name, preregisteredLandmarks, callback) {
    rosnodejs.log.info('Running PbD action: ' + name);
    const service_name = '/code_it/api/run_pbd_action';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/RunPbdAction');

    // preregisteredLandmarks is an object like this:
    // {'id1234': landmark1, 'id2345': landmark2}
    // We transform it into [landmark1, landmark2], where landmark1 and
    // landmark2 have their IDs replaced with 'id1234' and 'id2345'.
    var preregistered = [];
    for (var id in preregisteredLandmarks) {
      var landmark = preregisteredLandmarks[id];
      if (landmark && landmark.name) {
        preregistered.push(landmark);
      }
    }
    rosnodejs.log.info('Pre-registered landmarks', preregistered);

    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.RunPbdAction.Request(
            {action_id: '', name: name, landmarks: preregistered});
        client.call(request).then((response) => {
          if (response.error !== '') {
            rosnodejs.log.info(response.error);
          }
          callback(response.error === '');
        });
      } else {
        this.error = 'RunPbdAction service not available!';
        callback(false);
      }
    });
  }

  runRapidPbdProgram(name, callback) {
    rosnodejs.log.info('Running Rapid PbD program: ' + name);
    const service_name = '/code_it/api/run_pbd_action';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/RunPbdAction');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.RunPbdAction.Request(
            {action_id: '', name: name, landmarks: []});
        client.call(request).then((response) => {
          if (response.error !== '') {
            rosnodejs.log.info(response.error);
          }
          callback(response.error === '');
        });
      } else {
        this.error = 'RunPbdAction service not available!';
        callback(false);
      }
    });
  }

  say(text, callback) {
    rosnodejs.log.info('Saying: ' + text);
    const service_name = '/code_it/api/say';
    const client = this._nh.serviceClient(service_name, 'code_it_msgs/Say');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.Say.Request({text: text});
        client.call(request).then((response) => {
          this.error = response.error;
          callback();
        });
      } else {
        this.error = 'Say service not available!';
        callback();
      }
    });
  }

  setGripper(side, action, max_effort, callback) {
    rosnodejs.log.info(
        'Setting gripper, side: ' + side + ', action: ' + action +
        ', effort: ' + max_effort);
    const service_name = '/code_it/api/set_gripper';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/SetGripper');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.SetGripper.Request(
            {gripper: {id: side}, action: action, max_effort: max_effort});
        client.call(request).then((response) => {
          this.error = response.error;
          callback();
        });
      } else {
        this.error = 'SetGripper service not available!';
        callback();
      }
    });
  }

  startTorso(height) {
    rosnodejs.log.info('Starting to set torso to ' + height + ' meters');
    this.torsoClient.sendGoal({goal: {height: height}});
  }

  startHead(pan, tilt) {
    rosnodejs.log.info(
        'Starting to move head to ' + pan + ', ' + tilt + ' degrees');
    this.headClient.sendGoal({goal: {pan_degrees: pan, tilt_degrees: tilt}});
  }

  isDone(resource) {
    var status = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
    rosnodejs.log.info(resource);
    if (resource === 'TORSO') {
      status = this.torsoStatus;
      rosnodejs.log.info(status);
    }

    if (resource === 'HEAD') {
      status = this.headStatus;
      rosnodejs.log.info(status);
    }

    if (status === actionlib_msgs.msg.GoalStatus.Constants.PREEMPTED ||
        status === actionlib_msgs.msg.GoalStatus.Constants.RECALLED ||
        status === actionlib_msgs.msg.GoalStatus.Constants.REJECTED ||
        status === actionlib_msgs.msg.GoalStatus.Constants.ABORTED ||
        status === actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED ||
        status === actionlib_msgs.msg.GoalStatus.Constants.LOST) {
      return true;
    }

    return false;
  }

  cancel(resource) {
    rosnodejs.log.info('Cancelling ', resource);
    if (resource === 'TORSO') {
      this.torsoClient.cancel();
    }
    // add cancel for HEAD
  }

  setTorso(height, callback) {
    rosnodejs.log.info('Setting torso to ' + height + ' meters');
    this.torsoClient.sendGoal({goal: {height: height}});
    this.torsoClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback();
    });
  }

  tuckArms(tuck_left, tuck_right, callback) {
    rosnodejs.log.info(
        'Setting arms, tuck left: ' + tuck_left +
        ', tuck_right: ' + tuck_right);
    const service_name = '/code_it/api/tuck_arms';
    const client =
        this._nh.serviceClient(service_name, 'code_it_msgs/TuckArms');
    this._nh.waitForService(service_name, 1000).then((ok) => {
      if (ok) {
        const request = new code_it_msgs.srv.TuckArms.Request(
            {tuck_left: tuck_left, tuck_right: tuck_right});
        client.call(request).then((response) => {
          this.error = response.error;
          callback();
        });
      } else {
        this.error = 'TuckArms service not available!';
        callback();
      }
    });
  }

  waitForDuration(seconds, callback) {
    if (seconds <= 0) {
      callback();
    }
    setTimeout(() => {
      callback();
    }, seconds * 1000);
  }
}

module.exports = Robot;
