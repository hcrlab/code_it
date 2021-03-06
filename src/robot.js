'use strict';

const rosnodejs = require('rosnodejs');
const actionlib_msgs = rosnodejs.require('actionlib_msgs');
const code_it_msgs = rosnodejs.require('code_it_msgs');

// The implementation of the primitives.
class Robot {
  constructor(nh) {
    this._nh = nh;
    this.error = '';  // Most recent error message, empty string for no error.

    this.timer_on = false;
    this.timer_id = '';

    this.askClient = this._nh.actionClientInterface(
        '/code_it/api/ask_multiple_choice', 'code_it_msgs/AskMultipleChoice');
    this.askMCResult = null;
    this.askClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.askStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.askStatus = msg.status_list[msg.status_list.length - 1].status;
      }
    });

    this.displayClient = this._nh.actionClientInterface(
        '/code_it/api/display_message', 'code_it_msgs/DisplayMessage');
    this.displayClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.displayStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.displayStatus = msg.status_list[msg.status_list.length - 1].status;
      }
    });

    this.goToClient = this._nh.actionClientInterface(
        '/code_it/api/go_to', 'code_it_msgs/GoTo');
    this.goToResult = null;
    this.goToClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.goToStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.goToStatus = msg.status_list[msg.status_list.length - 1].status;
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
    });

    this.rapidPbDClient = this._nh.actionClientInterface(
        '/code_it/api/run_pbd_action', 'code_it_msgs/RunPbdAction');
    this.rapidPbDResult = null;
    this.rapidPbDClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.rapidPbDStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.rapidPbDStatus =
            msg.status_list[msg.status_list.length - 1].status;
      }
    });

    this.slipGripperClient = this._nh.actionClientInterface(
        '/code_it/api/slip_gripper', 'code_it_msgs/SlipGripper');
    this.slipGripperResult = null;
    
    this.collectSpeechClient = this._nh.actionClientInterface(
	    '/code_it/api/collect_speech', 'code_it_msgs/CollectSpeech');
    this.collectSpeechResult = null;
    this.collectSpeechClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.collectSpeechStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.collectSpeechStatus = msg.status_list[msg.status_list.length - 1].status;
      }
    });
    
    this.collectSpeechWakeWordClient = this._nh.actionClientInterface(
	'/code_it/api/collect_speech_wake_word', 'code_it_msgs/CollectSpeechWakeWord');
    this.collectSpeechWakeWordResult = null;
    this.collectSpeechWakeWordClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.collectSpeechWakeWordStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.collectSpeechWakeWordStatus = msg.status_list[msg.status_list.length - 1].status;
      }
    });
     
    this.speechContainsClient = this._nh.actionClientInterface(
            '/code_it/api/speech_contains', 'code_it_msgs/SpeechContains');
    this.speechContainsResult = null;

    this.resetSensorsClient = this._nh.actionClientInterface(
        '/code_it/api/reset_sensors', 'code_it_msgs/Empty');

    this.gripperClient = this._nh.actionClientInterface(
        '/code_it/api/set_gripper', 'code_it_msgs/SetGripper');
    this.gripperClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.gripperStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.gripperStatus = msg.status_list[msg.status_list.length - 1].status;
      }
    });

    this.torsoClient = this._nh.actionClientInterface(
        '/code_it/api/set_torso', 'code_it_msgs/SetTorso');
    this.torsoClient.on('status', (msg) => {
      if (msg.status_list.length == 0) {
        this.torsoStatus = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
      } else {
        this.torsoStatus = msg.status_list[msg.status_list.length - 1].status;
      }
    });

    this.positionClient = this._nh.actionClientInterface(
        '/code_it/api/get_position', 'code_it_msgs/GetPosition');

    this.locationClient = this._nh.actionClientInterface(
        '/code_it/api/get_location', 'code_it_msgs/GetLocation');
  }

  // Service implemented actions

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
            rosnodejs.log.info('Error', response.error);
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

  // Concurrent action implementation

  startAskMultipleChoice(question, choices) {
    rosnodejs.log.info(
        'Starting to ask: ' + question + ', choices: ' + choices);
    this.askMCResult = null;
    this.askClient.once('result', (msg) => {
      this.askMCResult = msg.result.choice;
    });
    this.askClient.sendGoal({goal: {question: question, choices: choices}});
  }

  startDisplayMessage(h1text, h2text) {
    rosnodejs.log.info('Starting to display h1: ' + h1text + ', h2: ' + h2text);
    this.displayClient.sendGoal({goal: {h1_text: h1text, h2_text: h2text}});
  }

  startGoTo(location) {
    rosnodejs.log.info('Starting to go to: ' + location);
    this.goToResult = null;
    this.goToClient.once('result', (msg) => {
      this.goToResult = (msg.result.error === '');
    });
    this.goToClient.sendGoal({goal: {location: location}});
  }

  startHead(pan, tilt) {
    rosnodejs.log.info(
        'Starting to move head to ' + pan + ', ' + tilt + ' degrees');
    this.headClient.sendGoal({goal: {pan_degrees: pan, tilt_degrees: tilt}});
  }

  startRapidPbD(program) {
    rosnodejs.log.info('Starting to run: ' + program);
    this.rapidPbDResult = null;
    this.rapidPbDClient.once('result', (msg) => {
      this.rapidPbDResult = (msg.result.error === '');
    });
    this.rapidPbDClient.sendGoal(
        {goal: {action_id: '', name: program, landmarks: []}});
  }

  startCloseGripper(force) {
    rosnodejs.log.info('Starting to close gripper with ' + force + ' N');
    this.gripperClient.sendGoal(
        {goal: {gripper: 0, action: 2, max_effort: force}});
  }

  startOpenGripper() {
    rosnodejs.log.info('Starting to open gripper');
    this.gripperClient.sendGoal({goal: {gripper: 0, action: 1, max_effort: 0}});
  }

  startTorso(height) {
    rosnodejs.log.info('Starting to set torso to ' + height + ' meters');
    this.torsoClient.sendGoal({goal: {height: height}});
  }

  slipGripper(callback) {
    this.slipGripperClient.sendGoal({goal: {}});
    this.slipGripperResult = null;
    this.slipGripperClient.once('result', (msg) => {
      this.slipGripperResult = msg.result.slipped;
      callback(this.slipGripperResult);
    });
  }

  startCollectSpeech(time) {
    rosnodejs.log.info('Starting to collect speech for ' + time + ' seconds');
    this.collectSpeechResult = null;
    this.collectSpeechClient.once('result', (msg) => {
      this.collectSpeechResult = msg.result.data;
    });
    this.collectSpeechClient.sendGoal({goal: {time: time}});
  }

  startCollectSpeechWakeWord(wake_word) {
    rosnodejs.log.info('Starting to listen for speech input beginning with ' + wake_word);
    this.collectSpeechWakeWordResult = null;
    this.collectSpeechWakeWordClient.once('result', (msg) => {
      this.collectSpeechWakeWordResult = msg.result.data;
    });
    this.collectSpeechWakeWordClient.sendGoal({goal: {wake_word: wake_word}});
  } 

  speechContains(speech_data, program_input, callback) { 
    rosnodejs.log.info("checking if speech contains a phrase");
    this.speechContainsClient.sendGoal({goal: {speech_data: speech_data, program_input: program_input}});
    this.speechContainsResult = null;
    this.speechContainsClient.once('result', (msg) => {
	this.speechContainsResult = msg.result.contains;
	callback(this.speechContainsResult);
    });
  }  
  
  resetRobotSensors() {
    this.resetSensorsClient.sendGoal({goal: {}});
  }

  startTimer(seconds) {
    clearTimeout(this.timer_id);
    if (seconds > 0) {
      this.timer_on = true;
      this.timer_id = setTimeout(() => {
        this.timer_on = false;
      }, seconds * 1000);
    }
  }

  isDone(resource) {
    var status = actionlib_msgs.msg.GoalStatus.Constants.SUCCEEDED;
    if (resource === 'TORSO') {
      status = this.torsoStatus;
    } else if (resource === 'HEAD') {
      status = this.headStatus;
    } else if (resource === 'GRIPPER') {
      status = this.gripperStatus;
    } else if (resource === 'QUESTION') {
      status = this.askStatus;
    } else if (resource === 'NAVIGATION') {
      status = this.goToStatus;
    } else if (resource === 'PBD') {
      status = this.rapidPbDStatus;
    } else if (resource === 'TIMER') {
      return !this.timer_on;
    } else if (resource === 'SPEECH_INPUT') {
      rosnodejs.log.info('speech input status: ' + this.collectSpeechStatus);
      status = this.collectSpeechStatus;
    } else if (resource === 'SPEECH_INPUT_WAKE_WORD') {
      status = this.collectSpeechWakeWordStatus;
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

  // had the current waitForAction uncommented code inside
  // waitHelper(resource, callback) {}

  async waitForAction(resource, callback) {
    /* if (resource === 'ALL_ACTIONS') {
      // this doesn't work //
      var a = await this.waitHelper('TORSO', callback);
      var b = await this.waitHelper('HEAD', callback);
      var c = await this.waitHelper('GRIPPER', callback);
      rosnodejs.log.info(await a);
      rosnodejs.log.info(await b);
      rosnodejs.log.info(await c);
    } else {
      // this worked //
      await this.waitHelper(resource, callback);
    } */

    rosnodejs.log.info('Waiting for ' + resource + ' to be done');
    if (this.isDone(resource)) {
      callback();
    } else {
      var client = null;
      if (resource === 'TORSO') {
        client = this.torsoClient;
      } else if (resource === 'HEAD') {
        client = this.headClient;
      } else if (resource === 'GRIPPER') {
        client = this.gripperClient;
      } else if (resource === 'QUESTION') {
        client = this.askClient;
      } else if (resource === 'NAVIGATION') {
        client = this.goToClient;
      } else if (resource === 'PBD') {
        client = this.rapidPbDClient;
      }
      client.once('result', (actionResult) => {
        callback();
      });
    }
  }

  getResult(resource) {
    if (resource === 'QUESTION') {
      return this.askMCResult;
    } else if (resource === 'NAVIGATION') {
      return this.goToResult;
    } else if (resource === 'PBD') {
      return this.rapidPbDResult;
    } else if (resource === 'SPEECH_INPUT') {
      return this.collectSpeechResult;
    } else if (resource === 'SPEECH_INPUT_WAKE_WORD') {
      return this.collectSpeechWakeWordResult;
    }
  }

  cancel(resource) {
    if (resource === 'TORSO') {
      this.torsoClient.cancel();
    } else if (resource === 'HEAD') {
      this.headClient.cancel();
    } else if (resource === 'GRIPPER') {
      this.gripperClient.cancel();
    } else if (resource === 'QUESTION') {
      this.askClient.cancel();
    } else if (resource === 'NAVIGATION') {
      this.goToClient.cancel();
    } else if (resource === 'PBD') {
      this.rapidPbDClient.cancel();
    } else if (resource === 'TIMER') {
      this.timer_on = false;
      clearTimeout(this.timer_id);
    } else if (resource === 'SPEECH_INPUT') {
      this.collectSpeechClient.cancel();
    } else if (resource === 'SPEECH_INPUT_WAKE_WORD') {
      this.collectSpeechWakeWordClient.cancel();
    }
  }

  cancelAll() {
    this.torsoClient.cancel();
    this.headClient.cancel();
    this.gripperClient.cancel();
    this.askClient.cancel();
    this.goToClient.cancel();
    this.rapidPbDClient.cancel();
    this.slipGripperClient.cancel();
    this.collectSpeechClient.cancel();
    this.speechContainsClient.cancel();
    this.collectSpeechWakeWordClient.cancel();
    this.timer_on = false;
    clearTimeout(this.timer_id);
  }

  // Consecutive action implementation

  askMultipleChoice(question, choices, callback) {
    rosnodejs.log.info('Asking: ' + question + ', choices: ' + choices);
    this.askClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback(actionResult.result.choice);
    });
    this.askClient.sendGoal({goal: {question: question, choices: choices}});
  }

  displayMessage(h1text, h2text, callback) {
    rosnodejs.log.info('Displaying h1: ' + h1text + ', h2: ' + h2text);
    this.displayClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback();
    });
    this.displayClient.sendGoal({goal: {h1_text: h1text, h2_text: h2text}});
  }

  goTo(location, callback) {
    rosnodejs.log.info('Going to: ' + location);
    this.goToClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback(actionResult.result.error === '');
    });
    this.goToClient.sendGoal({goal: {location: location}});
  }

  moveHead(pan, tilt, callback) {
    rosnodejs.log.info('Moving head to: ' + pan + ', ' + tilt + ' degrees');
    this.headClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback();
    });
    this.headClient.sendGoal({goal: {pan_degrees: pan, tilt_degrees: tilt}});
  }

  runRapidPbdProgram(name, callback) {
    rosnodejs.log.info('Running Rapid PbD program: ' + name);
    this.rapidPbDClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback(actionResult.result.error === '');
    });
    this.rapidPbDClient.sendGoal(
        {goal: {action_id: '', name: name, landmarks: []}});
  }

  setGripper(side, action, max_effort, callback) {
    rosnodejs.log.info(
        'Setting gripper, action: ' + action + ', effort: ' + max_effort);
    this.gripperClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback();
    });
    this.gripperClient.sendGoal(
        {goal: {gripper: 0, action: action, max_effort: max_effort}});
  }

  setTorso(height, callback) {
    rosnodejs.log.info('Setting torso to ' + height + ' meters');
    this.torsoClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback();
    });
    this.torsoClient.sendGoal({goal: {height: height}});
  }

  getPosition(resource, callback) {
    rosnodejs.log.info('Getting position of ' + resource);
    this.positionClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback(actionResult.result.position);
    });
    this.positionClient.sendGoal({goal: {name: resource}});
  }

  getLocation(callback) {
    rosnodejs.log.info('Getting location of the robot');
    this.locationClient.once('result', (actionResult) => {
      if (actionResult.result.error !== '') {
        this.error = actionResult.result.error;
      }
      callback(actionResult.result.name);
    });
    this.locationClient.sendGoal({goal: {}});
  }

  collectSpeech(time, callback) {
    rosnodejs.log.info("Collecting speech for " + time + " seconds");
    this.collectSpeechClient.once('result', (speechResult) => {
      if (speechResult.result.error !== '') {
        this.error = speechResult.result.error;
      }
      rosnodejs.log.info(speechResult.result.data);
      callback(speechResult.result.data);
    });
    this.collectSpeechClient.sendGoal({goal: {time: time}});
  }

  collectSpeechWakeWord(wake_word, callback) {
    rosnodejs.log.info("Collecting speech (waiting for " + wake_word + ")");
    this.collectSpeechWakeWordClient.once('result', (speechResult) => {
      if (speechResult.result.error !== '') {
        this.error = speechResult.result.error;
      }
      rosnodejs.log.info(speechResult.result.data);
      callback(speechResult.result.data);
    });
    this.collectSpeechWakeWordClient.sendGoal({goal: {wake_word: wake_word}});
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
