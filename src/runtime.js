'use strict';

const rosnodejs = require('rosnodejs');
const interpreterApi = require('./interpreter_api.js');
const jsinterpreter = require('./jsinterpreter/interpreter.js');

class Runtime {
  constructor(robot, isRunningPub, errorPub) {
    this._robot = robot;
    this._isRunningPub = isRunningPub;
    this._errorPub = errorPub;
    this._isRunning = false;
    this._isRunningPub.publish({data: false});
  }

  _publishError(message) {
    this._errorPub.publish({data: message});
  }

  _onProgramEnd() {
    rosnodejs.log.info('The program has ended.');
    this._isRunningPub.publish({data: false});
  }

  execute(goalHandle) {
    const program = goalHandle.getGoal().program;
    const interpreter =
        new jsinterpreter.Interpreter(program, (interpreter, scope) => {
          interpreterApi(interpreter, scope, this._robot);
        });
    this._isRunning = true;
    this._isRunningPub.publish({data: true});
    goalHandle.setAccepted();

    let lastBlockId = '';  // Last block ID executed.

    let nextStep = () => {
      if (!this._isRunning) {
        this._onProgramEnd();
        goalHandle.setCancelled({});
        return;
      }
      try {
        var ok = interpreter.step();
        if (ok) {
          const error = this._robot.error;
          if (error) {
            this._publishError(error);
            this._robot.error = '';
          }
          if (lastBlockId !== interpreter.blockId) {
            lastBlockId = interpreter.blockId;
            var feedback = {block_id: interpreter.blockId || ''};
            goalHandle.publishFeedback(feedback);
          }
          setTimeout(nextStep, 0);
        } else {
          rosnodejs.log.info('Program complete.');
          goalHandle.setSucceeded({});
          this._onProgramEnd();
          return;
        }
      } catch (e) {
        console.log('Error: ');
        console.log(e);
        e.stack && console.log(e.stack);
        goalHandle.setAborted({}, e.toString());
        this._publishError(
            'There was an error while running the program: ' + e.toString());
        this._onProgramEnd();
        return;
      }
    };
    nextStep();
  }

  stop() {
    rosnodejs.log.info('Program was stopped by the user.');
    this._robot.cancelAll();
    this._isRunning = false;
  }

  shutdown() {
    this._isRunning = false;
    this._onProgramEnd();
  }
}

module.exports = Runtime;
