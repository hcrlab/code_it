# CodeIt!
A standalone [Blockly](https://developers.google.com/blockly/) programming application, integrated with ROS.
It allows you to use a graphical interface to generate code for a robot, and run it.
You implement "primitives" for the robot, which are combined with a subset of JavaScript to form programs.
From the interface, you can run programs and stop them mid-program.

CodeIt! is compatible with [RWS](https://github.com/hcrlab/rws).

## How it works
CodeIt! provides a frontend and a backend.
The frontend is a website that lets you create programs with the Blockly programming interface.
It also has a simple interface for adding, updating, or deleting programs.

The backend provides a ROS actionlib server that runs JavaScript programs.
The programs are run through a sandboxed interpreter.
You must define the robot primitives that the interpreter will run.
Most primitives do very little work and just call a ROS service.
The backend also includes a database of programs that have been created.

Note that CodeIt! itself does not implement any robot functionality.
Instead, it calls ROS services to do the actual work on the robot, assuming that some nodes exist which provide these services.
It is up to you to implement these services to do things on your robot.
We have some pre-made implementations of these services for the [PR2](https://github.com/hcrlab/code_it_pr2) and the [Turtlebot](https://github.com/hcrlab/code_it_turtlebot).
However, these are still in the early stages of development and are not documented.
If you are actually interested in using these, please contact us by filing an issue and we'll be happy to help.

## Getting started
### Installing
This project uses [Git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules), so use `git clone --recursive git@github.com:hcrlab/code_it.git` to clone this repo.

The requirements are:
- MongoDB - install MongoDB and pymongo
- Meteor - install from the Meteor website
- Node - install from the Node website.
  It is highly recommended to use [nvm](https://github.com/creationix/nvm) to install node.
  `nvm` allows you to install and switch between arbitrary versions of node easily.
  When deploying to production, Meteor requires an older version of node to build properly, whereas gulp requires a newer version of node.
  Use `nvm` to switch between the two.
- From the `frontend` folder, run `npm install -g gulp bower && npm install && bower install`

### Running
- `roslaunch rosbridge_server rosbridge_websocket.launch`
- `rosrun code_it programs.py` - This is the backend that saves and loads the programs to and from MongoDB.
- From the `backend` folder, run `meteor` - This is the JavaScript interpreter that runs the programs.
- From the `frontend` folder, run `gulp` - This is the frontend. Go to localhost:5000 to see the page.

### Deploying
#### Build the frontend
If you are not going to host CodeIt! on RWS, then edit `frontend/app/elements/code-it-app/code-it-app.html`.
In the `ready` function, change:
```
ready: function() {
  if (window.location.port === '') {
    this.baseUrl = '/'; // Was /a/code_it/index.html/
  }
},
```

Make sure you are using a relatively new version of node: `nvm use node`
Go to the `frontend` folder and run `gulp`.
This should generate a `www` folder in the root of the repository, with the built frontend.

#### Build the backend
Meteor requires an older version of node to run (0.10.40).
Run `nvm use 0.10.40` to switch to this version.
Go to the `backend` folder and run `meteor build --directory ../build`.
This will generate an application bundle that's ready to be deployed.
Next go to `build/bundle/programs/server` and run `npm install`.

#### Deploy to [RWS](https://github.com/hcrlab/rws)
If you are not using RWS, then simply serve the `www` folder as static content and use `roslaunch code_it app.launch` to run the backend.

If you are using RWS, then copy the repository (with the built frontend and backend) to the RWS catkin workspace.
One limitation is that if you use CodeIt! on RWS, then all RWS apps need to use Node v0.10.40.
Make sure you are configured to use Node v0.10.40 (`nvm use 0.10.40`) and then restart the RWS server in the same terminal window.
CodeIt! should appear in the app list.

## How to add / modify primitives
### Implement the backend
To start with, the primitive should be implemented and accessible through ROS, either as a node listening to a topic, a service, or an action.
Note that your primitive can only take in arguments that are primitive types (Strings, Booleans, Numbers) or Arrays.
Passing in an arbitrary object or a callback function is not supported.

Next, you need to add the primitive to the interpreter.
In `backend/server/robot.js`, add a call to your primitive using roslibjs to the `Robot` object.
Be sure to update the return value of the `Robot` object, at the bottom of the file.

Finally, register your primitive with the interpreter's sandbox.
Edit the function `interpreterApi` in `backend/server/interpreter.js`.
The line `interpreter.setProperty(myRobot, 'myFunction', ...)` means that your primitive will be available as the function `robot.myFunction(...)` in the interpreter.
To create a global function, change the line to `interpreter.setProperty(scope, 'myFunction', ...)`, which creates the global function `myFunction(...)`.

#### Using objects or arrays as arguments or return values
This project uses a slightly modified version of the JS-Interpreter that also allows you to pass objects in or return objects from methods.

To return an object from a primitive, use `toPseudoObject`:
```js
var wrapper = function() {
  var people = Robot.findPeople();
  return interpreter.toPseudoObject(people);
}
```

To use an object passed into a primitive, use `toNativeObject`:
```js
var wrapper = function(pseudoPerson) {
  var person = interpreter.toNativeObject(pseudoPerson);
  var name = Robot.recognizePerson(person);
  return interpreter.createPrimitive(name);
}
```

### Implement the frontend
It's recommended that you read the [Custom Blocks](https://developers.google.com/blockly/custom-blocks/overview) section of the Blockly documentation to learn how to make custom blocks in detail.

The general process is to create the code for your block in the [Block Factory](https://developers.google.com/blockly/custom-blocks/block-factory), which is a meta-programming interface that uses Blockly to generate the code for other Blockly blocks.
The video linked above is helpful for understanding how to use the Block Factory.

Once you have the code generated, put the language code in `frontend/app/blockly/blocks/robot.js`, and the generator stub in `frontend/app/blockly/generators/javascript/robot.js`.
You will need to implement the code generation for your block, such that it calls `robot.myFunction()` with the appropriate arguments.

Finally, add the block to the Blockly toolbox, so that users can see the block and drag it into the program.
To do that, edit `frontend/app/elements/code-it-blockly-toolbox/code-it-blockly-toolbox.html` and add your block to the toolbox according to the [toolbox documentation](https://developers.google.com/blockly/installation/toolbox).
Because toolbox configurations can vary from robot to robot, we don't check in robot-specific blocks to the toolbox in this repository.
You can tell Git not to track your toolbox changes using `git update-index --assume-unchanged frontend/app/elements/code-it-toolbox/code-it-toolbox.html`.

## About
This project is based on *Custom Programs*, as described in [Design and Evaluation of a Rapid Programming System for Service Robots](https://drive.google.com/a/cs.washington.edu/file/d/0B77PnOCaAq8seFE2UFl6ZHBzZVk/view).
If you use this work in your research, we would appreciate you citing it:
```bib
@inproceedings{huang2016design,
  title={Design and Evaluation of a Rapid Programming System for Service Robots},
  author={Huang, Justin and Lau, Tessa and Cakmak, Maya},
  booktitle={Proceedings of the 2016 ACM/IEEE international conference on Human-robot interaction (HRI)},
  pages={295--302},
  year={2016},
  organization={ACM}
}
```
