# CodeIt!
A standalone [Blockly](https://developers.google.com/blockly/) programming application, integrated with ROS.
It allows you to use a graphical interface to generate code for a robot, and run it.
You implement "primitives" for the robot, which are combined with a subset of JavaScript to form programs.
From the interface, you can run programs and stop them mid-program.

CodeIt! is compatible with [RWS](https://github.com/hcrlab/rws).

<img alt="Screenshot of CodeIt! programming interface" src="https://cloud.githubusercontent.com/assets/1175286/13483675/77a27a02-e0ab-11e5-8c65-40e44c49d274.png" width="450" />
<img alt="Screenshot of CodeIt! program list" src="https://cloud.githubusercontent.com/assets/1175286/13483676/79681482-e0ab-11e5-8bd1-5556c81fd12c.png" width="400" />

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
We have some pre-made implementations of these services for the [PR2](https://github.com/hcrlab/code_it_pr2), [Fetch](https://github.com/hcrlab/code_it_fetch), and the [Turtlebot](https://github.com/hcrlab/code_it_turtlebot).
However, these are still in the early stages of development and are not documented.
If you are actually interested in using these, please contact us by filing an issue and we'll be happy to help.

## Getting started
### Installing
- [ ] Clone to your catkin workspace: `git clone --recursive git@github.com:hcrlab/code_it.git` (Note that this project uses [Git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules))
- [ ] Clone [code_it_msgs](https://github.com/hcrlab/code_it_msgs): `git clone git@github.com:hcrlab/code_it_msgs.git` 
- [ ] Install a robot implementation package like [code_it_pr2](https://github.com/hcrlab/code_it_pr2), [code_it_fetch](https://github.com/hcrlab/code_it_fetch), or [code_it_turtlebot](https://github.com/hcrlab/code_it_turtlebot).
  These implement ROS services to actually control the robot, which CodeIt! calls while running programs.
  You can also implement these services yourself for another robot, see `code_it_msgs` to see services currently defined.
- [ ] Build your workspace: `catkin build` or `catkin_make`
- [ ] Install MongoDB
- [ ] Install pymongo 3.4: `sudo pip install pymongo==3.4`
- [ ] [Install Node.js using NVM](https://github.com/hcrlab/wiki/blob/master/web_development/installing_node.md).
      [nvm](https://github.com/creationix/nvm) allows you to install and switch between arbitrary versions of node easily.
- [ ] Install [our fork of rosnodejs](https://github.com/jstnhuang/rosnodejs)
  ```
  npm install jstnhuang/rosnodejs
  cd node_modules/rosnodejs
  npm install
  npm run compile
  ```
- [ ] Generate messages: `node -e "rosnodejs = require('rosnodejs'); rosnodejs.loadAllPackages();"`
- [ ] Build the frontend.
      From the `frontend` folder, run `npm install && bower update && gulp`

### Running
1. `roslaunch rosbridge_server rosbridge_websocket.launch`
1.  `rosrun code_it programs.py` - This is the backend that saves and loads the programs to and from MongoDB.
1.  `rosrun code_it program_server.js` - This is the JavaScript interpreter that runs the programs.
1.  From the `frontend` folder, run `gulp serve` - This is the frontend. Go to localhost:5000 to see the page.

### Deploying
#### Build the frontend
1. If you are not going to host CodeIt! on RWS, then edit `frontend/app/elements/code-it-app/code-it-app.html`.
1. In the `ready` function, change:

   ```
   ready: function() {
     if (window.location.port === '') {
       this.baseUrl = '/'; // Was /a/code_it/index.html/
     }
   },
   ```

1. Make sure you are using a relatively new version of node: `nvm use node`
1. Go to the `frontend` folder and run `gulp`.
1. This should generate a `www` folder in the root of the repository, with the built frontend.

If you are not using RWS, then simply serve the `www` folder as static content and use `roslaunch code_it pr2_app.launch` to run the backend.
Be sure to double-check the contents of pr2_app.launch or fetch_app.launch, which are designed for the PR2 and Fetch robots running an instance of RWS.

#### Deploy to [RWS](https://github.com/hcrlab/rws)
If you are using RWS, then copy the repository (with the built frontend) to the RWS catkin workspace.
- [ ] Create a symlink for `app.launch`:
      ```
      cd launch
      ln -s fetch_app.launch app.launch
      ```
- [ ] Create a symlink for `code-it-blockly-toolbox.html`
      ```
      cd frontend/app/elements/code-it-blockly-toolbox/
      ln -s code-it-blockly-toolbox-fetch.html code-it-blockly-toolbox.html
      ```
CodeIt! should appear in the app list.

## How to add / modify primitives
### Implement the backend
#### 1. ROS-level implementation
To start with, the primitive should be implemented and accessible through ROS, either as a node listening to a topic, a service, or an action.

#### 2. Add your function to the interpreter
In `src/robot.js`, add a call to your primitive using rosnodejs to the `Robot` object, using existing methods as a template.
- Most primitives will call an asynchronous method (e.g., calling a ROS service).
  In that case, you need to pass a callback argument as the last argument to your function.
  To "return," call the callback with your return value as the only argument.
- If you would like the program to end due to an irrecoverable error, assign `this.error` an error message.

Next, register your primitive with the interpreter's sandbox.
Edit `src/interpreter_api.js` using existing methods as a template.
- The line `interpreter.setProperty(robotObj, 'myFunction', ...)` means that your primitive will be available as the function `robot.myFunction(...)` in the interpreter.
- To create a global function, change the line to `interpreter.setProperty(scope, 'myFunction', ...)`, which creates the global function `myFunction(...)`.
- If your primitive uses an asynchronous method, then pass a callback as the last argument to the wrapper and to your `robot.myFunction` call.
  You will also use `interpreter.createAsyncFunction(wrapper)`.
- If your primitive *does not* use any asynchronous methods, then do not pass a callback to the wrapper.
  You will use `interpreter.createNativeFunction(wrapper)` instead of `createAsyncFunction`.
- Refer to the [JS-Interpreter docs](https://neil.fraser.name/software/JS-Interpreter/docs.html) for more details.

**Using objects or arrays as arguments or return values**

To return an object from a primitive, use `interpreter.nativeToPseudo` as shown below:
```js
var wrapper = function(callback) {
  function nativeToPseudoCallback(result) {
    callback(interpreter.nativeToPseudo(result));
  }
  robot.findPeople(nativeToPseudoCallback);
}
```

To use an object passed into a primitive, use `interpreter.pseudoToNative` as shown below:
```js
var wrapper = function(pseudoPerson, callback) {
  var person = interpreter.pseudoToNative(pseudoPerson);
  robot.recognizePerson(person, callback);
}
```

### Implement the frontend
It's recommended that you read the [Custom Blocks](https://developers.google.com/blockly/custom-blocks/overview) section of the Blockly documentation to learn how to make custom blocks in detail.

The general process is to create the code for your block in the [Block Factory](https://developers.google.com/blockly/custom-blocks/block-factory), which is a meta-programming interface that uses Blockly to generate the code for other Blockly blocks.
The video linked above is helpful for understanding how to use the Block Factory.

Once you have the code generated, put the language code in `frontend/app/blockly/blocks/robot.js`, and the generator stub in `frontend/app/blockly/generators/javascript/robot.js`.
You will need to implement the code generation for your block, such that it calls `robot.myFunction()` with the appropriate arguments.

Build your blocks using `python build.py` inside of the `blockly` folder.

Finally, add the block to the Blockly toolbox, so that users can see the block and drag it into the program.
To do that, edit `frontend/app/elements/code-it-blockly-toolbox/code-it-blockly-toolbox.html` and add your block to the toolbox according to the [toolbox documentation](https://developers.google.com/blockly/installation/toolbox).
Because toolbox configurations can vary from robot to robot, we have created multiple toolbox files for each robot (the PR2 and Fetch).
code-it-blockly-toolbox.html is a symlink that points to one of the robot-specific toolboxes.
You can tell Git not to track your toolbox changes using `git update-index --assume-unchanged frontend/app/elements/code-it-blockly-toolbox/code-it-blockly-toolbox.html`.

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
