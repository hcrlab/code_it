/**
 * @license
 * Visual Blocks Language
 *
 * Copyright 2012 Google Inc.
 * https://developers.google.com/blockly/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview Generating JavaScript for robot blocks.
 * @author jstn@cs.washington.edu (Justin Huang)
 */
'use strict';

goog.provide('Blockly.JavaScript.robot');

goog.require('Blockly.JavaScript');

Blockly.JavaScript['robot_display_message_h1h2'] = function(block) {
  var value_h1text = Blockly.JavaScript.valueToCode(block, 'h1Text', Blockly.JavaScript.ORDER_COMMA) || '\'\'';
  var value_h2text = Blockly.JavaScript.valueToCode(block, 'h2Text', Blockly.JavaScript.ORDER_COMMA) || '\'\'';
  var value_timeout = Blockly.JavaScript.valueToCode(block, 'timeout', Blockly.JavaScript.ORDER_COMMA) || 0;
  var code = 'robot.displayMessage(' + value_h1text + ', ' + value_h2text + ', ' + value_timeout + ');\n';
  return code;
};
