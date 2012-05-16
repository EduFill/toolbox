/**
 * Blockly Demo: Maze
 *
 * Copyright 2012 Google Inc.
 * http://code.google.com/p/google-blockly/
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
 * @fileoverview Demonstration of Blockly: Solving a maze.
 * @author fraser@google.com (Neil Fraser)
 */

// Extensions to Blockly's language and JavaScript generator.

// Define Language and JavaScript, in case this file is loaded too early.
if (!Blockly.Language) {
  Blockly.Language = {};
}
Blockly.JavaScript = Blockly.Generator.get('JavaScript');
Blockly.Dart = Blockly.Generator.get('Dart');
Blockly.Python = Blockly.Generator.get('Python');

Blockly.Language.maze_move = {
  // Block for moving forward or backwards.
  category: 'Maze',
  helpUrl: null,
  init: function() {
    this.setColour(290);
    this.addTitle('move');
    var dropdown = new Blockly.FieldDropdown(function() {
      return Blockly.Language.maze_move.DIRECTIONS;
    });
    this.addTitle(dropdown);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setTooltip('Moves the mouse forward or backward one space.');
  }
};

Blockly.Language.maze_move.DIRECTIONS = ['forward', 'backward'];

Blockly.JavaScript.maze_move = function() {
  // Generate JavaScript for moving forward or backwards.
  var direction = Blockly.Language.maze_move.DIRECTIONS
      .indexOf(this.getTitleText(1));
  return 'Maze.move(' + direction + ');\n';
};
Blockly.Dart.maze_move = Blockly.JavaScript.maze_move;
Blockly.Python.maze_move = Blockly.JavaScript.maze_move;

Blockly.Language.maze_turnLeft = {
  // Block for turning left or right.
  category: 'Maze',
  helpUrl: null,
  init: function() {
    this.setColour(290);
    this.addTitle('turn');
    var dropdown = new Blockly.FieldDropdown(function() {
      return Blockly.Language.maze_turnLeft.DIRECTIONS;
    });
    this.addTitle(dropdown);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setTooltip('Turns the mouse left or right by 90 degrees.');
  }
};

Blockly.Language.maze_turnLeft.DIRECTIONS = ['left', 'right'];

Blockly.Language.maze_turnRight = {
  // Block for turning left or right.
  category: 'Maze',
  helpUrl: null,
  init: function() {
    this.setColour(290);
    this.addTitle('turn');
    var dropdown = new Blockly.FieldDropdown(function() {
      return Blockly.Language.maze_turnLeft.DIRECTIONS;
    });
    this.addTitle(dropdown)
    this.setTitleText(Blockly.Language.maze_turnLeft.DIRECTIONS[1], 1);
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setTooltip('Turns the mouse left or right by 90 degrees.');
  }
};

Blockly.JavaScript.maze_turnLeft = function() {
  // Generate JavaScript for turning left or right.
  var direction = Blockly.Language.maze_turnLeft.DIRECTIONS
      .indexOf(this.getTitleText(1));
  return 'Maze.turn(' + direction + ');\n';
};
Blockly.Dart.maze_turnLeft = Blockly.JavaScript.maze_turnLeft;
Blockly.Python.maze_turnLeft = Blockly.JavaScript.maze_turnLeft;

// Turning left and right use the same code.
Blockly.JavaScript.maze_turnRight = Blockly.JavaScript.maze_turnLeft;
Blockly.Dart.maze_turnRight = Blockly.Dart.maze_turnLeft;
Blockly.Python.maze_turnRight = Blockly.Python.maze_turnLeft;

Blockly.Language.maze_isWall = {
  // Block for checking if there a wall.
  category: 'Maze',
  helpUrl: null,
  init: function() {
    this.setColour(290);
    this.setOutput(true);
    this.addTitle('wall');
    var dropdown = new Blockly.FieldDropdown(function() {
      return Blockly.Language.maze_isWall.DIRECTIONS;
    });
    this.addTitle(dropdown);
    this.addTitle('?');
    this.setTooltip('Returns true if there is a wall in ' +
                    'the specified direction.');
  }
};

Blockly.Language.maze_isWall.DIRECTIONS =
    ['ahead', 'to the left', 'to the right', 'behind'];

Blockly.JavaScript.maze_isWall = function() {
  // Generate JavaScript for checking if there is a wall.
  var direction = Blockly.Language.maze_isWall.DIRECTIONS
      .indexOf(this.getTitleText(1));
  return 'Maze.isWall(' + direction + ')';
};
Blockly.Dart.maze_isWall = Blockly.JavaScript.maze_isWall;
Blockly.Python.maze_isWall = Blockly.JavaScript.maze_isWall;


/**
 * Create a namespace for the maze.
 */
var Maze = {};

/**
 * Pixel height and width of each maze square.
 */
Maze.SIZE = 50;

/**
 * Miliseconds between each animation frame.
 */
Maze.STEP_SPEED = 150;

/**
 * The maze's map is a 2D array of numbers.
 * 0: Empty space.
 * 1: Wall.
 * 2: Starting square.
 * 3. Finish square.
 */
Maze.MAP = [
  [1, 1, 1, 1, 1, 1, 1, 1],
  [1, 0, 0, 1, 0, 1, 3, 1],
  [1, 0, 0, 1, 0, 0, 0, 1],
  [1, 0, 1, 1, 0, 1, 1, 1],
  [1, 0, 0, 0, 0, 0, 0, 1],
  [1, 1, 0, 1, 1, 1, 0, 1],
  [1, 2, 0, 0, 0, 1, 0, 1],
  [1, 1, 1, 1, 1, 1, 1, 1]];

/**
 * Constants for cardinal directions.
 */
Maze.NORTH = 0;
Maze.EAST = 1;
Maze.SOUTH = 2;
Maze.WEST = 3;

/**
 * PIDs of animation tasks currently executing.
 */
Maze.pidList = [];

/**
 * Initialize Blockly and the maze.  Called on page load.
 */
Maze.init = function() {
  //window.onbeforeunload = function() {
  //  return 'Leaving this page will result in the loss of your work.';
  //};

  Blockly.pathToBlockly = '../../';
  Blockly.inject(document.getElementById('editors'));

  // Find and name the injected SVG object.
  document.getElementsByTagName('svg')[0].id = 'content_blocks';

  // Load the editor with a starting block.
  var xml = Blockly.Xml.textToDom(
      '<xml>' +
      '  <block type="maze_move" x="84" y="99">' +
      '    <title>move</title>' +
      '    <title>forward</title>' +
      '    <next></next>' +
      '  </block>' +
      '</xml>');
  Blockly.Xml.domToWorkspace(Blockly.mainWorkspace, xml);

  // Locate the start and finish squares.
  for (var y = 0; y < Maze.MAP.length; y++) {
    for (var x = 0; x < Maze.MAP[0].length; x++) {
      if (Maze.MAP[y][x] == 2) {
        Maze.start_ = {x: x, y: y};
      } else if (Maze.MAP[y][x] == 3) {
        Maze.finish_ = {x: x, y: y};
      }
    }
  }

  // Record the map's offset.
  Maze.mapOffsetLeft_ = 0;
  Maze.mapOffsetTop_ = 0;
  var element = document.getElementById('map');
  while (element) {
    Maze.mapOffsetLeft_ += element.offsetLeft;
    Maze.mapOffsetTop_ += element.offsetTop;
    element = element.offsetParent;
  }

  // Make the 'Blocks' tab line up with the toolbox.  
  Blockly.bindEvent_(window, 'resize', null, function() {
    document.getElementById('tab_blocks').style.minWidth =
        (Blockly.Toolbox.width - 40) + 'px';
        // Account for the 19 pixel margin and 1 pixel border on each side.
    });
  Blockly.fireUiEvent(document, window, 'resize');

  // Move the finish icon into position.
  var finishIcon = document.getElementById('finish');
  finishIcon.style.top = Maze.mapOffsetTop_ +
      Maze.SIZE * (Maze.finish_.y + 0.5) - finishIcon.offsetHeight;
  finishIcon.style.left = Maze.mapOffsetLeft_ +
      Maze.SIZE * (Maze.finish_.x + 0.5) - finishIcon.offsetWidth / 2;

  Maze.reset();
};

Maze.reset = function() {
  Maze.pegmanX = Maze.start_.x;
  Maze.pegmanY = Maze.start_.y;
  Maze.pegmanD = Maze.EAST;
  Maze.displayPegman(Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4);
  // Kill all tasks.
  for (var x = 0; x < Maze.pidList.length; x++) {
    window.clearTimeout(Maze.pidList[x]);
  }
  Maze.pidList = [];
}

/**
 * List of tab names.
 * @private
 */
Maze.TABS_ = ['blocks', 'javascript', 'dart', 'python', 'xml'];

/**
 * Switch the visible pane when a tab is clicked.
 */
Maze.tabClick = function(id) {
  // First, deselect all tabs and hide all panes.
  for (var x in Maze.TABS_) {
    document.getElementById('tab_' + Maze.TABS_[x]).className = 'taboff';
    document.getElementById('content_' + Maze.TABS_[x]).style.display = 'none';
  }
  // Second, select the active tab.
  document.getElementById(id).className = 'tabon';
  // Third, show the selected pane.
  var content = document.getElementById(id.replace('tab_', 'content_'));
  content.style.display = 'block';
  // Fourth, initialize the pane.
  if (id == 'tab_xml') {
    var xmlDom = Blockly.Xml.workspaceToDom(Blockly.mainWorkspace);
    var xmlText = Blockly.Xml.domToPrettyText(xmlDom);
    var xmlHtml = xmlText.replace(/&/g, '&amp;')
        .replace(/</g, '&lt;').replace(/>/g, '&gt;');
    content.innerHTML = xmlHtml;
  } else if (id == 'tab_javascript') {
    content.innerHTML = Blockly.Generator.workspaceToCode('JavaScript');
  } else if (id == 'tab_dart') {
    content.innerHTML = Blockly.Generator.workspaceToCode('Dart');
  } else if (id == 'tab_python') {
    content.innerHTML = Blockly.Generator.workspaceToCode('Python');
  }
};

Maze.runButtonClick = function() {
  document.getElementById('runButton').style.display = 'none';
  document.getElementById('resetButton').style.display = 'inline';
  Maze.execute();
};

Maze.resetButtonClick = function() {
  document.getElementById('runButton').style.display = 'inline';
  document.getElementById('resetButton').style.display = 'none';
  Maze.reset();
};

/**
 * Execute the user's code.  Heaven help us...
 */
Maze.execute = function() {
  Maze.path = [];
  var code = Blockly.Generator.workspaceToCode('JavaScript');
  try {
    eval(code);
  } catch (e) {
    // A boolean is thrown for normal termination.
    // Abnormal termination is a user error.
    if (typeof e != 'boolean') {
      alert(e);
    }
  }
  // Maze.path now contains a transcript of all the user's actions.
  // Reset the maze and animate the transcript.
  Maze.reset();
  Maze.pidList.push(window.setTimeout(Maze.animate, 100));
};

/**
 * Iterate through the recorded path and animate pegman's actions.
 */
Maze.animate = function() {
  // All tasks should be complete now.  Clean up the PID list.
  Maze.pidList = [];
  var action;
  do {
    action = Maze.path.shift();
    if (!action) {
      return;
    }
  } while (action == 'look');

  if (action == 'north') {
    Maze.schedule([Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4],
                  [Maze.pegmanX, Maze.pegmanY - 1, Maze.pegmanD * 4]);
    Maze.pegmanY--;
  } else if (action == 'east') {
    Maze.schedule([Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4],
                  [Maze.pegmanX + 1, Maze.pegmanY, Maze.pegmanD * 4]);
    Maze.pegmanX++;
  } else if (action == 'south') {
    Maze.schedule([Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4],
                  [Maze.pegmanX, Maze.pegmanY + 1, Maze.pegmanD * 4]);
    Maze.pegmanY++;
  } else if (action == 'west') {
    Maze.schedule([Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4],
                  [Maze.pegmanX - 1, Maze.pegmanY, Maze.pegmanD * 4]);
    Maze.pegmanX--;
  } else if (action.substring(0, 4) == 'fail') {
    Maze.scheduleFail(action.substring(5) == 'forwards');
  } else if (action == 'left') {
    Maze.schedule([Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4],
                  [Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4 - 4]);
    Maze.pegmanD = Maze.constrainDirection4(Maze.pegmanD - 1);
  } else if (action == 'right') {
    Maze.schedule([Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4],
                  [Maze.pegmanX, Maze.pegmanY, Maze.pegmanD * 4 + 4]);
    Maze.pegmanD = Maze.constrainDirection4(Maze.pegmanD + 1);
  } else if (action == 'finish') {
    Maze.scheduleFinish();
  }
  
  Maze.pidList.push(window.setTimeout(Maze.animate, Maze.STEP_SPEED * 5));
};

/**
 * Schedule the animations for a move or turn.
 * @param {!Array.<number>} startPos X, Y and direction starting points.
 * @param {!Array.<number>} endPos X, Y and direction ending points.
 */
Maze.schedule = function(startPos, endPos) {
  var deltas = [(endPos[0] - startPos[0]) / 4,
                (endPos[1] - startPos[1]) / 4,
                (endPos[2] - startPos[2]) / 4];
  Maze.displayPegman(startPos[0] + deltas[0],
                     startPos[1] + deltas[1],
                     Maze.constrainDirection16(startPos[2] + deltas[2]));
  Maze.pidList.push(window.setTimeout(function() {
      Maze.displayPegman(startPos[0] + deltas[0] * 2,
          startPos[1] + deltas[1] * 2,
          Maze.constrainDirection16(startPos[2] + deltas[2] * 2));
    }, Maze.STEP_SPEED));
  Maze.pidList.push(window.setTimeout(function() {
      Maze.displayPegman(startPos[0] + deltas[0] * 3,
          startPos[1] + deltas[1] * 3,
          Maze.constrainDirection16(startPos[2] + deltas[2] * 3));
    }, Maze.STEP_SPEED * 2));
  Maze.pidList.push(window.setTimeout(function() {
      Maze.displayPegman(endPos[0], endPos[1],
          Maze.constrainDirection16(endPos[2]));
    }, Maze.STEP_SPEED * 3));
};

/**
 * Schedule the animations for a failed move.
 * @param {boolean} forwards True if forwards, false if backwards.
 */
Maze.scheduleFail = function(forwards) {
  var deltaX = 0;
  var deltaY = 0;
  if (Maze.pegmanD == 0) {
    deltaY = -0.25;
  } else if (Maze.pegmanD == 1) {
    deltaX = 0.25;
  } else if (Maze.pegmanD == 2) {
    deltaY = 0.25;
  } else if (Maze.pegmanD == 3) {
    deltaX = -0.25;
  }
  if (!forwards) {
    deltaX = - deltaX;
    deltaY = - deltaY;
  }
  var direction16 = Maze.constrainDirection16(Maze.pegmanD * 4);
  Maze.displayPegman(Maze.pegmanX + deltaX,
                     Maze.pegmanY + deltaY,
                     direction16);
  Maze.pidList.push(window.setTimeout(function() {
    Maze.displayPegman(Maze.pegmanX,
                       Maze.pegmanY,
                       direction16);
    }, Maze.STEP_SPEED));
  Maze.pidList.push(window.setTimeout(function() {
    Maze.displayPegman(Maze.pegmanX + deltaX,
                       Maze.pegmanY + deltaY,
                       direction16);
    }, Maze.STEP_SPEED * 2));
  Maze.pidList.push(window.setTimeout(function() {
      Maze.displayPegman(Maze.pegmanX, Maze.pegmanY, direction16);
    }, Maze.STEP_SPEED * 3));
};

/**
 * Schedule the animations for a victory dance.
 */
Maze.scheduleFinish = function() {
  var direction16 = Maze.constrainDirection16(Maze.pegmanD * 4);
  Maze.displayPegman(Maze.pegmanX, Maze.pegmanY, 16);
  Maze.pidList.push(window.setTimeout(function() {
    Maze.displayPegman(Maze.pegmanX, Maze.pegmanY, 17);
    }, Maze.STEP_SPEED));
  Maze.pidList.push(window.setTimeout(function() {
    Maze.displayPegman(Maze.pegmanX, Maze.pegmanY, 16);
    }, Maze.STEP_SPEED * 2));
  Maze.pidList.push(window.setTimeout(function() {
      Maze.displayPegman(Maze.pegmanX, Maze.pegmanY, direction16);
    }, Maze.STEP_SPEED * 3));
};

/**
 * Display Pegman at a the specified location, facing the specified direction.
 * @param {number} x Horizontal grid (or fraction thereof).
 * @param {number} y Vertical grid (or fraction thereof).
 * @param {number} d Direction (0 - 15) or dance (16 - 17).
 */
Maze.displayPegman = function(x, y, d) {
  var pegmanIcon = document.getElementById('pegman');
  pegmanIcon.style.top = Maze.mapOffsetTop_ +
      Maze.SIZE * (y + 0.5) - pegmanIcon.offsetHeight / 2 - 8;
  pegmanIcon.style.left = Maze.mapOffsetLeft_ +
      Maze.SIZE * (x + 0.5) - pegmanIcon.offsetHeight / 2 + 2;
  pegmanIcon.style.backgroundPosition = -d * pegmanIcon.offsetWidth;
};

/**
 * Keep the direction within 0-3, wrapping at both ends.
 * @param {number} d Potentially out-of-bounds direction value.
 * @return {number} Legal direction value.
 */
Maze.constrainDirection4 = function(d) {
  if (d < 0) {
    d += 4;
  } else if (d > 3) {
    d -= 4;
  }
  return d;
};

/**
 * Keep the direction within 0-15, wrapping at both ends.
 * @param {number} d Potentially out-of-bounds direction value.
 * @return {number} Legal direction value.
 */
Maze.constrainDirection16 = function(d) {
  if (d < 0) {
    d += 16;
  } else if (d > 15) {
    d -= 16;
  }
  return d;
};

/**
 * If the user has executed too many actions, we're probably in an infinite
 * loop.  Sadly I wasn't able to solve the Halting Problem for this demo.
 * @throws {false} Throws an error to terminate the user's program.
 */
Maze.checkTimeout = function() {
  if (Maze.path.length >= 1000) {
    throw false;
  }
};

// API

/**
 * Move pegman forwards or backwards.
 * @param {number} direction Direction to move (0 = forward, 1 = backward).
 */
Maze.move = function(direction) {
  if (Maze.isWall(direction ? 3: 0)) {
    Maze.path.push('fail_' + (direction ? 'backwards': 'forwards'));
    return;
  }
  var effectiveDirection = Maze.pegmanD;
  if (direction) {
    // Moving backwards.  Flip the effective direction.
    effectiveDirection = Maze.constrainDirection4(effectiveDirection + 2);
  }
  if (effectiveDirection == Maze.NORTH) {
    Maze.pegmanY--;
    Maze.path.push('north');
  } else if (effectiveDirection == Maze.EAST) {
    Maze.pegmanX++;
    Maze.path.push('east');
  } else if (effectiveDirection == Maze.SOUTH) {
    Maze.pegmanY++;
    Maze.path.push('south');
  } else if (effectiveDirection == Maze.WEST) {
    Maze.pegmanX--;
    Maze.path.push('west');
  }
  if (Maze.pegmanX == Maze.finish_.x && Maze.pegmanY == Maze.finish_.y) {
    // Finished.  Terminate the user's program.
    Maze.path.push('finish');
    throw true;
  }
};

/**
 * Turn pegman left or right.
 * @param {number} direction Direction to turn (0 = left, 1 = right).
 */
Maze.turn = function(direction) {
  Maze.checkTimeout();
  if (direction) {
    // Right turn (clockwise).
    Maze.pegmanD++;
    Maze.path.push('right');
  } else {
    // Left turn (counterclockwise).
    Maze.pegmanD--;
    Maze.path.push('left');
  }
  Maze.pegmanD = Maze.constrainDirection4(Maze.pegmanD);
};

/**
 * Is there a wall next to pegman?
 * @param {number} direction Direction to look
 *     (0 = ahead, 1 = left, 2 = right, 3 = behind).
 * @return {boolean} True if there is a wall.
 */
Maze.isWall = function(direction) {
  Maze.checkTimeout();
  Maze.path.push('look');
  var effectiveDirection = Maze.pegmanD;
  if (direction == 1) {  // Left
    effectiveDirection--; 
  } else if (direction == 2) { // Right
    effectiveDirection++; 
  } else if (direction == 3) { // Behind
    effectiveDirection += 2;
  }
  effectiveDirection = Maze.constrainDirection4(effectiveDirection);
  var square;
  if (effectiveDirection == Maze.NORTH) {
    square = Maze.MAP[Maze.pegmanY - 1][Maze.pegmanX];
  } else if (effectiveDirection == Maze.EAST) {
    square = Maze.MAP[Maze.pegmanY][Maze.pegmanX + 1];
  } else if (effectiveDirection == Maze.SOUTH) {
    square = Maze.MAP[Maze.pegmanY + 1][Maze.pegmanX];
  } else if (effectiveDirection == Maze.WEST) {
    square = Maze.MAP[Maze.pegmanY][Maze.pegmanX - 1];
  }
  return square == 1;
};
