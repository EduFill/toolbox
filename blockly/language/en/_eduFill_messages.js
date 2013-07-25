/**
 * Visual Blocks Language
 *
 * Copyright 2012 Google Inc.
 * http://code.google.com/p/blockly/
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
 * @fileoverview English strings for categories added by the eduFill project.
 * @author marc.wollenweber@smail.inf.h-brs.de (Marc Wollenweber)
 */
 
'use strict';

Blockly.ChangeModeTooltip = 'ChangeMode (checkbox): Activate the Changemode whenever a deeper view into the blocks functioning is required.';

Blockly.LANG_UNITS_COLOUR = 190;

// Low level Blocks.
Blockly.LANG_LOW_LEVEL_COLOUR = 0;

Blockly.LANG_LOW_LEVEL_ROS_INIT_NODE_EMPTY = ''
Blockly.LANG_LOW_LEVEL_ROS_INIT_NODE_TRUE = 'true';
Blockly.LANG_LOW_LEVEL_ROS_INIT_NODE_FALSE = 'false';

Blockly.LANG_LOW_LEVEL_ROS_LOG_EMPTY = '';
Blockly.LANG_LOW_LEVEL_ROS_LOG_DEBUG = 'debug';
Blockly.LANG_LOW_LEVEL_ROS_LOG_INFO = 'info';
Blockly.LANG_LOW_LEVEL_ROS_LOG_WARN = 'warn';
Blockly.LANG_LOW_LEVEL_ROS_LOG_ERROR = 'err';
Blockly.LANG_LOW_LEVEL_ROS_LOG_FATAL = 'fatal';

Blockly.LANG_LOW_LEVEL_IMPORT_MSG = 'import';
Blockly.LANG_LOW_LEVEL_FROM_IMPORT_MSG = 'from x import';
Blockly.LANG_LOW_LEVEL_MAIN_FRAME = 'import frame';

// Mid level Blocks.
Blockly.LANG_MID_LEVEL_COLOUR = 90;

Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_LINEAR_X = 'linear.x (m/s)';
Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_LINEAR_Y = 'linear.y (m/s)';
Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_LINEAR_Z = 'linear.z (m/s)';
Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_ANGULAR_X = 'angular.x (rad/s)';
Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_ANGULAR_Y = 'angular.y (rad/s)';
Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_ANGULAR_Z = 'angular.z (rad/s)';
Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_SLEEP_TIME = 'sleep time';
Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_TOOLTIP = 'Move robot with twist. \n' + 
                                                    'Linear values in meter per seconds \n' +
                                                    'Angular values in radians per second';

Blockly.gripperString = [
            ['open', [0.0115, 0.0115]],
            ['close', [0.0, 0.0]]
            ];

Blockly.LANG_MID_LEVEL_FIND_CUBE_TOOLTIP_GENERAL = 'and return its position according to the camera. \n' +
                                         'TODO and THINK about: Return all cube positions and its framename in relation to base, arm, gripper';
Blockly.LANG_MID_LEVEL_FIND_CUBE_TOOLTIP_BLUE = 'Search for a blue cube';
Blockly.LANG_MID_LEVEL_FIND_CUBE_TOOLTIP_RED = 'Search for a red cube';
Blockly.LANG_MID_LEVEL_FIND_CUBE_TOOLTIP_GREEN = 'Search for a green cube';
Blockly.LANG_MID_LEVEL_FIND_CUBE_TOOLTIP_YELLOW = 'Search for a yellow cube';
Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_RED = 0;
Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_YELLOW = 60;
Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_GREEN = 120;
Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_BLUE = 180;

Blockly.LANG_MID_LEVEL_MOVE_BASE_OA_TITLE = 'move base (with obstacle avoidance)';
Blockly.LANG_MID_LEVEL_MOVE_BASE_OA_TOOLTIP = 'Use the navigation stack to move the robot to the specified coordinate and orientation. \n' +
                        'A true or false will be returned depending on success or fail';
                    
Blockly.LANG_MID_LEVEL_REFERENCE_FRAME_BASE = 'base';
Blockly.LANG_MID_LEVEL_REFERENCE_FRAME_ARM = 'arm';
Blockly.LANG_MID_LEVEL_REFERENCE_FRAME_GRIPPER = 'gripper';

Blockly.LANG_MID_LEVEL_IK_SOLVER_ANALYTICAL = 'analytical';
Blockly.LANG_MID_LEVEL_IK_SOLVER_GEOMETRICAL = 'geometrical';

// High level Blocks.
Blockly.LANG_HIGH_LEVEL_COLOUR = 180;

Blockly.LANG_HIGH_LEVEL_FROM_IMPORT_1_MSG = 'from';
Blockly.LANG_HIGH_LEVEL_FROM_IMPORT_2_MSG = 'import';

Blockly.LANG_HIGH_LEVEL_MOVE_ROBOT_TO_POSITION_PLATFORM1 = 'platform1';
Blockly.LANG_HIGH_LEVEL_MOVE_ROBOT_TO_POSITION_ENTRANCE = 'arena entrance';
Blockly.LANG_HIGH_LEVEL_MOVE_ROBOT_TO_POSITION_OTHERS   = 'others';

Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_ONTO = 'onto';
Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_LEFT = 'to the left of';
Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_RIGHT = 'to the right of';
Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_INFRONT = 'in front of';
Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_PLATFORM1 = 'platform1';
Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_ROBOT = 'robot plate';
Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_OTHERS = 'other objects';

Blockly.LANG_HIGH_LEVEL_WALL_FOLLOWER_LEFT  = 'left';
Blockly.LANG_HIGH_LEVEL_WALL_FOLLOWER_RIGHT = 'right';

// Are these values on the parameter server?
Blockly.MoveRobotBaseDirectionString = [
                          ['forward', ['0.1', '0', '0', '0', '0', '0']],
                          ['backward', ['-0.1', '0', '0', '0', '0', '0']],
                          ['left', ['0', '0.1', '0', '0', '0', '0']],
                          ['right', ['0', '-0.1', '0', '0', '0', '0']],
                          ['rotate_clockwise', ['0', '0', '0', '0', '0', '-0.1']],
                          ['rotate_anticlockwise', ['0', '0', '0', '0', '0', '0.1']]
                        ];

Blockly.LANG_HIGH_LEVEL_FIND_AND_GRASP_OBJECT_CUBE = 'cube';

// Action Blocks
Blockly.LANG_ACTION_COLOUR = 60;

// ROS message Blocks
Blockly.LANG_ROS_MESSAGE_COLOUR = 30;

// ROS node Blocks
Blockly.LANG_ROS_NODE_COLOUR = 30;

// Dummy category
Blockly.LANG_DUMMY_COLOUR = 330;

// Connection types
Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE = 'rosMessage';
Blockly.LANG_CONNECTION_TYPE_ROS_NODE = 'rosNode';
Blockly.LANG_CONNECTION_TYPE_FROM_IMPORT = 'FromImport';
Blockly.LANG_CONNECTION_TYPE_IMPORT = 'Import';
Blockly.LANG_CONNECTION_TYPE_TRANSFORM = 'Transform';
Blockly.LANG_CONNECTION_TYPE_ARM_JOINT_POSITION = 'ArmJointPosition';
Blockly.LANG_CONNECTION_TYPE_BASE_POSE_MSG = 'BasePoseMsg';
Blockly.LANG_CONNECTION_TYPE_SIMPLE_ACTION_CLIENT = 'SimpleActionClient';
Blockly.LANG_CONNECTION_TYPE_QUATERNION = 'Quaternion';

Blockly.EDUFILL_BOOL_OPERATORS_TRUE  = 'x'; 
Blockly.EDUFILL_BOOL_OPERATORS_FALSE = '_'; 

Blockly.EDUFILL_CHANGEMODE_OPERATORS_OPEN = 'open';
Blockly.EDUFILL_CHANGEMODE_OPERATORS_MINIMIZE = 'modified';
Blockly.EDUFILL_CHANGEMODE_OPERATORS_CLOSE = 'default';

// Additional entries
Blockly.EDUFILL_ROS_PKG_NAME = 'edufill_blockly';
