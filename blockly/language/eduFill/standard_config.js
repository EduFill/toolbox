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
 * @fileoverview config1 strings and dropdown entries added by the eduFill project.
 * @author marc.wollenweber@smail.inf.h-brs.de (Marc Wollenweber)
 */
 
'use strict';

goog.require('Blockly.Language');


Blockly.CONFIG_PRE_DEF_POSITIONS_S1 = 'S1';
Blockly.CONFIG_PRE_DEF_POSITIONS_S2 = 'S2';
Blockly.CONFIG_PRE_DEF_POSITIONS_S3 = 'S3';
Blockly.CONFIG_PRE_DEF_POSITIONS_D1 = 'D1';
Blockly.CONFIG_PRE_DEF_POSITIONS_EXIT = 'EXIT';
Blockly.CONFIG_PRE_DEF_POSITIONS = 
    [[Blockly.CONFIG_PRE_DEF_POSITIONS_S1, 'S1'],
     [Blockly.CONFIG_PRE_DEF_POSITIONS_S2, 'S2'],
     [Blockly.CONFIG_PRE_DEF_POSITIONS_S3, 'S3'],
     [Blockly.CONFIG_PRE_DEF_POSITIONS_D1, 'D1'],
     [Blockly.CONFIG_PRE_DEF_POSITIONS_EXIT, 'EXIT']];

Blockly.CONFIG_BASE_DIRECTION_FORWARDS = 'forward'
Blockly.CONFIG_BASE_DIRECTION_BACKWARDS = 'backward'
Blockly.CONFIG_BASE_DIRECTION_LEFT = 'left'
Blockly.CONFIG_BASE_DIRECTION_RIGHT = 'right'
Blockly.CONFIG_BASE_DIRECTION_CLOCK = 'rotate_clockwise'
Blockly.CONFIG_BASE_DIRECTION_COUNTERCLOCK = 'rotate_anticlockwise'
Blockly.CONFIG_BASE_DIRECTION_STOP = 'stop'
Blockly.CONFIG_BASE_DIRECTION =
    [[Blockly.CONFIG_BASE_DIRECTION_FORWARDS, 'FORWARDS'],
     [Blockly.CONFIG_BASE_DIRECTION_BACKWARDS, 'BACKWARDS'],
     [Blockly.CONFIG_BASE_DIRECTION_LEFT, 'LEFT'],
     [Blockly.CONFIG_BASE_DIRECTION_RIGHT, 'RIGHT'],
     [Blockly.CONFIG_BASE_DIRECTION_CLOCK, 'CLOCK'],
     [Blockly.CONFIG_BASE_DIRECTION_COUNTERCLOCK, 'COUNTERCLOCK']];

// Required to make the names available for some generator functions
Blockly.CONFIG_CUBE_BLUE = 'blue';
Blockly.CONFIG_CUBE_RED = 'red';
Blockly.CONFIG_CUBE_GREEN = 'green';
Blockly.CONFIG_CUBE_YELLOW = 'yellow';
Blockly.CONFIG_CUBE_CYAN = 'cyan';
Blockly.CONFIG_CUBE_MAGENTA = 'magenta';

// Blockly.CONFIG_CUBE_ has to be fixed and the suffix must be equal to the second entry!
Blockly.CONFIG_CUBE_COLORS = 
    [[Blockly.CONFIG_CUBE_BLUE, 'BLUE'],
     [Blockly.CONFIG_CUBE_RED, 'RED'],
     [Blockly.CONFIG_CUBE_GREEN, 'GREEN'],
     [Blockly.CONFIG_CUBE_YELLOW, 'YELLOW'],
     [Blockly.CONFIG_CUBE_CYAN, 'CYAN'],
     [Blockly.CONFIG_CUBE_MAGENTA, 'MAGENTA']];

Blockly.CONFIG_FINGER_OPEN = 'open';
Blockly.CONFIG_FINGER_CLOSE = 'close';
// CONFIG_FINGER_POS_ has to be fixed and the suffix must be equal to the second entry!
Blockly.CONFIG_FINGER_POS =
    [[Blockly.CONFIG_FINGER_OPEN, 'OPEN'],
     [Blockly.CONFIG_FINGER_CLOSE, 'CLOSE']];

Blockly.CONFIG_PRE_DEF_ARM_POS_INITPOSITION = 'initposition';
Blockly.CONFIG_PRE_DEF_ARM_POS_FLATPOSITION = 'flatposition';
Blockly.CONFIG_PRE_DEF_ARM_POS_ZEROPOSITION = 'zeroposition';
Blockly.CONFIG_PRE_DEF_ARM_POS_ARM_OUT_OF_VIEW = 'arm_out_of_view';
Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_LAYING = 'arm_pregrasp_laying';
Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_STANDING = 'arm_pregrasp_standing';
// CONFIG_PRE_DEF_ARM_POS_ has to be fixed and the suffix must be equal to the second entry!
Blockly.CONFIG_PRE_DEF_ARM_POS = 
    [[Blockly.CONFIG_PRE_DEF_ARM_POS_INITPOSITION, 'INITPOSITION'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_ZEROPOSITION, 'ZEROPOSITION'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_ARM_OUT_OF_VIEW, 'ARM_OUT_OF_VIEW'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_LAYING, 'PREGRASP_LAYING'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_STANDING, 'PREGRASP_STANDING']];

Blockly.CONFIG_WALL_LEFT = 'left';
Blockly.CONFIG_WALL_RIGHT = 'right';
Blockly.CONFIG_WALL_FRONT = 'front';
// CONFIG_WALL_ has to be fixed and the suffix must be equal to the second entry!     
Blockly.CONFIG_WALL = 
    [[Blockly.CONFIG_WALL_LEFT, 'LEFT'],
     [Blockly.CONFIG_WALL_RIGHT, 'RIGHT'],
     [Blockly.CONFIG_WALL_FRONT, 'FRONT']];

Blockly.CONFIG_WALL_DISTANCE = '0.4'
Blockly.CONFIG_WALL_ANGLE_LEFT  = '1.56'
Blockly.CONFIG_WALL_ANGLE_RIGHT = '-1.56'
Blockly.CONFIG_WALL_ANGLE_FRONT = '0.0'
        
Blockly.CONFIG_IK_SOLVER_ANALYTICAL = 'analytical';
Blockly.CONFIG_IK_SOLVER_GEOMETRICAL = 'geometrical';
// CONFIG_IK_SOLVER_ has to be fixed and the suffix must be equal to the second entry!        
Blockly.CONFIG_IK_SOLVER = 
    [[Blockly.CONFIG_IK_SOLVER_ANALYTICAL, 'ANALYTICAL'],
    [Blockly.CONFIG_IK_SOLVER_GEOMETRICAL, 'GEOMETRICAL']];


Blockly.CONFIG_REF_FRAME_BASE = 'base';
Blockly.CONFIG_REF_FRAME_ARM = 'arm';
Blockly.CONFIG_REF_FRAME_GRIPPER = 'finger';
Blockly.CONFIG_REF_FRAME_PY_BASE = '\"/base_link\"';
Blockly.CONFIG_REF_FRAME_PY_ARM = '\"/arm_link_0\"';
Blockly.CONFIG_REF_FRAME_PY_GRIPPER = '\"/arm_link_5\"';
// CONFIG_REF_FRAME_ has to be fixed and the suffix must be equal to the second entry!
Blockly.CONFIG_REF_FRAME = 
    [[Blockly.CONFIG_REF_FRAME_BASE, 'BASE'],
     [Blockly.CONFIG_REF_FRAME_ARM, 'ARM'],
     [Blockly.CONFIG_REF_FRAME_GRIPPER, 'GRIPPER']];

// Parameter for parameter server 
//Blockly.CONFIG_PARAMETER_GLOBAL = '\"/global_param\"';
//Blockly.CONFIG_PARAMETER_RELATIVE = '\"relative_param\"';
//Blockly.CONFIG_PARAMETER_PRIVATE = '\'~private_param\'';
//Blockly.CONFIG_PARAMETER_DEFAULT = '\'default_param\'';
//
Blockly.CONFIG_PARAMETER_OPEN = '\'/script_server/gripper/open\'';
Blockly.CONFIG_PARAMETER_CLOSE = '\'/script_server/gripper/close\'';
//
Blockly.CONFIG_PARAMETER_INITPOSITION = '\'/script_server/arm/initposition\'';
Blockly.CONFIG_PARAMETER_ZEROPOSITION = '\'/script_server/arm/zeroposition\'';
Blockly.CONFIG_PARAMETER_PREGRASP_LAYING = '\'/script_server/arm/arm_pregrasp_laying\'';
Blockly.CONFIG_PARAMETER_PREGRASP_STANDING = '\'/script_server/arm/arm_pregrasp_standing\'';
Blockly.CONFIG_PARAMETER_ARM_OUT_OF_VIEW = '\'/script_server/arm/arm_out_of_view\'';
//
Blockly.CONFIG_PARAMETER_S1 = '\'/script_server/base/S1\'';
Blockly.CONFIG_PARAMETER_S2 = '\'/script_server/base/S2\'';
Blockly.CONFIG_PARAMETER_S3 = '\'/script_server/base/S3\'';
Blockly.CONFIG_PARAMETER_D1 = '\'/script_server/base/D1\'';
Blockly.CONFIG_PARAMETER_EXIT = '\'/script_server/base/EXIT\'';
//
Blockly.CONFIG_PARAMETER_NUMBER_ARM_JOINTS = '\'number_of_arm_joints\'';
// CONFIG_PARAMETER_ has to be fixed and the suffix must be equal to the second entry!
Blockly.CONFIG_PARAMETER_SERVER = 
    [
     [Blockly.CONFIG_PARAMETER_OPEN, 'OPEN'],
     [Blockly.CONFIG_PARAMETER_CLOSE, 'CLOSE'],
     [Blockly.CONFIG_PARAMETER_INITPOSITION, 'INITPOSITION'],
     [Blockly.CONFIG_PARAMETER_ZEROPOSITION, 'ZEROPOSITION'],
     [Blockly.CONFIG_PARAMETER_PREGRASP_LAYING, 'PREGRASP_LAYING'],
     [Blockly.CONFIG_PARAMETER_PREGRASP_STANDING, 'PREGRASP_STANDING'],
     [Blockly.CONFIG_PARAMETER_ARM_OUT_OF_VIEW, 'ARM_OUT_OF_VIEW'],
     [Blockly.CONFIG_PARAMETER_S1, 'S1'],
     [Blockly.CONFIG_PARAMETER_S2, 'S2'],
     [Blockly.CONFIG_PARAMETER_S3, 'S3'],
     [Blockly.CONFIG_PARAMETER_D1, 'D1'],
     [Blockly.CONFIG_PARAMETER_EXIT, 'EXIT'],
     [Blockly.CONFIG_PARAMETER_NUMBER_ARM_JOINTS, 'NUMBER_ARM_JOINTS']
     //[Blockly.CONFIG_PARAMETER_GLOBAL, 'GLOBAL'],
     //[Blockly.CONFIG_PARAMETER_RELATIVE, 'RELATIVE'],
     //[Blockly.CONFIG_PARAMETER_PRIVATE, 'PRIVATE'],
     //[Blockly.CONFIG_PARAMETER_DEFAULT, 'DEFAULT']
    ];
    

//
// The entries below do not need to be changed and partially shouldn't be modified at all!
//

Blockly.ChangeModeTooltip = 'ChangeMode (checkbox): Activate the Changemode whenever a deeper view into the blocks functioning is required.';

// Block colour
Blockly.LANG_LOW_LEVEL_COLOUR = 0;
Blockly.LANG_ROS_MESSAGE_COLOUR = 30;
Blockly.LANG_ROS_NODE_COLOUR = 30;
Blockly.LANG_ACTION_COLOUR = 60;
Blockly.LANG_MID_LEVEL_COLOUR = 90;
Blockly.LANG_HIGH_LEVEL_COLOUR = 180;
Blockly.LANG_UNITS_COLOUR = 190;
Blockly.LANG_DUMMY_COLOUR = 330;

// Low level Blocks.
Blockly.LANG_LOW_LEVEL_ROS_INIT_NODE_EMPTY = ''
Blockly.LANG_LOW_LEVEL_ROS_INIT_NODE_TRUE = 'true';
Blockly.LANG_LOW_LEVEL_ROS_INIT_NODE_FALSE = 'false';

Blockly.LANG_LOW_LEVEL_IMPORT_MSG = 'import';
Blockly.LANG_LOW_LEVEL_FROM_IMPORT_MSG = 'from x import';
Blockly.LANG_LOW_LEVEL_MAIN_FRAME = 'import frame';

// Please do not modify! Directly related to ROS commands
Blockly.LANG_LOW_LEVEL_ROS_LOG_EMPTY = '';
Blockly.LANG_LOW_LEVEL_ROS_LOG_DEBUG = 'debug';
Blockly.LANG_LOW_LEVEL_ROS_LOG_INFO = 'info';
Blockly.LANG_LOW_LEVEL_ROS_LOG_WARN = 'warn';
Blockly.LANG_LOW_LEVEL_ROS_LOG_ERROR = 'err';
Blockly.LANG_LOW_LEVEL_ROS_LOG_FATAL = 'fatal';

// Mid level Blocks.
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

// High level Blocks.
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

Blockly.MoveRobotBaseDirectionString = [
                          ['forward', ['0.1', '0', '0', '0', '0', '0']],
                          ['backward', ['-0.1', '0', '0', '0', '0', '0']],
                          ['left', ['0', '0.1', '0', '0', '0', '0']],
                          ['right', ['0', '-0.1', '0', '0', '0', '0']],
                          ['rotate_clockwise', ['0', '0', '0', '0', '0', '-0.1']],
                          ['rotate_anticlockwise', ['0', '0', '0', '0', '0', '0.1']]
                        ];

Blockly.LANG_HIGH_LEVEL_FIND_AND_GRASP_OBJECT_CUBE = 'cube';

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

// Additional entries
Blockly.EDUFILL_ROS_PKG_NAME = 'edufill_blockly';
