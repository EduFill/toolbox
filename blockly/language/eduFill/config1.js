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

// Required to make the names available for the generator functions
Blockly.CONFIG_CUBE_BLUE = 'blue';
Blockly.CONFIG_CUBE_RED = 'red';
Blockly.CONFIG_CUBE_GREEN = 'green';
Blockly.CONFIG_CUBE_YELLOW = 'yellow';
Blockly.CONFIG_CUBE_CYAN = 'cyan';
Blockly.CONFIG_CUBE_MAGENTA = 'magenta';

// Blockly.CONFIG_CUBE_ is fixed and the color of the cube has to be equal to the string on the right side
Blockly.CONFIG_CUBE_COLORS = 
    [[Blockly.CONFIG_CUBE_BLUE, 'BLUE'],
     [Blockly.CONFIG_CUBE_RED, 'RED'],
     [Blockly.CONFIG_CUBE_GREEN, 'GREEN'],
     [Blockly.CONFIG_CUBE_YELLOW, 'YELLOW'],
     [Blockly.CONFIG_CUBE_CYAN, 'CYAN'],
     [Blockly.CONFIG_CUBE_MAGENTA, 'MAGENTA']];

Blockly.CONFIG_FINGER_OPEN = 'open';
Blockly.CONFIG_FINGER_CLOSE = 'close';

Blockly.CONFIG_FINGER_POS =
    [[Blockly.CONFIG_FINGER_OPEN, 'OPEN'],
     [Blockly.CONFIG_FINGER_CLOSE, 'CLOSE']];

Blockly.CONFIG_PRE_DEF_ARM_POS_INITPOSITION = 'initposition';
Blockly.CONFIG_PRE_DEF_ARM_POS_FLATPOSITION = 'flatposition';
Blockly.CONFIG_PRE_DEF_ARM_POS_ZEROPOSITION = 'zeroposition';
Blockly.CONFIG_PRE_DEF_ARM_POS_ARM_OUT_OF_VIEW = 'arm_out_of_view';
Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_LAYING = 'arm_pregrasp_laying';
Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_STANDING = 'arm_pregrasp_standing';

Blockly.CONFIG_PRE_DEF_ARM_POS = 
    [[Blockly.CONFIG_PRE_DEF_ARM_POS_INITPOSITION, 'INITPOSITION'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_ZEROPOSITION, 'ZEROPOSITION'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_ARM_OUT_OF_VIEW, 'ARM_OUT_OF_VIEW'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_LAYING, 'PREGRASP_LAYING'],
     [Blockly.CONFIG_PRE_DEF_ARM_POS_PREGRASP_STANDING, 'PREGRASP_STANDING']];

Blockly.CONFIG_WALL_LEFT = 'left';
Blockly.CONFIG_WALL_RIGHT = 'right';
Blockly.CONFIG_WALL_FRONT = 'front';
     
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
        
Blockly.CONFIG_IK_SOLVER = 
    [[Blockly.CONFIG_IK_SOLVER_ANALYTICAL, 'ANALYTICAL'],
    [Blockly.CONFIG_IK_SOLVER_GEOMETRICAL, 'GEOMETRICAL']];

/* 
 * "/global_name"
 * "relative_name"
 * '~private_name'
 * 'default_param'
*/

Blockly.CONFIG_PARAMETER_OPEN = '\'/script_server/gripper/open\'';
Blockly.CONFIG_PARAMETER_CLOSE = '\'/script_server/gripper/close\'';

Blockly.CONFIG_PARAMETER_INITPOSITION = '\'/script_server/arm/initposition\'';
Blockly.CONFIG_PARAMETER_ZEROPOSITION = '\'/script_server/arm/zeroposition\'';
Blockly.CONFIG_PARAMETER_PREGRASP_LAYING = '\'/script_server/arm/arm_pregrasp_laying\'';
Blockly.CONFIG_PARAMETER_PREGRASP_STANDING = '\'/script_server/arm/arm_pregrasp_standing\'';
Blockly.CONFIG_PARAMETER_ARM_OUT_OF_VIEW = '\'/script_server/arm/arm_out_of_view\'';

Blockly.CONFIG_PARAMETER_S1 = '\'/script_server/base/S1\'';
Blockly.CONFIG_PARAMETER_S2 = '\'/script_server/base/S2\'';
Blockly.CONFIG_PARAMETER_S3 = '\'/script_server/base/S3\'';
Blockly.CONFIG_PARAMETER_D1 = '\'/script_server/base/D1\'';
Blockly.CONFIG_PARAMETER_EXIT = '\'/script_server/base/EXIT\'';
//Blockly.CONFIG_PARAMETER_GLOBAL = '\"/global_param\"';
//Blockly.CONFIG_PARAMETER_RELATIVE = '\"relative_param\"';
//Blockly.CONFIG_PARAMETER_PRIVATE = '\'~private_param\'';
//Blockly.CONFIG_PARAMETER_DEFAULT = '\'default_param\'';

Blockly.CONFIG_PARAMETER_NUMBER_ARM_JOINTS = '\'number_of_arm_joints\'';

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
