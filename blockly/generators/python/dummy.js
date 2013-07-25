/**
 * Visual Blocks Language
 *
 * Copyright 2012 ECHORD EduFill
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
 *  * @fileoverview Generating Python for dummy blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
 Blockly.Python = Blockly.Generator.get('Python');
 
 if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.test = function() {
    var changeModeState = this.getTitleValue('STATE');
    var code;
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput1);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0;
    }
    else if (changeModeState == 'FALSE') {
        code = 'dummyFunction(closesBlock) \n'; 
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    return code;
};

Blockly.Python.output_test = function() {
    var code = 'no code required'; 
    return [code, Blockly.Python.ORDER_ATOMIC];
}

Blockly.Python.store_in_db = function() {
    var input = Blockly.Python.valueToCode(this, 'STORE_VALUE', Blockly.Python.ORDER_NONE) || "nothing";
    var code = 'print \"store: ' + input + '\" \n';
    
    return code;
};

Blockly.Python.load_from_db = function() {
    var code = 'load value';
    
    return [code,Blockly.Python.ORDER_NONE];
};

Blockly.Python.move_robot_to_cube = function() {
    Blockly.Python.definitions_['list_of_transforms'] = 'list_of_transforms = None';
    Blockly.Python.definitions_['def_find_cube']            = Blockly.GEN_MID_LEVEL_FIND_CUBE_DEF;
    Blockly.Python.definitions_['def_move_robot_to_cube']   = Blockly.GEN_HIGH_LEVEL_MOVE_ROBOT_TO_CUBE_DEF;
    var cube_color  = this.getTitleValue('MODE');
    var code = 'move_robot_to_cube(\"' + cube_color  + '\") \n';
    return code;
};

Blockly.Python.move_robot_to_position = function() {
    var position = "LANG_HIGH_LEVEL_PLACE_CUBE_" + this.getTitleValue('MODE');
    return 'print \"Move robot to ' + Blockly[position] + '\" \n';
};

Blockly.Python.turn_robot = function() {
    var degree = Blockly.Python.valueToCode(this, 'DEGREE', Blockly.Python.ORDER_NONE) || 0;
    var code = 'print \"Turn robot around \" + ' + degree + ' + \" degree.\" \n';
    
    return code;
};

Blockly.Python.grasp_cube = function() {
    Blockly.Python.definitions_['list_of_transforms'] = 'list_of_transforms = None';
    Blockly.Python.definitions_['import_brics_actuator_msg'] = 'import brics_actuator.msg';
    Blockly.Python.definitions_['from_brics_actuator.msg_import_JointPosition_JointValue_Poison'] = 'from brics_actuator.msg import JointPositions, JointValue, Poison';
    Blockly.Python.RESERVED_WORDS_ += 'brics_actuator,brics,JointPositions,JointValue,Poison,';
    
    var cube_color  = this.getTitleValue('MODE');
    var code = 'grasp_cube(\"' + cube_color  + '\") \n';
    
    return code;
};

Blockly.Python.place_cube = function() {
    var destination = "LANG_HIGH_LEVEL_PLACE_CUBE_" + this.getTitleValue('MODE_DEST');
    var reference = "LANG_HIGH_LEVEL_PLACE_CUBE_" + this.getTitleValue('MODE_REF');
    return 'print \"Place grasped cube ' + Blockly[destination] + ' ' + Blockly[reference] + '\" \n';
};

Blockly.Python.wall_follower = function() {
    var wall = "LANG_HIGH_LEVEL_WALL_FOLLOWER_" + this.getTitleValue('MODE');
    
    return 'print \"Follow the ' + Blockly[wall] + ' wall until ...\" \n'
};

Blockly.Python.fetch_and_carry = function() {
    var cube_color  = "LANG_FIND_CUBE_" + this.getTitleValue('MODE_COL');
    var destination = "LANG_HIGH_LEVEL_PLACE_CUBE_" + this.getTitleValue('MODE_DEST');
    var reference   = "LANG_HIGH_LEVEL_PLACE_CUBE_" + this.getTitleValue('MODE_REF');
    var code =  'print \"Move robot to init position.\" \n' +
                'print \"Search for a ' + Blockly[cube_color] + ' cube.\" \n' +
                'print \"Move to the ' + Blockly[cube_color] + ' cube and grasp it.\" \n' +
                'print \"Put it ' + Blockly[destination] + ' ' + Blockly[reference] +'.\" \n';
    
    return code;
};

Blockly.Python.tftransform = function() {
    var obj_frame = Blockly.Python.valueToCode(this, 'OBJ_FRAME', Blockly.Python.ORDER_NONE) || "/undefined_string";
    var ref_frame = Blockly.Python.valueToCode(this, 'REF_FRAME', Blockly.Python.ORDER_NONE) || "/undefined_string";
    var duration = Blockly.Python.valueToCode(this, 'DURATION', Blockly.Python.ORDER_NONE) || 2;
    var code =  'print \"Get the transformation from object frame \" + ' + obj_frame + 
                ' + \" to reference frame \" + ' + ref_frame + 
                '+ \" and search for maximal \" + ' + duration + ' + \" seconds\" \n';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.base_placement = function() {
    Blockly.Python.definitions_['import_rospy'] = 'import rospy';
    Blockly.Python.definitions_['import_geometry_msgs_msg'] = 'import geometry_msgs.msg';
    Blockly.Python.definitions_['import_raw_srvs_srv'] = 'import raw_srvs.srv';
    Blockly.Python.definitions_['import_std_srvs_srv'] = 'import std_srvs.srv';
    Blockly.Python.definitions_['import_tf'] = 'import tf';
    Blockly.Python.definitions_['from_simple_script_server_import_*'] = 'from simple_script_server import *';
    Blockly.Python.RESERVED_WORDS_ += 'rospy,geometry_msgs,raw_srvs,std_srvs,tf,simple_script_server,';
    
    Blockly.Python.definitions_['def_base_placement'] = Blockly.GEN_HIGH_LEVEL_BASE_PLACEMENT_DEF;
    var code = 'base_placement() \n';
    
    return code
};

Blockly.Python.find_and_grasp_object = function() {
    Blockly.Python.definitions_['import_find_and_grasp_component'] = 'import find_and_grasp_component  #/edufill_scenario/src/find_and_grasp_component.py'
    Blockly.Python.RESERVED_WORDS_ += 'find_and_grasp_component,';
    
    var code = 'find_and_grasp_component() \n';
    
    return code;
}

Blockly.Python.detect_and_reach_object = function() {
    var object = 'LANG_HIGH_LEVEL_FIND_AND_GRASP_OBJECT_' + this.getTitleValue('MODE'); //value is defined by find_and_grasp_object block
    var code = 'print \"detect and reach ' + Blockly[object] + '\"\n';
    
    return code;
};

Blockly.Python.highlevel_move_base_distance = function() {
    var code;
    var direction = "CONFIG_BASE_DIRECTION_" + this.getTitleValue('MODE');
    var distance  = Blockly.Python.valueToCode(this, 'DISTANCE', Blockly.Python.ORDER_NONE) || '0.1';
    code = 'TODO ' + Blockly[direction] + ' ' + distance;
    return code;
};
