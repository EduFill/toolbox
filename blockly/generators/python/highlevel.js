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
 *  * @fileoverview Generating Python for high level blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
 Blockly.Python = Blockly.Generator.get('Python');
 
 if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.highlevel_application = function() {
    var roslib = 'roslib; roslib.load_manifest(\"' + this.pkg_name + '\")';
    roslib = roslib.replace(/[']/g,'');
    Blockly.Python.definitions_['import_'+ roslib] = 'import ' + roslib; 
    Blockly.Python.definitions_['import_rospy'] = 'import rospy';
    Blockly.Python.RESERVED_WORDS_ += 'roslib,rospy,';
    var branch0 = Blockly.Python.statementToCode(this, 'APP_STACK') || '  pass\n';
    var code = 'if __name__==\"__main__\":\n' +
               '  rospy.init_node(\'edufill_blockly_node\') \n\n' + //TODO if this.rospkg_ = 1 then show a separate statement and modify this line
               branch0;
    
    return code;
};

Blockly.Python.highlevel_move_base_to_goal = function() {
    var changeModeState = this.getTitleValue('STATE');
    var code;
    
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0;
    }
    else if (changeModeState == 'FALSE') {
        var goal_pose_string = 'CONFIG_PRE_DEF_POSITIONS_' + this.getTitleValue('MODE');
        Blockly.Python.definitions_['import_move_base_component'] = 'import move_base_component #/edufill_navigation/edufill_base_cmds/src/move_base_component.py';
        Blockly.Python.RESERVED_WORDS_ += 'move_base_component';
    
        var code = 'move_base_component.to_goal(\"' + Blockly[goal_pose_string] + '\")\n';
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    
    return code;
};

Blockly.Python.highlevel_move_gripper_string = function() {
    var changeModeState = this.getTitleValue('STATE');
    var code;
    
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0;
    }
    else if (changeModeState == 'FALSE') {
        var openclose = 'CONFIG_FINGER_' + this.getTitleValue('MODE')
        Blockly.Python.definitions_['import_move_gripper_component'] = 'import move_gripper_component  #/edufill_manipulation/edufill_arm_cmds/src/move_gripper_component.py'
        Blockly.Python.RESERVED_WORDS_ += 'move_gripper_component,';
    
        code = 'move_gripper_component.to_pose(\"' + Blockly[openclose] + '\")\n';
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    return code;
};

Blockly.Python.highlevel_move_arm_joint_string = function() {
    var changeModeState = this.getTitleValue('STATE');
    var code;
    
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0;
    }
    else if (changeModeState == 'FALSE') {
        var joint_string = "CONFIG_PRE_DEF_ARM_POS_" + this.getTitleValue('MODE');
        Blockly.Python.definitions_['import_move_arm_component'] = 'import move_arm_component  #/edufill_manipulation/edufill_arm_cmds/src/move_arm_component.py'
        Blockly.Python.RESERVED_WORDS_ += 'move_arm_component,';
    
        var code = 'move_arm_component.to_pose(\"' + Blockly[joint_string] + '\") \n';
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    return code;
};

Blockly.Python.highlevel_move_base_distance = function() {
    var code;
    var direction = "CONFIG_BASE_DIRECTION_" + this.getTitleValue('MODE');
    var distance  = Blockly.Python.valueToCode(this, 'DISTANCE', Blockly.Python.ORDER_NONE) || '0.1';
    code = 'TODO ' + Blockly[direction] + ' ' + distance;
    return code;
};

Blockly.Python.highlevel_move_base_to_direction = function() {
    var changeModeState = this.getTitleValue('STATE');
    var code;
    
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0;
    }
    else if (changeModeState == 'FALSE') {
        Blockly.Python.definitions_['import_move_base_component'] = 'import move_base_component #edufill_navigation/edufill_base_cmds/src/move_base_component.py';
        Blockly.Python.RESERVED_WORDS_ += 'move_base_component,';
        
        var direction = "CONFIG_BASE_DIRECTION_" + this.getTitleValue('MODE');
        var duration = Blockly.Python.valueToCode(this, 'DURATION', Blockly.Python.ORDER_NONE) || '2';
        var code = 'move_base_component.command(\"' + Blockly[direction] + '\", ' + duration + ')\n';
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    return code;
};

Blockly.Python.highlevel_move_arm_through_ik = function() {
    var ref = "/arm_link_0";
    
    var pose = Blockly.Python.valueToCode(this, 'POSE6D', Blockly.Python.ORDER_NONE) || 'NONE';
    var var_name = this.getTitleValue('VAR');
    
    var changeModeState = this.getTitleValue('STATE');
    var code;
    
    if (changeModeState == 'TRUE') {
        // procedure call 
        code = Blockly.Python.valueToCode(this, this.changemode.appendInput, Blockly.Python.ORDER_NONE);
    }
    else if (changeModeState == 'FALSE') {
        Blockly.Python.definitions_['import_move_arm_component'] = 'import move_arm_component  #/edufill_manipulation/edufill_arm_cmds/src/move_arm_component.py';
        Blockly.Python.RESERVED_WORDS_ += 'arm_kinematics,';
        var ik_solver = 'move_arm_component';
        code = ik_solver + '.to_cartesian_pose(' + pose + ', \"' + ref + '\") \n';
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.highlevel_find_cube = function() { 
    Blockly.Python.definitions_['import_detect_objects'] = 'import detect_objects'; 
    Blockly.Python.RESERVED_WORDS_ += 'detect_objects,';
    
    var colour  = "CONFIG_CUBE_" + this.getTitleValue('MODE');
    var code = 'detect_objects.cube(\''+ Blockly[colour] + '\')';
                
    return [code, Blockly.Python.ORDER_MEMBER];
};

Blockly.Python.highlevel_check_wall = function() {
    Blockly.Python.definitions_['import_read_laser_scan_component'] = 'import read_laser_scan_component'; 
    Blockly.Python.RESERVED_WORDS_ += 'read_laser_scan_component,';
    
    var changeModeState = this.getTitleValue('STATE');
    var code;
    
    if (changeModeState == 'TRUE') {
        code = Blockly.Python.valueToCode(this, this.changemode.appendInput, Blockly.Python.ORDER_NONE);
    }
    else if (changeModeState == 'FALSE') {
        var direction = 'CONFIG_WALL_' + this.getTitleValue('MODE')
        code = 'read_laser_scan_component.check_wall(\"' + Blockly[direction] + '\")'
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};
