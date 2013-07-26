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
 *  * @fileoverview Generating Python for mid level blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
Blockly.Python = Blockly.Generator.get('Python');
 
 if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.midlevel_move_gripper = function() {
    // Code generator for finger (gripper) movements (midlevel) and handling of next level (lowlevel)
    var changeModeState = this.getTitleValue('STATE');
    var code;
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0; 
    }
    else if (changeModeState == 'FALSE') {
        Blockly.Python.definitions_['import_move_gripper_component'] = 'import move_gripper_component #/edufill_manipulation/edufill_arm_cmds/src/move_gripper_component.py';
        Blockly.Python.RESERVED_WORDS_ += 'geometry_msg,Twist,';
        var gripperL = Blockly.Python.valueToCode(this,'GRIPPER_LEFT',Blockly.Python.ORDER_NONE) || 'None';
        var gripperR = Blockly.Python.valueToCode(this,'GRIPPER_RIGHT',Blockly.Python.ORDER_NONE) || 'None';
        code = 'move_gripper_component.to_joint_positions([' +  gripperL + ', ' + gripperR + '])\n'; 
    }
    
    return code;
};
 
Blockly.Python.midlevel_reference_frame = function() {
    // Code generator of reference frames (midlevel)
    var reference_name = this.getTitleValue('MODE');
    var code = Blockly['CONFIG_REF_FRAME_PY_' + reference_name];
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

// Complete python function, that is added to Blockly.Python.definitions_
// NOTE: There won't be a procedure block present on the workspace and 
//       a newly created procedure (by the user) may exists next to this 
//       procedure and might lead to an run-time error.
Blockly.GEN_MID_LEVEL_ROS_MOVE_BASE_TWIST_DEF = 
                'def ros_move_base_twist (lx, ly, lz, ax, ay, az):\n' +
                '  pub = rospy.Publisher(\"cmd_vel\", Twist) \n'+
                '\n'                    +
                '  twist = Twist() \n'  +
                '  # set velocities \n' +
                '  twist.linear.x = lx \n' + 
                '  twist.linear.y = ly \n' +
                '  twist.linear.z = lz \n' +
                '  twist.angular.x = ax \n' +
                '  twist.angular.y = ay \n' +
                '  twist.angular.z = az \n' +
                '  # publish the topic \n'+
                '  pub.publish(twist) \n'   +
                '\n';
 
Blockly.Python.midlevel_ros_move_base_twist = function() {
    // Code generator for ROS base movements w.r.t. twist (midlevel)
    Blockly.Python.definitions_['import_twist'] = 'from geometry_msgs.msg import Twist';
    Blockly.Python.RESERVED_WORDS_ += 'geometry_msg,Twist,';
     
    var lx = Blockly.Python.valueToCode(this, 'L_X', Blockly.Python.ORDER_NONE) || 0;
    var ly = Blockly.Python.valueToCode(this, 'L_Y', Blockly.Python.ORDER_NONE) || 0;
    var lz = Blockly.Python.valueToCode(this, 'L_Z', Blockly.Python.ORDER_NONE) || 0;
    var ax = Blockly.Python.valueToCode(this, 'A_X', Blockly.Python.ORDER_NONE) || 0;
    var ay = Blockly.Python.valueToCode(this, 'A_Y', Blockly.Python.ORDER_NONE) || 0;
    var az = Blockly.Python.valueToCode(this, 'A_Z', Blockly.Python.ORDER_NONE) || 0;
    
    Blockly.Python.definitions_['ros_move_base_twist'] = Blockly.GEN_MID_LEVEL_ROS_MOVE_BASE_TWIST_DEF
    
    var code = 'ros_move_base_twist(' + lx + ', ' + ly + ', ' + lz + ', ' + ax + ', ' + ay + ', ' + az + ') \n'
    return code;
};

Blockly.Python.midlevel_move_arm_joint_position = function() {
    // Code generator for moving the arm to a joint configuration (midlevel) and handling of next level (lowlevel)
    var changeModeState = this.getTitleValue('STATE');
    var code;
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0;
    }
    else if (changeModeState == 'FALSE') {
        var joint_array = Blockly.Python.valueToCode(this, this.getInputJnt, Blockly.Python.ORDER_MEMBER) || ['None'];
        Blockly.Python.definitions_['import_move_arm_component'] = 'import move_arm_component  #/edufill_manipulation/edufill_arm_cmds/src/move_arm_component.py'
        Blockly.Python.RESERVED_WORDS_ += 'move_arm_component,';
        code = 'move_arm_component.to_joint_positions(' + joint_array + ') \n'
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}

    return code;
};

Blockly.Python.midlevel_move_base_to_pose = function() {
    // Code generator for moving the base to a 6D pose (midlevel) and handling of next level (lowlevel)
    var changeModeState = this.getTitleValue('STATE');
    var code;
    if (changeModeState == 'TRUE') {
        var branch0 = Blockly.Python.statementToCode(this, this.changemode.appendInput);
        code = 'if True: # \'' + this.type + '\' block is open and the underlying blocks are visible/used. \n' + branch0; 
    }
    else if (changeModeState == 'FALSE') {
        Blockly.Python.definitions_['import_move_base_component'] = 'import move_base_component #/edufill_navigation/edufill_base_cmds/src/move_base_component.py';
        Blockly.Python.RESERVED_WORDS_ += 'move_base_component';
        
        var pose = Blockly.Python.valueToCode(this, this.getInputPose, Blockly.Python.ORDER_NONE) || '[None]';
        code =  'move_base_component.to_pose(' + pose + ')\n';
    }
    return code;
};

Blockly.Python.midlevel_move_base_relative = function() {
    // Code generator for relative base movements (midlevel)
    Blockly.Python.definitions_['import_move_base_component'] = 'import move_base_component #/edufill_navigation/edufill_base_cmds/src/move_base_component.py';
    Blockly.Python.RESERVED_WORDS_ += 'move_base_component';
        
    var pose = Blockly.Python.valueToCode(this, 'POSE', Blockly.Python.ORDER_NONE) || '[None]';
    var code = 'move_base_component.relative(' + pose + ')\n';
    
    return code;
};

Blockly.Python.midlevel_ik_checker = function() {
    // Code generator for testing whether a inverse kinematics solution exists (midlevel)
    Blockly.Python.definitions_['from_arm_kinematics_import_*'] = 'from arm_kinematics import *  #/edufill_manipulation/edufill_arm_cmds/src/arm_kinematics.py';
    Blockly.Python.RESERVED_WORDS_ += 'arm_kinematics,';
    var pose6D = Blockly.Python.valueToCode(this, 'POSE6D', Blockly.Python.ORDER_MEMBER) || ['None'];
    var frame_of_reference = Blockly.Python.valueToCode(this, 'REF', Blockly.Python.ORDER_NONE) || '' ;
    var ik_approach = Blockly['LANG_MID_LEVEL_IK_SOLVER_' + this.getTitleValue('MODE')];
    var ik_solver = ik_approach + '_solver';
    var code = 'KinematicsSolver(\'' + ik_approach + '\')' + '.check_ik_solver_has_solution(' + pose6D + ', str(' + frame_of_reference + '))';
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.midlevel_ik_solver = function() {
    // Code generator for actually calculating a inverse kinematics solution (midlevel)
    Blockly.Python.definitions_['from_arm_kinematics_import_*'] = 'from arm_kinematics import *  #/edufill_manipulation/edufill_arm_cmds/src/arm_kinematics.py';
    Blockly.Python.RESERVED_WORDS_ += 'arm_kinematics,';
    var pose6D = Blockly.Python.valueToCode(this, 'POSE6D', Blockly.Python.ORDER_MEMBER) || ['None'];
    var frame_of_reference = Blockly.Python.valueToCode(this, 'REF', Blockly.Python.ORDER_NONE) || '' ;
    var ik_approach = Blockly['LANG_MID_LEVEL_IK_SOLVER_' + this.getTitleValue('MODE')];
    var ik_solver = ik_approach + '_solver';
    var code = 'KinematicsSolver(\'' + ik_approach + '\')' + '.get_ik_solution(' + pose6D + ', str(' + frame_of_reference + '))';
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.midlevel_fk_solver = function() {
    // Code generator for calculating joint configurations from a given 6D pose (midlevel)
    Blockly.Python.definitions_['from_arm_kinematics_import_KinematicsSolver'] = 'from arm_kinematics import KinematicsSolver #/edufill_manipulation/edufill_arm_cmds/src/arm_kinematics.py';
    Blockly.Python.definitions_['\'' + this.variableName + '\''] = this.variableName + ' = KinematicsSolver()';
    var joint_angles = Blockly.Python.valueToCode(this, 'JOINTS', Blockly.Python.ORDER_MEMBER) || ['None'];
    var code = this.variableName + '.get_fk_solution(' + joint_angles + ')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.midlevel_mapping = function() {
    // Code generator for mapping (midlevel)
    Blockly.Python.definitions_['import_mapping_component'] = 'import mapping_component';
    Blockly.Python.RESERVED_WORDS_ += 'mapping_component,';
    var code = '';
    var dropdown = this.getTitleValue('MODE');
    if (dropdown == 'START') {
        code = 'mapping_component.start()\n';
    }
    else if (dropdown == 'STOP') {
        code = 'mapping_component.stop()\n';
    }
    else if (dropdown == 'STORE') {
        var file = Blockly.Python.valueToCode(this, 'FILE', Blockly.Python.ORDER_NONE) || '\'\'';
        code = 'mapping_component.store(' + file + ')\n';
    }
    
    return code;
};

Blockly.Python.midlevel_read_map_location = function() {
    // Code generator for reading the actual (mabe guessed) robot location (midlevel)
    Blockly.Python.definitions_['import_read_base_component'] = 'import read_base_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_base_component,';
    var code = 'read_base_component.location()';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.midlevel_check_wall = function() {
    // Code generator for testing the existence of a wall within a given range (midlevel) and handling of next level (lowlevel)
    Blockly.Python.definitions_['import_read_laser_scan_component'] = 'import read_laser_scan_component'; 
    Blockly.Python.RESERVED_WORDS_ += 'read_laser_scan_component,';
    
    var changeModeState = this.getTitleValue('STATE');
    var code;
    
    if (changeModeState == 'TRUE') {
        code = Blockly.Python.valueToCode(this, this.changemode.appendInput, Blockly.Python.ORDER_NONE);
    }
    else if (changeModeState == 'FALSE') {
        var direction = 'CONFIG_WALL_' + this.getTitleValue('MODE')
        var distance = Blockly.Python.valueToCode(this, 'DISTANCE', Blockly.Python.ORDER_NONE);
        code = 'read_laser_scan_component.check_wall_with_distance(\"' + Blockly[direction] + '\", ' + distance + ')'
    }
    else {throw 'user changed value to an undefined state ' + changeModeState}
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

