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
 *  * @fileoverview Generating Python for ROS message blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
 Blockly.Python = Blockly.Generator.get('Python');
 
if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.ros_string_msg_type = function() {
    Blockly.Python.definitions_['from_std_msgs.msg_import_String'] = 'from std_msgs.msg import String';
    Blockly.Python.RESERVED_WORDS_ += 'std_msgs.std,String,';
    var code = 'String';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.ros_twist_msg_type = function() {
    Blockly.Python.definitions_['from_geometry_msgs.msg_import_Twist'] = 'from geometry_msgs.msg import Twist';
    Blockly.Python.RESERVED_WORDS_ += 'geometry_msgs.msg,Twist,';
    var code = 'Twist';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
}

Blockly.Python.ros_twist_msg = function() {
    Blockly.Python.definitions_['from_geometry_msgs.msg_import_Twist'] = 'from geometry_msgs.msg import Twist';
    Blockly.Python.RESERVED_WORDS_ += 'geometry_msgs.msg,Twist,';
    var code = 'Twist()';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.ros_jointstate_msg_type = function() {
    Blockly.Python.definitions_['from_sensor_msgs.msg_import_JointState'] = 'from sensor_msgs.msg import JointState';
    Blockly.Python.RESERVED_WORDS_ += 'sensor_msgs,msg,JointState,';
    var code = 'JointState';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
}

Blockly.Python.ros_jointstate_msg = function() {
    Blockly.Python.definitions_['from_sensor_msgs.msg_import_JointState'] = 'from sensor_msgs.msg import JointState';
    Blockly.Python.RESERVED_WORDS_ += 'sensor_msgs,msg,JointState,';
    var code = 'JointState()';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.ros_get_twist_element = function() {
    var variable = Blockly.Python.valueToCode(this, 'VARIABLE', Blockly.Python.ORDER_NONE) || 'None';
    var lin_ang = this.getTitleValue('MODE1');
    var xyz = this.getTitleValue('MODE2');
    var code = variable + '.' + Blockly[lin_ang] + '.' + Blockly[xyz];
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.ros_set_twist_element = function() {
    var variable = Blockly.Python.valueToCode(this, 'VARIABLE', Blockly.Python.ORDER_NONE) || 'None';
    var newVal = Blockly.Python.valueToCode(this, 'NEW_VAL', Blockly.Python.ORDER_NONE) || 'None';
    var lin_ang = this.getTitleValue('MODE1');
    var xyz = this.getTitleValue('MODE2');
    var code = variable + '.' + Blockly[lin_ang] + '.' + Blockly[xyz] + ' = ' + newVal + '\n';
    
    return code;
};

Blockly.Python.ros_jointpositions_msg_type = function() {
    var code = 'JointPositions';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
}

Blockly.Python.ros_other_msg_type = function() {
    var code = this.getTitleValue('MSG_NAME');
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.rosmessage_base_pose_msg = function() {
    Blockly.Python.definitions_['from_geometry_msgs.msg_import_*'] = 'from geometry_msgs.msg import *';
    Blockly.Python.RESERVED_WORDS_ += 'geometry_msgs.msg,';
    
    Blockly.Python.definitions_['base_pose_msg'] = 
        'def base_pose_msg(x, y, z, q):\n' +
        '  pose = PoseStamped()\n' +
        '  pose.header.stamp = rospy.Time.now()\n' +
        '  pose.header.frame_id = "/map"\n' +
        '  pose.pose.position.x = x \n' +
        '  pose.pose.position.y = y\n' +
        '  pose.pose.position.z = 0.0\n' +
        '  pose.pose.orientation.x = q[0]\n' +
        '  pose.pose.orientation.y = q[1]\n' +
        '  pose.pose.orientation.z = q[2]\n' +
        '  pose.pose.orientation.w = q[3]\n\n' +
        '  return pose\n';
    
    var  x = Blockly.Python.valueToCode(this, 'X', Blockly.Python.ORDER_NONE) || '0';
    var  y = Blockly.Python.valueToCode(this, 'Y', Blockly.Python.ORDER_NONE) || '0';
    var  z = Blockly.Python.valueToCode(this, 'Z', Blockly.Python.ORDER_NONE) || '0';
    var  q = Blockly.Python.valueToCode(this, 'Q', Blockly.Python.ORDER_NONE);
    
    var code = 'base_pose_msg(' + x + ', ' + y + ', ' + z + ', ' + q +')';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.rosmessage_gripper_position_msg = function() {
    Blockly.Python.definitions_['import_rospy'] = 'import rospy';
    Blockly.Python.definitions_['import_brics_actuator.msg'] = 'import brics_actuator.msg';
    Blockly.Python.definitions_['from_brics_actuator.msg_import_JointPositions_JointValue_Poison'] = 'from brics_actuator.msg import JointPositions, JointValue, Poison';
    Blockly.Python.RESERVED_WORDS_ += 'rospy,brics_actuator.msg,JointPositions,JointValue,Poison,';
    
    Blockly.Python.definitions_['def_create_gripper_msg'] = 
            'def create_gripper_msg (left_gripper, right_gripper): \n' +
            '  jp = JointPositions() \n' +
            '  jv1 = JointValue()\n' +
            '  jv1.joint_uri = "gripper_finger_joint_l"\n' +
            '  jv1.value = left_gripper\n' +
            '  jv1.unit = "m"\n' +
            '  jv2 = JointValue()\n' +
            '  jv2.joint_uri = "gripper_finger_joint_r"\n' +
            '  jv2.value = right_gripper\n' +
            '  jv2.unit = "m"\n' +
            '  p = Poison()\n' +
            '  jp.poisonStamp = p\n\n' +    
            '  jp.positions = [jv1, jv2]\n\n' +
            '  return jp\n';
    
    var left_gripper  = Blockly.Python.valueToCode(this, 'GRIPPER_L', Blockly.Python.ORDER_NONE);
    var right_gripper = Blockly.Python.valueToCode(this, 'GRIPPER_R', Blockly.Python.ORDER_NONE);
    var code = 'create_gripper_msg(' + left_gripper + ', ' + right_gripper + ')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.rosmessage_joint_state_msg_name = function() {
    var prefix = Blockly.Python.valueToCode(this, 'PREFIX', Blockly.Python.ORDER_NONE) || '\'\'';
    var numberOfJoints = Blockly.Python.valueToCode(this, 'NUMBER_OF_JOINTS', Blockly.Python.ORDER_NONE);
    var variable = this.getTitleValue('VAR');
    var code = variable + '.name = [' + prefix + '+str(i+1) for i in range(' + numberOfJoints + ')]\n';
    
    return code;
};

Blockly.Python.rosmessage_joint_state_msg_position = function() {
    var list = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE);
    var numberOfJoints = Blockly.Python.valueToCode(this, 'NUMBER_OF_JOINTS', Blockly.Python.ORDER_NONE);
    var variable = this.getTitleValue('VAR');
    var code = variable + '.position = [' + list + '[i] for i in range(' + numberOfJoints + ')]\n';
    
    return code;
};

Blockly.Python.rosmessage_joint_state_msg_velocity = function() {
    var list = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE);
    var numberOfJoints = Blockly.Python.valueToCode(this, 'NUMBER_OF_JOINTS', Blockly.Python.ORDER_NONE);
    var variable = this.getTitleValue('VAR');
    var code = variable + '.velocity = [' + list + '[i] for i in range(' + numberOfJoints + ')]\n';
    
    return code;
};

Blockly.Python.rosmessage_joint_state_msg_effort = function() {
    var list = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE);
    var numberOfJoints = Blockly.Python.valueToCode(this, 'NUMBER_OF_JOINTS', Blockly.Python.ORDER_NONE);
    var variable = this.getTitleValue('VAR');
    var code = variable + '.effort = [' + list + '[i] for i in range(' + numberOfJoints + ')]\n';
    
    return code;
};

Blockly.Python.rosmessage_arm_joint_position_msg = function() {
    var listLength = this.jointCount_;
    Blockly.Python.definitions_['import_rospy'] = 'import rospy';
    Blockly.Python.definitions_['import_brics_actuator.msg'] = 'import brics_actuator.msg';
    Blockly.Python.definitions_['from_brics_actuator.msg_import_JointPositions_JointValue_Poison'] = 'from brics_actuator.msg import JointPositions, JointValue, Poison';
    Blockly.Python.RESERVED_WORDS_ += 'rospy,brics_actuator.msg,JointPositions,JointValue,Poison,';
    
    var msg = '';
    var list = '';
    for (var i=0; i<listLength; i++) {
        msg += '  jv' + i + ' = JointValue()\n' +
               '  jv' + i + '.joint_uri = jointNameList[' + i +']\n' +
               '  jv' + i + '.value = jointValueList[' + i + ']\n' +
               '  jv' + i + '.unit = "rad"\n\n';
        list +='jv' + i + ((i != listLength-1) ? ',' : '');
    }
    Blockly.Python.definitions_['def_create_joint_position_msg']   = 
            'def create_joint_position_msg (jointNameList, jointValueList):\n' +
            '  jp = JointPositions()\n\n' +
            '  for i in jointNameList:\n' +
            '    jv[i] = JointValue()\n' +
            '    jv[i].joint_uri = jointNameList[i]\n' +
            '    jv[i].value = jointValueList[i]\n' +
            '    jv[i].unit = "rad"\n\n' +
            '  \n' +
            '  p = Poison() \n' +
            '  jp.poisonStamp = p\n' +
            '  jp.positions = jv\n\n' +
            '  return jp.positions\n';
    
    var jointNameList = [];
    var jointValueList = [];
    for (var i=0; i < listLength; i++) {
        var jointname = this.getTitleValue('JOINTNAME' + i);
        var jointvalue = Blockly.Python.valueToCode(this, 'JOINTVALUE' + i, Blockly.Python.ORDER_NONE) || 'None';
        jointNameList.push('\'' + jointname + '\''); 
        jointValueList.push(jointvalue);
    }
    var code = 'create_joint_position_msg([' + jointNameList + '],[' + jointValueList + '])';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};
