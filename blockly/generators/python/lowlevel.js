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
 *  * @fileoverview Generating Python for low level blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
Blockly.Python = Blockly.Generator.get('Python');
 
if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.lowlevel_package_main = function() {
    
    for (var i = 0;  i < this.importCount_; i++) {
        var bool = 0;
        var import_str = Blockly.Python.valueToCode(this, 'IMPORT' + i, Blockly.Python.ORDER_NONE) || (bool = 1);
        if (bool == 0 && import_str.length > 2) {
            import_str = import_str.replace(/[']/g,''); // find '. Replace it (delete it)
            Blockly.Python.definitions_['import_' + import_str] = 'import ' + import_str;
            Blockly.Python.RESERVED_WORDS_ += import_str + ',';
        }
    }
    
    for (var i = 0; i < this.from_importCount_; i++) {
        var bool = 0;
        var from_import = Blockly.Python.valueToCode(this, 'FROM_IMPORT' + i, Blockly.Python.ORDER_NONE) || (bool = 1);
        if (bool == 0) {
            Blockly.Python.definitions_[from_import] = from_import;
        }
    }
    
    var ros_package_name = this.getTitleValue('ROSPaName');
    Blockly.Python.definitions_['import_roslib'] = 'import roslib; roslib.load_manifest(\"'+ ros_package_name +'\")'; 
    Blockly.Python.definitions_['import_rospy'] = 'import rospy';
    Blockly.Python.RESERVED_WORDS_ += 'roslib,rospy,';
    var branch0 = Blockly.Python.statementToCode(this, 'MAINSTACK') || '  pass\n';
    var code = 'if __name__==\"__main__\":\n' + branch0;
    
    return code;
};

Blockly.Python.lowlevel_from_x_import = function() {
    var from_ = this.getTitleValue('FROM');
    var import_ = this.getTitleValue('IMPORT') || '*';
    var code = '';
    if (from_) {
        Blockly.Python.RESERVED_WORDS_ += from_ + ',' + import_ + ',';
        var code = 'from ' + from_ + ' import ' + import_; 
    }
    
    return [code,Blockly.Python.ORDER_MEMBER];
};

Blockly.Python.lowlevel_create_publisher = function() {
    var node = Blockly.Python.valueToCode(this ,'NODE', Blockly.Python.ORDER_NONE) || '';
    node = node.replace(/[']/g,''); // find '. Replace it (delete it)
    var variable = this.getTitleValue('VAR') || 'pub';
    var msg_type = Blockly.Python.valueToCode(this,'TYPE',Blockly.Python.ORDER_NONE) || '';
    msg_type = msg_type.replace(/['"]/g,''); // find ' and " and replace it by '' (delete it)
    var code = variable + ' = rospy.Publisher(\'' + node + '\', ' + msg_type + ') \n';
    
    return code;
};

Blockly.Python.lowlevel_publish = function() {
    var obj_name = this.getTitleValue('VAR') || undefined;
    var msg = Blockly.Python.valueToCode(this, 'MSG', Blockly.Python.ORDER_NONE) || '';
    var code = obj_name + '.publish('+ msg + ')\n';
    
    return code;
};

Blockly.Python.lowlevel_subscriber = function() {
    var topic = Blockly.Python.valueToCode(this,'NODE',Blockly.Python.ORDER_NONE) || '';
    var msg_type = Blockly.Python.valueToCode(this,'TYPE',Blockly.Python.ORDER_NONE) || '';
    var callback = this.getTitleValue('PROC_NAME') || '';
    var code = 'rospy.Subscriber(' + topic + ', ' + msg_type + ', ' + callback + ')\n';
    
    return code;
};

Blockly.Python.lowlevel_ros_init_node = function() {
    var name = Blockly.Python.valueToCode(this, 'NAME', Blockly.Python.ORDER_NONE) || '\'edufill_blockly_node\'';
    var anonym = (this.getTitleValue('TOGGLE_ANONYM')  == 'TRUE' ? 'True' : 'False');
    var log = this.getTitleValue('MODE_LOG');
    var sig = (this.getTitleValue('TOGGLE_SIG') == 'TRUE' ? 'True' : 'False');
    
    var code = 'rospy.init_node(' + name;
        if(anonym != 'EMPTY') {
            code += ', anonymous=' + anonym;
        }
        if(log != 'EMPTY') {
            code += ', log_level=rospy.' + log; 
        }
        if(sig != 'EMPTY') {
            code += ', disable_signals=' + sig;
        }
    code += ') \n';
    
    return code;
};

Blockly.Python.lowlevel_ros_spin = function() {
    var code = 'rospy.spin() \n';
    
    return code;
};

Blockly.Python.lowlevel_ros_cancelled = function() {
    var code = 'rospy.is_shutdown()';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_ros_log = function() {
    var log_type = "LANG_LOW_LEVEL_ROS_LOG_" + this.getTitleValue('MODE');
    var msg = Blockly.Python.valueToCode(this,'LOG_MSG',Blockly.Python.ORDER_NONE) || ''; 
    var code = 'rospy.log'+ Blockly[log_type] +'('+ msg +')\n';
    
    return code;
};

Blockly.Python.lowlevel_ros_sleep = function() {
    var time = Blockly.Python.valueToCode(this,'TIME', Blockly.Python.ORDER_NONE) || 0.1;
    var code = 'rospy.sleep(' + time + ') \n';
    
    return code;
};

Blockly.Python.lowlevel_move_base_goal = function() {
    Blockly.Python.definitions_['from_move_base_msgs.msg_import_*'] = 'from move_base_msgs.msg import *';
    Blockly.Python.RESERVED_WORDS_ += 'move_base_msgs.msg,';
    Blockly.Python.definitions_['def_move_base_goal'] = 
        'def move_base_goal(pose):\n' +
        '  goal = MoveBaseGoal()\n' +
        '  goal.target_pose = pose\n\n' +
        '  return goal\n';
    var pose = Blockly.Python.valueToCode(this, 'POSE', Blockly.Python.ORDER_NONE);
    var code = 'move_base_goal(' + pose +')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_get_ros_time = function() {
    Blockly.Python.definitions_['import_rospy'] = 'import rospy';
    Blockly.Python.RESERVED_WORDS_ += 'rospy,';
    var code = 'rospy.get_time()';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_get_ros_time_in_unit = function() {
    var variable = Blockly.Python.valueToCode(this, 'TIME', Blockly.Python.ORDER_NONE) || 'None';
    var code = variable + '.secs';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_laserscan_ranges_and_angles = function() {
    Blockly.Python.definitions_['import_read_laser_scan_component'] = 'import read_laser_scan_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_laser_scan_component,';
    var code = 'read_laser_scan_component.ranges_and_angles()';

    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_laserscan_inrange = function() {
    Blockly.Python.definitions_['import_read_laser_scan_component'] = 'import read_laser_scan_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_laser_scan_component,';
    var from = Blockly.Python.valueToCode(this, 'FROM', Blockly.Python.ORDER_NONE);
    var to = Blockly.Python.valueToCode(this, 'TO', Blockly.Python.ORDER_NONE);
    var code = 'read_laser_scan_component.distances([' + from + ', ' + to + '])';

    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_laserscan_closest_distance = function() {
    Blockly.Python.definitions_['import_read_laser_scan_component'] = 'import read_laser_scan_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_laser_scan_component,';
    var code = 'read_laser_scan_component.angle_of_closest_distance()';

    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_read_arm_joint_positions = function() {
    Blockly.Python.definitions_['import_read_arm_component'] = 'import read_arm_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_arm_component,';
    var code = 'read_arm_component.arm_joint_positions()';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};



Blockly.Python.lowlevel_read_finger_positions = function() {
    Blockly.Python.definitions_['import_read_arm_component'] = 'import read_arm_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_arm_component,';
    var code = 'read_arm_component.gripper_joint_positions()';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_read_ultrasonic = function() {
    Blockly.Python.definitions_['import_read_ultrasonic_range_component'] = 'import read_ultrasonic_range_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_ultrasonic_range_component,';
    var code = 'read_ultrasonic_range_component.distance()';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_odometry = function() {
    Blockly.Python.definitions_['import_read_base_component'] = 'import read_base_component';
    Blockly.Python.RESERVED_WORDS_ += 'read_base_component,';
    var code = 'read_base_component.odometry()'
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_quaternion_from_euler = function() {
    Blockly.Python.definitions_['import_tf'] = 'import tf';
    Blockly.Python.RESERVED_WORDS_ += 'tf,';
    
    var roll = Blockly.Python.valueToCode(this, 'ROLL', Blockly.Python.ORDER_NONE) || '0';
    var pitch = Blockly.Python.valueToCode(this, 'PITCH', Blockly.Python.ORDER_NONE) || '0';
    var yaw = Blockly.Python.valueToCode(this, 'YAW', Blockly.Python.ORDER_NONE) || '0';
    var code = 'tf.transformations.quaternion_from_euler(' + roll + ',' + pitch + ',' + yaw + ')'
    
    return [code ,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_is_wall = function() {
    Blockly.Python.definitions_['import_read_laser_scan_component'] = 'import read_laser_scan_component'; 
    Blockly.Python.RESERVED_WORDS_ += 'read_laser_scan_component,';
    
    var angle = Blockly.Python.valueToCode(this, 'ANGLE', Blockly.Python.ORDER_NONE) || '0'
    var distance = Blockly.Python.valueToCode(this, 'DISTANCE', Blockly.Python.ORDER_NONE);
    var code = 'read_laser_scan_component.is_wall(' + angle + ', ' + distance + ')'
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.lowlevel_get_param = function() {
    Blockly.Python.definitions_['import_rospy'] = 'import rospy'; 
    Blockly.Python.RESERVED_WORDS_ += 'rospy,';
    var dropdown = 'CONFIG_PARAMETER_' + this.getTitleValue('MODE')
    var code = 'rospy.get_param(' + Blockly[dropdown] +')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.ros_other_node = function() {
    var code = this.getTitleValue('NODE') || '';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.try_catch = function() {
    var try_branch = Blockly.Python.statementToCode(this,'TRY') || '  pass\n';
    var exception_var = Blockly.Python.variableDB_.getName(this.getTitleValue('VAR'), Blockly.Variables.NAME_TYPE);
    var catch_branch = Blockly.Python.statementToCode(this,'CATCH') || '  pass\n';
    var code =  'try: \n' + try_branch +
                'except Exception, ' + exception_var + ':\n' + catch_branch;
                
    return code;
};
