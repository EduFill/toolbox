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
 *  * @fileoverview Generating Python for unit blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
Blockly.Python = Blockly.Generator.get('Python');
 
if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.units_duration = function() {
    var value = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE) || 0;
    
    return [value, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_distance = function() {
    var value = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE) || 0;
    
    return [value, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_velocity = function() {
    var value = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE) || 0;
    
    return [value, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_6dPose = function() {
    var x = Blockly.Python.valueToCode(this, 'LIN_X', Blockly.Python.ORDER_NONE) || 0;
    var y = Blockly.Python.valueToCode(this, 'LIN_Y', Blockly.Python.ORDER_NONE) || 0;
    var z = Blockly.Python.valueToCode(this, 'LIN_Z', Blockly.Python.ORDER_NONE) || 0;
    var roll = Blockly.Python.valueToCode(this, 'ROLL', Blockly.Python.ORDER_NONE) || 0;
    var pitch = Blockly.Python.valueToCode(this, 'PITCH', Blockly.Python.ORDER_NONE) || 0;
    var yaw = Blockly.Python.valueToCode(this, 'YAW', Blockly.Python.ORDER_NONE) || 0;

    var code = '[' + x + ',' +  y+ ',' + z + ',' + roll + ',' + pitch + ',' + yaw + ']';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_get_6dpose_element = function() {
    var pose = Blockly.Python.valueToCode(this, 'POSE6D', Blockly.Python.ORDER_NONE) || 'None';
    var element = Blockly.Python.units_6dpose_element_operators[this.getTitleValue('MODE')];
    var code = pose + element;
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_6dpose_element_operators = {
    LIN_X: '[0]',
    LIN_Y: '[1]',
    LIN_Z: '[2]', 
    ROLL:  '[3]', 
    PITCH: '[4]', 
    YAW:   '[5]' 
}

Blockly.Python.units_set_6dpose_element = function() {
    var pose = Blockly.Python.valueToCode(this, 'POSE6D', Blockly.Python.ORDER_NONE) || 'None';
    var value = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE) || '0';
    var element = Blockly.Python.units_6dpose_element_operators[this.getTitleValue('MODE')];
    var code = pose + element + ' = ' + value +'\n';
    
    return code;
};

Blockly.Python.units_base_pose = function() {
    var x = Blockly.Python.valueToCode(this, 'X', Blockly.Python.ORDER_NONE) || 0;
    var y = Blockly.Python.valueToCode(this, 'Y', Blockly.Python.ORDER_NONE) || 0;
    var theta = Blockly.Python.valueToCode(this, 'THETA', Blockly.Python.ORDER_NONE) || 0;
    
    var code = '[' + x + ',' +  y + ',' + theta + ']';
    
    return [code, Blockly.Python.ORDER_ATOMIC]
};

Blockly.Python.units_joints = function() {
    var joints = this.jointCount_;
    var code = '';
    var value = [];
    for (var i=0; i<joints;i++) {
        value[i] = Blockly.Python.valueToCode(this, 'JOINTVALUE' + i, Blockly.Python.ORDER_NONE) || 'None';
    } 
    code = '[' + value.join(', ') + ']';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_get_joints_element = function() {
    var element = Blockly.Python.valueToCode(this, 'VALUE_POS', Blockly.Python.ORDER_NONE) || '1';
    element = parseInt(element)
    var jointList = Blockly.Python.valueToCode(this, 'JOINTS', Blockly.Python.ORDER_NONE) || '[]';
    var code = jointList + '[' + (element-1) + ']';
    
    return [code,Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_set_joints = function() {
    var element = Blockly.Python.valueToCode(this, 'VALUE_POS', Blockly.Python.ORDER_NONE) || '1';
    element = parseInt(element);
    var jointList = Blockly.Python.valueToCode(this, 'JOINTS', Blockly.Python.ORDER_NONE) || '[]';
    var value = Blockly.Python.valueToCode(this, 'NEW_VALUE', Blockly.Python.ORDER_NONE) || 'None';
    var code = jointList + '[' + (element-1) + ']' + ' = ' + value + '\n';
    
    return code;
}

Blockly.Python.units_degToRad = function() {
    Blockly.Python.definitions_['import_math'] = 'import math'
    Blockly.Python.RESERVED_WORDS_ += 'math,';
    var value = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE) || 0;
    
    var code = 'math.radians(' + value + ')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_radToDeg = function() {
    Blockly.Python.definitions_['import_math'] = 'import math'
    Blockly.Python.RESERVED_WORDS_ += 'math,';
    var value = Blockly.Python.valueToCode(this, 'VALUE', Blockly.Python.ORDER_NONE) || 0;
    
    var code = 'math.degrees(' + value + ')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.units_eulerToQuaternion = function() {
    Blockly.Python.definitions_['import_math'] = 'import math'
    Blockly.Python.RESERVED_WORDS_ += 'math,';
    
    var roll = Blockly.Python.valueToCode(this, 'ROLL', Blockly.Python.ORDER_NONE) || 0;
    var pitch = Blockly.Python.valueToCode(this, 'PITCH', Blockly.Python.ORDER_NONE) || 0;
    var yaw = Blockly.Python.valueToCode(this, 'YAW', Blockly.Python.ORDER_NONE) || 0;
      
    var qx = 'math.sin(' + roll + '/2) * math.cos(' + pitch + '/2) * math.cos(' + yaw + '/2) + math.cos(' + roll + '/2) * math.sin(' + pitch + '/2) * math.sin(' + yaw + '/2)';
    var qy = 'math.cos(' + roll + '/2) * math.sin(' + pitch + '/2) * math.cos(' + yaw + '/2) - math.sin(' + roll + '/2) * math.cos(' + pitch + '/2) * math.sin(' + yaw + '/2)';
    var qz = 'math.cos(' + roll + '/2) * math.cos(' + pitch + '/2) * math.sin(' + yaw + '/2) + math.sin(' + roll + '/2) * math.sin(' + pitch + '/2) * math.cos(' + yaw + '/2)';
    var qw = 'math.cos(' + roll + '/2) * math.cos(' + pitch + '/2) * math.cos(' + yaw + '/2) - math.sin(' + roll + '/2) * math.sin(' + pitch + '/2) * math.sin(' + yaw + '/2)';
    var code = '[' + qx + ', \\\n' + qy + ', \\\n' + qz + ', \\\n' + qw +']';
      
    return [code, Blockly.Python.ORDER_MEMBER];
};

Blockly.Python.units_quaternionToEuler = function() {
    Blockly.Python.definitions_['import_math'] = 'import math'
    Blockly.Python.RESERVED_WORDS_ += 'math,';

    var qx = Blockly.Python.valueToCode(this, 'X', Blockly.Python.ORDER_NONE) || 0;
    var qy = Blockly.Python.valueToCode(this, 'Y', Blockly.Python.ORDER_NONE) || 0;
    var qz = Blockly.Python.valueToCode(this, 'Z', Blockly.Python.ORDER_NONE) || 0;
    var qw = Blockly.Python.valueToCode(this, 'W', Blockly.Python.ORDER_NONE) || 0;
  
    var eX = 'math.atan2(-2 * (' + qy + ' * ' + qz + ' - ' + qw + ' * ' + qx + '), ' + qw + ' * ' + qw + ' - ' + qx + ' * ' + qx + ' - ' + qy + ' * ' + qy + ' + ' + qz + ' * ' + qz + ')';
    var eY = 'math.asin(2 * (' + qx + ' * ' + qz + ' + ' + qw + ' * ' + qy + ')';
    var eZ = 'math.atan2(-2 * (' + qx + ' * ' + qy + ' - ' + qw + ' * ' + qz + '), ' + qw + ' * ' + qw + ' + ' + qx + ' * ' + qx + ' - ' + qy + ' * ' + qy + ' - ' + qz + ' * ' + qz + ')';
    var code = '[' + eX + ', \\\n' + eY + ', \\\n' + eZ + ']';
  
    return [code, Blockly.Python.ORDER_MEMBER];
};
