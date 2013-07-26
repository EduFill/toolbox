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
 * @fileoverview ROS message blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';

Blockly.Language.ros_string_msg_type = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('String (ROS message type)');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE);
        this.setTooltip('ROS message type to represent a String message');
    }
};

Blockly.Language.ros_twist_msg_type = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('Twist (ROS message type)');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE);
        this.setTooltip('ROS message type to represent a Twist message');
    }
}; 

Blockly.Language.ros_twist_msg = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('Twist()');
        this.setOutput(true, null);
        this.setTooltip('');
    }
};

Blockly.Language.ros_jointstate_msg_type = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('JointState (ROS message type)');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE);
        this.setTooltip('ROS message type to represent a JointState message');
    }
}; 

Blockly.Language.ros_jointstate_msg = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('JointState()');
        this.setOutput(true, null);
        this.setTooltip('');
    }
};



Blockly.Language.ros_get_twist_element = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.dropdown1 = new Blockly.FieldDropdown(this.OPERATORS1);
        this.dropdown2 = new Blockly.FieldDropdown(this.OPERATORS2);
        this.appendValueInput('VARIABLE')
            .appendTitle('get element')
            .appendTitle(this.dropdown1, 'MODE1')
            .appendTitle('.')
            .appendTitle(this.dropdown2, 'MODE2')
            .appendTitle('of Twist variable:');
        this.setOutput(true,null);
        this.setTooltip('');
    }
};

Blockly.TWIST_LINEAR = 'linear';
Blockly.TWIST_ANGULAR = 'angular';
Blockly.TWIST_X = 'x';
Blockly.TWIST_Y = 'y';
Blockly.TWIST_Z = 'z';

Blockly.Language.ros_get_twist_element.OPERATORS1 = 
    [
     [Blockly.TWIST_LINEAR, 'TWIST_LINEAR'],
     [Blockly.TWIST_ANGULAR, 'TWIST_ANGULAR']
    ];
    
Blockly.Language.ros_get_twist_element.OPERATORS2 = 
    [
     [Blockly.TWIST_X,'TWIST_X'],
     [Blockly.TWIST_Y,'TWIST_Y'], 
     [Blockly.TWIST_Z,'TWIST_Z']
    ];

Blockly.Language.ros_set_twist_element = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.dropdown1 = new Blockly.FieldDropdown(Blockly.Language.ros_get_twist_element.OPERATORS1);
        this.dropdown2 = new Blockly.FieldDropdown(Blockly.Language.ros_get_twist_element.OPERATORS2);
        this.appendDummyInput()
            .appendTitle('set element')
            .appendTitle(this.dropdown1, 'MODE1')
            .appendTitle('.')
            .appendTitle(this.dropdown2, 'MODE2');
        this.appendValueInput('VARIABLE')
            .appendTitle('of twist variable:')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('NEW_VAL')
            .appendTitle('with new value:')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.ros_jointpositions_msg_type = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('JointPositions (ROS message type)');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE);
        this.setTooltip('ROS message type to represent a jointPositions message');
    }
}; 

Blockly.Language.ros_other_msg_type = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput('T1').appendTitle(new Blockly.FieldTextInput('',null),'MSG_NAME');
        this.appendDummyInput('T2').appendTitle('(ROS message type)');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE);
        this.setInputsInline(true);
        this.setTooltip(
                        'Specify a ROS message name which is not listed\n' +
                        'Node: Do not forget to import appropriate modules to prevent run time errors.'
                        );
    }
};

Blockly.Language.rosmessage_base_pose_msg = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('base pose message');
        this.appendValueInput('X')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number')
            .appendTitle('x');
        this.appendValueInput('Y')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number')
            .appendTitle('y');
        this.appendValueInput('Z')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number')
            .appendTitle('z');
        this.appendValueInput('Q')
            .setCheck(Blockly.LANG_CONNECTION_TYPE_QUATERNION)
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendTitle('quaternion');
        this.setTooltip('TODO');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_BASE_POSE_MSG);
    }
};

Blockly.Language.rosmessage_gripper_position_msg = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('gripper position message');
        this.appendValueInput('GRIPPER_L')
            .appendTitle('left gripper')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('GRIPPER_R')
            .appendTitle('right gripper')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ARM_JOINT_POSITION);
    }
};

Blockly.Language.rosmessage_joint_state_msg_name = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput()
            .appendTitle(new Blockly.FieldVariable('jp'), 'VAR')
            .appendTitle('.name');
        this.appendValueInput('PREFIX')
            .appendTitle(' = prefix')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('String');
        this.appendValueInput('NUMBER_OF_JOINTS')
            .appendTitle('+ counting number (for all joints until')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle(')');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('TODO');
    },
    getVars: function() {
        return [this.getTitleValue('VAR')];
    },
    renameVar: function(oldName, newName) {
        if (Blockly.Names.equals(oldName, this.getTitleValue('VAR'))) {
            this.setTitleValue(newName, 'VAR');
        }
    }
};

Blockly.Language.rosmessage_joint_state_msg_position = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput()
            .appendTitle(new Blockly.FieldVariable('jp'), 'VAR')
            .appendTitle('.position');
        this.appendValueInput('VALUE')
            .appendTitle(' = value in list')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck(null);
        this.appendValueInput('NUMBER_OF_JOINTS')
            .appendTitle('(for all joints until')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle(')');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('TODO');
    },
    getVars: function() {
        return [this.getTitleValue('VAR')];
    },
    renameVar: function(oldName, newName) {
        if (Blockly.Names.equals(oldName, this.getTitleValue('VAR'))) {
            this.setTitleValue(newName, 'VAR');
        }
    }
};

Blockly.Language.rosmessage_joint_state_msg_velocity = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput()
            .appendTitle(new Blockly.FieldVariable('jp'), 'VAR')
            .appendTitle('.velocity');
        this.appendValueInput('VALUE')
            .appendTitle(' = value in list')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck(null);
        this.appendValueInput('NUMBER_OF_JOINTS')
            .appendTitle('(for all joints until')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle(')');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('TODO');
    },
    getVars: function() {
        return [this.getTitleValue('VAR')];
    },
    renameVar: function(oldName, newName) {
        if (Blockly.Names.equals(oldName, this.getTitleValue('VAR'))) {
            this.setTitleValue(newName, 'VAR');
        }
    }
};

Blockly.Language.rosmessage_joint_state_msg_effort = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput()
            .appendTitle(new Blockly.FieldVariable('jp'), 'VAR')
            .appendTitle('.effort');
        this.appendValueInput('VALUE')
            .appendTitle(' = value in list')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck(null);
        this.appendValueInput('NUMBER_OF_JOINTS')
            .appendTitle('(for all joints until')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle(')');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('TODO');
    },
    getVars: function() {
        return [this.getTitleValue('VAR')];
    },
    renameVar: function(oldName, newName) {
        if (Blockly.Names.equals(oldName, this.getTitleValue('VAR'))) {
            this.setTitleValue(newName, 'VAR');
        }
    }
};

Blockly.Language.rosmessage_arm_joint_position_msg = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('arm joint position message');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ARM_JOINT_POSITION);
        this.setMutator(new Blockly.Mutator(['rosmessage_arm_joint_position_msg_jointvalue']));
        this.setTooltip('');
        this.jointCount_ = 0;
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        if (this.jointCount_) {
            container.setAttribute('joints', this.jointCount_);
        }
        return container;
    },
    domToMutation: function(xmlElement) {
        for (var i = 0; i < this.jointCount_; i++) {
            if (this.getInput('JOINTVALUE' + i))
            {
                this.removeInput('JOINTVALUE' + i);
            }
        }
        this.jointCount_ = window.parseInt(xmlElement.getAttribute('joints'), 10);
        for (var i = 0; i < this.jointCount_; i++) {
            this.appendValueInput('JOINTVALUE' + i)
                .appendTitle('joint name')
                .appendTitle(new Blockly.FieldTextInput('joint_name', null),'JOINTNAME' + i)
                .appendTitle(', joint value');
        }
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'rosmessage_arm_joint_position_msg_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        for (var i = 0; i < this.jointCount_; i++) {
            var jointBlock = new Blockly.Block(workspace, 'rosmessage_arm_joint_position_msg_jointvalue');
            jointBlock.initSvg();
            connection.connect(jointBlock.previousConnection);
            connection = jointBlock.nextConnection;
        }
        return containerBlock;
    },
    compose: function(containerBlock) {
        // Disconnect all input blocks and remove all inputs.
        for (var i = this.jointCount_ - 1; i >= 0; i--) {
            if (this.getInput('JOINTVALUE' + i))
            {
                this.removeInput('JOINTVALUE' + i);
            }
        }   
        this.jointCount_ = 0;
        // Rebuild the block's inputs.
        var jointBlock = containerBlock.getInputTargetBlock('STACK');
        while (jointBlock) {
            var input = this.appendValueInput('JOINTVALUE' + this.jointCount_)
                            .appendTitle('joint name')
                            .appendTitle(new Blockly.FieldTextInput('joint_name_'+ (this.jointCount_+1), null),'JOINTNAME' + this.jointCount_)
                            .appendTitle(', joint value');
                
              // Reconnect any child blocks.
              if (jointBlock.valueConnection_) {
                input.connection.connect(jointBlock.valueConnection_);
              }
              this.jointCount_++;
              jointBlock = jointBlock.nextConnection && 
              jointBlock.nextConnection.targetBlock();
        }
    },
    saveConnections: function(containerBlock) {
        // Store a pointer to any connected child blocks.
        var jointBlock = containerBlock.getInputTargetBlock('STACK');
        var jointCount_ = 0;
        while (jointBlock) {
            var input = this.getInput('JOINTVALUE' + jointCount_);
            jointBlock.valueConnection_ = input && input.connection.targetConnection;
            jointCount_++;
            jointBlock = jointBlock.nextConnection &&
                jointBlock.nextConnection.targetBlock();
        }
    }
};

Blockly.Language.rosmessage_arm_joint_position_msg_container = {
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('arm joint position message');
        this.appendStatementInput('STACK');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.rosmessage_arm_joint_position_msg_jointvalue = {
    init: function() {
        this.setColour(Blockly.LANG_ROS_MESSAGE_COLOUR);
        this.appendDummyInput().appendTitle('joint value');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};
