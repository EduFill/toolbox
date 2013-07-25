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
 * @fileoverview Unit blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';

Blockly.Language.units_duration = {
    // numeric value of a duration
    helpUrl: null,
    init: function() {
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS);
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendValueInput('VALUE')
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle('(duration in')
            .appendTitle(dropdown, 'MODE')
            .appendTitle(')');
        this.setOutput(true, 'Number');
        this.setInputsInline(true);
        this.setTooltip(
                        'Indicate that these value is considered as duration.\n' +
                        'Note: The block does not change the value it just indicates a type.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* Unit (dropdown): Select the unit from the list\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* Unlabeled (value input): Specify the duration time.\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the specified number.\n' +
                        'the output type is \'Number\'.'
                        );
    }
};

Blockly.Language.units_duration.OPERATORS = 
    [['seconds', 'SEC'],
     ['minutes', 'MIN'],
     ['hours', 'H']];

Blockly.Language.units_distance = {
    // numeric value of a distance
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS); 
        this.appendValueInput('VALUE')
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle('(distance in')
            .appendTitle(dropdown, 'MODE')
            .appendTitle(')');
        this.setOutput(true, 'Number');
        this.setInputsInline(true);
        this.setTooltip(
                        'Indicate that these value is considered as a distance.\n' +
                        'Note: The block does not change the value it just indicates a type.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* Unit (dropdown): Select the unit from the list\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* Unlabeled (value input): Specify the distance.\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the specified number.\n' +
                        'the output type is \'Number\'.'
                        );
    }
};

Blockly.Language.units_distance.OPERATORS = 
    [['meters', 'M'],
     ['centimeter', 'CM'],
     ['millimeter', 'MM']];
     
Blockly.Language.units_velocity = {
    // numeric value of a velocity
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS); 
        this.appendValueInput('VALUE')
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle('(velocity in')
            .appendTitle(dropdown, 'MODE')
            .appendTitle(')');
        this.setOutput(true, 'Number');
        this.setInputsInline(true);
        this.setTooltip(
                        'Indicate that these value is considered as velocity.\n' +
                        'Note: The block does not change the value it just indicates a type.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* Unit (dropdown): Select the unit from the list\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* Unlabeled (value input): Specify the velocity.\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the specified number.\n' +
                        'the output type is \'Number\'.'
                        );
    }
};

Blockly.Language.units_velocity.OPERATORS = 
    [['meters/sec', 'M_SEC'],
     ['kilometers/hour', 'KM_H']];
     

Blockly.Language.units_6dPose = {
    // 6D pose (x,y,z, roll,pitch,yaw) 
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendDummyInput().appendTitle('6D pose');
        this.appendValueInput('LIN_X')
            .appendTitle('x')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.appendValueInput('LIN_Y')
            .appendTitle('y')
            .setCheck('Number')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('LIN_Z')
            .appendTitle('z')
            .setCheck('Number')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('ROLL')
            .appendTitle('roll (rad)')
            .setCheck('Number')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('PITCH')
            .appendTitle('pitch (rad)')
            .setCheck('Number')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('YAW')
            .appendTitle('yaw (rad)')
            .setCheck('Number')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setOutput(true,'Pose6D');
        this.setTooltip(
                        '6D pose\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* x (value input): Linear distance in x direction (in meters).\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '* y (value input): Linear distance in y direction (in meters).\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '* z (value input): Linear distance in z direction (in meters).\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '* roll (value input): Angular rotations around the x-axis (in radians).\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '* pitch (value input): Angular rotations around the y-axis (in radians).\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '* yaw (value input): Angular rotations around the z-axis (in radians).\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '---\n' +
                        'Output:\n' + 
                        '* Return a list of all six entries.\n' +
                        'The output type is \'Pose6D\'.'
                        );
    }
};

Blockly.Language.units_get_6dpose_element = {
    // Get access to elements of 6D pose 
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        var dropdown = new Blockly.FieldDropdown(Blockly.Language.units_6dpose_element_operators);
        this.appendValueInput('POSE6D')
            .appendTitle('get')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('of 6D pose')
            .setCheck('Pose6D');
        this.setOutput(true, 'Number');
        this.setTooltip(
                        'Get one of the elements of a 6D pose.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* element (dropdown): Specify the element that you would like to access.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* 6D pose(value input): Expected value is a 6D pose.\n' +
                        'The only allowed connection type is \'Pose6D\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the value of the specified element of the 6D pose.'
                        )
    }
};

Blockly.Language.units_6dpose_element_operators =
    [
     ['x','LIN_X'],
     ['y','LIN_Y'],
     ['z','LIN_Z'],
     ['roll','ROLL'],
     ['pitch','PITCH'],
     ['yaw','YAW']
     ]

Blockly.Language.units_set_6dpose_element = {
    // Set element of 6D pose to a numeric value
    helpUrl: null,
    init: function(){
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        var dropdown = new Blockly.FieldDropdown(Blockly.Language.units_6dpose_element_operators);
        this.appendValueInput('POSE6D')
            .appendTitle('set')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('of 6D pose')
            .setCheck('Pose6D')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('VALUE')
            .appendTitle('to')
            .setCheck('Number')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip(
                        'Set the value of the 6D pose list at the chosen position to a new one.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* element (dropwdown): Specify the element that you would like to address.\n' +
                        'Inputs:\n' +
                        '* 6d pose (value input): Expected value is a 6D pose.\n' +
                        '* to (value input): The new value of the chosen element.'
                        );
    }
};

Blockly.Language.units_joints = {
    // Create a list of joint angles with their numeric values. 
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendDummyInput()
            .appendTitle('joints (rad)');
        this.setMutator(new Blockly.Mutator(['units_joints_jointvalue']))
        this.setOutput(true, 'JointAngles');
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
                .appendTitle('joint value ' + (i+1))
                .setCheck('Number')
                .setAlign(Blockly.ALIGN_RIGHT);
        }
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'units_joints_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        for (var i = 0; i < this.jointCount_; i++) {
            var jointBlock = new Blockly.Block(workspace, 'units_joints_jointvalue');
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
                            .appendTitle('joint value ' + (this.jointCount_+1))
                            .setCheck('Number')
                            .setAlign(Blockly.ALIGN_RIGHT);
                
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

Blockly.Language.units_joints_container = {
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendDummyInput().appendTitle('joints');
        this.appendStatementInput('STACK');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
    }
};

Blockly.Language.units_joints_jointvalue = {
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendDummyInput().appendTitle('joint value');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
    }
};

Blockly.Language.units_get_joints_element = {
    // get numeric value of specific joint
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendValueInput('VALUE_POS')
            .appendTitle('get value ')
            .setCheck('Number')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('JOINTS')
            .appendTitle('of joints')
            .setCheck('JointAngles')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setOutput(true, 'Number');
        this.setInputsInline(true);
    }
};

Blockly.Language.units_set_joints = {
    // Set one joint to a numeric value
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendValueInput('VALUE_POS')
            .appendTitle('set value')
            .setCheck('Number')
        this.appendValueInput('JOINTS')
            .appendTitle('of joints')
            .setCheck('JointAngles')
        this.appendValueInput('NEW_VALUE')
            .appendTitle('to')
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle('(rad)');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
    }
};

Blockly.Language.units_degToRad = {
    // Perform a degree to radian conversion of a numeric value
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendValueInput('VALUE')
            .appendTitle('convert degree to radian')
            .setCheck('Number');
        this.setOutput(true, 'Number');
        this.setTooltip(
                        'Convert a value from degree to radian.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* Unlabeled (value input): Attach a value in degree.\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the value in radian.\n' +
                        'The output type is \'Number\'.'
                        );
    }
};

Blockly.Language.units_radToDeg = {
    // Perform a radian to degree conversion of a numeric value
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_UNITS_COLOUR);
        this.appendValueInput('VALUE')
            .appendTitle('convert radian to degree')
            .setCheck('Number');
        this.setOutput(true, 'Number');
        this.setTooltip(
                        'Convert a value from radian to degree.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* Unlabeled (value input): Attach a value in radian.\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the value in degree.\n' +
                        'The output type is \'Number\'.'
                        );
    }
};

Blockly.Language.units_eulerToQuaternion = {
  // Perform an euler angles to quaternion conversion of a numeric value
  helpUrl: null,
  init: function() {
    this.setColour(Blockly.LANG_UNITS_COLOUR);
    this.appendDummyInput()
        .appendTitle("convert euler to quaternion");
    this.appendValueInput("ROLL")
        .setCheck('Number')
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendTitle("roll (rad)");
    this.appendValueInput("PITCH")
        .setCheck('Number')
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendTitle("pitch (rad)");
    this.appendValueInput("YAW")
        .setCheck('Number')
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendTitle("yaw (rad)");
    this.setOutput(true, 'Quaternion');
    this.setTooltip(
        'Convert euler angles to quaternions.\n' +
        '---\n' +
        'Inputs:\n' +
        '* roll (value input): The rotation around X-axis in radians\n' +
        'The only allowed connection type is \'Number\'.\n' +
        '* pitch (value input): The rotation around Y-axis in radians\n' +
        'The only allowed connection type is \'Number\'.\n' +
        '* yaw (value input): The rotation around Z-axis in radians\n' +
        'The only allowed connection type is \'Number\'.\n' +
        '---\n' +
        'Output:\n' +
        'Return a list containing the four quaternion values x,y,z,w'
        );
  }
};

Blockly.Language.units_quaternionToEuler = {
  // Perform a quaternion to euler angles conversion of a numeric value  
  helpUrl: null,
  init: function() {
    this.setColour(Blockly.LANG_UNITS_COLOUR);
    this.appendDummyInput()
        .appendTitle("convert quaternion to euler");
    this.appendValueInput("X")
        .setCheck('Number')
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendTitle("x");
    this.appendValueInput("Y")
        .setCheck('Number')
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendTitle("y");
    this.appendValueInput("Z")
        .setCheck('Number')
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendTitle("z");
    this.appendValueInput("W")
        .setCheck('Number')
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendTitle("w");
    this.setOutput(true, 'Euler');
    this.setTooltip(
                'Convert quaternions to euler angles.\n' +
                '---\n' +
                'Inputs:\n' +
                'x (value input):\n' +
                'The only allowed connection type is \'Number\'.\n' +
                'y (value input):\n' +
                'The only allowed connection type is \'Number\'.\n' +
                'z (value input): \n' +
                'The only allowed connection type is \'Number\'.\n' +
                'w (value input): \n' +
                'The only allowed connection type is \'Number\'.\n' +
                '---\n' +
                'Output:\n' +
                'Return a list containing the euler angles (radians) roll, pitch, yaw'
    );
  }
};
