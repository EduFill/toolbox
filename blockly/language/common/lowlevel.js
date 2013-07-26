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
 * @fileoverview Low level blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';

Blockly.Language.lowlevel_package_main = {
    //main function
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle('main in')
            .appendTitle(new Blockly.FieldTextInput('ros_package_name',null),'ROSPaName')
            .appendTitle('package');
        this.appendValueInput('IMPORT0')
                .setCheck(['String', Blockly.LANG_CONNECTION_TYPE_IMPORT])
                .appendTitle(Blockly.LANG_LOW_LEVEL_IMPORT_MSG)
                .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('FROM_IMPORT0')
                .setCheck(Blockly.LANG_CONNECTION_TYPE_FROM_IMPORT)
                .appendTitle(Blockly.LANG_LOW_LEVEL_FROM_IMPORT_MSG)
                .setAlign(Blockly.ALIGN_RIGHT);
        this.appendStatementInput('MAINSTACK');
        this.setPreviousStatement(false);
        this.setNextStatement(true);
        this.setMutator(new Blockly.Mutator(['lowlevel_application_import','lowlevel_application_from_import']));
        this.setTooltip(
                        'Use this block exactly ONCE to prevent unintended effects: \n' +
                        'Never used: File cannot necessarily be reused as import file.\n' +
                        'Multiple times used: All main functions will be executed and may cause misbehaviors. \n' +
                        '---\n' +
                        'Fields:\n' +
                        '* Choose the ROS package name, so that it meets the ROS package in use, \n' +
                        'i.e. \'edufill_blockly\' if the generated code will be stored within the ROS package \'edufill_blockly\'.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* import (value input): Imports a python module. All definitions, variables, etc. of that module can be used. \n' + 
                        '  Allowed connection types are \'String\' and \'Import\'.\n' +
                        '* from x import (value input): Imports specified objects from a module.\n' +
                        '  The only allowed connection type is \'FromImport\'.\n' +
                        '* unlabeled (statement input): ' 
                        ); 
        this.importCount_ = 1;
        this.from_importCount_ = 1;     
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        if (this.importCount_) {
            container.setAttribute('imports', this.importCount_);
        }
        if (this.from_importCount_) {
            container.setAttribute('from_imports', this.from_importCount_);
        }
        
        return container;
    },
    domToMutation: function(xmlElement) {
        for (var i = 0; i < this.importCount_; i++) {
            if (this.getInput('IMPORT' + i)) {
                this.removeInput('IMPORT' + i);
            }
        }
        for (var i = 0; i < this.from_importCount_; i++) {
            if (this.getInput('FROM_IMPORT' + i)) {
                this.removeInput('FROM_IMPORT' + i);
            }
        }
        
        this.removeInput('MAINSTACK');
        
        this.importCount_ = window.parseInt(xmlElement.getAttribute('imports'), 10);
        for (var i = 0; i < this.importCount_; i++) {
            this.appendValueInput('IMPORT' + i)
                .setCheck(['String', Blockly.LANG_CONNECTION_TYPE_IMPORT])
                .appendTitle(Blockly.LANG_LOW_LEVEL_IMPORT_MSG)
                .setAlign(Blockly.ALIGN_RIGHT);
        }
        
        this.from_importCount_ = window.parseInt(xmlElement.getAttribute('from_imports'), 10);
        for (var i = 0; i < this.from_importCount_; i++) {
            this.appendValueInput('FROM_IMPORT' + i)
                .setCheck(Blockly.LANG_CONNECTION_TYPE_FROM_IMPORT)
                .appendTitle(Blockly.LANG_LOW_LEVEL_FROM_IMPORT_MSG)
                .setAlign(Blockly.ALIGN_RIGHT);
        }
        this.appendStatementInput('MAINSTACK');
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'lowlevel_application_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        
        for (var i = 0; i < this.importCount_; i++) {
            var importBlock = new Blockly.Block(workspace, 'lowlevel_application_import');
            importBlock.initSvg();
            connection.connect(importBlock.previousConnection);
            connection = importBlock.nextConnection;
        }
        
        for (var i = 0; i < this.from_importCount_; i++) {
            var fromBlock = new Blockly.Block(workspace, 'lowlevel_application_from_import');
            fromBlock.initSvg();
            connection.connect(fromBlock.previousConnection);
            connection = fromBlock.nextConnection;  
        }
        
        return containerBlock;
    },
    compose: function(containerBlock) {
        // Disconnect all input blocks and destroy all inputs.
        for (var i = this.importCount_ - 1; i >= 0; i--) {
            if(this.getInput('IMPORT' + i)) {   
                this.removeInput('IMPORT' + i);
            }
        }
        this.importCount_ = 0;
        
        for (var i = this.from_importCount_ - 1; i >= 0; i--) {
            if(this.getInput('FROM_IMPORT' + i)) {
                this.removeInput('FROM_IMPORT' + i);
            }
        }
        this.from_importCount_ = 0;
        
        if (this.getInput('MAINSTACK')) {
            this.removeInput('MAINSTACK');
        }
        
        var statementBlock = containerBlock.getInputTargetBlock('STACK');
        while(statementBlock) {
            if(statementBlock.type == 'lowlevel_application_import') {
                var input = this.appendValueInput('IMPORT' + this.importCount_)
                                .setCheck(['String',Blockly.LANG_CONNECTION_TYPE_IMPORT])
                                .setAlign(Blockly.ALIGN_RIGHT);
                input.appendTitle(Blockly.LANG_LOW_LEVEL_IMPORT_MSG);
                this.importCount_++;
                // Reconnect any child blocks.
                if (statementBlock.valueConnection_) {
                    input.connection.connect(statementBlock.valueConnection_);
                }
            }
            else if(statementBlock.type == 'lowlevel_application_from_import'){
                var input = this.appendValueInput('FROM_IMPORT' + this.from_importCount_)
                                .setCheck(Blockly.LANG_CONNECTION_TYPE_FROM_IMPORT)
                                .setAlign(Blockly.ALIGN_RIGHT);
                input.appendTitle(Blockly.LANG_LOW_LEVEL_FROM_IMPORT_MSG);
                this.from_importCount_++;
                if (statementBlock.valueConnection_) {
                    input.connection.connect(statementBlock.valueConnection_);
                }
            }
            else {
                throw 'Unknown block type';
            }
            statementBlock = statementBlock.nextConnection && statementBlock.nextConnection.targetBlock();
        }
        
        var input = this.appendStatementInput('MAINSTACK');
        if(this.valueConnection_) {
            input.connection.connect(this.valueConnection_);
        }
    },
    saveConnections: function(containerBlock) {
        // Store a pointer to any connected child blocks.
        var x_import = 0;
        var x_from = 0;
        
        var statementBlock = containerBlock.getInputTargetBlock('STACK');
        while(statementBlock) {
            if(statementBlock.type == 'lowlevel_application_import') {
                var input = this.getInput('IMPORT' + x_import);
                statementBlock.valueConnection_ = input && input.connection.targetConnection;
                x_import++;
            }
            else if(statementBlock.type == 'lowlevel_application_from_import') {
                var input = this.getInput('FROM_IMPORT' + x_from);
                statementBlock.valueConnection_ = input && input.connection.targetConnection;
                x_from++;
            }
            else {
                throw 'Unknown block type';
            }
            statementBlock = statementBlock.nextConnection && statementBlock.nextConnection.targetBlock();
        }
        var input = this.getInput('MAINSTACK');
        this.valueConnection_ = input && input.connection.targetConnection; 
    },
    onchange: function() {
        // If this block or the highlevel_application block are more than once (or both) on the workspace, show a warning
        var allowedNumberOfThisBlock = 1;
        var topBlocks = this.workspace && this.workspace.topBlocks_;
        var appCount = 0;
        if (topBlocks) {
            for (var i=0; i<topBlocks.length; i++) {
                if(topBlocks[i].type == 'highlevel_application' || topBlocks[i].type == this.type) {
                    appCount++;
                }
            }
        }
        
        if (appCount > allowedNumberOfThisBlock) {
            this.setWarningText('This block should not be used more than once and not at the same time as the "robot application frame" block');
        }
        else {
            this.setWarningText(null);
        }
    }
};

Blockly.Language.lowlevel_application_import = {
    // part of lowlevel_package_main
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle(Blockly.LANG_LOW_LEVEL_IMPORT_MSG);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
        this.contextMenu = false;
    }
};

Blockly.Language.lowlevel_application_from_import = {
    // part of lowlevel_package_main
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle(Blockly.LANG_LOW_LEVEL_FROM_IMPORT_MSG);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
        this.contextMenu = false;
    }
}; 

Blockly.Language.lowlevel_application_container = {
    // part of lowlevel_package_main
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle(Blockly.LANG_LOW_LEVEL_MAIN_FRAME);
        this.appendStatementInput('STACK');
        this.setTooltip('Mutator import container');
        this.contextMenu = false;
    }
};

Blockly.Language.lowlevel_from_x_import = {
    // block to alow to import functionalities of modules
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle('from')
            .appendTitle(new Blockly.FieldTextInput('module_name',null),'FROM')
            .appendTitle('import')
            .appendTitle(new Blockly.FieldTextInput('function_name',null),'IMPORT');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_FROM_IMPORT);
        this.setTooltip(
                        'Specify a module name of choice and which functions should be used.\n' +
                        'Note: If the function_name is left empty all functions within the module are available (equal to "*").\n' +
                        '---\n' +
                        'Fields:\n' +
                        'From (text): Module name\n' +
                        'import (text): Functions (comma separated, e.g. func1, func2, ...).\n' +
                        '---\n' +
                        'Output:\n' +
                        'The output is a complete import statement. Note if the module is left empty an empty string is returned.\n' +
                        'The output type is FromImport'
                        );
    }
};

Blockly.Language.lowlevel_create_publisher = {
    // Instantiate ROS publisher
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle(new Blockly.FieldVariable('pub'), 'VAR');
        this.appendValueInput('NODE')
            .setCheck('String')
            .appendTitle(' = create publisher to topic:');
        this.appendValueInput('TYPE')
            .setCheck(Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE)
            .appendTitle('with message type:');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip(
                        'Create a publisher, which enables message sending to the defined topic\n' +
                        '---\n' +
                        'Fields:\n' +
                        '(variable): Specify the variable in which the instance of the publisher should be stored.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        'topic (value input): Destination (topic) where the message should be send to. The only allowed connection type is \'String\'\n.' +
                        'message type (value input): Specification of the ROS message type. The only allowed connection type is \'rosNode\'.'
                        );
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

Blockly.Language.lowlevel_publish = {
    // Publish a ROS message to the previously specified topic
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendValueInput('MSG')
            .appendTitle('publish')
            .appendTitle('object:')
            .appendTitle(new Blockly.FieldVariable('pub'), 'VAR')
            .setCheck(null)
            .appendTitle('message:'); 
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip(
                        'Publish a message using a publisher instance.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '(variable): ROS publisher instance (create publisher block)\n' +
                        '---\n' + 
                        'Inputs:\n' +
                        'message (input value): To be transmitted ROS message.'
                        );
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

Blockly.Language.lowlevel_subscriber = {
    // Subscribe (listen) to a ROS topic
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('subscribe');
        this.appendValueInput('NODE')
            .setCheck('String')
            .appendTitle('to topic:');
        this.appendValueInput('TYPE')
            .setCheck(Blockly.LANG_CONNECTION_TYPE_ROS_MESSAGE)
            .appendTitle('with message type:');
        this.appendDummyInput('CALLBACK')
            .appendTitle('and call')
            .appendTitle(new Blockly.FieldTextInput('procedure_name',null),'PROC_NAME');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip(
                        'Whenever a new message arrives at the listened topic, the function is called.\n' +
                        '---\n' +
                        'Fields:\n' +
                        'call (text): Name of the function that should be called whenever a new message arrives.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        'topic (value input): Specification to which topic is listened to. The only allowed connection type is \'String\'\n' +
                        'message type (value input): Type of arriving messages.\n' 
                        );
    }
};

Blockly.Language.lowlevel_ros_init_node = {
    // Initialize a ROS node
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        var dropdown_log = new Blockly.FieldDropdown(Blockly.Language.lowlevel_ros_log.OPERATORS);
        var checkbox_anonym = new Blockly.FieldCheckbox('FALSE');
        var checkbox_sig = new Blockly.FieldCheckbox('FALSE');
        this.appendValueInput('NAME')
            .setCheck(['String', Blockly.LANG_CONNECTION_TYPE_ROS_NODE])
            .appendTitle('init node  name:');
        this.appendDummyInput()
            .appendTitle(checkbox_anonym, 'TOGGLE_ANONYM')
            .appendTitle('anonymous:');
        this.appendDummyInput()
            .appendTitle('log level:')
            .appendTitle(dropdown_log, 'MODE_LOG');
        this.appendDummyInput()
            .appendTitle(checkbox_sig, 'TOGGLE_SIG')
            .appendTitle('disable signals:');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(false);
        this.setTooltip('Initialize this node');
    }
};

Blockly.Language.lowlevel_ros_spin = {
    // Keeps the ROS node alive
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('ros spin');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('Do not exit the node until it is shutdown.');
    }
};

Blockly.Language.lowlevel_ros_cancelled = {
    // Check whether ROS was cancelled or not
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('ROS cancelled');
        this.setOutput(true, null);
        this.setTooltip(
                        'Check whether ROS is cancelled by ctr+c or similar. \n' +
                        '---\n' +
                        'Output:\n' +
                        'Return true if cancelled and false elsewise.'
                        );
    }
};

Blockly.Language.lowlevel_ros_log = {
    // ROS logging
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS);
        this.appendDummyInput()
            .appendTitle('ROS log <')
            .appendTitle(dropdown, 'MODE');
        this.appendValueInput('LOG_MSG')
            .setCheck('String')
            .appendTitle('>:');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip('');
    }
};

// Please do not change the names, they are directly related to ROS
Blockly.Language.lowlevel_ros_log.OPERATORS = 
    [[Blockly.LANG_LOW_LEVEL_ROS_LOG_EMPTY, 'EMPTY'],
     [Blockly.LANG_LOW_LEVEL_ROS_LOG_DEBUG, 'DEBUG'],
     [Blockly.LANG_LOW_LEVEL_ROS_LOG_INFO, 'INFO'],
     [Blockly.LANG_LOW_LEVEL_ROS_LOG_WARN, 'WARN'],
     [Blockly.LANG_LOW_LEVEL_ROS_LOG_ERROR, 'ERROR'],
     [Blockly.LANG_LOW_LEVEL_ROS_LOG_FATAL, 'FATAL']];

Blockly.Language.lowlevel_ros_sleep = {
    // ROS sleep
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendValueInput('TIME')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number')
            .appendTitle('ROS sleep');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.lowlevel_move_base_goal = {
    // Creates a movebaseGoal goal with a target pose
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('move base goal');
        this.appendValueInput('POSE')
            .appendTitle('pose')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck(Blockly.LANG_CONNECTION_TYPE_BASE_POSE_MSG);
        this.setOutput(true);
        this.setTooltip('');
    }
};

Blockly.Language.lowlevel_get_ros_time = {
    // Get ROS time
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('get ros time');
        this.setTooltip('');
        this.setOutput(true);
    }
};

Blockly.Language.lowlevel_get_ros_time_in_unit = {
    // Get rostime in seconds
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendValueInput('TIME').appendTitle('ros time');
        this.appendDummyInput().appendTitle('in seconds');
        this.setInputsInline(true);
        this.setTooltip('');
        this.setOutput(true);
    }
};

Blockly.Language.lowlevel_laserscan_ranges_and_angles = {
    // Get laser scan results
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle('laser scan data (full range)');
        this.setOutput(true, 'Array');
        this.setTooltip(
                        'Provide the full set of laser scan data.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a list of tuples containing the laser scan measurement and the related angles (radians).\n' +
                        'The output type is \'Array\'.'
                        );
    }
};

Blockly.Language.lowlevel_laserscan_inrange = {
    // Get laser scan results within a specific range
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('laser scan data');
        this.appendValueInput('FROM')
            .appendTitle('from angle (rad)')
            .setCheck('Number');
        this.appendValueInput('TO')
            .appendTitle('to angle (rad)')
            .setCheck('Number');
        this.setOutput(true, 'Array');
        this.setInputsInline(true);
        this.setTooltip(
                        'Provide data for laser scans within a certain angular range.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* From (value input): Specify the smallest (most left) angle (radians) of interest. \n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '* To (value input): Specify the biggest (most right) angle (radians) of interest.\n' + 
                        'The only allowed connection type is \'Number\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a list of tuples containing the laser scan measurement and the related angles (radians).\n' +
                        'The output type is \'Array\'.'
                        );
    }
};

Blockly.Language.lowlevel_laserscan_closest_distance = {
    // Get the scan with the closest distance to the robot
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('laser scan with closest distance');
        this.setOutput(true, 'Array');
        this.setTooltip(
                        'Determine the laser scan value with the closest distance.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a tuple containing the laser scan value (distance) and the related angle (radians).\n' +
                        'The output type is \'Array\'.'
                        );
    }
};

Blockly.Language.lowlevel_read_arm_joint_positions = {
    // Read the actual joint configuration
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('read arm joint positions');
        this.setOutput(true, null);
        this.setTooltip(
                        'Provide the actual joint angles (radians) of the robot arm.\n' + 
                        '---\n' +
                        'Output:\n' +
                        '* Return the joint angles of the robot arm as a ROS message (joint_positions).\n' +
                        'The output type is not specified yet.'
                        );
    }
};

Blockly.Language.lowlevel_read_finger_positions = {
    // Read the actual finger configuration (position)
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('read finger positions');
        this.setOutput(true, null);
        this.setTooltip(
                        'Finger is the general term for the gripper joints or other components realizing a hand/fingers and a means to grasp\n' +
                        'The block provides the actual position of the fingers.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the finger positions of the robot end-effector as a ROS message (joint_positions).\n' +
                        'The output type is not specified yet.'
                        );
    }
};

Blockly.Language.lowlevel_read_ultrasonic = {
    // Read the actual ultrasound value
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('read ultrasonic reading');
        this.setOutput(true, null);
        this.setTooltip(
                        'Provide the result of the ultrasonic reading.\n' + 
                        '---\n' +
                        'Output:\n' +
                        '* Return the distance (m) of the object in front of the ultrasonic sensors).\n' +
                        'The output type is not specified yet.'
                        );
    }
};


Blockly.Language.lowlevel_odometry = {
    // Read the actual odometry information
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('read odometry data');
        this.setOutput(true, 'Pose6D');
        this.setTooltip(
                        'Read the odometry information from the robot base.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a 6D pose.'+
                        'The only allowed output type is \'Pose6D\'.'
                        );
    }
};

Blockly.Language.lowlevel_quaternion_from_euler = {
    // Use ROS to provide a conversion from quaternion to euler angles
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('quaternion from euler');
        this.appendValueInput('ROLL')
            .appendTitle('roll (rad)')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.appendValueInput('PITCH')
            .appendTitle('pitch (rad)')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.appendValueInput('YAW')
            .appendTitle('yaw (rad)')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Number');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_QUATERNION);
        this.setTooltip('');
    }
};

Blockly.Language.lowlevel_is_wall = {
    // Check for a wall within a specific direction and distance 
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        this.appendValueInput('ANGLE')
            .appendTitle('check for obstacle at angle (rad)')
            .setCheck('Number');
        this.appendValueInput('DISTANCE')
            .appendTitle('and max. distance (m) of')
            .setCheck('Number');
        this.setInputsInline(true);
        this.setOutput(true, 'Boolean');
        this.setTooltip(
                        'Test whether something (e.g. an obstacle or a wall) is closer than the specified distance.\n'+
                        '---\n' +
                        'Inputs:\n' +
                        '* angle (value input): The angle (radians) to specify the direction of interest (with respect to the sensor).\n' +
                        '* distance (value input): Maximum distance of interest in meters.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return True if an obstacle is perceived and False otherwise.' 
                        )
    }
}

Blockly.Language.lowlevel_get_param = {
    // Get values from a ROS parameter server 
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_LOW_LEVEL_COLOUR);
        var dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_PARAMETER_SERVER);
        this.appendDummyInput()
            .appendTitle('get')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('from parameter server');
        this.setOutput(true,null);
    }
};

Blockly.Language.ros_other_node = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ROS_NODE_COLOUR);
        this.appendDummyInput().appendTitle(new Blockly.FieldTextInput('',null), 'NODE');
        this.appendDummyInput().appendTitle('node');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_ROS_NODE);
        this.setInputsInline(true);
        this.setTooltip('');
    }
};

Blockly.Language.try_catch = {
    // try and catch (except) to allow error handling. Note: For now the only supportes exception type is 'Exception'. 'ValueError' and cannot be chosen.
    helpUrl: null,
    init: function() {
        this.setColour(120);
        this.appendDummyInput()
            .appendTitle('try');
        this.appendStatementInput('TRY');
        this.appendDummyInput() 
            .appendTitle('catch')
            .appendTitle(new Blockly.FieldVariable('e'), 'VAR');
        this.appendStatementInput('CATCH');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
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
