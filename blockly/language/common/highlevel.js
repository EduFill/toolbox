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
 * @fileoverview High level blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
goog.provide('Blockly.Language.highlevel');

goog.require('Blockly.Language');

Blockly.Language.highlevel_application = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.appendDummyInput('APP_FRAME').appendTitle('robot application frame');
        this.appendStatementInput('APP_STACK');
        this.setPreviousStatement(false);
        this.setNextStatement(false);
        this.setTooltip(
            'Application Frame: \n' +
            'Each and every application should used this block exactly once. It provides basic functionalities. \n' +
            'Note: If used more than once a warning occurs and informs you about the fact.\n' +
            '---\n' +
            'Inputs:\n' +
            '* unlabeled (statement input): Content of the main function.'
        ); 
        this.pkg_name = Blockly.EDUFILL_ROS_PKG_NAME;
    },
    onchange: function() {
        var allowedNumberOfThisBlock = 1;
        var topBlocks = this.workspace && this.workspace.topBlocks_;
        var appCount = 0;
        if (topBlocks) {
            for (var i=0; i<topBlocks.length; i++) {
                if(topBlocks[i].type == 'highlevel_application' || topBlocks[i].type == 'lowlevel_package_main') {
                    appCount++;
                }
            }
        }
        
        if (appCount > allowedNumberOfThisBlock) {
            this.setWarningText('This block should not be used more than once and not at the same time as the "main in ... package" block');
        }
        else {
            this.setWarningText(null);
        }
    }
};

Blockly.Language.highlevel_move_base_to_goal = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_PRE_DEF_POSITIONS);
        this.inputNameDropdown = 'DROPDOWN';
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;
                
            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox, 'STATE')
            .appendTitle('move robot base to pose');
        this.appendDummyInput(this.inputNameDropdown)
            .appendTitle(this.dropdown, 'MODE');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip(
            'Move the robot base (platform) to a global goal pose represented by a label\n' +
            '---\n' +
            'Fields:\n' +
            '* ' + Blockly.ChangeModeTooltip + '\n' +
            '* Pose (dropdown): Select a desired global goal pose from the list (The location can be seen within the map).'
        );
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('STATE');
        container.setAttribute('changemodestate', state);
        if (this.dropdownValue) {
            container.setAttribute('dropdownstate', this.dropdownValue);
        }
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var block = this;
        
        var changeModeState = xmlElement.getAttribute('changemodestate');
        if(changeModeState == 'TRUE') {
            if(!block.getInput(this.changemode.appendInput)) {
                block.appendStatementInput(this.changemode.appendInput);
            }
            
            if (block.getInput(block.inputNameDropdown)) {
                block.removeInput(block.inputNameDropdown);
                delete block.dropdown;
            }
            block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        }
        
        var dropdownState = xmlElement.getAttribute('dropdownstate');
        if (dropdownState) {
            block.dropdownValue = dropdownState;
        }
    },
    changeModeToDom: function() {
        var state = this.getTitleValue('STATE');
        return this.changemode.changeModeToDom(state);
    },
    domToChangeMode: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if (changeModeState == 'TRUE') {
            this.changemode.domToChangeMode(xmlElement);
        }
    },
    showChangeMode: function() {
        var block = this;
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
                
        if (block.getInput(block.inputNameDropdown)) {
            block.removeInput(block.inputNameDropdown);
            block.dropdownValue = block.dropdown.getValue();
            delete block.dropdown;
        }
        
        var midlevel_move_base_to_poseBlock0 = block.changemode.appendNoneMutationStatement('midlevel_move_base_to_pose');
        midlevel_move_base_to_poseBlock0.setTitleValue('FALSE','STATE');
        var midlevel_move_base_to_poseChangemode0 = new Blockly.ChangeMode(midlevel_move_base_to_poseBlock0);
        var lowlevel_get_paramBlock1 = midlevel_move_base_to_poseChangemode0.appendNoneMutationOutput('lowlevel_get_param', 1);
        lowlevel_get_paramBlock1.setTitleValue(block.dropdownValue,'MODE');
    },
    disposeChangeMode: function() {
        var block = this;
                
        if (!block.getInput(block.inputNameDropdown)) {
            block.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_PRE_DEF_POSITIONS);
            if (this.dropdownValue) {
                block.dropdown.setValue(block.dropdownValue);
            }
            block.appendDummyInput(block.inputNameDropdown)
                .appendTitle(block.dropdown, 'MODE');
        }
        
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};

Blockly.Language.highlevel_move_base_distance = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.titleText = 'distance (m) or angle (rad)';
        this.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_BASE_DIRECTION);
        this.inputNameDropdown = 'DISTANCE';
        this.appendDummyInput()
            .appendTitle('move robot base');
        this.appendValueInput('DURATION')
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendTitle('direction')
            .appendTitle(this.dropdown, 'MODE')
            .appendTitle(this.titleText)
            .setCheck(Number);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
    }
};
     
Blockly.Language.highlevel_move_base_to_direction = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_BASE_DIRECTION);
        this.inputNameDropdown = 'DURATION';
        var emptyDropdown = new Blockly.FieldDropdown([['','EMPTY']]);
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;
            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('move robot base');
        this.appendValueInput('DURATION')
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendTitle('direction')
            .appendTitle(this.dropdown, 'MODE')
            .appendTitle('duration (sec)')
            .setCheck(Number);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip(
            'Move or turn the robot for a defined amount of time (in seconds).\n' +
            'Note: The movement is a blocking operation so that no other operations can be performed until the movement has been finished.\n' +
            '---\n' +
            'Fields:\n' +
            '* ' + Blockly.ChangeModeTooltip + '\n' +
            '* Direction (dropdown):Select a direction of movement from the list.\n' +
            '* Duration (value input): Attach a duration is seconds. The only allowed connection type is \'Number\'.'
        );
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('STATE');
        container.setAttribute('changemodestate', state);
        if (this.dropdownValue) {
            container.setAttribute('dropdownstate', this.dropdownValue);
        }
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var block = this;
        
        var changeModeState = xmlElement.getAttribute('changemodestate');
        if(changeModeState == 'TRUE') {
            if(!block.getInput(this.changemode.appendInput)) {
                block.appendStatementInput(this.changemode.appendInput);
            }
            if (block.getInput(block.inputNameDropdown)) {
                block.removeInput(block.inputNameDropdown);
                delete block.dropdown
            }
            block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        }
        
        var dropdownState = xmlElement.getAttribute('dropdownstate');
        if (dropdownState) {
            block.dropdownValue = dropdownState;
        }
    },
    changeModeToDom: function() {
        var state = this.getTitleValue('STATE');
        return this.changemode.changeModeToDom(state);
    },
    domToChangeMode: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if (changeModeState == 'TRUE') {
            this.changemode.domToChangeMode(xmlElement);
        }
    },
    showChangeMode: function() {
        var block = this;
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
                
        var attachedBlockDuration = block.getInputTargetBlock(block.inputNameDropdown);
        var text = Blockly['CONFIG_BASE_DIRECTION_' + block.getTitleValue('MODE')];
        var directionValues = [];
        var directionString = text;
        var directionValueList = Blockly.MoveRobotBaseDirectionString;
        for (var i=0; i<directionValueList.length; i++) {
            if (directionValueList[i][0] == directionString) {
                directionValues = directionValueList[i][1];
                break;
            }
        }
        if (block.getInput(block.inputNameDropdown)) {
            block.removeInput(block.inputNameDropdown);
            block.dropdownValue = block.dropdown.getValue();
            delete block.dropdown;
        }
    
        var variables_setBlock0 = block.changemode.appendNoneMutationStatement('variables_set');
        variables_setBlock0.setTitleValue('duration','VAR');
        var variables_setChangemode0 = new Blockly.ChangeMode(variables_setBlock0);
        variables_setChangemode0.reconnectOutputConnection(attachedBlockDuration, 1);
        block.changemode.addToReconnectMonitor(variables_setBlock0, 1, 1);
        var variables_setBlock2 = block.changemode.appendNoneMutationStatement('variables_set');
        variables_setBlock2.setTitleValue('init_time','VAR');
        var variables_setChangemode2 = new Blockly.ChangeMode(variables_setBlock2);
        var lowlevel_get_ros_timeBlock3 = variables_setChangemode2.appendNoneMutationOutput('lowlevel_get_ros_time', 1);
        var controls_whileUntilBlock4 = block.changemode.appendNoneMutationStatement('controls_whileUntil');
        controls_whileUntilBlock4.setTitleValue('WHILE','MODE');
        var controls_whileUntilChangemode4 = new Blockly.ChangeMode(controls_whileUntilBlock4);
        var logic_compareBlock5 = controls_whileUntilChangemode4.appendNoneMutationOutput('logic_compare', 1);
        logic_compareBlock5.setTitleValue('LTE','OP');
        var logic_compareChangemode5 = new Blockly.ChangeMode(logic_compareBlock5);
        var variables_getBlock6 = logic_compareChangemode5.appendNoneMutationOutput('variables_get', 1);
        variables_getBlock6.setTitleValue('init_time','VAR');
        var math_numberBlock7 = logic_compareChangemode5.appendNoneMutationOutput('math_number', 2);
        math_numberBlock7.setTitleValue('0','NUM');
        controls_whileUntilChangemode4.changeStatementInputTo('DO');
        var variables_setBlock8 = controls_whileUntilChangemode4.appendNoneMutationStatement('variables_set');
        variables_setBlock8.setTitleValue('init_time','VAR');
        var variables_setChangemode8 = new Blockly.ChangeMode(variables_setBlock8);
        var lowlevel_get_ros_timeBlock9 = variables_setChangemode8.appendNoneMutationOutput('lowlevel_get_ros_time', 1);
        var variables_setBlock10 = block.changemode.appendNoneMutationStatement('variables_set');
        variables_setBlock10.setTitleValue('now','VAR');
        var variables_setChangemode10 = new Blockly.ChangeMode(variables_setBlock10);
        var lowlevel_get_ros_timeBlock11 = variables_setChangemode10.appendNoneMutationOutput('lowlevel_get_ros_time', 1);
        var controls_whileUntilBlock12 = block.changemode.appendNoneMutationStatement('controls_whileUntil');
        controls_whileUntilBlock12.setTitleValue('WHILE','MODE');
        var controls_whileUntilChangemode12 = new Blockly.ChangeMode(controls_whileUntilBlock12);
        var logic_compareBlock13 = controls_whileUntilChangemode12.appendNoneMutationOutput('logic_compare', 1);
        logic_compareBlock13.setTitleValue('LT','OP');
        var logic_compareChangemode13 = new Blockly.ChangeMode(logic_compareBlock13);
        var lowlevel_get_ros_timeBlock14 = logic_compareChangemode13.appendNoneMutationOutput('lowlevel_get_ros_time', 1);
        var math_arithmeticBlock15 = logic_compareChangemode13.appendNoneMutationOutput('math_arithmetic', 2);
        math_arithmeticBlock15.setTitleValue('ADD','OP');
        var math_arithmeticChangemode15 = new Blockly.ChangeMode(math_arithmeticBlock15);
        var variables_getBlock16 = math_arithmeticChangemode15.appendNoneMutationOutput('variables_get', 1);
        variables_getBlock16.setTitleValue('now','VAR');
        var variables_getBlock17 = math_arithmeticChangemode15.appendNoneMutationOutput('variables_get', 2);
        variables_getBlock17.setTitleValue('duration','VAR');
        controls_whileUntilChangemode12.changeStatementInputTo('DO');
        var midlevel_ros_move_base_twistBlock18 = controls_whileUntilChangemode12.appendNoneMutationStatement('midlevel_ros_move_base_twist');
        var midlevel_ros_move_base_twistChangemode18 = new Blockly.ChangeMode(midlevel_ros_move_base_twistBlock18);
        var math_numberBlock19 = midlevel_ros_move_base_twistChangemode18.appendNoneMutationOutput('math_number', 1);
        math_numberBlock19.setTitleValue(directionValues[0],'NUM');
        var math_numberBlock20 = midlevel_ros_move_base_twistChangemode18.appendNoneMutationOutput('math_number', 2);
        math_numberBlock20.setTitleValue(directionValues[1],'NUM');
        var math_numberBlock21 = midlevel_ros_move_base_twistChangemode18.appendNoneMutationOutput('math_number', 3);
        math_numberBlock21.setTitleValue(directionValues[2],'NUM');
        var math_numberBlock22 = midlevel_ros_move_base_twistChangemode18.appendNoneMutationOutput('math_number', 4);
        math_numberBlock22.setTitleValue(directionValues[3],'NUM');
        var math_numberBlock23 = midlevel_ros_move_base_twistChangemode18.appendNoneMutationOutput('math_number', 5);
        math_numberBlock23.setTitleValue(directionValues[4],'NUM');
        var math_numberBlock24 = midlevel_ros_move_base_twistChangemode18.appendNoneMutationOutput('math_number', 6);
        math_numberBlock24.setTitleValue(directionValues[5],'NUM');
        var midlevel_ros_move_base_twistBlock25 = block.changemode.appendNoneMutationStatement('midlevel_ros_move_base_twist');
        var midlevel_ros_move_base_twistChangemode25 = new Blockly.ChangeMode(midlevel_ros_move_base_twistBlock25);
        var math_numberBlock26 = midlevel_ros_move_base_twistChangemode25.appendNoneMutationOutput('math_number', 1);
        math_numberBlock26.setTitleValue('0','NUM');
        var math_numberBlock27 = midlevel_ros_move_base_twistChangemode25.appendNoneMutationOutput('math_number', 2);
        math_numberBlock27.setTitleValue('0','NUM');
        var math_numberBlock28 = midlevel_ros_move_base_twistChangemode25.appendNoneMutationOutput('math_number', 3);
        math_numberBlock28.setTitleValue('0','NUM');
        var math_numberBlock29 = midlevel_ros_move_base_twistChangemode25.appendNoneMutationOutput('math_number', 4);
        math_numberBlock29.setTitleValue('0','NUM');
        var math_numberBlock30 = midlevel_ros_move_base_twistChangemode25.appendNoneMutationOutput('math_number', 5);
        math_numberBlock30.setTitleValue('0','NUM');
        var math_numberBlock31 = midlevel_ros_move_base_twistChangemode25.appendNoneMutationOutput('math_number', 6);
        math_numberBlock31.setTitleValue('0','NUM');
    },
    disposeChangeMode: function() {
        var block = this;
        if (!block.getInput(block.inputNameDropdown)) {
            block.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_BASE_DIRECTION);
            if (block.dropdownValue) {
                block.dropdown.setValue(block.dropdownValue);
            }
            block.appendValueInput(block.inputNameDropdown)
                .setAlign(Blockly.ALIGN_RIGHT)
                .appendTitle('direction')
                .appendTitle(block.dropdown, 'MODE')
                .appendTitle('duration (sec)');
        }
                
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};

Blockly.Language.highlevel_move_gripper_string = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_FINGER_POS);
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.inputNameDropdown = 'DROPDOWN';
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;
            
            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('move fingers');
        this.appendDummyInput(this.inputNameDropdown).appendTitle(this.dropdown, 'MODE');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        var thisBlock = this;
        this.setTooltip(
                        'Perform grasp and release actions with the robot fingers.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* ' + Blockly.ChangeModeTooltip + '\n' +
                        '* Finger movement (dropdown): Select a finger movement from the list'
                        );
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('STATE');
        container.setAttribute('changemodestate', state);
        if (this.dropdownValue) {
            container.setAttribute('dropdownstate', this.dropdownValue);
        }
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var block = this;
        
        var changeModeState = xmlElement.getAttribute('changemodestate');
        if(changeModeState == 'TRUE') {
            if(!block.getInput(block.changemode.appendInput)) {
                block.appendStatementInput(block.changemode.appendInput);
            }
            if (block.getInput(block.inputNameDropdown)) {
                block.removeInput(block.inputNameDropdown);
                delete block.dropdown;
            }
            block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        }
        
        var dropdownState = xmlElement.getAttribute('dropdownstate');
        if (dropdownState) {
            block.dropdownValue = dropdownState;
        }
    },
    changeModeToDom: function() {
        var state = this.getTitleValue('STATE');
        return this.changemode.changeModeToDom(state);
    },
    domToChangeMode: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if (changeModeState == 'TRUE') {
            this.changemode.domToChangeMode(xmlElement);
        }
    },
    showChangeMode: function() {
        var block = this;
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        
        if(block.getInput(block.inputNameDropdown)) {
            block.removeInput(block.inputNameDropdown);
            block.dropdownValue = block.dropdown.getValue();
            delete block.dropdown;
        }
        var try_catchBlock0 = block.changemode.appendNoneMutationStatement('try_catch');
        try_catchBlock0.setTitleValue('e','VAR');
        var try_catchChangemode0 = new Blockly.ChangeMode(try_catchBlock0);
        try_catchChangemode0.changeStatementInputTo('TRY');
        var variables_setBlock1 = try_catchChangemode0.appendNoneMutationStatement('variables_set');
        variables_setBlock1.setTitleValue('gripper_values','VAR');
        var variables_setChangemode1 = new Blockly.ChangeMode(variables_setBlock1);
        var lowlevel_get_paramBlock2 = variables_setChangemode1.appendNoneMutationOutput('lowlevel_get_param', 1);
        lowlevel_get_paramBlock2.setTitleValue('OPEN','MODE');
        var midlevel_move_gripperBlock3 = try_catchChangemode0.appendNoneMutationStatement('midlevel_move_gripper');
        midlevel_move_gripperBlock3.setTitleValue('FALSE','STATE');
        var midlevel_move_gripperChangemode3 = new Blockly.ChangeMode(midlevel_move_gripperBlock3);
        var lists_getIndexBlock4 = midlevel_move_gripperChangemode3.appendNoneMutationOutput('lists_getIndex', 1);
        lists_getIndexBlock4.setTitleValue('GET','MODE');
        lists_getIndexBlock4.setTitleValue('FROM_START','WHERE');
        var lists_getIndexChangemode4 = new Blockly.ChangeMode(lists_getIndexBlock4);
        var variables_getBlock5 = lists_getIndexChangemode4.appendNoneMutationOutput('variables_get', 1);
        variables_getBlock5.setTitleValue('gripper_values','VAR');
        var math_numberBlock6 = lists_getIndexChangemode4.appendNoneMutationOutput('math_number', 2);
        math_numberBlock6.setTitleValue('1','NUM');
        var lists_getIndexBlock7 = midlevel_move_gripperChangemode3.appendNoneMutationOutput('lists_getIndex', 2);
        lists_getIndexBlock7.setTitleValue('GET','MODE');
        lists_getIndexBlock7.setTitleValue('FROM_START','WHERE');
        var lists_getIndexChangemode7 = new Blockly.ChangeMode(lists_getIndexBlock7);
        var variables_getBlock8 = lists_getIndexChangemode7.appendNoneMutationOutput('variables_get', 1);
        variables_getBlock8.setTitleValue('gripper_values','VAR');
        var math_numberBlock9 = lists_getIndexChangemode7.appendNoneMutationOutput('math_number', 2);
        math_numberBlock9.setTitleValue('2','NUM');
        try_catchChangemode0.changeStatementInputTo('CATCH');
        var lowlevel_ros_logBlock10 = try_catchChangemode0.appendNoneMutationStatement('lowlevel_ros_log');
        lowlevel_ros_logBlock10.setTitleValue('ERROR','MODE');
        var lowlevel_ros_logChangemode10 = new Blockly.ChangeMode(lowlevel_ros_logBlock10);
        var variables_getBlock11 = lowlevel_ros_logChangemode10.appendNoneMutationOutput('variables_get', 1);
        variables_getBlock11.setTitleValue('e','VAR');
    },
    disposeChangeMode: function() {
        var block = this;
        
        if(!block.getInput(block.inputNameDropdown)) {
            block.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_FINGER_POS);
            if (block.dropdownValue) {
                block.dropdown.setValue(block.dropdownValue);
            }
            block.appendDummyInput(block.inputNameDropdown)
                .appendTitle(block.dropdown, 'MODE');
        }
                
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};

Blockly.Language.highlevel_move_arm_joint_string = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_PRE_DEF_ARM_POS);
        this.inputNameDropdown = 'DROPDOWN';
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;

            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('move robot arm to');
        this.appendDummyInput(this.inputNameDropdown)
            .appendTitle(this.dropdown, 'MODE');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip(
                'Move the robot arm to a predefined position.\n' +
                'Note: The arm does neither follow a predefined path nor perform a collision check, so that it might bump into obstacles.\n' +
                '---\n' +
                'Fields:\n' +
                '* ' + Blockly.ChangeModeTooltip + '\n' +
                '* Arm movement (dropdown): Select an arm position from the list.'
        );
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('STATE');
        container.setAttribute('changemodestate', state);
        if (this.dropdownValue) {
            container.setAttribute('dropdownstate', this.dropdownValue);
        }
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var block = this;
        
        var changeModeState = xmlElement.getAttribute('changemodestate');
        if (changeModeState == 'TRUE') {
            if(!block.getInput(this.changemode.appendInput)) {
                block.appendStatementInput(this.changemode.appendInput);
            }
            
            if (block.getInput(block.inputNameDropdown)) {
                block.removeInput(block.inputNameDropdown);
                delete block.dropdown;
            }
            block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        }
        
        var dropdownState = xmlElement.getAttribute('dropdownstate');
        if (dropdownState) {
            block.dropdownValue = dropdownState;
        }
    },
    changeModeToDom: function() {
        var state = this.getTitleValue('STATE');
        return this.changemode.changeModeToDom(state);
    },
    domToChangeMode: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if (changeModeState == 'TRUE') {
            this.changemode.domToChangeMode(xmlElement);
        }
    },
    showChangeMode: function() {
        var block = this;
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
                
        if(block.getInput(block.inputNameDropdown)) {
            block.removeInput(block.inputNameDropdown);
            block.dropdownValue = block.dropdown.getValue();
            delete block.dropdown;
        }
        
        var midlevel_move_arm_joint_positionBlock0 = block.changemode.appendNoneMutationStatement('midlevel_move_arm_joint_position');
        midlevel_move_arm_joint_positionBlock0.setTitleValue('FALSE','STATE');
        var midlevel_move_arm_joint_positionChangemode0 = new Blockly.ChangeMode(midlevel_move_arm_joint_positionBlock0);
        var lowlevel_get_paramBlock1 = midlevel_move_arm_joint_positionChangemode0.appendNoneMutationOutput('lowlevel_get_param', 1);
        lowlevel_get_paramBlock1.setTitleValue(block.dropdownValue,'MODE');
    },
    disposeChangeMode: function() {
        var block = this;
                
        if(!block.getInput(block.inputNameDropdown)) {
            block.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_PRE_DEF_ARM_POS);
            if (block.dropdownValue) {
                block.dropdown.setValue(block.dropdownValue);
            }
            block.appendDummyInput(block.inputNameDropdown)
                .appendTitle(block.dropdown, 'MODE');
        }
        
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};

Blockly.Language.highlevel_move_arm_through_ik = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var inputRef = 'REF';
        var inputPose6d = 'POSE6D';
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;

            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('move robot arm to');
        this.appendValueInput(inputPose6d)
            .appendTitle('6D pose')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck(['Pose6D', Array]);
        this.setOutput(true, Boolean);
        this.setTooltip(
                        'Move robot arm to a cartesian pose.\n' +
                        'Note: No obstacle avoidance is implemented yet.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* ' + Blockly.ChangeModeTooltip + '\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* 6D pose (value input): xyz cartesian position and orientation (roll pitch yaw). \n' +
                        'A list with six entries is required, but the connection type is only to restrict to \'Array\' (any list length is possible).\n' +
                        '---\n' +
                        'Output:\n' +
                        '* If succeeded: True. Else: False.' +
                        'The output type is \'Boolean\'.'
                        );
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('STATE');
        container.setAttribute('changemodestate', state);
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        var inputRef = 'REF';
        var inputPose6d = 'POSE6D';
        if(changeModeState == 'TRUE') {
            if (block.getInput(inputPose6d)) {
                block.removeInput(inputPose6d);
            }
            if(!block.getInput(block.changemode.appendInput)) {
                block.appendValueInput(block.changemode.appendInput);
            }
            block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        }
    },
    changeModeToDom: function() {
        var state = this.getTitleValue('STATE');
        return this.changemode.changeModeToDom(state);
    },
    domToChangeMode: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if (changeModeState == 'TRUE') {
            this.changemode.domToChangeMode(xmlElement);
        }
    },
    showChangeMode: function() {
        var block = this;
        var inputPose6d = 'POSE6D';
        
        block.changemode.setChangeMode(true, 'output');
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        var attachedBlock6d = block.getInputTargetBlock(inputPose6d);
        
        if(block.getInput(inputPose6d)) {
            block.removeInput(inputPose6d);
        }
                
        var procedure = block.changemode.createProcedure(true, [['arguments_', ['pose6D', 'frame_of_reference']]]);
        var procedureBlock = procedure[0];
        var callBlock = procedure[1];
        // rename procedure
        var name = Blockly.Procedures.findLegalName('move_robot_arm_procedure', procedureBlock);
        procedureBlock.setTitleValue(name, 'NAME');
        block.changemode.addToProcedureMonitor(name);
        block.changemode.appendProcedureCall(callBlock, 1);
        
        var changemodeCall = new Blockly.ChangeMode(callBlock);
        changemodeCall.reconnectOutputConnection(attachedBlock6d, 1);
        block.changemode.addToReconnectMonitor(callBlock, 1, 1);
        var midlevel_reference_frameBlock = changemodeCall.appendNoneMutationOutput('midlevel_reference_frame', 2);
        midlevel_reference_frameBlock.setTitleValue('ARM','MODE');
                
        var changemodeProcedure = new Blockly.ChangeMode(procedureBlock, 'STACK');
        var controlBlock = changemodeProcedure.appendMutationStatement('controls_if', [['elseifCount_', 0],['elseCount_', 1]]);
        var changemodeControl = new Blockly.ChangeMode(controlBlock,'DO0');
        var ikCheckerBlock = changemodeControl.appendNoneMutationOutput('midlevel_ik_checker', 1);
        var changemodeIkChecker = new Blockly.ChangeMode(ikCheckerBlock);
        changemodeIkChecker.appendNoneMutationOutput('variables_get', 1).setTitleValue('frame_of_reference', 'VAR');
        changemodeIkChecker.appendNoneMutationOutput('variables_get', 2).setTitleValue('pose6D', 'VAR');
        var setBlock = changemodeControl.appendNoneMutationStatement('variables_set');
        setBlock.setTitleValue('joint_angles', 'VAR');
        var changemodeIf = new Blockly.ChangeMode(setBlock);
        var ikSolverBlock = changemodeIf.appendNoneMutationOutput('midlevel_ik_solver', 1);
        var changemodeIkSolver = new Blockly.ChangeMode(ikSolverBlock);
        changemodeIkSolver.appendNoneMutationOutput('variables_get', 1).setTitleValue('frame_of_reference', 'VAR');
        changemodeIkSolver.appendNoneMutationOutput('variables_get', 2).setTitleValue('pose6D', 'VAR');
        var moveArmJointPositionBlock = changemodeControl.appendNoneMutationStatement('midlevel_move_arm_joint_position');
        new Blockly.ChangeMode(moveArmJointPositionBlock).appendNoneMutationOutput('variables_get',1).setTitleValue('joint_angles', 'VAR');
        var setTrueBlock = changemodeControl.appendNoneMutationStatement('variables_set');
        setTrueBlock.setTitleValue('ik_result', 'VAR');
        var changemodeValue = new Blockly.ChangeMode(setTrueBlock);
        changemodeValue.appendNoneMutationOutput('logic_boolean', 1).setTitleValue('TRUE','BOOL');
        changemodeControl.changeStatementInputTo('ELSE');
        var setFalseBlock = changemodeControl.appendNoneMutationStatement('variables_set');
        setFalseBlock.setTitleValue('ik_result', 'VAR');
        var changemodeValue = new Blockly.ChangeMode(setFalseBlock);
        changemodeValue.appendNoneMutationOutput('logic_boolean', 1).setTitleValue('FALSE','BOOL');
                
        changemodeProcedure.appendNoneMutationOutput('variables_get',1).setTitleValue('ik_result', 'VAR');
    },
    disposeChangeMode: function() {
        var block = this;
        var inputPose6d = 'POSE6D';
        if (!block.getInput(inputPose6d)) {
            block.appendValueInput(inputPose6d)
                .appendTitle('6D pose')
                .setAlign(Blockly.ALIGN_RIGHT)
                .setCheck(['Pose6D', Array]);
        }
        // To make sure that the reconnectMonitor is intuitiv
        block.moveInputBefore(inputPose6d, block.changemode.appendInput);
        
        block.changemode.setChangeMode(false, 'output');
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
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

Blockly.Language.highlevel_find_cube = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        var dropdown_colour = new Blockly.FieldDropdown(Blockly.CONFIG_CUBE_COLORS);    
        this.appendDummyInput()
            .appendTitle('find')
            .appendTitle(dropdown_colour,'MODE')
            .appendTitle('cube');
        this.setInputsInline(true);
        this.setTooltip(
                        'Detect a cube with the specified color.\n' +
                        '---\n' + 
                        'Fields:\n' +
                        'color (dropdown): Select a color from the list.\n' +
                        '---\n' +
                        'Output:\n' + 
                        'The output is a ROS based message type (geometry_msgs/PoseStamped).\n' +
                        'The output type is not restricted yet.'
                        );
        this.setOutput(true,null);
    }
};

Blockly.Language.highlevel_check_wall = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_HIGH_LEVEL_COLOUR);
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;

            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.setOutput(true, Boolean);
        this.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_WALL);
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('check for a wall');
        this.appendDummyInput('DROPDOWN')
            .appendTitle('to the')
            .appendTitle(this.dropdown, 'MODE');
        this.setInputsInline(true);
        this.setTooltip(
                        'Test if a wall (or an obstacle in general) is nearer than a maximum (fixed) distance in the specified direction.\n' +
                        '---\n' +
                        'Fields:\n' +
                        'wall direction (dropdown): Select the direction of interest from the list.\n' +
                        '---\n' +
                        'Output:\n' +
                        'Return True if a wall or obstacle is in range and otehrwiese False.\n' +
                        'The output type is \'Boolean\''
                        );
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('STATE');
        container.setAttribute('changemodestate', state);
        if (this.dropdownValue) {
            container.setAttribute('dropdownstate', this.dropdownValue);
        }
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if(changeModeState == 'TRUE') {
            if(!block.getInput(block.changemode.appendInput)) {
                block.appendValueInput(block.changemode.appendInput);
            }
            if (block.getInput('DROPDOWN')) {
                block.removeInput('DROPDOWN');
                delete block.dropdown
            }
            block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        }
        
        var dropdownState = xmlElement.getAttribute('dropdownstate');
        if (dropdownState) {
            block.dropdownValue = dropdownState;
        }
    },
    changeModeToDom: function() {
        var state = this.getTitleValue('STATE');
        return this.changemode.changeModeToDom(state);
    },
    domToChangeMode: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if (changeModeState == 'TRUE') {
            this.changemode.domToChangeMode(xmlElement);
        }
    },
    showChangeMode: function() {
        var block = this;
        block.setInputsInline(false);
        block.changemode.setChangeMode(true, 'output');
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        
        if (block.getInput('DROPDOWN')) {
            block.removeInput('DROPDOWN');
            block.dropdownValue = block.dropdown.getValue();
            delete block.dropdown;
        }
        
        var midlevel_check_wallBlock = block.changemode.appendNoneMutationOutput('midlevel_check_wall', 1);
        midlevel_check_wallBlock.setTitleValue(block.dropdownValue,'MODE');
        var midlevel_check_wallChangemode = new Blockly.ChangeMode(midlevel_check_wallBlock);
        var math_numberBlock = midlevel_check_wallChangemode.appendNoneMutationOutput('math_number', 1);
        math_numberBlock.setTitleValue(Blockly.CONFIG_WALL_DISTANCE,'NUM');
    },
    disposeChangeMode: function() {
        var block = this;
        
        if (!block.getInput('DROPDOWN')) {
            block.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_WALL);
            if (block.dropdownValue) {
                block.dropdown.setValue(block.dropdownValue);
            }
            block.appendDummyInput('DROPDOWN')
                .appendTitle('to the')
                .appendTitle(block.dropdown, 'MODE');
        }
        block.setInputsInline(true);
        block.changemode.setChangeMode(false, 'output');
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
}
