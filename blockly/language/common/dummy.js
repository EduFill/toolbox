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
 * @fileoverview Dummy blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';

goog.provide('Blockly.Language.dummy');

goog.require('Blockly.Language');

Blockly.Language.EDUFILL_CHANGEMODE_OPERATORS = 
    [[Blockly.EDUFILL_CHANGEMODE_OPERATORS_CLOSE, 'CLOSE'],
     [Blockly.EDUFILL_CHANGEMODE_OPERATORS_OPEN, 'OPEN']];

Blockly.Language.output_test = {
    helpUrl: null,
    init: function() {
       this.setColour(Blockly.LANG_DUMMY_COLOUR); 
       this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
       this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
       var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState){
            var block = this.sourceBlock_;
            
            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput('BASE')
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('block','TITLE'); // Fixed and never changing part of the block 
        this.appendValueInput('TEST').appendTitle('test'); 
        this.setOutput(true, null);
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
        if(changeModeState == 'TRUE') {
            if (block.getInput('TEST')) {
                block.removeInput('TEST');
            }
            
            if(!block.getInput(this.changemode.appendInput)) {
                block.appendValueInput(this.changemode.appendInput);
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
    block.changemode.setChangeMode(true, 'output');
    block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
    var blockAttachedAtTEST = block.getInputTargetBlock('TEST');
    if (block.getInput('TEST')) {
        block.removeInput('TEST');
    }
    var procedure = block.changemode.createProcedure(true, [['arguments_', ['x']]]);
    var procedureBlock = procedure[0];
    var callBlock = procedure[1];
    // rename procedure
    var name = Blockly.Procedures.findLegalName('test', procedureBlock);
    procedureBlock.setTitleValue(name, 'NAME');
    block.changemode.addToProcedureMonitor(name);
    block.changemode.appendProcedureCall(callBlock, 1);
    var changemodeCall = new Blockly.ChangeMode(callBlock);

    changemodeCall.reconnectOutputConnection(blockAttachedAtTEST , 1);
    block.changemode.addToReconnectMonitor(callBlock, 1, 1);
    var changemodeProcedure = new Blockly.ChangeMode(procedureBlock, 'STACK');
    var controls_ifBlock0 = changemodeProcedure.appendMutationStatement('controls_if', [['elseCount_',1]] );
    var controls_ifChangemode0 = new Blockly.ChangeMode(controls_ifBlock0);
    var midlevel_ik_checkerBlock1 = controls_ifChangemode0.appendNoneMutationOutput('midlevel_ik_checker', 1);
    var midlevel_ik_checkerChangemode1 = new Blockly.ChangeMode(midlevel_ik_checkerBlock1);
    var variables_getBlock2 = midlevel_ik_checkerChangemode1.appendNoneMutationOutput('variables_get', 1);
    variables_getBlock2.setTitleValue('frame_of_reference','VAR');
    var variables_getBlock3 = midlevel_ik_checkerChangemode1.appendNoneMutationOutput('variables_get', 2);
    variables_getBlock3.setTitleValue('poseList','VAR');
    controls_ifChangemode0.changeStatementInputTo('DO0');
    var variables_setBlock4 = controls_ifChangemode0.appendNoneMutationStatement('variables_set');
    variables_setBlock4.setTitleValue('joint_angles','VAR');
    var variables_setChangemode4 = new Blockly.ChangeMode(variables_setBlock4);
    var midlevel_ik_solverBlock5 = variables_setChangemode4.appendNoneMutationOutput('midlevel_ik_solver', 1);
    var midlevel_ik_solverChangemode5 = new Blockly.ChangeMode(midlevel_ik_solverBlock5);
    var variables_getBlock6 = midlevel_ik_solverChangemode5.appendNoneMutationOutput('variables_get', 1);
    variables_getBlock6.setTitleValue('frame_of_reference','VAR');
    var variables_getBlock7 = midlevel_ik_solverChangemode5.appendNoneMutationOutput('variables_get', 2);
    variables_getBlock7.setTitleValue('poseList','VAR');
    var midlevel_move_arm_joint_positionBlock8 = controls_ifChangemode0.appendNoneMutationStatement('midlevel_move_arm_joint_position');
    midlevel_move_arm_joint_positionBlock8.setTitleValue('FALSE','STATE');
    var midlevel_move_arm_joint_positionChangemode8 = new Blockly.ChangeMode(midlevel_move_arm_joint_positionBlock8);
    var variables_getBlock9 = midlevel_move_arm_joint_positionChangemode8.appendNoneMutationOutput('variables_get', 1);
    variables_getBlock9.setTitleValue('joint_angles','VAR');
    var variables_setBlock10 = controls_ifChangemode0.appendNoneMutationStatement('variables_set');
    variables_setBlock10.setTitleValue('ik_result','VAR');
    var variables_setChangemode10 = new Blockly.ChangeMode(variables_setBlock10);
    var logic_booleanBlock11 = variables_setChangemode10.appendNoneMutationOutput('logic_boolean', 1);
    logic_booleanBlock11.setTitleValue('TRUE','BOOL');
    controls_ifChangemode0.changeStatementInputTo('ELSE');
    var variables_setBlock12 = controls_ifChangemode0.appendNoneMutationStatement('variables_set');
    variables_setBlock12.setTitleValue('ik_result','VAR');
    var variables_setChangemode12 = new Blockly.ChangeMode(variables_setBlock12);
    var logic_booleanBlock13 = variables_setChangemode12.appendNoneMutationOutput('logic_boolean', 1);
    logic_booleanBlock13.setTitleValue('FALSE','BOOL');
    var variables_getBlock14 = changemodeProcedure.appendNoneMutationOutput('variables_get', 1);
    variables_getBlock14.setTitleValue('ik_result','VAR');
},
disposeChangeMode: function() {
    var block = this;
    if (!block.getInput('TEST')) {
        this.appendValueInput('TEST').appendTitle('test'); 
    }
    block.moveInputBefore('TEST', block.changemode.appendInput);
    block.changemode.setChangeMode(false, 'output');
    block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
}
};

Blockly.Language.test = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState){
            var block = this.sourceBlock_;
            
            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput('BASE')
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('block','TITLE'); // Fixed and never changing part of the block 
        this.dropdown = new Blockly.FieldDropdown([['test1', 'TEST1'], ['test2', 'TEST2']])
        this.appendDummyInput('DROPDOWN')
            .appendTitle('test to hide dropdown')
            .appendTitle(this.dropdown, 'MODE');
        delete this.dropdown;
        this.setInputsInline(false);
        this.setPreviousStatement(true);
        this.setNextStatement(false);
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
        if(changeModeState == 'TRUE') {
            if(!block.getInput(this.changemode.appendInput)) {
                block.appendStatementInput(this.changemode.appendInput);
            }
            if (block.getInput('DROPDOWN')) {
                block.removeInput('DROPDOWN');
                delete this.dropdown;
            }
            block.setTitleValue('frame', 'TITLE');
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
        //*
        var block = this;
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        block.setTitleValue('frame', 'TITLE');
                
        if (block.getInput('DROPDOWN')) {
            block.removeInput('DROPDOWN');
            delete block.dropdown;
        }
                
        // Create a new procedure with its call block
        var procedure = block.changemode.createProcedure(true, [['arguments_', ['x1','x2','x3']]]);
        var procedureBlock = procedure[0];
        var callBlock = procedure[1];
        // rename procedure
        var name = Blockly.Procedures.findLegalName('test', procedureBlock);
        procedureBlock.setTitleValue(name, 'NAME');
        block.changemode.addToProcedureMonitor(name);
                
        block.changemode.appendNoneMutationStatement('highlevel_move_arm_joint_string');
        block.changemode.appendNoneMutationStatement('highlevel_move_gripper_string').setTitleValue('OPEN','MODE');
        var storeBlock = block.changemode.appendNoneMutationStatement('store_in_db');
        storeBlock.setTitleValue('POSE','MODE');
        storeBlock.setTitleValue('object1','OBJECT_NAME');
        var listsBlock = new Blockly.ChangeMode(storeBlock, '').appendMutationOutput('lists_create_with', 1, [['itemCount_', 6]]);
        var changemodeValue = new Blockly.ChangeMode(listsBlock, '');
        changemodeValue.appendNoneMutationOutput('math_number', 1).setTitleValue('1','NUM');
        changemodeValue.appendNoneMutationOutput('math_number', 2).setTitleValue('1','NUM');
        changemodeValue.appendNoneMutationOutput('math_number', 3).setTitleValue('1','NUM');
        changemodeValue.appendNoneMutationOutput('math_number', 4).setTitleValue('90','NUM');
        changemodeValue.appendNoneMutationOutput('math_number', 5).setTitleValue('45','NUM');
        changemodeValue.appendNoneMutationOutput('math_number', 6).setTitleValue('90','NUM');
        block.changemode.appendNoneMutationStatement('highlevel_move_gripper_string').setTitleValue('CLOSE','MODE'); 
        var setBlock = block.changemode.appendNoneMutationStatement('variables_set');
        setBlock.renameVar('item','testProc');
        var changemodeSet = new Blockly.ChangeMode(setBlock, '');
        changemodeSet.appendProcedureCall(callBlock, 1);
    },
    disposeChangeMode: function() {
        var block = this;
        
        block.setTitleValue('block', 'TITLE');
        block.dropdown = new Blockly.FieldDropdown([['test1', 'TEST1'], ['test2', 'TEST2']])
        if (!block.getInput('DROPDOWN')) {
            block.appendDummyInput('DROPDOWN')
                .appendTitle('test to hide dropdown')
                .appendTitle(block.dropdown, 'MODE');
        }
        
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};

Blockly.Language.turn_robot = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('turn robot');
        this.appendValueInput('DEGREE').setCheck(Number).appendTitle('about');
        this.appendDummyInput().appendTitle('degree');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip('TODO');
    } 
}

Blockly.Language.store_in_db = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown(Blockly.Language.DB_TYPES, function(state){
            var dropdown_entries = this.getOptions_();
            for(var i = 0; i < dropdown_entries.length; i++) {
                if (dropdown_entries[i][0] == state) {
                    this.sourceBlock_.getInput('STORE_VALUE').setCheck(dropdown_entries[i][1]);
                    break;
                }
            }
            this.setText(state);
        });
        
        this.appendValueInput('STORE_VALUE')
            .appendTitle('store type')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('of')
            .appendTitle(new Blockly.FieldTextInput('object name',null),'OBJECT_NAME');
        this.setPreviousStatement(true);
        this.setNextStatement(true); 
        this.setTooltip('Store the actual value with the given type into a database according to the specified object name');
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var store_type = this.getTitleValue('MODE'); 
        container.setAttribute('store_type',store_type);
        return container;
    },
    domToMutation: function(xmlElement) {
        var store_type = (xmlElement.getAttribute('store_type'));
        this.getInput('STORE_VALUE').setCheck(store_type);
    }
};

Blockly.Language.load_from_db = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown(Blockly.Language.DB_TYPES, function(state){
            var dropdown_entries = this.getOptions_();
            for(var i = 0; i < dropdown_entries.length; i++) {
                if (dropdown_entries[i][0] == state) {
                    this.sourceBlock_.setOutput(true, dropdown_entries[i][1]);
                    break;
                }
            }
            this.setText(state);
        });
        this.appendDummyInput()
            .appendTitle('load type')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('of')
            .appendTitle(new Blockly.FieldTextInput('object name',null),'OBJECT_NAME');
        this.setOutput(true, 'POSE');
        this.setTooltip('Load a value from a database depending on the specified type and object name');
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var load_type = this.getTitleValue('MODE'); 
        container.setAttribute('load_type',load_type);
        return container;
    },
    domToMutation: function(xmlElement) {
        var load_type = (xmlElement.getAttribute('load_type'));
        this.setOutput(true,load_type);
    }
};

Blockly.Language.DB_TYPES =
    [['pose','POSE'],
     ['color', 'COLOR'],
     ['shape','SHAPE']];

Blockly.Language.move_robot_to_cube = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        // Take the same OPERATORS as been created for find_cube in midlevel.js
        var dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_CUBE_COLORS);
        this.appendDummyInput('DUMMY')
            .appendTitle('move robot to ')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('cube')
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('TODO');
    }
};

Blockly.Language.move_robot_to_position = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS);
        this.appendDummyInput()
            .appendTitle('move robot to')
            .appendTitle(dropdown, 'MODE');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip('TODO');
    } 
};

Blockly.Language.move_robot_to_position.OPERATORS = 
    [[Blockly.LANG_HIGH_LEVEL_MOVE_ROBOT_TO_POSITION_PLATFORM1, 'PLATFORM1'],
     [Blockly.LANG_HIGH_LEVEL_MOVE_ROBOT_TO_POSITION_ENTRANCE, 'ENTRANCE'],
     [Blockly.LANG_HIGH_LEVEL_MOVE_ROBOT_TO_POSITION_OTHERS, 'OTHERS']];

Blockly.Language.grasp_cube = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        // Take the same OPERATORS as been created for find_cube in highlevel.js
        var dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_CUBE_COLORS, function(colour) {
            if (colour == 'blue') {
                var newColour= Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_BLUE;
            }
            else if (colour == 'red') {
                var newColour= Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_RED;
            }
            else if (colour == 'green') {
                var newColour= Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_GREEN;
            }
            else if (colour == 'yellow') {
                var newColour= Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_YELLOW;
            }
            else {
                var newColour= Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_BLUE;
            }
            this.sourceBlock_.setColour(newColour);
            this.setText(colour);
        });
        this.appendDummyInput()
            .appendTitle('grasp')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('cube');
        this.setInputsInline(true);
        this.setTooltip('TODO');
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var colour = this.getTitleValue('MODE'); 
        container.setAttribute('colour',colour);
        return container;
    },
    domToMutation: function(xmlElement) {
        var colour = (xmlElement.getAttribute('colour'));
        if (colour == 'BLUE') {
            this.setColour(Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_BLUE);
        }
        else if (colour == 'RED') {
            this.setColour(Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_RED);
        }
        else if (colour == 'GREEN') {
            this.setColour(Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_GREEN);
        }
        else if (colour == 'YELLOW') {
            this.setColour(Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_YELLOW);
        }
        else {
            this.setColour(Blockly.LANG_MID_LEVEL_FIND_CUBE_COLOUR_BLUE);
        }
    }
};

Blockly.Language.place_cube = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown_destination = new Blockly.FieldDropdown(this.OPERATORS_DEST);
        var dropdown_reference = new Blockly.FieldDropdown(this.OPERATORS_REF);
        this.appendDummyInput()
            .appendTitle('place cube')
            .appendTitle(dropdown_destination, 'MODE_DEST')
            .appendTitle('')
            .appendTitle(dropdown_reference, 'MODE_REF');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip('TODO');
    } 
};

Blockly.Language.place_cube.OPERATORS_DEST = 
    [[Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_ONTO,'ONTO'],
     [Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_LEFT,'LEFT'],
     [Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_RIGHT,'RIGHT'],
     [Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_INFRONT,'INFRONT']];

Blockly.Language.place_cube.OPERATORS_REF = 
    [[Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_PLATFORM1,'PLATFORM1'],
     [Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_ROBOT,'ROBOT'],
     [Blockly.LANG_HIGH_LEVEL_PLACE_CUBE_OTHERS,'OTHERS']];

     
Blockly.Language.wall_follower = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS);
        this.appendDummyInput()
            .appendTitle('follow the')
            .appendTitle(dropdown, 'MODE')
            .appendTitle('wall until ...');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip('TODO');
    }
};

Blockly.Language.wall_follower.OPERATORS = 
    [[Blockly.LANG_HIGH_LEVEL_WALL_FOLLOWER_LEFT, 'LEFT'],
     [Blockly.LANG_HIGH_LEVEL_WALL_FOLLOWER_RIGHT, 'RIGHT']];
     
Blockly.Language.fetch_and_carry = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown_color = new Blockly.FieldDropdown(Blockly.Language.find_cube.OPERATORS);
        var dropdown_destination = new Blockly.FieldDropdown(Blockly.Language.place_cube.OPERATORS_DEST);
        var dropdown_reference = new Blockly.FieldDropdown(Blockly.Language.place_cube.OPERATORS_REF);
        this.appendDummyInput()
            .appendTitle('take')
            .appendTitle(dropdown_color, 'MODE_COL')
            .appendTitle('cube and put it')
            .appendTitle(dropdown_destination,'MODE_DEST')
            .appendTitle('')
            .appendTitle(dropdown_reference, 'MODE_REF');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip('TODO');
    }  
};

Blockly.Language.tftransform = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput()
            .appendTitle('get transform');
        this.appendValueInput('OBJ_FRAME')
            .setCheck(String)
            .appendTitle('from object frame:');
        this.appendValueInput('REF_FRAME')
            .setCheck(String)
            .appendTitle('to reference frame:');
        this.appendValueInput('DURATION')
            .setCheck(Number)
            .appendTitle('and search for');
        this.appendDummyInput()
            .appendTitle('second(s)');
        this.setOutput(true,Blockly.LANG_CONNECTION_TYPE_TRANSFORM);
        this.setInputsInline(true);
        this.setTooltip('');
    }   
};

Blockly.Language.base_placement = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('base placement');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('TODO');
    }
};

Blockly.Language.find_and_grasp_object = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS); 
        this.appendDummyInput()
            .appendTitle('find and grasp')
            .appendTitle(dropdown, 'MODE');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    } 
};

Blockly.Language.find_and_grasp_object.OPERATORS = 
    [[Blockly.LANG_HIGH_LEVEL_FIND_AND_GRASP_OBJECT_CUBE, 'CUBE']];

Blockly.Language.detect_and_reach_object = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown(this.OPERATORS); // the dropdown menu values are defined by find_and_grasp_object block
        this.appendDummyInput()
            .appendTitle('detect and reach')
            .appendTitle(dropdown, 'MODE');
        this.setInputsInline(true);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('TODO');
    }
};

Blockly.Language.detect_and_reach_object.OPERATORS = 
    [[Blockly.LANG_HIGH_LEVEL_FIND_AND_GRASP_OBJECT_CUBE, 'CUBE']];
