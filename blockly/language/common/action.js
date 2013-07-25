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
 * @fileoverview Action blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';

Blockly.Language.action_simple_action_client = {
    // Make use of a SimpleActionClient (actionlib) 
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ACTION_COLOUR);
        this.appendDummyInput().appendTitle('simple action client')
        this.appendValueInput('ACTION')
            .appendTitle('action server name:')
            .appendTitle(new Blockly.FieldTextInput('server_name',null),'ACTION_SERVER_NAME')
            .appendTitle('action:');
        this.setOutput(true, Blockly.LANG_CONNECTION_TYPE_SIMPLE_ACTION_CLIENT);
        this.setTooltip('');
    }
};

Blockly.Language.action_client_wait_for_server = {
    // Allows a client to wait for a server to come up (blocking)
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ACTION_COLOUR);
        this.appendDummyInput().appendTitle('wait for server');
        this.appendValueInput('CLIENT')
            .appendTitle('client:')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.action_client_wait_for_result = {
    // Allows a client to wait for results (blocking)
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ACTION_COLOUR);
        this.appendDummyInput().appendTitle('wait for result');
        this.appendValueInput('CLIENT')
            .appendTitle('client:')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.action_client_send_goal = {
    // Allows a client to send a goal message
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ACTION_COLOUR);
        this.appendDummyInput().appendTitle('send goal');
        this.appendValueInput('CLIENT')
            .appendTitle('client:')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.appendValueInput('GOAL')
            .appendTitle('goal message:')
            .setAlign(Blockly.ALIGN_RIGHT);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.action_move_base_action = {
    // An action to move the robot base
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_ACTION_COLOUR);
        this.appendDummyInput().appendTitle('MoveBaseAction');
        this.setOutput(true, 'MoveBaseAction');
        this.setTooltip('');
    }
};
