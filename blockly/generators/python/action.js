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
 *  * @fileoverview Generating Python for action blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
 Blockly.Python = Blockly.Generator.get('Python');
 
if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.action_simple_action_client = function() {
    Blockly.Python.definitions_['import_actionlib'] = 'import actionlib';
    Blockly.Python.RESERVED_WORDS_ += 'actionlib,';
    
    var action_server_name = this.getTitleValue('ACTION_SERVER_NAME');
    var action = Blockly.Python.valueToCode(this, 'ACTION', Blockly.Python.ORDER_NONE) || 'None';
    var code = 'actionlib.SimpleActionClient("' + action_server_name +'", ' + action + ')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.action_client_wait_for_server = function() {
    var client = Blockly.Python.valueToCode(this, 'CLIENT', Blockly.Python.ORDER_NONE);
    var code = client + '.wait_for_server()\n';

    return code;
};

Blockly.Python.action_client_wait_for_result = function() {
    var client = Blockly.Python.valueToCode(this, 'CLIENT', Blockly.Python.ORDER_NONE);
    var code = client + '.wait_for_result()\n';

    return code;
};

Blockly.Python.action_client_send_goal = function() {
    var client = Blockly.Python.valueToCode(this, 'CLIENT', Blockly.Python.ORDER_NONE);
    var goal = Blockly.Python.valueToCode(this, 'GOAL', Blockly.Python.ORDER_NONE);
    var code = client + '.send_goal(' + goal + ')\n';

    return code;
};

Blockly.Python.action_move_base_action = function() {
    Blockly.Python.definitions_['from_move_base_msgs.msg_import_*'] = 'from move_base_msgs.msg import *';
    Blockly.Python.RESERVED_WORDS_ += 'move_base_msgs.msg,';
    var code = 'MoveBaseAction';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
}
