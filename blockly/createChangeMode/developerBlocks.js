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
 * @fileoverview Developer blocks to get changeMode children.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
goog.provide('Blockly.Language.developchangemode');

goog.require('Blockly.Language');

Blockly.Language.developer_changemodeStatement = {
    helpUrl: null,
    init: function() {
        this.setColour(1);
        this.appendDummyInput('APP_FRAME').appendTitle('changemode statement');
        this.appendStatementInput('APP_STACK');
        this.setPreviousStatement(false);
        this.setNextStatement(false);
        this.setMutator(new Blockly.Mutator(['developer_changemode_item']));
        this.itemCount_ = 0;
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        if (this.itemCount_) {
            container.setAttribute('items', this.itemCount_);
        }
        
        return container;
    },
    domToMutation: function(container) {
        for (var i = 0; i < this.itemCount_; i++) {
            this.removeInput('ADD' + i);
        }
        this.removeInput('APP_STACK');
        
        this.itemCount_ = window.parseInt(container.getAttribute('items'), 10);
        for (var i = 0; i < this.itemCount_; i++) {
            var input = this.appendDummyInput('ADD' + i)
                            .appendTitle('monitor value input area')
                            .appendTitle(new Blockly.FieldTextInput(''), 'TEXT' + i);
        }
        this.appendStatementInput('APP_STACK');
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'developer_changemode_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        for (var i = 0; i < this.itemCount_; i++) {
            var itemBlock = new Blockly.Block(workspace, 'developer_changemode_item');
            itemBlock.initSvg();
            connection.connect(itemBlock.previousConnection);
            connection = itemBlock.nextConnection;
        }
        return containerBlock;
    },
    compose: function(containerBlock) {
        // Disconnect all input blocks and remove all inputs.
        for (var i = this.itemCount_ - 1; i >= 0; i--) {
            this.removeInput('ADD' + i);
        }
        this.itemCount_ = 0;
        
        if (this.getInput('APP_STACK')) {
            this.removeInput('APP_STACK');
        }
        // Rebuild the block's inputs.
        var itemBlock = containerBlock.getInputTargetBlock('STACK');
        while (itemBlock) {
            var text = 'name' + (this.itemCount_+1)
            if(itemBlock.textValue_) {
                text = itemBlock.textValue_;
            }
            var input = this.appendDummyInput('ADD' + this.itemCount_)
                            .appendTitle('monitor value input area')
                            .appendTitle(new Blockly.FieldTextInput(text), 'TEXT' + this.itemCount_);
            
            this.itemCount_++;
            itemBlock = itemBlock.nextConnection && itemBlock.nextConnection.targetBlock();
        }
        
        var input = this.appendStatementInput('APP_STACK');
        if(this.valueConnection_) {
            input.connection.connect(this.valueConnection_);
        }
    },
    saveConnections: function(containerBlock) {
        var textNumber = 0;
        
        var statementBlock = containerBlock.getInputTargetBlock('STACK');
        while(statementBlock) {
            statementBlock.textValue_ = this.getTitleValue('TEXT' + textNumber);
            textNumber++;
            statementBlock = statementBlock.nextConnection && statementBlock.nextConnection.targetBlock();
        }
        var input = this.getInput('APP_STACK');
        this.valueConnection_ = input && input.connection.targetConnection; 
    }
};

Blockly.Language.developer_changemode_container = {
    // Container.
    init: function() {
        this.setColour(1);
        this.appendDummyInput()
            .appendTitle('value input monitor');
        this.appendStatementInput('STACK');
        this.contextMenu = false;
  }
};

Blockly.Language.developer_changemode_item = {
    init: function() {
        this.setColour(1);
    this.appendDummyInput().appendTitle('value input');
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.contextMenu = false;
    }
};

Blockly.Language.developer_changemodeOutput = {
    helpUrl: null,
    init: function() {
        this.setColour(1);
        this.appendDummyInput('APP_FRAME').appendTitle('changemode ouput');
        this.appendValueInput('APP_VALUE');
        this.setPreviousStatement(false);
        this.setNextStatement(false);
        this.setMutator(new Blockly.Mutator(['developer_changemode_item']));
        this.itemCount_ = 0;
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        if (this.itemCount_) {
            container.setAttribute('items', this.itemCount_);
        }
        
        return container;
    },
    domToMutation: function(container) {
        for (var i = 0; i < this.itemCount_; i++) {
            this.removeInput('ADD' + i);
        }
        this.removeInput('APP_VALUE');
        
        this.itemCount_ = window.parseInt(container.getAttribute('items'), 10);
        for (var i = 0; i < this.itemCount_; i++) {
            var input = this.appendDummyInput('ADD' + i)
                            .appendTitle('monitor value input area')
                            .appendTitle(new Blockly.FieldTextInput(''), 'TEXT' + i);
        }
        this.appendValueInput('APP_VALUE');
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'developer_changemode_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        for (var i = 0; i < this.itemCount_; i++) {
            var itemBlock = new Blockly.Block(workspace, 'developer_changemode_item');
            itemBlock.initSvg();
            connection.connect(itemBlock.previousConnection);
            connection = itemBlock.nextConnection;
        }
        return containerBlock;
    },
    compose: function(containerBlock) {
        // Disconnect all input blocks and remove all inputs.
        for (var i = this.itemCount_ - 1; i >= 0; i--) {
            this.removeInput('ADD' + i);
        }
        this.itemCount_ = 0;
        
        if (this.getInput('APP_VALUE')) {
            this.removeInput('APP_VALUE');
        }
        // Rebuild the block's inputs.
        var itemBlock = containerBlock.getInputTargetBlock('STACK');
        while (itemBlock) {
            var text = 'name' + (this.itemCount_+1)
            if(itemBlock.textValue_) {
                text = itemBlock.textValue_;
            }
            var input = this.appendDummyInput('ADD' + this.itemCount_)
                            .appendTitle('monitor value input area')
                            .appendTitle(new Blockly.FieldTextInput(text), 'TEXT' + this.itemCount_);
            
            this.itemCount_++;
            itemBlock = itemBlock.nextConnection && itemBlock.nextConnection.targetBlock();
        }
        
        var input = this.appendValueInput('APP_VALUE');
        if(this.valueConnection_) {
            input.connection.connect(this.valueConnection_);
        }
    },
    saveConnections: function(containerBlock) {
        var textNumber = 0;
        
        var statementBlock = containerBlock.getInputTargetBlock('STACK');
        while(statementBlock) {
            statementBlock.textValue_ = this.getTitleValue('TEXT' + textNumber);
            textNumber++;
            statementBlock = statementBlock.nextConnection && statementBlock.nextConnection.targetBlock();
        }
        var input = this.getInput('APP_VALUE');
        this.valueConnection_ = input && input.connection.targetConnection; 
    }
};

Blockly.Language.developer_reconnectOutputConnection = {
    helpUrl: null,
    init: function() {
        this.setColour(1);
        this.appendDummyInput('')
            .appendTitle('reconnect output connection')
            .appendTitle(new Blockly.FieldTextInput(''), 'TEXT');
        this.setOutput(true, null);
    }
};
