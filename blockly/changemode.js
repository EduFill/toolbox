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
 *  * @fileoverview Functionality to use ChangeMode.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
/* Convention for the usage of the ChangeMode functionality within a block. 
 * 
 * * Please use "this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');" instead of any other object instantiation 
 * * and "STATE" as language independent name of the checkbox in charge of the ChangeMode 
 * * to simplify unit tests and to reduce other confusions (e.g. using an own naming for each block, etc.)
 * 
 * example:
 * * this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
 * * var checkbox = new Blockly.FieldCheckbox([...]);
 * * ...
 * * [this.append{Value|Statement|Dummy}Input].[...].appendTitle(checkbox, 'STATE');
 * 
*/
 
 
Blockly.ChangeMode = function(block, appendInput) {
    this.block_ = block;
    if (typeof appendInput == 'undefined') {
        this.appendInput = '';
    }
    else {
        this.appendInput = appendInput;
    }
    this.statementConnection = null;
    this.newBlockEditable = true;
    this.newBlockDeletable = true;
    this.reconnectMonitor = [];
    this.procedureMonitor = [];
};

// Pre and post operations of changeMode. 
Blockly.ChangeMode.prototype.setChangeMode = function (isSet, type) {
    if (typeof type == 'undefined') {
        type = 'statement';
    }
    
    var block = this.block_;
    if(isSet) {
        if(type == 'statement') {    
            if(!block.getInput(this.appendInput)) {
                block.appendStatementInput(this.appendInput);
            }
        }
        else if(type == 'output') {
            if(!block.getInput(this.appendInput)) {
                block.appendValueInput(this.appendInput);
            }
        }
        else {alert('Unknown type within setChangeMode');}
        block.setWarningText('You activated the next level of complexity. If you are not familiar with that level feel free to deactivate it or investigate further.');
    }
    else {
        if(block.getInput(this.appendInput)) {
            this.reconnectAndRemoveChildren();
            this.removeProcedures();
            this.reconnectMonitor = [];
            this.procedureMonitor = [];
            block.removeInput(this.appendInput); 
        }
        block.setWarningText(null);
    }
};

Blockly.ChangeMode.prototype.changeStatementInputTo = function(appendInputName) {
    this.appendInput = appendInputName;
    this.statementConnection = null;
}

// Create Statement connection if not already existent.
Blockly.ChangeMode.prototype.createStatementConnection = function() {
    if (!this.statementConnection) {
        this.statementConnection = this.block_.getInput(this.appendInput).connection;
    }
    
    return this.statementConnection;
};

// Remove all children from the current block. 
Blockly.ChangeMode.prototype.removeChildren = function(){
    var block = this.block_;
    var statementBlock = block.getInputTargetBlock(this.appendInput);
    if (statementBlock) {
        statementBlock.dispose(false,false);
    }
    
    this.statementConnection = null;
};

// Should be called after all children have been deleted (to prevent survival of to be deleted procedures)
Blockly.ChangeMode.prototype.removeProcedures = function() {
    var allBlocks = this.block_.workspace.getAllBlocks(); 
    for (var i=0; i<this.procedureMonitor.length; i++) {
        var externalCallBlock = false;
        for (var j=0; j<allBlocks.length; j++) {
            var callBlockName = allBlocks[j].getProcedureCall && allBlocks[j].getProcedureCall();
            if (callBlockName == this.procedureMonitor[i]) {
                externalCallBlock = true;
            }
        }
        if(!externalCallBlock) {
            var topBlocks = this.block_.workspace.getTopBlocks(false);
            for (var k=0; k<topBlocks.length; k++) {
                if (topBlocks[k].getTitleValue('NAME') == this.procedureMonitor[i]) {
                    topBlocks[k].dispose();
                    break;
                }
            }
        }    
    }
    
};

Blockly.ChangeMode.getNumberOfValueInputs = function(inputBlock) {
    var numberOfValueInputs = 0;
    for (var i = 0; i < inputBlock.inputList.length; i++) {
        if (inputBlock.inputList[i].type == Blockly.INPUT_VALUE) {
            numberOfValueInputs++;
        }
    }
    
    return numberOfValueInputs;
};

Blockly.ChangeMode.getNamesOfValueInputs = function(inputBlock) {
    var namesOfValueInputs = [];
    for (var i = 0; i < inputBlock.inputList.length; i++) {
        if (inputBlock.inputList[i].type == Blockly.INPUT_VALUE) {
            namesOfValueInputs.push(inputBlock.inputList[i].name);
        }
    }
    return namesOfValueInputs;
}

Blockly.ChangeMode.getPositionInInputList = function(block, inputType, position) {
    if (inputType == 'value') {
        var number = 0;
        for (var i = 0; i < block.inputList.length; i++) {
            if (block.inputList[i].type == Blockly.INPUT_VALUE) {
                number++;
                if (number == position) {
                    return i;
                }
            }
        }
    }
};

Blockly.ChangeMode.validateAndDetermineValueInputPosition = function(inputBlock, valueInputPosition) {
    if (valueInputPosition < 1) {
        valueInputPosition = 1;
    }
    var totalNumberOfValueInputs = Blockly.ChangeMode.getNumberOfValueInputs(inputBlock);
    
    if (totalNumberOfValueInputs == 0) {
        throw 'Number of value inputs is zero';
        return -1;
    } 
    if (valueInputPosition > totalNumberOfValueInputs) {
        throw 'The specified value input position is greater than the max. number of inputs.';
        return -1;
    } 
    return Blockly.ChangeMode.getPositionInInputList(inputBlock, 'value', valueInputPosition);
};

Blockly.ChangeMode.performBlockMutation = function(block, mutationList) {
    var storeInitValue = [];
    if (mutationList) {
        // save actual mutator variable values in a separated list. Change the mutator variable values to the required ones.
        for (var i = 0; i < mutationList.length; i++) {
            var mutList = mutationList[i][0];
            var blockMutList = block[mutationList[i][0]];
            storeInitValue.push([mutationList[i][0], block[mutationList[i][0]]]);
            block[mutationList[i][0]] = mutationList[i][1];
        }
    }
    // store the new mutator variable values into an xml container (operating depends on mutationToDom function of each individual block)
    var container = block.mutationToDom();
    if (container) {
        // put the mutator variable values from the list back into place. This is necessary to prevent misbehaviours within domtoMutation.
         // In some cases domToMutation deletes all existing inputs depending on the mutator variable values and 
         // rebuilds them then with the new values stored in the xml container
        //
        for (var i = 0; i < storeInitValue.length; i++) {
            block[storeInitValue[i][0]] = storeInitValue[i][1];
        }
        block.domToMutation(container);
    };
};

// Create and attach a none mutating statement block
Blockly.ChangeMode.prototype.appendNoneMutationStatement = function(statementBlockName) {
    var statementBlock = new Blockly.Block(this.block_.workspace, statementBlockName);
    
    if(statementBlock.outputConnection) {
        statementBlock.dispose();
        throw '\"' + statementBlock.type + '\" has a wrong input type. Expected: Statement \nNote: Function \"appendNoneMutationOutput\" may be used for this kind of block';
        return null;
    }
    
    statementBlock.initSvg();
    statementBlock.render();
    statementBlock.editable = this.newBlockEditable;  
    statementBlock.deletable = this.newBlockDeletable;
            
    this.createStatementConnection();
    if (this.statementConnection.checkType_(statementBlock.previousConnection)) {
        this.statementConnection.connect(statementBlock.previousConnection);
        this.statementConnection = statementBlock.nextConnection;
    }
    else {
       statementBlock.dispose();
       throw '\"' + statementBlock.type + '\" cannot be connected.';
       return null;
    }
    
    return statementBlock;
};

// Creates, mutates and attaches a statement block
// parameters: name of the block, a list of lists containing variable name (string) and its value (number)  
Blockly.ChangeMode.prototype.appendMutationStatement = function(statementBlockName, mutationList) {
    var statementBlock = new Blockly.Block(this.block_.workspace, statementBlockName);
    
    if(statementBlock.outputConnection) {
        statementBlock.dispose();
        throw '\"' + statementBlock.type + '\" has a wrong input type. Expected: Statement \nNote: Function \"appendMutationOutput\" may be used for this kind of block';
        return null;
    }
    if (!statementBlock.mutator) {
        statementBlock.dispose();
        throw '\"' + statementBlock.type + '\" has no mutator. \nNote: Function \"appendNoneMutationStatement\" may be used for this kind of block.';
        return null;
    }
    
    statementBlock.initSvg();
    Blockly.ChangeMode.performBlockMutation(statementBlock, mutationList);
    statementBlock.render();
    statementBlock.editable = this.newBlockEditable;  
    statementBlock.deletable = this.newBlockDeletable;
    
    this.createStatementConnection();
    if (this.statementConnection.checkType_(statementBlock.previousConnection)) {
        this.statementConnection.connect(statementBlock.previousConnection);
        this.statementConnection = statementBlock.nextConnection;
    }
    else {
       statementBlock.dispose();
       throw '\"' + statementBlock.type + '\" cannot be connected.';
       return null;
    }
    
    return statementBlock;
};

// append an output block to a specified value input
// name of new output block, reference to input block , position where the block should be connected (starting with 1) 
Blockly.ChangeMode.prototype.appendNoneMutationOutput = function(outputBlockName, valueInputPosition) {
    var inputBlock = this.block_;
    var valueInputPositionInInputList = Blockly.ChangeMode.validateAndDetermineValueInputPosition(inputBlock, valueInputPosition);
    if(valueInputPositionInInputList == -1) {
        return null;
    }
    
    var outputBlock = new Blockly.Block(this.block_.workspace, outputBlockName);
    if (!outputBlock.outputConnection) {
        outputBlock.dispose();
        throw '\"' + outputBlock.type + '\" has a wrong input type. Expected: Output \nNote: Function \"appendNoneMutationStatement\" may be used for this kind of block.';
        return null;
    }
    
    outputBlock.initSvg();
    outputBlock.render();
    outputBlock.editable = this.newBlockEditable;  
    outputBlock.deletable = this.newBlockDeletable;
    
    var connection = inputBlock.getInput(inputBlock.inputList[valueInputPositionInInputList].name).connection;
    if (connection.checkType_(outputBlock.outputConnection)) {
        connection.connect(outputBlock.outputConnection);
    }
    else {
       outputBlock.dispose();
       throw '\"' + outputBlock.type + '\" cannot be connected to \"' + inputBlock.type + '\"';
       return null;
    }
    
    return outputBlock; 
};

Blockly.ChangeMode.prototype.appendMutationOutput = function(outputBlockName, valueInputPosition, mutationList) {
    var inputBlock = this.block_;
    var valueInputPositionInInputList = Blockly.ChangeMode.validateAndDetermineValueInputPosition(inputBlock, valueInputPosition);
    if(valueInputPositionInInputList == -1) {
        return null;
    }
    
    var outputBlock = new Blockly.Block(this.block_.workspace, outputBlockName);
    if (!outputBlock.outputConnection) {
        outputBlock.dispose();
        throw '\"' + outputBlock.type + '\" has a wrong input type. Expected: Output \nNote: Function \"appendNoneMutationStatement\" may be used for this kind of block.';
        return null;
    }
    if (!outputBlock.mutator) {
        outputBlock.dispose();
        throw '\"' + outputBlock.type + '\" has no mutator. \nNote: Function \"appendMutationOutput\" may be used for this kind of block.';
        return null;
    }
    
    outputBlock.initSvg();
    Blockly.ChangeMode.performBlockMutation(outputBlock, mutationList);
    outputBlock.render();
    outputBlock.editable = this.newBlockEditable;  
    outputBlock.deletable = this.newBlockDeletable;
    
    var connection = inputBlock.getInput(inputBlock.inputList[valueInputPositionInInputList].name).connection;
    if (connection.checkType_(outputBlock.outputConnection)) {
        connection.connect(outputBlock.outputConnection);
    }
    else {
       outputBlock.dispose();
       throw '\"' + outputBlock.type + '\" cannot be connected to \"' + inputBlock.type + '\"';
       return null;
    }
    
    return outputBlock; 
};

// NOTE the return value is a list of two blocks: The procedure Block itself and the corresponding call block.
Blockly.ChangeMode.prototype.createProcedure = function(hasReturn, mutationList) {
    if (hasReturn) {
        var procedureBlock = new Blockly.Block(this.block_.workspace, 'procedures_defreturn');
        var callBlock = new Blockly.Block(this.block_.workspace, 'procedures_callreturn');
        
    }
    else {
        var procedureBlock = new Blockly.Block(this.block_.workspace, 'procedures_defnoreturn');
        var callBlock = new Blockly.Block(this.block_.workspace, 'procedures_callnoreturn');
    }
    
    procedureBlock.initSvg();
    procedureBlock.moveBy(700, 0);
    callBlock.initSvg();
    Blockly.ChangeMode.performBlockMutation(procedureBlock, mutationList);
    callBlock.arguments_ = mutationList[0][1];
    Blockly.ChangeMode.performBlockMutation(callBlock, null);
    procedureBlock.render();
    callBlock.render();
    procedureBlock.editable = this.newBlockEditable;  
    callBlock.editable = this.newBlockEditable;  
    procedureBlock.deletable = this.newBlockDeletable;
    callBlock.deletable = this.newBlockDeletable;
    
    return [procedureBlock, callBlock];
}

Blockly.ChangeMode.prototype.appendProcedureCall = function(procedureCallBlock, valueInputPosition) {
    var inputBlock = this.block_;
    var valueInputPositionInInputList = Blockly.ChangeMode.validateAndDetermineValueInputPosition(inputBlock, valueInputPosition);
    if(valueInputPositionInInputList == -1) {
        return null;
    }
    
    var connection = inputBlock.getInput(inputBlock.inputList[valueInputPositionInInputList].name).connection;
    if (connection.checkType_(procedureCallBlock.outputConnection)) {
        connection.connect(procedureCallBlock.outputConnection);
    }
    else {
       throw '\"' + procedureCallBlock.type + '\" cannot be connected to \"' + inputBlock.type + '\"';
       return null;
    }
    
    return procedureCallBlock; 
}

Blockly.ChangeMode.prototype.reconnectOutputConnection = function(attachedBlock, valueInputPosition) {
    if (attachedBlock != null) {
        var inputBlock = this.block_;
        var valueInputPositionInInputList = Blockly.ChangeMode.validateAndDetermineValueInputPosition(inputBlock, valueInputPosition);
        var connection = inputBlock.getInput(inputBlock.inputList[valueInputPositionInInputList].name).connection;
        if (connection.checkType_(attachedBlock.outputConnection)) {
            connection.connect(attachedBlock.outputConnection);
        }
        else {
           throw '\"' + attachedBlock.type + '\" cannot be connected to \"' + inputBlock.type + '\" at value input position ' + valueInputPosition;
        }
    }
};

// Reconnect through duplicated blocks, remove all connected blocks and connect the duplicates
Blockly.ChangeMode.prototype.reconnectAndRemoveChildren = function(){
    var blockList = [];
    var inputPositionList = [];
    
    // Loop through the whole list of blocks which should be reconnected and duplicate them
    for (var i=0; i<this.reconnectMonitor.length; i++) {
        var inputBlock = this.reconnectMonitor[i][0];
        var valueInputPositionInInputList = Blockly.ChangeMode.validateAndDetermineValueInputPosition(inputBlock, this.reconnectMonitor[i][1]);
        var connection = inputBlock.getInput(inputBlock.inputList[valueInputPositionInInputList].name).connection;
        if (connection && connection.targetConnection && connection.targetConnection.sourceBlock_) {
            var targetBlock = connection.targetConnection.sourceBlock_; 
            blockList.push(targetBlock.duplicate_());
            inputPositionList.push(this.reconnectMonitor[i][2]);
        }
    }
    
    // Delete all children of the main block (this.sourceBlock_)
    this.removeChildren();  
    // Loop a second time through the list of blocks to be reconnected to connect them to the specified position. 
    if (this.reconnectMonitor.length > 0) {
        this.reconnectAllListedBlocks(blockList, inputPositionList);   
    }
};

Blockly.ChangeMode.prototype.reconnectAllListedBlocks = function(blockList, inputPositionList) {
    if(!blockList || !inputPositionList || blockList.length != inputPositionList.length) {
        return;
    }
    for (var i=0; i<inputPositionList.length; i++) {
        if (blockList[i] != null) {
            this.reconnectOutputConnection(blockList[i], inputPositionList[i]);
        }
    }
};

Blockly.ChangeMode.prototype.addToReconnectMonitor = function(block, innerConnectionPosition, outerReconnectPosition) {
    this.reconnectMonitor.push([block, innerConnectionPosition, outerReconnectPosition]);
};

Blockly.ChangeMode.prototype.addToProcedureMonitor = function(procedureName) {
    this.procedureMonitor.push(procedureName);
};

Blockly.ChangeMode.prototype.changeModeToDom = function(state) {
    var changemodeElement = document.createElement('changemode');
    changemodeElement.setAttribute('changemodestate', state);
    
    var firstStatementBlock = this.block_.getInputTargetBlock(this.appendInput);
    var blocks = firstStatementBlock && firstStatementBlock.getDescendants();
    var reconnectMonitorLength = this.reconnectMonitor.length;
    var posCount = 0;
    if (blocks)  {
        for (var i=0; i < blocks.length; i++) {
            for (var j=0; j<reconnectMonitorLength; j++) {
                var monitoredBlock = this.reconnectMonitor[j][0];
                if (monitoredBlock && (blocks[i].id == monitoredBlock.id)) {
                    var element = document.createElement('reconnect' + posCount);
                    element.setAttribute('blockposition', i);
                    element.setAttribute('valueinputposition', this.reconnectMonitor[j][1]);
                    element.setAttribute('reconnectposition', this.reconnectMonitor[j][2]);
                    changemodeElement.appendChild(element);
                    posCount++;
                }
            }
        }
    }
    posCount = 0;
    for (var i=0; i<this.procedureMonitor.length; i++) {
        var element = document.createElement('procedure' + posCount);
        element.setAttribute('name', this.procedureMonitor[i]);
        changemodeElement.appendChild(element);
    }

    return changemodeElement;
};

Blockly.ChangeMode.prototype.domToChangeMode = function(xmlElement) {
    var firstStatementBlock = this.block_.getInputTargetBlock(this.appendInput);
    var blocks = firstStatementBlock && firstStatementBlock.getDescendants();
    var blocksLength = blocks && blocks.length;
    if (xmlElement) {
        this.block_.setWarningText('You activated the next level of complexity. If you are not familiar with that level feel free to deactivate it or investigate further.');
        for (var i=0; i<xmlElement.childNodes.length; i++) {
            var element = xmlElement.childNodes[i];
            var nodeName = element.nodeName;
            if(nodeName.match(/reconnect/)) {
                var blockPosition = parseInt(element.getAttribute('blockposition'));
                if (blockPosition >= 0 && blockPosition <= blocksLength) {
                    var valueInputPosition = parseInt(element.getAttribute('valueinputposition'));
                    var reconnectPosition = parseInt(element.getAttribute('reconnectposition'));
                    var monitoredBlock = blocks[blockPosition];
                    this.addToReconnectMonitor(monitoredBlock ,valueInputPosition, reconnectPosition);
                }
            }
            else if(nodeName.match(/procedure/)) {
                this.addToProcedureMonitor(element.getAttribute('name'));
            }
        } 
    }
};
