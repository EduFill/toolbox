/* The resulting code should help to increase the development process
 * of the visible blocks within an activated ChangeMode
 * 
 * The result contains just code for the children added to the mode
 * The implementation itself has to be done by the developers when they
 * define the block
 */

var cnt = 0;
var changemodeBlock = null;
var HTMLwhitespace  = '&nbsp;';
var ws4 = HTMLwhitespace + HTMLwhitespace + HTMLwhitespace + HTMLwhitespace;
var show = [];
var dispose = [];

setAllTitleValues = function(block) {
    var inputList = block.inputList;
    for (var i=0; i<inputList.length; i++) {
        var titleRow = inputList[i].titleRow;
        for (var j=0; j<titleRow.length; j++) {
            var titleName = titleRow[j].name;
            if (titleName) {
                show.push(ws4 + block.type + 'Block' + cnt + '.setTitleValue(\''+ block.getTitleValue(titleName) + '\',\'' + titleName + '\');');
            }
        } 
    }
}

addBlocksRecursive = function(block, changemodeName) {
    var tempCnt = cnt;
    var notMoreThanOnce = true;
    appendInput(block, changemodeName);
    var inputList = block.inputList;
    for (var i=0; i<inputList.length; i++) {
        var input = inputList[i];
        // value input (just one block with children)
        if (block.getInputTargetBlock(input.name)) {
            if (notMoreThanOnce) {
                changemodeName = block.type + 'Changemode' + tempCnt;
                show.push(ws4 + 'var ' + changemodeName + ' = new Blockly.ChangeMode('+ block.type + 'Block' + tempCnt +');');
                notMoreThanOnce = false;
            }
            if (input.type == 1) {
                var child = block.getInputTargetBlock(input.name);
                addBlocksRecursive(child, changemodeName);
            }
            // statement input (possibly many blocks with children)
            else if (input.type == 3) {
                show.push(ws4 + changemodeName + '.changeStatementInputTo(\'' + input.name + '\');');
                var child = block.getInputTargetBlock(input.name);
                while (child) {
                    addBlocksRecursive(child, changemodeName);
                    child = child.nextConnection && child.nextConnection.targetBlock();
                } 
            }
        }
    }
};

appendInput = function(childBlock, changemodeName) {
    var outputBlock = false;
    var setTitles = true;
    if (childBlock.outputConnection) {
        var valueInputAreaNumber = getValueInputAreaPositionWithUnknownInputAreaName(childBlock);
        if (valueInputAreaNumber == null) {
            throw 'valueInputAreaNumber is null';
        } 
        outputBlock = true;
    }
    
    if (!childBlock.mutator) {
        if(outputBlock) {
            if(childBlock.type == 'developer_reconnectOutputConnection') {
                var outerValueInputAreaNumber = getValueInputAreaPositionWithRespectToParameterName(changemodeBlock,childBlock.getTitleValue('TEXT'));
                if (outerValueInputAreaNumber == null) {
                    alert('Have you tried to reconnect a none-existent output block? Please mutate(+) the main block (changemode ...) accordingly');
                } 
                show.push(ws4 + changemodeName + '.reconnectOutputConnection(blockAttachedAt' + childBlock.getTitleValue('TEXT') + ', ' + valueInputAreaNumber + ');');
                show.push(ws4 + 'block.changemode.addToReconnectMonitor(' + childBlock.parentBlock_.type + 'Block' + (cnt-1) + ', ' + valueInputAreaNumber + ', '  + outerValueInputAreaNumber + ');');
                setTitles = false;
            }
            else {
                show.push(ws4 + 'var ' + childBlock.type + 'Block' + cnt + ' = ' + changemodeName + '.appendNoneMutationOutput(\'' + childBlock.type + '\', ' + valueInputAreaNumber + ');');
            }
        }
        else {
            show.push(ws4 + 'var ' + childBlock.type + 'Block' + cnt + ' = ' + changemodeName + '.appendNoneMutationStatement(\'' + childBlock.type + '\');');
        }
    } 
    else {
        var container = childBlock.mutationToDom();
        var mutatorList = '';
        if(container) {
            var attributes = container.attributes;
            var attrList = [];
            for (var i=0; i<attributes.length; i++) {
                attrList.push('[\'' + attributes[i].nodeName + 'Count_\','+ parseInt(attributes[i].nodeValue) +']');
            }
            mutatorList = '[' + attrList.join(',') + ']';
            
        }
        else {
            mutatorList = '[No information available. Add manually!]';
        }
        
        if (outputBlock) {
            show.push(ws4 + 'var ' + childBlock.type + 'Block' + cnt + ' = ' + changemodeName + '.appendMutationOutput(\'' + childBlock.type + '\',' + valueInputAreaNumber + ',' + mutatorList + ' );')
        }
        else {
            show.push(ws4 + 'var ' + childBlock.type + 'Block' + cnt + ' = ' + changemodeName + '.appendMutationStatement(\'' + childBlock.type + '\', ' + mutatorList + ' );')
        }
    }
    
    if(setTitles) {
        setAllTitleValues(childBlock);
    }
    cnt++    
};

// for reconnectOutputConnection ONLY! 
getValueInputAreaPositionWithRespectToParameterName = function(block, valueName) {
    var valueInputAreaPosition = 0;
    for (var i=0; i< block.inputList.length; i++) {
        if (block.inputList[i].titleRow[1] && block.inputList[i].titleRow[1].text_) {
            valueInputAreaPosition++;
            if (block.inputList[i].titleRow[1].text_ == valueName) {
                return valueInputAreaPosition;
            }
        }
    }
    
    return null;
};

getValueInputAreaPosition = function(block, inputAreaName) {
    var valueInputAreaPosition = 0;
    for (var i = 0; i < block.inputList.length; i++) {
        if (block.inputList[i].type == Blockly.INPUT_VALUE) {
            valueInputAreaPosition++;
        }
        
        if (block.inputList[i].name == inputAreaName) {
            return valueInputAreaPosition;
        }
    }
    
    return null;
};

getValueInputAreaPositionWithUnknownInputAreaName = function(child) {
    var parent = child.parentBlock_;
    for (var i=0; i<parent.inputList.length; i++) {
        input = parent.inputList[i];
        if (input.connection && input.connection.targetConnection) {
            var attachedBlock = input.connection.targetConnection.sourceBlock_;
            if (attachedBlock.id == child.id) {
                return getValueInputAreaPosition(parent, input.name);
            }
        }
    }
    return null;
}

domFunctions = function() {
    var domString = [];
    domString.push('// Manually include entries for domToMutation');
    domString.push('// e.g. removeInput(), block.appendStatementInput(this.changemode.appendInput);, etc. </br>');
    domString.push('changeModeToDom: function() {');
    domString.push(ws4 + 'var state = this.getTitleValue(\'STATE\');');
    domString.push(ws4 + 'return this.changemode.changeModeToDom(state);');
    domString.push('},');
    domString.push('domToChangeMode: function(xmlElement) {');
    domString.push(ws4 + 'var changeModeState = xmlElement.getAttribute(\'changemodestate\');');
    domString.push(ws4 + 'var block = this;');
    domString.push(ws4 + 'if (changeModeState == \'TRUE\') {');
    domString.push(ws4 + ws4 + 'this.changemode.domToChangeMode(xmlElement);');
    domString.push(ws4 + '}');
    domString.push('},');
    
    return domString;
};

attachedBlocks = function(block, output) {
    for (var i=0; i<block.itemCount_; i++) {
        var inputName = block.getTitleValue('TEXT' + i);
        show.push(ws4 + 'var blockAttachedAt' + inputName + ' = block.getInputTargetBlock(\'' + inputName + '\');');
        show.push(ws4 + 'if (block.getInput(\'' + inputName + '\')) {');
        show.push(ws4 + ws4 + 'block.removeInput(\''+ inputName + '\');');
        show.push(ws4 + '}');
            
        dispose.push(ws4 + 'if (!block.getInput(\'' + inputName + '\')) {');
        dispose.push(ws4 + ws4 + '// Add the appropriate input (name: \'' + inputName + '\') here.');
        dispose.push(ws4 + '}');
        if (output) {
            dispose.push(ws4 + 'block.moveInputBefore(\'' + inputName + '\', block.changemode.appendInput);');
        }
    }
};

getProcedureArguments = function(block) {
    var arguments = []
    for (var i=0; i<block.arguments_.length; i++) {
        arguments.push('\'' + block.arguments_[i] + '\'');
    }
    var argumentsString = arguments.join(', ');
    
    return argumentsString;
};

initCode = function(outputBlock) {
    show.push('showChangeMode: function() {');
    show.push(ws4 + 'var block = this;');
    if (outputBlock) {
        show.push(ws4 + 'block.changemode.setChangeMode(true, \'output\');');
    }
    else {
        show.push(ws4 + 'block.changemode.setChangeMode(true);');
    }
    show.push(ws4 + 'block.image.setText(Blockly.pathToBlockly + \'media/menu1.png\');');
    dispose.push('disposeChangeMode: function() {');
    dispose.push(ws4 + 'var block = this;');
};

finishCode = function(outputBlock) {
    show.push('},</br>');
    if (outputBlock) {
        dispose.push(ws4 + 'block.changemode.setChangeMode(false, \'output\');');
    }
    else {
        dispose.push(ws4 + 'block.changemode.setChangeMode(false);');
    }
    dispose.push(ws4 + 'block.image.setText(Blockly.pathToBlockly + \'media/menu0.png\');');
    dispose.push('}');
};

getChangeModeGeneratorResult = function() {
  show = [];
  dispose = [];
  var topBlocks = Blockly.mainWorkspace.getTopBlocks(false);
  cnt = 0;
  for (var i=0; i<topBlocks.length; i++) {
    var block = topBlocks[i];
    if(block.type == 'developer_changemodeStatement') {
        initCode(false);
        changemodeBlock = block;
        attachedBlocks(block, false);
        var statementChildBlock = block.getInputTargetBlock('APP_STACK');
        while (statementChildBlock) {
            addBlocksRecursive(statementChildBlock, 'block.changemode');
            statementChildBlock = statementChildBlock.nextConnection && statementChildBlock.nextConnection.targetBlock();
        }
        finishCode(false);
    }
    else if(block.type == 'developer_changemodeOutput') {
        initCode(true);
        changemodeBlock = block;
        attachedBlocks(block, true); 
        var valueChildBlock = block.getInputTargetBlock('APP_VALUE');
        if (valueChildBlock.type == 'procedures_callreturn') {
            show.push(ws4 + 'var procedure = block.changemode.createProcedure(true, [[\'arguments_\', [' + getProcedureArguments(valueChildBlock) + ']]]);');
            show.push(ws4 + 'var procedureBlock = procedure[0];');
            show.push(ws4 + 'var callBlock = procedure[1];');
            show.push(ws4 + '// rename procedure');
            show.push(ws4 + 'var name = Blockly.Procedures.findLegalName(\'' + valueChildBlock.getProcedureCall() + '\', procedureBlock);');
            show.push(ws4 + 'procedureBlock.setTitleValue(name, \'NAME\');');
            show.push(ws4 + 'block.changemode.addToProcedureMonitor(name);');
            show.push(ws4 + 'block.changemode.appendProcedureCall(callBlock, 1);');
            show.push(ws4 + 'var changemodeCall = new Blockly.ChangeMode(callBlock);</br>');
            
            for (var j=0; j<valueChildBlock.arguments_.length; j++) {
                show.push(ws4 + 'changemodeCall.reconnectOutputConnection(blockAttachedAt' + block.getTitleValue('TEXT' + j) +' , ' + (j+1) + ');');
                show.push(ws4 + 'block.changemode.addToReconnectMonitor(callBlock, ' + (j+1) + ', ' + (j+1) + ');');
            }
            
            show.push(ws4 + 'var changemodeProcedure = new Blockly.ChangeMode(procedureBlock, \'STACK\');');
            
            var procedure = null;
            for (var j=0; j<topBlocks.length; j++) {
                if(topBlocks[j].type == 'procedures_defreturn') {
                    var name = topBlocks[j].getTitleValue('NAME');
                    if (name == valueChildBlock.getProcedureCall()) {
                        procedure = topBlocks[j];
                    }
                }
            }
            if (procedure) {
                var statementChildBlock = procedure.getInputTargetBlock('STACK');
                while (statementChildBlock) {
                    addBlocksRecursive(statementChildBlock, 'changemodeProcedure');
                    statementChildBlock = statementChildBlock.nextConnection && statementChildBlock.nextConnection.targetBlock();
                }
            }
            else {
                alert('no fitting procedure on the workspace')
            }
            if (procedure.getInputTargetBlock('RETURN')) {
                addBlocksRecursive(procedure.getInputTargetBlock('RETURN'), 'changemodeProcedure');
            }
        }
        finishCode(true);
    }
    else {
        //result.push('//ignore ' + topBlocks[i].type);
    }
  }
  var result = domFunctions().join('</br>') + '</br>' + show.join('</br>') + dispose.join('</br>');
  return result;
};
