var unitTestResults = [];
var errorLog = [];
var passedCnt = 0;
var failedCnt = 0;

unitTest = function() {
    unitTestArray = [];
    var blockCnt = 0;
    
    var testWorkspace = Blockly.mainWorkspace;
    //alert(toolbox.children[0].attributes.name.nodeValue);
    for (var i=0; i<toolbox.children.length; i++) {
        var cat = toolbox.children[i];
        
        unitTestResults.push('<b><u>Test category: ' + cat.attributes.name.nodeValue +'</u></b></br>');
        //*
        //var catEntries = Blockly.languageTree[cat];
        var catEntries = cat.children;
        for (var j=0; j<catEntries.length; j++) {
            var entry = catEntries[j].attributes.type.nodeValue;
            blockCnt++;
            
            unitTestResults.push('<b>' + blockCnt + ' Test block: ' + entry + '</b></br>');
            // Try to create a block
            var blockTest_TwoElemList_BoolAndBlock = unitTestCreateBlock(entry, testWorkspace);
            blockTest_Bool = blockTest_TwoElemList_BoolAndBlock[0];
            blockTest_Block = blockTest_TwoElemList_BoolAndBlock[1];
            if (blockTest_Bool) {
                // If the block is an output block then attach it to a set variable.
                if (blockTest_Block.outputConnection) {
                    var setBlock = new Blockly.Block(testWorkspace, 'variables_set');
                    setBlock.initSvg();
                    setBlock.render();
                    var connection = setBlock.getInput('VALUE').connection;
                    connection.connect(blockTest_Block.outputConnection);
                }
                // Try to generate python code
                unitTestPythonGenerator(blockTest_Block, 'standard block');
                // For all value input areas attach a get variable block
                appendOutputBlocksToAllValueInputAreas(blockTest_Block);
                // Try to enable the ChangeMode
                var enableChangemodeTest_Bool = unitTestEnableChangeMode(blockTest_Block);
                if (enableChangemodeTest_Bool) {
                    // Try to generate python code (has changed because of enabled ChangeMode)
                    unitTestPythonGenerator(blockTest_Block, 'ChangeMode enabled block');
                    // Try to transform the block object to XML and back (with enabled ChangeMode)
                    unitTestBlockToDomToBlock(blockTest_Block, 'ChangeMode enabled block');
                    // Try to disable the ChangeMode
                    unitTestDisableChangeMode(blockTest_Block);
                    // Verify if all get blocks are still connected to the value input areas.
                    unitTestAreAllValueInputAreasStillOccupied(blockTest_Block);
                    // Check if artefacts like a still existing procedure block or others are on the workspace (next to the intended top-level block with its children)
                    unitTestAreChangeModeProcedureAndOtherArtefactsDeleted(blockTest_Block);
                }
                // Try to transform the block object to XML and back
                unitTestBlockToDomToBlock(blockTest_Block, 'standard block');
                // Delete all blocks on the workspace
                testWorkspace.clear();
            }
            
            unitTestResults.push('</br>');

        }
        //*/ 
    }
    
    if (failedCnt > 0) { 
        return false; 
    }
    else {
        return true;
    }
};
unitTestAreAllValueInputAreasStillOccupied = function(block) {
    unitTestResults.push('Check if all value input areas are still occupied: ');
    var namesOfValueInputAreas = Blockly.ChangeMode.getNamesOfValueInputs(blockTest_Block);
    var oneFailed = false;
    for (var i=0; i<namesOfValueInputAreas.length; i++) {
        var name = namesOfValueInputAreas[i];
        if (block.getInputTargetBlock(name) == null) {
            errorLog.push('\"' + block.type + '\" Reconnect error: <font color="red">No block at value input area ' + name +'</font></br>');
            oneFailed = true;
        }
    }
    
    if (oneFailed) {
        unitTestResults.push('<font color="red">failed</font></br>');
        failedCnt++;
        return false;
    }
    unitTestResults.push('<font color="green">passed</font></br>');
    passedCnt++;
    return true
};

unitTestBlockToDomToBlock = function(block, string) {
    unitTestResults.push('Try to encode a block as XML (' + string +'): ');
    try {
        var blockToDom = Blockly.Xml.blockToDom_(block);
    }
    catch(error) {
        errorLog.push('\"' + block.type + '\" Encode block as XML error: <font color="red">' + error + '</font></br>');
        unitTestResults.push('<font color="red">failed</font></br>');
        failedCnt++;
        return false;
    }
    unitTestResults.push('<font color="green">passed</font></br>');
    passedCnt++;
    
    unitTestResults.push('Try to decode XML back to the block (' + string +'): ');
    try {
        var domToBlock = Blockly.Xml.domToBlock_(Blockly.mainWorkspace, blockToDom); 
        domToBlock.dispose(); 
    }
    catch(error) {
        errorLog.push('\"' + block.type + '\" Decode XML to block error: <font color="red">' + error + '</font></br>');
        unitTestResults.push('<font color="red">failed</font></br>');
        failedCnt++;
        return false;
    }
    unitTestResults.push('<font color="green">passed</font></br>');
    passedCnt++;
    return true;
    
};

unitTestEnableChangeMode = function(block) {
    if(block.changemode) {
        unitTestResults.push('Try to enable ChangeMode: ');
        try {
            block.setTitleValue('TRUE','STATE');
            block.getTitle_('STATE').changeHandler_(true);
        }
        catch (error)  {
            errorLog.push('\"' + block.type + '\" ChangeMode with error: <font color="red">' + error + '</font></br>');
            unitTestResults.push('<font color="red">failed</font></br>');
            failedCnt++;
            return false;
        }
        unitTestResults.push('<font color="green">passed</font></br>');
        passedCnt++;
        return true;
    }
};

unitTestDisableChangeMode = function(block) {
    if(block.changemode) {
        unitTestResults.push('Try to disable ChangeMode: ');
        try {
            block.setTitleValue('FALSE','STATE');
            block.getTitle_('STATE').changeHandler_(false);
        }
        catch (error)  {
            errorLog.push('\"' + block.type + '\" ChangeMode with error: <font color="red">' + error + '</font></br>');
            unitTestResults.push('<font color="red">failed</font></br>');
            failedCnt++;
            return false;
        }
        
        unitTestResults.push('<font color="green">passed</font></br>');
        passedCnt++;
        return true;
    }
};

unitTestAreChangeModeProcedureAndOtherArtefactsDeleted = function(block) {
    unitTestResults.push('Check for workspace artefacts: ');
    var topBlocks = Blockly.mainWorkspace.getTopBlocks(false);
    if (topBlocks.length != 1) {
        var listOfTopBlocks = '';
        for (var i=0; i<topBlocks.length; i++) {
            listOfTopBlocks += topBlocks[i].type + ' ';
        }
        errorLog.push('\"' + block.type + '\" artefacts on workspace error: <font color="red">Unwanted blocks on workspace (top blocks: ' + listOfTopBlocks + ')</font></br>');
        unitTestResults.push('<font color="red">failed</font></br>');
        failedCnt++;
        return false;
    }
    unitTestResults.push('<font color="green">passed</font></br>');
    passedCnt++;
    return true;
};


unitTestPythonGenerator = function(block, string) {
    unitTestResults.push('Try to generate Python code (' + string +'): ');
    try {
        Blockly.Generator.workspaceToCode('Python');
    }
    catch (error) {
        errorLog.push('\"' + block.type + '\" python generator with error: <font color="red">' + error + '</font></br>');
        unitTestResults.push('<font color="red">failed</font></br>');
        failedCnt++;
        return false;
    }
    unitTestResults.push('<font color="green">passed</font></br>');
    passedCnt++;
    return true;
};

unitTestCreateBlock = function(blockName, testWorkspace) {
    unitTestResults.push('Try to create block \"' + blockName + '\": ');
    try {
        var block = new Blockly.Block(testWorkspace, blockName);
    }
    catch(error) {
        errorLog.push('\"' + blockName + '\" block with error: <font color="red">' + error + '</font></br>');
        unitTestResults.push('<font color="red">failed</font></br>');
        failedCnt++;
        return [false,''];
    }
    unitTestResults.push('<font color="green">passed</font></br>');
    passedCnt++;
    block.initSvg();
    block.render();
    return [true, block];
};

getUnitTestResults = function() {
    var testNumber = passedCnt + failedCnt;
    var additionalString = '';
    if (failedCnt > 0) {
        additionalString = '(<font color="red">' + (failedCnt == 1 ? 'One test ' : failedCnt +' tests') + 'failed</font>)' + '</br></br>';
    }
    else {
       additionalString = '(<font color="green">All tests passed</font>)</br></br>'; 
    }
    var results = '<b>Unit Test</b></br>' + 'Total number of tests: ' + testNumber + ' ' + additionalString;
    if (failedCnt > 0) {
        results += '<b>Error Log</b></br>';
        results += errorLog.join('');
        results += '</br></br></br>';
    }
    results += '<b>Unit Test results</b></br>';
    results += unitTestResults.join('');
    
    return results;
};

appendOutputBlocksToAllValueInputAreas = function(block) {
    var numberOfValueInputAreas = Blockly.ChangeMode.getNumberOfValueInputs(block);
    var getBlock = [];
    for (var i = 0; i< numberOfValueInputAreas; i++) {
        getBlock[i] = new Blockly.Block(Blockly.mainWorkspace, 'variables_get');
        getBlock[i].initSvg();
        getBlock[i].render();
        var valueInputPositionInInputList = Blockly.ChangeMode.getPositionInInputList(block, 'value', i+1);
        var connection = block.getInput(block.inputList[valueInputPositionInInputList].name).connection;
        connection.connect(getBlock[i].outputConnection);
    }
};

