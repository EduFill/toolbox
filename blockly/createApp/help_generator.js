var helpContent = '';

generateHelpContent = function() {
    var helpContentArray = [];
    var helpWorkspace = new Blockly.Workspace(false);
    /*
    helpContentArray.push('<h1>Edufill Blockly Help Page</h1>');
    helpContentArray.push('<table height="100%" width="100%">');
        
        helpContentArray.push('<tr>');
            helpContentArray.push('<td>');
                helpContentArray.push('empty');
            helpContentArray.push('</td>');
        helpContentArray.push('</tr>');
    helpContentArray.push('</table>');
    * */
    // perform some operations to generate help text etc.
    helpContentArray.push('<b>Edufill Blockly Help</b>');
    helpContentArray.push('<table height="100%" width="100%" border="1">');
        for (var cat in Blockly.Toolbox.languageTree) {
           helpContentArray.push('<tr>'); 
           helpContentArray.push('<td>');
           helpContentArray.push('<b><u>Category: ' + decodeURI(cat.substring(Blockly.Toolbox.PREFIX_.length)) +'</u></b>');
           var catEntries = Blockly.Toolbox.languageTree[cat];
           for (var i=0; i<catEntries.length; i++) {
                var entry = catEntries[i];
                helpContentArray.push('<table height="100%" width="100%">');
                helpContentArray.push('<tr>'); 
                helpContentArray.push('<td>');
                getBlockInformation(helpContentArray, entry, helpWorkspace);
                helpContentArray.push('</td>');
                helpContentArray.push('</tr>');
                helpContentArray.push('</table>');
           }
           helpContentArray.push('</td>');
           helpContentArray.push('</tr>'); 
        }
    helpContentArray.push('</table>');
    
    helpContent = helpContentArray.join('');
};

getBlockInformation = function(helpContentArray, blockName, workspace) {
    helpContentArray.push('<div>');
    var block = new Blockly.Block(workspace, blockName);
    helpContentArray.push('<b>' + block.type + '</b>');
    //TODO more content out of the block
    // Show workspace with block inside or picture of that
    
    // GetColour 
    helpContentArray.push('<li>Block color: ' + block.getColour() + '</li>');
    // Show whether the block is an output, statement or isolated block
    if (block.outputConnection) {
        helpContentArray.push('<li>Blocktype: Output</li>');
    }
    else if (block.previousConnection || block.nextConnection) {
        helpContentArray.push('<li>Blocktype: Statement (previous block: ' + ((block.previousConnection) ? 'enabled' : 'disabled') + ', next block: ' + ((block.nextConnection) ? 'enabled' : 'disabled') + ')</li>');
    }
    else {
        helpContentArray.push('<li>Blocktype: Isolated</li>');
    }
    // Is it a mutator block?
    if (block.mutator) {
        helpContentArray.push('<li>Mutator block</li>');
        // TODO provide more information
    }
    
    // Can the changemode be activated?
    if (block.changeModeToDom) {
        helpContentArray.push('<li>Changemode block</li>');
    }
    
    helpContentArray.push('</div>');
    block.dispose();
    delete block;
};

getHelpContent = function() {
    return helpContent;
};
