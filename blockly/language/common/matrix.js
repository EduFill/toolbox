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
 * @fileoverview matrix and vector blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
'use strict';

Blockly.Language.vector = {
    // Vector 
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('vector');
        this.setMutator(new Blockly.Mutator(['vector_value']));
        this.setOutput(true,'Vector');
        this.setTooltip(
                        'Vector\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* value (value input): Specify the value for the vector\n' +
                        'The only allowed connection type is \'Number\'\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the vector.\n' +
                        'The output type is \'Vector\''
                        );
        this.valueCount_ = 0;
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        if (this.valueCount_) {
            container.setAttribute('values', this.valueCount_);
        }
        return container;
    },
    domToMutation: function(xmlElement) {
        for (var i = 0; i < this.valueCount_; i++) {
            if (this.getInput('VALUE' + i))
            {
                this.removeInput('VALUE' + i);
            }
        }
        this.valueCount_ = window.parseInt(xmlElement.getAttribute('values'), 10);
        for (var i = 0; i < this.valueCount_; i++) {
            this.appendValueInput('VALUE' + i)
                .appendTitle('value '  + (i+1))
                .setAlign(Blockly.ALIGN_RIGHT)
                .setCheck('Number');
        }
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'vector_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        for (var i = 0; i < this.valueCount_; i++) {
            var valueBlock = new Blockly.Block(workspace, 'vector_value');
            valueBlock.initSvg();
            connection.connect(valueBlock.previousConnection);
            connection = valueBlock.nextConnection;
        }
        return containerBlock;
    },
    compose: function(containerBlock) {
        // Disconnect all input blocks and remove all inputs.
        for (var i = this.valueCount_ - 1; i >= 0; i--) {
            if (this.getInput('VALUE' + i))
            {
                this.removeInput('VALUE' + i);
            }
        }   
        this.valueCount_ = 0;
        // Rebuild the block's inputs.
        var valueBlock = containerBlock.getInputTargetBlock('STACK');
        while (valueBlock) {
            var input = this.appendValueInput('VALUE' + this.valueCount_)
                            .appendTitle('value ' + (this.valueCount_+1))
                            .setAlign(Blockly.ALIGN_RIGHT)
                            .setCheck('Number');
                
              // Reconnect any child blocks.
              if (valueBlock.valueConnection_) {
                input.connection.connect(valueBlock.valueConnection_);
              }
              this.valueCount_++;
              valueBlock = valueBlock.nextConnection && 
              valueBlock.nextConnection.targetBlock();
        }
    },
    saveConnections: function(containerBlock) {
        // Store a pointer to any connected child blocks.
        var valueBlock = containerBlock.getInputTargetBlock('STACK');
        var valueCount_ = 0;
        while (valueBlock) {
            var input = this.getInput('VALUE' + valueCount_);
            valueBlock.valueConnection_ = input && input.connection.targetConnection;
            valueCount_++;
            valueBlock = valueBlock.nextConnection &&
                valueBlock.nextConnection.targetBlock();
        }
    }
};

Blockly.Language.vector_container = {
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('vector');
        this.appendStatementInput('STACK');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.vector_value = {
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('value');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.matrix = {
    // Matrix frame
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('matrix');
        this.setMutator(new Blockly.Mutator(['matrix_row_value']));
        this.setOutput(true,'Matrix');
        this.setTooltip(
                        'Used to create a matrix.\n' + 
                        'Note: The number of columns for each row have to be equal.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* row (value input): Add row blocks with columns\n' +
                        'The only allowed connection type is \Row\'\n' + 
                        '---\n' +
                        'Output:\n' +
                        '* Return the matrix.\n' +
                        'The output type is \'Matrix\'' 
                        );
        this.rowCount_ = 0;
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        if (this.rowCount_) {
            container.setAttribute('rows', this.rowCount_);
        }
        return container;
    },
    domToMutation: function(xmlElement) {
        for (var i = 0; i < this.rowCount_; i++) {
            if (this.getInput('ROW' + i))
            {
                this.removeInput('ROW' + i);
            }
        }
        this.rowCount_ = window.parseInt(xmlElement.getAttribute('rows'), 10);
        for (var i = 0; i < this.rowCount_; i++) {
            this.appendValueInput('ROW' + i)
                .appendTitle('row ' + (i+1))
                .setAlign(Blockly.ALIGN_RIGHT)
                .setCheck('Row');
        }
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'matrix_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        for (var i = 0; i < this.rowCount_; i++) {
            var rowBlock = new Blockly.Block(workspace, 'matrix_row_value');
            rowBlock.initSvg();
            connection.connect(rowBlock.previousConnection);
            connection = rowBlock.nextConnection;
        }
        return containerBlock;
    },
    compose: function(containerBlock) {
        // Disconnect all input blocks and remove all inputs.
        for (var i = this.rowCount_ - 1; i >= 0; i--) {
            if (this.getInput('ROW' + i))
            {
                this.removeInput('ROW' + i);
            }
        }   
        this.rowCount_ = 0;
        // Rebuild the block's inputs.
        var rowBlock = containerBlock.getInputTargetBlock('STACK');
        while (rowBlock) {
            var input = this.appendValueInput('ROW' + this.rowCount_)
                            .appendTitle('row ' + (this.rowCount_+1))
                            .setAlign(Blockly.ALIGN_RIGHT)
                            .setCheck('Row');
                
              // Reconnect any child blocks.
              if (rowBlock.valueConnection_) {
                input.connection.connect(rowBlock.valueConnection_);
              }
              this.rowCount_++;
              rowBlock = rowBlock.nextConnection && 
              rowBlock.nextConnection.targetBlock();
        }
    },
    saveConnections: function(containerBlock) {
        // Store a pointer to any connected child blocks.
        var rowBlock = containerBlock.getInputTargetBlock('STACK');
        var rowCount_ = 0;
        while (rowBlock) {
            var input = this.getInput('ROW' + rowCount_);
            rowBlock.valueConnection_ = input && input.connection.targetConnection;
            rowCount_++;
            rowBlock = rowBlock.nextConnection &&
                rowBlock.nextConnection.targetBlock();
        }
    }
};

Blockly.Language.matrix_container = {
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('matrix');
        this.appendStatementInput('STACK');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.matrix_row_value = {
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('row');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.row = {
    // Matrix row with columns
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('row');
        this.setMutator(new Blockly.Mutator(['row_column_value']));
        this.setOutput(true,'Row');
        this.setInputsInline(true);
        this.setTooltip(
                        'A matrix row with columns\n' +
                        '---\n' +
                        'Inputs: \n' +
                        '* column (value input): Specify the value for the column in a row.\n' +
                        'The only allowed connection type ist \'Number\'\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a matrix row.\n' +
                        'The output type is \'Row\'' 
                        );
        this.columnCount_ = 0;
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        if (this.columnCount_) {
            container.setAttribute('columns', this.columnCount_);
        }
        return container;
    },
    domToMutation: function(xmlElement) {
        for (var i = 0; i < this.columnCount_; i++) {
            if (this.getInput('COL' + i))
            {
                this.removeInput('COL' + i);
            }
        }
        this.columnCount_ = window.parseInt(xmlElement.getAttribute('columns'), 10);
        for (var i = 0; i < this.columnCount_; i++) {
            this.appendValueInput('COL' + i)
                .appendTitle('column ' + (i+1))
                .setAlign(Blockly.ALIGN_RIGHT)
                .setCheck('Number');
        }
    },
    decompose: function(workspace) {
        var containerBlock = new Blockly.Block(workspace, 'row_container');
        containerBlock.initSvg();
        var connection = containerBlock.getInput('STACK').connection;
        for (var i = 0; i < this.columnCount_; i++) {
            var colBlock = new Blockly.Block(workspace, 'row_column_value');
            colBlock.initSvg();
            connection.connect(colBlock.previousConnection);
            connection = colBlock.nextConnection;
        }
        return containerBlock;
    },
    compose: function(containerBlock) {
        // Disconnect all input blocks and remove all inputs.
        for (var i = this.columnCount_ - 1; i >= 0; i--) {
            if (this.getInput('COL' + i))
            {
                this.removeInput('COL' + i);
            }
        }   
        this.columnCount_ = 0;
        // Rebuild the block's inputs.
        var colBlock = containerBlock.getInputTargetBlock('STACK');
        while (colBlock) {
            var input = this.appendValueInput('COL' + this.columnCount_)
                            .appendTitle('column ' + (this.columnCount_+1))
                            .setAlign(Blockly.ALIGN_RIGHT)
                            .setCheck('Number');
                
              // Reconnect any child blocks.
              if (colBlock.valueConnection_) {
                input.connection.connect(colBlock.valueConnection_);
              }
              this.columnCount_++;
              colBlock = colBlock.nextConnection && 
              colBlock.nextConnection.targetBlock();
        }
    },
    saveConnections: function(containerBlock) {
        // Store a pointer to any connected child blocks.
        var colBlock = containerBlock.getInputTargetBlock('STACK');
        var columnCount_ = 0;
        while (colBlock) {
            var input = this.getInput('COL' + columnCount_);
            colBlock.valueConnection_ = input && input.connection.targetConnection;
            columnCount_++;
            colBlock = colBlock.nextConnection &&
                colBlock.nextConnection.targetBlock();
        }
    }
};

Blockly.Language.row_container = {
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('row');
        this.appendStatementInput('STACK');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.row_column_value = {
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendDummyInput().appendTitle('column');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip('');
    }
};

Blockly.Language.axis_rotation = {
    // Rotate matrix along an axis.
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown([['x axis','X'],['y axis','Y'], ['z axis', 'Z']]);
        this.appendValueInput('DEG')
            .setCheck('Number');
        this.appendDummyInput()
            .appendTitle('degree rotation about')
            .appendTitle(dropdown, 'MODE');
        this.setOutput(true,'Matrix');
        this.setInputsInline(true);
        this.setTooltip(
                        'Rotate matrix along an axis.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* axis (dropdown): Specify the axis of rotation\n' + 
                        '---\n' +
                        'Inputs:\n' +
                        '* unlabeld (value input): Specify the degree of rotation\n' +
                        'the only allowed connection type is \'Number\'\n' +
                        '---\n' + 
                        'Output:\n' +
                        '* Return the rotated matrix.\n' +
                        'The output type is \'matrix\'.'
                        );
    }
};

Blockly.Language.matrix_multiplication = {
    // Matrix multiplication operator
    helpUrl: null,
    init: function(){
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        this.appendValueInput('A')
        .setCheck(['Matrix','Vector', 'Number']);
        this.appendValueInput('B')
        .setCheck(['Matrix','Vector', 'Number'])
        .appendTitle('\u00D7');
        this.setInputsInline(true);
        this.setOutput(true,['Matrix','Vector']);
        this.setTooltip(
                        'Matrix multiplication.\n' + 
                        'Note: Please make sure that you perform allowed operations. Otherwise a run-time error will occur. Unallowed operation are e.g. multiplication of two vectors, etc.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* unlabeled (value input): Specify the vector or matrix of interest.\n' +
                        'The allowed connector types are \'Vector\' and \'Matrix\ as well as \'Number\'\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the result of the multiplication\n' +
                        'Output types are \'Vector\' and \'Matrix\'.'
                        );
    }
};

Blockly.Language.matrix_add_and_sub = {
    // Matrix addition and subtration 
    helpUrl: null,
    init: function(){
        this.setColour(Blockly.LANG_DUMMY_COLOUR);
        var dropdown = new Blockly.FieldDropdown([['+','ADD'],['-','SUB']]);
        this.appendValueInput('A')
        .setCheck(['Matrix','Vector','Number']);
        this.appendValueInput('B')
        .setCheck(['Matrix','Vector', 'Number'])
        .appendTitle(dropdown, 'MODE');
        this.setInputsInline(true)
        this.setOutput(true,['Matrix','Vector']);
        this.setTooltip(
                        'Perform matrix addition or subtraction\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* (dropdown): Choose an operator from the list.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* unlabeled (value input): Specify the vector or matrix of interest.\n' +
                        'The allowed connector types are \'Vector\' and \'Matrix\ as well as \'Number\'\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return the result of the specified operation\n' +
                        'Output types are \'Vector\' and \'Matrix\'.'
                        );
    }
};

Blockly.Language.matrix_create_frame = {
    // Create a homogeneous transform matrix
    helpUrl: null,
    init: function() {
      this.setColour(Blockly.LANG_DUMMY_COLOUR);
      this.appendDummyInput()
          .appendTitle("homogeneous transform")
          .appendTitle(new Blockly.FieldTextInput("frame_name"), "NAME");
      this.appendValueInput("TRANSLATION")
          .setCheck("Translation")
          .setAlign(Blockly.ALIGN_RIGHT)
          .appendTitle("translation (linear offset)")
          .setCheck(['Vector','Matrix']);
      this.appendValueInput("ROTATION")
          .setCheck("Matrix")
          .setAlign(Blockly.ALIGN_RIGHT)
          .appendTitle("rotation matrix (angular offset)");
      this.setOutput(true, "Matrix");
      this.setTooltip('');
  }
};
