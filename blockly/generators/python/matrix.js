/**
 * Visual Blocks Language
 *
 * Copyright 2012 Google Inc.
 * http://code.google.com/p/blockly/
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
 *  * @fileoverview Generating Python for matrix and vector blocks.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';
 
Blockly.Python = Blockly.Generator.get('Python');
 
if (!Blockly.Python.RESERVED_WORDS_) {
  Blockly.Python.RESERVED_WORDS_ = '';
}

Blockly.Python.vector = function() {
    Blockly.Python.definitions_['import_numpy_as_np'] = 'import numpy as np'
    Blockly.Python.RESERVED_WORDS_ += 'numpy,np,';
    var code = '['
    var valueList = [];
    for (var i=0; i < this.valueCount_; i++) {
        var value = Blockly.Python.valueToCode(this, 'VALUE' + i, Blockly.Python.ORDER_NONE) || '0';
        valueList.push('[' + value + ']');
    }
    
    code = 'np.array([' + valueList.join(',') + '])';
    return [code, Blockly.Python.ORDER_MEMBER]
};

Blockly.Python.matrix = function() {
    Blockly.Python.definitions_['import_numpy_as_np'] = 'import numpy as np'
    Blockly.Python.RESERVED_WORDS_ += 'numpy,np,';
    var code = '['
    var rowList = [];
    for (var i=0; i < this.rowCount_; i++) {
        var row = Blockly.Python.valueToCode(this, 'ROW' + i, Blockly.Python.ORDER_NONE) || '[0]';
        rowList.push(row);
    }
    
    code = 'np.array([' + rowList.join(',') + '])';
    return [code, Blockly.Python.ORDER_MEMBER]
};

Blockly.Python.row = function() {
    Blockly.Python.definitions_['import_numpy_as_np'] = 'import numpy as np'
    Blockly.Python.RESERVED_WORDS_ += 'numpy,np,';
    var code = '[';
    var colList = [];
    for (var i=0; i < this.columnCount_; i++) {
        var column = Blockly.Python.valueToCode(this, 'COL' + i, Blockly.Python.ORDER_NONE) || '0';
        colList.push(column);
    }
    
    code = '[' + colList.join(',') + ']';
    return [code, Blockly.Python.ORDER_MEMBER]
};

Blockly.Python.axis_rotation = function(){
    Blockly.Python.definitions_['import_numpy_as_np'] = 'import numpy as np';
    Blockly.Python.definitions_['import_math'] = 'import math';
    Blockly.Python.RESERVED_WORDS_ += 'numpy,np,math,';
    var axis = this.getTitleValue('MODE');
    var degree = Blockly.Python.valueToCode(this, 'DEG', Blockly.Python.ORDER_NONE) || '0';
    var code = '';
    if (axis == 'X'){
        code = 'np.array([\\\n' +
        '[1,0,0],\\\n'+ 
        '[0,math.cos(' + degree + ' / 180.0 * math.pi),-math.sin(' + degree + ' / 180.0 * math.pi)],\\\n' +
        '[0,math.sin(' + degree + ' / 180.0 * math.pi),math.cos(' + degree + ' / 180.0 * math.pi)]\\\n' +
        '])'
    }
    else if (axis == 'Y') {
        code = 'np.array([\\\n' +
        '[math.cos(' + degree + ' / 180.0 * math.pi), 0 ,math.sin(' + degree +' / 180.0 * math.pi)],\\\n' + 
        '[0,1,0],\\\n' + 
        '[-math.sin(' + degree + ' / 180.0 * math.pi),0,math.cos(' + degree + ' / 180.0 * math.pi)]\\\n' +
        '])';
    }
    else if (axis == 'Z') {
        code = 'np.array([\\\n' + 
        '[math.cos(' + degree + ' / 180.0 * math.pi),-math.sin(' + degree + ' / 180.0 * math.pi),0],\\\n' +
        '[math.sin(' + degree + ' / 180.0 * math.pi),math.cos(' + degree + ' / 180.0 * math.pi),0],\\\n' +
        '[0,0,1]\\\n' +
        '])'
    }   
    
    return [code, Blockly.Python.ORDER_ATOMIC];
}

Blockly.Python.matrix_multiplication = function(){
    Blockly.Python.definitions_['import_numpy_as_np'] = 'import numpy as np'
    Blockly.Python.RESERVED_WORDS_ += 'numpy,np,';
    var A = Blockly.Python.valueToCode(this, 'A', Blockly.Python.ORDER_NONE);
    var B = Blockly.Python.valueToCode(this, 'B', Blockly.Python.ORDER_NONE);
    var code = A + '.dot(' + B + ')';
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.matrix_add_and_sub = function() {
    Blockly.Python.definitions_['import_numpy_as_np'] = 'import numpy as np'
    Blockly.Python.RESERVED_WORDS_ += 'numpy,np,';
    var A = Blockly.Python.valueToCode(this, 'A', Blockly.Python.ORDER_NONE);
    var B = Blockly.Python.valueToCode(this, 'B', Blockly.Python.ORDER_NONE);
    var operator = Blockly.Python.matrix_add_and_sub_operators[this.getTitleValue('MODE')];
    
    var code = A + operator + B ;
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python.matrix_add_and_sub_operators = {
    ADD: '+',
    SUB: '-',
};

Blockly.Python.matrix_create_frame = function() {
    Blockly.Python.definitions_['import_numpy_as_np'] = 'import numpy as np'
    Blockly.Python.RESERVED_WORDS_ += 'numpy,np,';
    var value_translation = Blockly.Python.valueToCode(this, 'TRANSLATION', Blockly.Python.ORDER_NONE) || '[[0],[0],[0]]';
    var value_rotation = Blockly.Python.valueToCode(this, 'ROTATION', Blockly.Python.ORDER_NONE) || '[[1,0,0],[0,1,0],[0,0,1]]';
    var text_name = this.getTitleValue('NAME');
    var code = 'np.concatenate((np.concatenate((' + value_rotation + ',' + value_translation + '),axis=1),[[0,0,0,1]]),axis=0)'
    
    return [code, Blockly.Python.ORDER_ATOMIC];
};
