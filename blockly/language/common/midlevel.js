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
 * @fileoverview Mid level blocks for Blockly.
 * @author marc.wollenweber@smail.inf.h-brs.de 
 */
 'use strict';

Blockly.Language.midlevel_move_gripper = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;
            
            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('move gripper');
        this.appendValueInput('GRIPPER_LEFT')
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendTitle('left gripper value')
            .setCheck('Number');
        this.appendValueInput('GRIPPER_RIGHT')
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendTitle('right gripper value')
            .setCheck('Number');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        var thisBlock = this;
        this.setTooltip(
                        'Move the youBot gripper to desired positions.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* ' + Blockly.ChangeModeTooltip + '\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* Left gripper (value input): Specify the position for the left gripper.\n' +
                        'The only allowed connection type is \'Number\'.\n' +
                        '* Right gripper (value input): Specify the position for the right gripper.\n' +
                        'The only allowed connection type is \'Number\'.'
                        );
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
            if(!block.getInput(block.changemode.appendInput)) {
                block.appendStatementInput(block.changemode.appendInput);
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
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        var attachedBlockGripperL = block.getInputTargetBlock('GRIPPER_LEFT');
        var attachedBlockGripperR = block.getInputTargetBlock('GRIPPER_RIGHT');
                
        if(block.getInput('GRIPPER_LEFT')){
            block.removeInput('GRIPPER_LEFT');
        }
        if(block.getInput('GRIPPER_RIGHT')){
            block.removeInput('GRIPPER_RIGHT');
        }
                
        var createPubBlock = block.changemode.appendNoneMutationStatement('lowlevel_create_publisher');
        var changemodeCreatePub = new Blockly.ChangeMode(createPubBlock)
        changemodeCreatePub.appendNoneMutationOutput('text',1).setTitleValue('arm_1/gripper_controller/position_command','TEXT');
        changemodeCreatePub.appendNoneMutationOutput('ros_jointpositions_msg_type',2);
        var sleepBlock = block.changemode.appendNoneMutationStatement('lowlevel_ros_sleep');
        new Blockly.ChangeMode(sleepBlock).appendNoneMutationOutput('math_number',1).setTitleValue('0.5','NUM');
        var publishBlock = block.changemode.appendNoneMutationStatement('lowlevel_publish');
        var gripperBlock = new Blockly.ChangeMode(publishBlock).appendNoneMutationOutput('rosmessage_gripper_position_msg', 1);
        var changemodeGripper = new Blockly.ChangeMode(gripperBlock);
        changemodeGripper.reconnectOutputConnection(attachedBlockGripperL, 1);
        block.changemode.addToReconnectMonitor(gripperBlock, 1, 1);
        changemodeGripper.reconnectOutputConnection(attachedBlockGripperR, 2);
        block.changemode.addToReconnectMonitor(gripperBlock, 2, 2);
    },
    disposeChangeMode: function() {
        var block = this;
        
        if(!block.getInput('GRIPPER_LEFT')) {
            block.appendValueInput('GRIPPER_LEFT')
                .setAlign(Blockly.ALIGN_RIGHT)
                .appendTitle('left gripper value');
        }
        if(!block.getInput('GRIPPER_RIGHT')) {
            block.appendValueInput('GRIPPER_RIGHT')
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendTitle('right gripper value');
        }
                
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};


Blockly.Language.midlevel_reference_frame = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        var dropdown =  new Blockly.FieldDropdown(this.OPERATORS);
        this.appendDummyInput()
            .appendTitle('reference frame')
            .appendTitle(dropdown,'MODE');
        this.setOutput(true, 'String');
        var thisBlock = this;
        this.setTooltip(
                        'Specify the frame of reference for an action, \n' + 
                        'i.e. move the robot arm to a goal pose with a coordinate system origin at the end-effector, or the robot base, etc.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* Reference frame (dropdown): Select the required frame of reference from the list.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a ROS transform (tf) entry according to the choice, e.g. /base_link.\n' +
                        'The output type is \'String\'.'
                        );
    }
};

Blockly.Language.midlevel_reference_frame.OPERATORS = 
    [[Blockly.LANG_MID_LEVEL_REFERENCE_FRAME_BASE, 'BASE'],
     [Blockly.LANG_MID_LEVEL_REFERENCE_FRAME_ARM, 'ARM'],
     [Blockly.LANG_MID_LEVEL_REFERENCE_FRAME_GRIPPER, 'GRIPPER']];

Blockly.Language.midlevel_ros_move_base_twist = {
  helpUrl: null,
  init: function() {
    this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
    this.appendDummyInput()
        .appendTitle('move robot base (twist)');
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.appendValueInput('L_X')
        .setAlign(Blockly.ALIGN_RIGHT)
        .setCheck('Number')
        .appendTitle(Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_LINEAR_X); // linear x
    this.appendValueInput('L_Y')
        .setAlign(Blockly.ALIGN_RIGHT)
        .setCheck('Number')
        .appendTitle(Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_LINEAR_Y); // linear y
    this.appendValueInput('L_Z')
        .setAlign(Blockly.ALIGN_RIGHT)
        .setCheck('Number')
        .appendTitle(Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_LINEAR_Z); // linear z
    this.appendValueInput('A_X')
        .setAlign(Blockly.ALIGN_RIGHT)
        .setCheck('Number')
        .appendTitle(Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_ANGULAR_X);// angular x
    this.appendValueInput('A_Y')
        .setAlign(Blockly.ALIGN_RIGHT)
        .setCheck('Number')
        .appendTitle(Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_ANGULAR_Y);// angular y
    this.appendValueInput('A_Z')
        .setAlign(Blockly.ALIGN_RIGHT)
        .setCheck('Number')
        .appendTitle(Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_ANGULAR_Z);// angular z
    this.setTooltip(Blockly.LANG_MID_LEVEL_MOVE_BASE_TWIST_TOOLTIP);
    this.setInputsInline(false);
    this.setTooltip(
                    'Move the robot base by velocities. \n' +
                    '---\n' +
                    'Inputs:\n' +
                    '* linear.x: Specify the velocity (m/s) in x direction.\n' +
                    'The only allowed connection type is \'Number\'.\n' +
                    '* linear.y: Specify the velocity (m/s) in y direction.\n' +
                    'The only allowed connection type is \'Number\'.\n' +
                    '* linear.z: Specify the velocity (m/s) in z direction.\n' +
                    'Note: This value is only required if the robot can fly, dive, or operate in weightlessness.\n' +
                    'The only allowed connection type is \'Number\'.\n' +
                    '* angular.x: Specify the velocity (rad/s) to rotate around the X-axis\n' +
                    'The only allowed connection type is \'Number\'.\n' +
                    'Note: This value is only required if the robot can fly, dive, or operate in weightlessness.\n' +
                    '* angular.y: Specify the velocity (rad/s) to rotate around the Y-axis\n' +
                    'The only allowed connection type is \'Number\'.\n' +
                    'Note: This value is only required if the robot can fly, dive, or operate in weightlessness.\n' +
                    '* angular.z: Specify the velocity (rad/s) to rotate around the Z-axis\n' +
                    'The only allowed connection type is \'Number\'.'
                    );
  }
};

Blockly.Language.midlevel_move_arm_joint_position = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.getInputJnt = 'JOINTS';
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;
            
            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('move robot arm to joint position');
        this.appendValueInput(this.getInputJnt)
            .appendTitle('<joint list>')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck(['JointAngles', 'Array']);
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip(
                        'Move the robot arm to the specified joint configuration.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* ' + Blockly.ChangeModeTooltip + '\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* Joint list (value input): Expected entries are \n' + 
                        '[Joint 1 angle (radians), joint 2 angle (radians), ... joint N angle (radians)].\n' +
                        'The only allowed connection type is \'JointAngles\'.'
                        );
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
        if (changeModeState == 'TRUE') {
            if (!block.getInput(this.changemode.appendInput)) {
                block.appendStatementInput(this.changemode.appendInput);
            }
            if (block.getInput(this.getInputJnt)) {
                block.removeInput(this.getInputJnt);
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
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        var blockAttachedAtJoint = block.getInputTargetBlock(block.getInputJnt);
        if (block.getInput(block.getInputJnt)) {
            block.removeInput(block.getInputJnt);
        }
        var lowlevel_create_publisherBlock0 = block.changemode.appendNoneMutationStatement('lowlevel_create_publisher');
        lowlevel_create_publisherBlock0.setTitleValue('pub','VAR');
        var lowlevel_create_publisherChangemode0 = new Blockly.ChangeMode(lowlevel_create_publisherBlock0);
        var textBlock1 = lowlevel_create_publisherChangemode0.appendNoneMutationOutput('text', 1);
        textBlock1.setTitleValue('arm_controller_handler/position_command','TEXT');
        var ros_jointstate_msg_typeBlock2 = lowlevel_create_publisherChangemode0.appendNoneMutationOutput('ros_jointstate_msg_type', 2);
        var lowlevel_ros_sleepBlock3 = block.changemode.appendNoneMutationStatement('lowlevel_ros_sleep');
        var lowlevel_ros_sleepChangemode3 = new Blockly.ChangeMode(lowlevel_ros_sleepBlock3);
        var math_numberBlock4 = lowlevel_ros_sleepChangemode3.appendNoneMutationOutput('math_number', 1);
        math_numberBlock4.setTitleValue('0.5','NUM');
        var try_catchBlock5 = block.changemode.appendNoneMutationStatement('try_catch');
        var try_catchChangemode5 = new Blockly.ChangeMode(try_catchBlock5);
        try_catchChangemode5.changeStatementInputTo('TRY');
        var variables_setBlock6 = try_catchChangemode5.appendNoneMutationStatement('variables_set');
        variables_setBlock6.setTitleValue('numberOfJoints','VAR');
        var variables_setChangemode6 = new Blockly.ChangeMode(variables_setBlock6);
        var lowlevel_get_paramBlock7 = variables_setChangemode6.appendNoneMutationOutput('lowlevel_get_param', 1);
        lowlevel_get_paramBlock7.setTitleValue('NUMBER_ARM_JOINTS','MODE');
        var variables_setBlock8 = try_catchChangemode5.appendNoneMutationStatement('variables_set');
        variables_setBlock8.setTitleValue('armJointValueList','VAR');
        var variables_setChangemode8 = new Blockly.ChangeMode(variables_setBlock8);
        variables_setChangemode8.reconnectOutputConnection(blockAttachedAtJoint, 1);
        block.changemode.addToReconnectMonitor(variables_setBlock8, 1, 1);
        var variables_setBlock10 = try_catchChangemode5.appendNoneMutationStatement('variables_set');
        variables_setBlock10.setTitleValue('jp','VAR');
        var variables_setChangemode10 = new Blockly.ChangeMode(variables_setBlock10);
        var ros_jointstate_msgBlock11 = variables_setChangemode10.appendNoneMutationOutput('ros_jointstate_msg', 1);
        var rosmessage_joint_state_msg_nameBlock12 = try_catchChangemode5.appendNoneMutationStatement('rosmessage_joint_state_msg_name');
        rosmessage_joint_state_msg_nameBlock12.setTitleValue('jp','VAR');
        var rosmessage_joint_state_msg_nameChangemode12 = new Blockly.ChangeMode(rosmessage_joint_state_msg_nameBlock12);
        var textBlock13 = rosmessage_joint_state_msg_nameChangemode12.appendNoneMutationOutput('text', 1);
        textBlock13.setTitleValue('arm_joint_','TEXT');
        var variables_getBlock14 = rosmessage_joint_state_msg_nameChangemode12.appendNoneMutationOutput('variables_get', 2);
        variables_getBlock14.setTitleValue('numberOfJoints','VAR');
        var rosmessage_joint_state_msg_positionBlock15 = try_catchChangemode5.appendNoneMutationStatement('rosmessage_joint_state_msg_position');
        rosmessage_joint_state_msg_positionBlock15.setTitleValue('jp','VAR');
        var rosmessage_joint_state_msg_positionChangemode15 = new Blockly.ChangeMode(rosmessage_joint_state_msg_positionBlock15);
        var variables_getBlock16 = rosmessage_joint_state_msg_positionChangemode15.appendNoneMutationOutput('variables_get', 1);
        variables_getBlock16.setTitleValue('armJointValueList','VAR');
        var variables_getBlock17 = rosmessage_joint_state_msg_positionChangemode15.appendNoneMutationOutput('variables_get', 2);
        variables_getBlock17.setTitleValue('numberOfJoints','VAR');
        var lowlevel_publishBlock18 = try_catchChangemode5.appendNoneMutationStatement('lowlevel_publish');
        lowlevel_publishBlock18.setTitleValue('pub','VAR');
        var lowlevel_publishChangemode18 = new Blockly.ChangeMode(lowlevel_publishBlock18);
        var variables_getBlock19 = lowlevel_publishChangemode18.appendNoneMutationOutput('variables_get', 1);
        variables_getBlock19.setTitleValue('jp','VAR');
        
        var lowlevel_ros_logBlock20 = try_catchChangemode5.appendNoneMutationStatement('lowlevel_ros_log');
        lowlevel_ros_logBlock20.setTitleValue('INFO','MODE');
        var lowlevel_ros_logChangemode6 = new Blockly.ChangeMode(lowlevel_ros_logBlock20);
        var textBlock21 = lowlevel_ros_logChangemode6.appendNoneMutationOutput('text', 1);
        textBlock21.setTitleValue('Move robot arm successful','TEXT');
        
        try_catchChangemode5.changeStatementInputTo('CATCH');
        var lowlevel_ros_logBlock22 = try_catchChangemode5.appendNoneMutationStatement('lowlevel_ros_log');
        lowlevel_ros_logBlock22.setTitleValue('ERROR','MODE');
        var lowlevel_ros_logChangemode7 = new Blockly.ChangeMode(lowlevel_ros_logBlock22);
        var text_joinBlock23 = lowlevel_ros_logChangemode7.appendMutationOutput('text_join',1,[['itemsCount_',2]] );
        var text_joinChangemode8 = new Blockly.ChangeMode(text_joinBlock23);
        var textBlock24 = text_joinChangemode8.appendNoneMutationOutput('text', 1);
        textBlock24.setTitleValue('Move robot arm failed:','TEXT');
        var variables_getBlock25 = text_joinChangemode8.appendNoneMutationOutput('variables_get', 2);
        variables_getBlock25.setTitleValue('e','VAR');
    },
    disposeChangeMode: function() {
        var block = this;
        if (!block.getInput(block.getInputJnt)) {
            block.appendValueInput(block.getInputJnt)
                .setAlign(Blockly.ALIGN_RIGHT)
                .appendTitle('<joint list>')
                .setAlign(Blockly.ALIGN_RIGHT)
                .setCheck('JointAngles');
            }
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};

Blockly.Language.midlevel_move_base_relative = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle('move robot base relative');
        this.appendValueInput('POSE')
            .appendTitle('<6D pose>')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Pose6D');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip(
                        'Move the robot base relative to its actual position.\n' +
                        'Note: The local coordinate system has the robot in its origin (origin: all values are zero). \n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* 6D pose (value input): The expected entry is a 6d pose  \n' + 
                        'The only allowed connection type is \'Pose6D\'.'
                        );
    }
};

Blockly.Language.midlevel_move_base_to_pose = {
    helpUrl: null,
    init: function() {
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        this.getInputPose = 'POSE';
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;
            if(changeModeState == true) {
                block.showChangeMode();
             }
             else if (changeModeState == false) {
                block.disposeChangeMode();
            }   
        });
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('move robot base to pose');
        this.appendValueInput(this.getInputPose)
            .appendTitle('<6D pose>')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Pose6D');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setTooltip(
                        'Move the robot base to a specified 6D pose.\n' +
                        'Note that the pose is relative to the world (or the robots believe of the world) and not to the robot itself.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* ' + Blockly.ChangeModeTooltip + '\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* 6D pose (value input): The expected entry is a 6d pose  \n' + 
                        'The only allowed connection type is \'Pose6D\'.'
                        );
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
        if (changeModeState == 'TRUE') {
            if (!block.getInput(this.changemode.appendInput)) {
                block.appendStatementInput(this.changemode.appendInput);
            }
            if (block.getInput(this.getInputPose)) {
                block.removeInput(this.getInputPose);
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
        block.changemode.setChangeMode(true);
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        var attachedPoseBlock = block.getInputTargetBlock(block.getInputPose);
        if (block.getInput(block.getInputPose)) {
            block.removeInput(block.getInputPose);
        }
                
        var setListBlock = block.changemode.appendNoneMutationStatement('variables_set');
        setListBlock.setTitleValue('basePoseList', 'VAR');
        var changemodePose = new Blockly.ChangeMode(setListBlock);
        changemodePose.reconnectOutputConnection(attachedPoseBlock, 1);
        block.changemode.addToReconnectMonitor(setListBlock, 1, 1);
        var trycatchBlock = block.changemode.appendNoneMutationStatement('try_catch');
        var changemodeTryCatch = new Blockly.ChangeMode(trycatchBlock, 'TRY');
        var setQuaternionBlock = changemodeTryCatch.appendNoneMutationStatement('variables_set');
        setQuaternionBlock.setTitleValue('quaternion', 'VAR');
        var qFromEuBlock = new Blockly.ChangeMode(setQuaternionBlock).appendNoneMutationOutput('lowlevel_quaternion_from_euler', 1);
        var rollBlock = new Blockly.ChangeMode(qFromEuBlock).appendNoneMutationOutput('math_number',1);
        var pitchBlock = new Blockly.ChangeMode(qFromEuBlock).appendNoneMutationOutput('math_number',2);
        var yawBlock = new Blockly.ChangeMode(qFromEuBlock).appendNoneMutationOutput('units_get_6dpose_element',3);
        yawBlock.setTitleValue('YAW', 'MODE');
        var changemodeYaw = new Blockly.ChangeMode(yawBlock);
        changemodeYaw.appendNoneMutationOutput('variables_get',1).setTitleValue('basePoseList', 'VAR');
        var setPoseBlock = changemodeTryCatch.appendNoneMutationStatement('variables_set');
        setPoseBlock.setTitleValue('pose', 'VAR');
        var poseMsgBlock = new Blockly.ChangeMode(setPoseBlock).appendNoneMutationOutput('rosmessage_base_pose_msg',1);
        var changemodePoseMsg = new Blockly.ChangeMode(poseMsgBlock);
        var xBlock = changemodePoseMsg.appendNoneMutationOutput('units_get_6dpose_element',1);
        xBlock.setTitleValue('LIN_X', 'MODE');
        var changemodeX = new Blockly.ChangeMode(xBlock);
        changemodeX.appendNoneMutationOutput('variables_get',1).setTitleValue('basePoseList', 'VAR');
        var yBlock = changemodePoseMsg.appendNoneMutationOutput('units_get_6dpose_element',2);
        yBlock.setTitleValue('LIN_Y', 'MODE');
        var changemodeY = new Blockly.ChangeMode(yBlock);
        changemodeY.appendNoneMutationOutput('variables_get',1).setTitleValue('basePoseList', 'VAR');
        changemodePoseMsg.appendNoneMutationOutput('math_number',3);
        changemodePoseMsg.appendNoneMutationOutput('variables_get',4).setTitleValue('quaternion', 'VAR');
        var setClientBlock = changemodeTryCatch.appendNoneMutationStatement('variables_set');
        setClientBlock.setTitleValue('client', 'VAR');
        var simpleActionClientBlock = new Blockly.ChangeMode(setClientBlock).appendNoneMutationOutput('action_simple_action_client',1);
        simpleActionClientBlock.setTitleValue('/move_base','ACTION_SERVER_NAME');
        new Blockly.ChangeMode(simpleActionClientBlock).appendNoneMutationOutput('action_move_base_action',1);
        var waitForServerBlock = changemodeTryCatch.appendNoneMutationStatement('action_client_wait_for_server');
        new Blockly.ChangeMode(waitForServerBlock).appendNoneMutationOutput('variables_get', 1).setTitleValue('client', 'VAR');
        var sendGoalBlock = changemodeTryCatch.appendNoneMutationStatement('action_client_send_goal');
        var changemodeSendGoal = new Blockly.ChangeMode(sendGoalBlock);
        changemodeSendGoal.appendNoneMutationOutput('variables_get',1).setTitleValue('client', 'VAR');
        var moveBaseGoalBlock = changemodeSendGoal.appendNoneMutationOutput('lowlevel_move_base_goal',2);
        new Blockly.ChangeMode(moveBaseGoalBlock).appendNoneMutationOutput('variables_get',1).setTitleValue('pose', 'VAR');
        var waitForServerBlock = changemodeTryCatch.appendNoneMutationStatement('action_client_wait_for_result');
        new Blockly.ChangeMode(waitForServerBlock).appendNoneMutationOutput('variables_get', 1).setTitleValue('client', 'VAR');
                
        changemodeTryCatch.changeStatementInputTo('CATCH');
        var lowlevel_ros_logBlock = changemodeTryCatch.appendNoneMutationStatement('lowlevel_ros_log');
        lowlevel_ros_logBlock.setTitleValue('ERROR','MODE');
        var changemode_lowlevel_ros_log = new Blockly.ChangeMode(lowlevel_ros_logBlock);
        var getBlock = changemode_lowlevel_ros_log.appendNoneMutationOutput('variables_get', 1);
        getBlock.setTitleValue('e','VAR');
    },
    disposeChangeMode: function() {
        var block = this;
        if (!block.getInput(block.getInputPose)) {
            block.appendValueInput(block.getInputPose)
                .appendTitle('<6D pose>')
                .setAlign(Blockly.ALIGN_RIGHT)
                .setCheck('Pose6D');
        }
        block.changemode.setChangeMode(false);
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
};

Blockly.Language.midlevel_ik_checker = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle('inverse kinematics checker')
            .appendTitle('(')
            .appendTitle(new Blockly.FieldDropdown(Blockly.CONFIG_IK_SOLVER), 'MODE')
            .appendTitle(')');
        this.appendValueInput('REF')
            .appendTitle('frame of reference')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('String');
        this.appendValueInput('POSE6D')
            .appendTitle('6D pose')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('Pose6D');
        this.setOutput(true, 'Boolean'); 
        this.setTooltip(
                        'Determine if a cartesian pose can be reached.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* Solver (dropdown): Select the algorithm to calculate the inverse kinematics from the list.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* 6D pose: Expected entries are \n' + 
                        'x position (meters), y position (meters), z position (meters), angle around X-axis (radians), angle around Y-axis (radians), angle around Z-axis (radians)]\n' +
                        'The only allowed connection type is \'Pose6D\'.\n' +
                        '* Frame of reference: Any available frame.\n' +
                        'The only allowed connection type is \'String\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* If joint configuration available: True. Else: False.\n' +
                        'The output type is \'Boolean\'.'
                        );
    }
};

Blockly.Language.midlevel_ik_solver = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle('inverse kinematics solver')
            .appendTitle('(')
            .appendTitle(new Blockly.FieldDropdown(Blockly.CONFIG_IK_SOLVER), 'MODE')
            .appendTitle(')');
        this.appendValueInput('REF')
            .appendTitle('frame of reference')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('String');
        this.appendValueInput('POSE6D')
            .setAlign(Blockly.ALIGN_RIGHT)
            .appendTitle('6D pose')
            .setCheck('Pose6D');
        this.setOutput(true,['Array', 'JointAngles']); 
        this.setTooltip(
                        'Calculate the joint positions (angles) depending on a given cartesian pose and a frame of reference.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* Solver (dropdown): Select the algorithm to calculate the inverse kinematics from the list.\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* 6D pose: Expected entries are \n' + 
                        'x position (meters), y position (meters), z position (meters), angle around X-axis (radians), angle around Y-axis (radians), angle around Z-axis (radians)]\n' +
                        'The only allowed connection type is \'Pose6D\'.\n' +
                        '* Frame of reference: Any available frame.\n' +
                        'The only allowed connection type is \'String\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a set of joint angles [joint angle 1 (radians), joint angle 2 (radians, joint angle N (radians)].\n' +
                        'The output types are \'JointAngles\' and \'Array\'.'
                        );
    }
};

Blockly.Language.midlevel_fk_solver = {
    helpUrl: null,
    init: function() {
        this.variableName = 'kinematics_solver';
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.appendDummyInput()
            .appendTitle('forward kinematics solver');
        this.appendValueInput('JOINTS')
            .appendTitle('<joint list>')
            .setAlign(Blockly.ALIGN_RIGHT)
            .setCheck('JointAngles');
        this.setOutput(true,['Array', 'Pose6D']); 
        this.setTooltip(
                        'According to a set of joint angles calculate the resulting cartesian pose (6D pose).\n' +
                        '---\n' + 
                        'Inputs:\n' +
                        '* Joint list (value input): Expected entries are [Joint angle 1 (radians), joint angle 2 (radians), ... , joint angle N (radians)].\n' +
                        'The only allowed connection type is \'JointAngles\'.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a 6D pose (x y z position in meters and rotation around X-, Y-, and Z-axis in radians).\n' +
                        'The output types are \'Pos6d\' and \'Array\'.'
                        );
    },
    getVars: function() {
        return [this.variableName];
    }
};

Blockly.Language.midlevel_mapping = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.dropdown = new Blockly.FieldDropdown([['start','START'],['store','STORE'],['stop','STOP']], function(state) {
            var block = this.sourceBlock_;
            if(state == 'STORE') {
                if(!block.getInput('FILE')) {
                    block.appendValueInput('FILE')
                        .appendTitle('to file')
                        .setCheck('String');
                }
            }
            else {
                if(block.getInput('FILE')) {
                    block.removeInput('FILE');
                }
            }
        }); 
        this.appendDummyInput('MAP')
        .appendTitle('mapping:')
        .appendTitle(this.dropdown, 'MODE');
        this.setPreviousStatement(true);
        this.setNextStatement(true);
        this.setInputsInline(true);
        this.setTooltip(
                        'Start, store or stop mapping.\n' +
                        '---\n' +
                        'Fields:\n' +
                        '* mapping (dropdown): Select an operation from the list.\n' +
                        'Note: Whenever the store entry is chosen an additional value input area occurs (more information below).\n' +
                        '---\n' +
                        'Inputs:\n' +
                        '* File (value input): This entry is only visible, when the option store is chosen. The file name can be specified.\n' +
                        'The only allowed connection type is \'String\'.' 
                        );
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('MODE');
        container.setAttribute('dropdown', state);
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var dropdownState = xmlElement.getAttribute('dropdown');
        var block = this;
        if(dropdownState == 'STORE') {
            if(!block.getInput('FILE')) {
                block.appendValueInput('FILE')
                .appendTitle('to file')
                .setCheck('String');
            }
        }
    }
};

Blockly.Language.midlevel_read_map_location = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.appendDummyInput().appendTitle('get map location');
        this.setOutput(true, 'Pose6D');
        this.setTooltip(
                        'Provide the global (perhaps guessed or inaccurate) global location of the robot within the map.\n' +
                        '---\n' +
                        'Output:\n' +
                        '* Return a 6D pose.\n' +
                        'The only allowed output type is \'Pose6D\'.'
                        );
    }
};

Blockly.Language.midlevel_check_wall = {
    helpUrl: null,
    init: function() {
        this.setColour(Blockly.LANG_MID_LEVEL_COLOUR);
        this.changemode = new Blockly.ChangeMode(this, 'INCHANGEMODE');
        this.image = new Blockly.FieldImage(Blockly.pathToBlockly + 'media/menu0.png', 12, 12);
        var checkbox = new Blockly.FieldCheckbox('FALSE', function(changeModeState) {
            var block = this.sourceBlock_;

            if(changeModeState == true) {
                block.showChangeMode();
            }
            else if (changeModeState == false) {
                block.disposeChangeMode();
            }
        });
        this.setOutput(true, 'Boolean');
        this.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_WALL);
        this.appendDummyInput()
            .appendTitle(this.image)
            .appendTitle(checkbox,'STATE')
            .appendTitle('check for a wall');
        this.appendValueInput('DISTANCE')
            .appendTitle('to the')
            .appendTitle(this.dropdown, 'MODE')
            .appendTitle('with max. distance (m) of')
            .setCheck('Number');
        this.setInputsInline(true);
    },
    mutationToDom: function() {
        var container = document.createElement('mutation');
        var state = this.getTitleValue('STATE');
        container.setAttribute('changemodestate', state);
        if (this.dropdownValue) {
            container.setAttribute('dropdownstate', this.dropdownValue);
        }
        
        return container;
    },
    domToMutation: function(xmlElement) {
        var changeModeState = xmlElement.getAttribute('changemodestate');
        var block = this;
        if(changeModeState == 'TRUE') {
            if(!block.getInput(block.changemode.appendInput)) {
                block.appendValueInput(block.changemode.appendInput);
            }
            if (block.getInput('DISTANCE')) {
                block.removeInput('DISTANCE');
                delete block.dropdown
            }
            block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        }
        
        var dropdownState = xmlElement.getAttribute('dropdownstate');
        if (dropdownState) {
            block.dropdownValue = dropdownState;
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
        block.setInputsInline(false);
        block.changemode.setChangeMode(true, 'output');
        block.image.setText(Blockly.pathToBlockly + 'media/menu1.png');
        
        var blockAttachedAtDistance = block.getInputTargetBlock('DISTANCE');
        var direction = Blockly['CONFIG_WALL_ANGLE_' + block.getTitleValue('MODE')];
        
        if (block.getInput('DISTANCE')) {
            block.removeInput('DISTANCE');
            block.dropdownValue = block.dropdown.getValue();
            delete block.dropdown;
        }
        var lowlevel_is_wallBlock = block.changemode.appendNoneMutationOutput('lowlevel_is_wall', 1);
        var lowlevel_is_wallChangemode = new Blockly.ChangeMode(lowlevel_is_wallBlock);
        var math_numberBlock = lowlevel_is_wallChangemode.appendNoneMutationOutput('math_number', 1);
        math_numberBlock.setTitleValue(direction,'NUM');
        lowlevel_is_wallChangemode.reconnectOutputConnection(blockAttachedAtDistance, 2);
        block.changemode.addToReconnectMonitor(lowlevel_is_wallBlock, 2, 1);
        
    },
    disposeChangeMode: function() {
        var block = this;
        
        if (!block.getInput('DISTANCE')) {
            block.dropdown = new Blockly.FieldDropdown(Blockly.CONFIG_WALL);
            if (block.dropdownValue) {
                block.dropdown.setValue(block.dropdownValue);
            }
            block.appendValueInput('DISTANCE')
                .appendTitle('to the')
                .appendTitle(block.dropdown, 'MODE')
                .appendTitle('with max. distance (m) of')
                .setCheck('Number');
            block.setInputsInline(true);
        }
        block.setInputsInline(true);
        block.moveInputBefore('DISTANCE', block.changemode.appendInput);
        block.changemode.setChangeMode(false, 'output');
        block.image.setText(Blockly.pathToBlockly + 'media/menu0.png');
    }
}
