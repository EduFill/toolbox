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
 * @fileoverview Code String.
 * @author marc.wollenweber@smail.inf.h-brs.de
 */
'use strict';

/**
 * Due to the frequency of long strings, the 80-column wrap rule need not apply
 * to message files.
 */
 
 
Blockly.GEN_HIGH_LEVEL_MOVE_ROBOT_TO_CUBE_DEF = 'def move_robot_to_cube(cube_color): \n' +
                                                 '  print \"Search for cube.\" \n' +
                                                 '  var cube_pos = find_cube(cube_color, 1, \"base_link\") \n' +
                                                 '  print \"IF found: Move base near cube. \\n ELSE: Turn on the spot OR move randomly and search again.\" \n';

Blockly.GEN_HIGH_LEVEL_GRASP_CUBE_DEF =  'def grasp_cube(cube_color): \n' +
                                              '  var cube_pos = find_cube(cube_color, 1, \"arm_link_0\") \n' +
                                              '  print \"Search for cube.\"' +
                                              '  print \"IF found: Move arm to cube. ELSE: error\" \n' +
                                              '  print \"Close gripper\" \n' +
                                              '  move_gripper_component.move(\"CLOSE\") \n';
                                              
Blockly.GEN_HIGH_LEVEL_BASE_PLACEMENT_DEF = 
    'def base_placement(): \n' +
    '  rospy.init_node(\'raw_base_placement_test_script\') \n\n' +
    '  ### tf listener \n' +
    '  tf_listener = tf.TransformListener() \n\n' +
    '  # BASE PLACEMENT \n' +
    '  shiftbase_srv = rospy.ServiceProxy(\'/raw_relative_movements/shiftbase\', raw_srvs.srv.SetPoseStamped) \n\n' +
    '  print \"wait for service: /raw_relative_movements/shiftbase\"   \n' +
    '  rospy.wait_for_service(\'/raw_relative_movements/shiftbase\', 30) \n\n' +
    '  goalpose = geometry_msgs.msg.PoseStamped() \n' +
    '  goalpose.pose.position.x = 0.1 \n' +
    '  goalpose.pose.position.y = 0.1 \n' +
    '  goalpose.pose.position.z = 0.1 \n' +
    '  quat = tf.transformations.quaternion_from_euler(0,0,0) \n' +
    '  goalpose.pose.orientation.x = quat[0] \n' +
    '  goalpose.pose.orientation.y = quat[1] \n' +
    '  goalpose.pose.orientation.z = quat[2] \n' +
    '  goalpose.pose.orientation.w = quat[3] \n\n' +
    '  print \"GOAL POSE TRANSFORMED: \", goalpose \n' +
    '  # call base placement service \n' +
    '  base_pose = moveoptimalbase_srv(goalpose) \n\n' +  
    '  goalpose = geometry_msgs.msg.PoseStamped() \n' +
    '  goalpose.pose.position.x = -0.1 \n' +
    '  goalpose.pose.position.y = -0.1 \n' +
    '  goalpose.pose.position.z = 0.1 \n' +
    '  quat = tf.transformations.quaternion_from_euler(0,0,0) \n' +
    '  goalpose.pose.orientation.x = quat[0] \n' +
    '  goalpose.pose.orientation.y = quat[1] \n' +
    '  goalpose.pose.orientation.z = quat[2] \n' +
    '  goalpose.pose.orientation.w = quat[3] \n\n' +
    '  print \"GOAL POSE TRANSFORMED: \", goalpose \n' +
    '  # call relative movment service \n' +
    '  base_pose = shiftbase_srv (goalpose)  \n';

Blockly.GEN_HIGH_LEVEL_SIMPLE_ARM_MOVEMENT_DEF = 
    'def simple_arm_movement: \n'+ 
    '  pub = rospy.Publisher(\'arm_1/arm_controller/position_command\', JointPositions) \n' +
    '  rospy.init_node(\'simple_arm_gripper_position\') \n' +
    '  rospy.sleep(0.5) \n' +
    '\n' +
    '  try: \n' +
    '    jp = JointPositions() \n' +
    '\n' +
    '    jv1 = JointValue() \n' +
    '    jv1.joint_uri = \"arm_joint_1\" \n' +
    '    jv1.value = 0.1 \n' +
    '    jv1.unit = \"rad\" \n' +
    '\n' +    
    '    jv2 = JointValue() \n' +
    '    jv2.joint_uri = \"arm_joint_2\" \n' +
    '    jv2.value = 0.1 \n' +
    '    jv2.unit = \"rad\" \n' +
    '\n' +
    '    jv3 = JointValue() \n' +
    '    jv3.joint_uri = \"arm_joint_3\" \n' +
    '    jv3.value = -0.1 \n' +
    '    jv3.unit = \"rad\" \n' +
    '\n' +    
    '    jv4 = JointValue() \n' +
    '    jv4.joint_uri = \"arm_joint_4\" \n' +
    '    jv4.value = 0.1 \n' +
    '    jv4.unit = \"rad\" \n' +
    '\n' +   
    '    jv5 = JointValue() \n' +
    '    jv5.joint_uri = \"arm_joint_5\" \n' +
    '    jv5.value = 0.1 \n' +
    '    jv5.unit = \"rad\" \n' +
    '\n' +   
    '    p = Poison() \n' +
    '    print p  \n' +
    '    jp.poisonStamp = p \n' +
    '    jp.positions = [jv1, jv2, jv3, jv4, jv5] \n' +
    '\n' +  
    '    pub.publish(jp) \n' +
    '    pub.publish(jp) \n' +
    '\n' +    
    '  except Exception, e: \n' +
    '    print e';
                                
Blockly.GEN_MID_LEVEL_FIND_CUBE_DEF =  'def find_cube (cube_color, reference_frame):\n' +
                              '  #Dummy body with a dummy list and a print command \n' +
                              '  print \"Search for \" + cube_color + \" colored cube related to \" + reference_frame + \".\" \n'  +
                              '  list_of_transforms = [[0, 0, 0],[1,0,1]] \n' +
                              '  return list_of_transforms \n';

Blockly.GEN_MID_LEVEL_ROS_MOVE_BASE_TWIST_DEF = 
                'def ros_move_base_twist (lx, ly, lz, ax, ay, az):\n' +
                '  pub = rospy.Publisher(\"cmd_vel\", Twist) \n'+
                '\n'                    +
                '  twist = Twist() \n'  +
                '  # set velocities \n' +
                '  twist.linear.x = lx \n' + 
                '  twist.linear.y = ly \n' +
                '  twist.linear.z = lz \n' +
                '  twist.angular.x = ax \n' +
                '  twist.angular.y = ay \n' +
                '  twist.angular.z = az \n' +
                '  # publish the topic \n'+
                '  pub.publish(twist) \n'   +
                '\n'
