#!/usr/bin/env python
""" 
Manipulation state machines
"""

__author__ = "Gonzalo Olave, Rodrigo Munoz"

import rospy
import smach
import smach_ros

# robot
from bender_skills import robot_factory

# msgs
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import FollowJointTrajectoryResult

# temporal
from bender_arm_control.arm_commander import Limb
from tf import transformations
import tf

class Sleep(smach.State):
    def __init__(self,robot,stime):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.stime = stime
    def execute(self,userdata):
        rospy.sleep(self.stime)
        return 'succeeded'

def get_pose(position, roll=0, pitch=0, yaw=0):
    p = Pose()
    p.position = position
    q = transformations.quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p



def get_collision_cylinder(posestamped, name, target=["CYLINDER", 0.05, 0.05]):
    obj = CollisionObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = posestamped.header.frame_id
    obj.id = name
    # Create a cylinder primitive shape
    cylinder_pose = posestamped.pose
    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER

    cylinder.dimensions = [0.0]*2
    cylinder.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = target[1]
    cylinder.dimensions[SolidPrimitive.CYLINDER_RADIUS] = target[2]
    # Fill collision object with the cylinder
    obj.primitives.append(cylinder)
    obj.primitive_poses.append(cylinder_pose)
    obj.operation = CollisionObject.ADD
    return obj

def get_object_definition(name="water"):
    objects_list = ""
    if not rospy.has_param("/objects_list"):
        rospy.logerr("\"/objects_list\" parameter does not exists.")
        rospy.logerr("Mapper file can be loaded using \"roslaunch uchile_states manipulation.launch\"")
        rospy.logerr("Using default object description.")
        return ["CYLINDER", 0.10, 0.05]
        
    objects_list = rospy.get_param("/objects_list")
    if name in objects_list:
        return objects_list[name]

    return objects_list['default']

class ArmDesignator(smach.State):
    REF_FRAME = "bender/base_link"
    """
    Select arm based on Y axis position.

    io_keys:
        object (world_definition.Object): Recognized object.

    side:
        side (str): Select side.

    outcomes:
        succeeded: Succeeded arm selection.
        aborted: Failed transform between object pose and reference frame.
    """
    def __init__(self, robot,use_arm="both"):
        smach.State.__init__(self,  outcomes=["succeeded", "aborted"],
                                    io_keys=["object"],
                                    output_keys=["side"])
        self.robot = robot
        self.use_arm = use_arm

    def execute(self, userdata):
        # Transform to reference frame
        pose = userdata.object.posestamped
        print pose

        target_pose = PoseStamped()
        pose.header.stamp = rospy.Time() # Use last transform
        try:
            target_pose = self.robot.context.get_tf_listener().transformPose(ArmDesignator.REF_FRAME, pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.logerr("Error on transform from \"{}\" to \"{}\"".format(pose.header.frame_id, ArmDesignator.REF_FRAME))
            return "aborted"
        # Designate arm based on Y axis
        selected_side = "r"
        if target_pose.pose.position.y > 0.0:
            selected_side = "l"
        rospy.loginfo("Selected arm: \"{0}_arm\"".format(selected_side))
        # Override original frame
        userdata.object.posestamped = target_pose
        print target_pose

        if not self.use_arm == "both":
            selected_side = self.use_arm

            
        dim = get_object_definition(userdata.object.name)
        print dim[2]
        userdata.object.posestamped.pose.position.x += dim[2]/2.0
        # Hardcode arm offset for l_arm (gripper is more fat)
        userdata.object.posestamped.pose.position.z += 0.03
        if selected_side == "l":
            userdata.object.posestamped.pose.position.z += 0.02 # +2cm
        # Copy value to output key

        userdata.side = selected_side
        return "succeeded"

class JointGoal(smach.State):
    """
    Send joint goal to arm.

    input_keys:
        side (str): Arm side, must be "l" or "r".
        joint_goal (list of float): Joint target configuration, must follow arm.get_joint_names() order.

    output_keys:
        error_code (int): Error code from action server (FollowJointTrajectoryResult.error_code).

    outcomes:
        succeeded: Arm reach goal position.
        aborted: Arm timeout or other error (see error code).
    """
    def __init__(self, robot):
        smach.State.__init__(self,  outcomes=["succeeded", "aborted"],
                                    input_keys=["side", "joint_goal"],
                                    output_keys=["error_code"])
        self.robot = robot
        self.arm = None
        self.arm_name = None

    def execute(self, userdata):
        # Check arm name
        self.arm_name = userdata.side + "_arm"
        if self.arm_name != "l_arm" and self.arm_name != "r_arm":
            # Wrong name
            rospy.logerr("\"side\" must be \"l\" or \"r\"")
            userdata.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            return "aborted"
        # Get arm
        self.arm = self.robot.get(self.arm_name)
        if self.arm is None:
            # Error getting
            rospy.logerr("Error getting arm skill with name \"{0}\"".format(self.arm_name))
            userdata.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            return "aborted"
        # Send goal and wait
        self.arm.send_joint_goal(userdata.joint_goal)
        if not self.arm.wait_for_motion_done():
            # Arm did not reach the goal
            userdata.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            return "aborted"
        # Get result
        result = self.arm.get_result()
        if result is not None:
            if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                userdata.error_code = result.error_code
                return "aborted"
            return "succeeded"
        else:
            # Arm did not reach the goal
            userdata.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            return "aborted"

class OpenGripper(smach.State):
    """
    Open the gripper.

    input_keys:
        side (str): Gripper side, must be "l" or "r".
        effort (float): Max effort used in the movement. Must be between 0.0 and 1.0. Value zero may cause null movement.

    outcomes:
        succeeded: Gripper did not stalled and reach open default position.
        aborted: Gripper timeout, bad gripper side, 
    """
    def __init__(self, robot):
        smach.State.__init__(self,  outcomes=["succeeded","aborted"],
                                    input_keys=["side","effort"])
        self.robot = robot
        self.gripper = None
        self.gripper_name = None

    def execute(self,userdata):
        # Get gripper
        self.gripper_name = userdata.side + "_gripper"
        if self.gripper_name != "l_gripper" and self.gripper_name != "r_gripper":
            # Wrong name
            rospy.logerr("\"side\" must be \"l\" or \"r\"")
            return "aborted"
        self.gripper = self.robot.get(self.gripper_name)
        if self.gripper is None:
            # Error getting
            rospy.logerr("Error getting gripper skill with name \"{0}\"".format(self.gripper_name))
            return "aborted"
        # Send goal and wait
        self.gripper.open(userdata.effort)
        result = self.gripper.get_result()
        # Check result
        if not result.reached_goal:
            return "aborted"
        return "succeeded"

class CloseGripper(smach.State):
    """
    Close the gripper.

    input_keys:
        side (str): Gripper side, must be "l" or "r".
        effort (float): Max effort used in the movement. Must be between 0.0 and 1.0. Value zero may cause null movement.

    outcomes:
        succeeded: Gripper did not stalled and reach close default position.
        aborted: Gripper timeout.
    """
    def __init__(self,robot):
        smach.State.__init__(self,  outcomes=["succeeded","aborted"],
                                    input_keys=["side","effort"])
        self.robot = robot
        self.gripper = None
        self.gripper_name = None
        self.timeout = 10.0

    def execute(self,userdata):
        # Get gripper
        self.gripper_name = userdata.side + "_gripper"
        if self.gripper_name != "l_gripper" and self.gripper_name != "r_gripper":
            # Wrong name
            rospy.logerr("\"side\" must be \"l\" or \"r\"")
            return "aborted"
        self.gripper = self.robot.get(self.gripper_name)
        if self.gripper is None:
            # Error getting
            rospy.logerr("Error getting gripper skill with name \"{0}\"".format(self.gripper_name))
            return "aborted"
        self.gripper.close(userdata.effort, self.timeout)
        result = self.gripper.get_result()
        # Check result
        if result is None:
            rospy.logerr("Gripper \"{0}\" reached timeout.".format(self.gripper_name))
            return "aborted"
        # Check close condition
        if result.reached_goal:
            return "succeeded"
        return "aborted"

class GrabGripper(smach.State):
    """
    Grab object closing the gripper.

    outcomes:
        succeeded: Gripper stalled and did not reach close default position, 
            therefore have something (possibly an object) that prevents reach the goal.
        aborted: Gripper timeout.
    """
    def __init__(self,robot):
        smach.State.__init__(self,  outcomes=["succeeded", "aborted"],
                                    input_keys=["side", "effort"])
        self.robot = robot
        self.gripper = None
        self.gripper_name = None
        self.timeout = 10.0

    def execute(self,userdata):
        # Get gripper
        self.gripper_name = userdata.side + "_gripper"
        if self.gripper_name != "l_gripper" and self.gripper_name != "r_gripper":
            # Wrong name
            rospy.logerr("\"side\" must be \"l\" or \"r\"")
            return "aborted"
        self.gripper = self.robot.get(self.gripper_name)
        if self.gripper is None:
            # Error getting
            rospy.logerr("Error getting gripper skill with name \"{0}\"".format(self.gripper_name))
            return "aborted"
        # Send goal
        self.gripper.close(userdata.effort, self.timeout)
        result = self.gripper.get_result()
        if result is None:
            rospy.logerr("Gripper \"{0}\" reached timeout.".format(self.gripper_name))
            return "aborted"
        # Check grab condition
        if result.stalled and not result.reached_goal:
            return "succeeded"
        return "aborted"

class SetPositionNamed(smach.State):
    def __init__(self,robot,arm,blind=False,init=None,goal=None):
        smach.State.__init__(self,  outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['trayectory_name','side'])
        self.arm = arm
        self.blind = blind
        self.init = init
        self.goal = goal

    def execute(self, userdata):
        selected_arm = self.arm[userdata.side]
        if self.blind:
            selected_arm.arm.move_joint_blind(self.init,self.goal)
            rospy.sleep(2.0)
            selected_arm.arm.wait()
            return 'succeeded'
        elif self.blind is False or self.init is None or self.goal is None:
            if selected_arm.check_planning():
                if type(userdata.trayectory_name) == str:
                    if selected_arm.arm.set_position_named(userdata.trayectory_name): # move_group trayectory
                        return 'succeeded'
                    else:
                        return 'aborted'
                elif type(userdata.trayectory_name) == list:
                    for trayectory in userdata.trayectory_name:
                        result = selected_arm.arm.set_position_named(trayectory)
                        rospy.sleep(2.0)
                        if result: # move_group trayectory
                            continue
                        else:
                            return 'aborted'
                return 'succeeded'
            else:
                rospy.logwarn('No planning selected')
                return 'aborted'

class GetPossibleGrasp_capmap(smach.State): 
    def __init__(self, arm_dict):
        smach.State.__init__(self,  outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['object','side','it','possible_grasp'],
                                    output_keys=['possible_grasp','side','selected_grasp','selected_pregrasp'])
        self.arms = arm_dict
        self.dim = None
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):

        selected_arm = self.arms[userdata.side]
        dim = get_object_definition(userdata.object.name)
        cylinder = get_collision_cylinder(userdata.object.posestamped, userdata.object.name, dim)
        possible_grasp = selected_arm.arm.get_grasp_capmap(cylinder)

        index = 0
        if userdata.it>0:
            index = userdata.it%len(possible_grasp['grasp'])
            
        if not possible_grasp:
            # rospy.logwarn('Grasp net found using capability map, trying using online generator')
            # axial_res = 10
            # angle_res = 5
            # selected_arm.arm.generate_grasp(cylinder, axial_res, angle_res)
            # possible_grasp = selected_arm.arm.get_grasp()
            # if not possible_grasp.ik_solutions:
            #     rospy.logerr('Grasp not found')
            #     return 'aborted'
            # # 2n
            # userdata.selected_pregrasp = possible_grasp.ik_solutions[2*possible_grasp.order[0]].positions
            # # 2n+1
            # userdata.selected_grasp = possible_grasp.ik_solutions[2*possible_grasp.order[0]+1].positions
            
            #return 'succeeded'
            return 'aborted'
        rospy.loginfo('Se encontraron {} grasps'.format(len(possible_grasp['pregrasp'])))
        userdata.possible_grasp = possible_grasp
        userdata.selected_grasp = possible_grasp['grasp'][index]
        userdata.selected_pregrasp = possible_grasp['pregrasp'][index]

        return 'succeeded'

    def get_limb(self, posestamped):
        frame_out = "bender/base_link"
        # print posestamped
        if posestamped.header.frame_id!=frame_out:
            posestamped.header.stamp = rospy.Time() # Use last transform
            try:
                target_pose = self.tf_listener.transformPose(frame_out, posestamped)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print posestamped.header.frame_id + " / " + frame_out
                print "Error on transform"
                return 'l'

            posestamped = target_pose
            print "new pose"
            print posestamped

        if posestamped.position.y >= 0:
            return 'l'
        else:
            return 'r'

class PositionObjectAndGrasp_capmap(smach.State):
    def __init__(self,robot,arm_dict):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                            input_keys=['possible_grasp','side'])
        self.arms = arm_dict

    def execute(self,userdata):
        selected_arm = self.arms[userdata.side]

        self.pregrasp_joints = userdata.possible_grasp['pregrasp'][0]
        self.grasp_joints = userdata.possible_grasp['grasp'][0]
  
        self.result = selected_arm.arm.set_joint(self.pregrasp_joints) # Movimiento con planificador

        if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
            selected_arm.arm.move_joint(self.grasp_joints, interval = 2.5,segments = 20) # Movimiento con collisiones permitidas
            selected_arm.arm.wait()
            return 'succeeded'
        else:
            return 'aborted'

class PositionObject(smach.State):
    def __init__(self,robot,arm_dict):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                            input_keys=['possible_grasp','side'])
        self.arms = arm_dict

    def execute(self,userdata):
        selected_arm = self.arms[userdata.side]

        pregrasp_joints = userdata.possible_grasp['pregrasp'][0]
  
        self.result = selected_arm.arm.set_joint(pregrasp_joints) # Movimiento con planificador

        if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
            return 'succeeded'
        return 'aborted'


class Grasp_capmap(smach.State):
    def __init__(self,robot,arm_dict):
        smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'],
                            input_keys=['possible_grasp','side'])
        self.arms = arm_dict

    def execute(self,userdata):
        selected_arm = self.arms[userdata.side]

        self.grasp_joints = userdata.possible_grasp['grasp'][0]
  

        selected_arm.arm.move_joint(self.grasp_joints, interval = 1.5,segments = 20) # Movimiento con collisiones permitidas
        selected_arm.arm.wait()
        return 'succeeded'

        

class PositionEfector_capmap(smach.State):
    
    def __init__(self,limb):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys=['posestamped','height','radius','limb_side'])
        self.arms = limb

    def execute(self,userdata):

        self.limb = self.arms[userdata.limb_side]
        # Grasp obj
        self.object_pose = self.get_pose(userdata.posestamped.pose.position.x,userdata.posestamped.pose.position.y,userdata.posestamped.pose.position.z)
        self.pringles = self.get_collision_cylinder(self.object_pose, 'pringles', ["CYLINDER",userdata.height,userdata.radius])

        #self.limb.arm.generate_grasp(self.pringles, axial_res = 5, angle_res = 10)

        self.possible_grasp = self.limb.arm.get_grasp_capmap(self.pringles)
        if not self.possible_grasp:
            rospy.logerr('No se encontraron grasps')
            return

        rospy.loginfo('Se encontraron {} grasps'.format(len(self.possible_grasp['pregrasp'])))
            
        self.pregrasp_joints = self.possible_grasp['pregrasp'][0]
        self.grasp_joints = self.possible_grasp['grasp'][0]
  
        self.result = self.limb.arm.set_joint(self.pregrasp_joints) # Movimiento con planificador

        if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
            self.limb.arm.move_joint(self.grasp_joints, interval = 2.5,segments = 20) # Movimiento con collisiones permitidas
            rospy.sleep(3.0)
            self.limb.gripper.open(effort = 0.3)
            rospy.sleep(1.0)
            return 'succeeded'
        else:
            return 'aborted'
    
class grasp_capmap(smach.State):
    
    def __init__(self,limb):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                  input_keys=['possible_grasp','limb_side','max_attempts'])
        self.arms = limb

    def execute(self,userdata):

        self.limb = self.arms[userdata.limb_side]

        self.possible_grasp = userdata.possible_grasp
         
        #self.pregrasp_joints = self.possible_grasp['pregrasp'][0]
        #self.grasp_joints = self.possible_grasp['grasp'][0]
  
        for i,pregrasp_joints in enumerate(self.possible_grasp['pregrasp']):
            self.grasp_joints = self.possible_grasp['grasp'][i]
            self.result = self.limb.arm.set_joint(pregrasp_joints) # Movimiento con planificador

            if i == userdata.max_attempts:
                return 'aborted'
            if (self.result.error_code.val == MoveItErrorCodes.SUCCESS):
                self.limb.gripper.open(effort = 0.3)
                rospy.sleep(1.0)
                self.limb.arm.move_joint(self.grasp_joints, interval = 2.5,segments = 20) # Movimiento con collisiones permitidas
                rospy.sleep(3.0)
                self.limb.gripper.close(effort = 0.3)
                rospy.sleep(1.0)
                self.limb.gripper.open(effort = 0.3)
                #      self.limb.gripper.close(effort = 300)
                #      self.limb.gripper.open(effort = 300)
                return 'succeeded'
            else:
                continue

class setJointTrajectoryPosition(smach.State):

    def __init__(self,arms):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                    input_keys=['joint_position','interval','segments','limb_side'])
        self.arms = arms

    def execute(self, userdata):
        self.limb = self.arms[userdata.limb_side]
        self.limb.arm.move_joint(userdata.joint_position,interval=userdata.interval,segments=userdata.segments)
        self.limb.arm.wait()
        return 'succeeded'

class ErrorManager(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['succeeded','aborted'],
                                input_keys=['error_code'])

        self.robot = robot

    def execute(self,userdata):

        print userdata.error_code
        self.robot.say("ERROR")
        
        return 'succeeded'


def getInstance(robot):

    sm = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'])

    sm.userdata.arm_side = 'r'
    sm.userdata.effort = 0.5
    # sm.userdata.arm_joint_goal = [0.1,0.1,0.1,0.1,0.1,0.1]
    # sm.userdata.arm_joint_goal = [0,0,0,0,0,0]

    sm.userdata.trayectory_home_pre2 = ['home','premanip_1','premanip_2']

    arms = {'l':Limb('l'),'r':Limb('r')}

    sm.userdata.name = 'papayamilk'

    p = PoseStamped()
    p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
    p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.58, -0.25, 0.85
    
    sm.userdata.posestamped = p

    with sm:

        smach.StateMachine.add('PREMANIP_1', SetPositionNamed(robot,arms,blind=True,init='home',goal='pre_1'),
            transitions = {'succeeded':'PREMANIP_2','aborted':'aborted'},
            remapping = {'trayectory_name':'trayectory_name_before','limb_side':'arm_side'})

        smach.StateMachine.add('PREMANIP_2', SetPositionNamed(robot,arms,blind=True,init='pre_1',goal='pre_2'),
            transitions = {'succeeded':'GO_TO_OBJECT','aborted':'GET_ERROR'},
            remapping = {'trayectory_name':'trayectory_name_before','limb_side':'arm_side'})

        smach.StateMachine.add('GET_ERROR', ErrorManager(robot),
            transitions = {'succeeded':'aborted'},
            remapping = {'error_code':'error_code'})

        smach.StateMachine.add('GO_TO_OBJECT', PositionObjectAndGrasp_capmap(robot,arms),
            transitions = {'succeeded':'succeeded','aborted':'aborted'},
            remapping = {'posestamped':'posestamped','name':'name','limb_side':'arm_side'})


        # smach.StateMachine.add('OPEN_GRIPPER', OpenGripper(robot),
        #     transitions = {'succeeded':'CLOSE_GRIPPER'},
        #     remapping = {'lr_side':'gripper_side','effort':'effort'}
        #     )

        # smach.StateMachine.add('CLOSE_GRIPPER', GrabGripper(robot),
        #     transitions = {'succeeded':'succeeded'},
        #     remapping = {'lr_side':'gripper_side','effort':'effort'}
        #     )

       
    return sm

if __name__ == "__main__":
    rospy.init_node("sm_arm")

    robot = robot_factory.build(['l_gripper','r_gripper','l_arm','r_arm', 'arm'], core=False)
    sm = getInstance(robot)
    sm.execute()
