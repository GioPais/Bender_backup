#!/usr/bin/env python

__author__ = "Giovanni Pais"

import rospy

import smach
import smach_ros
import math

# robot
from bender_skills import robot_factory
from uchile_skills import world_definition
from uchile_states.head.states import LookTable

from uchile_states.manipulation import basic, basic2, octomap, manipulate, ManipulateG
from uchile_states.perception import get_object


# msgs
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist

# temporal
from bender_arm_control.arm_commander import Limb

# cap_map
from shape_msgs.msg import SolidPrimitive
from hb_workspace_analysis.msg import GraspObject

#StoringGroceries
from uchile_states.interaction.states import Speak, Hear


class Iteration(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','continue'],
                                    io_keys=["it"])
        self.it_max = 6

    def execute(self,userdata):
        
        userdata.it +=1
        if (userdata.it < self.it_max):
            userdata.it = -1
            return 'succeeded'
        return 'continue'

class RestartIt(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    io_keys=["it"])
    def execute(self,userdata):
        
        userdata.it = -1
        return 'succeeded'


class LookBestPosition(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes = ['succeeded','aborted'],
                                input_keys=['object','side'])
        #self.arms = arm_dict
        self.cmd_vel_pub = rospy.Publisher('/bender/nav/base/cmd_vel', Twist)

        self.max_linear_vel = 0.25
        self.min_linear_vel = -0.25

    def execute(self,userdata):
        #selected_arm = self.arms[userdata.side]
        #dim = basic.get_object_definition(userdata.object.name)
        #cylinder = basic.get_collision_cylinder(userdata.object.posestamped, userdata.object.name, dim)

        #best_distance = selected_arm.arm.get_distance_capmap(cylinder, range_width = 0.5)
        #print best_distance
        #cmd = Twist()
        # cmd.linear.x  = best_distance
        # cmd.angular.z = =
        
        # saturation
        #cmd.linear.x = 0.1*math.copysign(1.0,best_distance)
        # cmd.angular.z = min(cmd.angular.z,self.max_angular_vel)
        # cmd.angular.z = max(cmd.angular.z,self.min_angular_vel)
        # time = rospy.Time().now() + rospy.Time(min(best_distance/cmd.linear.x,1.0))
        #rate = rospy.Rate(10)
        #time = rospy.Time.now() 
        #move_time = rospy.Duration(min(best_distance/cmd.linear.x,3.0))
        #rospy.loginfo("Movement time: {0}".format(move_time.to_sec()))
        #while (rospy.Time.now() - time  < move_time ):
        #    #self.cmd_vel_pub.publish(cmd)
        #    rate.sleep()

        #cmd = Twist()
        #self.cmd_vel_pub.publish(cmd)

        return 'aborted'


def getInstance(robot):

    sm = smach.StateMachine(outcomes = ['succeeded','failed','aborted','preempted'],
                                    input_keys=['name'])

    sm.userdata.object = world_definition.Object('coffee','','')
    sm.userdata.side = 'l'
    sm.userdata.effort = 0.8



    #sm.userdata.trayectory_name_pre = ['home','premanip_1','premanip_2']
    #sm.userdata.trayectory_name_pos = ['premanip_2','premanip_1','home']
    #sm.userdata.trayectory_name_posfail = ['premanip_2']

    #arms = {'l':robot.get("l_arm"),'r':robot.get('r_arm')}


    sm.userdata.name = 'pringles_gazebo'

    ps = PoseStamped()
    ps.header.frame_id = 'bender/base_link'  #'bender/sensors/rgbd_head_rgb_optical_frame'
    ps.header.stamp = rospy.Duration(10.0)   
    ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = 0.55, 0.25, 0.60 # original (0.90,-0.2,-0.55) (0.57,0.2,-0.55) 
    
    p = Pose()
    p.position.x, p.position.y, p.position.z = 0.63, 0.25, 0.65

    sm.userdata.object.posestamped=ps
    sm.userdata.object_pose = ps
    sm.userdata.selected_pregrasp= [0.0 , 0.15 , 0.0 , 0.0 , 0.0 , 0.0]
    sm.userdata.selected_grasp= [0.0 , 0.15 , 0.0 , 0.0 , 0.0 , 0.0]
    sm.userdata.pos_grasp=[0.279, 0.054, 0.383, 1.776, 0.061, 0.895]
    sm.userdata.home_safe=[0.0 , 0.15 , 0.0 , 0.0 , 0.0 , 0.0]

    sm.userdata.it = -1
    
    sm.userdata.recognized_sentence = ""

    with sm:
        # smach.StateMachine.add('SETUP', Setup(robot),
        #      transitions={'succeeded'    :'LOOKFRONT'}
        # )
        # smach.StateMachine.add('LOOKFRONT',LookFront(robot),
        #     transitions={'succeeded':'FACE'}
        # )
        # smach.StateMachine.add('FACE', FaceLantern(robot),
        #      transitions={'succeeded'    :'INITIAL_SPEECH'}
        # )
        smach.StateMachine.add('INITIAL_SPEECH',Speak(robot,text="I am ready"),
            transitions={
                'succeeded':'HEAR_START'
            }
        )
        smach.StateMachine.add('HEAR_START', Hear(robot,dictionary='Stage1/StoringGroceries/starttest'),
            transitions={
                'succeeded':'CLOSE_GRIPPER',
                'preempted':'HEAR_START'
            },
            remapping={
                'recognized_sentence':'recognized_sentence'
            }
        )
        smach.StateMachine.add('CLOSE_GRIPPER', basic2.CloseBothGrippers(robot),
            transitions = {'succeeded':'OCTOMAP',
           'aborted':'OCTOMAP'},
            remapping = {'effort':'effort'}
        )
        smach.StateMachine.add('OCTOMAP', octomap.getInstance(robot),
            transitions = {'succeeded':'IT_OBJECT'})
       
        smach.StateMachine.add('IT_OBJECT', Iteration(),
            transitions = {'succeeded':'GET_OBJECT','continue':'failed'}) 
        
        smach.StateMachine.add('GET_OBJECT', get_object.getInstance(robot),
            transitions = {'succeeded':'ARM_DESIGNATOR','aborted':'IT_OBJECT','failed':'IT_OBJECT'},
            remapping = {'object_name':'name','object':'object'})

        smach.StateMachine.add('ARM_DESIGNATOR', basic.ArmDesignator(robot),
            transitions = {'succeeded':'GET_GRASPS','aborted':'aborted'},
            remapping = {'object':'object','side':'side'})
 

        smach.StateMachine.add('GET_GRASPS', basic2.GetGrasp(robot),
            transitions = {'succeeded':'MANIPULATION','aborted':'aborted'},
            remapping = {'object_pose':'object_pose','selected_pregrasp':'selected_pregrasp','selected_grasp':'selected_grasp','side':'side'})

        smach.StateMachine.add('LOOK_POSITION', LookBestPosition(robot),
            transitions = {'succeeded':'GET_GRASPS','aborted':'aborted'})

        smach.StateMachine.add('MANIPULATION', manipulate.getInstance(robot),
            transitions = {'succeeded':'RESTART_IT','aborted':'aborted'},
            remapping = {'object':'object',
                        'side':'side',
                        'selected_pregrasp':'selected_pregrasp',
                        'selected_grasp':'selected_grasp',
                        'pos_grasp':'pos_grasp'})
        smach.StateMachine.add('RESTART_IT', RestartIt(),
            transitions = {'succeeded':'succeeded'})

    return sm


if __name__ == "__main__":
    rospy.init_node("sm_arm")
    robot = robot_factory.build(['r_arm','r_gripper', 'l_arm','l_gripper', "neck","tts","audition", "octomap","capability_map","object_recognition","marker"], core=False)
    sm = getInstance(robot)
    sm.execute()
