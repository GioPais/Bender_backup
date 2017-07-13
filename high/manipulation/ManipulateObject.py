#!/usr/bin/env python

__author__ = "Gonzalo Olave"

import rospy

import smach
import smach_ros
import math

# robot
from bender_skills import robot_factory
from uchile_skills import world_definition


from uchile_states.manipulation import basic, octomap, Manipulate
from uchile_states.perception import get_object


# msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

# temporal
from bender_arm_control.arm_commander import Limb


class Iteration(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    io_keys=["it"])
        self.it_max = 3

    def execute(self,userdata):
        
        userdata.it +=1
        if (userdata.it < self.it_max):
            userdata.it = -1
            return 'succeeded'
        return 'aborted'

class RestartIt(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    io_keys=["it"])
    def execute(self,userdata):
        
        userdata.it = -1
        return 'succeeded'


class LookBestPosition(smach.State):
    def __init__(self,robot,arm_dict):
        smach.State.__init__(self, outcomes = ['succeeded','aborted'],
                                input_keys=['object','side'])
        self.arms = arm_dict
        self.cmd_vel_pub = rospy.Publisher('/bender/nav/base/cmd_vel', Twist)

        self.max_linear_vel = 0.25
        self.min_linear_vel = -0.25

    def execute(self,userdata):
        selected_arm = self.arms[userdata.side]
        dim = basic.get_object_definition(userdata.object.name)
        cylinder = basic.get_collision_cylinder(userdata.object.posestamped, userdata.object.name, dim)

        best_distance = selected_arm.arm.get_distance_capmap(cylinder, range_width = 0.5)
        print best_distance
        cmd = Twist()
        # cmd.linear.x  = best_distance
        # cmd.angular.z = =
        
        # saturation
        cmd.linear.x = 0.1*math.copysign(1.0,best_distance)
        # cmd.angular.z = min(cmd.angular.z,self.max_angular_vel)
        # cmd.angular.z = max(cmd.angular.z,self.min_angular_vel)
        # time = rospy.Time().now() + rospy.Time(min(best_distance/cmd.linear.x,1.0))
        rate = rospy.Rate(10)
        time = rospy.Time.now() 
        move_time = rospy.Duration(min(best_distance/cmd.linear.x,3.0))
        rospy.loginfo("Movement time: {0}".format(move_time.to_sec()))
        #while (rospy.Time.now() - time  < move_time ):
        #    #self.cmd_vel_pub.publish(cmd)
        #    rate.sleep()

        cmd = Twist()
        #self.cmd_vel_pub.publish(cmd)

        return 'succeeded'


def getInstance(robot):

    sm = smach.StateMachine(outcomes = ['succeeded','failed','aborted','preempted'],
                                    input_keys=['name'])

    sm.userdata.object = world_definition.Object('coffee','','')
    sm.userdata.side = 'l'
    sm.userdata.effort = 0.5

    sm.userdata.trayectory_name_pre = ['home','premanip_1','premanip_2']
    sm.userdata.trayectory_name_pos = ['premanip_2','premanip_1','home']
    sm.userdata.trayectory_name_posfail = ['premanip_2']

    arms = {'l':Limb('l'),'r':Limb('r')}

    sm.userdata.name = 'coca_gazebo'

    p = PoseStamped()
    p.header.frame_id = 'bender/sensors/rgbd_head_rgb_optical_frame'
    p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.90, -0.2, -0.55
    
    sm.userdata.object.posestamped = p
    sm.userdata.object_pose = p
    sm.userdata.home_safe=[0.0 , 0.15 , 0.0 , 0.0 , 0.0 , 0.0]

    sm.userdata.it = -1
    
    with sm:
        # smach.StateMachine.add('OCTOMAP', octomap.getInstance(robot),
        #     transitions = {'succeeded':'GET_OBJECT'})

        smach.StateMachine.add('GET_OBJECT', get_object.getInstance(robot),
            transitions = {'succeeded':'ARM_DESIGNATOR','aborted':'aborted','failed':'GET_OBJECT'},
            remapping = {'object_name':'name','object':'object'})
       
        smach.StateMachine.add('ARM_DESIGNATOR', basic.ArmDesignator(robot),
            transitions = {'succeeded':'GET_GRASPS','aborted':'aborted'},
            remapping = {'object':'object','side':'side'})

        smach.StateMachine.add('GET_GRASPS', basic.GetPossibleGrasp_capmap(arms),
            transitions = {'succeeded':'MANIPULATION','aborted':'LOOK_POSITION'},
            remapping = {'object':'object','side':'side','possible_grasp':'possible_grasp'})

        smach.StateMachine.add('LOOK_POSITION', LookBestPosition(robot, arms),
            transitions = {'succeeded':'OCTOMAP','aborted':'aborted'})

        smach.StateMachine.add('MANIPULATION', Manipulate.getInstance(robot),
            transitions = {'succeeded':'RESTART_IT','aborted':'aborted'},
            remapping = {'object':'object',
                        'side':'side',
                        'possible_grasp':'possible_grasp'})
        smach.StateMachine.add('RESTART_IT', RestartIt(),
            transitions = {'succeeded':'succeeded'})

    return sm


if __name__ == "__main__":
    rospy.init_node("sm_arm")
    robot = robot_factory.build(['l_arm','l_gripper','r_arm','r_gripper', "neck", "object_recognition", "octomap"], core=False)
    sm = getInstance(robot)
    sm.execute()
