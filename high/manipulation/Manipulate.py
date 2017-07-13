#!/usr/bin/env python
'''

'''

__author__ = "Gonzalo Olave"

import rospy

import smach
import smach_ros

# skills
from bender_skills import robot_factory
from uchile_states.manipulation import basic, advanced, octomap

# temporal
from bender_arm_control.arm_commander import Limb


class Iteration(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    io_keys=["it"])
        self.it = -1
        self.it_max = 3

    def execute(self,userdata):

        self.it +=1
        if (self.it < self.it_max):
            userdata.it = self.it
            return 'succeeded'
        return 'aborted'


def getInstance(robot):

    sm = smach.StateMachine(outcomes = ['succeeded','failed','aborted','preempted'],
                                    input_keys=['object','side','possible_grasp','selected_pregrasp','selected_grasp'])

    sm.userdata.effort = 0.5

    sm.userdata.trayectory_name_pre = ['home','premanip_1','premanip_2']
    sm.userdata.trayectory_name_pos = ['premanip_2','premanip_1','home']
    sm.userdata.trayectory_name_posfail = ['premanip_2']

    arms = {'l':Limb('l'),'r':Limb('r')}

    sm.userdata.home_safe=[0.0 , 0.15 , 0.0 , 0.0 , 0.0 , 0.0]
    sm.userdata.it =0
    
    with sm:
        smach.StateMachine.add('PRE_MANIPULACION', advanced.GoPremanipulation(robot,arms),
            transitions = {'succeeded':'ITERATION','aborted':'FAILED'},
            remapping = {'trayectory_name':'trayectory_name_pre','side':'side'})

        smach.StateMachine.add('ITERATION', Iteration(),
            transitions = {'succeeded':'GO_TO_OBJECT','aborted':'FAILED'})

        smach.StateMachine.add('GO_TO_OBJECT', basic.PositionObject(robot,arms),
            transitions = {'succeeded':'OPEN_GRIPPER','aborted':'RE_MANIPULACION_SECURE'},
            remapping = {'object':'object','side':'side'})

        smach.StateMachine.add('OPEN_GRIPPER', basic.OpenGripper(robot),
            transitions = {'succeeded':'GRASP_OBJECT'},
            remapping = {'side':'side','effort':'effort'})

        smach.StateMachine.add('GRASP_OBJECT', basic.Grasp_capmap(robot,arms),
            transitions = {'succeeded':'GRAB_GRIPPER','aborted':'RE_MANIPULACION_SECURE'},
            remapping = {'object':'object','side':'side'})

        smach.StateMachine.add('RE_MANIPULACION_SECURE', basic.JointGoal(robot),
            transitions = {'succeeded':'RE_MANIPULACION','aborted':'FAILED'},
            remapping = {'joint_goal':'selected_pregrasp','side':'side'})

        smach.StateMachine.add('RE_MANIPULACION', advanced.GoMoveit(robot,arms),
            transitions = {'succeeded':'ITERATION','aborted':'FAILED'},
            remapping = {'trayectory_name':'trayectory_name_posfail','side':'side'})

        smach.StateMachine.add('GRAB_GRIPPER', basic.GrabGripper(robot),
            transitions = {'succeeded':'GO_POSTGRASP','aborted':'RE_MANIPULACION_SECURE'},
            remapping = {'side':'side','effort':'effort'})

        smach.StateMachine.add('GO_POSTGRASP', basic.JointGoal(robot),
            transitions = {'succeeded':'GO_HOME','aborted':'FAILED'},
            remapping = {'joint_goal':'selected_pregrasp','side':'side'})

        # smach.StateMachine.add('POS_MANIPULACION', basic.SetPositionNamed(robot,arms,blind=False,init='',goal='pre_1'),
        #     transitions = {'succeeded':'succeeded','aborted':'GO_HOME'},
        #     remapping = {'trayectory_name':'trayectory_name_pos','side':'side'})

        smach.StateMachine.add('GO_HOME', advanced.GoHomeSafe(robot,arms),
            transitions = {'succeeded':'succeeded'},
            remapping = {'joint_goal':'home_safe','side':'side'})


        smach.StateMachine.add('FAILED', advanced.GoMoveit(robot,arms),
            transitions = {'succeeded':'failed'},
            remapping = {'trayectory_name':'trayectory_name_pos','error_code':'error_code'})
       



    return sm


if __name__ == "__main__":
    rospy.init_node("sm_arm")

    robot = robot_factory.build(['l_gripper','r_gripper','l_arm','r_arm', "neck", "object_recognition", "octomap"], core=False)

    sm = getInstance(robot)
    sm.execute()
