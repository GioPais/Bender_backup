#!/usr/bin/env python
'''

'''

__author__ = "Gonzalo Olave"

import rospy

import smach
import smach_ros
from smach import Sequence
from uchile_states.manipulation import basic


def GoPremanipulation(robot,arms):

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        input_keys = ['side','trayectory_name'],
        connector_outcome = 'succeeded')

    with sq:
        Sequence.add('HOME_PRE1', basic.SetPositionNamed(robot,arms,blind=True,init='home',goal='pre_1'))
        Sequence.add('PRE1_PRE2', basic.SetPositionNamed(robot,arms,blind=True,init='pre_1',goal='pre_2'))

    return sq

def GoHomeSafe(robot,arms):

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        input_keys = ['side','joint_goal'],
        connector_outcome = 'succeeded')

    sq.userdata.home_safe=[0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]

    with sq:
        Sequence.add('PRE2_PRE1', basic.SetPositionNamed(robot,arms,blind=True,init='pre_2',goal='pre_1'))
        Sequence.add('SLEEP', basic.Sleep(robot,1))
        Sequence.add('PRE1_HOME', basic.JointGoal(robot),
            remapping = {'joint_goal':'joint_goal','side':'side'})
        Sequence.add('HOME', basic.JointGoal(robot),
            remapping = {'joint_goal':'home_safe','side':'side'})
    return sq


def GoHome(robot,arms):

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        input_keys = ['side','trayectory_name'],
        connector_outcome = 'succeeded')

    with sq:
        Sequence.add('PRE2_PRE1', basic.SetPositionNamed(robot,arms,blind=True,init='pre_2',goal='pre_1'))
        Sequence.add('PRE1_HOME', basic.JO(robot,arms,blind=False,init='pre_1',goal='home'))

    return sq

def GoMoveit(robot,arms):

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        input_keys = ['side','trayectory_name'],
        connector_outcome = 'succeeded')

    with sq:
        Sequence.add('TRAYECTORY', basic.SetPositionNamed(robot,arms,blind=False))
    return sq

