import sys
sys.path.append('../src/')
#_#_#_sys.path.append('../../robots/')
import baldor
from openravepy import *
from pylab import *
ion()

import time
import string
import numpy as np
import random
import copy
import toppra

import rrt
import utils



def main():

    env = Environment()
    env.Load('../xml/btenv.xml')
    collision_checker = RaveCreateCollisionChecker(env, 'ode')
    env.SetCollisionChecker(collision_checker)
    env.SetViewer('qtosg')
    viewer = env.GetViewer()

    robot = env.GetRobots()[0]
    floor = env.GetKinBody('floor')
    box_1 = env.GetKinBody('box1')
    box_2 = env.GetKinBody('box2')
    box_3 = env.GetKinBody('box3')
    box_4 = env.GetKinBody('box4')
    box_5 = env.GetKinBody('box5')

    floor.Enable(False)

    opening_height = 0.44#_#_#
    table_z_offset = 0.59

    box_1.SetTransform( np.array( [
        [  0.0,  1.0,  0.0,  0.00],
        [ -1.0,  0.0,  0.0,  0.38],
        [  0.0,  0.0,  1.0,  0.695],
        [  0.0,  0.0,  0.0,  1.00] ] ) )

    box_2.SetTransform( np.array( [
        [  1.0,  0.0,  0.0,  0.00],
        [  0.0,  1.0,  0.0, -0.285],
        [  0.0,  0.0,  0.0,  opening_height + 0.32 + 0.2 + table_z_offset],
        [  0.0,  0.0,  0.0,  1.00] ] ) )

    box_3.SetTransform( np.array( [
        [  1.0,  0.0,  0.0,  0.00],
        [  0.0,  1.0,  0.0, -0.625],
        [  0.0,  0.0,  1.0,  0.60],
        [  0.0,  0.0,  0.0,  1.00] ] ) )

    #_#_#box_3 z 0.455

    box_4.SetTransform( np.array( [
        [  1.0,  0.0,  0.0,  0.00],
        [  0.0,  1.0,  0.0,  0.101],
        [  0.0,  0.0,  1.0,  0.85 + table_z_offset],
        [  0.0,  0.0,  0.0,  1.00] ] ) )

    #_#_#box_4 z 0.75

    box_5.SetTransform( np.array( [
        [  1.0,  0.0,  0.0,  0.00],
        [  0.0,  1.0,  0.0, -0.876],
        [  0.0,  0.0,  1.0,  0.40 + table_z_offset],
        [  0.0,  0.0,  0.0,  1.00] ] ) )

    #_#_#T : homogeneous transformation matrices
    T_initial = np.array([
        [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0.59], [0, 0, 0, 1] ])
    robot.SetTransform(T_initial)

    RANDOM = random.SystemRandom()

    q_start = np.deg2rad([-90, 70, 10, 0, -80, 0.])
    q_goal = np.deg2rad([90, 70, 10, 0, -80, 0.])

    robot.SetDOFValues(q_start)

    tray = robot.GetLink('tray')
    T_bottle = tray.GetTransform()

    amount_dof = robot.GetDOF()
    joint_limit = robot.GetDOFLimits()[1]
    joint_limit[4] = np.pi/2
    robot.SetDOFLimits(-joint_limit, joint_limit)
    velocity_max = robot.GetDOFVelocityLimits()
    acceleration_max = robot.GetDOFAccelerationLimits()
    #_# reduce actual acceleration bounds for safety
    acceleration_max = acceleration_max*0.5
    robot.SetDOFAccelerationLimits(acceleration_max)

    ## set the manipulator to the flange (end-effector)
    manipulator = robot.SetActiveManipulator('Flange')
    ik_type = IkParameterization.Type.Transform6D
    ik_model = databases.inversekinematics.InverseKinematicsModel(robot, 
        iktype = ik_type)

    #_#_#configure the constraint
    velocity_limit_temp = robot.GetActiveDOFMaxVel()
    velocity_limit = np.vstack((-velocity_limit_temp, velocity_limit_temp)).T
    acceleration_limit_temp = robot.GetActiveDOFMaxAccel()
    acceleration_limit = np.vstack((-acceleration_limit_temp, 
        acceleration_limit_temp)).T
    constraint_velocity = toppra.constraint.JointVelocityConstraint(velocity_limit)
    constraint_acceleration = toppra.constraint.JointAccelerationConstraint(
        acceleration_limit, discretization_scheme = 
        toppra.constraint.DiscretizationType.Interpolation)
    all_constraints = [constraint_velocity, constraint_acceleration]

    ## initialize the planner
    nearest_neighbor = -1
    metric_type = 1

    #_#_#
    SMALL = 1e-6

    config_start = rrt.Config(q_start)
    vertex_start = rrt.Vertex(config_start, rrt.FORWARD)
    config_goal = rrt.Config(q_goal)
    vertex_goal = rrt.Vertex(config_goal, rrt.BACKWARD)

    status_0 = False ## topp status before shortcutting
    status_1 = False ## topp status after shortcutting

    time_sum = 0
    for i in arange(1):
    #_#_#while (not status_0):

        birrt_instance = rrt.BiRRTPlanner(vertex_start, vertex_goal, 
            robot, all_constraints, nearest_neighbor, metric_type)

        allotted_time = 36000
        birrt_instance.run(allotted_time)
        
        coefficient_descend, amount_segment = birrt_instance.generate_final_coefficient()

        path_instance = toppra.PiecewisePolynomialPath(coefficient_descend)
        grid_points = np.linspace(0, amount_segment, 100*amount_segment)
        toppra_instance = toppra.algorithm.TOPPRA(all_constraints, path_instance, 
            grid_points, solver_wrapper='qpoases')
        joint_trajectory, _ = toppra_instance.compute_trajectory(0, 0)

        #_#_#view the trajectory
        manip = robot.GetManipulator('Flange')
        t_duration = joint_trajectory.get_duration()
        P = []
        time_sum += birrt_instance.running_time
    for t in np.arange(0, t_duration, 0.01):
        robot.SetActiveDOFValues(joint_trajectory.eval(t))
        P.append(manip.GetTransform()[0:3, 3])
        time.sleep(0.05)  # 5x slow down 
    robot.SetActiveDOFValues(joint_trajectory.eval(t_duration))
    handle = env.drawlinestrip(points = np.vstack((P)), linewidth = 3)
    import IPython ; IPython.embed()#_#_#
if __name__ == '__main__':
    main()

