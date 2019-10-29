from openravepy import *
from pylab import *

import string
import numpy as np
import random

INF = np.infty
EPS = 1e-12
CLA_NOTHING = 0
FORWARD = 0
BACKWARD = 1
############################## POLYNOMIALS ##############################
"""
NB: we adopt the weak-term-first convention for inputs
"""

#_#_#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#_#_#all coefficient_list and trajectory_list are ascending
#_#_#but numpy.poly need descending coefficients, so use [::-1]
#_#_#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


def normalize(vector):
    vector_normal = np.linalg.norm(vector)
    assert(not vector_normal == 0)
    return vector/vector_normal


def interpolate_polynomial_1st(q_0, q_1):
    a1 = q_1 - q_0
    a0 = q_0
    return a0, a1


def coefficient_set_1st(a0, a1):
    A0 = np.array(a0)[:, np.newaxis]
    A1 = np.array(a1)[:, np.newaxis]
    coefficient_array = np.concatenate( (A0, A1), axis=1  )
    return coefficient_array

def check_path_collision(robot_input, path_input, n_grid_input, direction_input=FORWARD):
    """Check_Path_Collision accepts a robot and a path object as its inputs.
       It returns True if any config along the path is IN-COLLISION.
    """
    env = robot_input.GetEnv()
    amount_dof = robot_input.GetDOF()
    grid_point_temp = np.delete(np.linspace(0, 1, n_grid_input, endpoint=False),[0])
    if direction_input == FORWARD:
        grid_point_set = grid_point_temp[::-1]
    if direction_input == BACKWARD:
        grid_point_set = grid_point_temp
    for s_hat in grid_point_set:
        with robot_input:
            robot_input.SetDOFValues( path_input.eval(s_hat), 
                range(amount_dof), CLA_NOTHING)
            is_in_collision = ( 
                env.CheckCollision( robot_input, CollisionReport() ) 
                or robot_input.CheckSelfCollision( CollisionReport() ) )
            if (is_in_collision):
                return True
    return False

def check_configuration_collision(robot_input, q_input):
    env = robot_input.GetEnv()
    with robot_input:
        robot_input.SetActiveDOFValues(q_input)
        in_collision = (env.CheckCollision(robot_input, CollisionReport()) 
            or robot_input.CheckSelfCollision(CollisionReport()))
        return in_collision





