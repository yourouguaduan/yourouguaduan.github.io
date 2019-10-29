import numpy as np
import time
import string
import random
import pylab
from openravepy import CollisionReport

import utils
import heap
import copy
import toppra
# suppress openrave complain
CLA_NOTHING = 0
X = np.array([1, 0, 0])
Y = np.array([0, 1, 0])
Z = np.array([0, 0, 1])
# global variables
FORWARD = 0
BACKWARD = 1
NOT_INTERSECT = -2
IN_COLLISION = -1
OK = 1
SMALL = 1e-9
is_BACKWARD_enable = True
grid_points = np.linspace(0, 1, 30)

class Config(object):
    def __init__(self, q):
        self.q = q

class Vertex(object):
    """
    Attributes:
        config     -- stores a Config object
        parent     -- the parent for FORWARD vertex, the child for BACKWARD vertex

        level      -- its level from the root of the tree (0 for the root)
        #_#_#???
        drawn      -- True if this vertex has been plotted via Vertex::Plot
    """
    def __init__(self, config, vertex_type = FORWARD):
        self.config = config
        self.type = vertex_type
        self.parent_index = None # this is to be assigned when added to a tree
        self.index = 0 # this is to be assigned when added to a tree
        self.level = 0

        self.coefficient_descend = []

class Tree(object):
    """
    Attributes:
        vertex_list   -- stores all vertices added to the tree
        tree_type     -- FORWARD or BACKWARD
    """
    def __init__(self, tree_type = FORWARD, vertex_root = None):
        self.vertex_list = []
        if vertex_root is not None:
            self.vertex_list.append(vertex_root)
            self.length = 1
        else:
            self.length = 0
        self.type = tree_type
        self.last_vertex_index = 0

    def __len__(self):
        return len(self.vertex_list)

    def __getitem__(self, index):
        return self.vertex_list[index]

    def add_vertex(self, parent_index, vertex_new):
        parent = self.vertex_list[parent_index]
        vertex_new.parent_index = parent_index
        vertex_new.level = parent.level + 1
        vertex_new.index = self.length
        self.vertex_list.append(vertex_new)
        self.length += 1
        # check soundness
        assert(self.length == len(self.vertex_list))

class RRTPlanner(object):
    REACHED = 1
    ADVANCED = 0
    TRAPPED = -1
    def __init__(self, vertex_start, vertex_goal, robot):
        """#_#_#
        Initialize a planner. 
        RRTPlanner always has two trees. For a unidirectional planner, 
        the treeend will not be extended and always has only one vertex, 
        vertex_goal.
        """
        self.tree_forward = Tree(FORWARD, vertex_start)
        self.tree_backward = Tree(BACKWARD, vertex_goal)
        self.robot = robot
        self.connecting_string = ''
        self.running_time = 0.0
        #_#_# -1 means using the whole set of vertices as a nearest_neighbor set
        self.nearest_neighbor = -1
        self.amount_iteration = 0
        self.is_found = False
        self.RANDOM_NUMBER_GENERATOR = random.SystemRandom()        
        # default parameters
        self.STEP_SIZE = 0.3

    def __str__(self):
        s = 'Total running time: {0} s.\n'.format(self.running_time)
        s += 'Total number of iterations: {0}\n'.format(self.amount_iteration)
        return s

    def extend(self, config_rand):
        raise RRTException("Virtual method not implemented.")

    def connect(self):
        raise RRTException("Virtual method not implemented.")

    def is_feasible_configuration(self, config_rand):
        """
        IsFeasibleConfig checks feasibility of the given Config object. 
        Feasibility conditions are to be determined by each RRT planner.
        """
        raise RRTException("Virtual method not implemented.")

    def is_feasible_trajectory(self):
        """
        IsFeasibleTrajectory checks feasibility of the given trajectorystring.
        Feasibility conditions are to be determined by each RRT planner.
        """
        raise RRTException("Virtual method not implemented.")

    def random_joint_values(self, amount_dof):
        # assign lower and upper limits of joint values
        [lower_limits, upper_limits] = self.robot.GetDOFLimits()

        q_rand = np.zeros(amount_dof)
        for dof_range in xrange(amount_dof):
            q_rand[dof_range] = self.RANDOM_NUMBER_GENERATOR.uniform(
                lower_limits[dof_range], upper_limits[dof_range])

        return q_rand


    def distance(self, config_1, config_2, metric_type=1):
        delta_q = config_1.q - config_2.q
        if (metric_type == 1):
            # norm-2 squared
            return np.dot(delta_q, delta_q)
        elif (metric_type == 2):
            # norm-1
            return np.linalg.norm(delta_q, 1)        

        else:
            raise RRTException("Unknown Distance Metric.")

    def nearest_neighbor_index(self, config_rand, tree_type, 
        custom_nearest_neighbor = 0):
        """
        nearest_neighbor_index returns index of self.nearest_neighbor nearest 
        neighbors of config_rand on the tree specified by treetype.
        """
        if (tree_type == FORWARD):
            tree = self.tree_forward
            amount_vertex = len(tree)
        else:
            tree = self.tree_backward
            amount_vertex = len(tree)
        
        distance_list = [self.distance(config_rand, vertex_hat.config, 
            self.metric_type) for vertex_hat in tree.vertex_list]
        distance_heap = heap.Heap(distance_list)
        if (custom_nearest_neighbor == 0):
            nearest_neighbor = self.nearest_neighbor
        else:
            nearest_neighbor = custom_nearest_neighbor
        if (nearest_neighbor == -1):
            nearest_neighbor = amount_vertex
        else:
            nearest_neighbor = min(self.nearest_neighbor, amount_vertex)
        nearest_neighbor_index = [ distance_heap.extract_min()[0] for 
            i_range in range(nearest_neighbor) ]
        return nearest_neighbor_index

    def run(self, allotted_time):
        if (self.is_found):
            print "The planner has already found a path."
            return True
        time_sum = 0.0 # total running time for this run
        prev_iter = self.amount_iteration
        amount_dof = self.robot.GetActiveDOF()
        while (time_sum < allotted_time):
            self.amount_iteration += 1
            print "\033[1;34m iteration:", self.amount_iteration, "\033[0m"
            #_#_#Bold_Blue="\[\033[1;34m\]" 
            #_#_#Color_Off="\[\033[0m\]"
            running_time_begin = time.time()
            q_rand = self.random_joint_values(amount_dof)

            config_rand = Config(q_rand)
            if (self.extend(config_rand) != self.TRAPPED):
                print "\033[1;32mTree start : ", len(self.tree_forward.vertex_list), 
                print "; Tree end : ", len(self.tree_backward.vertex_list), "\033[0m"
                if (self.connect() == self.REACHED):
                    print "\033[1;32mPath found"
                    print "    Total number of iteration: {0}".format(
                        self.amount_iteration)
                    running_time_end = time.time()
                    time_sum += running_time_end - running_time_begin
                    self.running_time += time_sum
                    print "    Total running time: {0} s.\033[0m".format(
                        self.running_time)
                    self.is_found = True
                    return True # running result
                    
            running_time_end = time.time()
            time_sum += running_time_end - running_time_begin
            self.running_time += running_time_end - running_time_begin
        print "\033[1;31mAllotted time ({0} s. is exhausted after {1} \
        amount_iteration\033[0m".format(allotted_time, 
            self.amount_iteration - prev_iter)
        return False


class BiRRTPlanner(RRTPlanner):
    def __init__(self, vertex_start, vertex_goal, robot, all_constraints, 
        nearest_neighbor = -1, metric_type = 1):

        super(BiRRTPlanner, self).__init__(vertex_start, vertex_goal, robot)

        self.all_constraints = all_constraints
        self.nearest_neighbor = nearest_neighbor
        self.metric_type = metric_type
        self._max_repeat = -1
        self.handles_vertex = []
        self.handles_edge = []
        self.handles_plot = []
        self.connected_coefficient_descend = []

    def plot_vertex(self, q_input):
        with self.robot:
            self.robot.SetActiveDOFValues(q_input)
            vertex_transform = self.robot.GetLink('link6').GetTransform()
        vertex_position = vertex_transform[0:3, 3]
        self.handles_vertex.append( self.robot.GetEnv().plot3( 
            points=np.array(vertex_position ), pointsize=5, 
            colors=np.array( (0,0,1) ), drawstyle=1) )

    def plot_edge(self, path_input):
        edge_transform_set = []
        for s_grid in np.linspace(0, 1, 100):
            with self.robot:
                self.robot.SetActiveDOFValues(path_input.eval(s_grid))
                transform = self.robot.GetLink('link6').GetTransform()
                position = transform[0:3, 3]
                edge_transform_set.append(position)
        self.handles_edge.append(self.robot.GetEnv().drawlinestrip(
            points=np.array(edge_transform_set),linewidth=0.5,
            colors=np.array((0,1,0)) ) )

    def plot_parameterization(self, reachable_set_input=None, 
        controllable_set_input=None, feasible_set_input=None, s_d_set=None):
        pylab.ion()
        if reachable_set_input is not None:
            pylab.plot(reachable_set_input[:, 0], 'g--', label="reachable sets")
            pylab.plot(reachable_set_input[:, 1], 'g--')
        if reachable_set_input is not None:
            pylab.plot(controllable_set_input[:, 0], 'k.', 
                label="controllable sets")
            pylab.plot(controllable_set_input[:, 1], 'k.',)
        if feasible_set_input is not None:
            pylab.plot(feasible_set_input[:, 0], 'r:', label="feasible sets")
            pylab.plot(feasible_set_input[:, 0], 'r:')
        if s_d_set is not None:
            pylab.plot(s_d_vec, label="Velocity profile")
        pylab.title("Path-position path-velocity plot")
        pylab.xlabel("Path position")
        pylab.ylabel("Path velocity square")
        pylab.legend()
        pylab.ioff()
        pylab.show()

    def extend(self, config_rand):
        if not is_BACKWARD_enable:
            return self.extend_forward(config_rand)
        #_#_#if there is no is_BACKWARD_enable, 
        #_#_#then will do ExtendFORWARD & ExtendBACKWARD alternatively
        if (np.mod(self.amount_iteration - 1, 2) == FORWARD):
            return self.extend_forward(config_rand)
        else:
            return self.extend_backward(config_rand)

    def extend_forward(self, config_rand):
        amount_dof = self.robot.GetActiveDOF()
        status = self.TRAPPED
        #_#_#the nearest vertex in tree_forward to the vertex_rand
        nearest_neighbor_index = self.nearest_neighbor_index(config_rand, FORWARD)
        for n_n_i_hat in nearest_neighbor_index:
            print "extend_forward from index = {0}".format(n_n_i_hat)
            vertex_near = self.tree_forward.vertex_list[n_n_i_hat]
            q_beg = vertex_near.config.q

            #_#_# check if config_rand is too far from vertex_near
            delta = self.distance(vertex_near.config, config_rand)
            if (delta <= self.STEP_SIZE):
                q_end = config_rand.q
                status = self.REACHED
            else:
                q_end = q_beg + self.STEP_SIZE*(config_rand.q - 
                    q_beg)/np.sqrt(delta)
                #_#_#???
                status = self.ADVANCED
            

            config_new = Config(q_end)
            if not (delta <= self.STEP_SIZE):
                delta = self.distance(vertex_near.config, config_new)
            n_check_grid = int(delta * 20)

            #_#_#print "\textend_forward : Check_Configuration_Collision"
            config_in_collision = utils.check_configuration_collision(
                self.robot, config_new.q)
            if config_in_collision:
                status = self.TRAPPED
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                continue

            a0, a1 = utils.interpolate_polynomial_1st(q_beg, q_end)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            #_#_#print "\textend_forward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, n_check_grid, direction_input=FORWARD)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            vertex_new = Vertex(config_new)

            vertex_new.level = vertex_near.level + 1
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            vertex_new.coefficient_descend = coefficient_descend
            self.tree_forward.add_vertex(n_n_i_hat, vertex_new)
            self.plot_vertex(vertex_new.config.q)
            self.plot_edge(path_instance)
            status = self.REACHED

            del path_instance
            print "extend_forward : Successful extension"
            return status
        return status

    def extend_backward(self, config_rand):
        amount_dof = self.robot.GetActiveDOF()
        status = self.TRAPPED
        #_#_#the nearest vertex in tree_backward to the vertex_rand
        nearest_neighbor_index = self.nearest_neighbor_index(config_rand, BACKWARD)
        for n_n_i_hat in nearest_neighbor_index:
            print "extend_backward from index = {0}".format(n_n_i_hat)
            vertex_near = self.tree_backward.vertex_list[n_n_i_hat]
            q_end = vertex_near.config.q

            #_#_# check if config_rand is too far from vertex_near
            delta = self.distance(vertex_near.config, config_rand)
            if (delta <= self.STEP_SIZE):
                q_beg = config_rand.q
                status = self.REACHED
            else:
                q_beg = q_end + self.STEP_SIZE*(config_rand.q - 
                    q_end)/np.sqrt(delta)
                #_#_#???
                status = self.ADVANCED

            config_new = Config(q_beg)
            if not (delta <= self.STEP_SIZE):
                delta = self.distance(vertex_near.config, config_new)
            n_check_grid = int(delta * 20)

            #_#_#print "\textend_backward : Check_Configuration_Collision"
            config_in_collision = utils.check_configuration_collision(
                self.robot, config_new.q)
            if config_in_collision:
                status = self.TRAPPED
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                continue

            a0, a1 = utils.interpolate_polynomial_1st(q_beg, q_end)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            #_#_#print "\textend_backward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, n_check_grid, direction_input=BACKWARD)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            vertex_new = Vertex(config_new)

            vertex_new.level = vertex_near.level + 1
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            vertex_new.coefficient_descend = coefficient_descend
            self.tree_backward.add_vertex(n_n_i_hat, vertex_new)
            self.plot_vertex(vertex_new.config.q)
            self.plot_edge(path_instance)
            status = self.REACHED
            del path_instance
            print "extend_backward : Successful extension"
            return status
        status = self.TRAPPED
        return status
   
    def connect(self):
        if not is_BACKWARD_enable:
            return self.connect_backward()
        if (np.mod(self.amount_iteration - 1, 2) == FORWARD):
            return self.connect_backward()
        else:
            return self.connect_forward()

    def connect_backward(self):
        vertex_test = self.tree_forward.vertex_list[-1]
        #_#_#the nearest vertex in tree_backward to the last vertex in tree_forward
        #_#_#if is_BACKWARD_enable == False, tree_backward has only one vertex
        nearest_neighbor_index = self.nearest_neighbor_index(vertex_test.config, BACKWARD)
        status = self.TRAPPED
        for n_n_i_hat in nearest_neighbor_index:
            print "connect_backward from tree_forward.index={0} to tree_backward.index={1}".format(vertex_test.index, n_n_i_hat)
            vertex_near = self.tree_backward.vertex_list[n_n_i_hat]
            q_end = vertex_near.config.q
            
            q_beg = vertex_test.config.q

            a0, a1 = utils.interpolate_polynomial_1st(
                q_beg, q_end)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)
            
            #_#_#print "\tconnect_backward : check_dof_limit"

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            #_#_#print "\tconnect_backward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, 100)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            status = self.REACHED
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            self.connected_coefficient_descend = coefficient_descend
            self.tree_forward.last_vertex_index = vertex_test.index
            self.tree_backward.last_vertex_index = vertex_near.index
            self.plot_edge(path_instance)
            del path_instance
            return status
        status = self.TRAPPED
        return status

    def connect_forward(self):
        vertex_test = self.tree_backward.vertex_list[-1]
        #_#_#the nearest vertex in tree_backward to the last vertex in tree_forward
        #_#_#if is_BACKWARD_enable == False, tree_backward has only one vertex
        nearest_neighbor_index = self.nearest_neighbor_index(vertex_test.config, FORWARD)
        status = self.TRAPPED
        for n_n_i_hat in nearest_neighbor_index:
            print "connect_forward from tree_forward.index={0} to tree_backward.index={1}".format(
                n_n_i_hat, vertex_test.index)
            vertex_near = self.tree_forward.vertex_list[n_n_i_hat]
            q_end = vertex_test.config.q

            q_beg = vertex_near.config.q

            a0, a1 = utils.interpolate_polynomial_1st(q_beg, q_end)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            #_#_#print "\tconnect_forward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, 100)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            status = self.REACHED
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            self.connected_coefficient_descend = coefficient_descend
            self.tree_forward.last_vertex_index = vertex_near.index
            self.tree_backward.last_vertex_index = vertex_test.index
            self.plot_edge(path_instance)
            del path_instance
            return status
        status = self.TRAPPED
        return status

    def generate_final_coefficient(self):
        connected_coefficient_descend = self.connected_coefficient_descend
        coefficient_set_temp = np.array(
            connected_coefficient_descend)[np.newaxis, :, :]
        amount_segment = 1
        vertex_index = self.tree_forward.last_vertex_index
        while vertex_index is not 0:
            coefficient_temp_1 = self.tree_forward.vertex_list[
                vertex_index].coefficient_descend
            coefficient_temp_2 = np.array(coefficient_temp_1)[np.newaxis, :, :]
            coefficient_set_temp = np.concatenate(
                (coefficient_temp_2, coefficient_set_temp), axis=0 )
            vertex_index = self.tree_forward.vertex_list[vertex_index].parent_index
            amount_segment += 1

        vertex_index = self.tree_backward.last_vertex_index
        while vertex_index is not 0:
            coefficient_temp_1 = self.tree_backward.vertex_list[
                vertex_index].coefficient_descend
            coefficient_temp_2 = np.array(coefficient_temp_1)[np.newaxis, :, :]
            coefficient_set_temp = np.concatenate(
                (coefficient_set_temp, coefficient_temp_2), axis=0 )
            vertex_index = self.tree_backward.vertex_list[vertex_index].parent_index
            amount_segment += 1

        shape = np.array(coefficient_set_temp.shape)
        coefficient_set = np.zeros((shape[2], shape[0], shape[1]))

        for i in range(shape[1]):
            for j in range(shape[0]):
                for k in range(shape[2]):
                    coefficient_set[k, j, i] = coefficient_set_temp[j, i, k]

        return coefficient_set, amount_segment

class RRTException(Exception):
    """Base class for exceptions for RRT planners"""
    pass