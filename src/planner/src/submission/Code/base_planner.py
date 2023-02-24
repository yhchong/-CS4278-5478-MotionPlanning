#!/usr/bin/env python
import math
from math import *
import json
import copy
import argparse

import rospy
import numpy as np
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from const import *

from planner.msg import ContinuousAction, DiscreteAction, ContinuousState, DiscreteState
from planner.srv import (
    DiscreteActionSequenceExec,
    ContinuousActionSequenceExec,
    DiscreteActionStochasticExec,
)

####
import heapq

ROBOT_SIZE = 0.2552  # width and height of robot in terms of stage unit


def dump_action_table(action_table, filename):
    """dump the MDP policy into a json file

    Arguments:
        action_table {dict} -- your mdp action table. It should be of form {(1,2,0): (1, 0), ...}
        filename {str} -- output filename
    """
    tab = dict()
    for k, v in action_table.items():
        key = [str(int(i)) for i in k]
        key = ",".join(key)
        tab[key] = v

    with open(filename, 'w') as fout:
        json.dump(tab, fout)


class Planner:
    def __init__(self, world_width, world_height, world_resolution, inflation_ratio=3):
        """init function of the base planner. You should develop your own planner
        using this class as a base.

        For standard mazes, width = 200, height = 200, resolution = 0.05. 
        For COM1 map, width = 2500, height = 983, resolution = 0.02

        Arguments:
            world_width {int} -- width of map in terms of pixels
            world_height {int} -- height of map in terms of pixels
            world_resolution {float} -- resolution of map

        Keyword Arguments:
            inflation_ratio {int} -- [description] (default: {3})
        """
        self.map = None
        self.pose = None
        self.goal = None
        self.action_seq = None  # output
        self.aug_map = None  # occupancy grid with inflation
        self.action_table = {}

        self.world_width = world_width
        self.world_height = world_height
        self.resolution = world_resolution
        self.inflation_ratio = inflation_ratio
        self.setup_map() 
        rospy.sleep(1)

    def setup_map(self):
        """Get the occupancy grid and inflate the obstacle by some pixels.

        You should implement the obstacle inflation yourself to handle uncertainty.
        """
        # Hint: search the ROS message defintion of OccupancyGrid
        occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid) 
        self.map = occupancy_grid.data

        # TODO: FILL ME! implement obstacle inflation function and define self.aug_map = new_mask        
        map_a = np.array(self.map).reshape(self.world_height, self.world_width)
        map_a[0, :] = 100
        map_a[-1, :] = 100
        map_a[:, 0] = 100
        map_a[:, -1] = 100
        mv = [0, 1, 2]
        for k in range(self.inflation_ratio):
            map_a = np.pad(map_a, 1, mode="constant")
            maps = [map_a[i:self.world_height+i, j:self.world_width+j] for i in mv for j in mv]
            maps_padded = np.stack(maps).sum(axis=0)
            map_a = (maps_padded > 0) * 100
        self.aug_map = map_a
        

    def _get_goal_position(self):
        goal_position = self.goal.pose.position
        return (goal_position.x, goal_position.y)

    def set_goal(self, x, y, theta=0):
        """set the goal of the planner

        Arguments:
            x {int} -- x of the goal
            y {int} -- y of the goal

        Keyword Arguments:
            theta {int} -- orientation of the goal; we don't consider it in our planner (default: {0})
        """
        a = PoseStamped()
        a.pose.position.x = x
        a.pose.position.y = y
        a.pose.orientation.z = theta
        self.goal = a
        
    def arr_checker(self, pose):
        if type(pose) != np.ndarray:
            return np.array(pose)
        else:
            return pose
        
    def l1_dist(self, pose, goal):
        """Compute L1 distance from pose to goal.

        Arguments:
            pose {list} -- robot pose
            goal {list} -- goal pose

        Returns:
            float -- L1 distance to the target_pose
        """
        l1_dist = np.abs(np.subtract(pose[:2], goal[:2])).sum()
        return l1_dist
   
    def sample_pts(self, cp_x_y, ns_x_y, density=2):
        cp_x_y = self.arr_checker(cp_x_y)
        ns_x_y = self.arr_checker(ns_x_y)
        dist = self.l1_dist(cp_x_y, ns_x_y)
        sample_size = int(np.ceil(1.0*dist/density))+1
        seg_sample = np.random.uniform(size=sample_size)
        seg_sample[-1] = 0
        pts_sample = seg_sample * cp_x_y[::-1].reshape(-1, 1) + (1-seg_sample) * ns_x_y[::-1].reshape(-1, 1)
        pts_sample = np.round(pts_sample).astype(np.int)
        pts_sample = set(tuple(u) for u in pts_sample.T)
        return tuple(zip(*pts_sample))   
    
    def generate_plan(self, init_pose):
        """TODO: FILL ME! This function generates the plan for the robot, given
        an initial pose and a goal pose.

        You should store the list of actions into self.action_seq, or the policy
        into self.action_table.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """
        self.action_seq = []

    def collision_checker(self, x, y):
        """TODO: FILL ME!
        You should implement the collision checker.
        Hint: you should consider the augmented map and the world size
        
        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
        
        Returns:
            bool -- True for collision, False for non-collision
        """
        xy = np.round(np.array([x, y]) / self.resolution).astype(int)
        return ((xy < 0).any() or xy[0] >= self.world_width or xy[1] >= self.world_height) or self.aug_map[xy[1], xy[0]] > 0

    def motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """Predict the next pose of the robot given controls. Returns None if
        the robot collide with the wall.

        The robot dynamics is provided in the assignment description.

        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
            theta {float} -- current theta of robot
            v {float} -- linear velocity 
            w {float} -- angular velocity

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w != 0:
                dx = - v / w * np.sin(theta) + v / w * \
                    np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * \
                    np.cos(theta + w / frequency)
            else:
                dx = v*np.cos(theta)/frequency
                dy = v*np.sin(theta)/frequency
            x += dx
            y += dy

            if self.collision_checker(x, y):
                return None
            theta += w / frequency
        return x, y, theta

    def discrete_motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """Discrete version of the motion predict.

        Note that since the ROS simulation interval is set to be 0.5 sec and the
        robot has a limited angular speed, to achieve 90 degree turns, we have
        to execute two discrete actions consecutively. This function wraps the
        discrete motion predict.

        Please use it for your discrete planner.

        Arguments:
            x {int} -- current x of robot
            y {int} -- current y of robot
            theta {int} -- current theta of robot
            v {int} -- linear velocity
            w {int} -- angular velocity (0, 1, 2, 3)

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        w_radian = w * np.pi/2
        first_step = self.motion_predict(x, y, theta*np.pi/2, v, w_radian)
        if first_step:
            second_step = self.motion_predict(
                first_step[0], first_step[1], first_step[2], v, w_radian)
            if second_step:
                return (round(second_step[0]), round(second_step[1]), round(second_step[2] / (np.pi / 2)) % 4)
        return None

class DNode(object):
    def __init__(self, ex_id, est_d, tie_d):
        self.id = ex_id
        self.est_d = est_d
        self.tie_d = tie_d
        
    def __repr__(self):
        return 'DNode (%s) %s: %s' % (self.id, self.est_d, self.tie_d)

    def __lt__(self, other):
        if self.est_d < other.est_d:
            return True
        elif self.est_d == other.est_d:
            for i in range(len(self.tie_d)):
                if self.tie_d[i] != other.tie_d[i]:
                    return self.tie_d[i] < other.tie_d[i]
        else:
            return False

class A_Star_Planner(Planner):
    
    def __init__(self, *args, **kwargs):
        Planner.__init__(self, *args, **kwargs)
    
    def l2_dist(self, pose, goal):
        """Compute L2 distance from pose to goal.

        Arguments:
            pose {list} -- robot pose
            goal {list} -- goal pose

        Returns:
            float -- L2 distance to the target_pose
        """
        return math.sqrt(
            (pose[0] - goal[0]) ** 2 + (pose[1] - goal[1]) ** 2
        )
    
    def actions_to_take(self, start, end, cur_theta):
        actions = []
        # turn to face end
        tar_facing = {
            (-1, 0): 2, 
            (1, 0): 0, 
            (0, -1): 3,
            (0, 1): 1,
        }
        delta = tuple((end - start).tolist())
        tar_theta = tar_facing[delta]
        if (tar_theta - 1) % 4 == cur_theta:
            actions.append((0, 1)) #turn left
        else:
            while (cur_theta != tar_theta):
                actions.append((0, -1)) #turn right
                cur_theta = (cur_theta - 1) % 4                
        actions.append((1,0))
        return actions, tar_theta
    
    
    def generate_plan(self, init_pose):
        """TODO: FILL ME! This function generates the plan for the robot, given
        an initial pose and a goal pose.

        You should store the list of actions into self.action_seq, or the policy
        into self.action_table.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree
        """
        # A Star
        multiplier = 1/self.resolution
        init_pose_a = (np.array(init_pose)*multiplier).astype(int)
        cur_pose = init_pose_a
        goal_init = self._get_goal_position()
        goal = (np.array(goal_init) * multiplier).astype(int)
        sur_cells = np.array([(1, 0), (-1, 0), (0, -1), (0, 1)]) #up, down, left, right
        cur_node = (cur_pose[0], cur_pose[1])
        visited_nodes = {cur_node: (cur_node, 0)}
        pq = []
        ex_id = 0

        while (np.abs(np.subtract(cur_node, goal)).sum() > (multiplier/2)):
            cur_d = visited_nodes[cur_node][1]
            # check surround cells
            for delta in sur_cells:
                nn_cell_x_y = cur_pose[:-1] + delta * multiplier
                next_node = int(cur_node[0] + delta[0]* multiplier), int(cur_node[1] + delta[1] * multiplier)
                # Check next_node in bound
                if (np.array(next_node) < 0).any() or next_node[0] >= self.world_width or next_node[1] >= self.world_height:
                    continue
                pt_samples = self.sample_pts(cur_node, next_node, density=1)
                if self.aug_map[pt_samples].sum() == 0: 
                    # Add to priority queue
                    dist_ = DNode(ex_id, cur_d + multiplier + self.l1_dist(next_node, goal), (-ex_id,))
                    e = (dist_, next_node, cur_node, cur_d + multiplier)
                    heapq.heappush(pq, e)      
                    ex_id += 1
            next_node = cur_node
            prev_node = None
            while len(pq) > 0 and (next_node in visited_nodes and next_node != prev_node):
                _, next_node, prev_node, real_d = heapq.heappop(pq)
            if len(pq) == 0 and next_node in visited_nodes:
                print("goal not reached and pq is empty!")
                assert False
            else:
                visited_nodes[next_node] = (prev_node, real_d)
            cur_node = next_node
            
        # traceback
        cur_node = tuple(goal.tolist())
        path = [goal]
        while np.abs(np.subtract(cur_node, init_pose_a[:-1])).sum() > (multiplier/2): 
            prev_node = visited_nodes[cur_node][0]
            path.append(prev_node)
            cur_node = prev_node
        path.reverse()    
        path = (np.stack(path) * self.resolution).astype(int)

        cur_theta = init_pose[-1]
        actions_seq = []
        for i in range(len(path) - 1):
            actions_, cur_theta = self.actions_to_take(path[i], path[i+1], cur_theta)
            actions_seq.extend(actions_)
        self.action_seq = actions_seq
        

class A_Star_Hybrid_Planner(Planner):
    
    def __init__(self, *args, **kwargs):
        Planner.__init__(self, *args, **kwargs)
    
    def l2_dist(self, pose, goal):
        """Compute L2 distance from pose to goal.

        Arguments:
            pose {list} -- robot pose
            goal {list} -- goal pose

        Returns:
            float -- L2 distance to the target_pose
        """
        return math.sqrt(
            (pose[0] - goal[0]) ** 2 + (pose[1] - goal[1]) ** 2
        )
        
    def l1_dist(self, pose, goal):
        """Compute L1 distance from pose to goal.

        Arguments:
            pose {list} -- robot pose
            goal {list} -- goal pose

        Returns:
            float -- L1 distance to the target_pose
        """
        l1_dist = np.abs(np.subtract(pose[:2], goal[:2])).sum()
        return l1_dist
    
    def normalize_theta(self, a):
        while( a < -np.pi): a += 2.0*np.pi
        while( a >  np.pi): a -= 2.0*np.pi
        return a
    
    def arr_checker(self, pose):
        if type(pose) != np.ndarray:
            return np.array(pose)
        else:
            return pose
    
    def motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w != 0:
                dx = - v / w * np.sin(theta) + v / w * \
                    np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * \
                    np.cos(theta + w / frequency)
            else:
                dx = v*np.cos(theta)/frequency
                dy = v*np.sin(theta)/frequency
            x += dx
            y += dy
            theta += w / frequency
        theta = self.normalize_theta(theta)
        return x, y, theta

    def next_state(self, cur_pose, control, dt=0.1, num_times=5):
        cur_pose = self.motion_predict(cur_pose[0], cur_pose[1], cur_pose[2], control[0], control[1])
        cur_pose = self.arr_checker(cur_pose) 
        return cur_pose

    def get_aug_coord(self, pose, multiplier):
        pose = self.arr_checker(pose) 
        rtn = pose * multiplier
        rtn = np.round(rtn).astype(np.int)[:2]
        return rtn       
   
    def cal_steering_angle(self, cur_pose, goal):
        goal = goal[:2]
        _d_xy = goal - cur_pose[:-1]
        steering_angle = np.arctan2(_d_xy[1], _d_xy[0]) - cur_pose[-1]
        if np.abs(steering_angle) > np.pi:
            direction = [1, -1][int(steering_angle>0)]
            steering_angle = direction * (2*np.pi - np.abs(steering_angle))
        return steering_angle
    
    def adaptive_v(self, cp_x_y, obs_scan_threshold=10):
        min_x = np.floor(max(cp_x_y[0]-obs_scan_threshold, 0)).astype(int)
        min_y = np.floor(max(cp_x_y[1]-obs_scan_threshold, 0)).astype(int)
        sur_obs = self.aug_map[min_y: (min_y+2*obs_scan_threshold), min_x:min_x+2*obs_scan_threshold]
        max_dist = self.l2_dist((-1, -1), sur_obs.shape) / 2
        cp_x_y_reframe = cp_x_y - np.array((min_x, min_y))
        obs_coord = np.stack(sur_obs.nonzero()).T
        if obs_coord.shape[0] == 0:
            # no obstacles
            return 1
        else:
            obs_dist = ((obs_coord - cp_x_y_reframe)**2).sum(axis=1)**0.5
            return np.min(obs_dist)/max_dist  
            
    def round_orientation(self, pose, decimals=3):
        return pose[0],pose[1], np.round(pose[-1], decimals=decimals)

    def generate_plan(self, init_pose):
        def arr_to_tuple(arr):
            return tuple(arr.tolist())
        multiplier = 1/self.resolution
        goal_init = self._get_goal_position()
        
        init_pose_a = np.array(init_pose)
        cur_pose = init_pose_a
        goal = np.array(goal_init)

        # bool ndarray to discretize
        grid_size_max = 4 # discretize map to grid_size x grid_size
        visited_arr = np.zeros_like(self.aug_map).astype(np.int) # set w, h for different granunity
        visited_arr_loc_dict = {}
        cur_pose_t = arr_to_tuple(cur_pose)
        visited_poses = {cur_pose_t: (cur_pose, (None, None), 0, DNode(0,0,0))} # prev_node, controls, g(n)
        visited_poses_set = set([cur_pose_t])
        visited_nodes_set = set([cur_pose_t[:-1]])
        pq = []
        ex_id = 1
        dist_threshold = 0.3 # same as is_close_to_goal threshold
        orientation_threshold = 1 # decimal place to round orientation to
        T = 0
        isDirect = False
        num_controls_samples = 1
        
        # A* Hydrid
        while self.l2_dist(cur_pose, goal) >= dist_threshold:
            cur_d = visited_poses[arr_to_tuple(cur_pose)][2] # get g(n) of current node
            
            # Sample controls, always 1 positive and 1 negative direction
            cp_x_y = self.get_aug_coord(cur_pose, multiplier)
            # change fixed control speed near obstacles
            v_ = self.adaptive_v(cp_x_y)*0.85 + 0.15
            grid_size = int(np.round(v_*grid_size_max, 0)) + 2
            fixed_controls = [((1, 0),), ((0, -np.pi/2), (v_, 0)), ((0, np.pi/2), (v_, 0))]
            fixed_control_cost = np.array([0, 0.0005, 0.0005])
            w_sampled = np.random.uniform(0, 0.5*np.pi, 2*num_controls_samples)
            w_sampled = np.array([(i%2)*2-1*v for i,v in enumerate(w_sampled)])
            controls_sampled = [((v_, w), (0.5, 0))  for w in w_sampled] 
            cost_sampled = np.abs(w_sampled)/np.pi*0.001

            all_controls = fixed_controls + controls_sampled
            control_cost = np.concatenate([fixed_control_cost, cost_sampled])
            
            # Take controls
            for cid, cs in enumerate(all_controls):
                next_s = cur_pose
                for c in cs:
                    next_s = self.next_state(next_s, c)            
                ns_x_y = self.get_aug_coord(next_s, multiplier)
                if (self.round_orientation(next_s, orientation_threshold) in visited_poses_set) or \
                    ((ns_x_y < 0).any() or ns_x_y[0] >= self.world_width or ns_x_y[1] >= self.world_height) or \
                        (isDirect and np.array(cs)[:,0].sum() == 0):
                    continue
                min_x = np.ceil(max(ns_x_y[0] - grid_size/2, 0)).astype(int)
                min_y = np.ceil(max(ns_x_y[1] - grid_size/2, 0)).astype(int)
                sur_cells = visited_arr[min_y: min_y+grid_size, min_x:min_x+grid_size]
                # Sample points for colllision check
                if (cp_x_y == ns_x_y).all():
                    pt_samples = ((ns_x_y[1],), (ns_x_y[0],))
                else:
                    pt_samples = self.sample_pts(cp_x_y, ns_x_y, density=1)
                if self.aug_map[pt_samples].sum() == 0: 
                    delta_g = self.l2_dist(cur_pose, next_s)
                    h = self.l2_dist(next_s, goal)
                    dist_ = DNode(ex_id, cur_d + delta_g + h + control_cost[cid], (-cur_d, -ex_id,))
                    if sur_cells.sum() != 0:
                        #compare with previous cells
                        overlapped_nodes = np.unique(sur_cells[sur_cells != 0])
                        skipped_b = False
                        for _ex_id in overlapped_nodes:
                            total_d = visited_arr_loc_dict[_ex_id][0]
                            min_x_other, min_y_other, gridsize_other = visited_arr_loc_dict[_ex_id][1]
                            cur_d_, delta_g_, h_, control_cost_ = visited_arr_loc_dict[_ex_id][2]
                            if total_d < dist_.est_d and (min_x_other != min_x or min_y != min_y_other):
                                skipped_b = True
                                break
                        if skipped_b:           
                            continue
                        for _ex_id in overlapped_nodes:
                            _min_x, _min_y, _grid_size = visited_arr_loc_dict[_ex_id][1]
                            visited_arr[_min_y: _min_y+_grid_size+1, _min_x:_min_x+_grid_size+1] = 0
                            del visited_arr_loc_dict[_ex_id]
                    # Add to priority queue
                    e = (dist_, next_s, cur_pose, cs, cur_d + delta_g, False)
                    heapq.heappush(pq, e)
                    visited_arr[min_y: min_y+grid_size, min_x:min_x+grid_size] = ex_id
                    visited_arr_loc_dict[ex_id] = (cur_d + delta_g + h + control_cost[cid], 
                                                (min_x, min_y, grid_size), 
                                                (cur_d, delta_g, h, control_cost[cid]))
                    ex_id += 1
            next_node = arr_to_tuple(cur_pose)[:-1]
            prev_node = None
            while len(pq) > 0 and (next_node in visited_nodes_set and next_node != prev_node):
                dn, next_pose, prev_pose, cs, real_d, isDirect = heapq.heappop(pq)
                next_pose_t = arr_to_tuple(next_pose)
                next_node = next_pose_t[:-1]
                prev_node = arr_to_tuple(prev_pose[:-1])
            if len(pq) == 0 and next_node in visited_nodes_set:
                print("goal not reached and pq is empty!")
                assert False
            visited_poses[next_pose_t] = (prev_pose, cs, real_d, dn)
            visited_nodes_set.add(next_node)
            next_pose_t = self.round_orientation(next_pose, orientation_threshold) 
            if np.abs(next_pose_t[-1] - np.pi) <= 1**(-orientation_threshold):
                # theta near pi, double stash it to prevent repeat
                visited_poses_set.add(next_pose_t[:-1] + (-next_pose_t[-1],))
            visited_poses_set.add(next_pose_t)
            cur_pose = next_pose
        
        # traceback
        if self.l2_dist(cur_pose, goal) >= dist_threshold:
            print("goal not reached")
            assert False
        else:
            # control_seq = []
            path = []
            while np.abs(cur_pose - init_pose_a).sum() > 1e-5:
                path.append(cur_pose)
                prev_pose, cs, _, dn = visited_poses[arr_to_tuple(cur_pose)]
                cur_pose = prev_pose
        path.append(cur_pose)
        path_found = np.stack(path)
        path_found = path_found[::-1]
        non_repeated_path = [path_found[0]]
        for i in range(path_found.shape[0]-1):
            if np.abs(non_repeated_path[-1][:2] - path_found[i+1, :2]).sum() < 1e-8:
                continue
            else:
                non_repeated_path.append(path_found[i+1])
        path_found = np.stack(non_repeated_path[::-1])
        
        # optimize path
        cur_pose = np.array(goal_init + (0,))
        cur_xy = self.get_aug_coord(cur_pose, multiplier)
        shorten_path = [cur_pose]
        i = 1
        while np.abs(cur_pose - path_found[-1,:]).sum() > 1e-5:
            next_pose = path_found[i,:]
            next_xy = self.get_aug_coord(next_pose, multiplier)
            pt_samples = self.sample_pts(cur_xy, next_xy, density=1)
            is_visible = self.aug_map[pt_samples].sum() == 0
            if not is_visible:
                if np.abs(path_found[i-1,:] - shorten_path[-1]).sum() < 1e-5:
                    i += 1
                cur_pose = path_found[i-1,:]
                cur_xy = self.get_aug_coord(cur_pose, multiplier)
                shorten_path.append(cur_pose)
            else:
                if np.abs(next_pose - path_found[-1,:]).sum() < 1e-5:
                    # last node
                    shorten_path.append(next_pose)
                    break
                i += 1     
        shorten_path = np.stack(shorten_path)[::-1]
        cur_pose = shorten_path[0,:]
        shorten_action = []
        for next_pose in shorten_path[1:]:
            steering_angle = self.cal_steering_angle(cur_pose, next_pose[:-1]) 
            angular_v = steering_angle/0.5
            if np.abs(angular_v) > np.pi:
                # rotate twice
                direction = [-1, 1][int(angular_v>0)]
                shorten_action.append((0, direction*np.pi))
                cur_pose = self.next_state(cur_pose, shorten_action[-1])
                steering_angle_2 = self.cal_steering_angle(cur_pose, next_pose[:-1]) 
                angular_v = steering_angle_2/0.5
            shorten_action.append((0, angular_v))
            cur_pose = self.next_state(cur_pose, shorten_action[-1])
            #calculate linear distance
            dist = self.l2_dist(cur_pose, next_pose)
            full_speed_num = int(dist//0.5)
            last_stretch = dist%0.5*2
            shorten_action.extend(((1, 0),) * full_speed_num)
            shorten_action.append((last_stretch, 0))
            next_pose[-1] = cur_pose[-1] 
            cur_pose = next_pose
        self.action_seq = shorten_action 

class MDP_Hybrid_Planner(Planner):
    
    def __init__(self, *args, **kwargs):
        Planner.__init__(self, *args, **kwargs)
        # self.path = None

    def motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w != 0:
                dx = - v / w * np.sin(theta) + v / w * \
                    np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * \
                    np.cos(theta + w / frequency)
            else:
                dx = v*np.cos(theta)/frequency
                dy = v*np.sin(theta)/frequency
            x += dx
            y += dy

            theta += w / frequency
        return x, y, theta

    def discrete_motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        w_radian = w * np.pi/2
        first_step = self.motion_predict(x, y, theta*np.pi/2, v, w_radian)
        if first_step:
            second_step = self.motion_predict(
                first_step[0], first_step[1], first_step[2], v, w_radian)
            if second_step:
                return (round(second_step[0]), round(second_step[1]), round(second_step[2] / (np.pi / 2)) % 4)
        return None
    
    def next_state(self, cur_pose, control, dt=0.1, num_times=5, oob=True):
        # Each control exert for 0.5 seconds and refresh rate for env is 0.1 (new position is is calculated every 0.1s) 
        v, w = control[0], control[1]
        next_pose = self.discrete_motion_predict(cur_pose[0], cur_pose[1], cur_pose[2], v, w)
        next_pose = self.arr_checker(next_pose)
        if (not oob) and (self.collision_checker(*cur_pose[:2]) or self.collision_checker(*next_pose[:2])):
            return self.arr_checker(cur_pose)
        return next_pose

    def get_aug_coord(self, pose, multiplier):
        pose = self.arr_checker(pose) 
        rtn = pose * multiplier
        rtn = np.round(rtn).astype(np.int)[:-1]
        return rtn

    def xy2ij(self, xy_coord):
        return (xy_coord[1], xy_coord[0]) + xy_coord[2:]

    def generate_plan(self):
        multiplier = int(1/self.resolution)
        goal_init = self._get_goal_position()
        goal = np.array(goal_init).astype(int)
        num_y, num_x = int(self.world_height * self.resolution)+1, int(self.world_width * self.resolution)+1
        num_t = 4
        num_action = 4
        T_max = 10000
        
        # s00, s01, s0-1, s10, s11, s1-1
        t_matrix = np.array([
            [1, 0, 0, 0,   0,    0],
            [0, 1, 0, 0,   0,    0],
            [0, 0, 1, 0,   0,    0],
            [0, 0, 0, 0.9, 0.05, 0.05],
        ])
        state_prev = np.zeros((num_y, num_x, num_t))

        actions_space = [(0, 0), (0, 1), (0, -1), (1, 0)]
        movements = np.array(actions_space + [(1, 1), (1, -1)])
        yv, xv, tv = np.meshgrid(range(num_y), range(num_x), range(num_t), indexing='ij') 
        yv, xv, tv = yv.reshape(-1), xv.reshape(-1), tv.reshape(-1)
        all_states = list(zip(xv, yv, tv))
        all_s_next = np.array([[self.next_state(pose, c, oob=False) for c in movements] for pose in all_states]) #x,y
        all_s_next = all_s_next.reshape(num_y, num_x, num_t, *all_s_next.shape[1:]) # (y,x,t) : (x,y,t)
        all_s_next = all_s_next.astype(np.int)
        s_next_idx = tuple(tuple(states) for states in all_s_next.reshape(-1, 3).T)
        s_next_idx = s_next_idx[1], s_next_idx[0], s_next_idx[2] #(y, x, t)

        invalid_actions = np.zeros((num_x, num_y, num_t))
        rewards = np.zeros((num_x, num_y, num_t, num_action))
        t_matrix_ = np.repeat(t_matrix[np.newaxis, ...], np.prod((num_x, num_y, num_t)), axis=0)
        t_matrix_ = t_matrix_.reshape(num_x, num_y, num_t, 4, 6)

        for pose in all_states:
            cp_x_y = self.get_aug_coord(pose, multiplier)
            r_for_cur = np.zeros(6) - 1
            for cid, c in enumerate(movements):
                next_s = self.next_state(pose, c)
                ns_x_y = self.get_aug_coord(next_s, multiplier)
                pt_samples = self.sample_pts(cp_x_y, ns_x_y, density=1)
                if ((cp_x_y < 0).any() or cp_x_y[0] >= self.world_width or cp_x_y[1] >= self.world_height) or \
                    ((ns_x_y < 0).any() or ns_x_y[0] >= self.world_width or ns_x_y[1] >= self.world_height) or \
                    self.aug_map[pt_samples].sum() > 0: 
                        r_for_cur[cid] = -25#-8 
                        t_matrix_[pose][:, cid] = 0
            invalid_actions[pose] = r_for_cur[3]
            rewards[pose] = np.matmul(t_matrix, r_for_cur) 

        invalid_actions = np.transpose(invalid_actions, (1, 0, 2))  
        invalid_actions = invalid_actions < -1
        rewards = np.transpose(rewards, (1, 0, 2, 3))
        # -1 for all except goal states for no action
        rewards[goal[1], goal[0], :, 0] = 0
        t_matrix_ = np.transpose(t_matrix_, (1, 0, 2, 3, 4))
        norm = t_matrix_.sum(-1)
        norm[norm==0] = 1
        t_matrix_ = t_matrix_/norm[...,np.newaxis]       

        invalid_nodes = np.zeros((num_y, num_x)).astype(np.bool)
        for i in range(num_y):
            for j in range(num_x):
                y, x = i*multiplier, j*multiplier
                if x >= self.world_width or y >= self.world_height or self.aug_map[y, x] == 100:
                    invalid_nodes[i, j] = True

        while(True):
            # 1 iteration of value propagation
            v_prev = state_prev[s_next_idx].reshape(-1, 1)
            v_prev = v_prev.reshape(*(all_s_next.shape[:-1] + (1,)))
            v_prev = np.matmul(t_matrix_, v_prev).squeeze()
            q_values = rewards + v_prev
            q_values[:,:,:,3][invalid_actions] = np.nan
            q_values[:,:,:,0] = np.nan
            q_values[goal[1],goal[0],:,0] = 0
            v_cur = np.nanmax(q_values, axis=-1)
            v_cur[invalid_nodes] = 0
            old_state_prev = state_prev
            state_prev = v_cur
            if np.isclose(old_state_prev, v_cur).all():
                break
        self.action_table = {pose:actions_space[np.nanargmax(q_values[self.xy2ij(pose)])] for pose in all_states}    
        
class RobotClient:
    """A class to interface with the (simulated) robot.

    You can think of this as the "driver" program provided by the robot manufacturer ;-)
    """

    def __init__(self):
        self._cstate = None
        self.sb_cstate = rospy.Subscriber(
            "/lab1/continuous_state", ContinuousState, self._cstate_callback
        )
        self._dstate = None
        self.sb_dstate = rospy.Subscriber(
            "/lab1/discrete_state", DiscreteState, self._dstate_callback
        )

    def _cstate_callback(self, msg):
        """Callback of the subscriber."""
        self._cstate = msg

    def get_current_continuous_state(self):
        """Get the current continuous state.

        Returns:
            tuple -- x, y, \theta, as defined in the instruction document.
        """
        return (self._cstate.x, self._cstate.y, self._cstate.theta)

    def _dstate_callback(self, msg):
        self._dstate = msg

    def get_current_discrete_state(self):
        """Get the current discrete state.

        Returns:
            tuple -- x, y, \theta, as defined in the instruction document.
        """
        return (self._dstate.x, self._dstate.y, self._dstate.theta)

    def _d_from_target(self, target_pose):
        """Compute the distance from current pose to the target_pose.

        Arguments:
            pose {list} -- robot pose

        Returns:
            float -- distance to the target_pose
        """
        pose = self.get_current_continuous_state()
        return math.sqrt(
            (pose[0] - target_pose[0]) ** 2 + (pose[1] - target_pose[1]) ** 2
        )

    def is_close_to_goal(self, goal):
        """Check if close enough to the given goal.

        Arguments:
            pose {list} -- robot post

        Returns:
            bool -- goal or not
        """
        return self._d_from_target(goal) < 0.3

    def publish_discrete_control(self, action_seq, goal):
        """Publish the discrete controls"""
        proxy = rospy.ServiceProxy(
            "/lab1/discrete_action_sequence",
            DiscreteActionSequenceExec,
        )
        plan = [DiscreteAction(action) for action in action_seq]
        proxy(plan)
        assert self.is_close_to_goal(goal), "Didn't reach the goal."

    def publish_continuous_control(self, action_seq, goal):
        """Publish the continuous controls.

        TODO: FILL ME!

        You should implement the ROS service request to execute the motion plan.

        The service name is /lab1/continuous_action_sequence

        The service type is ContinuousActionSequenceExec

        Checkout the definitions in planner/msg/ and planner/srv/
        """
        proxy = rospy.ServiceProxy(
            "/lab1/continuous_action_sequence",
            ContinuousActionSequenceExec,
        )
        plan = [ContinuousAction(*action) for action in action_seq]
        proxy(plan)
        assert self.is_close_to_goal(goal)

    def execute_policy(self, action_table, goal):
        """Execute a given policy in MDP.

        Due to the stochastic dynamics, we cannot execute the motion plan
        without feedback. Hence, every time we execute a discrete action, we
        query the current state by `get_current_discrete_state()`.

        You don't have to worry about the stochastic dynamics; it is implemented
        in the simulator. You only need to send the discrete action.
        """
        # TODO: FILL ME!
        # Instantiate the ROS service client
        # Service name: /lab1/discrete_action_stochastic
        # Service type: DiscreteActionStochasticExec
        # Checkout the definitions in planner/msg/ and planner/srv/
        proxy = rospy.ServiceProxy(
            "/lab1/discrete_action_stochastic",
            DiscreteActionStochasticExec,
        )
        while not self.is_close_to_goal(goal):
            current_state = self.get_current_discrete_state()
            action = action_table[current_state]
            # TODO: FILL ME!
            # Put the action into proper ROS request and send it
            proxy(DiscreteAction(action))
        rospy.sleep(1)
        assert self.is_close_to_goal(goal)


if __name__ == "__main__":
    # TODO: You can generate and save the plan using the code below
    rospy.init_node('planner')
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', type=str, default='1,8',
                        help='goal position')
    parser.add_argument('--com', type=int, default=0,
                        help="if the map is com1 map")
    parser.add_argument('--task', type=str, default='DSDA',
                        help="Task name")
    parser.add_argument('--map', type=str, default='map1',
                        help="map name")
    args = parser.parse_args()

    try:
        goal = [int(pose) for pose in args.goal.split(',')]
    except:
        raise ValueError("Please enter correct goal format")

    if args.com:
        width = 2500
        height = 983
        resolution = 0.02
    else:
        width = 200
        height = 200
        resolution = 0.05
        
    TASK = args.task.upper() 
    if args.com:
        MAP = "com1building"
    else:
        MAP = args.map
    
    robot = RobotClient()
    robot_diag = (2* (ROBOT_SIZE**2))**0.5
    inflation_ratio =  int(np.ceil(robot_diag/resolution/2))
    
    if TASK == "DSDA":
        planner = A_Star_Planner(width, height, resolution, inflation_ratio=inflation_ratio)
        planner.set_goal(goal[0], goal[1])
        if planner.goal is not None:
            planner.generate_plan(robot.get_current_discrete_state())
        robot.publish_discrete_control(planner.action_seq, goal)
    elif TASK == "CSDA":
        inflation_ratio += 1  
        planner = A_Star_Hybrid_Planner(width, height, resolution, inflation_ratio=inflation_ratio)
        planner.set_goal(goal[0], goal[1],)
        if planner.goal is not None:
            planner.generate_plan(robot.get_current_continuous_state())
        robot.publish_continuous_control(planner.action_seq, goal)
    else:
        planner = MDP_Hybrid_Planner(width, height, resolution, inflation_ratio=inflation_ratio)
        planner.set_goal(goal[0], goal[1])
        if planner.goal is not None:
            planner.generate_plan()
            fn = 'DSPA_%s_%s_%s.json' % (MAP, goal[0], goal[1])
            dump_action_table(planner.action_table, fn)
        robot.execute_policy(planner.action_table, goal)
    
    # save your action sequence
    if TASK == "DSDA" or TASK == "CSDA":
        result = np.array(planner.action_seq)
        fn = './%s_%s_%s_%s.txt' % (TASK, MAP, goal[0], goal[1])
        np.savetxt(fn, result, fmt="%.2e")
        
