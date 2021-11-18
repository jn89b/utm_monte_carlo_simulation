# -*- coding: utf-8 -*-
"""
Created on Mon Nov 15 18:09:00 2021

@author: jnguy
TO DO:
    Track Inbound before you hit the control area

"""

import Drone
import random
import numpy as np
import math


# MAX_X = 100
# MAX_Y = 100
# MIN_X = -50
# MIN_Y = -50


MAX_X = 100
MAX_Y = 100
MIN_X = -50
MIN_Y = -50

INNER_MIN_X = 0
INNER_MIN_Y = 0
INNER_MAX_X = 50
INNER_MAX_Y = 50

def compute_euclidean(position, goal):
    """compute euclidiean with position and goal as 3 vector component"""
    distance =  math.sqrt(((position[0] - goal[0]) ** 2) + 
                    ((position[1] - goal[1]) ** 2))
    
    return distance


def check_spawn_okay(current_location, spawn_list):
    for spawn_loc in spawn_list:
        distance = compute_euclidean(current_location, spawn_loc)
        if distance < 4.0:
            return False
    
    return True


def randomize_drone_outer_locations(n_drones):
    """randommize drone locations and make sure its spaced
    from home position and where it wants to head towards to"""    
    """how to randomize values in this area"""
    
    spawned_locations = []
    loiter_locations = []
    case = ["left", "right", "down", "up"] 
    
    case_edge = {
                "left": MIN_X,
                "right": MAX_X,
                "down": MIN_Y,
                "up": MAX_Y
                }

    inner_case_edge = {
                "left": INNER_MIN_X,
                "right": INNER_MAX_X,
                "down": INNER_MIN_Y,
                "up": INNER_MAX_Y
                }
    
    
    for i in range(n_drones):
        decision = random.choice(case) 
        if decision == "left" or decision == "right":    
            coords = [case_edge[decision], np.random.choice(range(MIN_Y, MAX_Y)), np.random.choice(range(8, 15))]
            loiter = [inner_case_edge[decision], np.random.choice(range(INNER_MIN_Y, INNER_MAX_Y)), np.random.choice(range(8, 15))]
            if check_spawn_okay(coords, spawned_locations) == False:
                while check_spawn_okay(coords, spawned_locations) == False:
                    print("drones too close")
                    coords = [case_edge[decision], np.random.choice(range(MIN_Y, MAX_Y)), np.random.choice(range(8, 15))]
            
            if check_spawn_okay(loiter, loiter_locations) == False:
                while check_spawn_okay(loiter, loiter_locations) == False:
                    loiter = [inner_case_edge[decision], np.random.choice(range(INNER_MIN_Y, INNER_MAX_Y)), np.random.choice(range(8, 15))]
            
            print(coords,loiter)
            loiter_locations.append(loiter)
            spawned_locations.append(coords)
            
        else:
            coords = [np.random.choice(range(MIN_X, MAX_X)), case_edge[decision], np.random.choice(range(8, 15))]
            loiter = [np.random.choice(range(INNER_MIN_X, INNER_MAX_X)), inner_case_edge[decision], np.random.choice(range(8, 15))]
            if check_spawn_okay(coords, spawned_locations) == False:
                while check_spawn_okay(coords, spawned_locations) == False:
                    print("drones too close")
                    coords = [np.random.choice(range(MIN_X, MAX_X)),case_edge[decision], np.random.choice(range(8, 15))]
            
            if check_spawn_okay(loiter, loiter_locations) == False:
                while check_spawn_okay(loiter, loiter_locations) == False:
                    loiter = [ np.random.choice(range(INNER_MIN_X, INNER_MAX_X)), inner_case_edge[decision],np.random.choice(range(8, 15))]
            
            print(coords,loiter)
            loiter_locations.append(loiter)
            spawned_locations.append(coords)            

    return spawned_locations, loiter_locations

def spawn_uavs(home_position_list, loiter_locaiton_list):
    """spawn drones"""
    uavs_list = []
    for idx, home_position in enumerate(home_position_list):
        uav_id = "uav"+str(idx)
        uav = Drone.QuadCopter(uav_id, home_position, loiter_locaiton_list[idx], loiter_locaiton_list[idx], True)
        uavs_list.append(uav)

    return uavs_list

def compute_euclidean(position, goal):
    """compute euclidiean with position and goal as 2 vector component"""
    distance =  math.sqrt(((position[0] - goal[0]) ** 2) + 
                    ((position[1] - goal[1]) ** 2))
    
    return distance

def compute_path_length(point_list):
    """compute the total waypoint path"""
    apts = np.array(pts)
    apts = apts[:,:2]
    lengths = np.sqrt(np.sum(np.diff(apts, axis=0)**2, axis=1)) # Length between corners
    path_length = np.sum(lengths)
    
    return path_length, apts
    
if __name__ == '__main__':
    n_drones = 10
    pts = [(0,0,4), (0,1,2), (3,1,3), (3,0,4)] #
    # apts = np.array(pts)
    # lengths = np.sqrt(np.sum(np.diff(apts, axis=0)**2, axis=1)) # Length between corners
    # total_length = np.sum(lengths)
    path_length, array = compute_path_length(pts)
    best_distance = compute_euclidean(pts[0], pts[-1])
    
    if (best_distance/path_length) < 0.5:
        print("not optimal")
    else:
        print("optimal")
    
    # random_home_locations, random_loiters = randomize_drone_outer_locations(n_drones)
    # random_uavs = spawn_uavs(random_home_locations, random_loiters)
    
