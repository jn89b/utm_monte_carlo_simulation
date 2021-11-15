# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 18:19:31 2021

@author: jnguy
"""
import sys
import numpy as np
import math 
from scipy import spatial
from queue import PriorityQueue
import matplotlib.pyplot as plt

import Drone
import UTMDatabase
import PathFinding
"""
things to do 
need landing database
    - this is where I have inputs of UAVS 
need pre landing service 
need path sender service
am I really using 
need post landing service

"""

"""
set up path planner modules
"""

class HomeBase():
    LANDING_ZONES = {"Zone_0": [20, 20],
                     "Zone_1": [30, 20]}
    GRID_Z = 50
    GRID_X = 50
    GRID_Y = 50
    STATIC_OBSTACLE = [(30,10)]
    
    def __init__(self):
    
        self.ZONE_DB = self.generate_landing_zones()
    
    def generate_landing_zones(self):
        zone_db = {}
        zone_list =  [(20, 20,5), (20,30,5), (30, 20, 5), (30, 30, 5)]
        for idx, zone in enumerate(zone_list):
            zone_name = "Zone_"+ str(idx)
            zone_db[zone_name] = LandingZone(zone_name, zone, True)
        
        return zone_db
    
    def check_open_zones(self):
        for zone_name, zone in self.ZONE_DB.items():
            if zone.vacant == True:
                return True
            else:
                continue
        return False
                
    def get_open_zones(self):
        """return list of open zones if possible"""
        open_zone_list = []
        for zone_name, zone in self.ZONE_DB.items():
            if zone.vacant == True:
                open_zone_list.append(zone)
                
        return open_zone_list
    
    def get_open_zone_locations(self, open_zone_list):
        open_zone_locs = []
        for open_zone in open_zone_list:
            open_zone_locs.append(open_zone.zone_coordinates)
            
        return open_zone_locs


    def set_landing_zone_vacancy(self, zone_key, yes_or_no):
        """set vacancy with true or false"""
        self.ZONE_DB[zone_key].set_vacancy(yes_or_no)
    
class LandingZone():
    def __init__(self, zone_name, zone_coordinates, yes_or_no):
        self.zone_name = zone_name #string
        self.zone_coordinates = zone_coordinates #[x,y,z]
        self.vacant = yes_or_no

    def get_zone_name(self):
        """accessor"""
        return self.zone_name
    
    def get_zone_coordinates(self):
        """accessor"""
        return self.zone_coordinates
    
    def set_vacancy(self, yes_or_no):
        """accessor"""
        self.vacant = yes_or_no
    

class PreLandingService():
    def __init__(self, homeBase, landing_db):
        self.homeBase = homeBase
        self.landingDb = landing_db
        self.uav_service_state_num = 0
    
    def get_uavs(self):
        """get uavs that are in a specific state"""
        pass
    
    def generate_grid(self):
        """generates search space based on parameters"""
        grid = []
        grid = np.zeros((self.homeBase.GRID_Z, self.homeBase.GRID_X, self.homeBase.GRID_Y))
        
        return grid

    def generate_3d_obstacles(self, obstacle_list, height):
        """generate 3 dimensional obstacles ie trees and buildings etc"""
        obstacle_3d_list = []
        for static_obstacle in obstacle_list:
            x = static_obstacle[0]
            y = static_obstacle[1]
            for z in range(25):
                obstacle_3d_list.append((x,y,z))
        return obstacle_3d_list
    
    def add_obstacles(self,grid, obstacle_list):
        """"add obstacles to grid location"""
        for obstacle in obstacle_list:
            #print(obstacle)
            (grid[obstacle[2],obstacle[0], obstacle[1]]) = 1
            
        return obstacle_list
    

    def return_unassigned_list(self,some_list, index):
        """return all other zones or uavs not assigned to uav to make as a no fly zone"""
        copy = some_list
        copy.pop(index)
        print("copy", copy)
        return copy

    def get_dynamic_obstacles(self, idx, uav_path_obs, obstacle_list, zone_locations, \
                              zone_idx, path_list, uav_loc_list, grid):
        """generate dynamic obstacles from uav waypoints"""
        #should be a function to make dynamic obstacles
        if idx == 0:
            new_obstacle = obstacle_list + \
                self.return_unassigned_list(zone_locations[:], zone_idx)
        else:
            uav_path_obs.append(path_list[idx-1])
            flat_list = [item for sublist in uav_path_obs for item in sublist]
            new_obstacle = obstacle_list + \
                self.return_unassigned_list(zone_locations[:], zone_idx) + \
                self.return_unassigned_list(uav_loc_list[:], idx) + flat_list
        grid_copy = grid.copy()
        new_obstacle = self.add_obstacles(grid_copy, new_obstacle)

        return grid_copy, new_obstacle
    
    def find_closest_zone(self, uav_loc, landing_zones):
        """find closest zone location to uav location"""
        tree = spatial.KDTree(landing_zones)
        dist,zone_index = tree.query(uav_loc)
        
        return dist, zone_index    
    
    def check_uav_service_state(self, uav_service_num):
        """checks if uav wants the service of prelanding service"""
        if uav_service_num == self.uav_service_state_num:

            return True

    def assign_uav_zone(self,uav, landing_zone):
        """get open zone locations"""
        uav.set_zone_index(landing_zone.get_zone_name())
        uav.set_goal(landing_zone.get_zone_coordinates())

    def compute_vectors(self,point_1, point_2, point_3):
        vector_start = np.array(point_2)- np.array(point_1)
        vector_end = np.array(point_3) - np.array(point_2)
        
        return vector_start, vector_end
        
    def compute_cross_product(self,vector_1, vector_2):
        return np.cross(vector_1, vector_2)

    def reduce_waypoints(self,waypoint_list):
        print(waypoint_list)
        filtered_waypoints = []
        for i, waypoint in enumerate(waypoint_list):
            if i+2 - len(waypoint_list) == 0:
                filtered_waypoints.append(waypoint_list[i+1])
                """might want to append last waypoint value to new list"""
                return filtered_waypoints
            
            vec_start, vec_end = self.compute_vectors(waypoint, waypoint_list[i+1], waypoint_list[i+2])
            cross_product = self.compute_cross_product(vec_start, vec_end)
            if (cross_product[0] == 0 and cross_product[1] == 0
            and cross_product[2] == 0):
                continue
            else:
                #print("not collinear")
                filtered_waypoints.append(waypoint)
                filtered_waypoints.append(waypoint_list[i+2])
                
        return filtered_waypoints

        
    def find_waypoints(self, grid_copy, new_obstacle, uav):
        uav_loc = uav.current_position
        goal_point = uav.goal
        astar = PathFinding.Astar(grid_copy, new_obstacle,  uav_loc, goal_point)
        uav_wp = astar.main()
        uav_filtered_wp = self.reduce_waypoints(uav_wp)
        return uav_wp, uav_filtered_wp
    
    def return_uav_loc_list(self):
        """return list of uavs"""
        uav_loc_list = []
        for uav_id, uav in self.landingDb.items():
            uav_loc_list.append(uav.current_position)
            
        return uav_loc_list
            
    def main(self):
        """main implementation """
        grid = self.generate_grid()
        obstacle_list = self.generate_3d_obstacles(HomeBase.STATIC_OBSTACLE,25)
        obstacle_list = self.add_obstacles(grid, obstacle_list)
        uav_loc_list = self.return_uav_loc_list()
        
        idx = 0
        uav_path_obs = []
        path_list = []
        for uav_id, uav in self.landingDb.items():
            if self.homeBase.check_open_zones() == True:
                print("path list is", path_list)
                """assign uav to landing zone"""
                open_zones = self.homeBase.get_open_zones()
                open_zones_locs = self.homeBase.get_open_zone_locations(open_zones)
                dist, zone_index = self.find_closest_zone(uav.current_position, open_zones_locs)
                self.assign_uav_zone(uav, open_zones[zone_index])
                
                """generate dynamic obstacles"""
                grid_copy, new_obstacle = self.get_dynamic_obstacles(idx, uav_path_obs, obstacle_list, open_zones_locs, \
                              zone_index, path_list, uav_loc_list, grid)
                
                """get waypoints to arrive to landing zone"""
                uav_wp, uav_filtered_wp = self.find_waypoints(grid_copy, new_obstacle, uav)
                uav.set_path(uav_filtered_wp)
                self.homeBase.set_landing_zone_vacancy(open_zones[zone_index].zone_name, False)
                
                """set new obstacles"""
                path_list.append(uav_wp)
                
                idx += 1
                print("index", idx)

            else:
                print("No open Zones")
                
                
if __name__ == '__main__':
    

    #def __init__(id_name ,home_position, curr_position, want_service):
    """initialize randomized values"""
    drone2 = Drone.QuadCopter("uav0", [0,0,0], [0,1,10], True)
    drone1 = Drone.QuadCopter("uav1", [0,0,0], [6,0,10], True)
    uavs = [drone2, drone1]
    
    """databases"""
    overallDb = UTMDatabase.OverallDatabase(uavs)
    overall_db = overallDb.listen_for_incoming_uavs()    
    landingDb = UTMDatabase.LandingDatabase(overall_db)
    landingDb.does_uav_want_service()
    
    """landing services"""
    homeBase = HomeBase()
    preLandingService = PreLandingService(homeBase, landingDb.get_landing_db())
    preLandingService.main()
    # grid = preLandingService.generate_grid()
    # obstacle_list = preLandingService.generate_3d_obstacles(HomeBase.STATIC_OBSTACLE,25)
    # obstacle_list = preLandingService.add_obstacles(grid, obstacle_list)
    
    
    
    