# -*- coding: utf-8 -*-
"""
Need to randomize drones and its locations
Need to iterate over different heuristics  
Need to log each iteration
Need to keep track of success rate over iteration

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
import multiprocessing 
from multiprocessing import Process, Manager

import threading

class HomeBase():
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
    
    def get_closed_zones(self):
        """return list of open zones if possible"""
        open_zone_list = []
        for zone_name, zone in self.ZONE_DB.items():
            if zone.vacant == False:
                open_zone_list.append(zone)
                
        return open_zone_list
    
    def get_open_zone_locations(self, open_zone_list):
        open_zone_locs = []
        for open_zone in open_zone_list:
            open_zone_locs.append(open_zone.zone_coordinates)
            
        return open_zone_locs


    def set_landing_zone_vacancy(self, zone_key, yes_or_no):
        """set vacancy with true or false"""
        print("updated vacancy for", zone_key, yes_or_no)
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
    def __init__(self, homeBase, landing_db, min_h, max_h):
        self.homeBase = homeBase
        self.landingDb = landing_db
        self.uav_service_state_num = 0
        self.min_h = min_h
        self.max_h = max_h
        

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
        """reduce waypoints"""
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
        astar = PathFinding.Astar(grid_copy, new_obstacle,  uav_loc, goal_point, \
                                  self.min_h, self.max_h)
        success_or_no, uav_wp = astar.main()
        if success_or_no == False:
            uav_filtered_wp = None
        else:
            uav_filtered_wp = self.reduce_waypoints(uav_wp)
        return uav_wp, uav_filtered_wp, success_or_no
    
    def return_uav_loc_list(self):
        """return list of uavs"""
        uav_loc_list = []
        for uav_id, uav in self.landingDb.items():
            uav_loc_list.append(uav.current_position)
            
        return uav_loc_list
            
    def get_uavs_requesting_wps(self):
        uav_request_wp = []
        for uav_id, uav in self.landingDb.items():
            if uav.service_state == self.uav_service_state_num and uav.path == None:
                uav_request_wp.append(uav)
                
        return uav_request_wp
    
    def main(self):
        """main implementation"""
        grid = self.generate_grid()
        obstacle_list = self.generate_3d_obstacles(HomeBase.STATIC_OBSTACLE,25)
        obstacle_list = self.add_obstacles(grid, obstacle_list)
        uav_loc_list = self.return_uav_loc_list()
        
        idx = 0
        uav_path_obs = []
        path_list = []
        uav_request_wp = self.get_uavs_requesting_wps()
        for uav in uav_request_wp:
            if self.homeBase.check_open_zones() == True:
                #print("path list is", path_list)
                """assign uav to landing zone"""
                open_zones = self.homeBase.get_open_zones()
                open_zones_locs = self.homeBase.get_open_zone_locations(open_zones)
                dist, zone_index = self.find_closest_zone(uav.current_position, open_zones_locs)
                self.assign_uav_zone(uav, open_zones[zone_index])
                
                """generate dynamic obstacles"""
                grid_copy, new_obstacle = self.get_dynamic_obstacles(idx, uav_path_obs, obstacle_list, open_zones_locs, \
                              zone_index, path_list, uav_loc_list, grid)
                
                """get waypoints to arrive to landing zone"""
                uav_wp, uav_filtered_wp, success_or_no = self.find_waypoints(grid_copy, new_obstacle, uav)
                if success_or_no == True:
                    uav.set_path(uav_wp)
                    self.homeBase.set_landing_zone_vacancy(open_zones[zone_index].zone_name, False)
                    
                    """set new obstacles"""
                    path_list.append(uav_wp)
                    idx += 1

                else:
                    continue
            else:
                print("No open Zones")
                
class PathSenderService():
    """send uav waypoints"""
    def __init__(self, landing_db):
        self.landingDb = landing_db
        self.search_service_number = 0
        self.update_service_number = 1 
                        
    def is_arrived_to_zone(self,waypoint_coords, uav_curr_location):
        """check if close to distance"""
        zone_coords = np.array(waypoint_coords)
        uav_coords = np.array(uav_curr_location)
        dist = math.sqrt((zone_coords[0]- uav_coords[0])**2+ \
                         (zone_coords[1]- uav_coords[1])**2) 
        if dist <= 1.0:
            return True
        else:
            return False
        
    def get_uavs_with_wps(self):
        """return list of uavs that have wps"""
        uavs_with_wp_list = []
        for uav_id, uav in self.landingDb.items():
            if uav.service_state == self.search_service_number and uav.path != None:
                uavs_with_wp_list.append(uav)
                
        return uavs_with_wp_list
    
    def send_wp_commands(self, uavs_with_wp_list, uav):  
        """send waypoint commands to uav"""
        waypoint_list = uav.path
        print("controlling uav", uav.id)
        print("waypoint length", len(waypoint_list))
        
        for idx,wp in enumerate(waypoint_list):
            uav.go_to_wp(uav.current_position,wp)
            #print(idx)
            if idx > (len(waypoint_list)-2):
                print("reached final waypoint")
                uav.update_service_state(self.update_service_number)
                uav.go_to_wp(uav.current_position,waypoint_list[-1])
                break

            
    def main(self):
        uavs_with_wp_list = self.get_uavs_with_wps()
        if not uavs_with_wp_list:
            print("no drones looking")
        else:
            threads = []
            for idx, uav in enumerate(uavs_with_wp_list[:]):
                t = multiprocessing.Process(self.send_wp_commands(uavs_with_wp_list, uav))
                t.start()
                threads.append(t)
                
                if not uavs_with_wp_list:
                    print("list is empty")
                    break
            
            for t in threads:
                t.join()
                
class LandingStateService():
    """landing state service"""
    search_service_number = 1
    update_service_number = 2
    
    def __init__(self, landing_db):
        self.landingDb = landing_db
    
    def get_uavs_requesting_land(self):
        """return list of uavs that have wps"""
        uavs_landing = []
        for uav_id, uav in self.landingDb.items():
            if uav.service_state == self.search_service_number:
                uavs_landing.append(uav)
                
        return uavs_landing
    
    def allow_land(self, uavs_landing, uav):
        landing_spot = [uav.goal[0], uav.goal[1], 0]
        uav.path.append(tuple(landing_spot))
        uav.go_to_wp(uav.current_position, landing_spot)
        uav.update_service_state(self.update_service_number)
        print("uav has landed")
    
    def main(self):
        uavs_landing = self.get_uavs_requesting_land()
        if not uavs_landing:
            print("no drones looking for his")
        else:
            threads = []
            for idx, uav in enumerate(uavs_landing[:]):
                t = multiprocessing.Process(self.allow_land(uavs_landing, uav))
                t.start()
                threads.append(t)
                
                if not uavs_landing:
                    print("list is empty")
                    break
            
            for t in threads:
                t.join()
    
class PostFlightService():
    """this class is simliar to the preflight class"""
    search_service_number = 2
    update_service_number = 3
    
    def __init__(self, homeBase, landing_db, min_h, max_h):
        self.homeBase = homeBase
        self.landingDb = landing_db
        self.uav_service_state_num = 0
        self.min_h = min_h
        self.max_h = max_h
        
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
            (grid[int(obstacle[2]),int(obstacle[0]), int(obstacle[1])]) = 1
            
        return obstacle_list
    
    def return_unassigned_list(self,some_list, index):
        """return all other zones or uavs not assigned to uav to make as a no fly zone"""
        #copy = [int(i) for i in some_list]
        copy = some_list
        copy.pop(index) 
        return copy

    def get_dynamic_obstacles(self, idx, uav_path_obs, obstacle_list,  \
                               path_list, uav_loc_list, grid):
        """generate dynamic obstacles from uav waypoints"""
        #should be a function to make dynamic obstacles
        if idx == 0:
            new_obstacle = obstacle_list 
        else:
            if len(uav_path_obs) < idx:
                new_obstacle = obstacle_list + \
                    self.return_unassigned_list(uav_loc_list[:], idx)
                
            else:
                uav_path_obs.append(path_list[idx-1])
                flat_list = [item for sublist in uav_path_obs for item in sublist]
                new_obstacle = obstacle_list + \
                   self.return_unassigned_list(uav_loc_list[:], idx) + flat_list
            
        grid_copy = grid.copy()
        new_obstacle = self.add_obstacles(grid_copy, new_obstacle)

        return grid_copy, new_obstacle
    
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
        goal_point = uav.home_position
        astar = PathFinding.Astar(grid_copy, new_obstacle,  uav_loc, goal_point,\
                                  self.min_h, self.max_h)
        success_or_no, uav_wp = astar.main()
        if success_or_no == False:
            uav_filtered_wp = None
        else:
            uav_filtered_wp = self.reduce_waypoints(uav_wp)
        return uav_wp, uav_filtered_wp, success_or_no
    
    def return_uav_loc_list(self):
        """return list of uavs"""
        uav_loc_list = []
        for uav_id, uav in self.landingDb.items():
            if uav.service_state == self.search_service_number:
                uav_loc_list.append(uav.current_position)
                
        return uav_loc_list
    
    def get_uavs_requesting_departure(self):
        uavs_departure = []
        for uav_id, uav in self.landingDb.items():
            if uav.service_state == self.search_service_number:
                uavs_departure.append(uav)
                
        return uavs_departure
    
    def hover_up(self, uav, height_z):
        hover_spot = [uav.current_position[0], uav.current_position[1], height_z]
        uav.go_to_wp(uav.current_position, hover_spot)
        
    
    def main(self):
        """main implementation """
        grid = self.generate_grid()
        obstacle_list = self.generate_3d_obstacles(HomeBase.STATIC_OBSTACLE,25)
        obstacle_list = self.add_obstacles(grid, obstacle_list)
        uavs_departure = self.get_uavs_requesting_departure()
        uav_loc_list = self.return_uav_loc_list()
        
        idx = 0
        uav_path_obs = []
        path_list = []
        
        for idx, uav in enumerate(uavs_departure):
            self.hover_up(uav, 7.0)
            """generate dynamic obstacles"""
            grid_copy, new_obstacle = self.get_dynamic_obstacles(idx, uav_path_obs, obstacle_list, \
                           path_list, uav_loc_list, grid)
            
            """find waypoints with Astar pathfinding"""
            uav_wp, uav_filtered_wp, success_or_no = self.find_waypoints(grid_copy, new_obstacle, uav)
            
            if success_or_no == True:    
                uav.set_path_home(uav_wp)
                #uav.set_path(uav_filtered_wp)
                #set the landing zone as open now
                """set new obstacles"""
                path_list.append(uav_wp)
                idx += 1
                print("index", idx)
            else:
                continue

class HomeSenderService():
    """Need to refactor this code"""
    def __init__(self, homeBase, landing_db):
        self.landingDb = landing_db
        self.search_service_number = 2
        self.update_service_number = 3 
        self.homeBase = homeBase
        
    def is_arrived_to_zone(self,waypoint_coords, uav_curr_location):
        """check if close to distance"""
        zone_coords = np.array(waypoint_coords)
        uav_coords = np.array(uav_curr_location)
        dist = math.sqrt((zone_coords[0]- uav_coords[0])**2+ \
                         (zone_coords[1]- uav_coords[1])**2) 
        if dist <= 1.0:
            return True
        else:
            return False
        
    def get_uavs_with_wps(self):
        """return list of uavs that have wps"""
        uavs_with_wp_list = []
        for uav_id, uav in self.landingDb.items():
            if uav.service_state == self.search_service_number and uav.path_home != None:
                uavs_with_wp_list.append(uav)
                
        return uavs_with_wp_list
    
    def send_wp_commands(self, uavs_with_wp_list, uav):  
        """send waypoint commands to uav"""
        waypoint_list = uav.path_home
        print("SENDING HOME")
        for idx,wp in enumerate(waypoint_list):
            uav.go_to_wp(uav.current_position,wp)
            #self.homeBase.ZONE_DB[uav.zone_index].set_vacancy(True)
            if idx > (len(waypoint_list)-2):
                print("reached final waypoint")
                uav.update_service_state(self.update_service_number)
                uav.go_to_wp(uav.current_position,waypoint_list[-1])
                self.homeBase.set_landing_zone_vacancy(uav.zone_index, True)
                break

            
    def main(self):
        uavs_with_wp_list = self.get_uavs_with_wps()
        if not uavs_with_wp_list:
            print("no drones looking for this")
        else:
            threads = []
            for idx, uav in enumerate(uavs_with_wp_list[:]):
                self.homeBase.set_landing_zone_vacancy(uav.zone_index, True)
                t = multiprocessing.Process(self.send_wp_commands(uavs_with_wp_list, uav))
                
                t.start()
                threads.append(t)
                
                if not uavs_with_wp_list:
                    print("list is empty")
                    break
            
            for t in threads:
                t.join()


def check_mission_status():
    uavs_with_wp_list = []
    for uav_id, uav in global_landing_db.items():
        if uav.service_state == 0 and uav.path == None:
            uavs_with_wp_list.append(uav)
    print("uavs", uavs_with_wp_list)
    
    return uavs_with_wp_list

def test_simulation():
    uavs_leftover = check_mission_status()
    homeBase = HomeBase()
    preLandingService = PreLandingService(homeBase, global_landing_db, min_h, max_h)
    pathSenderService = PathSenderService(global_landing_db)
    landingServiceState = LandingStateService(global_landing_db)
    postFlightService = PostFlightService(homeBase, global_landing_db, min_h, max_h)
    homeSenderService = HomeSenderService(homeBase, global_landing_db)
    while uavs_leftover:
        preLandingService.main()
        pathSenderService.main()
        landingServiceState.main()
        postFlightService.main()
        homeSenderService.main()
        uavs_leftover = check_mission_status()
        
    

if __name__ == '__main__':
    
    """initialize randomized values I will need montecarlo randomization here"""
    drone0 = Drone.QuadCopter("uav0", [0,1,10], [0,1,10], True)
    drone1 = Drone.QuadCopter("uav1", [6,0,10], [6,0,10], True)
    drone2 = Drone.QuadCopter("uav2", [15,0,10], [15,0,10],True)
    drone3 = Drone.QuadCopter("uav3", [0,15,10], [0,15,10], True)
    drone4 = Drone.QuadCopter("uav4", [20,1,10], [20,1,10],True)

    uavs = [drone0, drone1, drone2, drone3, drone4]  
    
    min_h = 0.5
    max_h = 1.5
    
    """databases"""
    overallDb = UTMDatabase.OverallDatabase(uavs)
    overall_db = overallDb.listen_for_incoming_uavs()    
    landingDb = UTMDatabase.LandingDatabase(overall_db)
    landingDb.does_uav_want_service()
    
    """multiprocessing/threading for dictionary"""
    manager = Manager()
    global_landing_db = landingDb.get_landing_db()
    landing_db = manager.dict(global_landing_db)
    
    """instantiate classes"""
    test_simulation()
    #"""begin multiprocessing"""
    # uavs_leftover = check_mission_status()
    # homeBase = HomeBase()
    # preLandingService = PreLandingService(homeBase, global_landing_db, min_h, max_h)
    # pathSenderService = PathSenderService(global_landing_db)
    # landingServiceState = LandingStateService(global_landing_db)
    # postFlightService = PostFlightService(homeBase, global_landing_db, min_h, max_h)
    # homeSenderService = HomeSenderService(homeBase, global_landing_db)
    # while uavs_leftover:
    #     preLandingService.main()
    #     pathSenderService.main()
    #     landingServiceState.main()
    #     postFlightService.main()
    #     homeSenderService.main()
    #     uavs_leftover = check_mission_status()
    #     if not uavs_leftover:
    #         print("Simulation is over")
    
        

        

    
    
    
    
    
    
    
    
    
    