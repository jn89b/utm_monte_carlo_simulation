# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 18:19:16 2021

@author: jnguy
"""

import numpy as np 


class QuadCopter():
    """
    Class to represent quadcopter
    '''
    
    Attributes
    --------------
    id : str  might want to change to an int
        the id number or name of the UAV 
    starting_position  list[int]
        coordinates of the quadcopter
    goal : list[int]
        goal destination of quadcopter
    zone_index : int
        index number of zone destination
    path : list[[int_x,int_y], ...] 
        waypoint path of quad after assignment of landing zone
    service_state : int
        service state of the quadcopter 0,1,2,3
    
    Methods: 
    ----------------
    get_uav_state(self)
    go_to_wp()
    update_position()
    apply_pid()
    update_service_status()
    set_zone_index()
    set_path()
    set_goal()
    
    
    """
    
    speed_vector = [0.5]
    k = 1.0
    
    def __init__(self, id_name ,home_position, loiter_position, curr_position, want_service):
        self.id = id_name
        self.home_position = home_position
        self.loiter_position = loiter_position
        self.current_position = curr_position
        self.want_service = want_service
        self.zone_index = None
        self.path = None
        self.goal = None
        self.service_state = None
        self.distance_from_base = None 
        self.wp_index = 0
        self.path_home = None
        
    def get_uav_state(self):
        return self.service_state
    
    def __lt__(self, other):
        return self.distance_from_base < other.distance_from_base
    
    def go_to_wp(self, current_position, wp):
        """
        send the drone to the waypoint position
        waypoint is a tuple
        """
        while self.current_position != wp:
            gain = self.apply_pid(current_position, wp)
            current_position = np.array(self.current_position) + gain
            self.current_position = list(current_position)
            
            if tuple(self.current_position) == wp:
                self.update_position(wp)
                #print("arrived to wp", self.current_position)
                break
        
    def update_position(self, new_position):
        self.current_position = new_position

    def apply_pid(self, current_position,wp):
        """get gains needed to get to area"""
        error = np.array(wp) - np.array(current_position)
        
        return error*self.k
    
    def update_service_state(self, state_num):
        """mutator"""
        self.service_state = state_num
        
    def set_zone_index(self, zone_index):
        """mutator"""
        self.zone_index = zone_index

    def set_path(self, path):
        """mutator to assign path list"""
        self.path = path
        
    def set_path_home(self, path_home):
        self.path_home = path_home
        
    def set_goal(self, goal):
        """mutator to assign goal[x,y,z] point"""
        self.goal = goal 
    
    def begin_charging(self):
        print("I'm charging", self.id)
        
    def set_distance_from_base(self, distance_from_base):
        self.distance_from_base = distance_from_base
        
        
        
        