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
    k = 0.75
    
    def __init__(self, id_name ,home_position, curr_position, want_service):
        self.id = id_name
        self.home_position = home_position
        self.current_position = curr_position
        self.want_service = want_service
        self.zone_index = None
        self.path = None
        self.goal = None
        self.service_state = None
    
    def get_uav_state(self):
        return self.service_state
    
    def go_to_wp(self, current_position, wp):
        """
        send the drone to the waypoint position
        """
        while self.current_position != wp:
            gain = self.apply_pid(current_position, wp)
            current_position = np.array(self.current_position) + np.array(self.current_position*gain)
            self.current_position = list(current_position)
            if self.current_position == wp:
                print("arrived to wp")
        
    def update_position(self, curr_position):
        return self.current_position

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
        
    def set_goal(self, goal):
        """mutator to assign goal[x,y,z] point"""
        self.goal = goal 
    
        
        
        
        
        