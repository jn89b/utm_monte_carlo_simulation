# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 15:33:50 2021

@author: jnguy
"""
import math

class AbstractDatabaseInsert():
    """"""
    def __init__(self):
        pass
    
    def insert_to_db(self, db, key_name, info):
        db[key_name] = info
        #print("inserting to database", db, key_name, info)
        return db 
    
    def remove_from_db(self,key_name):
        pass
    
    def update_to_db(self, key_name, val):
        pass

class AbstractDatabaseQuery():
    """"""
    def __init__(self):
        pass
    
    def request_query(self, db, field_name):
        """method to remove from database"""
        return db[field_name]
        

class OverallDatabase(AbstractDatabaseQuery,AbstractDatabaseInsert):
    
    def __init__(self, uav_list):
        self.db = {}
        self.uav_list = uav_list
        
    def check_duplicate(self,uav):
        """check if uav is already in the database"""
        if uav in self.db:
            return True
    
    def listen_for_incoming_uavs(self):
        """listen for incoming uavs"""
        for uav in self.uav_list:
            
            if self.check_uav_has_id(uav) is False:
                #print("print uav has no ids")
                continue
            
            if not uav.id in self.db:
                self.insert_to_db(self.db, uav.id, uav)
                
        return self.db 
    
    
    def check_uav_has_id(self, uav):
        if uav.id == None:
            return False

class LandingDatabase(AbstractDatabaseQuery,AbstractDatabaseInsert):
    def __init__(self, overall_db, homeBase):
        self.overall_db = overall_db
        self.landing_db = {}
        self.homeBase = homeBase
            
    def does_uav_want_service(self):
        uavs_that_want_service = []
        """check if uav wants service or true"""
        for uav_id, uav in self.overall_db.items():
            if uav.want_service == True:
                uav.update_service_state(0)
                distance = self.compute_2d_euclidean(uav.home_position, self.homeBase.BASE_LOCATION)
                uav.set_distance_from_base(distance)
                uavs_that_want_service.append(uav)
            else:
                continue
            
        return uavs_that_want_service
        
    def get_landing_db(self):
        """accessor get landing database info"""
        return self.landing_db

    def compute_2d_euclidean(self, position, goal_position):
        """compute euclidiean with position and goal as 2d vector component"""
        distance =  math.sqrt(((position[0] - goal_position[0]) ** 2) + 
                        ((position[1] - goal_position[1]) ** 2))
        
        return distance


    def find_uavs_proximity(self, uav_list):
        """find uavs close"""
        for uav_id, uav in self.overall_db.items: 
            distance = self.compute_2d_euclidean(uav.home_position, self.homeBase.BASE_LOCATION)
            uav.set_distance_from_base(distance)
            
        return uav_list
    
    def insert_to_landing_db(self, uav):
        self.insert_to_db(self.landing_db, uav.id, uav)
    
    def main(self):
        """Sort uavs and insert to database"""
        uavs_that_want_service = self.does_uav_want_service()
        sorted_uavs = sorted(uavs_that_want_service)
        for uav in sorted_uavs:
            self.insert_to_landing_db(uav)
            
        
            
        

        
        
        
        
        