# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 15:33:50 2021

@author: jnguy
"""

class AbstractDatabaseInsert():
    """"""
    def __init__(self):
        pass
    
    def insert_to_db(self, db, key_name, info):
        db[key_name] = info
        print("inserting to database", db, key_name, info)
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
                print("print uav has no ids")
                continue
            
            if not uav.id in self.db:
                self.insert_to_db(self.db, uav.id, uav)
                
        return self.db 
    
    
    def check_uav_has_id(self, uav):
        if uav.id == None:
            return False

class LandingDatabase(AbstractDatabaseQuery,AbstractDatabaseInsert):
    print("incoming uavs")
    def __init__(self, overall_db):
        self.overall_db = overall_db
        self.landing_db = {}
            
    def does_uav_want_service(self):
        """check if uav wants service or true"""
        for uav_id, uav in self.overall_db.items():
            if uav.want_service == True:
                print("uav wants service", uav.want_service)
                uav.update_service_state(0)
                self.insert_to_db(self.landing_db, uav_id, uav)
            else:
                continue
        
    def get_landing_db(self):
        """accessor get landing database info"""
        return self.landing_db


        
        
        
        
        