import time
import os
import math
import multiprocessing
import subprocess
#import exiftool
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from array import array
import pymavlink.dialects.v20.all as dialect

class CLASS:
    def __init__(self):
        """
        Initializes the CLASS object.

        This method initializes the class and establishes connections to UAS and the camera.

        :return: None
        """
        #PARAMTERS
        self.ALTITUDE = 75.0
        self.WAYPOINT_RADIUS = 7.5
        self.PAYLOAD_RADIUS = 2
        self.SEARCH_AREA_RADIUS = 2
        #connecting to UAS with dronekit
        print("Connecting to UAS")
        self.connection_string = 'udpin:localhost:14551'
        #self.connection_string = "/dev/ttyACM0" #usb to micro usb
        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
        print("Connected with DroneKit")

        #connecting to mavlink
        print('Connecting MavLink')
        self.UAS_mav = mavutil.mavlink_connection(self.connection_string, baud=57600)
        self.UAS_mav.wait_heartbeat()
        print("hearbeat from system {system %u compenent %u}" %(self.UAS_mav.target_system, self.UAS_mav.target_component))
        print("Mavlink Connected")
        

        
        print('CREATING IMAGE DIRECTORY')
        image_dir = f'image_{time.ctime(time.time())}'
        print(f'MADE DIRECTORY {image_dir}')
        os.mkdir(image_dir)
        os.chdir(str(image_dir))
        print(f'MOVED TO {image_dir} DIRECTORY')
        print("CREATING TEST DATA FILE")
        with open('Data_log.txt', "a") as file:
                file.write("Time Log:\n")
        
        # writing file variable
        self.attitude_time = []
        self.deliver_payload_time = []
        self.geotag_time = []
        self.haversine_time = []
        self.search_area_waypoint_time = []
        self.subprocess_execute_time = []
        self.trigger_camera_time = []
        self.waypoint_lap_time = []

        #connect the camera
        print("Connecting to the camera")
        self.command = ["gphoto2", "--auto-detect"]
        #self.subprocess_execute(self.command)
        print('Camera Connected')

        #declaring initial variable
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.image_number = 1
        self.drone_sensory = [self.pitch, self.roll, self.yaw, self.lat, self.lon, self.alt]
        self.currWP_index = 0
        self.lap = 0
        self.filename = f"image"
        self.waypoint_lap_latitude = [
            -35.3623650,-35.3635315, -35.3627590
        ]
        self.waypoint_lap_longitude = [
            149.1661235,149.1636205, 149.1613438
        ]

        #predefined search area value for Kawainui test
        self.search_area_latitude = [
            -35.3623083, -35.3617606, -35.3617525, -35.3621956, 

        ]

        self.search_area_longitude = [
            149.1621797, 149.1621401, 149.1630390, 149.1630192, 

        ]

        #self.user_waypoint_input()
        '''
        print("AUTONOMOUS SCRIPT IS READY")
        while self.IS_ARMED != True:
            print("Waiting for arming....")
            time.sleep(0.5)

        print("UAS IS NOW ARMED")
        while self.IS_AUTO != True:
            print("Waiting for UAS to be in AUTO MODE.........")
            time.sleep(0.5)
        print("UAS IS NOW IN AUTO MODE")
        print("!------------------ MISSION STARTING ----------------------!")
        '''

    def toRadian(self, degree):
        """
        Convert from degree to radian.

        Args:
            degree (float): The angle in degrees to be converted to radians.

        Returns:
            float: The equivalent angle in radians.
        """
        pi = math.pi
        return degree * (pi / 180)

    def haversine(self, lon1, lat1):
        """
        Use the Haversine formula to calculate the distance between two coordinates.

        Args:
            lon1 (float): Longitude of the first coordinate.
            lat1 (float): Latitude of the first coordinate.

        Returns:
            float: The distance between the two coordinates in meters.
        """
        start = time.time()
        print(self.UAS_dk.location.global_frame)
        earth_radius = 6371000
        curr_location = self.UAS_dk.location.global_relative_frame
        lat1 = self.toRadian(lat1)
        lon1 = self.toRadian(lon1)
        lat2 = self.toRadian(curr_location.lat)
        lon2 = self.toRadian(curr_location.lon)
        dlat = lat2- lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2) * math.sin(dlon/2)**2
        c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = earth_radius * c
        end = time.time()
        difference = end - start

        self.haversine_time.append(difference)
        # feet conversion * earth radius * something
        return distance
    def IS_ARMED(self):
        """
        Check if the UAS is "ARMED" mode.

        Returns:
            bool: True if the UAS is ARMED, False otherwise.
        """  
        if self.UAS_dk.armed:
            return print("It is armed")
        return print("It is not armed")

    def IS_AUTO(self):
        """
        Check if the UAS is in "AUTO" mode.

        Returns:
            bool: True if the UAS is in AUTO, False otherwise.
        """        
        if self.UAS_dk.mode == "AUTO":
            return print('It is in auto' )
        return print("Not in auto")

    def RTL_stat( self ):
        """
        Check if the UAS is in "Return to Launch" mode.

        Returns:
            bool: True if the UAS is in RTL mode, False otherwise.
        """
        return self.UAS_dk.mode == "RTL"


    def waypoint_command(self, latitude, longitude, seq, curr):
        """
        Define a waypoint command (not implemented).

        Args:
            latitude (float): The latitude coordinate.
            longitude (float): The longitude coordinate.

        Returns:
            None
        """ 

        command = dialect.MAV_CMD_NAV_SPLINE_WAYPOINT

        print("ffd")
        message = dialect.MAVLink_mission_item_int_message(
            self.UAS_mav.target_system,  #target_system
            self.UAS_mav.target_component, #target_component
            seq,
            dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command, #MAV_CMD_NAV_WAYPOINT (16) or try to change it to  waypoint_command
            0,
            1, #auto continue 
            0, #hold (s)
            10, #Accept radius (m)
            10, #pass radius (m)
            0, #yaw (deg)
            int(latitude*1e7),  
            int(longitude*1e7),
            self.ALTITUDE,
            0
            )
        '''
        # Send a waypoint command
        msg = self.UAS_dk.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            0,
            float(0.0),
            float(0.0),
            float(0.0),
            float(0.0),
            int(latitude*1e7),
            int(longitude*1e7),
            float(self.ALTITUDE)
        )
        self.UAS_dk.send_mavlink(msg)
        '''
        # Send the message
        self.UAS_mav.mav.send(message)
        #msg = self.UAS_mav.recv_match(type = dialect.MAVLink_mission_item_int_message.msgname, blocking = True)
        return print("Waypoint reached")

    def waypoint_reached (self, latitude_deg, longitude_deg, radius ):
        """
        Check if the UAS has reached a specified waypoint.

        Args:
            latitude_deg (float): The latitude coordinate of the waypoint.
            longitude_deg (float): The longitude coordinate of the waypoint.

        Returns:
            bool: True if the waypoint is reached, False otherwise.
        """
        #distance between 2 points retuirn value in feet    
        distance = self.haversine(latitude_deg,longitude_deg )

        #checking is UAS reached within 15 feet in diameter of the desired coordinate desitination
        while(distance > radius):

            #distance between 2 points retuirn value in feet    
            distance = self.haversine(latitude_deg, longitude_deg)            
            print(f"Distance to waypoint: {distance}")
            time.sleep(.5)

        print("REACHED WAYPOINT")
        
        return True
    
    def waypoint_lap( self ):
        """
        Define a sequence of waypoints to be followed by the UAS in a lap.

        Returns:
            str: A message indicating lap completion.

        """
        self.UAS_mav.mav.mission_count_send(
        self.UAS_mav.target_system,  #target_system
        self.UAS_mav.target_component,#target_component
        4,0 
        )
        self.waypoint_command(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ],0,1)
        self.waypoint_command(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ],1,1)

        #self.waypoint_reached(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ], self.WAYPOINT_RADIUS)
        self.waypoint_command(self.waypoint_lap_latitude[ 1 ], self.waypoint_lap_longitude[ 1 ],2,1)
        self.waypoint_command(self.waypoint_lap_latitude[ 2 ], self.waypoint_lap_longitude[ 2 ],3,0)
        self.UAS_dk.mode = VehicleMode("AUTO")

            #self.waypoint_command(-35.3627590, 149.1633438)
            
            #self.UAS_dk.simple_goto(LocationGlobalRelative(-35.3627590, 149.1633438))
            #self.waypoint_reached(self.waypoint_lap_latitude[ curr_wp ], self.waypoint_lap_longitude[ curr_wp ], self.WAYPOINT_RADIUS)
        
        
        return f"Lap number {self.lap} is complete"

    def search_area_waypoint(self):
        """
        Define a search area waypoint.

        This method defines a search area waypoint.

        :return: None
        """
        start = time.time()
        self.UAS_mav.mav.mission_count_send(
        self.UAS_mav.target_system,  #target_system
        self.UAS_mav.target_component,#target_component
        4,0 
        )
        for x in range(len(self.search_area_latitude)):
            print(x)
            #go to wp
            print(f"############ GOING TO SEARCH AREA WAYPOINT: {x+1} ############") 
            self.waypoint_command(self.search_area_latitude[x],self.search_area_longitude[x],x,x) 
            self.UAS_dk.mode =VehicleMode("AUTO")
            #location = LocationGlobalRelative(self.search_area_latitude[x],self.search_area_longitude[x],float(self.ALTITUDE))
            #self.UAS_dk.simple_goto( location )
            
            #call the waypoint reached
            #self.waypoint_reached(self.search_area_latitude[x],self.search_area_longitude[x], self.SEARCH_AREA_RADIUS)
            #get attitide data
            #p1 = multiprocessing.Process(target=self.attitude())
            #self.attitude()
            #self.trigger_camera(f'image{x+1}.jpg')
            #take image
            #p2 = multiprocessing.Process(target=self.trigger_camera, args= (f"image{x+1}.jpg",))
            #start the execution and wait 
            #p1.start()
            #p2.start()
            #p1.join()
            #p2.join()
            #geotag
            #self.geotag(f'image{x+1}.jpg')

        end = time.time()
        difference = end - start
        self.search_area_waypoint_time.append(difference)

        return print("UAS COMPLETED SEARCH THE AREA")



if __name__ == '__main__':
    pass
    
