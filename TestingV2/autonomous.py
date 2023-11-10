import time
import os
import math
import multiprocessing
import subprocess
import exiftool
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from array import array
import pymavlink.dialects.v20.all as dialect
from haversine import haversine, Unit
class CLASS:
    def __init__(self):
        """
        Initializes the CLASS object.

        This method initializes the class and establishes connections to UAS and the camera.

        :return: None
        """
        #PARAMTERS
        self.ALTITUDE = 75.0
        self.WAYPOINT_RADIUS = 1
        self.PAYLOAD_RADIUS = 1
        self.SEARCH_AREA_RADIUS = 1
        #connecting to UAS with dronekit
        print("Connecting to UAS")
        self.connection_string = 'udpin:localhost:14551' #Software in the loop
        #self.connection_string = "/dev/ttyACM0" #usb to micro usb
        
        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
        print("Connected with DroneKit")

        #connecting to mavlink
        print('Connecting MavLink')
        self.UAS_mav = mavutil.mavlink_connection(self.connection_string, baud=57600)
        self.UAS_mav.wait_heartbeat()
        print("hearbeat from system {system %u compenent %u}" %(self.UAS_mav.target_system, self.UAS_mav.target_component))
        print("Mavlink Connected ")
        

        
        # print('CREATING IMAGE DIRECTORY')
        # image_dir = f'image_{time.ctime(time.time())}'
        # print(f'MADE DIRECTORY {image_dir}')
        # os.mkdir(image_dir)
        # os.chdir(str(image_dir))
        # print(f'MOVED TO {image_dir} DIRECTORY')
        # print("CREATING TEST DATA FILE")
        # with open('Data_log.txt', "a") as file:
        #         file.write("Time Log:\n")
        
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
            -35.3628465,-35.3629777, -35.3621116
        ]
        self.waypoint_lap_longitude = [
            149.1644059,149.1625927, 149.1632472
        ]

        #predefined search area value for Kawainui test
        self.search_area_latitude = [
            -35.3628465,-35.3629777, -35.3621116

        ]

        self.search_area_longitude = [
            149.1644059,149.1625927, 149.1632472

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
    def trigger_camera(self, image_name):
        """
        Trigger the camera to capture an image.

        This method triggers the camera to capture an image and saves it with the provided filename.

        :param filename: The filename to use for the captured image.
        :return: None
        """
        start = time.time()
        print(f'{image_name} IS BEING TAKEN')
        cmd = ('gphoto2', '--capture-image-and-download', '--filename', image_name)
        self.subprocess_execute(cmd)
        end = time.time()
        difference = end - start
        self.trigger_camera_time.append(difference)
        return print(f'{image_name} Captured \n')

    def subprocess_execute(self, command):
        """
        Execute a subprocess command with the provided arguments and record the execution time.

        Args:
            command (str): The subprocess command to execute.

        Returns:
            None
        """
        start = time.time()
        subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        end = time.time()
        difference = end - start
        self.subprocess_execute_time.append(difference)
    
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

    def haversine(self, lat1, lon1):
        """
        Use the Haversine formula to calculate the distance between two coordinates.

        Args:
            lon1 (float): Longitude of the first coordinate.
            lat1 (float): Latitude of the first coordinate.

        Returns:
            float: The distance between the two coordinates in meters.
        """
        start = time.time()
        curr_lat = 0
        curr_lon = 0
        message = self.UAS_mav.recv_match(type=[dialect.MAVLink_position_target_global_int_message.msgname,
                                       dialect.MAVLink_global_position_int_message.msgname],
                                 blocking=True)   
        #print(message)
        message = message.to_dict()
        if message["mavpackettype"] == dialect.MAVLink_global_position_int_message.msgname:
            curr_lat = message["lat"] * 1e-7
            curr_lon = message["lon"] * 1e-7
        print(f'{lat1},{lon1}')
        distance = haversine((curr_lat * 1e-7,curr_lon * 1e-7),(lat1 * 1e-7,lon1 * 1e-7), unit = 'ft')
        end = time.time()
        difference = end - start
        self.haversine_time.append(difference)
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

    def spline_waypoint_command(self, latitude, longitude, seq):
        """
        Define a spline waypoint command.

        Args:
            latitude (float): The latitude coordinate.
            longitude (float): The longitude coordinate.
            seq (int): The number sequence according to mission.

        Returns:
            None
        """ 

        command = dialect.MAV_CMD_NAV_SPLINE_WAYPOINT

        message = dialect.MAVLink_mission_item_int_message(
            self.UAS_mav.target_system,  #target_system
            self.UAS_mav.target_component, #target_component
            seq,
            dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command, #MAV_CMD_NAV_WAYPOINT (16) or try to change it to  waypoint_command
            0,
            1, #auto continue 
            0, #hold (s)
            self.PAYLOAD_RADIUS, #Accept radius (m)
            self.PAYLOAD_RADIUS, #pass radius (m)
            0, #yaw (deg)
            int(latitude*1e7),  
            int(longitude*1e7),
            self.ALTITUDE,
            0
            )

        # Send the message
        self.UAS_mav.mav.send(message)
        #msg = self.UAS_mav.recv_match(type = dialect.MAVLink_mission_item_int_message.msgname, blocking = True)
    

    def waypoint_command(self, latitude, longitude, seq):
        """
        Define a waypoint command.

        Args:
            latitude (float): The latitude coordinate.
            longitude (float): The longitude coordinate.
            seq (int): The number sequence according to mission.

        Returns:
            None
        """ 

        command = dialect.MAV_CMD_NAV_WAYPOINT

        message = dialect.MAVLink_mission_item_int_message(
            self.UAS_mav.target_system,  #target_system
            self.UAS_mav.target_component, #target_component
            seq,
            dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command, #MAV_CMD_NAV_WAYPOINT (16) or try to change it to  waypoint_command
            0,
            1, #auto continue 
            0, #hold (s)
            self.WAYPOINT_RADIUS, #Accept radius (m)
            self.WAYPOINT_RADIUS, #pass radius (m)
            math.nan, #yaw (deg)
            int(latitude*1e7),  
            int(longitude*1e7),
            self.ALTITUDE,
            0
            )

        # Send the message
        self.UAS_mav.mav.send(message)
        #msg = self.UAS_mav.recv_match(type = dialect.MAVLink_mission_item_int_message.msgname, blocking = True)
      

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
    
    def response(self, keyword):
        return print("-- Message Read " + str(self.UAS_mav.recv_match(type = keyword, blocking = True)))
    

    def count(self, waypoints):
        self.UAS_mav.mav.mission_count_send(
        self.UAS_mav.target_system,  #target_system
        self.UAS_mav.target_component,#target_component
        waypoints,#total number of commands in the mission
        0 #0-main mission
        )
        

    def mission_start(self):
        command = mavutil.mavlink.MAV_CMD_MISSION_START
        self.UAS_mav.mav.command_long_send(
        self.UAS_mav.target_system,  #target_system
        self.UAS_mav.target_component,#target_component
        command,#command
        0,0,0,0,0,0,0,0
        )
        return True


    def waypoint_lap( self ):
        """
        Define a sequence of waypoints to be followed by the UAS in a lap.

        Returns:
            str: A message indicating lap completion.

        """
        self.count(len(self.waypoint_lap_latitude)+1)
        self.spline_waypoint_command(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ],0)

        for wp in range(len(self.waypoint_lap_latitude)):
            self.spline_waypoint_command(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],wp+1)
            #self.waypoint_reached(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ], self.WAYPOINT_RADIUS)

        self.mission_start()
        self.response("MISSION_ACK")
        for reached in range(len(self.waypoint_lap_latitude)):
            self.response("MISSION_ITEM_REACHED")
        self.UAS_dk.commands.clear()
        self.UAS_dk.commands.upload()
        return print("DONE with lap")

    def search_area_waypoint(self):
        """
        Define a search area waypoint.

        This method defines a search area waypoint.

        :return: None
        """
        start= time.time()
        print('Now Conducting the search area')
        self.UAS_dk.commands.clear()
        self.UAS_dk.commands.upload()
        #print(self.UAS_dk.location.global_frame)
        self.UAS_dk.mode = VehicleMode("GUIDED")
        #self.UAS_dk.simple_goto(LocationGlobalRelative(self.search_area_latitude[2],self.search_area_longitude[2], self.ALTITUDE))
        for wp in range(len(self.search_area_latitude)):
            target_locaton=LocationGlobalRelative(self.search_area_latitude[wp],self.search_area_longitude[wp], self.ALTITUDE)
            self.UAS_dk.simple_goto(target_locaton)
            self.waypoint_reached(self.search_area_latitude[wp],self.search_area_longitude[wp],self.PAYLOAD_RADIUS)
            #print(wp)
            #print(self.UAS_dk.location.global_frame)
            #self.response("MISSION_ITEM_REACHED")
            #self.waypoint_reached(self.search_area_latitude[wp],self.search_area_longitude[wp], self.SEARCH_AREA_RADIUS)
            #get attitide data
            #p1 = multiprocessing.Process(target=self.attitude())
            #self.attitude()
            #self.trigger_camera(f'image{wp+1}.jpg')
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
    
