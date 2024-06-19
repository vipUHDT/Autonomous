import time
import os
import math
import multiprocessing
import subprocess
import exiftool
import re
import shutil
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from array import array
import pymavlink.dialects.v20.all as dialect
from haversine import haversine, Unit
from adafruit_servokit import ServoKit

class CLASS:
    def __init__(self):
        """
        Initializes the CLASS object.

        This method initializes the class and establishes connections to UAS and the camera.

        :return: None
        """
        #PARAMTERS
        self.ALTITUDE = 22.8 # meters
        self.alt_AD = 26
        self.alt_IP = 27
        self.WAYPOINT_RADIUS = 5 # feet
        self.PAYLOAD_RADIUS = 5 # feet
        self.SEARCH_AREA_RADIUS = 5 # feet
        self.WAYPOINT_SPEED = 15 # m/s
        self.SEARCH_SPEED = 10 # m/s
        self.DELIVER_SPEED = 10 # m/s
        #connecting to UAS with dronekit
        print("Connecting to UAS")
        # self.connection_string = 'udp:127.0.0.1:14551' #Software in the loop
        self.connection_string = "/dev/ttyACM0" #usb to micro usb
        self.SK = ServoKit( channels = 16 )

        #Connect to DroneKit
        self.connect_to_dronekit()
        
        print('CREATING IMAGE DIRECTORY')
        image_dir = f'buffer'
        full_dir = f'watchdog'
        print(f'MADE DIRECTORY {image_dir}')
        os.mkdir(image_dir)
        os.mkdir(full_dir)
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
        self.dk_waypoint_lap_time = []
        self.payload_delivery_time = []


        # connect the camera
        # print("Connecting to the camera")
        # self.command = ["gphoto2", "--auto-detect"]
        # self.subprocess_execute(self.command)
        # print('Camera Connected')

        #declaring initial variable
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 10.0
        self.image_number = 1
        self.drone_sensory = [self.pitch, self.roll, self.yaw, self.lat, self.lon, self.alt]
        self.currWP_index = 0
        self.lap = 1
        self.payload = 1
        self.filename = f"image"

        self.waypoint_lap_latitude = [
            21.3997862

        ]
        self.waypoint_lap_longitude = [
            -157.7643800

        ]

        self.search_area_latitude = [
            21.40034560,
            21.40037550,
            21.40040680,
            21.40043730,
            21.40045990,
            21.40049070,
            21.40051790,
            21.40054630,
            21.40057750,
            21.40060310,
            21.40063370,
            21.40066460,
            21.40069360,
            21.40072640,
            21.40080660,
            21.40078100,
            21.40075670,
            21.40072670,
            21.40069860,
            21.40067150,
            21.40064150,
            21.40062030,
            21.40059160,
            21.40056100,
            21.40053540,
            21.40050790,
            21.40047670,
            21.40044610
        ]

        self.search_area_longitude = [
            -157.76474810,
            -157.76467230,
            -157.76459580,
            -157.76452110,
            -157.76445170,
            -157.76437690,
            -157.76430380,
            -157.76423070,
            -157.76415800,
            -157.76408290,
            -157.76400980,
            -157.76393770,
            -157.76386530,
            -157.76379320,
            -157.76384650,
            -157.76391860,
            -157.76399170,
            -157.76406650,
            -157.76414190,
            -157.76421630,
            -157.76429140,
            -157.76435310,
            -157.76442490,
            -157.76449730,
            -157.76457310,
            -157.76464820,
            -157.76472190,
            -157.76479700
        ]

        self.payload_delivery_latitude = [
            
        ]

        self.payload_delivery_longitude = [
            
        ]

        self.payload_delivery_compartment = [

        ]

        self.end_mission_latitude = 0.0

        self.end_mission_longitude = 0.0

        print("AUTONOMOUS SCRIPT IS READY")

        while (self.IS_ARMED() != True):
            print("waiting to be armed")
            print(self.UAS_dk.armed)
            time.sleep(1)
        print("UAS IS NOW ARMED")

        while (self.IS_GUIDED() != True):
            print("waiting to be in GUIDED mode")
            print(self.UAS_dk.mode)
            time.sleep(1)

        

        print("UAS IS NOW IN GUIDED MODE")
        print("!------------------ MISSION STARTING ----------------------!")
         

    def connect_to_dronekit( self ):
        print( "Connecting to DroneKit" )

        self.UAS_dk = connect( self.connection_string, baud = 57600, wait_ready = True, heartbeat_timeout = 180, source_system=1)

        print( "Connected to DroneKit ")

    def attitude(self):
        """
        Retrieve attitude and GPS information.

        This method retrieves the attitude (pitch, roll, yaw) and GPS information (latitude, longitude, altitude)
        from the UAS and updates the class variables accordingly.

        :return: None
        """
        start = time.time()
        # Setting the variable with gps coordinates, yaw pitch and roll
        attitude = self.UAS_dk.attitude
        attitude = str(attitude)
        # Getting the UAS location in long and lat
        gps = self.UAS_dk.location.global_relative_frame
        gps = str(gps)
        # using split method to split string so we can get individual value of yaw, pitch, and roll
        attitude_split = attitude.split(",")
        pitch_split = attitude_split[0].split("=")
        # The pitch value
        self.pitch = pitch_split[1]
        yaw_split = attitude_split[1].split("=")
        # yaw value
        self.yaw = yaw_split[1]
        roll_split = attitude_split[2].split("=")
        # roll value
        self.roll = roll_split[1]
        # splitting the string so we can get the value of longitude and latitude
        gps_split = gps.split(",")
        lat_split = gps_split[0].split("=")
        # value of the lat
        self.lat = self.UAS_dk.location.global_relative_frame.lat
        lon_split = gps_split[1].split("=")
        # value of the long
        self.lon = self.UAS_dk.location.global_relative_frame.lon
        alt_split = gps_split[2].split("=")
        # altitude value
        self.alt = alt_split[1]
        # Send inputs as a string not int
        self.pitch = str(self.pitch)
        self.roll = str(self.roll)
        self.yaw = str(self.yaw)
        self.lat = str(self.lat)
        self.lon = str(self.lon)
        self.alt = str(self.alt)
        self.drone_sensory = [self.pitch, self.roll, self.yaw, self.lat, self.lon, self.alt]
        end = time.time()
        difference = end - start
        self.attitude_time.append(difference)
        return print("Drone Sensory Data Collected")    


    def geotag(self, image_name):
        """
        Geotag an image with sensory data.

        This method geotags a photo with attitude (pitch, roll, yaw) and GPS (latitude, longitude, altitude)
        information.

        :param filename: The filename of the image to geotag.
        :param drone_sensory: The drone sensory data.
        :return: None
        """
        start = time.time()
        # Geotagging photo with the attitude and GPS coordinate
        pyr = ('pitch:' + str(self.drone_sensory[0]) + ' yaw:' + str(self.drone_sensory[2]) + ' roll:' + str(self.drone_sensory[1]))
        print(pyr)
        print( self.drone_sensory[3])
        tag_pyr_command = ('exiftool','-overwrite_original', '-comment=' + str(pyr), str(image_name))
        tag_lat_command = ('exiftool', '-overwrite_original', '-exif:gpslatitude=' + '\'' + str(self.drone_sensory[3]) + '\'', str(image_name))
        tag_long_command = ('exiftool','-overwrite_original', '-exif:gpslongitude=' + '\'' + str(self.drone_sensory[4]) + '\'', str(image_name))
        tag_alt_command = ('exiftool','-overwrite_original', '-exif:gpsAltitude=' + '\'' + str(self.drone_sensory[5]) + '\'', str(image_name))
        self.image_number += 1 
        #executing the tag command in ssh
        '''subprocess.run(tag_pyr_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        subprocess.run(tag_lat_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        subprocess.run(tag_long_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        subprocess.run(tag_alt_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)'''
                
        p1 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_pyr_command,))
        p2 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_lat_command,))
        p3 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_long_command,))
        p4 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_alt_command,))
        
        p1.start()
        time.sleep( 1 )
        p2.start()
        time.sleep( 1 )
        p3.start()
        time.sleep( 1 )
        p4.start()
        
        p1.join()
        p2.join()
        p3.join()
        p4.join()

        end = time.time()
        difference = end - start
        self.geotag_time.append(difference)
        return print(f"{image_name} GEOTAGGED")


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
        return print(f'{image_name} CAPTURED \n')

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
    

    def haversine(self, lat1, lon1):
        """
        Use the Haversine formula to calculate the distance between two coordinates.

        Args:
            lon1 (float): Longitude of the first coordinate.
            lat1 (float): Latitude of the first coordinate.

        Returns:
            float: The distance between the two coordinates in meters.
        """
        current_location = self.UAS_dk.location.global_relative_frame
        distance = haversine((current_location.lat,current_location.lon),(lat1,lon1), unit = 'ft')
        return distance
    
    def IS_ARMED(self):
        """
        Check if the UAS is "ARMED" mode.

        Returns:
            bool: True if the UAS is ARMED, False otherwise.
        """  

        
        while not self.UAS_dk.armed:
            print( "Waiting to be ARMED" )
            time.sleep( 1 )

        print( "UAS is ARMED" )
        return True

    def IS_AUTO(self):
        """
        Check if the UAS is in "AUTO" mode.

        Returns:
            bool: True if the UAS is in AUTO, False otherwise.
        """    
        
        
        while self.UAS_dk.mode != "AUTO":
            self.IS_AUTO()
            print( "Waiting to be in AUTO" )
            time.sleep( 1 )

        print( "UAS is in AUTO" )
        return True
    
    def IS_GUIDED(self):
        """
        Check if the UAS is in "AUTO" mode.

        Returns:
            bool: True if the UAS is in AUTO, False otherwise.
        """        

        while self.UAS_dk.mode != "GUIDED":
            print( "Waiting to be in GUIDED" )

            time.sleep( 1 )
        
        print( "UAS is in GUIDED" )
        return True


    def RTL_stat( self ):
        """
        Check if the UAS is in "Return to Launch" mode.

        Returns:
            bool: True if the UAS is in RTL mode, False otherwise.
        """
        #self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)

        if self.UAS_dk.mode == "RTL":
            return True
        return False

    def gpio_servo_command( self, servo_x, angle ):
        """
        Triggers servo using i2c protocol using adafruit_servokit library
        servo_x: servo number
        angle: angle of the servo (0 to close, 120 to open)
        """
        print( "Dropping Payload" )
        self.SK.servo[ servo_x ].angle = angle
        time.sleep( 1 )
        print( "Dropped Payload" )

    def loiter_command(self, time, seq):
        """
        Define a waypoint command.

        Args:
            latitude (float): The latitude coordinate.
            longitude (float): The longitude coordinate.
            seq (int): The number sequence according to mission.

        Returns:
            None
        """ 

        command = dialect.MAV_CMD_NAV_LOITER_TIME

        message = dialect.MAVLink_mission_item_int_message(
            self.UAS_mav.target_system,  #target_system
            self.UAS_mav.target_component, #target_component
            seq,
            dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command, #MAV_CMD_NAV_WAYPOINT (16) or try to change it to  waypoint_command
            0,
            1, #auto continue 
            0, #hold (s)
            time,0,1,0,0,0,0
            )

        # Send the message
        self.UAS_mav.mav.send(message)
    
    def download_payload_coord( self, file_name ):
        
        with open( file_name, 'r' ) as file:
            for line in file:
                latitude_match = re.search( r"Latitude of payload (\w+): (-?\d+\.\d+)", line )
                longitude_match = re.search( r"Longitude of payload (\w+): (-?\d+\.\d+)", line)
                compartment_match = re.search( r"Compartment of payload (\w+): (\d+)", line )

                if latitude_match:
                    print( "Latitude match found:", latitude_match.group() )
                    latitude = float( latitude_match.group(2) )
                    self.payload_delivery_latitude.append(latitude)
                
                if longitude_match:
                    print( "Longitude match found:", longitude_match.group() )
                    longitude = float( longitude_match.group(2) )
                    self.payload_delivery_longitude.append(longitude)

                if compartment_match:
                    print( "Compartment match found:", compartment_match.group() )
                    compartment = int( compartment_match.group(2) )
                    self.payload_delivery_compartment.append(compartment)

    def deliver_payload_command(self):

        file_path = '/home/uhdt/UAV_software/Autonomous/Official/FINAL CODE/payload_coord.txt'

        while not os.path.exists( file_path ):
            print( "Waiting for file" )
            time.sleep( 1 )

        self.download_payload_coord( file_path )
        time.sleep( 5 )

        print( "Starting Payload Delivery Mission" )

        payloads_dropped = 0

        for i in range( len( self.payload_delivery_latitude ) ):

            print( f"Heading to payload #{i + 1}" )
            self.UAS_dk.simple_goto(LocationGlobalRelative( self.payload_delivery_latitude[i], self.payload_delivery_longitude[i], self.alt_AD ), groundspeed = self.DELIVER_SPEED )
            self.waypoint_reached(self.payload_delivery_latitude[i], self.payload_delivery_longitude[i], self.PAYLOAD_RADIUS)

            time.sleep( 2 )

            self.gpio_servo_command( self.payload_delivery_compartment[i], 0 )
            payloads_dropped += 1
            print( f"Payload #{i + 1} Delivered" )

            time.sleep( 10 )

            if payloads_dropped <= 3:

                print( "Performing Waypoint Lap" )
                self.dk_waypoint_lap()
                print( "Waypoint Lap completed " )

        print( "Payload Delivery Mission Completed" )

    def end_mission( self ):

        self.UAS_dk.simple_goto( LocationGlobalRelative( self.end_mission_latitude, self.end_mission_longitude, self.ALTITUDE ), groundspeed = self.WAYPOINT_SPEED )
        self.waypoint_reached( self.end_mission_latitude, self.end_mission_longitude, self.WAYPOINT_RADIUS )
    
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

        while(distance >= radius):
            if(self.RTL_stat() == True):
                print("IN RTL MODE")

                while (self.RTL_stat() == True):
                    print("IN RTL MODE")
                    time.sleep(.5)
                    pass
                self.UAS_dk.simple_goto(LocationGlobalRelative(latitude_deg,longitude_deg,self.ALTITUDE))
                self.waypoint_reached( latitude_deg, longitude_deg, radius )
                break
            #distance between 2 points retuirn value in feet    
            distance = self.haversine(latitude_deg, longitude_deg)            
            print(f"Distance to waypoint: {distance}")
            time.sleep(.5)
        print("REACHED WAYPOINT") 
        return True
    
    def response(self, keyword):
        """
        Receives a specific type of message from the UAS.

        Args:
            keyword (str): The type of message to receive.

        Returns:
            message: The received message.
        """
        message = self.UAS_mav.recv_match(type=keyword, blocking=True)
        print("-- Message Read " + str(message))
        return message

    def count(self, waypoints):
        """
        Sends a mission count command to the UAS to set the total number of commands in the mission.

        Args:
            waypoints (int): The total number of commands in the mission.
        """
        self.UAS_mav.mav.mission_count_send(
        self.UAS_mav.target_system,  #target_system
        self.UAS_mav.target_component,#target_component
        waypoints,#total number of commands in the mission
        0 #0-main mission
        )
        

    def mission_start(self):
        """
        Sends a mission start command to the UAS.

        Returns:
            bool: True if the mission start command is sent successfully.
        """
        command = mavutil.mavlink.MAV_CMD_MISSION_START
        self.UAS_mav.mav.command_long_send(
        self.UAS_mav.target_system,  #target_system
        self.UAS_mav.target_component,#target_component
        command,#command
        0,0,0,0,0,0,0,0
        )
        return True
    
    def mission_clear(self):
        """
        Sends a mission clear command to the UAS.

        Returns:
            None
        """
        self.UAS_mav.mav.mission_clear_all_send(
        self.UAS_mav.target_system,  # System ID of the vehicle
        self.UAS_mav.target_component  # Component ID
        )

    def dk_waypoint_lap( self ):
        """
        Define a sequence of waypoints using DroneKit to be followed by the UAS in a lap.

        Returns:
            str: A message indicating lap completion.

        """       
        self.UAS_dk.mode = VehicleMode("GUIDED") 

        start = time.time()

        for wp in range(len(self.waypoint_lap_latitude)):
            # self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
            print(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],self.ALTITUDE))
            self.UAS_dk.simple_goto(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],self.ALTITUDE), groundspeed = self.WAYPOINT_SPEED )
            self.waypoint_reached(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ], self.WAYPOINT_RADIUS)
        
        end = time.time()
        
        difference = end - start
        
        self.dk_waypoint_lap_time.append(difference)
        
        self.lap = self.lap + 1
        
        return print(f"DONE WITH LAP {self.lap - 1}")

    def search_area_command(self):
        """
        Define a search area waypoint. It will use the waypoint_reached() function to determine if the UAS arrived at location
        and then collect UAS attitude data, trigger camera and geotag image.
        This method defines a search area waypoint.

        :return: None
        """
        start = time.time()
        print('Now Conducting the search area')

        for x in range(len(self.search_area_latitude)):
            print(x)
            print(LocationGlobalRelative(self.search_area_latitude[x],self.search_area_longitude[x],self.alt_IP))
            self.UAS_dk.simple_goto(LocationGlobalRelative(self.search_area_latitude[x],self.search_area_longitude[x],self.alt_IP), groundspeed = self.SEARCH_SPEED )
            self.waypoint_reached(self.search_area_latitude[x],self.search_area_longitude[x], self.SEARCH_AREA_RADIUS)
            print(f"DONE WITH SEARCH AREA WAYPOINT {x}")
            #get attitide data
            p1 = multiprocessing.Process(target=self.attitude())
            #take image
            p2 = multiprocessing.Process(target=self.trigger_camera, args= (f"image{x+1}_{self.alt_IP}.jpg",))
            #start the execution and wait 
            p1.start()
            p2.start()
            p1.join()
            p2.join()
            #geotag
            self.geotag(f'image{x+1}_{self.alt_IP}.jpg')
            shutil.copy2( f'image{x+1}_{self.alt_IP}.jpg', f'/home/uhdt/UAV_software/Autonomous/watchdog/image{x+1}_{self.alt_IP}.jpg')

        end = time.time()
        difference = end - start
        self.search_area_waypoint_time.append(difference)

        return print("UAS COMPLETED SEARCH THE AREA")

    def sum(self, arr):
        """
        Calculate the sum of values in the input array.

        Args:
            arr (list): List of numeric values to be summed.

        Returns:
            float: The sum of values in the input list.
        """
        sum = 0
        for value in arr:
            sum += value
        return sum
    
    def avg(self, arr):
        """
        Calculate the average of values in the input array.

        Args:
            arr (list): List of numeric values to calculate the average from.

        Returns:
            float: The average of values in the input list.
        """
        if len(arr) == 0:
                return 0  # Avoid division by zero for an empty array

        total = sum(arr)
        average = total / len(arr)
        return average
        
    def export(self):
        """
        Export, Calculate and record the average and sum of execution times for various methods.

        Returns:
            None
        """
        #average calculation
        avg_attitude = self.avg(self.attitude_time)
        avg_geotag = self.avg(self.geotag_time)
        avg_search_area_waypoint = self.avg(self.search_area_waypoint_time)
        avg_subprocess_execute = self.avg(self.subprocess_execute_time)
        avg_trigger_camera = self.avg(self.trigger_camera_time)
        avg_waypoint_lap = self.avg(self.waypoint_lap_time)
        avg_dk_waypoint_lap = self.avg(self.dk_waypoint_lap_time)
        avg_payload_delivery_time = self.avg(self.payload_delivery_time)

        #sum calcualtion
        sum_attitude = self.sum(self.attitude_time)
        sum_geotag = self.sum(self.geotag_time)
        sum_search_area_waypoint = self.sum(self.search_area_waypoint_time)
        sum_subprocess_execute = self.sum(self.subprocess_execute_time)
        sum_trigger_camera = self.sum(self.trigger_camera_time)
        sum_waypoint_lap = self.sum(self.waypoint_lap_time)
        sum_dk_waypoint_lap = self.sum(self.dk_waypoint_lap_time)
        sum_payload_delivery_time = self.sum(self.payload_delivery_time)

            
        data_averages_and_time = [
            ("attitude", self.attitude_time, avg_attitude, sum_attitude),
            ("deliver_payload", self.deliver_payload_time, avg_payload_delivery_time,sum_payload_delivery_time ),
            ("geotag", self.geotag_time, avg_geotag,sum_geotag),
            ("search_area_waypoint", self.search_area_waypoint_time, avg_search_area_waypoint,sum_search_area_waypoint),
            ("subprocess_execute", self.subprocess_execute_time, avg_subprocess_execute,sum_subprocess_execute),
            ("trigger_camera", self.trigger_camera_time, avg_trigger_camera,sum_trigger_camera),
            ("waypoint_lap", self.waypoint_lap_time, avg_waypoint_lap,sum_waypoint_lap),
            ("dk_waypoint_lap", self.dk_waypoint_lap_time, avg_dk_waypoint_lap,sum_dk_waypoint_lap)

        ]
        with open('Data_log.txt', 'a') as file:
                for data_name, data_values, data_average, data_sum in data_averages_and_time:
                        file.write(f"{data_name}: {data_values}\n")
                        file.write(f"{data_name} average: {data_average} seconds\n")
                        file.write(f"{data_name} sum: {data_sum} seconds\n\n")



if __name__ == '__main__':
    pass
    
