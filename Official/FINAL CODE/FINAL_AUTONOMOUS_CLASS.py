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
        self.ALTITUDE = 25.0 #meters
        self.WAYPOINT_RADIUS = 3 #feet
        self.PAYLOAD_RADIUS = 2 #feet
        self.SEARCH_AREA_RADIUS = 2 #feet
        #connecting to UAS with dronekit
        print("Connecting to UAS")
        self.connection_string = 'udp:127.0.0.1:14551' #Software in the loop
        # self.connection_string = "/dev/ttyACM0" #usb to micro usb

        #connecting to mavlink
        print('Connecting MavLink')
        self.UAS_mav = mavutil.mavlink_connection(self.connection_string, baud=57600)
        self.UAS_mav.wait_heartbeat()
        print("hearbeat from system {system %u compenent %u}" %(self.UAS_mav.target_system, self.UAS_mav.target_component))
        print("Mavlink Connected ")

        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True, heartbeat_timeout= 120)
        print("Connected with DroneKit")
        
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
        self.dk_waypoint_lap_time = []
        self.payload_delivery_time = []


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
        self.lap = 1
        self.payload = 1
        self.filename = f"image"
        self.waypoint_lap_latitude = [
            21.4008762
            # 21.4009349
        ]
        self.waypoint_lap_longitude = [
            -157.7647729
            # -157.764608
        ]

        self.search_area_latitude = [
            21.4007988
            # 21.4008375
        ]

        self.search_area_longitude = [
            -157.7647327
            # -157.764811
        ]

        # self.user_input()
        
        print("AUTONOMOUS SCRIPT IS READY")
        # while (self.IS_ARMED() != True):
        #     print("waiting to be armed")
        #     print(self.UAS_dk.armed)
        #     time.sleep(1)
        # print("UAS IS NOW ARMED")
        while (self.IS_GUIDED()  != True):
            print("waiting to be in GUIDED mode")
            print(self.UAS_dk.mode)
            time.sleep(1)
        print("UAS IS NOW IN GUIDED MODE")
        print("!------------------ MISSION STARTING ----------------------!")
        
    
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
        self.lat = lat_split[1]
        lon_split = gps_split[1].split("=")
        # value of the long
        self.lon = lon_split[1]
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
        p2.start()
        p3.start()
        p4.start()
        
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
        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
        if self.UAS_dk.armed == True:
            return True
        return False

    def IS_AUTO(self):
        """
        Check if the UAS is in "AUTO" mode.

        Returns:
            bool: True if the UAS is in AUTO, False otherwise.
        """    
        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)    
        if self.UAS_dk.mode == "AUTO":
            return True
        return False
    
    def IS_GUIDED(self):
        """
        Check if the UAS is in "AUTO" mode.

        Returns:
            bool: True if the UAS is in AUTO, False otherwise.
        """        
        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
        if self.UAS_dk.mode == "GUIDED":
            return True
        return False


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

    def servo_command(self, servo_x, seq):
        """
        Define a waypoint command.

        Args:
            latitude (float): The latitude coordinate.
            longitude (float): The longitude coordinate.
            seq (int): The number sequence according to mission.

        Returns:
            None
        """ 

        command = dialect.MAV_CMD_DO_SET_SERVO

        message = dialect.MAVLink_mission_item_int_message(
            self.UAS_mav.target_system,  #target_system
            self.UAS_mav.target_component, #target_component
            seq,
            dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command, #MAV_CMD_NAV_WAYPOINT (16) or try to change it to  waypoint_command
            0,
            1, #auto continue 
            0, #hold (s)
            servo_x,1200,0,0,0,0,0
            )
        # Send the message
        self.UAS_mav.mav.send(message)

        
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
      
    def deliver_payload_command(self):
        """
        Activate a servo to deliver payload (not implemented).

        Args:
            servo_x (int): The servo number to be activated.

        Returns:
            None
        """
        #connecting to mavlink
        print('Connecting MavLink')
        self.UAS_mav = mavutil.mavlink_connection(self.connection_string, baud=57600)
        self.UAS_mav.wait_heartbeat()
        print("hearbeat from system {system %u compenent %u}" %(self.UAS_mav.target_system, self.UAS_mav.target_component))
        print("Mavlink Connected ")
        # Create a waypoint command
        start = time.time()
        print(f"UAS HEADING TO DROP PAYLOAD")
        self.count(len(self.waypoint_lap_latitude)*3+1)
        self.waypoint_command(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ],0)
        sequence = 1
        counter = 0

        while(counter < len(self.waypoint_lap_latitude)):
            self.waypoint_command(self.waypoint_lap_latitude[ counter ], self.waypoint_lap_longitude[ counter ],sequence)
            sequence = sequence+1 
            print(sequence)      
            self.servo_command(5,sequence)
            sequence = sequence+1
            print(sequence)
            self.loiter_command(15,sequence)
            sequence = sequence+1  
            print(sequence)
            counter = counter + 1
       
        self.mission_start()
        self.response("MISSION_ACK")
        # for reached in range(len(self.waypoint_lap_latitude)*3):
        #     self.response("MISSION_ITEM_REACHED")
        self.payload = self.payload + 1
        end = time.time()
        difference = end - start
        self.payload_delivery_time.append(difference)
        return print(f"PAYLOADS  DELIVERED")

    
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


    def spline_waypoint_lap( self ):
        """
        Define a sequence of spline waypoints to be followed by the UAS in a lap.

        Returns:
            str: A message indicating lap completion.

        """
        self.count(len(self.waypoint_lap_latitude)+1)
        self.spline_waypoint_command(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ],0)
        start = time.time()
        for wp in range(len(self.waypoint_lap_latitude)):
            self.spline_waypoint_command(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],wp+1)
            # self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
            # print(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],75))
            # self.UAS_dk.simple_goto(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],75))
            # self.waypoint_reached(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ], self.WAYPOINT_RADIUS)
        self.mission_start()
        self.response("MISSION_ACK")
        for reached in range(len(self.waypoint_lap_latitude)):
            self.response("MISSION_ITEM_REACHED")
        end = time.time()
        difference = end - start
        self.waypoint_lap_time.append(difference)
        self.lap = self.lap + 1
        return print(f"DONE WITH LAP {self.lap - 1}")

    def waypoint_lap( self ):
        """
        Define a sequence of waypoints to be followed by the UAS in a lap.

        Returns:
            str: A message indicating lap completion.

        """        
        self.count(len(self.waypoint_lap_latitude)+1)
        self.waypoint_command(self.waypoint_lap_latitude[ 0 ], self.waypoint_lap_longitude[ 0 ],0)
        start = time.time()
        for wp in range(len(self.waypoint_lap_latitude)):
            self.waypoint_command(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],wp+1)
            # self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
            # print(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],75))
            # self.UAS_dk.simple_goto(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],75))
            # self.waypoint_reached(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ], self.WAYPOINT_RADIUS)
        self.mission_start()
        self.response("MISSION_ACK")
        for reached in range(len(self.waypoint_lap_latitude)):
            self.response("MISSION_ITEM_REACHED")
        end = time.time()
        difference = end - start
        self.waypoint_lap_time.append(difference)
        self.lap = self.lap + 1
        return print(f"DONE WITH LAP {self.lap - 1}")
    
    def dk_waypoint_lap( self ):
        """
        Define a sequence of waypoints using DroneKit to be followed by the UAS in a lap.

        Returns:
            str: A message indicating lap completion.

        """       
        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
        self.UAS_dk.mode = VehicleMode("GUIDED") 
        start = time.time()
        for wp in range(len(self.waypoint_lap_latitude)):
            self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
            print(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],self.ALTITUDE))
            self.UAS_dk.simple_goto(LocationGlobalRelative(self.waypoint_lap_latitude[ wp ], self.waypoint_lap_longitude[ wp ],self.ALTITUDE))
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
        self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
        self.UAS_dk.mode = VehicleMode("GUIDED")

        for x in range(len(self.search_area_latitude)):
            self.UAS_dk = connect(self.connection_string, baud=57600, wait_ready=True)
            print(x)
            print(LocationGlobalRelative(self.search_area_latitude[x],self.search_area_longitude[x],self.ALTITUDE))
            self.UAS_dk.simple_goto(LocationGlobalRelative(self.search_area_latitude[x],self.search_area_longitude[x],self.ALTITUDE))
            self.waypoint_reached(self.search_area_latitude[x],self.search_area_longitude[x], self.WAYPOINT_RADIUS)
            print(f"DONE WITH SEARCH AREA WAYPOINT {x}")

            #get attitide data
            p1 = multiprocessing.Process(target=self.attitude())
            #take image
            p2 = multiprocessing.Process(target=self.trigger_camera, args= (f"image{x+1}.jpg",))
            #start the execution and wait 
            p1.start()
            p2.start()
            p1.join()
            p2.join()
            #geotag
            self.geotag(f'image{x+1}.jpg')

        end = time.time()
        difference = end - start
        self.search_area_waypoint_time.append(difference)

        return print("UAS COMPLETED SEARCH THE AREA")
    
    def user_input(self):
        """
        Allow the user to input a set of latitude and longitude coordinates for waypoints.

        Returns:
            None
        """
        # Ask for the number of coordinates and create a latitude and longitude array
        while 1:
            # Check for non-integer value
            try:
                number_of_coordinates = int(input("\nHow many coordinates?\n"))
                break
            except ValueError:
                print("Enter an integer")

        self.waypoint_lap_latitude  = array('f', [0] * number_of_coordinates)
        self.waypoint_lap_longitude = array('f', [0] * number_of_coordinates)

        # Ask for longitude and latitude coordinates and put them in their respective arrays
        for i in range(number_of_coordinates):
            while 1:
                # Check for non-integer values
                try:
                    self.waypoint_lap_latitude[i] = float(input(f"Enter latitude {i + 1}:\n"))
                    break
                except FloatingPointError:
                    print("Coordinate must be an integer")

            while 1:
                # Check for non-integer values
                try:
                    self.waypoint_lap_longitude[i] = float(input(f"Enter longitude {i + 1}:\n"))
                    break 
                except FloatingPointError:
                    print("Coordinate must be an integer")

        # Print the coordinates in the array
        print("\nLatitudes entered:")
        for i in range(number_of_coordinates):
            if (i == number_of_coordinates-1):
                print(self.waypoint_lap_latitude [i])
            else:
                print(self.waypoint_lap_latitude [i], end=", ")

                while True:
                    try:
                        response = int(input("\nIS THE VALUE OF LATITUDE AND LONGITUDE CORRECT?\n1-YES or 2-NO\n"))
                        if response in [1, 2]:
                            if (response ==2):
                                self.user_input()
                            else:
                                break
                        else:
                            raise ValueError("\nInvalid response. Please enter 1-YES or 2-NO.")

                    except ValueError as e:
                        print(e)
            

        print("\nLongitudes entered:")
        for i in range(number_of_coordinates):
            if (i == number_of_coordinates-1):
                print(self.waypoint_lap_longitude[i])
            else:
                print(self.waypoint_lap_longitude[i], end=", ") 

        #------------------------------------------------------------#
        # Display parameters to the user and give option to change the parameters

        while 1:
            print(f"\nSET PARAMETERS ARE:\n")
            print(f"ALTITUDE: {self.ALTITUDE}")
            print(f"WAYPOINT_RADIUS: {self.WAYPOINT_RADIUS}")
            print(f"PAYLOAD_RADIUS: {self.PAYLOAD_RADIUS}")
            print(f"SEARCH_AREA_RADIUS: {self.SEARCH_AREA_RADIUS}")

            try:
                # Ask user if the parameters are ok
                response = int(input("\nARE THESE PARAMETERS OK?\n1-YES or 2-NO\n"))
                if (response in [1, 2]):
                    if (response == 1):
                        break # Parameters are ok
                    if (response == 2):

                        # Ask user to enter new altitude
                        while 1:
                            try:
                                self.ALTITUDE = float(input("\nENTER NEW ALTITUDE\n"))
                                break
                            except ValueError:
                                print("\nInvalid Response. Please enter a number.\n")
                    
                        # Ask user to enter new waypoint_radius
                        while 1:
                            try:
                                self.WAYPOINT_RADIUS = float(input("\nENTER NEW WAYPOINT_RADIUS\n"))
                                break
                            except ValueError:
                                print("\nInvalid Response. Please enter a number.\n")

                        # Ask user to enter new payload_radius
                        while 1:
                            try:
                                self.PAYLOAD_RADIUS = float(input("\nENTER NEW PAYLOAD_RADIUS\n"))
                                break
                            except ValueError:
                                print("\nInvalid Response. Please enter a number.\n")

                        # Ask user to enter new search_area_radius
                        while 1:
                            try:
                                self.SEARCH_AREA_RADIUS = float(input("\nENTER NEW SEARCH_AREA_RADIUS\n"))
                                break
                            except ValueError:
                                print("\nInvalid Response. Please enter a number.\n")

                else:
                    raise ValueError("\nInvalid Response. Please enter 1-YES or 2-NO.\n") # response not 1 or 2

            except ValueError:
                print("\nInvalid Response. Please enter 1-YES or 2-NO.\n") # invalid response

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

    def KAMIKAZE():
        """
        Kills the UAS by completely cutting power to drone

        Returns:
            None
        """
        #send signal to relay to kill drone
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⠤⠖⠛⣷⣶⣶⠿⢿⣿⣿⣶⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡴⣾⣿⠋⠀⠀⠀⢾⣿⠏⠀⠀⠀⠀⠈⠛⠻⣿⣦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⡾⠋⠀⠙⠛⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⢻⣷⣶⣦⣤⡀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⢀⣠⣤⣤⣠⣶⣿⡅⠀⠀⠀⣤⣤⣴⣶⠗⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⠆⠀⠹⣷⣤⡀⠀⠀⠀⠀⠀")
        print("⠀⠀⣰⡿⠋⠉⢹⣿⠁⠉⠀⠀⠀⠀⠀⣿⠏⠀⠀⠀⠀⢀⣠⣤⠀⠀⠀⠀⠲⣤⣄⣀⣀⠀⠀⠀⠙⢻⣷⣦⡀⠀⠀")
        print("⠀⣰⣿⠇⠀⠀⠸⠿⠂⠀⠀⠀⠀⠀⠀⠟⠀⠀⠀⠀⠠⣿⡏⠁⠀⠀⠀⠀⠀⠈⢿⠁⠀⠀⠀⠀⠀⠸⣿⠉⢿⣆⠀")
        print("⢰⣿⡁⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⣦⡀⠛⢿⣦⠀⠀⡀⠀⠀⠀⠘⠀⠀⠀⠀⠀⠀⠀⠛⢀⠀⣿⡄")
        print("⢸⡿⠁⠀⠀⢿⣄⠀⡀⠀⠀⠀⠀⢀⣤⣀⣀⣀⣤⣿⣷⣶⡿⠻⣿⣿⠿⣷⣦⣄⣸⣷⠀⠀⠀⢠⣄⠀⠀⠈⠛⠿⣿")
        print("⠸⣿⣾⠃⠀⠀⠛⠿⠃⠀⠀⠰⣤⣴⣿⠿⠿⠿⠛⠉⠉⠀⠀⡄⠀⢰⣿⠟⢿⣿⣿⣄⡀⢰⣿⡟⠀⠀⠀⠀⠀⢹⡇")
        print("⠀⢸⣿⢸⡆⠀⠶⠿⠀⣀⡀⠠⣬⣭⣭⣀⠀⣆⠀⠀⢠⠀⠀⣰⠃⣰⣿⣿⠏⠀⠀⣉⣿⣿⠀⠙⠷⠀⠀⢠⣶⡀⣸⣧")
        print("⠀⠸⣿⣿⣷⡄⠀⠀⠘⠋⠁⠀⠀⠀⠉⡛⢷⣽⣦⠀⢸⡄⠀⣿⢀⣿⣿⣿⡶⠒⠛⢋⡅⠀⠀⠀⢀⣴⢀⡼⠟⠛⠟⠁")
        print("⠀⠀⠈⠙⠻⣷⣶⣦⣴⡾⠛⠶⠦⣶⠾⢿⣶⣬⣿⡆⠸⣧⢠⡇⢸⣿⣣⣥⣄⣀⣈⣤⣶⣦⣴⣿⠟⠋⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⣽⣿⠀⢿⠈⣧⣼⢹⣿⣀⣀⠉⠛⠛⠉⠀⠈⠉⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⡶⢿⡛⢹⣿⡄⠸⠇⣿⡇⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣶⡿⢛⣉⣣⣾⣿⠛⣿⣷⠀⡀⢸⣇⢸⡟⠛⣿⡇⢠⡟⢿⣷⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣅⠀⠀⡉⠛⢿⣅⣀⣿⣿⠀⣿⣄⣉⣹⣧⡴⢾⣯⡀⠈⠋⠉⢻⣿⡀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣏⣀⣾⣏⠀⠀⠈⠙⠿⠿⠋⢻⡟⠋⠛⢿⡀⠀⠸⠀⠀⣀⠀⠀⣽⣿⡆⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠿⠿⢿⣷⣀⣀⣼⣦⠀⠀⠀⢁⣀⣠⣶⡀⠀⣀⣀⣀⣼⣤⣾⣿⡿⠇⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠛⠿⠿⣿⣷⣶⣿⠿⠟⣿⠛⠿⣶⣿⣿⠿⠋⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⡇⠀⣰⠃⣿⡇⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⢻⣇⠀⣿⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠘⣿⠀⣿⠀⢿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⢿⣿⠀⣿⠀⣿⡆⠸⣿⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣯⣾⡟⠀⠁⠀⢿⣇⠀⢻⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⠋⣻⣷⠿⠋⠀⡀⠀⠀⠸⢿⡄⠀⢻⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠖⠛⠁⠀⠀⣰⠇⠀⠀⠀⠀⠀⠀⠀⠈⠙⠓⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")
        print("⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀")



        return print("UAS TERMINATED")



if __name__ == '__main__':
    pass
    
