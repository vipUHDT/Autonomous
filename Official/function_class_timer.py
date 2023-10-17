import time
import os
import math
import multiprocessing
import subprocess
import gphoto2 as gp
import exiftool
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

class CLASS:
    def __init__(self):
        """
        Initializes the CLASS object.

        This method initializes the class and establishes connections to UAS and the camera.

        :return: None
        """
        #connecting to UAS with dronekit
        print("Connecting to UAS")
        #self.connection_string = "/dev/ttyACM0" #usb to micro usb
        #self.vehicle = connect(self.connection_string, baud=57600, wait_ready=True)
        print("Connected with DroneKit")

        #connecting to mavlink
        print('Connecting MavLink')
        #self.UAS = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        print('Connecting to mavlink')

        #connect the camera
        camera = gp.Camera()
        camera.init
        print('Camera Connected')
        
        print('CREATING IMAGE DIRECTORY')
        image_dir = f'image_{time.ctime(time.time())}'
        print(f'MADE DIRECTORY {image_dir}')
        os.mkdir(image_dir)
        os.chdir(str(image_dir))
        print(f'MOVED TO {image_dir} DIRECTORY')
        print("CREATING TEST DATA FILE")
        with open('Data_log.txt', "a") as file:
                file.write("Time Log:\n")
        
        #file variable
        self.attitude_average = []
        self.deliver_payload_average = []
        self.geotag_average = []
        self.haversine_average = []
        self.search_area_waypoint_average = []
        self.subprocess_execute_average = []
        self.trigger_camera_average = []
        self.waypoint_lap_average = []

        #declaring variable
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.image_number = 1
        self.drone_sensory = [self.pitch, self.roll, self.yaw, self.lat, self.lon, self.alt]
        self.filename = f"image"

        self.search_area_latitude = [
            38.31455510, 38.31453830, 38.31452150, 38.31455510, 38.31453830, 38.31452150,
            38.31450570, 38.31448990, 38.31447520, 38.31445830, 38.31444260, 38.31442890,
            38.31441410, 38.31439630, 38.31438150, 38.31437100, 38.31435840, 38.31425310,
            38.31426740, 38.31428050, 38.31429730, 38.31431400, 38.31432680, 38.31434370,
            38.31436050, 38.31437420, 38.31439070, 38.31440710, 38.31442050, 38.31443710
        ]

        self.search_area_longitude = [
            -76.54514240, -76.54504980, -76.54496000, -76.54514240, -76.54504980, -76.54496000,
            -76.54486750, -76.54478030, -76.54469040, -76.54460060, -76.54451210, -76.54441950,
            -76.54432700, -76.54424120, -76.54415400, -76.54406280, -76.54396620, -76.54399840,
            -76.54408890, -76.54418350, -76.54427170, -76.54435990, -76.54445170, -76.54454130,
            -76.54463080, -76.54472400, -76.54481290, -76.54490170, -76.54499350, -76.54508310
        ]

    def trigger_camera(self):
        """
        Trigger the camera to capture an image.

        This method triggers the camera to capture an image and saves it with the provided filename.

        :param filename: The filename to use for the captured image.
        :return: None
        """
        start = time.time()
        print(f'image{self.image_number} IS BEING TAKEN')
        cmd = ('gphoto2', '--capture-image-and-download', '--filename', f'image{self.image_number}')
        #executing the trigger command in ssh
        subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f'Image{self.image_number} Captured \n')
        end = time.time()
        difference = end - start
        self.trigger_camera_average.append(difference)

        

    def attitude(self):
        """
        Retrieve attitude and GPS information.

        This method retrieves the attitude (pitch, roll, yaw) and GPS information (latitude, longitude, altitude)
        from the UAS and updates the class variables accordingly.

        :return: None
        """
        start = time.time()
        # Setting the variable with gps coordinates, yaw pitch and roll
        attitude = self.vehicle.attitude
        attitude = str(attitude)
        # Getting the UAS location in long and lat
        gps = self.vehicle.location.global_relative_frame
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
        self.attitude_average.append(difference)


        return print("Drone Sensory Data Collected")
    
    def subprocess_execute(self, command):
            start = time.time()
            subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            end = time.time()
            difference = end - start
            self.subprocess_execute_average.append(difference)

            
    def geotag(self):
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
        tag_pyr_command = ('exiftool', '-comment=' + str(pyr), self.filename + str(self.image_number))
        tag_lat_command = ('exiftool', '-exif:gpslatitude=' + '\'' + str(self.drone_sensory[3]) + '\'', self.filename + str(self.image_number))
        tag_long_command = ('exiftool', '-exif:gpslongitude=' + '\'' + str(self.drone_sensory[4]) + '\'', self.filename + str(self.image_number))
        tag_alt_command = ('exiftool', '-exif:gpsAltitude=' + '\'' + str(self.drone_sensory[5]) + '\'', self.filename + str(self.image_number))
        '''
        #executing the tag command in ssh
        subprocess.run(tag_pyr_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        subprocess.run(tag_lat_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        subprocess.run(tag_long_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        subprocess.run(tag_alt_command,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        '''        
        p1 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_pyr_command,))
        p2 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_lat_command,))
        p3 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_long_command,))
        p4 = multiprocessing.Process(target = self.subprocess_execute, args = (tag_alt_command,))
        

        p1.start()
        p2.start()
        p3.start()
        p4.start()
        
        p1.join()
        p2.join()
        p3.join()
        p4.join()

        self.image_number += 1
        end = time.time()
        difference = end - start

        self.geotag_average.append(difference)
        return print(f"{self.filename + str(self.image_number-1)} geotagged")

    def curr_waypoint_number(self):
        """
        Get the current waypoint number.

        This method retrieves the current waypoint number from the UAS.

        :return: The current waypoint number.
        """
        return UAS.command.next - 1

    def next_waypoint_number(self):
        """
        Get the next waypoint number.
        
        This method retrieves the next waypoint number from the UAS.

        :return: The next waypoint number.
        """
        return UAS.command.next

    #convert from degree to radian
    def toRadian(self, degree):
        pi = math.pi
        return degree * (pi / 180)

    #using haversine formula to calculate distance between two coordinates
    def haversine(self, lon1, lat1):
        start = time.time()
        curr_location = UAS.location.global_relative_frame
        lat1 = toRadian(lat1)
        lon1 = toRadian(lon1)
        lat2 = toRadian(curr_location.latitude)
        lon2 = toRadian(curr_location.longitude)

        diff_lat = lat2 - lat1
        diff_lon = lon2 - lon1

        end = time.time()
        difference = end - start

        self.haversine_average.append(difference)
        # feet conversion * earth radius * something
        return 5280 * 3963.0 * math.acos( (math.sin(lat1)*math.sin(lat2)) + (math.cos(lat1) * math.cos(lat2)) * math.cos(lon2 - lon1) )

    def waypoint_reached (self, atitude_deg, longitude_deg):

        #distance between 2 points retuirn value in feet    
        distance = haversine(latitude_deg,longitude_deg)

        #checking is UAS reached within 15 feet in diameter of the desired coordinate desitination
        while(distance > 7.5):
            #distance between 2 points retuirn value in feet    
            distance = self.haversine(latitude_deg,longitude_deg)            
            print("HAS NOT REACHED WAYPOINT YET")
            time.sleep(.5)

        return print("REACHED WAYPOINT")

    def waypoint_lap(self, waypoint_array):
        """
        Simulate a waypoint lap.

        This method simulates a waypoint lap by executing a loop 10 times.

        :return: None
        """
        start = time.time()
        for x in range(10):
            print("NOT IMPLEMENTED")
            
        end = time.time()
        difference = end - start

        self.waypoint_lap_average.append(difference)


    def deliver_payload(self, servo_x, longitude, latitude):
        """
        Deliver payload to a target.

        This method activates servo x to to deliver payload

        :param servo_x: The servo number to use for payload delivery.
        :return: None
        """
        start = time.time()

        
        #ENTER CODE HERE
        
        end = time.time()
        difference = end - start
        with open('Data_log.txt', 'a') as file:
            file.write(f'deliver_payload {servo_x}: {difference:.4f} seconds\n')

        return print("NOT IMPLEMENTED")

    def search_area_waypoint(self):
        """
        Define a search area waypoint.

        This method defines a search area waypoint.

        :return: None
        """
        start = time.time()
        for x in range(len(search_area_latitude)):
            #go to wp
            print(f"GOING TO SEARCH AREA WAYPOINT: {x}") 
            location = LocationGlobal(self.search_area_latitude[x],self.search_area_longitude[x],alt)
            #call the waypoint reached
            self.waypoint_reached(self.search_area_latitude[x],self.search_area_longitude[x])
            #get attitide data
            p1 = multiprocessing.Process(target=self.attitude())
            #take image
            p2 = multiprocessing.Process(targert=self.trigger_camera(f"IMAGE{x}.jpg"))
            #start the execution and wait 
            p1.start()
            p2.start()
            p1.join()
            p2.join()
            #geotag
            p3 = multiprocessing.Process(target = self.geotag(f"IMAGE{x}.jpg", drone_sensory))
            p3.start()
        end = time.time()
        difference = end - start

        self.deliver_payload_average.append(difference)

        return print("UAS COMPLETED SEARCH THE AREA")

    def avg(self, arr):
        if len(arr) == 0:
                return 0  # Avoid division by zero for an empty array

        total = sum(arr)
        average = total / len(arr)
        return average
        
    def average(self):
            
        avg_attitude = self.avg(self.attitude_average)
        avg_deliver_payload = self.avg(self.deliver_payload_average)
        avg_geotag = self.avg(self.geotag_average)
        avg_haversine = self.avg(self.haversine_average)
        avg_search_area_waypoint = self.avg(self.search_area_waypoint_average)
        avg_subprocess_execute = self.avg(self.subprocess_execute_average)
        avg_trigger_camera = self.avg(self.trigger_camera_average)
        avg_waypoint_lap = self.avg(self.waypoint_lap_average)
            
        data_averages = [
            ("attitude", self.attitude_average, avg_attitude),
            ("deliver_payload", self.deliver_payload_average, avg_deliver_payload),
            ("geotag", self.geotag_average, avg_geotag),
            ("haversine", self.haversine_average, avg_haversine),
            ("search_area_waypoint", self.search_area_waypoint_average, avg_search_area_waypoint),
            ("subprocess_execute", self.subprocess_execute_average, avg_subprocess_execute),
            ("trigger_camera", self.trigger_camera_average, avg_trigger_camera),
            ("waypoint_lap", self.waypoint_lap_average, avg_waypoint_lap)
        ]
        with open('Data_log.txt', 'a') as file:
                for data_name, data_values, data_average in data_averages:
                        file.write(f"{data_name}: {data_values}\n")
                        file.write(f"{data_name} average: {data_average} seconds\n")
                        
    def RTL_checker():
            
        return print("Not yet implemented")

        


if __name__ == '__main__':
    pass
    
