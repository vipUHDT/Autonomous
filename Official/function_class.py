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
        self.connection_string = "/dev/ttyACM0" #usb to micro usb
        self.vehicle = connect(self.connection_string, baud=57600, wait_ready=True)
        print("Connected with DroneKit")

        #connecting to mavlink
        print('Connecting MavLink')
        self.UAS = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        print('Connecting to mavlink')

        #connect the camera
        self.camera = gp.Camera()
        self.camera.init()
        print('Camera Connected')

        #declaring variable
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.image_number = 1
        self.drone_sensory = [self.pitch, self.roll, self.yaw, self.lat, self.lon, self.alt]
        self.filename = f"image{self.image_number}"

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

    def trigger_camera(self,filename):
        """
        Trigger the camera to capture an image.

        This method triggers the camera to capture an image and saves it with the provided filename.

        :param filename: The filename to use for the captured image.
        :return: None
        """
        print(filename)
        cmd = ('gphoto2', '--capture-image-and-download', '--filename', filename)
        #executing the trigger command in ssh
        result2 = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.image_number += 1
        return print('Image Captured \n')

    def attitude(self):
        """
        Retrieve attitude and GPS information.

        This method retrieves the attitude (pitch, roll, yaw) and GPS information (latitude, longitude, altitude)
        from the UAS and updates the class variables accordingly.

        :return: None
        """
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
        return print("Drone Sensory Data Collected")

    def geotag(filename, drone_sensory):
        """
        Geotag an image with sensory data.

        This method geotags a photo with attitude (pitch, roll, yaw) and GPS (latitude, longitude, altitude)
        information.

        :param filename: The filename of the image to geotag.
        :param drone_sensory: The drone sensory data.
        :return: None
        """
        # Geotagging photo with the attitude and GPS coordinate
        pyr = ('pitch:' + str(self.drone_sensory[0]) + ' yaw:' + str(self.drone_sensory[2]) + ' roll:' + str(self.drone_sensory[1]))
        print(pyr)
        tag_pyr_command = ('exiftool', '-comment=' + str(pyr), filename)
        tag_lat_command = ('exiftool', '-exif:gpslatitude=' + '\'' + str(self.drone_sensory[4]) + '\'', filename)
        tag_long_command = ('exiftool', '-exif:gpslongitude=' + '\'' + str(self.drone_sensory[5]) + '\'', filename)
        tag_alt_command = ('exiftool', '-exif:gpsAltitude=' + '\'' + str(self.drone_sensory[6]) + '\'', filename)
        return print("image geotagged")

        #executing the tag command in ssh
        p1 = multiprocessing.process(target = subprocess.run(tagPYRCommand,stdout=subprocess.PIPE,stderr=subprocess.PIPE))
        p2 = multiprocessing.process(target = subprocess.run(tagLatCommand,stdout=subprocess.PIPE,stderr=subprocess.PIPE))
        p3 = multiprocessing.process(target = subprocess.run(tagLongCommand,stdout=subprocess.PIPE,stderr=subprocess.PIPE))
        p4 = multiprocessing.process(target = subprocess.run(tagAltCommand,stdout=subprocess.PIPE,stderr=subprocess.PIPE))
        

        p1.start()
        p2.start()
        p3.start()
        p4.start()
        
        p1.join()
        p2.join()
        p3.join()
        p4.join()
        print("Geotagging image is finished\n")

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
        curr_location = UAS.location.global_relative_frame
        lat1 = toRadian(lat1)
        lon1 = toRadian(lon1)
        lat2 = toRadian(curr_location.latitude)
        lon2 = toRadian(curr_location.longitude)

        diff_lat = lat2 - lat1
        diff_lon = lon2 - lon1
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
        for x in range(10):
            print("NOT IMPLEMENTED")

    def deliver_payload(self, servo_x, longitude, latitude):
        """
        Deliver payload to a target.

        This method activates servo x to to deliver payload

        :param servo_x: The servo number to use for payload delivery.
        :return: None
        """
        return print("NOT IMPLEMENTED")

    def search_area_waypoint(self):
        """
        Define a search area waypoint.

        This method defines a search area waypoint.

        :return: None
        """
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

        return print("UAS COMPLETED SEARCH THE AREA")

if __name__ == '__main__':
    pass
    