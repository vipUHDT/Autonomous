import time
import os
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

    def waypoint_lap(self, waypoint_array):
        """
        Simulate a waypoint lap.

        This method simulates a waypoint lap by executing a loop 10 times.

        :return: None
        """
        for x in range(10):
            print("NOT IMPLEMENTED")

    def search_area_waypoint(self):
        """
        Define a search area waypoint.

        This method defines a search area waypoint.

        :return: None
        """
        print("NOT IMPLEMENTED")

    def trigger_camera(self, filename):
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

    def geotag(self, filename, drone_sensory):
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

    def deliver_payload(self, servo_x, longitude, latitude):
        """
        Deliver payload to a target.

        This method activates servo x to to deliver payload

        :param servo_x: The servo number to use for payload delivery.
        :return: None
        """
        return print("NOT IMPLEMENTED")

    def curr_waypoint_number(self):
        """
        Get the current waypoint number.

        This method retrieves the current waypoint number from the UAS.

        :return: The current waypoint number.
        """
        return self.vehicle.command.next - 1

    def next_waypoint_number(self):
        """
        Get the next waypoint number.
        
        This method retrieves the next waypoint number from the UAS.

        :return: The next waypoint number.
        """
        return self.vehicle.command.next