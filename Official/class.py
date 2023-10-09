import time
import os
import subprocess
import gphoto2 as gp
import exiftool
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

class CLASS:
    def __init__(self, connection_string, baud_rate):
        # Initialize variables here
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.pitch = None
        self.roll = None
        self.yaw = None
        self.lat = None
        self.lon = None
        self.alt = None
        self.image_number = 1
        self.filename = f"image{self.image_number}"

        # Connect to UAS and camera
        self.connect_to_uas()
        self.connect_camera()

    def connect_to_uas(self):
        print("Connecting to UAS")
        self.vehicle = connect(self.connection_string, baud=self.baud_rate, wait_ready=True)
        print("Connected")

    def connect_camera(self):
        self.camera = gp.Camera()
        self.camera.init()
        print('Camera Connected')

    def trigger_camera(self):
        cmd = ('gphoto2', '--capture-image-and-download', '--filename', self.filename)
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.image_number += 1
        return print('Image Captured \n')

    # Other methods...

if __name__ == "__main__":
    connection_string = "/dev/ttyACM0"
    baud_rate = 57600

    my_instance = CLASS(connection_string, baud_rate)
    my_instance.trigger_camera()
