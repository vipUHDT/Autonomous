import time
import os
import subprocess
import gphoto2
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal, Command


longitude = [0,0,0]
latitude = [0,0,0]
alt = 20


#usb(Pi) to micro usb (Pixhawk)
connection_string = "/dev/ttyACM0"
#baud rate
baud_rate = 921600

#connecting to UAS
print("Connecting to UAS")
vehicle = connect(connection_string, baud=baud_rate, wait_ready = True)
print("Connected")

#connecting to camera
print("Connecting to Camera")
camera = gp.Camera()
camera.init
print("Camera Connected")

#Trigger the camera function
def triggerCommand(filename):
    print(filename)
    cmd = ('gphoto2','--capture-image-and-download','--filename',filename)
    #executing the trigger command in ssh
    result2 = subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    print('Image Captured \n')

def WP(long, lat, alt):
    vehicle.simple_goto(long,lat,alt)

def searchArea():
    print("search the area")
    # the 2d array contains the waypoints in the search area
    #long_and_alt = [[]]
    #searchAreaWpSize = 0
    #for loop that loops the 2d array 
        #call WP function with long_and_alt parameter

#The waypoint lap function
def waypoint_Nav(waypoints):
    print("waypoint lap")
    # for loop that loops waypoints array
        #call WP function to execute the waypoint lap
        

#checking if UAS is armed. it wont move leave while loop till drone is armed
while not vehicle.armed:
    print("DRONE IS NOT ARMED")
    time.sleep(1)
print("DRONE IS ARMED")

#checking the UAS mode tp start the script
if (vehicle.mode == "AUTO"):
    for x in range(2):
        vehicle.simple_goto(longitude[x],latitude[x],alt)
        triggerCommand("image"+str(x))


vehicle.close()