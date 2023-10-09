import time
import os
import subprocess
import gphoto2 as gp
import exiftool
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

class CLASS:
    #declaring variable
    global pitch, roll, yaw, lat, lon, alt, image_number, drone_senory 
    drone_senory = [pitch, roll, yaw, lat, lon, alt]
    image_number = 1
    filename = f"image{image_number}"
    

    def WAYPOINT_LAP():
        for x in range(10):
            print("NOT IMPLEMENTED")

    def SEARCH_AREA_WAYPOINT():
        print("NOT IMPLEMENTED")

    def TRIGGER_CAMERA(filename):
        print(filename)
        cmd = ('gphoto2','--capture-image-and-download','--filename',filename)
        #executing the trigger command in ssh
        result2 = subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        image_number = image_number+1
        return print('Image Captured \n')

    #sets the attidue and gps coordinate to variables
    #
    #@return pitch 
    #@return roll
    #@return yaw
    #@return lat
    #@return lon
    #@return alt
    def attitude():
        #global pitch, roll, yaw, lat, lon, alt
        #Setting the variable  with gps coordinates, yaw pitch and roll
        attitude = vehicle.attitude
        attitude=str(attitude)
        #Getting the UAS location in long and lat
        gps = vehicle.location.global_relative_frame
        gps = str(gps)
        #using split method to split string so we can get individual value of yaw,pitch and roll
        attitudeSplit = attitude.split(",")
        pitchSplit = attitudeSplit[0].split("=")
        #The pitch value
        pitch = pitchSplit[1]
        yawSplit = attitudeSplit[1].split("=")
        #yaw value
        yaw = yawSplit[1]
        rollSplit = attitudeSplit[2].split("=")
        #roll value
        roll = rollSplit[1]
        #splitting the string so we can get the value of longitude and latitude
        gpsSplit = gps.split(",")
        latSplit = gpsSplit[0].split("=")
        #value of the lat
        lat = latSplit[1]
        lonSplit = gpsSplit[1].split("=")
        #value of the long
        lon = lonSplit[1]
        altSplit = gpsSplit[2].split("=")
        #altitude value
        alt = altSplit[1]
        #Send inputs as a string not int
        pitch=str(pitch)
        roll=str(roll)
        yaw=str(yaw)
        lat=str(lat)
        lon=str(lon)
        alt=str(alt)
        drone_senory = [pitch, roll, yaw, lat, lon, alt]
        return print("Drone Sensory Data Collected")

    #geotagging image with drone sensory data
    def GEOTAG(filename, drone_senory):
        #Geotagging photo with the attitude and gps coordinate
        pyr = ('pitch:'+str(drone_senory[0])+' yaw:'+str(drone_senory[2])+' roll:'+str(drone_senory[1]))
        print(pyr)
        tagPYRCommand = ('exiftool', '-comment=' + str(pyr) , filename)
        tagLatCommand = ('exiftool', '-exif:gpslatitude=' +'\''+ str(drone_senory[4]) +'\'' , filename)
        tagLongCommand = ('exiftool', '-exif:gpslongitude=' +'\''+ str(drone_senory[5]) +'\'' , filename)
        tagAltCommand = ('exiftool', '-exif:gpsAltitude=' +'\''+ str(drone_senory[6]) +'\'' , filename)
    
    #deliver the payload to one target
    def DELIVER_PAYLOAD(SERVO_X):
        print("NOT IMPLEMENTED")

    def WAYPOINT_NUMBER():
        print("NOT IMPLEMENTED")



