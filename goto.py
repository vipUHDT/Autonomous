from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

longitude = [0,0,0,0]
latitude = [0,0,0,0]
alt = 75


#usb(Pi) to micro usb (Pixhawk)
connection_string = "/dev/ttyACM0"
#baud rate
baud_rate = 921600

#connecting to UAS
print("Connecting to UAS")
vehicle = connect(connection_string, baud=baud_rate, wait_ready = True)
print("Connected")

#checking if UAS is armed. it wont move leave while loop till drone is armed
while not vehicle.armed:
    print("DRONE IS NOT ARMED")
    time.sleep(1)
print("DRONE IS ARMED")

VehicleModE("GUIDED")
#checking the UAS mode wp start the script

print(f"GOING TO WAYPOINT: {0}") 
location = LocationGlobal(longitude[0],latitude[0],alt)
vehicle.simple_goto(location)
print(f"GOING TO WAYPOINT: {1}") 
location = LocationGlobal(longitude[1],latitude[1],alt)
vehicle.simple_goto(location)
print(f"GOING TO WAYPOINT: {2}") 
location = LocationGlobal(longitude[2],latitude[2],alt)
vehicle.simple_goto(location)
print(f"GOING TO WAYPOINT: {3}") 
location = LocationGlobal(longitude[3],latitude[3],alt)
vehicle.simple_goto(location)



vehicle.close()
