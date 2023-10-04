from pymavlink import mavutil

UAS = mavutil.mavlink_connection('COM7', baud=57600)
print("connected")

#waiting for heartbeat before sending commands
x = UAS.wait_heartbeat()
print(x)