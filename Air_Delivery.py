def deliver_payload(self,servo_x,latitude,longitude):
    
    PWM = 1500
    Target = LocationGlobalRelative(latitude,longitude,alt)
    self.simple_goto(Target)
    waypoint_reached(latitude,longitude)
    self.mode = VehicleMode('Loiter')
    CMD = self.message_factory.MAV_CMD_DO_SET_SERVO(servo_x,PWM)
    self.send_mavlink(CMD)
    time.sleep(20)
    self.mode = VehicleMode('Auto')