import autonomous
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import asyncio
fc = autonomous.CLASS()
fc.deliver_payload_command()
