#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative , LocationGlobal
from pymavlink import mavutil # Needed for command message definitions
import rospy
from std_msgs.msg import Int32
from detection.msg import coordinate 
from sensor_msgs.msg import NavSatFix
jackal_location = NavSatFix()
RS_pose = coordinate()

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True) 

# Connect to the Vehicle (in this case a simulator running the same computer)
""" vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True) """

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)




def callback_rspose(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard \n d- %d \n x- %f \n y- %f \n z- %f', data.detection, data.x, data.y, data.z)
    global RS_pose
    RS_pose = data
    """ print("stored d-  ", RS_pose.detection)
    print("stored x-  ", RS_pose.x)
    print("stored y-  ", RS_pose.y)
    print("stored z-  ", RS_pose.z) """
    return
def callback_jackalpose(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard latitude- %s ', data)
    global jackal_location
    jackal_location = data
    """ print("stored lat-  ", jackal_location.latitude)
    print("stored lon-  ", jackal_location.longitude)
    print("stored alt-  ", jackal_location.altitude) """
    return
def listener(listen_type):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    if listen_type == 1:
        rospy.Subscriber('detect_locate', coordinate, callback_rspose)
    if listen_type == 2:
        rospy.Subscriber('jackal0/navsat/fix', NavSatFix, callback_jackalpose)
    time.sleep(1)
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin() 
    return


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.latitude - aLocation1.lat
    dlong = aLocation2.longitude - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def fly_tojackal(jackpose, altitude ):
    """
    Flys to given position and altitude and checks if arrives to given pose 
    """
    print("Going towards first point for 30 seconds ...")
    # goes to given location
    print("jack lat - ", jackal_location.latitude)
    print("jack long - ", jackal_location.longitude)
    point1 = LocationGlobalRelative(jackal_location.latitude, jackal_location.longitude, altitude)
    vehicle.simple_goto(point1)
    
    currentLocation = vehicle.location.global_relative_frame
    distance = get_distance_metres(currentLocation, jackpose)
    
    # gets current position to know if it arrives
    while distance >=0.1: 
        currentLocation = vehicle.location.global_relative_frame
        distance = get_distance_metres(currentLocation, jackpose)
        time.sleep(1)
    print("reached destination starting final approach")

    
def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)


    
arm_and_takeoff(5)


print("Set default/target airspeed to 3")
vehicle.airspeed = 3

 # sleep so we can see the change in map
time.sleep(10)

# gets jackal latitude an longitude 
listener(2)
time.sleep(2)
""" print("Going towards first point for 30 seconds ...")
print("jack lat - ", jackal_location.latitude)
print("jack long - ", jackal_location.longitude)
point1 = LocationGlobalRelative(jackal_location.latitude, jackal_location.longitude, 6)
vehicle.simple_goto(point1) """
fly_tojackal(jackal_location,5)
time.sleep(2)
listener(1)
if RS_pose.detection==1:
    # sleep so we can see the change in map
    print("UAV detected  ...")
    print("goint to  x-  ", RS_pose.x)
    print("goint to  y-  ", RS_pose.y)
    print("going to  z-  ", RS_pose.z)
    goto_position_target_local_ned(-RS_pose.x, RS_pose.y, RS_pose.y-1.5)
    while abs(RS_pose.x) >= 0.1 and abs(RS_pose.y) >= 0.1:
        listener(1)
        time.sleep(1)
    time.sleep(2)


"""
The example is completing. LAND at current location.
"""
print("Setting LAND mode...")
time.sleep(2)
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()


print("Completed")