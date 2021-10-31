#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

autoland.py: GUIDED mode 
More documentation is provided at http://python.dronekit.io/examples/simple_goto.html & https://github.com/MartimBraga/Detection

"""
from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative , LocationGlobal
from pymavlink import mavutil # Needed for command message definitions
import rospy
import PID
from std_msgs.msg import Int32
from detection.msg import coordinate 
from sensor_msgs.msg import NavSatFix
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import xlwt
jackal_location = NavSatFix()
mission_pose = NavSatFix()
RS_pose = coordinate()
ar_pose = AlvarMarkers
ar_tag_flag = 0 


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
def callback_artagpose(data):
    global ar_pose
    global ar_tag_flag
    ar_tag_flag = 0
    ar_tag_pose = data.markers
    for ar_tag_pose in data.markers:

        marker_id = ar_tag_pose.id
        ar_tag_flag = 0
        if marker_id == 187:
            ar_tag_flag = 1
            marker_pose = ar_tag_pose.pose.pose

            ar_pose = marker_pose.position

            #ori = marker_pose.orientation
            #print("id -  ", marker_id)
            #print("tag x-  ", ar_pose.x)
            #print("tag y-  ", ar_pose.y)
            #print("tag z-  ", ar_pose.y)
            #print("--------------------------")
            return
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
    if listen_type == 3:
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback_artagpose)
    time.sleep(0.1)
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


def fly_tojackal(jackpose, Altitude):
    """
    Flys to given position and altitude and checks if arrives to given pose 
    """
    print("Flying towards jackal ...")
    # Prints the given location
    print("jack lat - ", jackpose.latitude)
    print("jack long - ", jackpose.longitude)
    print("jack alt - ", jackpose.altitude)
    point1 = LocationGlobal(jackpose.latitude, jackpose.longitude, jackpose.altitude + Altitude)
    vehicle.simple_goto(point1)
    
    currentLocation = vehicle.location.global_frame
    distance = get_distance_metres(currentLocation, jackpose)
    
    # gets current position to check if it has arrived
    while distance >=0.3: 
        currentLocation = vehicle.location.global_frame
        distance = get_distance_metres(currentLocation, jackpose)
        print("distance - ", distance)
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

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 100 Hz cycle
    for x in np.arange(0,duration,0.01):
        vehicle.send_mavlink(msg)
        time.sleep(0.01)

class Position():
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.state_change_time = rospy.Time.now() 

    def pose(self):
            #variable pid        
            P= 0.27
            I= 0.0107
            D= 0.027
            #PID for pose control in x 
            pidX = PID.PID(P, I, D)
            pidX.SetPoint=0.0 #setpoint sistem = pose in meter
            pidX.setSampleTime(0.025)
            pidX.last_error=0.0
            
            feedbackX = 0.0 #feedback sistem 
            #listx
            self.feedback_listX = []
            self.time_listX = []
            self.setpoint_listX = []
            velocityX=[]
            timeX=[]
            time2X=[]
            pidstartX=[]
            valuePX=[]        
            valueIX=[]
            valueDX=[]
            deX=[]

            #PID for pose control in Y 
            pidY = PID.PID(P, I, D)
            pidY.SetPoint=0.0 #setpoint sistem = pose in meter
            pidY.setSampleTime(0.025)
            pidY.last_error=0.0
            
            feedbackY = 0.0 #feedback sistem
            
            #listY
            self.feedback_listY = []
            self.time_listY = []
            self.setpoint_listY = []
            velocityY=[]
            timeY=[]
            time2Y=[]
            pidstartY=[]
            valuePY=[]        
            valueIY=[]
            valueDY=[]
            deY=[]
                        
            Pz= 0.152
            Iz= 0.046
            Dz= 0.025
            #PID for pose control in Z 
            pidZ = PID.PID(Pz, Iz, Dz)
            pidZ.SetPoint=0.0 #setpoint sistem = pose in meter
            pidZ.setSampleTime(0.025)
            pidZ.last_error=0.0
            
            feedbackZ = 0.0 #feedback sistem
            
            #listZ
            self.feedback_listZ = []
            self.time_listZ = []
            self.setpoint_listZ = []
            velocityZ=[]
            timeZ=[]
            time2Z=[]
            pidstartZ=[]
            valuePZ=[]        
            valueIZ=[]
            valueDZ=[]
            deZ=[]
            

            #drone pose log
            dronepose = vehicle.location.local_frame
            dronex = []
            droney = []
            dronez = []
            artagposex= []
            artagposey= []
            artagposez= []
            rsposex= []
            rsposey= []
            rsposez= []
            totalY=0.0
            i=0       
            end_time=0.0
            totaltime=0.0
            
            #input range ==================================================
            pidX.SetPoint = 0 #you can input negative number to move in the opposite direction
            pidY.SetPoint = 0
            pidZ.SetPoint = 1
            #===============================================================
            ticinit = time.perf_counter()
            while pidX.SetPoint is not None:
                tic = time.perf_counter()

                listener(1)
                listener(3)
                pidX.update(feedbackX)
                pidY.update(feedbackY)
                pidZ.update(feedbackZ)

                self.outputX = pidX.output
                self.outputY = pidY.output
                self.outputZ = pidZ.output

                outputstartX=self.outputX
                outputstartY=self.outputY
                outputstartZ=self.outputZ

                if self.outputX > 1:
                    self.outputX = 1
                elif self.outputX < -1:
                    self.outputX = -1
                #print (self.outputX)
                
                if self.outputY > 1:
                    self.outputY = 1
                elif self.outputY < -1:
                    self.outputY = -1
                #print (self.outputY)

                if self.outputZ > 1:
                    self.outputZ = 1
                elif self.outputZ < -1:
                    self.outputZ = -1
                #print (self.outputZ)
                

                #sending command to ar drone
                start_time = time.time()
                #bless.SetCommand(self.output,0,0,0,0,0)
                send_ned_velocity(self.outputX, self.outputY, -self.outputZ, 0.01)
                
                end_time = time.time()
                
                #mileage calculation
                timer = end_time - start_time
                totaltime = totaltime + timer
                listener(1)
                listener(3)
                if (RS_pose.detection == 0):
                    send_ned_velocity(0, 0, -2, 1)
                    time.sleep(1)
                else:    
                    totalX = RS_pose.x
                    totalY = RS_pose.y
                    totalZ = RS_pose.z

                
                if ar_tag_flag == 1:
                    totalX = ar_pose.y
                    totalY = ar_pose.x
                    totalZ = ar_pose.z
                    print("-----------------pose ar tag-------------")
                #feedback
                feedbackX = totalX
                feedbackY = totalY
                feedbackZ = totalZ
                toc = time.perf_counter()

                timer2 = tic - ticinit
                i+=1
                dronepose.north = vehicle.location.local_frame.north
                dronepose.east = vehicle.location.local_frame.east
                dronepose.down = vehicle.location.local_frame.down
                #list data variabel x            
                velocityX.append(self.outputX)
                timeX.append(str(totaltime))
                time2X.append(str(timer))
                pidstartX.append(str(outputstartX))
                self.feedback_listX.append(feedbackX)
                self.setpoint_listX.append(pidX.SetPoint)
                self.time_listX.append(i)
                valuePX.append(pidX.PTerm)
                valueIX.append(pidX.ITerm)
                valueDX.append(pidX.DTerm)
                deX.append(pidX.delta_time)
                #list data variabel y  
                velocityY.append(self.outputY)
                timeY.append(str(totaltime))
                time2Y.append(str(timer))
                pidstartY.append(str(outputstartY))
                self.feedback_listY.append(feedbackY)
                self.setpoint_listY.append(pidY.SetPoint)
                self.time_listY.append(i)
                valuePY.append(pidY.PTerm)
                valueIY.append(pidY.ITerm)
                valueDY.append(pidY.DTerm)
                deY.append(pidY.delta_time)
                #list data variabel z  
                velocityZ.append(self.outputZ)
                timeZ.append(str(totaltime))
                time2Z.append(str(timer2))
                pidstartZ.append(str(outputstartZ))
                self.feedback_listZ.append(feedbackZ)
                self.setpoint_listZ.append(pidZ.SetPoint)
                self.time_listZ.append(i)
                valuePZ.append(pidZ.PTerm)
                valueIZ.append(pidZ.ITerm)
                valueDZ.append(pidZ.DTerm)
                deZ.append(pidZ.delta_time)
                dronex.append(dronepose.north)
                droney.append(dronepose.east)
                dronez.append(dronepose.down)
                if ar_tag_flag == 1:
                    artagposex.append(ar_pose.x)
                    artagposey.append(ar_pose.y)
                    artagposez.append(ar_pose.z)
                else:
                    artagposex.append(0.0)
                    artagposey.append(0.0)
                    artagposez.append(0.0)
                rsposex.append(RS_pose.x)
                rsposey.append(RS_pose.y)
                rsposez.append(RS_pose.z)
                #print on terminal
                
                #print("-------------------------------------")
                #print ("pose X- ", str(totalX), "Y-", str(totalY), "Z-", str(totalZ))
                #print("-------------------------------------")
                #print ("velocity X- ", str(outputstartX), "Y- ", str(outputstartY), "Z- ", str(outputstartZ))
                #print ("start time  "+str(start_time))
                #print ("end time= "+str(end_time))
                #print ("timer= "+str(timer))
                #print("-------------------------------------")
                #print ("PID X- ", str(outputstartX), "Y- ", str(outputstartY))            
                #print ("SetCommand X- ", str(self.outputX), "Y- ", str(self.outputY))
                #print ("last error X- ", str(pidX.last_error), "Y- ", str(pidY.last_error))
                #print (i)
                #print("=====================================\n")
                if (abs(totalX)<= 0.10 and abs(totalY)<= 0.10):
                    pidZ.SetPoint = 0.15
                
                if (abs(totalX)<= 0.10 and abs(totalY)<= 0.10 and abs(totalZ)<= 0.2):
                    break
        
                #export to xls to log movement
                wb = xlwt.Workbook()
                ws = wb.add_sheet("data")
                for i, row in enumerate(timeX):
                    ws.write(i, 0, row)
                for i, row2 in enumerate(time2X):
                    ws.write(i, 1, row2)
                for i, row3 in enumerate(time2Z):
                    ws.write(i, 2, row3)

                for i, row4 in enumerate(self.feedback_listX):
                    ws.write(i, 3, row4)
                for i, row5 in enumerate(self.feedback_listY):
                    ws.write(i, 4, row5)
                for i, row6 in enumerate(self.feedback_listZ):
                    ws.write(i, 5, row6)

                #actual drone pose
                for i, row7 in enumerate(dronex):
                    ws.write(i, 6, row7)
                for i, row8 in enumerate(droney):
                    ws.write(i, 7, row8)
                for i, row9 in enumerate(dronez):
                    ws.write(i, 8, row9)

                #for i, row7 in enumerate(velocityX):
                #    ws.write(i, 6, row7)
                #for i, row8 in enumerate(velocityY):
                #    ws.write(i, 7, row8)
                #for i, row9 in enumerate(velocityY):
                #    ws.write(i, 8, row9)

                
                for i, row10 in enumerate(rsposex):
                    ws.write(i, 9, row10)
                for i, row11 in enumerate(rsposey):
                    ws.write(i, 10, row11)
                for i, row12 in enumerate(rsposez):
                    ws.write(i, 11, row12)

                #for i, row10 in enumerate(timeX):
                #    ws.write(i, 9, row10)
                #for i, row11 in enumerate(timeY):
                #    ws.write(i, 10, row11)
                #for i, row12 in enumerate(timeZ):
                #    ws.write(i, 11, row12)
                
                for i, row13 in enumerate(artagposex):
                    ws.write(i, 12, row13)
                for i, row14 in enumerate(artagposey):
                    ws.write(i, 13, row14)
                for i, row15 in enumerate(artagposez):
                    ws.write(i, 14, row15)
                
                #for i, row13 in enumerate(time2X):
                #    ws.write(i, 12, row13)
                #for i, row14 in enumerate(time2Y):
                #    ws.write(i, 13, row14)
                #for i, row15 in enumerate(time2Z):
                #    ws.write(i, 14, row15)

                #for i, row12 in enumerate(self.time_listX):
                #    ws.write(i, 11, row12)
                #for i, row17 in enumerate(self.time_listY):
                #    ws.write(i, 16, row17)
                #for i, row18 in enumerate(self.time_listZ):
                #    ws.write(i, 17, row18)

                #for i, row13 in enumerate(pidstartX):
                #    ws.write(i, 12, row13)
                #for i, row14 in enumerate(pidstartY):
                #    ws.write(i, 13, row14)
                #for i, row15 in enumerate(pidstartZ):
                #    ws.write(i, 14, row15)

                for i, row16 in enumerate(valuePX):
                    ws.write(i, 15,row16)
                for i, row17 in enumerate(valuePY):
                    ws.write(i, 16,row17)
                for i, row18 in enumerate(valuePZ):
                    ws.write(i, 17,row18)

                for i, row19 in enumerate(valueIX):
                    ws.write(i, 18,row19)
                for i, row20 in enumerate(valueIY):
                    ws.write(i, 19,row20)
                for i, row21 in enumerate(valueIZ):
                    ws.write(i, 20,row21)

                for i, row22 in enumerate(valueDX):
                    ws.write(i, 21,row22)
                for i, row23 in enumerate(valueDY):
                    ws.write(i, 22,row23)
                for i, row24 in enumerate(valueDZ):
                    ws.write(i, 23,row24)

                for i, row25 in enumerate(deX):
                    ws.write(i, 24,row25)
                for i, row26 in enumerate(deY):
                    ws.write(i, 25,row26)
                for i, row27 in enumerate(deZ):
                    ws.write(i, 26,row27)

                for i, row28 in enumerate(velocityX):
                    ws.write(i, 27,row28)
                for i, row29 in enumerate(velocityY):
                    ws.write(i, 28,row29)
                for i, row30 in enumerate(velocityZ):
                    ws.write(i, 29,row30)

                wb.save("dataX.xls")
                


def goto_rslocation(height):
    """Given a desired height, it goes to the calculated realsense position"""
    
    listener(1)
    desired_height = vehicle.location.local_frame.down + RS_pose.z - height
    listener(1)
    print("goint to  x-  ", RS_pose.x)
    print("goint to  y-  ", RS_pose.y)
    print("going to  z-  ", RS_pose.z)
    goto_position_target_local_ned(RS_pose.x, RS_pose.y, desired_height)
    time.sleep(10)


""" -------------landing routine ----------------- """


arm_and_takeoff(5)


print("Set default/target airspeed to 3")
vehicle.airspeed = 3
landed = 0
#to simulate a mission fly to a locating than returns
mission_pose.latitude = 38.533728
mission_pose.longitude = -7.916495
mission_pose.altitude = 232
fly_tojackal(mission_pose,6)
while landed == 0:


    # gets jackal latitude an longitude 
    listener(2)
    time.sleep(1)
    while jackal_location.longitude==0:
        listener(2)
        time.sleep(1)
        print("Base GPS position not received, waiting...")

    fly_tojackal(jackal_location,6)
    #time.sleep(0.5)
    listener(1)
    #iniciate pid variable
    pidcontrol = Position()
    if RS_pose.detection==1:
        # sleep so we can see the change in map
        print("UAV detected  ...")

        listener(1)
        listener(3)
        #pidcontrol.pose()
        #goto_rslocation(2)
        pidcontrol.pose()
        landed ==1
        break
    else:
        continue  
    
    #listener(1)

    #if RS_pose.detection==1:
    #    pidcontrol.pose()
    #    landed ==1
    #    break
    #else:
    #    continue

    """     while (abs(RS_pose.x) >= 0.1 or abs(RS_pose.y) >= 0.1):
            print("loop  x-  ", RS_pose.x)
            print("loop  y-  ", RS_pose.y)
            listener(1)
            #pose(RS_pose.x, RS_pose.y)
            time.sleep(1)
        time.sleep(2)
        print("out_loop  x-  ", RS_pose.x)
        print("out_loop  y-  ", RS_pose.y) """


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