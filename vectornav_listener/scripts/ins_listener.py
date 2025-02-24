#!/usr/bin/env python
import rospy
from vectornav.msg import Ins  # Adjust if your message package name is different
import lcm
import sys
import os
sys.path.append(os.path.join(os.getcwd(), 'src', 'vectornav_listener', 'scripts'))
from ins import ins_t


def callback(msg):
    buf = ins_t()
    buf.time = msg.time
    """ LCM Type: double
        # GPS time of week in seconds 
    """
    buf.week = msg.week
    """ LCM Type: int16_t 
        # GPS week (week) 
    """
    buf.utcTime = msg.utcTime
    """ LCM Type: int64_t 
        # The current UTC time. The year is given as a signed byte year offset from the year 2000. E.g. 2013 as 13.
            # Fields:       year    month    day    hour    min    sec    ms
            # Byte offset:  0       1        2      3       4      5      6|7 
    """
    buf.insStatus = msg.insStatus
    """ LCM Type: int16_t 
        # INS Status
        # Name		Bit Offset	Format	Description
        # Mode		0			2 bits	Indicates the current mode of the INS filter.
        #								0 = Not tracking. Insufficient dynamic motion to estimate attitude.
        #								1 = Sufficient dynamic motion, but solution not within performance specs.
        #								2 = INS is tracking and operating within specifications.
        # GpsFix	2			1 bit	Indicates whether the GPS has a proper fix
        # Error 	3			4 bits	Sensor measurement error code
        #								0 = No errors detected.
        # 								Name			Bit Offset	Format	Description
        #								Time Error		0			1 bit	High if INS filter loop exceeds 5 ms.
        #								IMU Error		1			1 bit	High if IMU communication error is detected.
        #								Mag/Pres Error	2			1 bit	High if Magnetometer or Pressure sensor error is detected.
        #								GPS Error		3			1 bit	High if GPS communication error is detected.
        #Reserved	7			9 bits	Reserved for future use.
    """
    buf.yaw = msg.yaw
    """ LCM Type: float 
        # Yaw angle relative to true north. (degree)
    """
    buf.pitch = msg.pitch
    """ LCM Type: float 
        # Yaw angle relative to true north (degree)
    """
    buf.roll = msg.roll
    """ LCM Type: float 
        # Pitch angle relative to horizon (degree)
    """
    buf.latitude = msg.latitude
    """ LCM Type: double 
        # INS solution position in geodetic latitude (degree)
    """
    buf.longitude = msg.longitude
    """ LCM Type: double 
        # INS solution position in geodetic longitude (degree)
    """
    buf.altitude = msg.altitude
    """ LCM Type: double 
        # Height above ellipsoid. (WGS84) (meter)
    """
    buf.nedVelX = msg.nedVelX
    """ LCM Type: float 
        # INS solution velocity in NED frame. (North) (m/s)
    """
    buf.nedVelY = msg.nedVelY
    """ LCM Type: float 
        # INS solution velocity in NED frame. (East) (m/s)
    """
    buf.nedVelZ = msg.nedVelZ
    """ LCM Type: float 
        # INS solution velocity in NED frame. (Down) (m/s)
    """
    buf.attUncertainty = msg.attUncertainty
    """ LCM Type: float[3] 
        # Uncertainty in attitude estimate (yaw, pitch and roll in degrees)
    """
    buf.posUncertainty = msg.posUncertainty
    """ LCM Type: float 
        # Uncertainty in position estimate (m)
    """
    buf.velUncertainty = msg.velUncertainty
    
    lc.publish("loc-ins", buf.encode())
    rospy.loginfo("Received INS message:")
    rospy.loginfo("  Time: %f", msg.time)
    rospy.loginfo("  Week: %d", msg.week)
    rospy.loginfo("  UTC Time: %d", msg.utcTime)
    rospy.loginfo("  INS Status: %d", msg.insStatus)
    rospy.loginfo("  Yaw: %f, Pitch: %f, Roll: %f", msg.yaw, msg.pitch, msg.roll)
    rospy.loginfo("  Latitude: %f, Longitude: %f, Altitude: %f", msg.latitude, msg.longitude, msg.altitude)
    rospy.loginfo("  NED Velocity: [%f, %f, %f]", msg.nedVelX, msg.nedVelY, msg.nedVelZ)
    rospy.loginfo("  Attitude Uncertainty: %s", str(msg.attUncertainty))
    rospy.loginfo("  Position Uncertainty: %f", msg.posUncertainty)
    rospy.loginfo("  Velocity Uncertainty: %f", msg.velUncertainty)

def listener():
    rospy.init_node('ins_listener', anonymous=True)
    rospy.Subscriber("/vectornav/ins", Ins, callback)
    rospy.spin()

lc = lcm.LCM()
if __name__ == '__main__':
    listener()
