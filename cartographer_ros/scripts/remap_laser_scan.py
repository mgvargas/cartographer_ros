#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import tf
import math
import numpy
import time
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
import copy


global front_scan
global back_scan
global comb_scan
global new_back
global new_front
front_scan = LaserScan()
back_scan = LaserScan()
comb_scan = LaserScan()
new_back = False
new_front = False


def callback_front(data):
    global front_scan
    global new_front
    front_scan = data
    new_front = True

def callback_back(data):
    global back_scan
    global new_back
    back_scan = data
    new_back = True


# Function array_remap: Calculations to map angle and distance fron the laser scan to the base
# Arguments: scan message, distance btwn reference frames, angle btwn frames
def array_remap(scan, trans, rot_ang):
    # Angle shift btwn laser scanner and base
    if rot_ang > 0:
        ang_shift = rot_ang + math.pi/4
    else:
        ang_shift = math.pi*2 + rot_ang + math.pi/4

    # Calculate distance from sensors to base (assume z is not relevant)
    dist_base = math.sqrt(trans[0] ** 2 + trans[1] ** 2)

    # Calc angles and range from sensors (initial)
    alpha = math.acos(abs(trans[1]/dist_base)) # Ang btwn sensor and base (upper)
    z = math.asin(abs(trans[1]/dist_base))     # Ang btwn sensor and base (lower)
    b = scan.ranges[0]
    c = dist_base
    a = math.sqrt( b**2 + c**2 - 2*b*c*math.cos(alpha) ) # Distance btwn base and laser scan
    beta = math.acos( (a**2 + c**2 - b**2) / (2*a*c) )   # Ang btwn dist_base and 'a'
    ang = math.pi*3/2 - beta - alpha
    num_scans = len(scan.ranges)

    for n in range(num_scans):
        b = scan.ranges[n]
        if math.isnan(b) == True:
            b = 0
        gamma = scan.angle_increment*n # Scan angle wrt laser scan
        angle = alpha + gamma  # Scan angle wrt initial point

        if angle > math.pi:
            angle = math.pi*2 - angle
            # Cosine theorem
            a = math.sqrt( b**2 + c**2 - 2*b*c*math.cos(angle) )
            beta = - math.acos( (a**2 + c**2 - b**2) / (2*a*c) )
            ang_ref_point = ang_shift - beta - alpha # Scan angle wrt base
        else:
            # Cosine theorem
            a = math.sqrt( b**2 + c**2 - 2*b*c*math.cos(angle) )
            beta = math.acos( (a**2 + c**2 - b**2) / (2*a*c) )
            ang_ref_point =  ang_shift - beta - alpha
        eq_scan_num = round ((ang_ref_point)/scan.angle_increment)
        #if (comb_scan.ranges[eq_scan_num] != 0 and comb_scan.ranges[eq_scan_num] < a):
        #    comb_scan.ranges[eq_scan_num] = comb_scan.ranges[eq_scan_num]
        #else:
        comb_scan.ranges[eq_scan_num] = a

    return comb_scan

def remap():
    global front_scan
    global back_scan
    global comb_scan
    global new_front
    global new_back

    rospy.init_node('remap_scans', anonymous=True)

    back_scan.angle_increment =  0.00436332309619 # Set initial value to avoud DIV0 if topic is not received yet

    # Read info from both sensors
    rospy.Subscriber("hokuyo_utm30_front/most_intense", LaserScan, callback_front)
    rospy.Subscriber("hokuyo_utm30_back/most_intense", LaserScan, callback_back)

    tf_listener = tf.TransformListener()

    # Publish the transformed scans
    pub = rospy.Publisher('laser_scans', LaserScan, queue_size=3)
    rate = rospy.Rate(100) # 10hz
    time.sleep(1)

    # Define initial properties of the combined scan
    comb_scan = copy.copy(back_scan) # fill initial values
    comb_scan.ranges = numpy.resize(comb_scan.ranges,(1440))
    comb_scan.ranges[:] = 0
    comb_scan.header.frame_id = 'base_link'
    comb_scan.header.stamp = back_scan.header.stamp

    while not rospy.is_shutdown():
        try:
            # Get the transforms from the laser sensors to base_link
            (trans_f,rot_f) = tf_listener.lookupTransform('/base_link', '/laser_reference_front', rospy.Time(0))
            (trans_b,rot_b) = tf_listener.lookupTransform('/base_link','/laser_reference_back', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Rotation btw front laser and base
        euler_f = tf.transformations.euler_from_quaternion(rot_f)
        yaw_f = euler_f[2]

        # Rotation btw rear laser and base
        euler_b = tf.transformations.euler_from_quaternion(rot_b)
        yaw_b = euler_b[2]

        # Scans required to cover 180 degrees
        req = int(math.pi/back_scan.angle_increment) # Num of req scans per sensor
        length = req*2
        comb_scan.ranges = numpy.resize(comb_scan.ranges,(length))

        # Set the starting angle of the scan
        comb_scan.angle_min = 0
        comb_scan.angle_max = math.pi*2-abs(comb_scan.angle_min)
        comb_scan.ranges[:] = 0

        if (new_front==True and new_back==True):
            array_remap(back_scan, trans_b, yaw_b)
            array_remap(front_scan, trans_f, yaw_f)
            comb_scan.header.stamp = back_scan.header.stamp
            pub.publish(comb_scan)
            new_front = False
            new_back = False

        #rospy.loginfo(comb_scan)
        rate.sleep()



if __name__ == '__main__':
    try:
        remap()
    except rospy.ROSInterruptException:
        pass
