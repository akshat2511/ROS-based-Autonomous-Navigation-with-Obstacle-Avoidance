#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped  # Import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import time
import math

pub = None
goal = PoseStamped()  # Initialize goal as a PoseStamped message
current_position = None
linear_speed_x = 0.5
angular_speed_z = 0.5
goal_threshold = 0.1  # Adjust this threshold as needed
turn_duration = 2.0  # Time in seconds to turn 90 degrees

def set_yaw(yaw_angle):
    """Set the yaw angle of the robot."""
    global pub

    msg = Twist()
    msg.angular.z = yaw_angle  # Set the angular velocity around the z-axis (yaw axis)
    msg.linear_speed_x = 0.5
    pub.publish(msg)  # Publish the Twist message to set the yaw

def get_rotation(msg):
    
    """Callback function to get the current orientation of the robot."""
    global current_position,yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    current_position = msg.pose.pose.position
    rospy.loginfo(f"Current Position: ({current_position.x}, {current_position.y}, {current_position.z}), Yaw: {math.degrees(yaw)}")

def laser_readings(msg):
    """Callback function to process laser scan data."""
    global goal, current_position, pub

    if goal is not None and current_position is not None:
        parts = {
            'right': min(min(msg.ranges[0:59]), 1),
            'straight': min(min(msg.ranges[60:119]), 1),
            'left': min(min(msg.ranges[120:179]), 1),
        }
        rospy.loginfo(parts)
        logic(parts)

def logic(parts):
    """Logic for robot movement based on laser scan data."""
    global goal, current_position, pub, linear_speed_x, angular_speed_z, turn

    x_distance = goal.pose.position.x - current_position.x
    y_distance = goal.pose.position.y - current_position.y

    msg = Twist()

    if abs(x_distance) > 0.7 or abs(y_distance) > 0.7:
        if parts['right'] > 0.7 and parts['straight'] > 0.7 and parts['left'] > 0.7:
            angle = math.atan2(y_distance , x_distance)
            # if turn==0:
                # turn = 1
            rospy.loginfo("===============wapis ghus gaya angle set==================================")

            rospy.loginfo(angle)
            rospy.loginfo("===============================turn changed kyuki angle to target angle")
            rotate_to_target_angle(angle,-2)
            #turn=1
                # msg.linear.x = 0
                # msg.angular.z = angle
                # pub.publish(msg)
                # start_time = time.time()  # Record the start time
                # while (time.time() - start_time) < 3:
                #     rospy.loginfo(time.time() - start_time)
                # Set the linear and angular velocities for normal movement
            # else:
            msg.linear.x = 0.5
                #msg.linear.y = 0.5
            msg.angular.z = 0.0
            rospy.loginfo("===============seedhe chalunga==================================")
            pub.publish(msg)
        else:
            rospy.loginfo("===============obstacle==================================")

            turn = 0
            turn = 0
            if parts['right'] > 0.7 and parts['straight'] < 0.7 and parts['left'] < 0.7:
                linear_speed_x = 0.5
                angular_speed_z = -0.5
            elif parts['right'] > 0.7 and parts['straight'] > 0.7 and parts['left'] < 0.7:
                linear_speed_x = 0.5
                angular_speed_z = 0.0
            elif parts['right'] < 0.7 and parts['straight'] > 0.7 and parts['left'] < 0.7:
                linear_speed_x = 0.5
                angular_speed_z = 0
            elif parts['right'] < 0.7 and parts['straight'] > 0.7 and parts['left'] > 0.7:
                linear_speed_x = 0.5
                angular_speed_z = 0.0
            elif parts['right'] > 0.7 and parts['straight'] < 0.7 and parts['left'] > 0.7:
                linear_speed_x = 0.5
                angular_speed_z = 0.5
            elif parts['right'] < 0.7 and parts['straight'] < 0.7 and parts['left'] > 0.7:
                linear_speed_x = 0.5
                angular_speed_z = 0.5
            elif parts['right'] < 0.7 and parts['straight'] < 0.7 and parts['left'] < 0.7:
                linear_speed_x = 0
                angular_speed_z = -0.5

            msg.linear.x = linear_speed_x
            msg.linear.y = 0
            msg.angular.z = angular_speed_z
            pub.publish(msg)
    else:
        linear_speed_x = 0.0
        angular_speed_z = 0.0
        msg.linear.x = linear_speed_x
        msg.angular.z = angular_speed_z
        pub.publish(msg)
        rospy.loginfo("Goal reached!")
        rospy.signal_shutdown("Goal reached")
        
def rotate_to_target_angle(target_angle_deg, kP_gain):
    """Function to rotate the robot to a target angle using a proportional controller."""
    global yaw
    target_angle_rad = target_angle_deg
    command = Twist()
    r = rospy.Rate(1000)  # 10 Hz rate for publishing commands

    while not rospy.is_shutdown():
        error = target_angle_rad - yaw  # Calculate error
        degreeangle = error * 57.3
        command.angular.z = kP_gain * error  # Proportional control
        pub.publish(command)  # Publish the Twist message
        r.sleep()  # Sleep to maintain the desired publishing rate
        rospy.loginfo("===============3==================================")
        if abs(degreeangle) < 5.0 :# If the error is
            command.angular.z = 0
            command.linear.x = 0
            command.linear.y = 0
            pub.publish(command) # Publish the Twist message
            break

def main():
    """Main function to initialize ROS node and setup subscribers."""
    global pub, goal
    rospy.loginfo("===============main==================================")
    #turn = 0  # Initialize turn variable
    rospy.init_node('obs_avoid')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_laser = rospy.Subscriber("/m2wr/laser/scan", LaserScan, laser_readings)
    sub_odom = rospy.Subscriber("/odom", Odometry, get_rotation)
    # Wait for the first message to arrive before starting the controller
    rospy.wait_for_message('/odom', Odometry)
    # Set the goal coordinates here
    goal.header.frame_id = "map"  # Set the frame_id for the header
    goal.pose.position.x = 1
    goal.pose.position.y = -1.5
    rospy.loginfo("===============================main func ne turn change kiya ")
    rospy.spin()

if __name__ == '__main__':
    main()
