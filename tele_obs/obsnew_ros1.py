#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import floor
from std_msgs.msg import Float64
import time

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        self.scan_msg = []
        self.current_vel = Twist()
        
        rospy.Subscriber('/webcmd_vel', Twist, self.vel_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/min_depth', Float64, self.min_callback)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Velocity thresholds
        self.threshold_min = 0.48
        self.threshold_max = 0.43
        self.threshold_recovery = 0.33
        
        self.recovery_started = False
        self.vel.linear = 0
        self.vel.angular = 0
   
        
        self.vel = Twist()
        self.timer = rospy.Timer(rospy.Duration(0.05), self.process_scan)

    def process_scan(self, event):
        if not self.scan_msg:
            return

        data_per_degree = len(self.scan_msg) / 360


        fov_along_x = 150

        region_front = self.get_region_distance(data_per_degree, fov_along_x,0)
        region_back = self.get_region_distance(data_per_degree, fov_along_x,1)

        regions = [region_front, region_back]

        closest_distance = float(min(self.scan_msg))

        if closest_distance > self.threshold_min or self.real > self.threshold_min+0.25: 
            self.vel.linear.x = self.current_vel.linear.x
            self.vel.angular.z = self.current_vel.angular.z
            self.pub.publish(self.vel)

            time.sleep(2)

        elif self.threshold_max < closest_distance <= self.threshold_min or self.threshold_max+0.25 < self.real <= self.threshold_min+0.25:
            self.vel.linear.x = 0.1 if self.current_vel.linear.x > 0 else -0.1
            self.pub.publish(self.vel)
            rospy.loginfo("Slowing down :/")

        elif closest_distance <= self.threshold_max or self.real <= self.threshold_max+0.25:
            if not self.recovery_started:
                self.recovery_started = True
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                rospy.loginfo("Stopped :(")
                time.sleep(2)
            else:
                self.recover_robot(regions)

    def get_region_distance(self, data_per_degree, fov, back):
        if back:
            return min(self.scan_msg[floor(data_per_degree * (180 - (fov / 2))): 
                                      floor(data_per_degree * (180 + (fov / 2)))])
        else:
            # Front region calculation (left and right split)
            region_front_left = min(self.scan_msg[floor(-data_per_degree * (fov / 2)):])
            region_front_right = min(self.scan_msg[:floor(data_per_degree * (fov / 2))])
            return min(region_front_left, region_front_right)

    def recover_robot(self, regions):
        self.recovery_started = False  # Reset recovery mode
        if  self.current_vel.linear.x:
            if min(regions) == regions[0]:
                if self.current_vel.linear.x > 0:
                    self.vel.linear.x = 0
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    rospy.loginfo("Stopped :(")
                    time.sleep(1)
                elif self.current_vel.linear.x < 0:
                    self.vel.linear.x = self.current_vel.linear.x
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    time.sleep(1)
            elif min(regions) == regions[1]:
                if self.current_vel.linear.x < 0:
                    self.vel.linear.x = 0
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    rospy.loginfo("Stopped :(")
                    time.sleep(1)
                elif self.current_vel.linear.x > 0:
                    self.vel.linear.x = self.current_vel.linear.x
                    self.vel.angular.z = self.current_vel.angular.z
                    self.pub.publish(self.vel)
                    time.sleep(1)
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = self.current_vel.angular.z
            self.pub.publish(self.vel)
            rospy.loginfo("Stopped :(")
            time.sleep(1)

    def vel_callback(self, msg):
        self.current_vel = msg  # Update current velocity
    def min_callback(self, msg):
        self.real= msg.data  # Update current velocity
    def lidar_callback(self, msg):
        size_of_plate = 0.25  # Size of the plate for LiDAR
        self.scan_msg = [12 if (y == float('inf') or y < size_of_plate) else y for y in msg.ranges]

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

