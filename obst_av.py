
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from math import floor
from sensor_msgs.msg import LaserScan
class Hello(Node):
    def __init__(self):
        super().__init__('Obstacle Avoidance')
        self.scan_msg = []
        self.current_vel = Twist()    
        self.create_subscription(Twist,'/webcmd_vel',self.vel_call,1)
        self.create_subscription(LaserScan,'/scan',self.lidar,1)
        time.sleep(2)
        self.pub=self.create_publisher(Twist,'/cmd_vel',1)
        self.vel=Twist()
        self.threshold_min = 0.60                    # Threshold to slow down
        self.threshold_max = 0.45                    # Threshold to stop
        self.threshold_recovery = 0.35               # Threshold until which automatic recovery is possible
                       # Having a record of current velocity
        self.recovery_started = False                # Facilitating recovery mode of the robot
        self.a=[]
        
        self.timer=self.create.timer(0.05,self.hello)
    def hello(self):
        if not self.scan_msg:
            return
        elif not self.current_vel.linear.x:
            data_per_degree = len(scan_msg) / 360
            scan_msg=self.scan_msg
            vel=self.vel
            threshold_min=self.threshold_min
            threshold_max=self.threshold_max
            # Initializing velocity variables to be published
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0

            # Defining the region of vision for checking the region of obstacles
            fov_along_x = 80
            fov_along_y = 70

            # Classifying regions and obtaining the closest obstacle in that region
            # Front region has to be splitted into two because we have to take defined data both below and above the index 0
            region_front_left = min(scan_msg[floor(-data_per_degree * (fov_along_x / 2)):])
            region_front_right = min(scan_msg[: floor(data_per_degree * (fov_along_x / 2))])
            region_front = min(region_front_left, region_front_right)
            region_back = min(scan_msg[floor(data_per_degree * (180 - (fov_along_x / 2))): floor(data_per_degree * (180 + (fov_along_x / 2)))])
            region_left = min(scan_msg[floor(data_per_degree * (90 - (fov_along_y / 2))): floor(data_per_degree * (90 + (fov_along_y / 2)))])
            region_right = min(scan_msg[floor(data_per_degree * (270 - (fov_along_y / 2))): floor(data_per_degree * (270 + (fov_along_y / 2)))])
            regions = [region_front, region_left, region_back, region_right]
            # print("Least of each regions", region_front, region_right, region_back, region_left)
            
            # print("The minimum distance found is: ", float(min(scan_msg)))

            # The below case assumes that the robot currently performs skid steering. If the motor_controller.py is re-written to facilitate differential drive then update the below with respective edge cases along with recovery scenarios
            # Handles the normal operation of obstacle avoidance and recovery
            if float(min(scan_msg)) > float(threshold_min):
                pass

            elif float(min(scan_msg)) < float(threshold_min) and float(min(scan_msg)) > float(threshold_max):
                # print(current_vel.linear.x, current_vel.angular.z)
                if self.current_vel.linear.x > 0:
                    vel.linear.x = 0.1
                else:
                    vel.linear.x = -0.1
                vel.angular.z = 0
                self.pub.publish(vel)
                print("Slowing down :/")

            # The robot is considerably in a closer range and hence will be stopped and this triggers the recovery mode
            elif float(min(scan_msg)) <= float(threshold_max) and not recovery_started:
                vel.linear.x = 0
                vel.angular.z = 0
                self.pub.publish(vel)
                time.sleep(2)
                print("Stopped :(")
                recovery_started = True

            elif float(min(scan_msg)) <= float(threshold_max) + 0.05 and recovery_started:
                recovery_started = self.recover_robot(scan_msg, regions, vel, self.pub)

            # Handles the recovery state condition
            else:
                vel.linear.x = 0
                vel.angular.z = 0
                self.pub.publish(vel)
                print("cannot recover")
                # playsound.playsound("cannot_recover.mp3")
        elif self.current_vel.linear.x:
            self.webrecov()
        else:
            pass
    def webrecov(self):
            
 
   
        if len(a)>4:
            a.clear()
    
            #playsound.playsound("cannot_recover.mp3")

        if float(min(scan_msg)) < float(threshold_min) and float(min(scan_msg)) > float(threshold_max) or  real < 0.40 and real > 0.38:
                    # print(current_vel.linear.x, current_vel.angular.z)
            vel.linear.x=current_vel.linear.x/3.2
            vel.angular.z=current_vel.angular.z/3.2
            pub.publish(vel)
    
                
                # The robot is considerably in a closer range and hence will be stopped and this triggers the recovery mode
        elif float(min(scan_msg)) <= float(threshold_max)+0.05 or real <= 0.38+0.05:
            print("ulla poiten")

            if web_vel.linear.x>0 :
                data_per_degree=len(scan_msg)/360
                angle_total=140
                field1_min=min(scan_msg[floor(-data_per_degree * (angle_total /4)):])
                field2_min=min(scan_msg[: floor(data_per_degree * (angle_total / 4))])
                field3_min=min(scan_msg[floor(-data_per_degree * (angle_total / 2)):floor(-data_per_degree * (angle_total / 4))])
                field4_min=min(scan_msg[floor(data_per_degree * (angle_total / 4)):floor(data_per_degree * (angle_total / 2))])
                field1=[field1_min,field2_min,field3_min,field4_min]
                field1.sort()
                field=field1[i]
                if field<12:
                    if field==field1_min:
                        print("1")
                        vel.angular.z=-0.1
                        vel.linear.x=0
                        pub.publish(vel)
                        time.sleep(1.5)
                        vel.angular.z=0
                        vel.linear.x=0
                        a.append(1)
                        check(pub)
                    

                        recovery_started = False
                    elif field==field2_min:
                        print("2")
                        vel.angular.z=0.1
                        vel.linear.x=0
                        pub.publish(vel)
                        time.sleep(1.5) 
                        vel.angular.z=0
                        vel.linear.x=0
                        a.append(2)
                        
                        check(pub)
                        
                        recovery_started = False
                    elif field==field3_min :
                        print("3")
                        vel.angular.z=-0.1
                        vel.linear.x=0
                        pub.publish(vel)
                        time.sleep(4.3)
                        vel.angular.z=0
                        vel.linear.x=0
                        a.append(3)
                        check(pub)

                        recovery_started = False
                    elif field==field4_min:
                        print("4")
                        vel.angular.z=0.1
                        vel.linear.x=0
                        pub.publish(vel)
                        time.sleep(4.3) 
                        vel.angular.z=0
                        vel.linear.x=0
                        a.append(4)
                        check(pub)
                        recovery_started = False
                    else:
                        pass
                else:
                    pass
            elif web_vel.linear.x<0:
                field3_min=min(scan_msg[floor(data_per_degree * (180 - (angle_total/ 2))):floor(data_per_degree * (180))])
                field4_min=min(scan_msg[floor(data_per_degree * (180)):floor(data_per_degree * (180 + (angle_total/ 2)))])
                field=max(field3_min,field4_min)
                if field==field3_min:
                    vel.angular.z=-0.39
                    vel.linear.x=0
                    pub.publish(vel)
                    
                    time.sleep(1)
                    vel.angular.z=0
                    vel.linear.x=0.2
                    pub.publish(vel)
                    recovery_started = False
                elif field==field4_min:
                    vel.angular.z=0.39
                    vel.linear.x=0
                    pub.publish(vel)
                    time.sleep(1) 
                    vel.linear.x=0.2
                    vel.angular.z=0
                    pub.publish(vel)
                    recovery_started = False
                else:
                    pass
            

            else :
                pass

        elif float(min(scan_msg)) > float(threshold_min) :
            vel.linear.x=web_vel.linear.x
            vel.angular.z=web_vel.angular.z
            pub.publish(vel)
            time.sleep(1) 
            print("culprit")       





    def recover_robot(self,msg,regions,vel,pub):
           # Finding the suitable region to move the robot to, and initiate recovery
        if float(min(msg)) > self.threshold_recovery:

            if max(regions) == regions[2]:
                vel.linear.x = -0.15
                vel.angular.z = 0
                pub.publish(vel)
                time.sleep(5)
                vel.linear.x = 0
                vel.angular.z = 0
                self.pub.publish(vel)
                return False

            elif max(regions) == regions[3]:
                vel.linear.x = 0
                vel.angular.z = -0.25
                pub.publish(vel)
                time.sleep(2)
                vel.linear.x = 0
                vel.angular.z = 0
                pub.publish(vel)
                return False

            elif max(regions) == regions[1]:
                vel.linear.x = 0
                vel.angular.z = 0.25
                pub.publish(vel)
                time.sleep(2)
                vel.linear.x = 0
                vel.angular.z = 0
                pub.publish(vel)
                return False

            elif max(regions) == regions[0]:
                vel.linear.x = 0.15
                vel.angular.z = 0
                pub.publish(vel)
                time.sleep(5)
                vel.linear.x = 0
                vel.angular.z = 0
                pub.publish(vel)
                return False

            else:
                pass


    



    def vel_call(self,msg):
        global current_vel                  # Specifying the use of global variable
        self.current_vel = msg                   # Copying the current msg received into global variable


    def lidar(self,msg):
        global scan_msg                          # Specifying the use of global variable
        size_of_the_plate = 0.25            # Size of the plate in which LiDAR is installed
        self.scan_msg = [12 if (x == "inf" or x < size_of_the_plate)
            else x for x in msg.ranges]   # Ignoring all the "inf" data and data lesser than than that of the size of the plate





def main(args=None):
    rclpy.init(args=args)
    node=Hello()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
