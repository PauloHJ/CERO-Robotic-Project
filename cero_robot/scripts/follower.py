#!/usr/bin/env python 
import math
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#This node recieves a Laser Scan and computes the angle to the closest object.


  
#This class will receive a number and an increment and it will publish the  
# result of adding number + increment in a recursive way. 
class FollowerClass(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        
        ## PUBLISHERS ## 
        self.pub_cmdvel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
   
        ## SUBSCRIBERS ## 
        rospy.Subscriber("scan", LaserScan, self.laser_cb) 
       
        ### CONSTANTS ## 
        self.my_vel = Twist()
        
        
        
        
        #********** INIT NODE **********### 
        r = rospy.Rate(1) #1Hz 
        print("Node initialized 1hz")
        while not rospy.is_shutdown(): 
            #print("I'm working fine")
            self.pub_cmdvel.publish(self.my_vel) #publish the number
            r.sleep() 
            
            
    def laser_cb(self, msg): 
        ## This function recieves a Laser Scan and computes the angle to the closest object.
        
        closest_range = min(msg.ranges)                             #meters
        index = msg.ranges.index(closest_range)
        closest_angle = msg.angle_min + index * msg.angle_increment #rads
        print("range: " + str(closest_range))
        print("angle: " + str(closest_angle))
        
        
        l_treshold = 0.7
        a_treshold = 0.4
        
        if(closest_range != math.inf):
            #adjust linear velocity
            if(closest_range > l_treshold and (closest_angle < 1 and closest_angle > -1)):
                self.my_vel.linear.x = 0.25 #0.2
            else:
                self.my_vel.linear.x = 0.0;
        
        
            #adjust angular
            if(closest_angle > a_treshold or closest_angle < -1*a_treshold):
                self.my_vel.angular.z = closest_angle * 1.0 #0.2
            else:
                self.my_vel.angular.z = 0.0;
                
        else:
            self.my_vel.linear.x = 0.0;
            self.my_vel.angular.z = 0.0;
        
        #pass 

    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.  
        stop_twist = Twist() 
        self.pub_cmdvel.publish(stop_twist) 
        #pass
        
        
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("follower", anonymous=True) 
    FollowerClass() 
