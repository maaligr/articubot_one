#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def controller_callback(data):
    global vel_x, vel_y, vel_theta

    vel_x = data.linear.x
    vel_y = data.linear.y
    vel_theta = data.angular.z

def commander():
    global vel_x, vel_y, vel_theta

    pub1 = rospy.Publisher('left_front_wheel_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('right_front_wheel_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('right_rear_wheel_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('left_rear_wheel_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        vel1 = 50 * (-vel_x + vel_y + 0.4045 * vel_theta)
        vel2 = 50 * (vel_x + vel_y - 0.4045 * vel_theta)
        vel3 = 50 * (-vel_x + vel_y - 0.4045 * vel_theta)
        vel4 = 50 * (vel_x + vel_y + 0.4045 * vel_theta)

        pub1.publish(vel1)
        pub2.publish(vel2)
        pub3.publish(vel3)
        pub4.publish(vel4)

        try:
            rate.sleep()
        except rospy.ROSException:
         print("restart simulation")

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, controller_callback)
    commander()
    rospy.spin()
