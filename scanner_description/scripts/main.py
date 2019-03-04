#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math
import sys
from threading import Thread

def talker():
    pub = rospy.Publisher('/scanner/joint0_position_controller/command', Float64, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    position = 0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        position += math.pi / 360 * 10
        rospy.loginfo(position)
        pub.publish(position)
        rate.sleep()

def init_pos(j1, j2):
    thread1 = Thread(target=go_to, args=('/scanner/joint1_position_controller/command', j1))
    thread2 = Thread(target=go_to, args=('/scanner/joint2_position_controller/command', j2))

    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()



def go_to(topic, pos):
    pub = rospy.Publisher(topic, Float64, queue_size=10)
    hz = 10
    rate = rospy.Rate(hz) # 10hz
    for i in range(3 * hz):
        pub.publish(pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        j1_pos = float(sys.argv[1])
        j2_pos = float(sys.argv[2])
        rospy.init_node('scanner_talker', anonymous=True)
        init_pos(j1_pos, j2_pos)
        talker()
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("Usage: rosrun scanner main.py [j1_pos] [j2_pos]")
