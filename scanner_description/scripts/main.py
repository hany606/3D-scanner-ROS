#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64,String
import math
import sys
from threading import Thread

increment = math.pi / 360 * 10


#Settings of the keyboard keys for Joints control
#index of the list indicate the joints number
joints_positions_settigns=[{'+':'a', '-':'d', 'min':0, 'max':2*math.pi},
                           {'+':'w', '-':'s', 'min':-math.pi/2, 'max':math.pi/2}, 
                           {'+':'q', '-':'e', 'min':-math.pi/2, 'max':math.pi/2},
                           {'+':'k', '-':'i', 'min':-math.pi/2, 'max':math.pi/2}]
joints_positions_vars = [0,0,0,0]     #List of variable to store the positions of the joints

def increment_positions(data):
    key = data.data
    print("Recieved key: %s"% key)
    #Choose the needed joint's position to be increased or decreased
    flag = 0
    for i in range(len(joints_positions_settigns)):
        if(key == joints_positions_settigns[i]['+']):
            joints_positions_vars[i] += increment
            flag = 1
        elif(key  == joints_positions_settigns[i]['-']):
            joints_positions_vars[i] -= increment
            flag = 1

    if(flag == 0):
        print("\n*************\nUnknown Keyboard key\n*************\n")

    
    #Set the boundries
    for i in range(len(joints_positions_settigns)):
        minn = joints_positions_settigns[i]['min']
        maxx = joints_positions_settigns[i]['max'] 
        if(joints_positions_vars[i] < minn):
            joints_positions_vars[i] = minn
        if(joints_positions_vars[i] > maxx):
            joints_positions_vars[i] = maxx%(2*math.pi)

def talker():
    #publishers for the Joints
    pub_joint0 = rospy.Publisher('/scanner/joint0_position_controller/command', Float64, queue_size=10)
    pub_joint1 = rospy.Publisher('/scanner/joint1_position_controller/command', Float64, queue_size=10)
    pub_joint2 = rospy.Publisher('/scanner/joint2_position_controller/command', Float64, queue_size=10)
    pub_joint_kinect = rospy.Publisher('/scanner/joint_kinect_position_controller/command', Float64, queue_size=10)


    #subscriber for the keys
    sub_keys  = rospy.Subscriber('/key_value',String, increment_positions)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #The log message to indicate the joints positions
        log_msg = "j0: %f,\t j1: %f,\t j2: %f\t time:%s" % (joints_positions_vars[0], joints_positions_vars[1],
                                                            joints_positions_vars[2], rospy.get_time())
        
        rospy.loginfo(log_msg)
        
        #Pulbish the positions of the joints
        pub_joint0.publish(joints_positions_vars[0])
        pub_joint1.publish(joints_positions_vars[1])
        pub_joint2.publish(joints_positions_vars[2])
        pub_joint_kinect.publish(joints_positions_vars[3])
        

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
